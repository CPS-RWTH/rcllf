// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file thread_role.cpp
/// @brief Guard and callback_start evidence for the one-scheduler invariant.
///        See thread_role.hpp.

#include "thread_role.hpp"

#include <dlfcn.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "clock_guard.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

namespace ros2_lf_binding
{

namespace
{
thread_local ThreadRole t_role = ThreadRole::UNSPECIFIED;
std::atomic<uint64_t> g_refused{0};
std::atomic<uint64_t> g_expired{0};
std::atomic<uint64_t> g_gated{0};
std::atomic<uint64_t> g_gated_dropped{0};
std::atomic<bool> g_guard_disabled{false};

long current_tid()  // NOLINT(runtime/int)
{
  return static_cast<long>(syscall(SYS_gettid));  // NOLINT(runtime/int)
}

using EnvsFn = int (*)(environment_t **);

/// Start time of *env*. A federate's reactor-c sets env->start_tag before the
/// RTI start-time negotiation and never updates it, while the global
/// start_time (lf_time_start()) becomes the negotiated start.
/// Resolve the EXECUTABLE's lf_time_start, as runtime.cpp does for
/// lf_request_stop. Without it, or before reactor-c set it, use the env.
int64_t start_time_of(const environment_t & env)
{
  using StartFn = int64_t (*)(void);
  static StartFn fn = reinterpret_cast<StartFn>(dlsym(RTLD_DEFAULT, "lf_time_start"));
  const int64_t t = fn != nullptr ? fn() : -1;
  return t > 0 ? t : env.start_tag.time;
}
std::atomic<bool> g_tag_override{false};
std::atomic<int64_t> g_tag_override_time{0};
std::atomic<uint32_t> g_tag_override_microstep{0};

/// Current tag of the first environment, or {-1, 0} outside an LF program.
std::pair<int64_t, uint32_t> current_tag()
{
  if (g_tag_override.load(std::memory_order_relaxed)) {
    return {g_tag_override_time.load(), g_tag_override_microstep.load()};
  }
  static EnvsFn fn = reinterpret_cast<EnvsFn>(dlsym(RTLD_DEFAULT, "_lf_get_environments"));
  if (fn == nullptr) {
    return {-1, 0};
  }
  environment_t * envs = nullptr;
  if (fn(&envs) <= 0 || envs == nullptr) {
    return {-1, 0};
  }
  return {envs[0].current_tag.time, envs[0].current_tag.microstep};
}

/// Elapsed logical tag (time since start, microstep), or {-1, 0} outside LF.
std::pair<int64_t, uint32_t> elapsed_tag()
{
  static EnvsFn fn = reinterpret_cast<EnvsFn>(dlsym(RTLD_DEFAULT, "_lf_get_environments"));
  if (fn == nullptr) {
    return {-1, 0};
  }
  environment_t * envs = nullptr;
  if (fn(&envs) <= 0 || envs == nullptr) {
    return {-1, 0};
  }
  return {envs[0].current_tag.time - start_time_of(envs[0]), envs[0].current_tag.microstep};
}

/// Start tag of the first environment, or {-1, 0} outside LF.
std::pair<int64_t, uint32_t> start_tag()
{
  static EnvsFn fn = reinterpret_cast<EnvsFn>(dlsym(RTLD_DEFAULT, "_lf_get_environments"));
  if (fn == nullptr) {
    return {-1, 0};
  }
  environment_t * envs = nullptr;
  if (fn(&envs) <= 0 || envs == nullptr) {
    return {-1, 0};
  }
  return {start_time_of(envs[0]), envs[0].start_tag.microstep};
}

/// Stop tag of the first environment, or {-1, 0} outside LF.
std::pair<int64_t, uint32_t> stop_tag()
{
  static EnvsFn fn = reinterpret_cast<EnvsFn>(dlsym(RTLD_DEFAULT, "_lf_get_environments"));
  if (fn == nullptr) {
    return {-1, 0};
  }
  environment_t * envs = nullptr;
  if (fn(&envs) <= 0 || envs == nullptr) {
    return {-1, 0};
  }
  return {envs[0].stop_tag.time, envs[0].stop_tag.microstep};
}

std::atomic<uint64_t> g_unbound{0};
}  // namespace

std::pair<int64_t, uint32_t> lf_current_tag() {return current_tag();}

void set_lf_tag_for_test(std::optional<std::pair<int64_t, uint32_t>> tag)
{
  if (tag) {
    g_tag_override_time.store(tag->first);
    g_tag_override_microstep.store(tag->second);
  }
  g_tag_override.store(tag.has_value());
}
std::pair<int64_t, uint32_t> lf_elapsed_tag() {return elapsed_tag();}
std::pair<int64_t, uint32_t> lf_start_tag() {return start_tag();}
std::pair<int64_t, uint32_t> lf_stop_tag() {return stop_tag();}

namespace
{
std::atomic<int> g_fast_override{-1};  // -1: the real flag; 0/1: a test's value

/// The executable's `bool fast` (reactor-c reactor_common.c, set by `-f true`
/// in process_args). `fast` is a common name, so it only counts when it comes
/// from the object that also defines _lf_get_environments.
const bool * reactor_c_fast()
{
  void * sym = dlsym(RTLD_DEFAULT, "fast");
  void * envs = dlsym(RTLD_DEFAULT, "_lf_get_environments");
  if (sym == nullptr || envs == nullptr) {
    return nullptr;
  }
  Dl_info a{};
  Dl_info b{};
  if (dladdr(sym, &a) == 0 || dladdr(envs, &b) == 0 || a.dli_fname == nullptr ||
    b.dli_fname == nullptr || std::strcmp(a.dli_fname, b.dli_fname) != 0)
  {
    return nullptr;
  }
  return static_cast<const bool *>(sym);
}

/// Resolve once and say what was found: an unresolved flag means real time,
/// which in a fast program brings back the fast-mode missed-sample race, so
/// it must show in the log.
const bool * resolve_reactor_c_fast()
{
  const bool * flag = reactor_c_fast();
  if (flag == nullptr) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"), "fast flag not found, assuming real time");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"), "fast flag resolved: %s", *flag ? "true" : "false");
  }
  return flag;
}
}  // namespace

bool lf_fast()
{
  const int o = g_fast_override.load(std::memory_order_relaxed);
  if (o >= 0) {
    return o != 0;
  }
  static const bool * const flag = resolve_reactor_c_fast();
  return flag != nullptr && *flag;
}

void set_lf_fast_for_test(std::optional<bool> fast)
{
  g_fast_override.store(fast ? (*fast ? 1 : 0) : -1);
}

ThreadRole current_thread_role() {return t_role;}
void set_thread_role(ThreadRole role) {t_role = role;}

const char * thread_role_name(ThreadRole role)
{
  switch (role) {
    case ThreadRole::LF_WORKER: return "lf_worker";
    case ThreadRole::INGRESS: return "ingress";
    case ThreadRole::OTHER: return "other";
    case ThreadRole::UNSPECIFIED: default: return "non-lf";
  }
}

ScopedThreadRoleForTest::ScopedThreadRoleForTest(ThreadRole role)
: previous_(t_role)
{
  t_role = role;
}

ScopedThreadRoleForTest::~ScopedThreadRoleForTest() {t_role = previous_;}

int executable_lf_thread_id()
{
  using Fn = int (*)(void);
  static Fn fn = reinterpret_cast<Fn>(dlsym(RTLD_DEFAULT, "lf_thread_id"));
  return fn ? fn() : -2;
}

bool is_lf_worker_thread()
{
  const ThreadRole role = t_role;
  if (role == ThreadRole::LF_WORKER) {
    return true;
  }
  if (role == ThreadRole::INGRESS || role == ThreadRole::OTHER) {
    return false;
  }
  return executable_lf_thread_id() >= 0;
}

bool require_lf_worker(const char * entry, int rid)
{
  if (g_guard_disabled.load(std::memory_order_relaxed)) {
    return true;
  }
  const ThreadRole role = t_role;
  if (role == ThreadRole::LF_WORKER) {
    return true;
  }
  bool refuse = role == ThreadRole::INGRESS || role == ThreadRole::OTHER;
  if (!refuse) {
    // Secondary check (R9): a reactor-c thread has lf_thread_id() >= 0. An
    // unresolvable symbol (-2) means no LF program is linked, e.g. a unit test
    // without a role override: nothing to compare against, admit.
    const int id = executable_lf_thread_id();
    refuse = id == -1;
  }
  if (!refuse) {
    return true;
  }
  g_refused.fetch_add(1);
  thread_local std::set<std::pair<std::string, int>> logged;
  if (logged.insert({entry, rid}).second) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: refused user callback %s (RID %d) on non-LF thread '%s': only LF workers may "
      "invoke user callbacks (one-scheduler invariant)",
      entry, rid, thread_role_name(role));
  }
  return false;
}

uint64_t refused_callback_count() {return g_refused.load();}

void set_guard_disabled_for_test(bool disabled) {g_guard_disabled.store(disabled);}

bool trace_callbacks_enabled()
{
  static const bool enabled = [] {
      const char * v = std::getenv("RCLLF_TRACE_CALLBACKS");
      return v != nullptr && std::strcmp(v, "1") == 0;
    }();
  return enabled;
}

namespace
{
void emit_callback_start(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  int64_t ingress_record_id, const std::string & consumed)
{
  set_current_callback(node, callback);  // clock_guard messages
  if (!trace_callbacks_enabled()) {
    return;
  }
  const auto tag = current_tag();
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding.trace"),
    "{\"event\":\"callback_start\",\"entry\":\"%s\",\"rid\":%d,\"node\":\"%s\","
    "\"callback\":\"%s\",\"lf_thread_id\":%d,\"gettid\":%ld,\"tag\":%lld,\"microstep\":%u,"
    "\"ingress_record_id\":%lld%s}",
    entry, rid, node.c_str(), callback.c_str(), executable_lf_thread_id(), current_tid(),
    static_cast<long long>(tag.first), tag.second,  // NOLINT(runtime/int)
    static_cast<long long>(ingress_record_id), consumed.c_str());  // NOLINT(runtime/int)
}
}  // namespace

void trace_callback_start(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  int64_t ingress_record_id)
{
  emit_callback_start(entry, rid, node, callback, ingress_record_id, std::string());
}

void trace_callback_start(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  const rmw_message_info_t & consumed, int64_t ingress_record_id)
{
  std::string pub;
  if (trace_callbacks_enabled()) {
    static const char kHex[] = "0123456789abcdef";
    std::string gid;
    const size_t n = std::min<size_t>(16, sizeof(consumed.publisher_gid.data));
    for (size_t i = 0; i < n; ++i) {
      gid += kHex[consumed.publisher_gid.data[i] >> 4];
      gid += kHex[consumed.publisher_gid.data[i] & 0xf];
    }
    pub = ",\"pub_gid\":\"" + gid + "\",\"pub_seq\":" +
      std::to_string(consumed.publication_sequence_number) + ",\"pub_src_ns\":" +
      std::to_string(consumed.source_timestamp);
  }
  emit_callback_start(entry, rid, node, callback, ingress_record_id, pub);
}

void trace_entity_expired(
  const char * entry, int rid, const std::string & node, const std::string & callback)
{
  g_expired.fetch_add(1);
  RCLCPP_DEBUG(
    rclcpp::get_logger("ros2_lf_binding"),
    "%s: RID %d (%s::%s) entity expired (released by its node); no-op", entry, rid,
    node.c_str(), callback.c_str());
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"expired\",\"entry\":\"%s\",\"rid\":%d,\"node\":\"%s\",\"callback\":\"%s\"}",
      entry, rid, node.c_str(), callback.c_str());
  }
}

uint64_t expired_entity_count() {return g_expired.load();}

void trace_lifecycle_gated(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  bool dropped)
{
  g_gated.fetch_add(1);
  if (dropped) {
    g_gated_dropped.fetch_add(1);
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("ros2_lf_binding"),
    "%s: RID %d (%s::%s) lifecycle node not active%s; no-op", entry, rid, node.c_str(),
    callback.c_str(), dropped ? ", arrival dropped" : "");
  if (trace_callbacks_enabled()) {
    const auto tag = elapsed_tag();
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"lifecycle_gated\",\"entry\":\"%s\",\"rid\":%d,\"node\":\"%s\","
      "\"callback\":\"%s\",\"dropped\":%s,\"elapsed\":%lld,\"microstep\":%u}",
      entry, rid, node.c_str(), callback.c_str(), dropped ? "true" : "false",
      static_cast<long long>(tag.first), tag.second);  // NOLINT(runtime/int)
  }
}

uint64_t lifecycle_gated_count() {return g_gated.load();}
uint64_t lifecycle_dropped_count() {return g_gated_dropped.load();}

void trace_unbound(const char * entry, int rid)
{
  g_unbound.fetch_add(1);
  const auto tag = elapsed_tag();
  thread_local std::set<std::pair<std::string, int>> logged;
  if (logged.insert({entry, rid}).second) {
    // A dispatch before its node's bind is an ordering defect, not
    // a quiet no-op. The generated bind-before-dispatch edge makes this
    // unreachable; this line is the runtime backstop and the falsification
    // evidence when the edge is removed.
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"), "%s: RID %d unbound at (%lld, %u)", entry, rid,
      static_cast<long long>(tag.first), tag.second);  // NOLINT(runtime/int)
  }
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"unbound\",\"entry\":\"%s\",\"rid\":%d,\"elapsed\":%lld,\"microstep\":%u}",
      entry, rid, static_cast<long long>(tag.first), tag.second);  // NOLINT(runtime/int)
  }
}

uint64_t unbound_dispatch_count() {return g_unbound.load();}

void trace_publisher(
  const std::string & node_fqn, const std::string & topic, const std::string & gid_hex)
{
  // Read the variable here, not through trace_callbacks_enabled(): publishers
  // are created before anything else runs (the /rosout publisher of the first
  // node), and that first read would freeze the process-wide cache before a
  // test sets RCLLF_TRACE_CALLBACKS (FanInGroup.NonLeadDispatchCarriesMemberEdge).
  // Publisher creation is rare, so the uncached read costs nothing measurable.
  const char * v = std::getenv("RCLLF_TRACE_CALLBACKS");
  if (v == nullptr || std::strcmp(v, "1") != 0) {
    return;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding.trace"),
    "{\"event\":\"publisher\",\"node\":\"%s\",\"topic\":\"%s\",\"gid\":\"%s\"}",
    node_fqn.c_str(), topic.c_str(), gid_hex.c_str());
}

void trace_bind(const char * node_class)
{
  if (!trace_callbacks_enabled()) {
    return;
  }
  const auto tag = elapsed_tag();
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding.trace"),
    "{\"event\":\"bind\",\"node\":\"%s\",\"lf_thread_id\":%d,\"elapsed\":%lld,"
    "\"microstep\":%u}",
    node_class, executable_lf_thread_id(), static_cast<long long>(tag.first),  // NOLINT
    tag.second);
}

}  // namespace ros2_lf_binding

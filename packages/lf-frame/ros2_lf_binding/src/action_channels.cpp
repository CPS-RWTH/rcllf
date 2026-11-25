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

/// @file action_channels.cpp
/// @brief Counted action channels, the binding side of the action gate.

#include "action_channels.hpp"

#include <rcl_action/action_client.h>
#include <rcl_action/action_server.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "action_client_guard.hpp"
#include "corr_runtime.hpp"
#include "handle_table.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_lf_binding/action_gate_api.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "rid_table.hpp"
#include "take_mode.hpp"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{
namespace
{

const char * const kChannelName[RCLLF_ACTION_CHANNELS] = {
  "goal_req", "result_req", "cancel_req", "goal_resp", "result", "cancel_resp", "feedback"};

/// Sends of one action on this thread since its last claim, per channel.
struct SentCounts
{
  std::string action;
  std::array<int, RCLLF_ACTION_CHANNELS> n{};
};

thread_local std::vector<SentCounts> t_sent;
thread_local std::array<int, RCLLF_ACTION_CHANNELS> t_take_ret{};
thread_local std::array<bool, RCLLF_ACTION_CHANNELS> t_take_seen{};

bool valid_channel(int channel) {return channel >= 0 && channel < RCLLF_ACTION_CHANNELS;}

std::atomic<bool> g_refusal_armed{false};
std::atomic<uint64_t> g_refused{0};

/// Item 3: the ingress of an action server whose action has no client inside
/// the LF system. Keyed by the goal server RID.
struct ExternalSlot
{
  std::atomic<int (*)(void *, int64_t)> wake{nullptr};
  void * action = nullptr;
  std::array<std::atomic<int>, 3> pending{};  // goal, result, cancel service
  bool registered = false;
  bool attached = false;
};

std::mutex g_ext_mu;
// A slot is shared with the on-ready callback, so it lives as long as the
// server holds that callback.
std::map<int, std::shared_ptr<ExternalSlot>> g_ext;

ExternalSlot & ext_slot(int rid)
{
  auto & p = g_ext[rid];
  if (!p) {
    p = std::make_shared<ExternalSlot>();
  }
  return *p;
}

/// Hook the server's on-ready callback once the slot is registered and the
/// RID bound. The middleware calls it per arrival, off any LF thread, so it
/// only counts and schedules the physical action. Caller holds g_ext_mu.
void ext_attach(int rid, const std::shared_ptr<ExternalSlot> & sp)
{
  ExternalSlot & slot = *sp;
  if (slot.attached || !slot.registered || rid < 0 ||
    rid >= static_cast<int>(g_handle_table.size()))
  {
    return;
  }
  auto waitable = g_handle_table[rid].action_waitable.lock();
  if (!waitable) {
    return;
  }
  waitable->set_on_ready_callback(
    [sp](size_t n, int entity) {
      if (entity < 0 || entity > 2 || n == 0) {
        return;
      }
      sp->pending[entity].fetch_add(static_cast<int>(n));
      auto wake = sp->wake.load();
      if (wake != nullptr) {
        wake(sp->action, 0);
      }
    });
  slot.attached = true;
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "action '%s' (RID %d) has no client inside the LF system: its requests enter as a "
    "physical action", g_handle_table[rid].topic.c_str(), rid);
}

/// The action name rcl stored for `handle`: a client handle for a request,
/// a server handle otherwise.
const char * action_name(int channel, const void * handle)
{
  const char * name = channel <= RCLLF_ACTION_CANCEL_REQ ?
    rcl_action_client_get_action_name(static_cast<const rcl_action_client_t *>(handle)) :
    rcl_action_server_get_action_name(static_cast<const rcl_action_server_t *>(handle));
  if (name == nullptr) {
    rcl_reset_error();
    return "";
  }
  return name;
}

/// `full` (as rcl stored it, resolved or not) names the action `wanted` (as the
/// generator wrote it): equal but for a leading slash, or `wanted` is its last
/// path segments.
bool names_action(const std::string & full, const char * wanted)
{
  std::string w = wanted;
  std::string f = full;
  if (!w.empty() && w[0] == '/') {
    w.erase(0, 1);
  }
  if (!f.empty() && f[0] == '/') {
    f.erase(0, 1);
  }
  if (f == w) {
    return true;
  }
  return f.size() > w.size() && f.compare(f.size() - w.size(), w.size(), w) == 0 &&
         f[f.size() - w.size() - 1] == '/';
}

/// Item 6: the pre-shutdown callbacks kept from the context, per binding key
/// of the node whose construction registered them.
using KeptCallback = std::shared_ptr<std::function<void()>>;
std::mutex g_pre_mu;
std::map<std::string, std::vector<KeptCallback>> g_pre_shutdown;

/// The node with binding key `key` serves an action: the generator emitted a
/// goal server reaction, and with it the node's reaction(shutdown).
bool serves_action(const std::string & key)
{
  for (const auto & e : get_rid_entries()) {
    if (e.node_class == key && e.kind == "action_goal_server") {
      return true;
    }
  }
  return false;
}

/// Remove and return the kept callbacks of `key`, or of every node for null.
std::vector<KeptCallback> take_pre_shutdown(const std::string * key)
{
  std::vector<KeptCallback> out;
  std::lock_guard<std::mutex> lk(g_pre_mu);
  for (auto it = g_pre_shutdown.begin(); it != g_pre_shutdown.end(); ) {
    if (key != nullptr && it->first != *key) {
      ++it;
      continue;
    }
    out.insert(out.end(), it->second.begin(), it->second.end());
    it = g_pre_shutdown.erase(it);
  }
  return out;
}

void trace_line(const std::string & json)
{
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(rclcpp::get_logger("ros2_lf_binding.trace"), "%s", json.c_str());
  }
}

std::string elapsed_json()
{
  const auto e = lf_elapsed_tag();
  return "\"elapsed\":" + std::to_string(e.first) + ",\"microstep\":" + std::to_string(e.second);
}

}  // namespace

void action_sends_begin(const char * entry, int rid)
{
  for (const auto & s : t_sent) {
    for (int c = 0; c < RCLLF_ACTION_CHANNELS; ++c) {
      if (s.n[c] > 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ros2_lf_binding"),
          "%s: RID %d starts while %d %s sample(s) of action '%s' sent on this thread were "
          "claimed by no reaction, so no port event carries them",
          entry, rid, s.n[c], kChannelName[c], s.action.c_str());
      }
    }
  }
  t_sent.clear();
}

void action_refusal_arm(bool armed) {g_refusal_armed.store(armed);}

uint64_t action_refused_count() {return g_refused.load();}

int action_counted_take(
  const char * entry, void * self, int rid, size_t entity_id, int channel, int count,
  bool active)
{
  auto & handle_entry = g_handle_table[rid];
  auto waitable = handle_entry.action_waitable.lock();
  if (!waitable) {
    trace_entity_expired(entry, rid, handle_entry.node_class, handle_entry.callback_name);
    return 0;
  }
  const int64_t timeout_ms = take_timeout_ms_env().value_or(handle_entry.take_timeout_ms);
  int executed = 0;
  for (int i = 0; i < count; ++i) {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    bool done = false;
    while (!done) {
      t_take_seen[channel] = false;
      std::shared_ptr<void> data;
      try {
        data = waitable->take_data_by_entity_id(entity_id);
      } catch (const std::runtime_error &) {
        data.reset();
      }
      const bool gated = t_take_seen[channel];
      if (data && (!gated || t_take_ret[channel] == RCL_RET_OK) && !active) {
        trace_lifecycle_gated(entry, rid, handle_entry.node_class, handle_entry.callback_name,
            true);
        done = true;
        if (channel == RCLLF_ACTION_GOAL_RESP || channel == RCLLF_ACTION_RESULT_RESP ||
          channel == RCLLF_ACTION_CANCEL_RESP)
        {
          // A response is still executed, so rclcpp erases its
          // pending entry. The guarded callback keeps it from user code.
          UserCallbackScope user;
          if (!user.admitted()) {
            return executed;
          }
          InactiveClientScope inactive(handle_entry.node_class);
          corr_rt::OriginScope origin(self, rid);
          waitable->execute(static_cast<const std::shared_ptr<void> &>(data));
        }
      } else if (data && (!gated || t_take_ret[channel] == RCL_RET_OK)) {
        UserCallbackScope user;  // none after shutdown began
        if (!user.admitted()) {
          return executed;
        }
        trace_callback_start(entry, rid, handle_entry.node_class, handle_entry.callback_name);
        corr_rt::DispatchTrace dt(rid, RCLLF_ENTRY_ACTION);
        dt.set_ran(true);
        corr_rt::OriginScope origin(self, rid);
        waitable->execute(static_cast<const std::shared_ptr<void> &>(data));
        ++executed;
        done = true;
      } else if (!gated) {
        // No gate in this binary: one take as before the counted channels.
        return executed;
      } else if (std::chrono::steady_clock::now() >= deadline) {
        corr_rt::DispatchTrace dt(rid, RCLLF_ENTRY_ACTION);
        dt.set_outcome(RCLLF_OUTCOME_LOST);
        dt.set_missing_hook(RCLLF_MISSING_HOOK_NONE);
        RCLCPP_ERROR(
          rclcpp::get_logger("ros2_lf_binding"),
          "%s: RID %d (%s::%s): %s sample %d of %d did not arrive within %lld ms, LOST "
          "(rclcpp_action has no deadline or message_lost handler to run)",
          entry, rid, handle_entry.node_class.c_str(), handle_entry.callback_name.c_str(),
          kChannelName[channel], i + 1, count, static_cast<long long>(timeout_ms));  // NOLINT
        trace_line(
          "{\"event\":\"action\",\"kind\":\"lost\",\"rid\":" + std::to_string(rid) +
          ",\"channel\":\"" + kChannelName[channel] + "\",\"sample\":" + std::to_string(i + 1) +
          ",\"count\":" + std::to_string(count) + ",\"missing_hook\":\"none\"," +
          elapsed_json() + "}");
        done = true;
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
      }
    }
  }
  return executed;
}

void action_external_on_bound(int rid)
{
  std::lock_guard<std::mutex> lk(g_ext_mu);
  auto it = g_ext.find(rid);
  if (it != g_ext.end()) {
    ext_attach(rid, it->second);
  }
}

void action_external_shutdown()
{
  // The on-ready callbacks stay installed with a null wake. Clearing them is
  // unsafe on Jazzy: clear_on_ready_callback() empties the expire timer's
  // reset callback that rcl still points to, so the next goal expiry reset
  // throws std::bad_function_call on a middleware thread.
  std::lock_guard<std::mutex> lk(g_ext_mu);
  for (auto & kv : g_ext) {
    kv.second->wake.store(nullptr);
  }
  g_ext.clear();
}

std::shared_ptr<std::function<void()>> action_keep_pre_shutdown(std::function<void()> & callback)
{
  const std::string & key = corr_rt::construct_key();
  if (key.empty() || !serves_action(key)) {
    return nullptr;
  }
  auto kept = std::make_shared<std::function<void()>>(std::move(callback));
  std::lock_guard<std::mutex> lk(g_pre_mu);
  g_pre_shutdown[key].push_back(kept);
  return kept;
}

bool action_drop_pre_shutdown(const std::shared_ptr<std::function<void()>> & callback)
{
  std::lock_guard<std::mutex> lk(g_pre_mu);
  for (auto & kv : g_pre_shutdown) {
    auto & v = kv.second;
    for (auto it = v.begin(); it != v.end(); ++it) {
      if (*it == callback) {
        v.erase(it);
        return true;
      }
    }
  }
  return false;
}

bool action_pre_shutdown_kept()
{
  std::lock_guard<std::mutex> lk(g_pre_mu);
  for (const auto & kv : g_pre_shutdown) {
    if (!kv.second.empty()) {
      return true;
    }
  }
  return false;
}

void action_pre_shutdown_run_left(const char * where)
{
  const auto left = take_pre_shutdown(nullptr);
  if (left.empty()) {
    return;
  }
  RCLCPP_WARN(
    rclcpp::get_logger("ros2_lf_binding"),
    "rcllf: %zu pre-shutdown callback(s) of an action server node ran %s, not in its "
    "reaction(shutdown), so what they send has no LF origin", left.size(), where);
  for (const auto & cb : left) {
    if (*cb) {
      (*cb)();
    }
  }
}

}  // namespace ros2_lf_binding

extern "C" {

int rcllf_action_gate_pre(int channel, const void * handle)
{
  namespace lfb = ros2_lf_binding;
  if (!lfb::g_refusal_armed.load() || !lfb::valid_channel(channel) ||
    lfb::corr_rt::origin_active() || lfb::is_lf_worker_thread())
  {
    return 1;
  }
  // Item 7: a send from a thread of the user's own (a worker started in the
  // constructor, say) has no LF tag, so it would break the repeatable tags of
  // every port it reaches. Refuse it instead of forwarding it.
  lfb::g_refused.fetch_add(1);
  const std::string name = lfb::action_name(channel, handle);
  static std::mutex logged_mu;
  static std::set<std::pair<std::string, int>> logged;
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(logged_mu);
    first = logged.insert({name, channel}).second;
  }
  if (first) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: refused %s on action '%s' from a thread with no LF origin: under lfc an action "
      "is sent only from a callback that an LF reaction runs (RCLLF-H011)",
      lfb::kChannelName[channel], name.c_str());
  }
  lfb::trace_line(
    "{\"event\":\"action\",\"kind\":\"refused\",\"channel\":\"" +
    std::string(lfb::kChannelName[channel]) + "\",\"action\":\"" + name + "\"}");
  return 0;
}

void rcllf_action_gate_post(int channel, const void * handle, int ret)
{
  if (ret != RCL_RET_OK || !ros2_lf_binding::valid_channel(channel)) {
    return;
  }
  const char * name = ros2_lf_binding::action_name(channel, handle);
  for (auto & s : ros2_lf_binding::t_sent) {
    if (s.action == name) {
      ++s.n[channel];
      return;
    }
  }
  ros2_lf_binding::SentCounts s;
  s.action = name;
  s.n[channel] = 1;
  ros2_lf_binding::t_sent.push_back(std::move(s));
}

void rcllf_action_gate_take(int channel, const void * handle, int ret)
{
  (void)handle;
  if (ros2_lf_binding::valid_channel(channel)) {
    ros2_lf_binding::t_take_seen[channel] = true;
    ros2_lf_binding::t_take_ret[channel] = ret;
  }
}

int ros2_lf_action_sent(const char * action, int channel)
{
  if (action == nullptr || !ros2_lf_binding::valid_channel(channel)) {
    return 0;
  }
  int n = 0;
  for (auto & s : ros2_lf_binding::t_sent) {
    if (ros2_lf_binding::names_action(s.action, action)) {
      n += s.n[channel];
      s.n[channel] = 0;
    }
  }
  return n;
}

void ros2_lf_action_external_register(
  void * self, int goal_rid, int (*wake)(void *, int64_t), void * action)
{
  (void)self;
  namespace lfb = ros2_lf_binding;
  std::lock_guard<std::mutex> lk(lfb::g_ext_mu);
  auto & slot = lfb::ext_slot(goal_rid);
  slot.action = action;
  slot.wake.store(wake);
  slot.registered = true;
  lfb::ext_attach(goal_rid, lfb::g_ext[goal_rid]);  // bind may have run first
}

int ros2_lf_action_external_pending(void * self, int goal_rid, int entity)
{
  (void)self;
  namespace lfb = ros2_lf_binding;
  if (entity < 0 || entity > 2) {
    return 0;
  }
  std::lock_guard<std::mutex> lk(lfb::g_ext_mu);
  auto it = lfb::g_ext.find(goal_rid);
  return it == lfb::g_ext.end() ? 0 : it->second->pending[entity].exchange(0);
}

void ros2_lf_action_server_shutdown(void * self, int goal_rid)
{
  namespace lfb = ros2_lf_binding;
  const char * entry = "ros2_lf_action_server_shutdown";
  if (!lfb::require_lf_worker(entry, goal_rid)) {
    return;
  }
  lfb::action_sends_begin(entry, goal_rid);
  lfb::lf_time_advance_ros_clock();
  if (goal_rid < 0 || goal_rid >= static_cast<int>(lfb::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"), "%s: reaction_id %d out of range", entry, goal_rid);
    return;
  }
  const std::string node = lfb::g_handle_table[goal_rid].node_class;
  lfb::UserCallbackScope user;  // none after rclcpp shutdown began
  if (!user.admitted()) {
    return;  // the callbacks stay kept, the runtime runs them at its shutdown
  }
  for (const auto & cb : lfb::take_pre_shutdown(&node)) {
    if (!*cb) {
      continue;
    }
    lfb::trace_callback_start(entry, goal_rid, node, "pre-shutdown callback");
    lfb::corr_rt::DispatchTrace dt(goal_rid, RCLLF_ENTRY_ACTION);
    dt.set_ran(true);
    lfb::corr_rt::OriginScope origin(self, goal_rid);
    (*cb)();
  }
}

}  // extern "C"

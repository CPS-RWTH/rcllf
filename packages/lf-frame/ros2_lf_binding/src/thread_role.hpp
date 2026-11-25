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

/// @file thread_role.hpp
/// @brief One-scheduler invariant: only LF workers invoke user
///        callbacks. Thread roles, the guard every ros2_lf_invoke_* passes
///        through, and the callback_start evidence line.

#ifndef THREAD_ROLE_HPP_
#define THREAD_ROLE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "rmw/types.h"

namespace ros2_lf_binding
{

enum class ThreadRole
{
  UNSPECIFIED,  ///< Decided by lf_thread_id(): >= 0 is a reactor-c thread.
  LF_WORKER,    ///< Test override: treat this thread as an LF worker.
  INGRESS,      ///< The binding's ingress thread: never runs user code.
  OTHER,        ///< Test override: a non-LF thread (rclcpp, clock, ...).
};

ThreadRole current_thread_role();
void set_thread_role(ThreadRole role);
const char * thread_role_name(ThreadRole role);

/// RAII role override for tests (and for the ingress thread itself).
class ScopedThreadRoleForTest
{
public:
  explicit ScopedThreadRoleForTest(ThreadRole role);
  ~ScopedThreadRoleForTest();
  ScopedThreadRoleForTest(const ScopedThreadRoleForTest &) = delete;
  ScopedThreadRoleForTest & operator=(const ScopedThreadRoleForTest &) = delete;

private:
  ThreadRole previous_;
};

/// The executable's lf_thread_id() (resolved via dlsym, see runtime.cpp for
/// why a link-time reference would bind the library's own reactor-c copy).
/// Returns -2 when the symbol is not resolvable.
int executable_lf_thread_id();

/// True on an LF worker: a LF_WORKER test override, or lf_thread_id() >= 0
/// without an override.
bool is_lf_worker_thread();

/// Guard at every user-callback entry point. Returns false (and logs once per
/// entry point, RID and thread) when the calling thread is not an LF worker.
bool require_lf_worker(const char * entry, int rid);

/// Number of refusals since process start (tests).
uint64_t refused_callback_count();

/// Test hook: when true, require_lf_worker always passes (falsification of
/// the guard's own tests). Never set outside tests.
void set_guard_disabled_for_test(bool disabled);

/// One `callback_start` JSON line on logger `ros2_lf_binding.trace` when
/// RCLLF_TRACE_CALLBACKS=1. `ingress_record_id` is -1 for non-ingress work.
void trace_callback_start(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  int64_t ingress_record_id = -1);

/// The same line for a subscription dispatch, plus the identity of the
/// publication it consumes: `pub_gid` (the writer gid, hex, the first
/// 16 bytes, as on the corr lines), `pub_seq` (the writer's publication
/// sequence number, the corr ordinal) and `pub_src_ns` (the source timestamp).
/// A checker joins every dispatch to its publication by (pub_gid, pub_seq).
void trace_callback_start(
  const char * entry, int rid, const std::string & node, const std::string & callback,
  const rmw_message_info_t & consumed, int64_t ingress_record_id = -1);

/// Traced no-op when a weakly held entity has expired (R14).
void trace_entity_expired(
  const char * entry, int rid, const std::string & node,
  const std::string & callback);

/// Traced no-op when the entry's lifecycle node is not active at this tag.
/// `dropped` says an arrival was taken and discarded.
void trace_lifecycle_gated(
  const char * entry, int rid, const std::string & node,
  const std::string & callback, bool dropped);

/// Number of lifecycle-gated dispatches and of arrivals they dropped (tests).
uint64_t lifecycle_gated_count();
uint64_t lifecycle_dropped_count();

/// Number of expired-entity no-ops since process start (tests).
uint64_t expired_entity_count();

bool trace_callbacks_enabled();

/// Current LF tag (absolute time, microstep) of the first environment, or
/// {-1, 0} outside an LF program; lf_elapsed_tag() is relative to start.
std::pair<int64_t, uint32_t> lf_current_tag();

/// Test support: make lf_current_tag() return `tag`, or the real tag again
/// for nothing.
void set_lf_tag_for_test(std::optional<std::pair<int64_t, uint32_t>> tag);
std::pair<int64_t, uint32_t> lf_elapsed_tag();
/// Start tag of the first environment, or {-1, 0} outside an LF program.
std::pair<int64_t, uint32_t> lf_start_tag();
/// Stop tag of the first environment (FOREVER until one is set), or {-1, 0}
/// outside an LF program.
std::pair<int64_t, uint32_t> lf_stop_tag();

/// reactor-c's fast mode (`-f true`): the executable's `fast` flag, resolved
/// like lf_time_start. False outside an LF program. In fast mode
/// a tag is granted as soon as every federate has reported it, so the physical
/// arrival of a DDS sample says nothing about its tag: a take that owes a
/// sample waits for it instead of declaring a miss.
bool lf_fast();
/// Test support: make lf_fast() return `fast`, or the real flag again for nothing.
void set_lf_fast_for_test(std::optional<bool> fast);

/// Backstop: a dispatch reached a RID its node has not bound yet.
/// ERROR once per entry point and RID ("<entry>: RID n unbound at (t, m)",
/// elapsed logical tag), an `unbound` trace line every time.
void trace_unbound(const char * entry, int rid);
uint64_t unbound_dispatch_count();

/// `bind` trace line (RCLLF_TRACE_CALLBACKS=1) at the end of ros2_lf_bind_node.
void trace_bind(const char * node_class);

/// `publisher` trace line (RCLLF_TRACE_CALLBACKS=1) for every publisher the
/// gate sees created: node, topic and writer gid (hex, as `pub_gid`). It names
/// each writer independently of who consumes it.
void trace_publisher(
  const std::string & node_fqn, const std::string & topic, const std::string & gid_hex);

}  // namespace ros2_lf_binding

#endif  // THREAD_ROLE_HPP_

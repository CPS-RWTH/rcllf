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

/// @file handle_matching.hpp
/// @brief One to one matching of analyzer reactions to ROS handles.
///
/// ros2_lf_bind_node used to bind every timer RID to the first timer with a
/// matching period and every subscription RID to the first subscription on its
/// topic. Two same-period timers (or two subscriptions on one topic) in one
/// node then shared one handle, and the second callback never ran.
///
/// match_handles() binds each handle at most once. Analyzer entries are taken
/// in the order the node creates the entities (RidEntry::creation_index), and
/// each takes the first still unbound handle that fits it and sits in the
/// entry's callback group (RidEntry::group_index). Within one group, discovery
/// order is creation order. Across groups it is group order, which says nothing
/// about creation order, so an entry whose group the analyzer could not
/// resolve and that fits handles in two groups refuses instead of guessing.
/// A kGroupForeign handle is never bound to an entry, whatever its group,
/// except a detached TransformListener handle to a kGroupTf2Listener entry.
/// Every entry must get a handle and every handle must get an entry (except a
/// handle with an acknowledged hazard, see HandleCandidate), otherwise
/// the result carries a precise error and the caller refuses to start. The
/// caller leaves rclcpp's own handles out of `handles` altogether.

#ifndef HANDLE_MATCHING_HPP_
#define HANDLE_MATCHING_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "rid_table.hpp"

namespace ros2_lf_binding
{

/// Group of a handle whose callback group another executor already spins at
/// bind time (a library's own thread, e.g. tf2_ros::TransformListener).
constexpr int kGroupForeign = -3;

/// One ROS handle as seen by the matcher.
struct HandleCandidate
{
  std::string description;  // e.g. "period 100000000 ns" or "topic '/twin'"
  /// Callback group: position among the groups the node creates itself, in
  /// creation order, kGroupDefault for the node's default group, or
  /// kGroupForeign.
  int group = kGroupDefault;
  /// Non-empty only for a kGroupForeign handle of a node that acknowledged the
  /// hazard covering it (e.g. "RCLLF-R002"): such a handle may stay without a
  /// reaction ID, and the caller logs it. Every other handle needs one.
  std::string acknowledged_hazard;
  /// A kGroupForeign handle in the TransformListener group the binding
  /// detached (tf2_ingress.hpp). Only a kGroupTf2Listener entry
  /// fits it, and it must be bound: acknowledging RCLLF-R002 does not exempt
  /// it, since tf2_ros no longer runs its callback.
  bool tf2_listener = false;
};

struct HandleMatch
{
  bool ok = true;
  std::string error;
  /// handle_of[i] is the handle index bound to entries[i] (valid when ok).
  std::vector<std::size_t> handle_of;
  /// Handles left without a reaction ID under an acknowledged hazard.
  std::vector<std::size_t> exempted;
  /// Entries left without a handle under MatchOptions::defer_missing; their
  /// handle_of slot holds SIZE_MAX.
  std::vector<std::size_t> deferred;
};

struct MatchOptions
{
  /// An entry without a fitting handle is deferred instead of refused: a
  /// lifecycle node that is not active yet creates its timers and
  /// subscriptions on a later transition.
  bool defer_missing = false;
  /// A handle without a reaction ID is left alone instead of refused. Only
  /// for binding late handles, where the startup match already judged them.
  bool allow_unclaimed = false;
};

/// Whether a timer handle of period `period_ns` (rcl_timer_get_period) can
/// serve entry `e`. The periods must be equal to the nanosecond.
/// The generator writes `e.period_ns` as the exact decimal period floored to
/// whole nanoseconds (policies.period_ms_to_ns), and rclcpp truncates a timer's
/// duration to whole nanoseconds, so equal periods meet exactly. A tolerance
/// would bind a timer of another period (a 1 ms timer changed to 0.5 ms fit
/// under the old 1 ms window) and the run would follow the analyzed schedule,
/// not the node's, with no error. An entry with period 0 (no analyzed period)
/// fits no timer.
bool timer_fits(int64_t period_ns, const RidEntry & e);

/// Match analyzer entries (one handle family of one node) to handles one to one.
///
/// @param node_class  node class or binding key, for messages
/// @param family      "timer" or "subscription", for messages
/// @param entries     the node's RID entries of this family, any order
/// @param handles     the node's handles of this family, in discovery order
/// @param fits        whether handle h can serve entry e (period or topic
///                    match); the callback group is checked here, not by `fits`
/// @param options     deferral of missing handles and of handles without a RID
HandleMatch match_handles(
  const std::string & node_class, const std::string & family,
  const std::vector<const RidEntry *> & entries,
  const std::vector<HandleCandidate> & handles,
  const std::function<bool(const RidEntry &, std::size_t)> & fits,
  const MatchOptions & options = MatchOptions{});

/// Check that no handle index is bound to two entries. Returns an empty string
/// when every handle is bound at most once, else the "bound twice" message.
std::string check_bound_once(
  const std::string & node_class, const std::string & family,
  const std::vector<const RidEntry *> & entries,
  const std::vector<HandleCandidate> & handles,
  const std::vector<std::size_t> & handle_of);

/// Log a bind failure as FATAL and terminate the process with EXIT_FAILURE.
/// A reaction whose handle is missing would silently never run, so the
/// runtime refuses to start instead.
[[noreturn]] void fail_bind(const std::string & message);

}  // namespace ros2_lf_binding

#endif  // HANDLE_MATCHING_HPP_

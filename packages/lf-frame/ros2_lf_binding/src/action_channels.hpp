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

/// @file action_channels.hpp
/// @brief Counted action channels.
///
/// The executable-side action gate (action_gate.cpp) reports every send,
/// feedback publish and take on an rcl_action handle. A send is counted per
/// thread under its action name and channel, and the generated reaction
/// claims the count after its invoke (ros2_lf_action_sent) and raises the
/// channel's port by it, so a port event carries the number of samples the
/// reaction really sent. The receiving reaction takes exactly that many
/// samples, each with a bounded wait, and traces LOST for a sample that does
/// not arrive in time.

#ifndef ACTION_CHANNELS_HPP_
#define ACTION_CHANNELS_HPP_

#include <cstddef>
#include <cstdint>

namespace ros2_lf_binding
{

/// Start of a dispatch on this thread: a send count no reaction claimed since
/// the last dispatch is reported as an error and dropped.
void action_sends_begin(const char * entry, int rid);

/// Take and execute `count` samples of `channel` from entity `entity_id` of
/// the action waitable bound to `rid`, each with a bounded wait (the entry's
/// take timeout). Returns the number executed. Without the gate (a binary
/// built without action_gate.cpp) one take is executed as before. When
/// `active` is false (a lifecycle node not active at this tag)
/// each sample is still taken and traced as gated, and dropped, except
/// that a client's goal, result or cancel response is executed with its user callback
/// suppressed, so rclcpp keeps no pending entry for it.
int action_counted_take(
  const char * entry, void * self, int rid, size_t entity_id, int channel, int count,
  bool active = true);

/// Arm or disarm the refusal of an action send with no LF origin (item 7).
/// The runtime arms it once the LF program is about to start and disarms it
/// when shutdown begins. While armed, a send or feedback publish from a
/// thread that is not an LF worker and runs no reaction's user callback is
/// not forwarded: an error is logged once per action and channel, and a
/// trace line is written per refusal.
void action_refusal_arm(bool armed);

/// Number of action sends refused since process start.
uint64_t action_refused_count();

/// Item 3: attach the external ingress registered for the goal
/// server RID `rid`, if any, now that the RID is bound. Called by bind.
void action_external_on_bound(int rid);

/// Detach every external action ingress, so no middleware callback schedules
/// an LF action after shutdown began.
void action_external_shutdown();

/// Item 6: a pre-shutdown callback of an action server node is kept and not
/// yet run.
bool action_pre_shutdown_kept();

/// Item 6: run the kept pre-shutdown callbacks no reaction(shutdown) ran, off
/// any reaction, and drop them. `where` names the caller in the warning.
void action_pre_shutdown_run_left(const char * where);

}  // namespace ros2_lf_binding

#endif  // ACTION_CHANNELS_HPP_

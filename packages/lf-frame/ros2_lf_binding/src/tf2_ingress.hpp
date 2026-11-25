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

/// @file tf2_ingress.hpp
/// @brief LF takes over the inputs of an admitted tf2_ros::TransformListener.
///
/// tf2_ros::TransformListener(buffer, node) subscribes /tf and /tf_static in a
/// callback group of its own and spins that group on a private executor
/// thread, so Buffer::setTransform would run outside LF at wall-clock times.
/// Right after such a node is constructed, the binding detaches the group from
/// that executor; the two subscriptions are then bound to the generator's
/// kGroupTf2Listener reaction IDs and their messages reach the Buffer from LF
/// reactions at the tag, like any other subscription (S0 gate of the plan).

#ifndef TF2_INGRESS_HPP_
#define TF2_INGRESS_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace rclcpp
{
class CallbackGroup;
}  // namespace rclcpp

namespace ros2_lf_binding
{

/// What the binding sees of one executor-associated callback group of a node.
struct Tf2GroupView
{
  std::vector<std::string> sub_topics;  ///< topic names of its subscriptions
  std::size_t other_entities = 0;  ///< timers, services, clients, waitables (no QoS events)
};

/// Index of the one group that is a TransformListener's (exactly the
/// subscriptions /tf and /tf_static, nothing else). Returns -1 and sets
/// `*error` when there is none or more than one.
int pick_tf2_listener_group(const std::vector<Tf2GroupView> & groups, std::string * error);

/// Detach the TransformListener group of the node `binding_key` names, if its
/// reaction IDs carry kGroupTf2Listener; a no-op otherwise. Refuses to start
/// (fail_bind) when such a node has no unique listener group or the group
/// does not stay detached. Called when the node's construction scope ends.
void detach_tf2_listener_groups(const std::string & binding_key);

/// True for a group detach_tf2_listener_groups detached and that is still
/// detached, so its handles are matched as kGroupTf2Listener candidates.
bool is_detached_tf2_group(const rclcpp::CallbackGroup * group);

/// Clear the flag again on every detached group something set back to true
/// (an executor callback in flight at detach time restores it on return) and
/// wake its executor. Returns the binding keys of those groups. A transform
/// the executor fed into the Buffer before this check is not undone.
std::vector<std::string> reassert_tf2_groups_detached();

}  // namespace ros2_lf_binding

#endif  // TF2_INGRESS_HPP_

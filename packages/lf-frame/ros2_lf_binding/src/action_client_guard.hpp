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

/// @file action_client_guard.hpp
/// @brief Action client responses of an inactive lifecycle node.
///
/// The action gate wraps the response callback of every goal, result and
/// cancel request an rclcpp_action client sends (action_client_guard in
/// action_gate_api.h). The counted take executes an inactive node's goal,
/// result and cancel responses under InactiveClientScope, so rclcpp erases
/// its pending entries, while the wrapper keeps the response from user code.
/// A response to a request sent before the node's last deactivate is kept
/// from user code the same way, even when it is taken after the node is
/// active again (each send carries the client's deactivate count):
/// - goal response: rclcpp's callback (promise, goal handle, the user's
///   goal-response callback, the result request) does not run. A goal the
///   server accepted is cancelled once.
/// - result response: runs only for a goal whose callbacks deactivate
///   stopped, which leaves no user callback and releases the goal handle.
/// - cancel response: runs only for a cancel the binding sent itself.
/// Every goal a running goal-response callback accepted is recorded, so
/// deactivate stops its feedback and result callbacks whether or not a result
/// request is pending.

#ifndef ACTION_CLIENT_GUARD_HPP_
#define ACTION_CLIENT_GUARD_HPP_

#include <cstdint>
#include <string>

#include "rclcpp_action/client.hpp"

namespace ros2_lf_binding
{

/// While alive, the action client responses this thread executes belong to
/// the lifecycle node `node_class`, which is not active at this tag.
class InactiveClientScope
{
public:
  explicit InactiveClientScope(const std::string & node_class);
  ~InactiveClientScope();
  InactiveClientScope(const InactiveClientScope &) = delete;
  InactiveClientScope & operator=(const InactiveClientScope &) = delete;

private:
  const std::string * previous_;
};

/// The lifecycle gate's deactivate hook for one action client: stop the
/// callbacks of each goal accepted since the last deactivate, then cancel all
/// goals, and start a new epoch. Returns the number of goals stopped, or -1
/// when no registered Ros2LfActionClientOps matches the client's type.
int64_t action_client_deactivate(rclcpp_action::ClientBase & client);

}  // namespace ros2_lf_binding

#endif  // ACTION_CLIENT_GUARD_HPP_

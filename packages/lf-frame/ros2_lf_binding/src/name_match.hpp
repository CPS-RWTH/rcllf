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
//
// Name rules that bind a service, client or action entry to a handle.
// Defined in node_binding.cpp.

#ifndef NAME_MATCH_HPP_
#define NAME_MATCH_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{

/// The names as written: equal, or equal but for one leading slash.
bool names_match(const std::string & handle_name, const std::string & rid_name);

/// True when a service, client or action handle named `handle_name` serves the
/// RID name: the name the node resolves `rid_name` to with its namespace and
/// remap rules, or, when rcl cannot resolve it, the names as written.
bool name_fits(
  const std::string & handle_name, const std::string & rid_name,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base);

/// The action name of an action server or client waitable, or "" when it
/// cannot be read.
std::string action_name_of(
  rclcpp::Waitable & waitable,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base);

/// True when an action waitable serves the RID's action. A waitable whose name
/// cannot be read serves none.
bool action_matches(
  rclcpp::Waitable & waitable, const std::string & rid_name,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base);

}  // namespace ros2_lf_binding

#endif  // NAME_MATCH_HPP_

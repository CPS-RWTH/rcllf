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

// Internal header - not installed, not part of the public API.
#ifndef NODE_REGISTRY_INTERNAL_H_
#define NODE_REGISTRY_INTERNAL_H_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

/// Clear all entries from the node registry.
/// Only call this before rclcpp::shutdown() in tests.
void ros2_lf_clear_node_registry();

/// Whether at least one node of a non-component class is registered, under
/// any binding key. Matches the class recorded at registration, not the key:
/// two or more instances are keyed `<class_name>__<suffix>`
/// (tools/colcon_lf/colcon_lf/generation/binding_key.py), and a registration
/// that names no class counts as the class its key spells. A registration
/// with a null node does not count.
bool ros2_lf_class_has_registered_node(const std::string & class_name);

/// Invoke fn once per registered non-component node, passing its base
/// interface. Base interface (not the concrete node) because that is the
/// only type every registered node shares regardless of its node base
/// (`rclcpp::Node`, `rclcpp_lifecycle::LifecycleNode`, ...).
void ros2_lf_for_each_registered_node(
  const std::function <
  void(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &) > &fn);

#endif  // NODE_REGISTRY_INTERNAL_H_

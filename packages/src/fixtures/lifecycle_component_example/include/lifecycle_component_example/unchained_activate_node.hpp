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

#ifndef LIFECYCLE_COMPONENT_EXAMPLE__UNCHAINED_ACTIVATE_NODE_HPP_
#define LIFECYCLE_COMPONENT_EXAMPLE__UNCHAINED_ACTIVATE_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace lifecycle_component_example
{

/// A DELIBERATELY BROKEN lifecycle node: the RCLLF-H005 fixture.
///
/// Its `on_activate` override does not chain to
/// `rclcpp_lifecycle::LifecycleNode::on_activate`, so its `LifecyclePublisher`
/// is never activated and every `publish()` is silently dropped. This is
/// exactly the bug that shipped in `lifecycle_example`.
///
/// Do not copy this node. Its sibling `LifecycleComponentNode`, in the same
/// package, is the correct shape and chains to the base. Keeping both here is
/// what lets the playground harness prove the check fires on the broken shape
/// and stays quiet on the correct one.
class UnchainedActivateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UnchainedActivateNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

private:
  void timer_callback();

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

}  // namespace lifecycle_component_example

#endif  // LIFECYCLE_COMPONENT_EXAMPLE__UNCHAINED_ACTIVATE_NODE_HPP_

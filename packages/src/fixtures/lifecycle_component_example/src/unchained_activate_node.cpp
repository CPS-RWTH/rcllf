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

#include "lifecycle_component_example/unchained_activate_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace lifecycle_component_example
{

UnchainedActivateNode::UnchainedActivateNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("unchained_activate_node", options), count_(0)
{
}

UnchainedActivateNode::CallbackReturn UnchainedActivateNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  publisher_ = this->create_publisher<std_msgs::msg::String>("unchained_output", 10);
  return CallbackReturn::SUCCESS;
}

// THE DEFECT THIS FIXTURE EXISTS FOR. No call to
// rclcpp_lifecycle::LifecycleNode::on_activate(state), so publisher_ is never
// activated and every publish() below is dropped. RCLLF-H005 must fire here.
//
// Kept deliberately broken. If you "fix" this node, the playground case for
// lifecycle.base_call_chaining stops proving anything, so fix the detection's
// test instead.
UnchainedActivateNode::CallbackReturn UnchainedActivateNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  timer_ = this->create_wall_timer(1s, std::bind(&UnchainedActivateNode::timer_callback, this));
  return CallbackReturn::SUCCESS;
}

void UnchainedActivateNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Unchained tick: " + std::to_string(count_++);
  publisher_->publish(message);
}

}  // namespace lifecycle_component_example

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_component_example::UnchainedActivateNode)

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

#include "lifecycle_component_example/lifecycle_component_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace lifecycle_component_example
{

LifecycleComponentNode::LifecycleComponentNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lifecycle_component_node", options), count_(0)
{
  RCLCPP_INFO(this->get_logger(), "LifecycleComponentNode has been created");
}

LifecycleComponentNode::CallbackReturn LifecycleComponentNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  publisher_ = this->create_publisher<std_msgs::msg::String>("lifecycle_component_output", 10);
  RCLCPP_INFO(this->get_logger(), "LifecycleComponentNode has been configured");
  return CallbackReturn::SUCCESS;
}

LifecycleComponentNode::CallbackReturn LifecycleComponentNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  // Chain to the base first. `LifecycleNode::on_activate` calls
  // `impl_->on_activate()`, which is what actually activates managed
  // publishers. An override that skips it leaves every LifecyclePublisher
  // deactivated, so `publish()` silently drops. That failure is nearly
  // invisible: `LifecyclePublisher` warns once and then latches
  // `should_log_ = false`, so a run that drops every message logs a single
  // line at the first tick and nothing after.
  const auto parent = rclcpp_lifecycle::LifecycleNode::on_activate(state);
  if (parent != CallbackReturn::SUCCESS) {
    return parent;
  }
  timer_ = this->create_wall_timer(1s, std::bind(&LifecycleComponentNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "LifecycleComponentNode has been activated");
  return CallbackReturn::SUCCESS;
}

LifecycleComponentNode::CallbackReturn LifecycleComponentNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  timer_.reset();
  RCLCPP_INFO(this->get_logger(), "LifecycleComponentNode has been deactivated");
  // Symmetric with on_activate: the base deactivates managed publishers.
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

LifecycleComponentNode::CallbackReturn LifecycleComponentNode::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  publisher_.reset();
  RCLCPP_INFO(this->get_logger(), "LifecycleComponentNode has been cleaned up");
  return CallbackReturn::SUCCESS;
}

void LifecycleComponentNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Lifecycle component tick: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

}  // namespace lifecycle_component_example

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_component_example::LifecycleComponentNode)

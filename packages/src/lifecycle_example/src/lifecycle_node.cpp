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

#include "lifecycle_example/lifecycle_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace lifecycle_example
{

LifecycleExampleNode::LifecycleExampleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lifecycle_node", options), count_(0)
{
  RCLCPP_INFO(this->get_logger(), "LifecycleExampleNode has been created");
}

LifecycleExampleNode::CallbackReturn LifecycleExampleNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  publisher_ = this->create_publisher<std_msgs::msg::String>("lifecycle_output", 10);
  RCLCPP_INFO(this->get_logger(), "LifecycleExampleNode has been configured");
  return CallbackReturn::SUCCESS;
}

LifecycleExampleNode::CallbackReturn LifecycleExampleNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  // Chain to the base FIRST. `LifecycleNode::on_activate` calls
  // `impl_->on_activate()`, which is what actually activates managed
  // publishers. Returning SUCCESS without it leaves every LifecyclePublisher
  // deactivated for the life of the process, so `publish()` drops every
  // message, after a single WARN (`LifecyclePublisher` logs only its first).
  const auto parent = rclcpp_lifecycle::LifecycleNode::on_activate(state);
  if (parent != CallbackReturn::SUCCESS) {
    return parent;
  }
  timer_ = this->create_wall_timer(1s, std::bind(&LifecycleExampleNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "LifecycleExampleNode has been activated");
  return CallbackReturn::SUCCESS;
}

LifecycleExampleNode::CallbackReturn LifecycleExampleNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  timer_.reset();
  RCLCPP_INFO(this->get_logger(), "LifecycleExampleNode has been deactivated");
  // Symmetric with on_activate: the base deactivates managed publishers.
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

LifecycleExampleNode::CallbackReturn LifecycleExampleNode::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  publisher_.reset();
  RCLCPP_INFO(this->get_logger(), "LifecycleExampleNode has been cleaned up");
  return CallbackReturn::SUCCESS;
}

void LifecycleExampleNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Lifecycle tick: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

}  // namespace lifecycle_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lifecycle_example::LifecycleExampleNode>(rclcpp::NodeOptions());

  // Drive the node to `active` before spinning. Without this the node sits in
  // `unconfigured` forever: the publisher is created in on_configure and the
  // timer in on_activate, so nothing is ever published and this binary does
  // nothing observable.
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "configure() failed");
    rclcpp::shutdown();
    return 1;
  }
  if (node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "activate() failed");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

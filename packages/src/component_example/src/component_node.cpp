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

#include "component_example/component_node.hpp"

#include <chrono>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace component_example
{

ComponentNode::ComponentNode(const rclcpp::NodeOptions & options)
: Node("component_node", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("component_output", 10);
  timer_ = this->create_wall_timer(1s, std::bind(&ComponentNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "ComponentNode has been started");
}

void ComponentNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Component tick: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

}  // namespace component_example

RCLCPP_COMPONENTS_REGISTER_NODE(component_example::ComponentNode)

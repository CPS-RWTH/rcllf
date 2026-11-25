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

#include "basic_components/dual_listener_node.hpp"

#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace basic_components
{

namespace
{
constexpr const char kInputTopicA[] = "basic_components/dual_in_a";
constexpr const char kInputTopicB[] = "basic_components/dual_in_b";
}  // namespace

DualListenerNode::DualListenerNode(const rclcpp::NodeOptions & options)
: Node("dual_listener", options)
{
  subscription_a_ = this->create_subscription<std_msgs::msg::String>(
    kInputTopicA, 10, std::bind(&DualListenerNode::on_message_a, this, std::placeholders::_1));
  subscription_b_ = this->create_subscription<std_msgs::msg::String>(
    kInputTopicB, 10, std::bind(&DualListenerNode::on_message_b, this, std::placeholders::_1));
}

void DualListenerNode::on_message_a(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received A: %s", msg->data.c_str());
}

void DualListenerNode::on_message_b(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received B: %s", msg->data.c_str());
}

}  // namespace basic_components

RCLCPP_COMPONENTS_REGISTER_NODE(basic_components::DualListenerNode)

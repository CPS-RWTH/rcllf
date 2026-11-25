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

#include "basic_components/listener_node.hpp"

#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace basic_components
{

namespace
{
constexpr const char kInputTopic[] = "basic_components/relay_out";
}

ListenerNode::ListenerNode(const rclcpp::NodeOptions & options)
: Node("listener_node", options)
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    kInputTopic, 10, std::bind(&ListenerNode::on_message, this, std::placeholders::_1));
}

void ListenerNode::on_message(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

}  // namespace basic_components

RCLCPP_COMPONENTS_REGISTER_NODE(basic_components::ListenerNode)

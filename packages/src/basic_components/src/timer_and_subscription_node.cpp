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

#include "basic_components/timer_and_subscription_node.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

namespace basic_components
{

namespace
{
constexpr const char kInputTopic[] = "basic_components/timer_sub_in";
constexpr const char kOutputTopic[] = "basic_components/timer_sub_out";
}  // namespace

TimerAndSubscriptionNode::TimerAndSubscriptionNode(const rclcpp::NodeOptions & options)
: Node("timer_and_subscription", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>(kOutputTopic, 10);
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    kInputTopic, 10, std::bind(&TimerAndSubscriptionNode::on_message, this, std::placeholders::_1));
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&TimerAndSubscriptionNode::on_timer, this));
}

void TimerAndSubscriptionNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = std::string("timer_and_subscription tick ") + std::to_string(count_++);
  publisher_->publish(message);
}

void TimerAndSubscriptionNode::on_message(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

}  // namespace basic_components

RCLCPP_COMPONENTS_REGISTER_NODE(basic_components::TimerAndSubscriptionNode)

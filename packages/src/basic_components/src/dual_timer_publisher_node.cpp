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

#include "basic_components/dual_timer_publisher_node.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

namespace basic_components
{

namespace
{
constexpr const char kOutputTopicA[] = "basic_components/dual_in_a";
constexpr const char kOutputTopicB[] = "basic_components/dual_in_b";
}  // namespace

DualTimerPublisherNode::DualTimerPublisherNode(const rclcpp::NodeOptions & options)
: Node("dual_timer_publisher", options), count_(0)
{
  publisher_a_ = this->create_publisher<std_msgs::msg::String>(kOutputTopicA, 10);
  publisher_b_ = this->create_publisher<std_msgs::msg::String>(kOutputTopicB, 10);
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&DualTimerPublisherNode::on_timer, this));
}

void DualTimerPublisherNode::on_timer()
{
  auto message_a = std_msgs::msg::String();
  auto message_b = std_msgs::msg::String();
  std::string payload = std::string("dual_timer_publisher tick ") + std::to_string(count_++);
  message_a.data = payload + " (a)";
  message_b.data = payload + " (b)";
  publisher_a_->publish(message_a);
  publisher_b_->publish(message_b);
}

}  // namespace basic_components

RCLCPP_COMPONENTS_REGISTER_NODE(basic_components::DualTimerPublisherNode)

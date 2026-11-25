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

#include "callback_group_example/callback_group_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace callback_group_example
{

CallbackGroupNode::CallbackGroupNode(const rclcpp::NodeOptions & options)
: Node("callback_group_node", options),
  count_default_(0),
  count_exclusive1_(0),
  count_exclusive2_(0),
  count_reentrant1a_(0),
  count_reentrant1b_(0),
  count_reentrant2_(0)
{
  exclusive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  reentrant_group_1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  reentrant_group_2_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  timer_default_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_default_callback, this));
  timer_exclusive1_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_exclusive1_callback, this), exclusive_group_);
  timer_exclusive2_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_exclusive2_callback, this), exclusive_group_);
  timer_reentrant1a_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_reentrant1a_callback, this), reentrant_group_1_);
  timer_reentrant1b_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_reentrant1b_callback, this), reentrant_group_1_);
  timer_reentrant2_ = this->create_wall_timer(
    500ms, std::bind(&CallbackGroupNode::timer_reentrant2_callback, this), reentrant_group_2_);

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    std::bind(&CallbackGroupNode::chatter_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Callback group node started.");
}

static void busy_work()
{
  volatile uint64_t sum = 0;
  for (uint64_t i = 0; i < 10000000; ++i) {
    sum += i;
  }
  (void)sum;
}

void CallbackGroupNode::timer_default_callback()
{
  RCLCPP_INFO(this->get_logger(), "[default] tick %zu start", count_default_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[default] tick %zu end", count_default_++);
}

void CallbackGroupNode::timer_exclusive1_callback()
{
  RCLCPP_INFO(this->get_logger(), "[exclusive_1] tick %zu start", count_exclusive1_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[exclusive_1] tick %zu end", count_exclusive1_++);
}

void CallbackGroupNode::timer_exclusive2_callback()
{
  RCLCPP_INFO(this->get_logger(), "[exclusive_2] tick %zu start", count_exclusive2_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[exclusive_2] tick %zu end", count_exclusive2_++);
}

void CallbackGroupNode::timer_reentrant1a_callback()
{
  RCLCPP_INFO(this->get_logger(), "[reentrant_1a] tick %zu start", count_reentrant1a_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[reentrant_1a] tick %zu end", count_reentrant1a_++);
}

void CallbackGroupNode::timer_reentrant1b_callback()
{
  RCLCPP_INFO(this->get_logger(), "[reentrant_1b] tick %zu start", count_reentrant1b_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[reentrant_1b] tick %zu end", count_reentrant1b_++);
}

void CallbackGroupNode::timer_reentrant2_callback()
{
  RCLCPP_INFO(this->get_logger(), "[reentrant_2] tick %zu start", count_reentrant2_);
  busy_work();
  RCLCPP_INFO(this->get_logger(), "[reentrant_2] tick %zu end", count_reentrant2_++);
}

void CallbackGroupNode::chatter_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "[default/chatter] Received: '%s'", msg->data.c_str());
}

}  // namespace callback_group_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<callback_group_example::CallbackGroupNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

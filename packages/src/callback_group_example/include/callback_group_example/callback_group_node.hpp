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

#ifndef CALLBACK_GROUP_EXAMPLE__CALLBACK_GROUP_NODE_HPP_
#define CALLBACK_GROUP_EXAMPLE__CALLBACK_GROUP_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace callback_group_example
{

class CallbackGroupNode : public rclcpp::Node
{
public:
  explicit CallbackGroupNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_default_callback();
  void timer_exclusive1_callback();
  void timer_exclusive2_callback();
  void timer_reentrant1a_callback();
  void timer_reentrant1b_callback();
  void timer_reentrant2_callback();
  void chatter_callback(const std_msgs::msg::String::SharedPtr msg);

  size_t count_default_;
  size_t count_exclusive1_;
  size_t count_exclusive2_;
  size_t count_reentrant1a_;
  size_t count_reentrant1b_;
  size_t count_reentrant2_;

  rclcpp::CallbackGroup::SharedPtr exclusive_group_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_1_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_2_;

  rclcpp::TimerBase::SharedPtr timer_default_;
  rclcpp::TimerBase::SharedPtr timer_exclusive1_;
  rclcpp::TimerBase::SharedPtr timer_exclusive2_;
  rclcpp::TimerBase::SharedPtr timer_reentrant1a_;
  rclcpp::TimerBase::SharedPtr timer_reentrant1b_;
  rclcpp::TimerBase::SharedPtr timer_reentrant2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace callback_group_example

#endif  // CALLBACK_GROUP_EXAMPLE__CALLBACK_GROUP_NODE_HPP_

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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace basic_components
{

class DualListenerNode : public rclcpp::Node
{
public:
  explicit DualListenerNode(const rclcpp::NodeOptions & options);

private:
  void on_message_a(const std_msgs::msg::String::SharedPtr msg);
  void on_message_b(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_a_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_b_;
};

}  // namespace basic_components

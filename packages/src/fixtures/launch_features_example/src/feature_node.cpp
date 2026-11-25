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

#include "launch_features_example/feature_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace launch_features_example
{

FeatureNode::FeatureNode(const rclcpp::NodeOptions & options)
: Node("feature_node", options)
{
  this->declare_parameter<int64_t>("timer_period_ms", 500);
  this->declare_parameter<std::string>("message_prefix", "hello");

  auto period_ms = this->get_parameter("timer_period_ms").as_int();
  message_prefix_ = this->get_parameter("message_prefix").as_string();

  publisher_ = this->create_publisher<std_msgs::msg::String>("feature_output", 10);
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "feature_input", 10,
    std::bind(&FeatureNode::subscription_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&FeatureNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "FeatureNode started (period=%ldms)", period_ms);
}

void FeatureNode::timer_callback()
{
  auto msg = std_msgs::msg::String();
  msg.data = message_prefix_ + ": feature tick";
  RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  publisher_->publish(msg);
}

void FeatureNode::subscription_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received: '%s'", msg->data.c_str());
}

}  // namespace launch_features_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<launch_features_example::FeatureNode>());
  rclcpp::shutdown();
  return 0;
}

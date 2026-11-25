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

#include "handle_matching_example/twin_subscriber_node.hpp"

#include <functional>
#include <memory>

namespace handle_matching_example
{

TwinSubscriberNode::TwinSubscriberNode(const rclcpp::NodeOptions & options)
: Node("twin_subscriber", options)
{
  first_ = this->create_subscription<std_msgs::msg::Int64>(
    "twin", 10, std::bind(&TwinSubscriberNode::on_first, this, std::placeholders::_1));
  second_ = this->create_subscription<std_msgs::msg::Int64>(
    "twin", 10, std::bind(&TwinSubscriberNode::on_second, this, std::placeholders::_1));
}

void TwinSubscriberNode::on_first(const std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "TWIN first %ld", static_cast<long>(msg->data));  // NOLINT
}

void TwinSubscriberNode::on_second(const std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "TWIN second %ld", static_cast<long>(msg->data));  // NOLINT
}

}  // namespace handle_matching_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handle_matching_example::TwinSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

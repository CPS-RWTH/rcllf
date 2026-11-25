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

#include "handle_matching_example/lambda_timer_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace handle_matching_example
{

LambdaTimerNode::LambdaTimerNode(const rclcpp::NodeOptions & options)
: Node("lambda_timer", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("lambda_count", 10);
  // Fed by twin_publisher, not by this node: a node that subscribes to its own
  // topic is a self loop, which the generator does not support.
  subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "twin", 10, [this](std_msgs::msg::Int64::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "LAMBDA got %ld", static_cast<long>(msg->data));  // NOLINT
    });
  timer_ = this->create_wall_timer(
    100ms, [this]() {
      std_msgs::msg::Int64 msg;
      msg.data = count_++;
      RCLCPP_INFO(this->get_logger(), "LAMBDA tick %ld", static_cast<long>(msg.data));  // NOLINT
      publisher_->publish(msg);
    });
}

}  // namespace handle_matching_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handle_matching_example::LambdaTimerNode>());
  rclcpp::shutdown();
  return 0;
}

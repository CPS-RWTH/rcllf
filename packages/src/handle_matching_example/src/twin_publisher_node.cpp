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

#include "handle_matching_example/twin_publisher_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace handle_matching_example
{

TwinPublisherNode::TwinPublisherNode(const rclcpp::NodeOptions & options)
: Node("twin_publisher", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("twin", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&TwinPublisherNode::timer_callback, this));
}

void TwinPublisherNode::timer_callback()
{
  std_msgs::msg::Int64 msg;
  msg.data = count_++;
  publisher_->publish(msg);
}

}  // namespace handle_matching_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handle_matching_example::TwinPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

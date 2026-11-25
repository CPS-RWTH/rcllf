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

#include "reentrant_fanout_example/fanout_publisher_node.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

namespace reentrant_fanout_example
{

FanoutPublisherNode::FanoutPublisherNode()
: Node("fanout_publisher"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("work", 10);
  timer_ = this->create_wall_timer(20ms, std::bind(&FanoutPublisherNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "FanoutPublisher started at 50 Hz on '/work'");
}

void FanoutPublisherNode::timer_callback()
{
  auto msg = std_msgs::msg::Int64();
  msg.data = count_++;
  publisher_->publish(msg);
}

}  // namespace reentrant_fanout_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<reentrant_fanout_example::FanoutPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

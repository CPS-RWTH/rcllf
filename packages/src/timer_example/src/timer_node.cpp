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

#include "timer_example/timer_node.hpp"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace timer_example
{

TimerNode::TimerNode(const rclcpp::NodeOptions & options)
: Node("timer_node", options), count_(0)
{
  timer_ = this->create_wall_timer(1000ms, std::bind(&TimerNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Timer node started.");
}

void TimerNode::timer_callback() {RCLCPP_INFO(this->get_logger(), "Timer tick: %zu", count_++);}

}  // namespace timer_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<timer_example::TimerNode>());
  rclcpp::shutdown();
  return 0;
}

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

#ifndef MAIN_ONLY_EXAMPLE__COUNTER_NODE_HPP_
#define MAIN_ONLY_EXAMPLE__COUNTER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace main_only_example
{

// Header-only node. Its source file (counter_main.cpp) holds only main().
class CounterNode : public rclcpp::Node
{
public:
  explicit CounterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("counter_node", options)
  {
    using std::chrono_literals::operator""ms;
    pub_ = create_publisher<std_msgs::msg::Int32>("count", 10);
    timer_ = create_wall_timer(100ms, std::bind(&CounterNode::tick, this));
  }

private:
  void tick()
  {
    std_msgs::msg::Int32 msg;
    msg.data = count_++;
    RCLCPP_INFO(get_logger(), "count=%d", msg.data);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_{0};
};

}  // namespace main_only_example

#endif  // MAIN_ONLY_EXAMPLE__COUNTER_NODE_HPP_

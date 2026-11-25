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

#include "basic_components/timer_only_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace basic_components
{

TimerOnlyNode::TimerOnlyNode(const rclcpp::NodeOptions & options)
: Node("timer_only", options), count_(0)
{
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(1s, std::bind(&TimerOnlyNode::on_timer, this));
}

void TimerOnlyNode::on_timer() {RCLCPP_INFO(this->get_logger(), "Timer only tick %zu", count_++);}

}  // namespace basic_components

RCLCPP_COMPONENTS_REGISTER_NODE(basic_components::TimerOnlyNode)

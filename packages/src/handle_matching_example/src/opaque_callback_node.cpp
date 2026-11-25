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

#include "handle_matching_example/opaque_callback_node.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

namespace handle_matching_example
{

OpaqueCallbackNode::OpaqueCallbackNode(const rclcpp::NodeOptions & options)
: Node("opaque_callback", options), fast_(true), count_(0)
{
  tick_ = [this]() {
      RCLCPP_INFO(this->get_logger(), "OPAQUE tick %ld", static_cast<long>(count_++));  // NOLINT
    };
  timer_ = this->create_wall_timer(100ms, tick_);
  // Which method runs is decided at run time, so the analyzer cannot pick one.
  chosen_timer_ = this->create_wall_timer(
    100ms, fast_ ? std::bind(&OpaqueCallbackNode::on_fast, this) :
    std::bind(&OpaqueCallbackNode::on_slow, this));
  chosen_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "twin", 10,
    fast_ ? std::bind(&OpaqueCallbackNode::on_msg_fast, this, std::placeholders::_1) :
    std::bind(&OpaqueCallbackNode::on_msg_slow, this, std::placeholders::_1));
}

void OpaqueCallbackNode::on_fast()
{
  RCLCPP_INFO(this->get_logger(), "OPAQUE fast");
}

void OpaqueCallbackNode::on_slow()
{
  RCLCPP_INFO(this->get_logger(), "OPAQUE slow");
}

void OpaqueCallbackNode::on_msg_fast(std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "OPAQUE fast got %ld", static_cast<long>(msg->data));  // NOLINT
}

void OpaqueCallbackNode::on_msg_slow(std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "OPAQUE slow got %ld", static_cast<long>(msg->data));  // NOLINT
}

}  // namespace handle_matching_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handle_matching_example::OpaqueCallbackNode>());
  rclcpp::shutdown();
  return 0;
}

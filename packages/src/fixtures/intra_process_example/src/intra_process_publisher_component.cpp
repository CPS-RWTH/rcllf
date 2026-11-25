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

#include "intra_process_example/intra_process_publisher_component.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace intra_process_example
{

IntraProcessPublisherComponent::IntraProcessPublisherComponent(const rclcpp::NodeOptions & options)
: Node("intra_process_publisher", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("intra_topic", 10);
  // 100 Hz
  timer_ = this->create_wall_timer(
    10ms, std::bind(&IntraProcessPublisherComponent::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "IntraProcessPublisher started at 100 Hz on '/intra_topic'");
}

void IntraProcessPublisherComponent::timer_callback()
{
  auto msg = std::make_unique<std_msgs::msg::Int64>();
  msg->data = count_++;
  // Log the address so the subscriber can confirm zero-copy (same pointer).
  RCLCPP_DEBUG(
    this->get_logger(), "Publishing %ld at addr %p", msg->data,
    static_cast<void *>(msg.get()));
  publisher_->publish(std::move(msg));
}

}  // namespace intra_process_example

RCLCPP_COMPONENTS_REGISTER_NODE(intra_process_example::IntraProcessPublisherComponent)

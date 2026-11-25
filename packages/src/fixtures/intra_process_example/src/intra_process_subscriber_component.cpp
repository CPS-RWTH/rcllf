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

#include "intra_process_example/intra_process_subscriber_component.hpp"

#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace intra_process_example
{

IntraProcessSubscriberComponent::IntraProcessSubscriberComponent(
  const rclcpp::NodeOptions & options)
: Node("intra_process_subscriber", options)
{
  subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "intra_topic", 10,
    std::bind(&IntraProcessSubscriberComponent::message_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "IntraProcessSubscriber started on '/intra_topic'");
}

void IntraProcessSubscriberComponent::message_callback(std::unique_ptr<std_msgs::msg::Int64> msg)
{
  // When intra-process comms are active the unique_ptr is moved directly from
  // publisher to subscriber - the address here matches what the publisher logged.
  RCLCPP_INFO(
    this->get_logger(), "Received %ld at addr %p (zero-copy if addr matches publisher)",
    msg->data, static_cast<void *>(msg.get()));
}

}  // namespace intra_process_example

RCLCPP_COMPONENTS_REGISTER_NODE(intra_process_example::IntraProcessSubscriberComponent)

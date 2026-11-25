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

#include "dynamic_topic_example/dynamic_publisher_node.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace dynamic_topic_example
{

DynamicPublisherNode::DynamicPublisherNode(const rclcpp::NodeOptions & options)
: Node("dynamic_publisher", options)
{
  instance_id_ = this->declare_parameter<int64_t>("instance_id", 0);

  // RCLLF-H004: the topic name is built at runtime from a parameter. A static
  // analyzer sees the "chatter_" literal plus additional operands and cannot
  // know the final name — before Phase 7 the analyzer silently recorded
  // "chatter_" as if it were complete. The edge is genuinely absent from the
  // extracted graph; the diagnostic says so out loud.
  dynamic_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "chatter_" + std::to_string(instance_id_), 10);

  // Plain string literal — fully resolvable, joins with static_subscriber_node.
  status_publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);

  timer_ = this->create_wall_timer(
    500ms, std::bind(&DynamicPublisherNode::timer_callback, this));
  RCLCPP_INFO(
    this->get_logger(), "Dynamic publisher node started (instance_id=%s).",
    std::to_string(instance_id_).c_str());
}

void DynamicPublisherNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "instance " + std::to_string(instance_id_);
  dynamic_publisher_->publish(message);
  status_publisher_->publish(message);
}

}  // namespace dynamic_topic_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamic_topic_example::DynamicPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

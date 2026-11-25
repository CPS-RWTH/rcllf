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

#include "qos_hazard_example/shallow_queue_subscriber_node.hpp"

namespace qos_hazard_example
{

ShallowQueueSubscriberNode::ShallowQueueSubscriberNode(const rclcpp::NodeOptions & options)
: Node("shallow_queue_subscriber", options)
{
  subscription_ = this->create_subscription<custom_interfaces::msg::SensorData>(
    "shallow_queue_topic", 1,
    std::bind(&ShallowQueueSubscriberNode::topic_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Shallow-queue subscriber node started.");
}

void ShallowQueueSubscriberNode::topic_callback(
  const custom_interfaces::msg::SensorData::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: temp=%.1f", msg->temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::ShallowQueueSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

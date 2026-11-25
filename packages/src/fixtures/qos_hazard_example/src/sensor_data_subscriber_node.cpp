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

#include "qos_hazard_example/sensor_data_subscriber_node.hpp"

namespace qos_hazard_example
{

SensorDataSubscriberNode::SensorDataSubscriberNode(const rclcpp::NodeOptions & options)
: Node("sensor_data_subscriber", options)
{
  subscription_ = this->create_subscription<custom_interfaces::msg::SensorData>(
    "sensor_data_topic", rclcpp::SensorDataQoS(),
    std::bind(&SensorDataSubscriberNode::topic_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "SensorDataQoS subscriber node started.");
}

void SensorDataSubscriberNode::topic_callback(
  const custom_interfaces::msg::SensorData::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: temp=%.1f", msg->temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::SensorDataSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

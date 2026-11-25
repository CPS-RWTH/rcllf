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

#include "qos_hazard_example/chained_qos_subscriber_node.hpp"

namespace qos_hazard_example
{

ChainedQosSubscriberNode::ChainedQosSubscriberNode(const rclcpp::NodeOptions & options)
: Node("chained_qos_subscriber", options)
{
  reliable_volatile_keep_last_ = this->create_subscription<custom_interfaces::msg::SensorData>(
    "chained_reliable_volatile_topic",
    rclcpp::QoS(10).reliable().durability_volatile().keep_last(3),
    std::bind(
      &ChainedQosSubscriberNode::reliable_volatile_keep_last_callback, this,
      std::placeholders::_1));

  transient_local_keep_all_ = this->create_subscription<custom_interfaces::msg::SensorData>(
    "chained_transient_local_topic",
    rclcpp::QoS(10).transient_local().keep_all(),
    std::bind(
      &ChainedQosSubscriberNode::transient_local_keep_all_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Chained-QoS subscriber node started.");
}

void ChainedQosSubscriberNode::reliable_volatile_keep_last_callback(
  const custom_interfaces::msg::SensorData::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard (reliable/volatile/keep_last 3): temp=%.1f",
    msg->temperature);
}

void ChainedQosSubscriberNode::transient_local_keep_all_callback(
  const custom_interfaces::msg::SensorData::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard (transient_local/keep_all): temp=%.1f",
    msg->temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::ChainedQosSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

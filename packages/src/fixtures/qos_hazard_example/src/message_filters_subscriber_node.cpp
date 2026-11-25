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

#include "qos_hazard_example/message_filters_subscriber_node.hpp"

namespace qos_hazard_example
{

MessageFiltersSubscriberNode::MessageFiltersSubscriberNode(const rclcpp::NodeOptions & options)
: Node("message_filters_subscriber", options)
{
  sub_a_.subscribe(this, "sync_topic_a");
  sub_b_.subscribe(this, "sync_topic_b");
  sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimePolicy>>(
    ApproxTimePolicy(10), sub_a_, sub_b_);
  sync_->registerCallback(
    std::bind(
      &MessageFiltersSubscriberNode::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "message_filters subscriber node started.");
}

void MessageFiltersSubscriberNode::sync_callback(
  const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
  const custom_interfaces::msg::SensorData::ConstSharedPtr & b)
{
  RCLCPP_INFO(
    this->get_logger(), "synced: temp_a=%.1f temp_b=%.1f", a->temperature, b->temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::MessageFiltersSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

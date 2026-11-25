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

#ifndef QOS_HAZARD_EXAMPLE__SENSOR_DATA_SUBSCRIBER_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__SENSOR_DATA_SUBSCRIBER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// Named-profile QoS fixture: rclcpp::SensorDataQoS() carries no chained
// .best_effort()/.keep_last() calls, so this exercises named-profile
// recognition in _extract_qos_attrs (the QoS hardening) rather than
// the literal-spelling scan the best_effort fixture exercises.
class SensorDataSubscriberNode : public rclcpp::Node
{
public:
  explicit SensorDataSubscriberNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void topic_callback(const custom_interfaces::msg::SensorData::SharedPtr msg);
  rclcpp::Subscription<custom_interfaces::msg::SensorData>::SharedPtr subscription_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__SENSOR_DATA_SUBSCRIBER_NODE_HPP_

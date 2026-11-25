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

#ifndef QOS_HAZARD_EXAMPLE__CHAINED_QOS_SUBSCRIBER_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__CHAINED_QOS_SUBSCRIBER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// Fixture: chained `rclcpp::QoS` builder calls that must resolve
// their stated reliability/durability/history/depth instead of falling back
// to RCLLF-H000 ("undetermined").
//
// - `reliable_volatile_keep_last_` exercises the chain
//   `.reliable().durability_volatile()` plus a trailing `.keep_last(3)`
//   that must override the base constructor's depth of 10.
// - `transient_local_keep_all_` exercises `.transient_local()` and
//   `.keep_all()`, the two builder methods not otherwise covered by any
//   in-tree fixture.
class ChainedQosSubscriberNode : public rclcpp::Node
{
public:
  explicit ChainedQosSubscriberNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void reliable_volatile_keep_last_callback(
    const custom_interfaces::msg::SensorData::SharedPtr msg);
  void transient_local_keep_all_callback(
    const custom_interfaces::msg::SensorData::SharedPtr msg);

  rclcpp::Subscription<custom_interfaces::msg::SensorData>::SharedPtr
    reliable_volatile_keep_last_;
  rclcpp::Subscription<custom_interfaces::msg::SensorData>::SharedPtr
    transient_local_keep_all_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__CHAINED_QOS_SUBSCRIBER_NODE_HPP_

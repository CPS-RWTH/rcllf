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

#ifndef BNA_EXAMPLE__ANCHOR_SENSOR_NODE_HPP_
#define BNA_EXAMPLE__ANCHOR_SENSOR_NODE_HPP_

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bna_example
{

// Publishes the anchor stream on bna/anchor: tick k is stamped now() and
// carries k in temperature. Stops after kAnchorTicks ticks.
class AnchorSensorNode : public rclcpp::Node
{
public:
  explicit AnchorSensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  int tick_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bna_example

#endif  // BNA_EXAMPLE__ANCHOR_SENSOR_NODE_HPP_

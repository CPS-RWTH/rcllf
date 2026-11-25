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

#ifndef FUSION_EXAMPLE__SENSOR_A_NODE_HPP_
#define FUSION_EXAMPLE__SENSOR_A_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fusion_example
{

// Sensor A: publishes on fusion/a every 30 ms, stamped with the node
// clock (this->now()), or with logical_stamp_origin_ns + seq * 30 ms when
// that parameter is >= 0. temperature carries the publish sequence number.
class SensorANode : public rclcpp::Node
{
public:
  explicit SensorANode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  static constexpr int64_t kPeriodNs = 30'000'000;
  int64_t seq_;
  int64_t logical_stamp_origin_ns_;
};

}  // namespace fusion_example

#endif  // FUSION_EXAMPLE__SENSOR_A_NODE_HPP_

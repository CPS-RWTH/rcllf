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

#ifndef BNA_EXAMPLE__CANDIDATE_SENSOR_NODE_HPP_
#define BNA_EXAMPLE__CANDIDATE_SENSOR_NODE_HPP_

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bna_example
{

// Publishes the candidate stream on bna/candidate: tick k publishes one
// sample per entry of candidate_offsets_ms()[k], stamped now() plus the
// offset, carrying k in temperature.
class CandidateSensorNode : public rclcpp::Node
{
public:
  explicit CandidateSensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  size_t tick_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bna_example

#endif  // BNA_EXAMPLE__CANDIDATE_SENSOR_NODE_HPP_

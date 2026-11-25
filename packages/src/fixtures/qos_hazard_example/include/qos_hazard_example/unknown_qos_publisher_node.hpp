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

#ifndef QOS_HAZARD_EXAMPLE__UNKNOWN_QOS_PUBLISHER_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__UNKNOWN_QOS_PUBLISHER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// Fixture: publishers whose reliability the analysis cannot see.
// "loose" is on no edge, "known" is analyzed reliable, and two twins share
// "twin_pub", one analyzed best effort and one unknown. The unknown ones run
// best effort when the comma separated QOS_HAZARD_BEST_EFFORT environment
// variable names their key ("loose", "twin"), reliable otherwise.
class UnknownQosPublisherNode : public rclcpp::Node
{
public:
  explicit UnknownQosPublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr loose_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr known_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr twin_best_effort_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr twin_unknown_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__UNKNOWN_QOS_PUBLISHER_NODE_HPP_

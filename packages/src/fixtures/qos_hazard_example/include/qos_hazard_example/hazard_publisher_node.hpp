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

#ifndef QOS_HAZARD_EXAMPLE__HAZARD_PUBLISHER_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__HAZARD_PUBLISHER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// Publishes on every topic the hazard subscriber nodes subscribe to, so
// RCLLF-H002 (shallow queue) has an in-model publisher to join against.
class HazardPublisherNode : public rclcpp::Node
{
public:
  explicit HazardPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr best_effort_topic_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr sensor_data_topic_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr shallow_queue_topic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__HAZARD_PUBLISHER_NODE_HPP_

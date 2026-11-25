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

#include "qos_hazard_example/hazard_publisher_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace qos_hazard_example
{

HazardPublisherNode::HazardPublisherNode(const rclcpp::NodeOptions & options)
: Node("hazard_publisher", options)
{
  best_effort_topic_pub_ =
    this->create_publisher<custom_interfaces::msg::SensorData>("best_effort_topic", 10);
  sensor_data_topic_pub_ =
    this->create_publisher<custom_interfaces::msg::SensorData>("sensor_data_topic", 10);
  shallow_queue_topic_pub_ =
    this->create_publisher<custom_interfaces::msg::SensorData>("shallow_queue_topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&HazardPublisherNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Hazard publisher node started.");
}

void HazardPublisherNode::timer_callback()
{
  auto message = custom_interfaces::msg::SensorData();
  message.header.stamp = this->now();
  message.temperature = 21.0;
  message.humidity = 45.0;
  message.pressure = 1012.0;
  best_effort_topic_pub_->publish(message);
  sensor_data_topic_pub_->publish(message);
  shallow_queue_topic_pub_->publish(message);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::HazardPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

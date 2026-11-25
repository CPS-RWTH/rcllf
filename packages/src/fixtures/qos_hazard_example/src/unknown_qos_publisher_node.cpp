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

#include "qos_hazard_example/unknown_qos_publisher_node.hpp"

#include <chrono>
#include <cstdlib>
#include <string>

using namespace std::chrono_literals;

namespace qos_hazard_example
{

namespace
{

// QoS depth 10, reliable unless QOS_HAZARD_BEST_EFFORT names the key. The key
// holds no QoS word, as the analysis matches such words in the argument text.
rclcpp::QoS runtime_qos(const std::string & key)
{
  rclcpp::QoS qos(10);
  const char * env = std::getenv("QOS_HAZARD_BEST_EFFORT");
  if (env != nullptr &&
    ("," + std::string(env) + ",").find("," + key + ",") != std::string::npos)
  {
    qos.best_effort();
  }
  return qos;
}

}  // namespace

UnknownQosPublisherNode::UnknownQosPublisherNode(const rclcpp::NodeOptions & options)
: Node("unknown_qos_publisher", options)
{
  loose_pub_ = this->create_publisher<custom_interfaces::msg::SensorData>(
    "loose_topic", runtime_qos("loose"));
  known_pub_ = this->create_publisher<custom_interfaces::msg::SensorData>(
    "known_topic", rclcpp::QoS(10).reliable());
  twin_best_effort_pub_ = this->create_publisher<custom_interfaces::msg::SensorData>(
    "twin_pub_topic", rclcpp::QoS(10).best_effort());
  twin_unknown_pub_ = this->create_publisher<custom_interfaces::msg::SensorData>(
    "twin_pub_topic", runtime_qos("twin"));
  timer_ = this->create_wall_timer(
    500ms, std::bind(&UnknownQosPublisherNode::timer_callback, this));
}

void UnknownQosPublisherNode::timer_callback()
{
  auto message = custom_interfaces::msg::SensorData();
  message.header.stamp = this->now();
  message.temperature = 22.0;
  loose_pub_->publish(message);
  known_pub_->publish(message);
  twin_best_effort_pub_->publish(message);
  twin_unknown_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "PUBLISHED temp=%.1f", message.temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::UnknownQosPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

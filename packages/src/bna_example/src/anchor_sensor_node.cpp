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

#include "bna_example/anchor_sensor_node.hpp"

#include <chrono>

#include "bna_example/schedule.hpp"

using namespace std::chrono_literals;

namespace bna_example
{

AnchorSensorNode::AnchorSensorNode(const rclcpp::NodeOptions & options)
: Node("anchor_sensor", options), tick_(0)
{
  publisher_ = this->create_publisher<custom_interfaces::msg::SensorData>("bna/anchor", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&AnchorSensorNode::timer_callback, this));
}

void AnchorSensorNode::timer_callback()
{
  if (tick_ >= kAnchorTicks) {
    return;
  }
  auto message = custom_interfaces::msg::SensorData();
  message.header.stamp = this->now();
  message.header.frame_id = "anchor";
  message.temperature = static_cast<double>(tick_);
  ++tick_;
  publisher_->publish(message);
}

}  // namespace bna_example

#ifndef BNA_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bna_example::AnchorSensorNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

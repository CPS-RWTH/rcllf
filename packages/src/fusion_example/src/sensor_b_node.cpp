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

#include "fusion_example/sensor_b_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace fusion_example
{

SensorBNode::SensorBNode(const rclcpp::NodeOptions & options)
: Node("sensor_b", options), seq_(0)
{
  // Stock fairness variant: with an origin >= 0, seq k is
  // stamped origin + k * period instead of the wall clock, so a stock run
  // differs from the logical pair list only through arrival order.
  logical_stamp_origin_ns_ =
    this->declare_parameter<int64_t>("logical_stamp_origin_ns", -1);
  publisher_ = this->create_publisher<custom_interfaces::msg::SensorData>("fusion/b", 10);
  timer_ = this->create_wall_timer(50ms, std::bind(&SensorBNode::timer_callback, this));
}

void SensorBNode::timer_callback()
{
  auto message = custom_interfaces::msg::SensorData();
  if (logical_stamp_origin_ns_ >= 0) {
    message.header.stamp =
      rclcpp::Time(logical_stamp_origin_ns_ + seq_ * kPeriodNs, RCL_ROS_TIME);
  } else {
    message.header.stamp = this->now();
  }
  message.header.frame_id = "sensor_b";
  message.temperature = static_cast<double>(seq_);
  ++seq_;
  publisher_->publish(message);
}

}  // namespace fusion_example

// main() is compiled out when this file is built into a component library
// (FUSION_COMPONENT_ONLY); the standalone executable keeps it.
#ifndef FUSION_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fusion_example::SensorBNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

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

#ifndef SENTRY_PIPELINE__IMU_NODE_HPP_
#define SENTRY_PIPELINE__IMU_NODE_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace sentry_pipeline
{

// Hot-path node: high-rate inertial sensor, 200 Hz by default. Publishes
// synthetic IMU readings on /sentry/imu.
class ImuNode : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options);

private:
  void timer_callback();

  int64_t busy_us_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  PublishLog imu_log_;  // after imu_pub_: closes after it
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__IMU_NODE_HPP_

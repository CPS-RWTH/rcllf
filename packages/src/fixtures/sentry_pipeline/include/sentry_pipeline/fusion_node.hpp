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

#ifndef SENTRY_PIPELINE__FUSION_NODE_HPP_
#define SENTRY_PIPELINE__FUSION_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/fused_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sentry_pipeline
{

// Hot-path node: sensor fusion, reactive on both /sentry/imu and
// /sentry/scan. Publishes a fused state estimate on /sentry/state.
class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options);

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  int64_t busy_us_;
  rclcpp::Publisher<custom_interfaces::msg::FusedState>::SharedPtr state_pub_;
  PublishLog state_log_;  // after state_pub_: closes after it
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__FUSION_NODE_HPP_

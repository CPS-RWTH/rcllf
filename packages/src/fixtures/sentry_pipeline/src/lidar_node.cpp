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

#include "sentry_pipeline/lidar_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "sentry_pipeline/scenario_env.hpp"
#include "sentry_pipeline/workload.hpp"

namespace sentry_pipeline
{

LidarNode::LidarNode(const rclcpp::NodeOptions & options)
: Node("lidar_node", options)
{
  this->declare_parameter<double>("rate_hz", 10.0);
  this->declare_parameter<int64_t>("qos_depth", 10);
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<int64_t>("busy_us", 500);

  const auto overlay = sentry_pipeline::ScenarioOverlay::load_from_env();
  const auto qos_profile = sentry_pipeline::qos_from_overlay(*this, overlay, "lidar_node");
  busy_us_ = sentry_pipeline::int_from_overlay(
    overlay, "lidar_node", "busy_us", this->get_parameter("busy_us").as_int());

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sentry/scan", qos_profile);
  scan_log_.open(*this, *scan_pub_);

  const double rate_hz = this->get_parameter("rate_hz").as_double();
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz)),
    std::bind(&LidarNode::timer_callback, this));
}

void LidarNode::timer_callback()
{
  sentry_pipeline::busy_wait(std::chrono::microseconds(busy_us_));

  auto message = sensor_msgs::msg::LaserScan();
  message.header.stamp = this->now();
  message.header.frame_id = "lidar_link";
  if (scan_log_.record(message)) {
    scan_pub_->publish(message);
  }
}

}  // namespace sentry_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_pipeline::LidarNode)

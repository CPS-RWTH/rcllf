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

#include "sentry_pipeline/fusion_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "sentry_pipeline/scenario_env.hpp"
#include "sentry_pipeline/workload.hpp"

namespace sentry_pipeline
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("fusion_node", options)
{
  this->declare_parameter<int64_t>("qos_depth", 10);
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<int64_t>("busy_us", 500);

  const auto overlay = sentry_pipeline::ScenarioOverlay::load_from_env();
  const auto qos_profile = sentry_pipeline::qos_from_overlay(*this, overlay, "fusion_node");
  busy_us_ = sentry_pipeline::int_from_overlay(
    overlay, "fusion_node", "busy_us", this->get_parameter("busy_us").as_int());

  state_pub_ = this->create_publisher<custom_interfaces::msg::FusedState>(
    "/sentry/state", qos_profile);
  state_log_.open(*this, *state_pub_);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/sentry/imu", qos_profile, std::bind(&FusionNode::imu_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/sentry/scan", qos_profile,
    std::bind(&FusionNode::scan_callback, this, std::placeholders::_1));
}

void FusionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  (void)msg;
  sentry_pipeline::busy_wait(std::chrono::microseconds(busy_us_));

  auto message = custom_interfaces::msg::FusedState();
  message.header.stamp = this->now();
  if (state_log_.record(message)) {
    state_pub_->publish(message);
  }
}

void FusionNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  (void)msg;
  sentry_pipeline::busy_wait(std::chrono::microseconds(busy_us_));

  auto message = custom_interfaces::msg::FusedState();
  message.header.stamp = this->now();
  if (state_log_.record(message)) {
    state_pub_->publish(message);
  }
}

}  // namespace sentry_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_pipeline::FusionNode)

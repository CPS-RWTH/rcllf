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

#include "sentry_pipeline/detector_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "sentry_pipeline/scenario_env.hpp"
#include "sentry_pipeline/workload.hpp"

namespace sentry_pipeline
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("detector_node", options)
{
  this->declare_parameter<int64_t>("qos_depth", 5);
  this->declare_parameter<std::string>("qos_reliability_image", "best_effort");
  this->declare_parameter<std::string>("qos_reliability_detections", "reliable");
  this->declare_parameter<int64_t>("busy_us", 8000);
  this->declare_parameter<int64_t>("payload_size", 32);

  const auto overlay = sentry_pipeline::ScenarioOverlay::load_from_env();
  const auto qos_profile_image = sentry_pipeline::qos_from_overlay(
    *this, overlay, "detector_node", "qos_reliability_image");
  const auto qos_profile_detections = sentry_pipeline::qos_from_overlay(
    *this, overlay, "detector_node", "qos_reliability_detections");
  busy_us_ = sentry_pipeline::int_from_overlay(
    overlay, "detector_node", "busy_us", this->get_parameter("busy_us").as_int());
  payload_size_ = sentry_pipeline::int_from_overlay(
    overlay, "detector_node", "payload_size", this->get_parameter("payload_size").as_int());

  detections_pub_ = this->create_publisher<custom_interfaces::msg::Detections>(
    "/sentry/detections", qos_profile_detections);
  detections_log_.open(*this, *detections_pub_);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/sentry/image", qos_profile_image,
    std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));
}

void DetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  (void)msg;
  sentry_pipeline::busy_wait(std::chrono::microseconds(busy_us_));

  auto message = custom_interfaces::msg::Detections();
  message.header.stamp = this->now();
  message.count = 0;
  message.boxes.assign(payload_size_, 0.0);
  if (detections_log_.record(message)) {
    detections_pub_->publish(message);
  }
}

}  // namespace sentry_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_pipeline::DetectorNode)

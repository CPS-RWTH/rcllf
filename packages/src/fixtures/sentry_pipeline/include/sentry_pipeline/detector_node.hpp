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

#ifndef SENTRY_PIPELINE__DETECTOR_NODE_HPP_
#define SENTRY_PIPELINE__DETECTOR_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/detections.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace sentry_pipeline
{

// Side-path node: "heavy" object detector, reactive on /sentry/image, with a
// parameterized busy-loop standing in for real detection work. Publishes
// detections on /sentry/detections.
//
// Uses two distinct QoS profiles (a second profile for the
// image subscription): the /sentry/image subscription is best_effort (matching
// CameraNode's publisher) and the /sentry/detections publisher is reliable
// (matching PlannerNode's subscriber; DiagnosticsSinkNode's best_effort
// subscriber remains compatible with a reliable publisher). A single shared
// profile cannot satisfy both pairings at once, so this node is the one
// exception to "at most one profile per node" the naming rule already
// anticipates via the `qos_profile_<topic_suffix>` convention.
class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  int64_t busy_us_;
  int64_t payload_size_;
  rclcpp::Publisher<custom_interfaces::msg::Detections>::SharedPtr detections_pub_;
  PublishLog detections_log_;  // after detections_pub_: closes after it
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__DETECTOR_NODE_HPP_

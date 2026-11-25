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

#ifndef SENTRY_PIPELINE__CAMERA_NODE_HPP_
#define SENTRY_PIPELINE__CAMERA_NODE_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace sentry_pipeline
{

// Side-path node: vision sensor, 25 Hz by default (R5: chosen for an
// integer-millisecond period, 40 ms). Publishes a small synthetic image
// (zeroed fill, sized per the scenario's payload_size, not a real image
// codec) on /sentry/image.
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options);

private:
  void timer_callback();

  int64_t busy_us_;
  int64_t payload_size_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  PublishLog image_log_;  // after image_pub_: closes after it
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__CAMERA_NODE_HPP_

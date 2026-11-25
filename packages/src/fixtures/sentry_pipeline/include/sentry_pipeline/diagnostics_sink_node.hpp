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

#ifndef SENTRY_PIPELINE__DIAGNOSTICS_SINK_NODE_HPP_
#define SENTRY_PIPELINE__DIAGNOSTICS_SINK_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/detections.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_pipeline
{

// Side-path node: terminal sink, reactive on /sentry/detections. No
// publisher — the end of the side chain.
class DiagnosticsSinkNode : public rclcpp::Node
{
public:
  explicit DiagnosticsSinkNode(const rclcpp::NodeOptions & options);

private:
  void detections_callback(const custom_interfaces::msg::Detections::SharedPtr msg);

  int64_t busy_us_;
  rclcpp::Subscription<custom_interfaces::msg::Detections>::SharedPtr detections_sub_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__DIAGNOSTICS_SINK_NODE_HPP_

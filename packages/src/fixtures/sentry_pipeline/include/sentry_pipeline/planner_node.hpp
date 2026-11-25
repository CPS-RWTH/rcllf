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

#ifndef SENTRY_PIPELINE__PLANNER_NODE_HPP_
#define SENTRY_PIPELINE__PLANNER_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/detections.hpp"
#include "custom_interfaces/msg/fused_state.hpp"
#include "custom_interfaces/msg/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"

namespace sentry_pipeline
{

// The deliberate timer-gated node (timer plus subscriptions): merges the hot
// (fused state) and side (detections) paths at a fixed 50 Hz rate,
// publishing a trajectory plan on /sentry/plan. This is the one node in the
// topology that has both a timer and at least one subscription, making
// PlannerNode->ControllerNode the pipeline's single gated hop.
class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options);

private:
  void state_callback(const custom_interfaces::msg::FusedState::SharedPtr msg);
  void detections_callback(const custom_interfaces::msg::Detections::SharedPtr msg);
  void timer_callback();

  int64_t busy_us_;
  int64_t payload_size_;
  rclcpp::Publisher<custom_interfaces::msg::Plan>::SharedPtr plan_pub_;
  PublishLog plan_log_;  // after plan_pub_: closes after it
  rclcpp::Subscription<custom_interfaces::msg::FusedState>::SharedPtr state_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Detections>::SharedPtr detections_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__PLANNER_NODE_HPP_

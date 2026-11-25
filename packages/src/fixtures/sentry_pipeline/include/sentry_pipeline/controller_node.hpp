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

#ifndef SENTRY_PIPELINE__CONTROLLER_NODE_HPP_
#define SENTRY_PIPELINE__CONTROLLER_NODE_HPP_

#include <cstdint>

#include "custom_interfaces/msg/plan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_pipeline/msg_log.hpp"

namespace sentry_pipeline
{

// Hot-path node: control loop, purely event-driven (no timer) — reacts to /sentry/plan and
// publishes /sentry/cmd.
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options);

private:
  void plan_callback(const custom_interfaces::msg::Plan::SharedPtr msg);

  int64_t busy_us_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  PublishLog cmd_log_;  // after cmd_pub_: closes after it
  rclcpp::Subscription<custom_interfaces::msg::Plan>::SharedPtr plan_sub_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__CONTROLLER_NODE_HPP_

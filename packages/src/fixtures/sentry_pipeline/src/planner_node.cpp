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

#include "sentry_pipeline/planner_node.hpp"

#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "sentry_pipeline/scenario_env.hpp"
#include "sentry_pipeline/workload.hpp"

namespace sentry_pipeline
{

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{
  this->declare_parameter<double>("rate_hz", 50.0);
  this->declare_parameter<int64_t>("qos_depth", 10);
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<int64_t>("busy_us", 1000);
  this->declare_parameter<int64_t>("payload_size", 16);

  const auto overlay = sentry_pipeline::ScenarioOverlay::load_from_env();
  const auto qos_profile = sentry_pipeline::qos_from_overlay(*this, overlay, "planner_node");
  busy_us_ = sentry_pipeline::int_from_overlay(
    overlay, "planner_node", "busy_us", this->get_parameter("busy_us").as_int());
  payload_size_ = sentry_pipeline::int_from_overlay(
    overlay, "planner_node", "payload_size", this->get_parameter("payload_size").as_int());

  plan_pub_ = this->create_publisher<custom_interfaces::msg::Plan>("/sentry/plan", qos_profile);
  plan_log_.open(*this, *plan_pub_);
  state_sub_ = this->create_subscription<custom_interfaces::msg::FusedState>(
    "/sentry/state", qos_profile,
    std::bind(&PlannerNode::state_callback, this, std::placeholders::_1));
  detections_sub_ = this->create_subscription<custom_interfaces::msg::Detections>(
    "/sentry/detections", qos_profile,
    std::bind(&PlannerNode::detections_callback, this, std::placeholders::_1));

  const double rate_hz = this->get_parameter("rate_hz").as_double();
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz)),
    std::bind(&PlannerNode::timer_callback, this));
}

void PlannerNode::state_callback(const custom_interfaces::msg::FusedState::SharedPtr msg)
{
  (void)msg;
}

void PlannerNode::detections_callback(const custom_interfaces::msg::Detections::SharedPtr msg)
{
  (void)msg;
}

void PlannerNode::timer_callback()
{
  sentry_pipeline::busy_wait(std::chrono::microseconds(busy_us_));

  auto message = custom_interfaces::msg::Plan();
  message.header.stamp = this->now();
  message.waypoints.assign(payload_size_, 0.0);
  if (plan_log_.record(message)) {
    plan_pub_->publish(message);
  }
}

}  // namespace sentry_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_pipeline::PlannerNode)

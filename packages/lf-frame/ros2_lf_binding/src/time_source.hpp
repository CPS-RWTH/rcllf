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

#ifndef TIME_SOURCE_HPP_
#define TIME_SOURCE_HPP_

#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription_base.hpp"

namespace ros2_lf_binding
{

// Shared RCL_ROS_TIME clock driven by LF logical time.
// Initialized by lf_clock_init(); accessed by reaction_dispatch.cpp.
extern std::shared_ptr<rclcpp::Clock> g_lf_clock;

// Create g_lf_clock and enable ROS time override on it.
// Called once from ros2_lf_runtime_init before components are loaded.
void lf_clock_init();

// Update g_lf_clock and every attached node clock to the current LF logical
// time. Called at the top of every ros2_lf_invoke_timer / ros2_lf_invoke_sub.
void lf_time_advance_ros_clock();

// NodeOptions for a node the binding constructs without seeing its type (a
// component): use_sim_time forced true and no clock thread, so the node's
// TimeSource owns a /clock subscription that nothing spins.
rclcpp::NodeOptions lf_time_node_options(rclcpp::NodeOptions options);

// Find the TimeSource /clock subscription of a node built with
// lf_time_node_options() and feed it the logical time synchronously at every
// lf_time_advance_ros_clock(). Returns false when the node has none, which
// leaves its clock on physical time.
bool lf_attach_time_source(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & key);

// True iff `sub` is a TimeSource /clock subscription fed by the binding.
bool lf_is_time_source_subscription(const rclcpp::SubscriptionBase * sub);

// True iff `group` is a callback group rclcpp created for a TimeSource clock
// thread (recorded by ros2_lf_detach_sim_time). Its handles are rclcpp's own
// and take no part in handle matching.
bool lf_is_rclcpp_owned_group(const rclcpp::CallbackGroup * group);
void lf_mark_rclcpp_owned_group(const rclcpp::CallbackGroup::SharedPtr & group);

// True once the first LF tag was pushed into the node clocks.
bool lf_time_started();

// Tests only: drop g_lf_clock and every attached clock and subscription.
void lf_clock_reset_for_test();

}  // namespace ros2_lf_binding

#endif  // TIME_SOURCE_HPP_

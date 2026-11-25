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

#ifndef CLOCK_PUBLISHER_HPP_
#define CLOCK_PUBLISHER_HPP_

#include <memory>
#include "rclcpp/node.hpp"

namespace ros2_lf_binding
{

enum class ClockPublishMode
{
  Off,             // Never publish. ADR 0001 baseline.
  OnTimeChange,    // Publish when logical time component changes (default).
  OnEveryReaction,  // Publish on every unique (time, microstep) tag.
};

// Create the publisher node and publishers. Called once from
// ros2_lf_runtime_init after g_executor exists.
void clock_publisher_init(ClockPublishMode mode);

// Returns the publisher node so runtime.cpp can add it to g_executor.
// Returns nullptr in Off mode.
rclcpp::Node::SharedPtr clock_publisher_node();

// Evaluate the current LF tag and publish if the configured mode requires it.
// Called from lf_time_advance_ros_clock() after rcl_set_ros_time_override.
void clock_publisher_tick();

// Destroy the publisher node. Called from ros2_lf_runtime_shutdown.
void clock_publisher_shutdown();

}  // namespace ros2_lf_binding

#endif  // CLOCK_PUBLISHER_HPP_

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

/// @file clock_gate_api.h
/// @brief Hooks the executable-side clock gate calls.
///
/// clock_gate.cpp (installed to share/ros2_lf_binding/src and compiled into
/// every generated executable next to publication_gate.cpp) defines
/// rclcpp::Clock::now, rclcpp::Clock::sleep_until,
/// rcl_clock_add_jump_callback and rcl_clock_fini, forwards each to the next definition and
/// calls these hooks. A hook either returns or ends the process with a
/// message that names the node and callback.

#ifndef ROS2_LF_BINDING__CLOCK_GATE_API_H_
#define ROS2_LF_BINDING__CLOCK_GATE_API_H_

#include <stdint.h>

#include "rcl/time.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Before rclcpp::Clock::sleep_until waits on the RCL_ROS_TIME `clock` until
 * `until_ns`. Ends the process when the caller is a reaction and
 * the binding attached the clock and its override is on, so the sleep could
 * never return. Outside a reaction, or on any other clock, it returns.
 */
void rcllf_clock_gate_on_sleep(rcl_clock_t * clock, int64_t until_ns);

/**
 * Before rcl_clock_add_jump_callback adds `callback` to the RCL_ROS_TIME
 * `clock`, whether it is rclcpp's trampoline for a JumpHandler or
 * a raw rcl callback. `in_sleep` is 1 for the handler
 * rclcpp::Clock::sleep_until adds for itself and 2 for the one
 * rclcpp::ClockConditionalVariable::wait_until adds, which rclcpp_action's
 * goal expire thread waits on. Both only wake their waiter. A
 * wait_until handler added in a reaction is still refused. Ends the process
 * for a forward handler added to a clock the binding attached, which LF time
 * would fire at every tag.
 */
void rcllf_clock_gate_on_jump_handler(
  rcl_clock_t * clock, const rcl_jump_threshold_t * threshold, const void * callback,
  int in_sleep);

/**
 * Before rcl_clock_fini finalizes `clock`. Forgets the clock, so a later clock
 * at the same address is not taken for it.
 */
void rcllf_clock_gate_on_clock_fini(rcl_clock_t * clock);

/**
 * After rclcpp::Clock::now() read `value` from the RCL_ROS_TIME `clock`.
 * `caller` is the return address of Clock::now(), which tells a
 * user read from rclcpp's own (a parameter event stamp in every Node
 * constructor). Reports a read before the first tag and a read of a clock
 * that does not follow LF logical time inside a reaction. Never ends the
 * process and never changes the value.
 */
void rcllf_clock_gate_on_ros_read(rcl_clock_t * clock, int64_t value, const void * caller);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_LF_BINDING__CLOCK_GATE_API_H_

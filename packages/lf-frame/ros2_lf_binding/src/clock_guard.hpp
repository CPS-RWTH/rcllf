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

/// @file clock_guard.hpp
/// @brief Clock uses rcllf cannot drive from LF logical time:
///        a ROS-time sleep in a reaction, a forward jump handler, a
///        program with several LF environments, a node's own ROS clock and a
///        read before the first tag. The executable-side hooks (clock_gate_api.h)
///        call into this file.

#ifndef CLOCK_GUARD_HPP_
#define CLOCK_GUARD_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rcl/time.h"

namespace ros2_lf_binding
{

/// Message texts, kept apart from the checks so tests match them exactly.
/// `node` is the binding key, `ros_nodes` the fully qualified names of the
/// ROS nodes created under it, empty when none is known.
std::string ros_sleep_refusal_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback, int64_t until_ns, int64_t now_ns);
std::string jump_handler_refusal_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback, int64_t min_forward_ns);
std::string multi_env_refusal_message(int n_envs);
std::string own_clock_read_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback);
std::string pre_tag_read_message(
  const std::string & node, const std::vector<std::string> & ros_nodes, int64_t value_ns);

/// FATAL log with `message`, flush, then _Exit(EXIT_FAILURE). Exiting rather
/// than throwing: dispatch paths catch std::runtime_error from user code, and
/// static destructors must not run while LF workers still hold binding state.
[[noreturn]] void refuse_clock_use(const std::string & message);

/// Refuses an LF program with more than one environment.
void check_single_environment(int n_envs);

/// Marks `clock` as attached to LF logical time by the binding's own call
/// (g_lf_clock, ros2_lf_attach_node_clock). rcl_clock_fini forgets it.
void register_lf_clock(const rcl_clock_t * clock);

/// Records the node and callback a dispatch site is about to run, for the
/// messages above. Read only while a UserCallbackScope is active.
void set_current_callback(const std::string & node, const std::string & callback);

/// Number of own-clock and pre-tag warnings since process start (tests).
uint64_t clock_diagnostic_count();
/// Test hook: when true, the gate hooks do nothing (falsification of the
/// guard's own tests). Never set outside tests.
void set_clock_guard_disabled_for_test(bool disabled);
/// Forgets which warnings were already logged once and which clocks are
/// registered (tests).
void clock_guard_reset_for_test();

}  // namespace ros2_lf_binding

#endif  // CLOCK_GUARD_HPP_

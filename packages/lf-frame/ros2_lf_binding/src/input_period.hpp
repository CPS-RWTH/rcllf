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


/// @file input_period.hpp
/// @brief Runtime check of a declared minimum input period.
///
/// `min_input_period_ms` in execution.yaml bounds how often a subscription
/// callback may run, and the generator sizes the correspondence bounds of the
/// edges it feeds from it (`window // period + 1` invocations in a window). A
/// faster input than declared can overflow those bounds. The binding checks
/// the declaration at every run of the callback: two runs whose LF tags are
/// closer than the period (several at one tag included) are a violation. Each
/// violation is counted, the first is logged at once with both identities,
/// and a summary per declared input is logged at shutdown. Nothing is dropped
/// or delayed; the bounds themselves report their own overflow.

#ifndef INPUT_PERIOD_HPP_
#define INPUT_PERIOD_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/message_info.hpp"

namespace ros2_lf_binding
{
namespace input_period
{

/// Bind time: the subscription handle of `reaction_id` (binding key
/// `node_class`, analyzed topic `topic`) is checked when a declaration for
/// that pair was registered (register_min_input_period).
void attach(int reaction_id, const std::string & node_class, const std::string & topic);

/// One run of the subscription callback of `reaction_id` at LF tag
/// (`time`, `microstep`) for the sample named `identity`. No-op for a
/// reaction without a declaration.
void observe(int reaction_id, int64_t time, uint32_t microstep, const std::string & identity);

/// observe() at the current LF tag for the sample `info` describes: the
/// call each dispatch site makes right before it runs the callback.
void observe_message(int reaction_id, const rclcpp::MessageInfo & info);

/// `writer gid hex:publication sequence number`, the identity observe() logs.
std::string identity(const uint8_t * gid, size_t gid_len, uint64_t sequence);

struct Summary
{
  int reaction_id = -1;
  std::string node_class;
  std::string topic;
  int64_t period_ns = 0;
  uint64_t runs = 0;
  uint64_t violations = 0;
  int64_t min_gap_ns = -1;  ///< smallest gap between two runs, -1 before the second
  // The first violation: the run that came too early and the run before it.
  int64_t first_time = 0;
  uint32_t first_microstep = 0;
  std::string first_identity;
  int64_t first_prev_time = 0;
  uint32_t first_prev_microstep = 0;
  std::string first_prev_identity;
};

/// One summary per attached declaration, in attach order.
std::vector<Summary> summaries();

/// Logs one `ros2_lf_input_period:` line per attached declaration, a WARN
/// when it has violations, else INFO.
void log_summary();

/// Tests: forget every declaration and attachment.
void reset_for_test();

}  // namespace input_period
}  // namespace ros2_lf_binding

#endif  // INPUT_PERIOD_HPP_

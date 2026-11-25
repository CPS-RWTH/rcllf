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

/// @file output_deadline.hpp
/// @brief Per-publisher output deadline check.
///
/// A generated reaction arms each bounded topic of its callback right before
/// the callback runs (ros2_lf_output_deadline_arm). The executable's
/// publication gate calls check() immediately before the real rcl publish, so
/// the time the callback spent before its publish counts. A publish is late
/// when lf_time_physical() - (reaction tag) > output_deadline_ns. A late
/// publish is traced (actuation_late) at once and logged after the reaction's
/// callback returns, never inside the gate, through rcutils' console handler
/// only, so the line never publishes on /rosout;
/// the message is always published, unchanged. This is not an LF deadline.

#ifndef OUTPUT_DEADLINE_HPP_
#define OUTPUT_DEADLINE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <utility>

namespace ros2_lf_binding
{
namespace output_deadline
{

/// Registers `resolved_topic` with `bound_ns` for reaction `rid`. Idempotent;
/// a different bound for the same (rid, topic) replaces the old one.
void arm(int rid, const std::string & resolved_topic, int64_t bound_ns);

/// Gate hook, called just before the real publish of `resolved_topic` by the
/// reaction `rid` (pass the running reaction, -1 outside one). Returns true
/// when the publish is late. Never logs and never blocks the publish; the
/// caller publishes whatever this returns.
bool check(int rid, const std::string & resolved_topic);

/// Whether (rid, resolved_topic) carries a bound (the recorder's
/// actuator publications).
bool armed(int rid, const std::string & resolved_topic);

/// Logs the late publishes this thread queued since the last flush. Called
/// when a reaction's origin scope closes, outside the publication gate.
void flush_pending();

/// WARN that `topic` of reaction `rid` could not be resolved against its node
/// and is checked as written. Console only, like the late lines.
void warn_unresolved_topic(int rid, const char * topic);

/// One summary line per armed topic (checked, late, max lateness), shutdown.
void log_summary();

struct Stats
{
  int64_t bound_ns = 0;
  uint64_t checked = 0;
  uint64_t late = 0;
  int64_t max_lateness_ns = INT64_MIN;
};

/// Per (rid, resolved topic) counters (tests and the summary).
std::map<std::pair<int, std::string>, Stats> stats();

/// Test support: forget every armed topic and counter.
void reset_for_test();

}  // namespace output_deadline
}  // namespace ros2_lf_binding

#endif  // OUTPUT_DEADLINE_HPP_

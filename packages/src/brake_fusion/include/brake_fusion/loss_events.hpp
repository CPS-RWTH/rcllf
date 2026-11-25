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

#ifndef BRAKE_FUSION__LOSS_EVENTS_HPP_
#define BRAKE_FUSION__LOSS_EVENTS_HPP_

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <functional>
#include <string>

#include "brake_fusion/jsonl_log.hpp"
#include "rclcpp/rclcpp.hpp"

// rcllf binding: what a deadline / message_lost callback was
// raised for. Declared weak and mirrored here, so the node builds and runs
// without the binding (stock) and with a binding that lacks the call; the
// pointer is then null and the context fields are left out of the log line.
extern "C" {
typedef struct brake_fusion_lost_ctx
{
  int64_t intended_elapsed_ns;
  uint64_t ordinal;
  uint8_t gid[16];
  int32_t tardy;
} brake_fusion_lost_ctx_t;
int ros2_lf_message_lost_context(brake_fusion_lost_ctx_t * out) __attribute__((weak));
}

namespace brake_fusion
{

// QoS deadlines (pilot P7; the stop-margin study for the fused one): D is the
// midpoint of the largest nominal interval and the smallest interval with
// one lost message, so it fires for one loss and not in nominal traffic with
// the largest jitter margin on both sides. For a periodic topic of period P
// those are P and 2 P, so D = 1.5 P: at P any lateness fires, at 2 P a
// single loss is missed. Stock ROS 2 runs D on wall time. rcllf runs it on
// logical time: the binding runs the deadline handler once per
// missing message at its tag (publication correspondence, corr_runtime.cpp),
// and once per D window with no message at all, at the tags t_ref + D,
// t_ref + 2 D, ... (t_ref: the tag of the last message, or the start tag
// before the first; silence_deadline.hpp). A missing message restarts the
// window, so one loss is reported once. A producer that publishes nothing
// is reported from t0 + D on, where stock reports nothing (see the history
// note below and ControllerNode::wheel_startup_check).
// Publisher and subscriber use the same value, since a requested deadline
// shorter than the offered one (infinite by default) is an incompatible QoS.
// /radar/range has two writers (radar, radar2) on one keyless topic, and a
// DDS deadline runs per instance, not per writer: one lost radar sample
// leaves a 20 ms gap in stock, which no D can tell from nominal traffic.
constexpr std::chrono::microseconds kRadarDeadline{30'000};    // period 20 ms
constexpr std::chrono::microseconds kCameraDeadline{45'000};   // period 30 ms
constexpr std::chrono::microseconds kBrakeDeadline{37'500};    // period 25 ms
// /wheel/speed is the exception: 1.5 P = 15 ms
// fired in nominal stock traffic on a lab workstation (5 of 20 stock runs, one
// each, callback gaps up to 18.8 ms after publication gaps of 10 to 14.2 ms,
// every published sample delivered). One lost sample (20 ms) cannot be told
// from that jitter on wall time, and it is harmless: the controller keeps the
// previous wheel speed, and the oracle covers one lost wheel sample anywhere
// in the run (loss_scan.py). The deadline detects a silent wheel sensor, so
// D = 2.5 P = 25 ms, the midpoint of one loss (2 P) and two consecutive
// losses (3 P), 6.2 ms above the largest nominal gap measured.
constexpr std::chrono::microseconds kWheelDeadline{25'000};    // period 10 ms, 2.5 P
// /fused/range: ApproximateTime publishes one pair per camera message, at
// intervals of 20 to 40 ms, and one lost camera sample leaves exactly 60 ms
// wherever it falls (ideal-timing oracle, t2approx, test_retune.py), so D = 50 ms.
// 1.5 times the 30 ms camera period, 45 ms, fired in nominal stock traffic
// (recorded Orin runs: a 44.6 ms startup interval, 40.3 ms at most after it, 9 runs;
// one lost camera sample 60.0 ms). ExactTime and the first-radar rule pair
// equal stamps, every 60 ms (lcm of 20 and 30), one loss 120 ms: 1.5 P.
constexpr std::chrono::microseconds kFusedApproxDeadline{50'000};  // intervals 40, loss 60 ms
constexpr std::chrono::microseconds kFusedPairDeadline{90'000};    // period 60 ms

// History depth KeepLast(10) on every topic except /sim/state.
// The literal stays at each call site because the rcllf analyzer reads it.
// Basis: on the fastest of these topics (/wheel/speed, 10 ms) ten samples
// span 100 ms, more than the largest deadline above (90 ms), so the history
// is never what decides whether a sample is lost before a deadline could
// have been raised. It does not make a stalled reader raise one: the
// requested deadline watches arrivals at the reader's DDS entity, which
// continue while its callbacks stall, so a stalled executor raises nothing,
// and the samples its history overwrites are not reported as lost either
//. A writer that stalls or a link that breaks after
// the first sample stops arrivals and does raise it (stock). A writer that
// never publishes raises nothing: the deadline counts per instance, and an
// instance exists only after its first sample (wheel0 run).
// On the slower topics
// the history spans more (camera 300 ms, brake 250 ms).
// /sim/state, one sample per 1 ms step, needs kSimStateDepth = 100 for the
// same reason (sim_state.hpp).

// Number of deadline windows of length d_ns that ended in
// (t0_ns, now_ns], each one deadline event (the stock startup timeout of a
// stream with no first sample). 0 for d_ns <= 0 or now_ns <= t0_ns.
inline int64_t startup_windows_elapsed(int64_t t0_ns, int64_t now_ns, int64_t d_ns)
{
  return d_ns <= 0 || now_ns <= t0_ns ? 0 : (now_ns - t0_ns) / d_ns;
}

// One JSON line per call: ev "deadline" or "lost", the topic, the event
// counts, t_ns (node clock: logical time under rcllf), mono_ns (steady clock)
// and, when the rcllf binding names the missing message, its context.
inline void log_loss_event(
  JsonlLog & log, const rclcpp::Clock::SharedPtr & clock, const char * ev,
  const std::string & topic, int64_t total, int64_t change)
{
  const int64_t mono = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  brake_fusion_lost_ctx_t ctx{};
  const bool have_ctx =
    ros2_lf_message_lost_context != nullptr && ros2_lf_message_lost_context(&ctx) == 1;
  if (have_ctx) {
    log.line(
      "\"ev\":\"%s\",\"topic\":\"%s\",\"total\":%" PRId64 ",\"change\":%" PRId64
      ",\"t_ns\":%" PRId64 ",\"mono_ns\":%" PRId64 ",\"ctx\":true"
      ",\"intended_elapsed_ns\":%" PRId64 ",\"ordinal\":%" PRIu64 ",\"tardy\":%d",
      ev, topic.c_str(), total, change, clock->now().nanoseconds(), mono,
      ctx.intended_elapsed_ns, ctx.ordinal, static_cast<int>(ctx.tardy));
  } else {
    log.line(
      "\"ev\":\"%s\",\"topic\":\"%s\",\"total\":%" PRId64 ",\"change\":%" PRId64
      ",\"t_ns\":%" PRId64 ",\"mono_ns\":%" PRId64 ",\"ctx\":false",
      ev, topic.c_str(), total, change, clock->now().nanoseconds(), mono);
  }
}

// Subscription options with a deadline and a message_lost handler that log.
// Takes the node's clock, not the node: the rcllf analyzer cannot bound the
// publishes of a node whose `this` escapes into a call.
inline rclcpp::SubscriptionOptions loss_options(
  JsonlLog & log, rclcpp::Clock::SharedPtr clock, const std::string & topic)
{
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.deadline_callback =
    [&log, clock, topic](rclcpp::QOSDeadlineRequestedInfo & info) {
      log_loss_event(log, clock, "deadline", topic, info.total_count, info.total_count_change);
    };
  options.event_callbacks.message_lost_callback =
    [&log, clock, topic](rclcpp::QOSMessageLostInfo & info) {
      log_loss_event(
        log, clock, "lost", topic, static_cast<int64_t>(info.total_count),
        static_cast<int64_t>(info.total_count_change));
    };
  return options;
}

// A message_lost handler that logs, for a subscription whose
// options are a local variable of the creating method: the rcllf analyzer
// follows such a variable and sees that no deadline handler is assigned, while
// loss_options() above is opaque to it (the analyzer skips the bodies of
// functions defined in headers) and leaves the callback's publishes unbounded.
inline std::function<void(rclcpp::QOSMessageLostInfo & )> lost_logger(
  JsonlLog & log, rclcpp::Clock::SharedPtr clock, const std::string & topic)
{
  return [&log, clock, topic](rclcpp::QOSMessageLostInfo & info) {
           log_loss_event(
             log, clock, "lost", topic, static_cast<int64_t>(info.total_count),
             static_cast<int64_t>(info.total_count_change));
         };
}

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__LOSS_EVENTS_HPP_

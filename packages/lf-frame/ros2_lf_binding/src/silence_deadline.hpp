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

/// @file silence_deadline.hpp
/// @brief Logical deadline of a silent periodic edge.
///
/// A subscription that declares a QoS deadline D and registers a
/// requested-deadline-missed handler declares its input periodic: at least
/// one message every D. The correspondence reports each missing message at
/// its own tag (corr_runtime.cpp), but a producer that publishes nothing
/// makes no record. This window raises the deadline of a silent producer
/// at logical tags instead.
///
/// The rule. Let t_ref be the tag time of the last record the drain delivered
/// on the subscription's edge (matched or missing, any member of a fan-in
/// group), or the start tag time if none was delivered yet. When no record is
/// delivered at a tag at or before (t_ref + D, 0), the deadline handler runs
/// once at (t_ref + D, 0), and once more at every (t_ref + k D, 0) while the
/// silence lasts. Records due at the window's end tag are drained first, so a
/// record released exactly at t_ref + D closes the window in time.
///
/// No double report. A missing message is a record: the correspondence runs
/// the deadline handler for it at its tag, and that tag restarts the window
/// (t_ref := its tag). So one lost sample of a stream with period P < D gives
/// exactly one deadline call (the per-sample one), never a window call too:
/// the next record comes at most P after it, before the window ends.
///
/// Stock behaviour mirrored. Fast DDS (eProsima Fast DDS 2.14,
/// src/cpp/fastdds/subscriber/DataReaderImpl.cpp, DataReaderImpl::deadline_missed
/// and deadline_timer_reschedule) runs one timer of period D per instance:
/// each expiry without a new sample increments total_count by one, calls
/// on_requested_deadline_missed once, and restarts the timer for the next D;
/// a sample restarts it too. rclcpp hands that status to the
/// deadline_callback as QOSDeadlineRequestedInfo{total_count,
/// total_count_change} (rclcpp/event_handler.hpp, rmw
/// rmw_requested_deadline_missed_status_t). So this window gives one call per
/// elapsed window with total_count_change = 1, and total_count continues the
/// per-sample count of the same subscription. One deliberate difference: Fast
/// DDS starts an instance's timer only at its first sample, so a stream that
/// never starts raises nothing in stock. rcllf starts the first window
/// at the start tag, so a producer that never publishes is reported at
/// t0 + D, t0 + 2 D, ... A keyless topic is one DDS instance, which is why a
/// fan-in group (several writers, one reader) shares one window.
///
/// Determinism. Every input is a logical tag (record release tags, the start
/// and stop tags), so the same trace gives the same windows on replay.

#ifndef SILENCE_DEADLINE_HPP_
#define SILENCE_DEADLINE_HPP_

#include <cstdint>
#include <set>
#include <utility>

#include "rmw/types.h"

namespace ros2_lf_binding
{
namespace silence_deadline
{

/// Pure window state of one subscription (the lead edge of its drain).
class Window
{
public:
  /// Arm with deadline `period_ns` (> 0) and the first window's start
  /// `ref_ns` (the start tag time). A period <= 0 leaves the window disarmed.
  void arm(int64_t period_ns, int64_t ref_ns);
  bool armed() const {return period_ns_ > 0;}
  int64_t period_ns() const {return period_ns_;}
  /// Start time of the window that ends next.
  int64_t ref_ns() const {return ref_ns_;}
  /// End time of the current window, -1 when disarmed.
  int64_t next_due_ns() const {return armed() ? next_due_ns_ : -1;}
  /// A record was delivered at tag time `now_ns`: the window restarts there.
  void on_record(int64_t now_ns);
  /// Number of windows that ended at or before `now_ns` with no record, each
  /// one deadline event; the window advances past `now_ns`.
  uint32_t take_elapsed(int64_t now_ns);

private:
  int64_t period_ns_ = 0;
  int64_t ref_ns_ = 0;
  int64_t next_due_ns_ = -1;
};

/// The wake the drain returns: the earlier of the correspondence's record wake
/// and the silence wake, each -1 for none.
int64_t combine_wake(int64_t record_wake, int64_t silence_wake);

/// Relative wake from `now_ns` to the window's end, or -1 when the window is
/// disarmed or ends past the stop tag (`stop_ns` < 0: no LF program, no stop).
/// An end past the stop tag is not scheduled: reactor-c would refuse it and
/// the refusal would terminalize the carrier (ros2_lf_corr_schedule_refused).
int64_t silence_wake(const Window & w, int64_t now_ns, int64_t stop_ns);

/// A QoS deadline in ns, or -1 for none: the rmw default (0 s, 0 ns, "no
/// deadline"), RMW_DURATION_INFINITE, and anything at or above 2^62 ns (146
/// years, kNoDeadlineNs), which no run reaches and whose sum with a tag time
/// would overflow int64.
int64_t deadline_ns(const rmw_time_t & d);
constexpr int64_t kNoDeadlineNs = int64_t{1} << 62;

/// The tag lf_schedule gives a wake of `w` ns (w >= 0) at the tag `now`:
/// (now + w, 0), or the next microstep for w = 0.
std::pair<int64_t, uint32_t> wake_tag(std::pair<int64_t, uint32_t> now, int64_t w);

/// Process state of one drain edge. Drains of one edge never run
/// concurrently (one LF reaction), so a Slot is used without a lock; slot()
/// itself is safe from concurrent drains of different edges.
struct Slot
{
  bool decided = false;   ///< the subscription was inspected (armed or not)
  bool started = false;   ///< a drain of this edge already ran
  Window window;
  /// Tag times of silence wakes this edge returned and has not drained yet: a
  /// drain at one of them with no record is a silence tick, not a dispatch.
  std::set<int64_t> ticks;
  /// Tags (time, microstep) of the drain events queued on this edge's drain
  /// action (claim_wake). Two chains schedule that one action, the records'
  /// wake and the window's, and reactor-c refuses a second event at a tag
  /// that already holds one: lf_schedule returns 0, the generated code calls
  /// ros2_lf_corr_schedule_refused, and the carrier is terminalized.
  std::set<std::pair<int64_t, uint32_t>> queued;
};
Slot & slot(uint32_t edge);

/// A reaction at `now` is about to schedule the wake `w` (ns, -1 for none) on
/// the drain action of `s`'s edge. Returns -1 when an event is already queued
/// at the wake's tag (the drain it runs computes every later wake), else `w`,
/// with its tag noted as queued.
int64_t claim_wake(Slot & s, std::pair<int64_t, uint32_t> now, int64_t w);

/// A drain of `s`'s edge runs at `now`: every queued event at or before
/// `now` was spent (a refused one never ran and is forgotten here too).
void drained(Slot & s, std::pair<int64_t, uint32_t> now);

/// Tests: forget every slot.
void reset_for_test();

}  // namespace silence_deadline
}  // namespace ros2_lf_binding

#endif  // SILENCE_DEADLINE_HPP_

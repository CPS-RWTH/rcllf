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

/// @file test_silence_deadline.cpp
/// @brief The logical deadline window of a silent periodic edge.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "rmw/types.h"
#include "silence_deadline.hpp"

namespace sd = ros2_lf_binding::silence_deadline;

namespace
{

constexpr int64_t kMs = 1000000;
constexpr int64_t kD = 25 * kMs;  // brake_fusion's /wheel/speed deadline

TEST(SilenceWindow, ASilentEdgeEndsOneWindowPerDeadlineFromTheStartTag)
{
  sd::Window w;
  w.arm(kD, 0);
  ASSERT_TRUE(w.armed());
  EXPECT_EQ(w.next_due_ns(), kD);
  EXPECT_EQ(w.take_elapsed(kD - 1), 0u) << "a window ended before t0 + D";
  EXPECT_EQ(w.take_elapsed(kD), 1u) << "no deadline event at t0 + D";
  EXPECT_EQ(w.ref_ns(), kD);
  EXPECT_EQ(w.next_due_ns(), 2 * kD);
  // A drain that runs late (at 3 D + 1) reports both windows it missed.
  EXPECT_EQ(w.take_elapsed(3 * kD + 1), 2u);
  EXPECT_EQ(w.next_due_ns(), 4 * kD);
}

TEST(SilenceWindow, ARecordRestartsTheWindowSoOneLossIsReportedOnce)
{
  // Period 10 ms, D 25 ms. Records at 10, 20 (missing: its own deadline call),
  // 30: the window restarts at each, so none ends until 30 + D.
  sd::Window w;
  w.arm(kD, 0);
  for (int64_t t : {10 * kMs, 20 * kMs, 30 * kMs}) {
    EXPECT_EQ(w.take_elapsed(t), 0u) << "a window ended at " << t;
    w.on_record(t);
  }
  EXPECT_EQ(w.take_elapsed(30 * kMs + kD - 1), 0u) << "the loss was reported twice";
  EXPECT_EQ(w.take_elapsed(30 * kMs + kD), 1u)
    << "no deadline event at t_last + D after the producer stopped";
}

TEST(SilenceWindow, ADisarmedWindowNeverWakes)
{
  sd::Window w;
  EXPECT_FALSE(w.armed());
  EXPECT_EQ(w.next_due_ns(), -1);
  EXPECT_EQ(w.take_elapsed(std::numeric_limits<int64_t>::max()), 0u);
  w.on_record(5);
  EXPECT_EQ(sd::silence_wake(w, 0, -1), -1);
  w.arm(0, 0);
  EXPECT_FALSE(w.armed()) << "a zero deadline armed the window";
  w.arm(-kD, 0);
  EXPECT_FALSE(w.armed()) << "a negative deadline armed the window";
}

TEST(SilenceWindow, TheWakeIsRelativeAndStopsAtTheStopTag)
{
  sd::Window w;
  w.arm(kD, 0);
  EXPECT_EQ(sd::silence_wake(w, 10 * kMs, -1), kD - 10 * kMs);
  EXPECT_EQ(sd::silence_wake(w, kD, -1), 0) << "an ended window must wake now";
  // The stop tag (T, m) admits (T, 0): a window ending at T still runs.
  EXPECT_EQ(sd::silence_wake(w, 0, kD), kD);
  EXPECT_EQ(sd::silence_wake(w, 0, kD - 1), -1)
    << "a wake past the stop tag would be refused and terminalize the carrier";
}

TEST(SilenceWindow, TheEarlierWakeWinsAndMinusOneMeansNone)
{
  EXPECT_EQ(sd::combine_wake(-1, -1), -1);
  EXPECT_EQ(sd::combine_wake(-1, 7), 7);
  EXPECT_EQ(sd::combine_wake(7, -1), 7);
  EXPECT_EQ(sd::combine_wake(0, 7), 0);
  EXPECT_EQ(sd::combine_wake(9, 7), 7);
}

TEST(SilenceWindow, OnlyAFiniteDeclaredDeadlineArms)
{
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{0, 0}), -1) << "the rmw default means no deadline";
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{0, 25000000}), kD);
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{1, 5}), 1000000005);
  // RMW_DURATION_INFINITE.
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{9223372036LL, 854775807LL}), -1);
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{4611686018LL, 427387904LL}), -1) << "2^62 ns";
  EXPECT_EQ(sd::deadline_ns(rmw_time_t{4611686018LL, 427387903LL}), sd::kNoDeadlineNs - 1);
}

TEST(SilenceWindow, AWindowNearTheEndOfTimeNeverOverflows)
{
  sd::Window w;
  w.arm(sd::kNoDeadlineNs - 1, std::numeric_limits<int64_t>::max() - 10);
  EXPECT_EQ(sd::silence_wake(w, 0, -1), -1) << "an end past int64 must never wake";
  EXPECT_EQ(w.take_elapsed(std::numeric_limits<int64_t>::max()), 0u);
}

TEST(SilenceWindow, SlotsAreKeptPerEdgeUntilReset)
{
  sd::reset_for_test();
  sd::slot(3).window.arm(kD, 0);
  EXPECT_TRUE(sd::slot(3).window.armed());
  EXPECT_FALSE(sd::slot(4).window.armed());
  sd::reset_for_test();
  EXPECT_FALSE(sd::slot(3).window.armed());
}

TEST(SilenceWindow, AWakeAtAnAlreadyQueuedDrainTagIsScheduledOnce)
{
  // Publishes every 10 ms, the one at 20 ms lost: the drain at (20 ms, 1)
  // queues its wake at 20 + D = 45 ms; the stale window wake at (25 ms, 0)
  // computes the same tag. reactor-c refuses a second event at a queued tag and the
  // carrier is terminalized, so the second claim must yield -1.
  sd::Slot s;
  EXPECT_EQ(sd::claim_wake(s, {20 * kMs, 1}, kD), kD);
  EXPECT_EQ(sd::claim_wake(s, {25 * kMs, 0}, 20 * kMs), -1)
    << "a second drain event at (45 ms, 0) was scheduled";
  // w = 0 is the next microstep, a tag of its own; -1 (no wake) passes through.
  EXPECT_EQ(sd::claim_wake(s, {25 * kMs, 0}, 0), 0);
  EXPECT_EQ(sd::claim_wake(s, {25 * kMs, 0}, 0), -1);
  EXPECT_EQ(sd::claim_wake(s, {25 * kMs, 0}, -1), -1);
  // The drain at (45 ms, 0) spent every event up to it: the tag is free again
  // for the next run of the same edge, and later tags stay claimed.
  EXPECT_EQ(sd::claim_wake(s, {30 * kMs, 1}, kD), kD);  // (55 ms, 0)
  sd::drained(s, {45 * kMs, 0});
  EXPECT_EQ(sd::claim_wake(s, {20 * kMs, 1}, kD), kD);
  EXPECT_EQ(sd::claim_wake(s, {45 * kMs, 0}, 10 * kMs), -1);
}

}  // namespace

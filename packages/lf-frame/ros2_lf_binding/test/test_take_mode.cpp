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

/// @file test_take_mode.cpp
/// @brief Tests for take-mode parsing, the optional env accessors, and
///        skip/block take behavior.

#include <gtest/gtest.h>
#include <unistd.h>

#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <iterator>
#include <chrono>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <new>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../src/corr_runtime.hpp"
#include "../src/handle_table.hpp"
#include "../src/take_mode.hpp"
#ifdef ROS2_LF_TAKE_BY_TAG
#include "../src/tag_log.hpp"
#endif
#include "../src/thread_role.hpp"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rosgraph_msgs/msg/clock.hpp"

using ros2_lf_binding::parse_take_mode;
using ros2_lf_binding::parse_take_timeout_ms;
using ros2_lf_binding::take_mode_env;
using ros2_lf_binding::take_stats_log_summary;
using ros2_lf_binding::take_stats_reset;
using ros2_lf_binding::take_stats_snapshot;
using ros2_lf_binding::take_timeout_ms_env;
using ros2_lf_binding::take_with_mode;
using ros2_lf_binding::TakeMode;

// ── Address reuse on demand ─────────────────────────────────────
//
// glibc hands a freed subscription's address to a later allocation only by
// chance (the block is too large for its per-thread cache), which is how a
// full-suite run once gave a new subscription an old one's address. This
// replacement operator new/delete makes that reuse deterministic: it records
// the blocks the test thread allocates while `g_reuse.record` is set, keeps
// the block named by `g_reuse.watch` instead of freeing it, and returns the
// kept block to the next allocation of the same size on the test thread
// while `g_reuse.hand_back` is set.
namespace
{
struct ReuseHook
{
  static constexpr size_t kMax = 4096;
  void * ptr[kMax];
  size_t size[kMax];
  size_t count = 0;
  std::atomic<void *> watch{nullptr};
  std::atomic<void *> kept{nullptr};
  size_t kept_size = 0;
};
ReuseHook g_reuse;
thread_local bool t_record = false;
thread_local bool t_hand_back = false;
}  // namespace

void * operator new(size_t n)
{
  if (t_hand_back && n == g_reuse.kept_size) {
    if (void * p = g_reuse.kept.exchange(nullptr)) {
      return p;
    }
  }
  void * p = std::malloc(n > 0 ? n : 1);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  if (t_record && g_reuse.count < ReuseHook::kMax) {
    g_reuse.ptr[g_reuse.count] = p;
    g_reuse.size[g_reuse.count] = n;
    ++g_reuse.count;
  }
  return p;
}

void operator delete(void * p) noexcept
{
  void * expected = p;
  if (p != nullptr && g_reuse.watch.compare_exchange_strong(expected, nullptr)) {
    g_reuse.kept.store(p);
    return;
  }
  std::free(p);
}

void operator delete(void * p, size_t) noexcept {operator delete(p);}

// ── Pure parser tests ────────────────────────────────────────────────────────

TEST(TakeModeParse, DefaultsToSkip)
{
  EXPECT_EQ(parse_take_mode(nullptr), TakeMode::Skip);
  EXPECT_EQ(parse_take_mode(""), TakeMode::Skip);
  EXPECT_EQ(parse_take_mode("skip"), TakeMode::Skip);
}

TEST(TakeModeParse, Block)
{
  EXPECT_EQ(parse_take_mode("block"), TakeMode::Block);
}

TEST(TakeModeParse, UnknownFallsBackToSkip)
{
  EXPECT_EQ(parse_take_mode("bogus"), TakeMode::Skip);
  EXPECT_EQ(parse_take_mode("BLOCK"), TakeMode::Skip);
}

TEST(TakeTimeoutParse, DefaultsTo100)
{
  EXPECT_EQ(parse_take_timeout_ms(nullptr), 100);
  EXPECT_EQ(parse_take_timeout_ms(""), 100);
}

TEST(TakeTimeoutParse, ParsesValue)
{
  EXPECT_EQ(parse_take_timeout_ms("0"), 0);
  EXPECT_EQ(parse_take_timeout_ms("250"), 250);
}

TEST(TakeTimeoutParse, RejectsGarbageAndNegative)
{
  EXPECT_EQ(parse_take_timeout_ms("abc"), 100);
  EXPECT_EQ(parse_take_timeout_ms("50x"), 100);
  EXPECT_EQ(parse_take_timeout_ms("-5"), 100);
}

// ── Optional env accessors ───────────────────────────────────────────────────
//
// The accessors must be re-readable across gtest cases (they are not
// permanently cached, unlike the old function-local `static const`), and must
// distinguish "unset" from "set to the built-in default value".

class TakeModeEnvTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // The dev container sets both vars (docker-compose.yml x-common-env), so
    // every case starts from a known state rather than assuming they are unset.
    unsetenv("ROS2_LF_TAKE_MODE");
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_MAXWAIT_MS");
  }

  void TearDown() override
  {
    unsetenv("ROS2_LF_TAKE_MODE");
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_MAXWAIT_MS");
  }
};

TEST_F(TakeModeEnvTest, UnsetIsNullopt)
{
  EXPECT_FALSE(take_mode_env().has_value());
  EXPECT_FALSE(take_timeout_ms_env().has_value());
}

TEST_F(TakeModeEnvTest, EmptyIsNullopt)
{
  setenv("ROS2_LF_TAKE_MODE", "", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "", 1);
  EXPECT_FALSE(take_mode_env().has_value());
  EXPECT_FALSE(take_timeout_ms_env().has_value());
}

TEST_F(TakeModeEnvTest, SetToDefaultValueIsStillEngaged)
{
  // "skip"/100 are the built-in defaults; an explicitly-set var must still be
  // reported as set, or a per-key override could never force the default.
  setenv("ROS2_LF_TAKE_MODE", "skip", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "100", 1);
  const auto mode = take_mode_env();
  ASSERT_TRUE(mode.has_value());
  EXPECT_EQ(*mode, TakeMode::Skip);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(*timeout, 100);
}

TEST_F(TakeModeEnvTest, RereadableWithinAndAcrossCases)
{
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  ASSERT_TRUE(take_mode_env().has_value());
  EXPECT_EQ(*take_mode_env(), TakeMode::Block);

  setenv("ROS2_LF_TAKE_MODE", "skip", 1);
  ASSERT_TRUE(take_mode_env().has_value());
  EXPECT_EQ(*take_mode_env(), TakeMode::Skip);

  unsetenv("ROS2_LF_TAKE_MODE");
  EXPECT_FALSE(take_mode_env().has_value());
}

TEST_F(TakeModeEnvTest, KeysAreIndependent)
{
  // Setting only the mode must leave the timeout accessor disengaged, so the
  // per-subscription timeout survives (design §2.2 per-key semantics).
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  EXPECT_TRUE(take_mode_env().has_value());
  EXPECT_FALSE(take_timeout_ms_env().has_value());

  unsetenv("ROS2_LF_TAKE_MODE");
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "250", 1);
  EXPECT_FALSE(take_mode_env().has_value());
  ASSERT_TRUE(take_timeout_ms_env().has_value());
  EXPECT_EQ(*take_timeout_ms_env(), 250);
}

TEST_F(TakeModeEnvTest, GarbageFallsBackToDefaultButStaysEngaged)
{
  setenv("ROS2_LF_TAKE_MODE", "bogus", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "abc", 1);
  const auto mode = take_mode_env();
  ASSERT_TRUE(mode.has_value());
  EXPECT_EQ(*mode, TakeMode::Skip);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(*timeout, 100);
}

// ROS2_LF_MAXWAIT_MS is the variable, ROS2_LF_TAKE_TIMEOUT_MS its
// alias; two that disagree are refused (nullopt: the resolved value stands).
TEST_F(TakeModeEnvTest, MaxwaitVarIsRead)
{
  setenv("ROS2_LF_MAXWAIT_MS", "250", 1);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value()) << "ROS2_LF_MAXWAIT_MS=250 was not read";
  EXPECT_EQ(*timeout, 250);
}

TEST_F(TakeModeEnvTest, TakeTimeoutVarStaysAnAlias)
{
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "250", 1);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(*timeout, 250);
}

TEST_F(TakeModeEnvTest, MaxwaitAndAliasWithTheSameValueAreRead)
{
  setenv("ROS2_LF_MAXWAIT_MS", "40", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "40", 1);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(*timeout, 40);
}

TEST_F(TakeModeEnvTest, MaxwaitAndAliasThatDisagreeAreRefused)
{
  setenv("ROS2_LF_MAXWAIT_MS", "200", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "100", 1);
  const auto timeout = take_timeout_ms_env();
  EXPECT_FALSE(timeout.has_value())
    << "ROS2_LF_MAXWAIT_MS=200 and ROS2_LF_TAKE_TIMEOUT_MS=100 gave " << timeout.value_or(-1);
  // An empty variable counts as unset, so the other one alone applies.
  setenv("ROS2_LF_MAXWAIT_MS", "", 1);
  ASSERT_TRUE(take_timeout_ms_env().has_value());
  EXPECT_EQ(*take_timeout_ms_env(), 100);
}

TEST_F(TakeModeEnvTest, GarbageMaxwaitFallsBackToDefaultButStaysEngaged)
{
  setenv("ROS2_LF_MAXWAIT_MS", "abc", 1);
  const auto timeout = take_timeout_ms_env();
  ASSERT_TRUE(timeout.has_value());
  EXPECT_EQ(*timeout, 100);
}

// ── Behavioral tests with a real pub/sub pair ────────────────────────────────

class TakeWithModeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    take_stats_reset();
    node_ = rclcpp::Node::make_shared("take_mode_test_node");
  }

  void TearDown() override {node_.reset();}

  // Create a matched pub/sub pair on `topic`; fail the test on match timeout.
  void make_matched_pair(const std::string & topic)
  {
    pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, 10);
    sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic, 10, [](rosgraph_msgs::msg::Clock::SharedPtr) {});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (pub_->get_subscription_count() > 0 && sub_->get_publisher_count() > 0) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "pub/sub on " << topic << " did not match within 5s";
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
};

TEST_F(TakeWithModeTest, SkipModeMissCountsSkip)
{
  make_matched_pair("/take_mode_test/skip_miss");
  rclcpp::MessageInfo info;
  auto msg = sub_->create_message();
  const bool taken = take_with_mode(
    *sub_, msg.get(), info, TakeMode::Skip, 100, "/take_mode_test/skip_miss");
  EXPECT_FALSE(taken);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.first_try, 0u);
  EXPECT_EQ(snap.topics.at("/take_mode_test/skip_miss").skipped, 1u);
}

TEST_F(TakeWithModeTest, FirstTrySuccessAfterDelivery)
{
  make_matched_pair("/take_mode_test/first_try");
  pub_->publish(rosgraph_msgs::msg::Clock());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::MessageInfo info;
  auto msg = sub_->create_message();
  const bool taken = take_with_mode(
    *sub_, msg.get(), info, TakeMode::Skip, 100, "/take_mode_test/first_try");
  EXPECT_TRUE(taken);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.first_try, 1u);
  EXPECT_EQ(snap.skipped, 0u);
}

TEST_F(TakeWithModeTest, BlockModeTimesOutWhenNothingArrives)
{
  make_matched_pair("/take_mode_test/block_timeout");
  rclcpp::MessageInfo info;
  auto msg = sub_->create_message();
  const auto start = std::chrono::steady_clock::now();
  const bool taken = take_with_mode(
    *sub_, msg.get(), info, TakeMode::Block, 50, "/take_mode_test/block_timeout");
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start);
  EXPECT_FALSE(taken);
  EXPECT_GE(elapsed.count(), 50);
  EXPECT_LT(elapsed.count(), 5000);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.timeouts, 1u);
  EXPECT_EQ(snap.topics.at("/take_mode_test/block_timeout").timeouts, 1u);
}

TEST_F(TakeWithModeTest, BlockModeCatchesLatePublish)
{
  make_matched_pair("/take_mode_test/block_late");
  std::thread publisher([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      pub_->publish(rosgraph_msgs::msg::Clock());
    });
  rclcpp::MessageInfo info;
  auto msg = sub_->create_message();
  const bool taken = take_with_mode(
    *sub_, msg.get(), info, TakeMode::Block, 2000, "/take_mode_test/block_late");
  publisher.join();
  EXPECT_TRUE(taken);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.waited, 1u);
  EXPECT_GT(snap.wait_total_ns, 0u);
  EXPECT_EQ(snap.topics.at("/take_mode_test/block_late").waited, 1u);
}

// ── Probe P17e: skip-mode debt ───────────────────────────────────────────────
//
// A skip-mode miss owes the edge one sample. Without the debt rule the late
// sample is delivered at the next trigger and every later trigger delivers
// the previous tag's sample (P17d permanent slip). These cases also build on
// the binding before the fix (no ROS2_LF_TAKE_DEBT), where they fail at run
// time with the slip; the debt counters they read only exist with the fix.

class TakeDebtTest : public TakeWithModeTest
{
protected:
  void make_pair(
    const std::string & topic, size_t depth, bool best_effort = false,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    auto qos = rclcpp::QoS(depth);
    if (best_effort) {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
    sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic, qos, [](rosgraph_msgs::msg::Clock::SharedPtr) {}, options);
    topic_ = topic;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (pub_->get_subscription_count() > 0 && sub_->get_publisher_count() > 0) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "pub/sub on " << topic << " did not match within 5s";
  }

  void publish(std::initializer_list<int> ids) {send(pub_, ids);}

  void send(
    const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr & pub,
    std::initializer_list<int> ids)
  {
    for (int id : ids) {
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = id;
      pub->publish(m);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  /// Publish `ids` with nanosec 1: a reader filtering on `clock.nanosec = 0`
  /// never sees them, and the holes they leave in the writer's publication
  /// numbers are by design, not a loss.
  void publish_filtered(std::initializer_list<int> ids)
  {
    for (int id : ids) {
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = id;
      m.clock.nanosec = 1;
      pub_->publish(m);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  /// Publish IDs `from` .. `to` (both included) in one burst, then let them land.
  void publish_range(int from, int to)
  {
    for (int id = from; id <= to; ++id) {
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = id;
      pub_->publish(m);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr second_publisher()
  {
    return node_->create_publisher<rosgraph_msgs::msg::Clock>(topic_, rclcpp::QoS(10).reliable());
  }

  /// Wait until `n` publishers are matched to the subscription and listed in the graph.
  void wait_publishers(size_t n)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while ((sub_->get_publisher_count() != n || node_->count_publishers(topic_) != n) &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(sub_->get_publisher_count(), n);
    ASSERT_EQ(node_->count_publishers(topic_), n);
  }

  /// One take of a trigger that still owes `remaining` writes. Returns the ID
  /// delivered or -1 on a miss, and adds the reported drops to reported_.
  int take(int remaining)
  {
    rclcpp::MessageInfo info;
    rosgraph_msgs::msg::Clock m;
#ifdef ROS2_LF_TAKE_DEBT
    uint64_t reported = 0;
    const bool taken = take_with_mode(
      *sub_, &m, info, TakeMode::Skip, timeout_ms_, topic_, remaining, &reported);
    reported_ += reported;
#else
    (void)remaining;
    const bool taken = take_with_mode(*sub_, &m, info, TakeMode::Skip, 100, topic_);
#endif
    return taken ? m.clock.sec : -1;
  }

  std::string topic_;
  uint64_t reported_ = 0;
  int64_t timeout_ms_ = 100;  // the take timeout of take(); a miss polls this long
};

// ── Fast mode: a sample not in the reader yet is on its way ────
//
// Under `-f true` a federate runs its tag as soon as the RTI grants it, so a
// cross-federate DDS sample can arrive after its trigger (in one recorded
// wall-clock pilot run, camera tick 78 was one plant state old).
// The count rule then waits for the trigger's sample instead of declaring the
// P17e miss. Real time waits the same way (the tests under
// "Count link in real time" below); an unknown count never waits.

TEST_F(TakeDebtTest, FastModeWaitsForTheTriggersSample)
{
  make_pair("/take_mode_test/fast_wait", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  std::thread late([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = 7;
      pub_->publish(m);
    });
  const int got = take(1);
  late.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(got, 7)
    << "fast mode: the trigger's sample arrived 30 ms after the tag and the take skipped the "
    "reaction (P17e miss) instead of waiting for it";
  EXPECT_EQ(snap.skipped, 0u);
  EXPECT_EQ(snap.waited, 1u);
  EXPECT_GT(snap.wait_total_ns, 0u);
  EXPECT_EQ(reported_, 0u);
  // The next trigger delivers its own sample: nothing is one tag late.
  publish({8});
  EXPECT_EQ(take(1), 8);
  EXPECT_EQ(take_stats_snapshot().late_dropped, 0u);
}

TEST_F(TakeDebtTest, FastModeMissAfterTheTimeoutStillOwesTheSample)
{
  make_pair("/take_mode_test/fast_timeout", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(take(1), -1) << "nothing was published, yet a sample was delivered";
  const auto waited_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  EXPECT_GE(waited_ms, 100) << "fast mode: the miss was declared before the take timeout";
  // The owed sample comes late, with the next trigger's own: the debt rule
  // drops the late one as before.
  publish({1, 2});
  EXPECT_EQ(take(1), 2);
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(reported_, 1u);
}

TEST_F(TakeDebtTest, FastModeUnknownWriteCountDoesNotWait)
{
  make_pair("/take_mode_test/fast_unknown", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(take(0), -1);
  const auto waited_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_LT(waited_ms, 50) << "a stub trigger owes nothing and must not wait";
  EXPECT_EQ(take_stats_snapshot().skipped, 1u);
}

// After a miss, the late owed sample is in the reader
// and the trigger's own arrives 20 ms later. Fast mode must poll for the
// second before it delivers the lone one, then drop the late one.
TEST_F(TakeDebtTest, FastModeDebtPollsForTheOnTimeSample)
{
  make_pair("/take_mode_test/fast_debt_poll", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_EQ(take(1), -1);
  rosgraph_msgs::msg::Clock late;
  late.clock.sec = 1;
  pub_->publish(late);
  std::thread on_time([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = 2;
      pub_->publish(m);
    });
  const int got = take(1);
  on_time.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(got, 2)
    << "fast mode delivered the late owed sample alone while the trigger's own was 20 ms away";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(reported_, 1u);
  EXPECT_EQ(snap.poll_timeouts, 1u) << "only the miss ran to the timeout";
}

// Each fast-mode poll that runs to the take timeout is counted per
// topic with its time. A lone sample whose partner never comes is delivered
// after one timeout, and the next lone sample of the same debt does not wait.
TEST_F(TakeDebtTest, FastModePollTimeoutsAreCountedPerTopic)
{
  make_pair("/take_mode_test/fast_poll_timeouts", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_EQ(take(1), -1);
  publish({5});
  EXPECT_EQ(take(1), 5) << "a lone sample must still be delivered after the poll";
  publish({6});
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(take(1), 6);
  const auto third_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_LT(third_ms, 50) << "the lone poll ran again for the same debt";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.poll_timeouts, 2u);
  EXPECT_GE(snap.poll_timeout_ns, 200'000'000u);
  const auto & ts = snap.topics.at("/take_mode_test/fast_poll_timeouts");
  EXPECT_EQ(ts.poll_timeouts, 2u);
  EXPECT_EQ(ts.poll_timeout_ns, snap.poll_timeout_ns);
}

TEST_F(TakeDebtTest, MissThenCatchUpDropsTheLateSample)
{
  make_pair("/take_debt_test/catch_up", 10);
  publish({0});
  EXPECT_EQ(take(1), 0);
  EXPECT_EQ(take(1), -1) << "tag 1: the payload is late, the take must miss";
  publish({1, 2});  // 1 arrives late, 2 on time for tag 2
  EXPECT_EQ(take(1), 2) << "tag 2 delivered the late ID 1: permanent slip (P17d)";
  publish({3});
  EXPECT_EQ(take(1), 3) << "tag 3 delivered a stale ID: the slip persists";
  EXPECT_EQ(reported_, 1u) << "late ID 1 was not reported as dropped";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).skipped, 1u);
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

TEST_F(TakeDebtTest, BurstOfTwoWithOneLatePayload)
{
  make_pair("/take_debt_test/burst", 10);
  publish({0, 1});
  EXPECT_EQ(take(2), 0);
  EXPECT_EQ(take(1), 1);
  publish({2});  // 3 is late
  EXPECT_EQ(take(2), 2);
  EXPECT_EQ(take(1), -1) << "the second write of the burst has no payload yet";
  publish({3, 4, 5});  // 3 late, 4 and 5 on time
  EXPECT_EQ(take(2), 4) << "burst tag delivered the late ID 3: permanent slip (P17d)";
  EXPECT_EQ(take(1), 5) << "second write of the burst tag delivered a stale ID";
  publish({6, 7});
  EXPECT_EQ(take(2), 6) << "the burst after the catch-up is off by one";
  EXPECT_EQ(take(1), 7);
  EXPECT_EQ(reported_, 1u) << "late ID 3 was not reported as dropped";
}

TEST_F(TakeDebtTest, UpstreamOneTagAheadKeepsItsSample)
{
  make_pair("/take_debt_test/ahead", 10);
  publish({0});
  EXPECT_EQ(take(1), 0);
  EXPECT_EQ(take(1), -1);
  publish({1, 2, 3});  // 1 late, 2 on time, 3 already sent for the next tag
  EXPECT_EQ(take(1), 2) << "tag 2 delivered the late ID 1: permanent slip (P17d)";
  EXPECT_EQ(take(1), 3) << "the ahead sample of tag 3 was dropped or replaced";
  EXPECT_EQ(take(1), -1) << "a sample was left over: the edge is still slipped";
  EXPECT_EQ(reported_, 1u) << "only the late ID 1 may be reported";
}

// ── Probe P17f: safe by default ─────────────────────────────────────────────
//
// A trigger whose write count is unknown (remaining 0, the stub port counts
// callback runs, RCLLF_COUNT_PUBLISHES=0) may be an empty tick, so its miss
// must owe nothing. On 698eb8df each such miss added debt, and a later backlog
// lost an on-time sample to it.

TEST_F(TakeDebtTest, UnknownWriteCountMissOwesNothing)
{
  make_pair("/take_debt_test/unknown_count", 10);
  publish({0});
  EXPECT_EQ(take(0), 0);
  EXPECT_EQ(take(0), -1) << "empty tick";
  EXPECT_EQ(take(0), -1) << "empty tick";
  publish({1, 2});  // backlog: 1 on time, 2 already sent for the next tag
  EXPECT_EQ(take(0), 1) << "an empty tick's phantom debt dropped the on-time ID 1";
  EXPECT_EQ(take(0), 2) << "the ahead ID 2 was delivered a tag early or lost";
  EXPECT_EQ(reported_, 0u) << "a sample was reported lost although none was";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 0u);
  }
#endif
}

// Reader depth 2: the owed ID 1 is evicted before the next take. The count
// rule holds IDs 2 and 3 for a trigger owing one write plus one owed sample and
// drops the oldest, ID 2 (documented count-rule residual; the ideal would
// deliver ID 2).
TEST_F(TakeDebtTest, MissBeforeTheFirstSampleFallsBackToTheCount)
{
  // Reader depth 2: the owed ID 1 is evicted before the writer was ever seen.
  // The first sample seen is the writer's baseline, not a gap (a writer may
  // have published before this subscription matched), so
  // nothing proves ID 1 lost: the count drops the oldest held sample. This is
  // the documented residual, not the ideal (which would deliver ID 2).
  make_pair("/take_debt_test/first_trigger", 2);
  EXPECT_EQ(take(1), -1);
  publish({1, 2, 3});
  EXPECT_EQ(take(1), 3) << "the count rule did not drop the oldest held sample";
  EXPECT_EQ(reported_, 1u) << "the dropped sample was not reported";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// A writer that published IDs 1..100 before the
// subscription existed. Its first sample seen carries sequence number 101, which
// must not be read as 100 lost samples: the count rule ignores sequence numbers.
TEST_F(TakeDebtTest, LateJoinedWriterIsNoGap)
{
  const std::string topic = "/take_debt_test/late_join";
  const auto qos = rclcpp::QoS(10).reliable();
  pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
  for (int id = 1; id <= 100; ++id) {
    rosgraph_msgs::msg::Clock m;
    m.clock.sec = id;
    pub_->publish(m);
  }
  sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    topic, qos, [](rosgraph_msgs::msg::Clock::SharedPtr) {});
  topic_ = topic;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "pub/sub did not match";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(take(1), -1) << "a sample published before the subscription existed was taken";
  publish({101, 102});  // 101 late for tag 1, 102 on time for tag 2
  EXPECT_EQ(take(1), 102) << "tag 2 delivered the late ID 101: the first sample of a "
    "late-joined writer was read as a 100-sample gap";
  publish({103});
  EXPECT_EQ(take(1), 103) << "tag 3 delivered a stale ID";
  EXPECT_EQ(reported_, 1u) << "only the late ID 101 may be reported";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

TEST_F(TakeDebtTest, TwoWritersLateDropsAreNotProofOfLoss)
{
  // Writers A (IDs 1..) and B (IDs 101..) on one topic, triggers alternate.
  // B's payload comes late twice. The first late B sample is dropped; on
  // 698eb8df its sequence number was not recorded, so B's next late sample
  // looked like a gap, the debt was written off as lost and the late sample
  // was delivered at A's tag: the slip came back.
  make_pair("/take_debt_test/two_writers", 10);
  auto pub_b = node_->create_publisher<rosgraph_msgs::msg::Clock>(
    topic_, rclcpp::QoS(10).reliable());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub_->get_publisher_count() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(sub_->get_publisher_count(), 2u);
  auto send = [&](rclcpp::Publisher<rosgraph_msgs::msg::Clock> & p, int id) {
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = id;
      p.publish(m);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };
  send(*pub_, 1);
  EXPECT_EQ(take(1), 1);
  send(*pub_b, 101);
  EXPECT_EQ(take(1), 101);
  EXPECT_EQ(take(1), -1) << "B's 102 is late";
  send(*pub_b, 102);
  send(*pub_, 2);
  EXPECT_EQ(take(1), 2) << "A's tag delivered the late 102";
  EXPECT_EQ(take(1), -1) << "B's 103 is late";
  send(*pub_b, 103);
  send(*pub_, 3);
  EXPECT_EQ(take(1), 3) << "A's tag delivered the late 103: debt written off as lost";
  send(*pub_b, 104);
  EXPECT_EQ(take(1), 104) << "B's tag delivered a stale sample: the edge slipped";
  EXPECT_EQ(reported_, 2u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 2u);
  }
#endif
}

// A's miss opens a debt; B joins, and
// at B's next tag (two writes) B's first sample is lost in DDS (reader depth 2
// evicts it) while the second arrives with A's late sample. Two samples held for
// a trigger owing two writes are not extra, so A's late ID 2 is delivered and
// the edge stays a tag behind (documented count-rule residual); no late drop
// is reported. The evicted ID 202 is a reader overflow, and
// that one is reported (B's numbers jump from 201 to 203).
TEST_F(TakeDebtTest, OtherWritersGapDoesNotClearTheDebt)
{
  make_pair("/take_debt_test/other_gap", 2);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's ID 2 is late, the take must miss";
  auto pub_b = node_->create_publisher<rosgraph_msgs::msg::Clock>(
    topic_, rclcpp::QoS(2).reliable());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub_->get_publisher_count() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(sub_->get_publisher_count(), 2u);
  auto send = [&](rclcpp::Publisher<rosgraph_msgs::msg::Clock> & p, int id) {
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = id;
      p.publish(m);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };
  send(*pub_b, 201);
  EXPECT_EQ(take(1), 201) << "B's first tag did not get B's ID 201";
  send(*pub_b, 202);  // evicted by the next two: lost in transit
  send(*pub_, 2);     // A's late sample
  send(*pub_b, 203);
  ASSERT_EQ(sub_->get_publisher_count(), 2u);
  EXPECT_EQ(take(2), 2) << "the count rule holds two samples for two writes: none is extra";
  EXPECT_EQ(take(1), 203) << "B's second write did not get the queued sample";
  send(*pub_, 3);
  EXPECT_EQ(take(1), 3) << "A's next tag did not get A's ID 3";
#ifdef ROS2_LF_READER_GAP
  EXPECT_EQ(reported_, 1u) << "the reader replaced B's ID 202: one sample lost";
  EXPECT_EQ(take_stats_snapshot().topics.at(topic_).reader_lost, 1u);
#else
  EXPECT_EQ(reported_, 0u) << "a sample was reported";
#endif
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 0u);
  }
#endif
}

// B matches, its write misses the take
// and B disconnects before a sample of it is ever taken. At the next tags only A
// is matched; its lone on-time samples are delivered (count rule), never dropped
// as the sample B owed.
TEST_F(TakeDebtTest, DebtFromSeveralPublishersIsNotDroppedExactly)
{
  make_pair("/take_debt_test/departed_writer", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  auto pub_b = node_->create_publisher<rosgraph_msgs::msg::Clock>(
    topic_, rclcpp::QoS(10).reliable());
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub_->get_publisher_count() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(sub_->get_publisher_count(), 2u);
  EXPECT_EQ(take(1), -1) << "B's write never arrives, the take must miss";
  pub_b.reset();
  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub_->get_publisher_count() > 1 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(sub_->get_publisher_count(), 1u);
  publish({2});
  EXPECT_EQ(take(1), 2) << "A's on-time ID 2 was dropped as the sample B owed";
  publish({3});
  EXPECT_EQ(take(1), 3) << "A's on-time ID 3 was dropped: the slip persists";
  EXPECT_EQ(reported_, 0u) << "an on-time sample of A was reported as lost";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 0u);
  }
#endif
}

// Mode "handover A to B". A miss after A left and
// before any sample of B is taken: the count rule drops B's late sample once B's
// on-time one is held with it.
TEST_F(TakeDebtTest, HandoverMissIsNotOwedByTheDepartedWriter)
{
  make_pair("/take_debt_test/handover", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  pub_.reset();
  auto pub_b = second_publisher();
  wait_publishers(1);
  EXPECT_EQ(take(1), -1) << "B's write is late, the take must miss";
  send(pub_b, {201, 202});  // 201 late, 202 on time
  EXPECT_EQ(take(1), 202) << "B's late ID 201 was delivered a tag late: the miss stayed "
    "charged to the departed writer A, so the count rule never dropped it";
  send(pub_b, {203});
  EXPECT_EQ(take(1), 203) << "the edge stays a tag behind: A's leaked debt keeps B's samples "
    "queued";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// Mode "B joins while A is still matched". A owes a
// miss and sends nothing more; B joins while A stays matched and listed.
// d289f20f kept the debt on A, so B's samples queued a tag behind for good. The
// count rule drops the extra one.
TEST_F(TakeDebtTest, QuietOwingWriterDoesNotHoldBackANewWriter)
{
  make_pair("/take_debt_test/quiet_owing", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201, 202});  // one of them is extra at this tag
  EXPECT_EQ(take(1), 202) << "the oldest extra was not dropped";
  send(pub_b, {203});
  EXPECT_EQ(take(1), 203) << "B's samples are delivered a tag behind for good: the debt stays "
    "charged to A, which is still matched and listed but sends nothing";
  send(pub_b, {204});
  EXPECT_EQ(take(1), 204);
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// Mode "A departs owing, its sample in flight". A
// owes a miss, B joins, B's two samples reach the reader before A's late ID 2,
// then A leaves. The count rule drops the extra B sample; A's late ID 2, taken
// alone at the next tag, is delivered a tag late (WARN once) and the edge stays a
// tag behind (documented count-rule residual).
TEST_F(TakeDebtTest, InFlightSampleOfADepartedWriterFallsToTheCountRule)
{
  make_pair("/take_debt_test/in_flight", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201, 202});
  publish({2});  // A's late sample, queued behind B's
  pub_.reset();
  wait_publishers(1);
  EXPECT_EQ(take(1), 202) << "the oldest extra was not dropped";
  send(pub_b, {203});
  EXPECT_EQ(take(1), 2) << "the count rule settled the debt, so A's late ID 2 is the oldest "
    "sample and is delivered (count-rule residual)";
  send(pub_b, {204});
  EXPECT_EQ(take(1), 203) << "the edge is a tag behind under the count rule";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// A's miss opens a debt, B joins while A stays matched, and B's two samples
// settle it by the count. A then resumes with an on-time sample, which must
// not be dropped as late: the count rule owes nothing once paid.
TEST_F(TakeDebtTest, SettledReattributedDebtDoesNotDropTheOverturnedWritersNextSample)
{
  make_pair("/take_debt_test/settled_moved", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201, 202});
  EXPECT_EQ(take(1), 202) << "the count rule did not drop the extra";
  EXPECT_EQ(reported_, 1u);
  publish({3});  // A resumes on time
  EXPECT_EQ(take(1), 3) << "A's on-time ID 3 was dropped as late and its take missed: the "
    "overturned writer kept its link to debt the count rule had already paid";
  EXPECT_EQ(reported_, 1u) << "A's on-time ID 3 was reported as lost";
  // No cascade: one sample per writer at the next tag.
  publish({4});
  send(pub_b, {203});
  EXPECT_EQ(take(1), 4) << "A's on-time ID 4 was dropped: the missed take opened a new debt";
  EXPECT_EQ(take(1), 203) << "the next tag did not take normally";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).skipped, 1u) << "a take after the settled debt missed";
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// While the debt is still open (B's lone
// sample could not settle it under the count rule), A's late sample in flight is
// the oldest of the two held at the next take and the count rule drops it.
TEST_F(TakeDebtTest, InFlightSampleIsDroppedWhileTheReattributedDebtIsOpen)
{
  make_pair("/take_debt_test/moved_open", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201});
  EXPECT_EQ(take(1), 201) << "a lone sample was dropped on an open debt";
  EXPECT_EQ(reported_, 0u);
  publish({2});  // A's late sample
  send(pub_b, {202});
  EXPECT_EQ(take(1), 202) << "A's late ID 2 was delivered although the debt was open";
  send(pub_b, {203});
  EXPECT_EQ(take(1), 203) << "the edge slipped a tag";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// After B's samples paid the debt, A's on-time ID 3 and B's ID 203 for
// the next tag are both queued. Reading one extra sample for the ambiguous ID 3
// would start a cascade of new debt. Under the count rule nothing is owed any
// more: ID 3, then 203, then 4.
TEST_F(TakeDebtTest, PaidOverturnedDebtDropsNoOnTimeSampleWithTheNextTagQueued)
{
  make_pair("/take_debt_test/sol2_cascade", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201, 202});
  EXPECT_EQ(take(1), 202) << "the count rule did not drop the extra";
  publish({3});      // A, on time
  send(pub_b, {203});  // B, for the next tag, already queued
  EXPECT_EQ(take(1), 3) << "A's on-time ID 3 was dropped as the ambiguous extra and B's 203 "
    "delivered a tag early";
  EXPECT_EQ(take(1), 203) << "the next take missed and opened new debt (cascade)";
  publish({4});
  EXPECT_EQ(take(1), 4) << "the new debt dropped A's on-time ID 4 (cascade)";
  EXPECT_EQ(reported_, 1u) << "an on-time sample was reported as lost";
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).skipped, 1u) << "a take after the paid debt missed";
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// A owes two misses; B's 201 and 202 settle one and the count drops 201.
// A's ID 2 is lost (filtered out: a gap in A's sequence) and its late ID 3
// arrives alone: the count rule delivers a lone sample (one tag late, WARN once),
// and the remaining debt unit drops the oldest extra at the next take that holds
// two. The gap writes nothing off.
TEST_F(TakeDebtTest, LostSampleAfterAnOverturnLeavesTheDebtToTheCountRule)
{
  const std::string topic = "/take_debt_test/sol1_gap";
  const auto qos = rclcpp::QoS(10).reliable();
  rclcpp::SubscriptionOptions options;
  options.content_filter_options.filter_expression = "clock.nanosec = 0";
  pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
  sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    topic, qos, [](rosgraph_msgs::msg::Clock::SharedPtr) {}, options);
  ASSERT_TRUE(sub_->is_cft_enabled()) << "the rmw has no content filter to inject a loss";
  topic_ = topic;
  wait_publishers(1);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  EXPECT_EQ(take(1), -1) << "A's next write is late too, the take must miss";
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {201, 202});
  EXPECT_EQ(take(1), 202) << "the count rule did not drop the oldest extra";
  {
    rosgraph_msgs::msg::Clock lost;
    lost.clock.sec = 2;
    lost.clock.nanosec = 1;  // filtered out: A's ID 2 never reaches the reader
    pub_->publish(lost);
  }
  publish({3});  // A's late ID 3, alone
  EXPECT_EQ(take(1), 3) << "the count rule delivers a lone sample while debt is open";
  publish({4});
  send(pub_b, {203});
  EXPECT_EQ(take(1), 203) << "the open debt unit did not drop the oldest extra";
  send(pub_b, {204});
  EXPECT_EQ(take(1), 204) << "the next tag did not take normally";
  EXPECT_EQ(reported_, 2u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 2u);
  }
#endif
}

// Mode "A departs owing, its sample lost". A owes a
// miss and leaves without sending; B takes over. The debt is not written off
// (graph absence is no proof): B's lone samples are delivered (WARN once) and the
// first take that holds two drops the extra one, which ends the debt: no
// permanent leak, at the cost of one reported sample that may have been on time
// (documented count-rule residual).
TEST_F(TakeDebtTest, DebtOfADepartedWriterIsSettledByTheCount)
{
  make_pair("/take_debt_test/departed_owing", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  pub_.reset();
  auto pub_b = second_publisher();
  wait_publishers(1);
  send(pub_b, {201});
  EXPECT_EQ(take(1), 201) << "a lone sample was dropped by the open debt";
  EXPECT_EQ(reported_, 0u);
  send(pub_b, {202, 203});
  EXPECT_EQ(take(1), 203) << "the debt A left was never settled by the count rule";
  send(pub_b, {204});
  EXPECT_EQ(take(1), 204) << "debt is still open after the count rule settled it";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

// A and B were both seen and B left. A's lone late
// sample is delivered a tag late and the count rule drops the next extra
// (documented count-rule residual).
TEST_F(TakeDebtTest, StaleWriterEntryFallsBackToTheCount)
{
  make_pair("/take_debt_test/stale_writer", 10);
  publish({1});
  EXPECT_EQ(take(1), 1);
  auto pub_b = second_publisher();
  wait_publishers(2);
  send(pub_b, {101});
  EXPECT_EQ(take(1), 101);
  pub_b.reset();
  wait_publishers(1);
  EXPECT_EQ(take(1), -1) << "A's write is late, the take must miss";
  publish({2});
  EXPECT_EQ(take(1), 2) << "a lone sample was dropped on an open debt";
  publish({3, 4});
  EXPECT_EQ(take(1), 4) << "the count rule did not drop the extra";
  EXPECT_EQ(reported_, 1u);
#ifdef ROS2_LF_TAKE_DEBT
  {
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  }
#endif
}

#ifdef ROS2_LF_TAKE_DEBT
// ── Differential test against the plain count rule
//
// Seeded schedules of 1 to 3 writers with silent joins, departures, losses
// (filtered out by a content filter, so the writer's sequence shows a gap),
// ahead, on-time and late samples, handovers and in-flight samples of departed
// writers. Each tag's trigger counts one write, and the sample of tag t has ID
// t. A sample reaches the reader before the take of its slot; it is on time by
// construction when its slot is not after its tag. Each schedule is fed to the
// real take_with_mode and to CountRule, a reference model of the plain count
// rule on the same arrival order. The schedule ends with kTail on-time tags of
// one writer and a drain. The take path is the count rule plus reporting, so
// the two must agree on every schedule: the same sample (or miss) at every
// take and in the drain, the same on-time samples dropped, the same lasting
// slip, and one report per dropped sample.

namespace
{
struct FuzzSample
{
  int tag;
  int writer;
  int slot;
  bool lost;
};

struct FuzzWriter
{
  int join = 0;     // slot whose takes it precedes when created
  int depart = -1;  // slot after whose publications it is destroyed; -1 never
};

struct FuzzSchedule
{
  int tags = 0;
  std::vector<FuzzWriter> writers;
  std::vector<FuzzSample> samples;  // publication order
};

constexpr int kTail = 4;

FuzzSchedule make_schedule(uint32_t seed)
{
  std::mt19937 rng(seed);
  auto pick = [&rng](uint32_t n) {return static_cast<int>(rng() % n);};
  FuzzSchedule s;
  const int nw = 1 + pick(3);
  const int main_tags = 6 + pick(7);
  s.tags = main_tags + kTail;
  s.writers.resize(nw);
  std::vector<int> owner(s.tags);
  int cur = 0;
  for (int t = 0; t < s.tags; ++t) {
    if (t > 0 && t < main_tags && pick(3) == 0) {
      cur = pick(nw);
    }
    owner[t] = cur;
  }
  std::vector<int> prev(nw, -1), first(nw, -1), last(nw, -1);
  std::vector<std::vector<std::deque<FuzzSample>>> per(
    s.tags + 3, std::vector<std::deque<FuzzSample>>(nw));
  for (int t = 0; t < s.tags; ++t) {
    const int w = owner[t];
    int slot = t;
    bool lost = false;
    if (t < main_tags) {
      const int r = pick(20);
      if (r < 3) {
        slot = t + 1;
      } else if (r < 5) {
        slot = t + 2;
      } else if (r < 7 && t > 0) {
        slot = t - 1;
      } else if (r < 9) {
        lost = true;
      }
    }
    slot = std::max(slot, prev[w]);  // one writer's samples arrive in order
    prev[w] = slot;
    if (first[w] < 0) {
      first[w] = slot;
    }
    last[w] = slot;
    per[slot][w].push_back({t, w, slot, lost});
  }
  // Within a slot, writers interleave at random, each in its own order.
  for (auto & slot : per) {
    for (;; ) {
      std::vector<int> ready;
      for (int w = 0; w < nw; ++w) {
        if (!slot[w].empty()) {
          ready.push_back(w);
        }
      }
      if (ready.empty()) {
        break;
      }
      const int w = ready[pick(ready.size())];
      s.samples.push_back(slot[w].front());
      slot[w].pop_front();
    }
  }
  for (int w = 0; w < nw; ++w) {
    FuzzWriter & fw = s.writers[w];
    if (first[w] < 0) {
      // A matched publisher that never sends.
      fw.join = pick(s.tags);
      fw.depart = pick(2) ? std::min(s.tags - 1, fw.join + pick(4)) : -1;
    } else {
      fw.join = std::max(0, first[w] - pick(3));
      if (w != owner[s.tags - 1] && pick(2)) {
        fw.depart = last[w] + pick(2);
      }
    }
  }
  return s;
}

/// The plain count rule: take up to remaining + debt samples, drop the oldest
/// extras, deliver a lone one (it may be a tag late), owe a known miss.
struct CountRule
{
  std::deque<int> queue;
  uint64_t debt = 0;
  std::vector<int> dropped;

  int take(bool known)
  {
    std::deque<int> held;
    while (held.size() < 1 + debt && !queue.empty()) {
      held.push_back(queue.front());
      queue.pop_front();
    }
    while (debt > 0 && held.size() > 1) {
      dropped.push_back(held.front());
      held.pop_front();
      --debt;
    }
    if (!held.empty()) {
      return held.front();
    }
    if (known && debt < 50) {
      ++debt;
    }
    return -1;
  }
};

struct FuzzOutcome
{
  int on_time_dropped = 0;
  bool slip = false;
};

FuzzOutcome judge(
  const FuzzSchedule & s, const std::vector<int> & last_take, const std::set<int> & delivered)
{
  FuzzOutcome o;
  for (const auto & x : s.samples) {
    if (!x.lost && x.slot <= x.tag && delivered.count(x.tag) == 0) {
      ++o.on_time_dropped;
    }
  }
  o.slip = last_take.back() != s.tags - 1;
  return o;
}
}  // namespace

TEST_F(TakeDebtTest, DifferentialAgainstTheCountRule)
{
  constexpr uint32_t kSchedules = 1000;
  // Every sample of a slot is settled before its takes, so a miss is final:
  // poll 1 ms instead of 100 ms per miss (thousands of misses per run).
  timeout_ms_ = 1;
  const char * log_path = std::getenv("TAKE_FUZZ_LOG");
  std::ofstream log;
  if (log_path != nullptr) {
    log.open(log_path);
  }
  int fix_a = 0, model_a = 0, fix_b = 0, model_b = 0;
  int drops = 0, losses = 0, writers3 = 0, departures = 0;
  std::vector<std::string> mismatches;
  for (uint32_t seed = 1; seed <= kSchedules; ++seed) {
    take_stats_reset();
    reported_ = 0;
    const FuzzSchedule s = make_schedule(seed);
    topic_ = "/take_debt_fuzz/s" + std::to_string(seed);
    rclcpp::SubscriptionOptions options;
    options.content_filter_options.filter_expression = "clock.nanosec = 0";
    sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic_, rclcpp::QoS(50).reliable(), [](rosgraph_msgs::msg::Clock::SharedPtr) {}, options);
    ASSERT_TRUE(sub_->is_cft_enabled()) << "the rmw has no content filter to inject a loss";
    const int nw = static_cast<int>(s.writers.size());
    writers3 += nw == 3;
    std::vector<rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr> pubs(nw);
    auto settle = [&]() {
        size_t live = 0;
        for (const auto & p : pubs) {
          live += p != nullptr;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        for (;; ) {
          bool ok = sub_->get_publisher_count() == live;
          for (const auto & p : pubs) {
            ok = ok && (p == nullptr || p->get_subscription_count() == 1);
          }
          if (ok) {
            return true;
          }
          if (std::chrono::steady_clock::now() > deadline) {
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      };
    CountRule model;
    std::vector<int> fix_takes, model_takes;
    std::set<int> fix_delivered, model_delivered;
    int arrived = 0;
    size_t next = 0;
    for (int slot = 0; slot < s.tags; ++slot) {
      bool changed = false;
      for (int w = 0; w < nw; ++w) {
        if (s.writers[w].join == slot) {
          pubs[w] = node_->create_publisher<rosgraph_msgs::msg::Clock>(
            topic_, rclcpp::QoS(50).reliable());
          changed = true;
        }
      }
      if (changed) {
        ASSERT_TRUE(settle()) << "seed " << seed << ": a join did not match";
      }
      for (; next < s.samples.size() && s.samples[next].slot == slot; ++next) {
        const auto & x = s.samples[next];
        ASSERT_NE(pubs[x.writer], nullptr) << "seed " << seed << ": schedule bug";
        rosgraph_msgs::msg::Clock m;
        m.clock.sec = x.tag;
        m.clock.nanosec = x.lost ? 1 : 0;
        pubs[x.writer]->publish(m);
        if (x.lost) {
          ++losses;
        } else {
          ++arrived;
          model.queue.push_back(x.tag);
        }
      }
      changed = false;
      for (int w = 0; w < nw; ++w) {
        if (s.writers[w].depart == slot) {
          pubs[w].reset();
          changed = true;
          ++departures;
        }
      }
      if (changed) {
        ASSERT_TRUE(settle()) << "seed " << seed << ": a departure did not unmatch";
      }
      fix_takes.push_back(take(1));
      model_takes.push_back(model.take(true));
    }
    ASSERT_EQ(next, s.samples.size()) << "seed " << seed << ": a sample arrives after the tail";
    for (int v : fix_takes) {
      fix_delivered.insert(v);
    }
    for (int v : model_takes) {
      model_delivered.insert(v);
    }
    std::vector<int> fix_drain, model_drain;
    for (int i = 0; i < 60; ++i) {
      const int v = take(0);
      if (v < 0) {
        break;
      }
      fix_drain.push_back(v);
      fix_delivered.insert(v);
    }
    for (int i = 0; i < 60; ++i) {
      const int v = model.take(false);
      if (v < 0) {
        break;
      }
      model_drain.push_back(v);
      model_delivered.insert(v);
    }
    fix_delivered.erase(-1);
    model_delivered.erase(-1);
    const auto snap = take_stats_snapshot();
    const auto it = snap.topics.find(topic_);
    const uint64_t late = it == snap.topics.end() ? 0 : it->second.late_dropped;
    drops += static_cast<int>(late);
    const FuzzOutcome f = judge(s, fix_takes, fix_delivered);
    const FuzzOutcome m = judge(s, model_takes, model_delivered);
    fix_a += f.on_time_dropped > 0;
    model_a += m.on_time_dropped > 0;
    fix_b += f.slip;
    model_b += m.slip;
    std::string why;
    for (const auto & x : s.samples) {
      if (!x.lost && x.slot <= x.tag && fix_delivered.count(x.tag) == 0 &&
        model_delivered.count(x.tag) == 1)
      {
        why += " the take path dropped the on-time sample ID " + std::to_string(x.tag) +
          " (writer " + std::to_string(x.writer) + ") that the count rule delivers;";
      }
    }
    for (size_t i = 0; i < fix_takes.size(); ++i) {
      if (fix_takes[i] != model_takes[i]) {
        why += " take " + std::to_string(i) + " delivered " + std::to_string(fix_takes[i]) +
          ", the count rule " + std::to_string(model_takes[i]) + ";";
        break;
      }
    }
    if (why.empty() && fix_drain != model_drain) {
      why += " the drain differs from the count rule;";
    }
    if (f.on_time_dropped != m.on_time_dropped) {
      why += " on-time drops " + std::to_string(f.on_time_dropped) + " vs " +
        std::to_string(m.on_time_dropped) + ";";
    }
    if (f.slip != m.slip) {
      why += std::string(" lasting slip ") + (f.slip ? "only in the take path;" : "only in the "
        "count rule;");
    }
    if (late != model.dropped.size() || reported_ != late) {
      why += " late_dropped=" + std::to_string(late) + " reported=" + std::to_string(reported_) +
        ", the count rule dropped " + std::to_string(model.dropped.size()) + ";";
    }
    size_t fix_count = fix_drain.size();
    for (int v : fix_takes) {
      fix_count += v >= 0;
    }
    if (fix_count != fix_delivered.size() || static_cast<uint64_t>(arrived) != fix_count + late) {
      why += " a sample was delivered twice or dropped without a report;";
    }
    if (!why.empty()) {
      mismatches.push_back("seed " + std::to_string(seed) + ":" + why);
    }
    if (log.is_open()) {
      log << "seed " << seed << " fix";
      for (int v : fix_takes) {
        log << ' ' << v;
      }
      log << " | drain";
      for (int v : fix_drain) {
        log << ' ' << v;
      }
      log << " | model";
      for (int v : model_takes) {
        log << ' ' << v;
      }
      log << " | late_dropped=" << late << " reported=" << reported_ << " on_time_dropped fix=" <<
        f.on_time_dropped << " model=" << m.on_time_dropped << " slip fix=" << f.slip <<
        " model=" << m.slip << " | samples";
      for (const auto & x : s.samples) {
        log << ' ' << x.tag << '/' << x.writer << '@' << x.slot << (x.lost ? "L" : "");
      }
      log << " | writers";
      for (const auto & w : s.writers) {
        log << ' ' << w.join << ':' << w.depart;
      }
      log << '\n';
    }
    sub_.reset();
    pubs.clear();
  }
  std::printf(
    "[differential] schedules=%u (3 writers: %d, departures: %d, lost samples: %d, late drops: "
    "%d)\n"
    "[differential] schedules with an on-time sample dropped: take path %d, count rule %d\n"
    "[differential] schedules ending with a lasting slip: take path %d, count rule %d\n"
    "[differential] schedules that differ from the count rule: %zu\n",
    kSchedules, writers3, departures, losses, drops, fix_a, model_a, fix_b, model_b,
    mismatches.size());
  std::string first;
  for (size_t i = 0; i < mismatches.size() && i < 5; ++i) {
    first += "\n  " + mismatches[i];
  }
  EXPECT_TRUE(mismatches.empty())
    << mismatches.size() << " schedule(s) differ from the count rule; first:" << first;
}
#endif

// The message_lost hook path of the debt rule, through the real
// subscription invoke. A late payload owed to an earlier trigger is dropped
// and handed to the subscription's message_lost handler, so a node that
// registered one sees the loss (report_owed_lost in reaction_dispatch.cpp).
TEST_F(TakeDebtTest, LateDropReachesTheMessageLostHandlerThroughInvokeSub)
{
  const std::string topic = "/take_debt_test/message_lost";
  const auto qos = rclcpp::QoS(10).reliable();
  std::vector<int> delivered;
  std::vector<std::pair<size_t, size_t>> lost;  // (total_count, total_count_change)
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.message_lost_callback = [&lost](rclcpp::QOSMessageLostInfo & info) {
      lost.emplace_back(info.total_count, info.total_count_change);
    };
  pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
  sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    topic, qos, [&delivered](rosgraph_msgs::msg::Clock::SharedPtr m) {
      delivered.push_back(m->clock.sec);
    }, options);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "pub/sub did not match";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ros2_lf_binding::g_handle_table.clear();
  ros2_lf_binding::g_handle_table.resize(1);
  auto & e = ros2_lf_binding::g_handle_table[0];
  e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  e.subscription = sub_;
  e.topic = topic;
  e.node_class = "TakeDebtNode";
  e.callback_name = "on_clock";
  e.take_mode = TakeMode::Skip;
  ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
  // One trigger of a reaction whose stub port counted one write.
  auto invoke = [&]() {
#ifdef ROS2_LF_TAKE_DEBT
      return ros2_lf_invoke_sub_n(nullptr, 0, topic.c_str(), 1);
#else
      return ros2_lf_invoke_sub(nullptr, 0, topic.c_str());
#endif
    };

  publish({0});
  EXPECT_EQ(invoke(), 1);
  EXPECT_EQ(invoke(), 0) << "tag 1: the payload is late, the take must miss";
  publish({1, 2});  // 1 arrives late, 2 on time for tag 2
  EXPECT_EQ(invoke(), 1);
  publish({3});
  EXPECT_EQ(invoke(), 1);
  ros2_lf_binding::g_handle_table.clear();

  EXPECT_EQ(delivered, (std::vector<int>{0, 2, 3}))
    << "the late ID 1 shifted every later payload by one tag (P17d slip)";
  ASSERT_EQ(lost.size(), 1u) << "message_lost handler did not receive the dropped late sample";
  EXPECT_EQ(lost[0], (std::pair<size_t, size_t>{1u, 1u}));
}

// Per-subscription state (the message_lost running total in
// reaction_dispatch.cpp, the take debt in take_mode.cpp) is keyed by the
// subscription's address. A subscription created at the address of a destroyed
// one must start clean: total 0, debt 0. The test destroys a subscription that
// reported one loss and still owes one sample, lets the reuse hook above hand
// its address to a new subscription, and runs the same schedule on it. `other`
// stays alive on the topic, unbound, so its queue never matters.
TEST_F(TakeDebtTest, SubscriptionAtAReusedAddressStartsWithZeroLossTotalAndDebt)
{
  const std::string topic = "/take_debt_test/reused_address";
  const auto qos = rclcpp::QoS(10).reliable();
  std::vector<int> delivered;
  std::vector<std::pair<size_t, size_t>> lost;  // (total_count, total_count_change)
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.message_lost_callback = [&lost](rclcpp::QOSMessageLostInfo & info) {
      lost.emplace_back(info.total_count, info.total_count_change);
    };
  auto make_sub = [&]() {
      return node_->create_subscription<rosgraph_msgs::msg::Clock>(
        topic, qos, [&delivered](rosgraph_msgs::msg::Clock::SharedPtr m) {
          delivered.push_back(m->clock.sec);
        }, options);
    };
  auto wait_match = [&]() {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "pub/sub did not match";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  auto bind = [&]() {
      ros2_lf_binding::g_handle_table.clear();
      ros2_lf_binding::g_handle_table.resize(1);
      auto & e = ros2_lf_binding::g_handle_table[0];
      e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
      e.subscription = sub_;
      e.topic = topic;
      e.node_class = "TakeDebtNode";
      e.callback_name = "on_clock";
      e.take_mode = TakeMode::Skip;
    };
  ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto invoke = [&]() {
#ifdef ROS2_LF_TAKE_DEBT
      return ros2_lf_invoke_sub_n(nullptr, 0, topic.c_str(), 1);
#else
      return ros2_lf_invoke_sub(nullptr, 0, topic.c_str());
#endif
    };

  // First subscription: one loss reported, then a miss that leaves one owed.
  pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
  g_reuse.count = 0;
  t_record = true;
  sub_ = make_sub();
  t_record = false;
  // The block make_shared allocated for the subscription holds its address.
  const char * const at = reinterpret_cast<const char *>(sub_.get());
  for (size_t i = 0; i < g_reuse.count; ++i) {
    const char * const b = static_cast<const char *>(g_reuse.ptr[i]);
    if (b <= at && at < b + g_reuse.size[i] &&
      g_reuse.size[i] > g_reuse.kept_size)
    {
      g_reuse.watch.store(g_reuse.ptr[i]);
      g_reuse.kept_size = g_reuse.size[i];
    }
  }
  ASSERT_NE(g_reuse.watch.load(), nullptr) << "no recorded block holds the subscription";
  wait_match();
  bind();
  publish({0});
  EXPECT_EQ(invoke(), 1);
  EXPECT_EQ(invoke(), 0);
  publish({1, 2});  // 1 is late and lost, 2 on time
  EXPECT_EQ(invoke(), 1);
  EXPECT_EQ(invoke(), 0) << "no sample: the trigger owes one";
  ASSERT_EQ(lost, (std::vector<std::pair<size_t, size_t>>{{1u, 1u}}));

  // Destroy it and get a new subscription at the same address.
  const void * const old_addr = sub_.get();
  std::weak_ptr<rclcpp::SubscriptionBase> old_weak = sub_;
  ros2_lf_binding::g_handle_table.clear();
  sub_.reset();
  ASSERT_TRUE(old_weak.expired()) << "the first subscription is still alive";
  old_weak.reset();
  // Weak references keep the block until their owners prune expired entries:
  // the callback group when the next subscription joins it, the binding's
  // per-subscription state when another subscription first uses it. A second
  // subscription runs the same schedule once, which does both.
  sub_ = make_sub();
  wait_match();
  bind();
  publish({20});
  EXPECT_EQ(invoke(), 1);
  EXPECT_EQ(invoke(), 0);
  publish({21, 22});
  EXPECT_EQ(invoke(), 1);
  const auto other = sub_;
  ros2_lf_binding::g_handle_table.clear();
  sub_.reset();
  ASSERT_NE(g_reuse.kept.load(), nullptr) << "the subscription's block was not freed";
  t_hand_back = true;
  sub_ = make_sub();
  t_hand_back = false;
  g_reuse.kept_size = 0;
  if (void * left = g_reuse.kept.exchange(nullptr)) {
    std::free(left);
  }
  ASSERT_EQ(static_cast<const void *>(sub_.get()), old_addr)
    << "the new subscription did not get the destroyed one's address";
  ASSERT_EQ(lost.size(), 2u) << "the second subscription did not report its loss";
  wait_match();
  bind();
  delivered.clear();
  lost.clear();

  // The new subscription owes nothing: two queued samples serve two triggers.
  // An inherited debt of one would drop 10 as late and leave the second
  // trigger without a sample.
  publish({10, 11});
  EXPECT_EQ(invoke(), 1);
  EXPECT_EQ(invoke(), 1) << "the new subscription inherited the old one's owed sample";
  EXPECT_EQ(delivered, (std::vector<int>{10, 11}))
    << "the new subscription inherited the old one's owed sample";
  // Then the first schedule again: one miss, one late sample lost.
  EXPECT_EQ(invoke(), 0);
  publish({12, 13});
  EXPECT_EQ(invoke(), 1);
  ros2_lf_binding::g_handle_table.clear();
  EXPECT_EQ(delivered, (std::vector<int>{10, 11, 13}));
  ASSERT_FALSE(lost.empty()) << "message_lost did not receive the dropped late sample";
  EXPECT_EQ(lost[0].first, 1u)
    << "the new subscription's message_lost total_count continued the old one's total";
  EXPECT_EQ(lost, (std::vector<std::pair<size_t, size_t>>{{1u, 1u}}));
}

// A decentralized ingress edge reports a tardy record
// through the drain (corr_runtime.cpp), never through this take path. A late
// take-mode sample is still dropped once and reported once, whether or not a
// tardy-drop edge exists in the process, and it carries no lost context.
class TakeLateWithTardyHook : public TakeDebtTest
{
protected:
  void run(TakeMode mode, bool tardy_hook)
  {
    const std::string topic = std::string("/take_debt_test/r3_") +
      (mode == TakeMode::Skip ? "skip" : "block") + (tardy_hook ? "_hook" : "_nohook");
    ros2_lf_binding::corr_rt::reset_for_test();
    // The compose env pins skip mode and 100 ms; the guard runs each mode
    // itself and restores the env for the tests after it.
    const char * env_mode = std::getenv("ROS2_LF_TAKE_MODE");
    const char * env_timeout = std::getenv("ROS2_LF_TAKE_TIMEOUT_MS");
    const std::string saved_mode = env_mode ? env_mode : "";
    const std::string saved_timeout = env_timeout ? env_timeout : "";
    setenv("ROS2_LF_TAKE_MODE", mode == TakeMode::Skip ? "skip" : "block", 1);
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    if (tardy_hook) {
      ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {}, topic + "_xfed", "Sub", 0, topic + "_xfed",
        5000000};
      s.role = ros2_lf_binding::CorrEdgeRole::kIngress;
      ros2_lf_binding::register_corr_edge(s);
      ros2_lf_binding::set_corr_edge_tardy_drop(0);
    }
    const auto qos = rclcpp::QoS(10).reliable();
    std::vector<int> delivered;
    int lost = 0;
    size_t lost_samples = 0;
    int with_context = 0;
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.message_lost_callback =
      [&lost, &lost_samples, &with_context](rclcpp::QOSMessageLostInfo & info) {
        ++lost;
        lost_samples += info.total_count_change;
        rcllf_lost_ctx_t ctx{};
        with_context += ros2_lf_message_lost_context(&ctx);
      };
    pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic, qos);
    sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic, qos, [&delivered](rosgraph_msgs::msg::Clock::SharedPtr m) {
        delivered.push_back(m->clock.sec);
      }, options);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "pub/sub did not match";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    take_stats_reset();
    ros2_lf_binding::g_handle_table.clear();
    ros2_lf_binding::g_handle_table.resize(1);
    auto & e = ros2_lf_binding::g_handle_table[0];
    e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
    e.subscription = sub_;
    e.topic = topic;
    e.node_class = "TakeDebtNode";
    e.callback_name = "on_clock";
    e.take_mode = mode;
    e.take_timeout_ms = 50;
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    auto invoke = [&]() {
#ifdef ROS2_LF_TAKE_DEBT
        return ros2_lf_invoke_sub_n(nullptr, 0, topic.c_str(), 1);
#else
        return ros2_lf_invoke_sub(nullptr, 0, topic.c_str());
#endif
      };

    publish({0});
    EXPECT_EQ(invoke(), 1);
    EXPECT_EQ(invoke(), 0) << "tag 1: the payload is late, the take must miss";
    publish({1, 2});  // 1 arrives late, 2 on time for tag 2
    EXPECT_EQ(invoke(), 1);
    publish({3});
    EXPECT_EQ(invoke(), 1);
    ros2_lf_binding::g_handle_table.clear();

    EXPECT_EQ(delivered, (std::vector<int>{0, 2, 3}));
    EXPECT_EQ(take_stats_snapshot().late_dropped, 1u);
    EXPECT_EQ(lost, 1) << "the late take-mode sample was not reported exactly once";
    EXPECT_EQ(lost_samples, 1u) << "the late take-mode sample was counted more than once";
    EXPECT_EQ(with_context, 0) << "a take-mode loss carried a correspondence lost context";
    ros2_lf_binding::corr_rt::reset_for_test();
    auto restore = [](const char * name, const std::string & v, bool had) {
        if (had) {
          setenv(name, v.c_str(), 1);
        } else {
          unsetenv(name);
        }
      };
    restore("ROS2_LF_TAKE_MODE", saved_mode, env_mode != nullptr);
    restore("ROS2_LF_TAKE_TIMEOUT_MS", saved_timeout, env_timeout != nullptr);
  }
};

TEST_F(TakeLateWithTardyHook, SkipWithoutTardyHook)
{
  run(TakeMode::Skip, false);
}

TEST_F(TakeLateWithTardyHook, SkipWithTardyHook)
{
  run(TakeMode::Skip, true);
}


// ── take_stats log lines ─────────────────────

namespace
{
std::vector<std::string> g_logged;

void capture_log(
  const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[1024];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_logged.emplace_back(buf);
}

/// The take_stats lines take_stats_log_summary logs now.
std::vector<std::string> logged_take_stats()
{
  g_logged.clear();
  const auto previous = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capture_log);
  take_stats_log_summary("test");
  rcutils_logging_set_output_handler(previous);
  std::vector<std::string> lines;
  for (const auto & l : g_logged) {
    if (l.find("ros2_lf_take_stats") != std::string::npos) {
      lines.push_back(l);
    }
  }
  return lines;
}

bool ends_with(const std::string & s, const std::string & tail)
{
  return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}
}  // namespace

TEST_F(TakeDebtTest, TakeStatsLinesUnchangedWithoutDebt)
{
  // One plain miss (remaining 0 owes nothing) creates the per-topic line.
  make_pair("/take_debt_test/stats_lines", 10);
  publish({0});
  EXPECT_EQ(take(0), 0);
  EXPECT_EQ(take(0), -1);
  const auto lines = logged_take_stats();
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(ends_with(lines[0], " sub_waited=0 wait_total_us=0 wait_max_us=0 wait_timeouts=0"))
    << "the ros2_lf_take_stats line changed for a program without skip debt: " << lines[0];
  EXPECT_EQ(
    lines[1], "ros2_lf_take_stats_topic: topic=/take_debt_test/stats_lines skipped=1 waited=0"
    " wait_total_us=0 wait_max_us=0 timeouts=0")
    << "the ros2_lf_take_stats_topic line changed for a program without skip debt";
}

#ifdef ROS2_LF_TAKE_DEBT
TEST_F(TakeDebtTest, TakeStatsLinesCarryTheDebtCounters)
{
  make_pair("/take_debt_test/stats_debt", 10);
  publish({0});
  EXPECT_EQ(take(1), 0);
  EXPECT_EQ(take(1), -1);
  publish({1, 2});
  EXPECT_EQ(take(1), 2);
  const auto lines = logged_take_stats();
  ASSERT_EQ(lines.size(), 2u);
  // The miss on a known count polled to the take timeout, so
  // the poll counters follow the debt counter.
  EXPECT_NE(lines[0].find(" wait_timeouts=0 late_dropped=1 poll_timeouts=1 poll_timeout_us="),
    std::string::npos) << lines[0];
  EXPECT_NE(lines[1].find(" timeouts=0 late_dropped=1 poll_timeouts=1 poll_timeout_us="),
    std::string::npos) << lines[1];
}
#endif

// The fast-mode poll timeouts follow the existing fields, so the
// rq3.py regexes for `wait_timeouts=` and `timeouts=` still read block timeouts.
TEST_F(TakeDebtTest, TakeStatsLinesCarryThePollTimeouts)
{
  make_pair("/take_debt_test/stats_poll", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_EQ(take(1), -1);
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto lines = logged_take_stats();
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[0].find(" wait_timeouts=0 poll_timeouts=1 poll_timeout_us="), std::string::npos)
    << lines[0];
  EXPECT_NE(lines[1].find(" timeouts=0 poll_timeouts=1 poll_timeout_us="), std::string::npos)
    << lines[1];
}

// ── Count link in real time ────────────────────────────────────
//
// On a count link the LF link carries the per-tag count and the data travels
// over DDS. The RTI grants the tag over TCP as soon as the writer's federate
// completes it; the sample itself is still on the UDP loopback, placed by the
// reader's receive thread, and under host load the grant wins the race. In
// real time the take did not wait, so the counted sample was dropped as late
// and reported lost, and the program's input depended on host scheduling
// (seen in a per-node brake_fusion federation). A trigger that knows its
// count waits for it in every mode, bounded by the take timeout.

namespace
{
/// The take warnings logged while `f` runs.
template<typename F>
std::vector<std::string> warnings_while(F && f)
{
  g_logged.clear();
  const auto previous = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capture_log);
  f();
  rcutils_logging_set_output_handler(previous);
  std::vector<std::string> lines;
  for (const auto & l : g_logged) {
    if (l.find("take_with_mode") != std::string::npos) {
      lines.push_back(l);
    }
  }
  return lines;
}
}  // namespace

// The skip debt of a reader is capped at its depth (more late
// samples than it holds cannot come back). A miss past the cap is not owed, so
// its sample, if it comes late, is delivered one tag late. Before the fix
// nothing counted such a miss; now take_stats does and the shutdown summary
// warns.
TEST_F(TakeDebtTest, AMissPastTheDebtCapIsNamedInTheRunSummary)
{
  make_pair("/take_debt_test/debt_cap", 2);
  timeout_ms_ = 10;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(take(1), -1) << "nothing was published, yet a sample was delivered";
  }
  std::string topic_line;
  for (const auto & l : logged_take_stats()) {
    if (l.find("ros2_lf_take_stats_topic: topic=" + topic_ + " ") != std::string::npos) {
      topic_line = l;
    }
  }
  EXPECT_NE(topic_line.find(" debt_capped=1"), std::string::npos)
    << "3 misses on a depth-2 reader, the third past the debt cap, but the run summary does "
    "not count it: " << topic_line;
  const auto warned = warnings_while([] {take_stats_log_summary("shutdown");});
  bool named = false;
  for (const auto & w : warned) {
    named = named || w.find("1 miss(es) were not owed") != std::string::npos;
  }
  EXPECT_TRUE(named) << "the shutdown summary does not warn that a miss was not owed";
}

TEST_F(TakeDebtTest, RealTimeWaitsForTheCountedSample)
{
  make_pair("/take_debt_test/real_time_wait", 10);
  ros2_lf_binding::set_lf_fast_for_test(false);
  std::thread late([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = 7;
      pub_->publish(m);
    });
  int got = -2;
  const auto warned = warnings_while([&] {got = take(1);});
  late.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(got, 7)
    << "real time: the counted sample arrived 30 ms after the tag and the take skipped the "
    "reaction instead of waiting for it"
    << (warned.empty() ? "" : "; logged: " + warned.front());
  EXPECT_TRUE(warned.empty()) << "the take warned on a sample it knew was owed: "
                              << warned.front();
  EXPECT_EQ(snap.skipped, 0u);
  EXPECT_EQ(snap.waited, 1u);
  EXPECT_GT(snap.wait_total_ns, 0u);
  EXPECT_EQ(reported_, 0u);
  // The next trigger delivers its own sample: nothing is one tag late.
  publish({8});
  EXPECT_EQ(take(1), 8);
  const auto after = take_stats_snapshot();
  EXPECT_EQ(after.late_dropped, 0u);
  EXPECT_EQ(after.poll_timeouts, 0u) << "a sample that came in time is no poll timeout";
}

TEST_F(TakeDebtTest, RealTimeMissAfterTheTimeoutStillOwesTheSample)
{
  make_pair("/take_debt_test/real_time_timeout", 10);
  ros2_lf_binding::set_lf_fast_for_test(false);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(take(1), -1) << "nothing was published, yet a sample was delivered";
  const auto waited_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  EXPECT_GE(waited_ms, 100)
    << "real time: the miss was declared before the take timeout";
  // The owed sample comes late, with the next trigger's own: the debt rule
  // drops the late one as before.
  publish({1, 2});
  EXPECT_EQ(take(1), 2);
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(snap.poll_timeouts, 1u);
  EXPECT_EQ(reported_, 1u);
}

TEST_F(TakeDebtTest, RealTimeUnknownWriteCountDoesNotWait)
{
  make_pair("/take_debt_test/real_time_unknown", 10);
  ros2_lf_binding::set_lf_fast_for_test(false);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(take(0), -1);
  const auto waited_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_LT(waited_ms, 50) << "a stub trigger owes nothing and must not wait";
  EXPECT_EQ(take_stats_snapshot().skipped, 1u);
  EXPECT_EQ(take_stats_snapshot().poll_timeouts, 0u);
}

// ── Tag deadline, unknown count, first-sample overflow ─────
//
// Before the fix: (a) the take timeout T ran from the take's own
// start, so a take that ran late accepted a sample later than tag + T with no
// report; (b) a federate's take with an unknown write count did not wait and
// skipped silently; (c) a reader overflow before a writer's first take was
// taken as the baseline and never reported. The owner's rule (2026-11-02): a
// message later than T counts as lost and is reported.

namespace
{
int64_t realtime_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/// Real time at tag `tag_ns` (CLOCK_REALTIME) while it lives.
struct RealTimeTag
{
  explicit RealTimeTag(int64_t tag_ns)
  {
    ros2_lf_binding::set_lf_fast_for_test(false);
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(tag_ns, 0u));
  }
  ~RealTimeTag()
  {
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
    ros2_lf_binding::corr_rt::set_federated(false);
  }
};
}  // namespace

TEST_F(TakeDebtTest, TakeTimeoutRunsFromTheTagNotFromTheTake)
{
  make_pair("/take_deadline/from_tag", 10);
  {
    // The take runs 300 ms after its tag, T = 100 ms: the deadline is past.
    RealTimeTag at(realtime_now_ns() - 300'000'000);
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_EQ(take(1), -1);
    const auto waited_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    EXPECT_LT(waited_ms, 50)
      << "the take waited " << waited_ms << " ms although tag + T had passed 200 ms ago: T ran "
      "from the take's start, not from the tag";
  }
  // The owed sample of that miss: drop it with the next trigger's own.
  publish({1, 2});
  EXPECT_EQ(take(1), 2);
  EXPECT_EQ(take_stats_snapshot().after_deadline, 0u)
    << "outside an LF tag no arrival is judged against a deadline";
}

TEST_F(TakeDebtTest, SampleReceivedAfterTagPlusTimeoutIsDroppedAndReported)
{
  make_pair("/take_deadline/late_arrival", 10);
  publish({5});  // received now
  std::vector<std::string> warned;
  int got = -2;
  {
    // publish() lets it land for 200 ms. Its trigger's tag is 500 ms back,
    // T = 100 ms: it came about 200 ms after tag + T.
    RealTimeTag at(realtime_now_ns() - 500'000'000);
    warned = warnings_while([&] {got = take(1);});
  }
  EXPECT_EQ(got, -1)
    << "a sample received 200 ms after tag + take_timeout_ms was delivered as on time; by the "
    "owner's rule it is lost and reported";
  EXPECT_EQ(reported_, 1u) << "the late sample did not reach the message_lost report";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.after_deadline, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
  ASSERT_FALSE(warned.empty());
  EXPECT_NE(warned.front().find("after tag + take_timeout_ms"), std::string::npos)
    << warned.front();
  // It settled its own trigger: no debt, so the next trigger keeps its sample.
  publish({6});
  {
    RealTimeTag at(realtime_now_ns());
    EXPECT_EQ(take(1), 6) << "an on-time sample after a late one must be delivered";
  }
  EXPECT_EQ(take_stats_snapshot().late_dropped, 1u);
}

TEST_F(TakeDebtTest, BlockModeDropsASampleReceivedAfterTagPlusTimeout)
{
  make_pair("/take_deadline/block_late", 10);
  publish({9});
  rclcpp::MessageInfo info;
  rosgraph_msgs::msg::Clock m;
  uint64_t reported = 0;
  bool taken = true;
  {
    RealTimeTag at(realtime_now_ns() - 500'000'000);  // received ~200 ms after tag + T
    taken = take_with_mode(*sub_, &m, info, TakeMode::Block, 100, topic_, 1, &reported);
  }
  EXPECT_FALSE(taken) << "block mode delivered a sample received after tag + take_timeout_ms";
  EXPECT_EQ(reported, 1u);
  EXPECT_EQ(take_stats_snapshot().after_deadline, 1u);
}

TEST_F(TakeDebtTest, FederateUnknownWriteCountWaitsForTheTagDeadline)
{
  make_pair("/take_deadline/unknown_fed", 10);
  int got = -2;
  {
    RealTimeTag at(realtime_now_ns());
    ros2_lf_binding::corr_rt::set_federated(true);
    std::thread late([this] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        rosgraph_msgs::msg::Clock m;
        m.clock.sec = 7;
        pub_->publish(m);
      });
    got = take(0);
    late.join();
  }
  EXPECT_EQ(got, 7)
    << "a federate's unknown-count take skipped a sample that arrived 30 ms after its tag, "
    "within T: the program's input depended on host scheduling";
  EXPECT_EQ(take_stats_snapshot().skipped, 0u);
  // Nothing comes: the miss is counted and warned, never silent.
  std::vector<std::string> warned;
  {
    RealTimeTag at(realtime_now_ns());
    ros2_lf_binding::corr_rt::set_federated(true);
    warned = warnings_while([&] {got = take(0);});
  }
  EXPECT_EQ(got, -1);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.unknown_missed, 1u)
    << "a federate's unknown-count miss was counted only as skipped, with no warning";
  ASSERT_FALSE(warned.empty()) << "a federate's unknown-count miss was silent";
  EXPECT_NE(warned.front().find("unknown write count"), std::string::npos) << warned.front();
  EXPECT_EQ(reported_, 0u) << "an unknown-count miss may be no write: it is not reported lost";
}

TEST_F(TakeDebtTest, OverflowBeforeTheFirstTakeOfAStartWriterIsReported)
{
  make_pair("/take_deadline/first_overflow", 10);
  wait_publishers(1);
  std::vector<std::array<uint8_t, 16>> gids;
  for (const auto & info : node_->get_publishers_info_by_topic(topic_)) {
    std::array<uint8_t, 16> gid{};
    std::copy_n(info.endpoint_gid().begin(), gid.size(), gid.begin());
    gids.push_back(gid);
  }
  ASSERT_EQ(gids.size(), 1u);
  ros2_lf_binding::take_note_start_writers(*sub_, gids);
  // 30 publishes before any take: the KeepLast(10) reader keeps 21 .. 30.
  publish_range(1, 30);
  ASSERT_EQ(take(1), 21);
  EXPECT_EQ(reported_, 20u)
    << "a writer seen before the first tag overflowed the KeepLast(10) reader before its first "
    "take, 20 samples replaced, reported lost = " << reported_
    << ": the first sample was taken as the baseline";
  EXPECT_EQ(take_stats_snapshot().reader_lost, 20u);
}

// ── Reader overflow ────────────────────────────────────────────
//
// A reliable KeepLast reader that is not serviced replaces its oldest sample
// when it is full, and the middleware raises no message_lost for the replaced
// ones (Fast DDS, measured).
// brake_fusion's sensors read /sim/state through a
// KeepLast(10) reader on a legacy edge, and a sensor whose executor stalled
// for more than 10 plant steps silently skipped plant states: no late drop,
// no lost event, and the stop moved (loaded depth-10 stall runs, 10 of 10).
// The binding counts the samples between the last taken publication number of
// a writer and the next one taken, and reports them as lost.

TEST_F(TakeDebtTest, ReaderOverflowIsReportedAsLost)
{
  make_pair("/take_debt/reader_overflow", 10);
  publish({1});
  ASSERT_EQ(take(1), 1);  // the writer's baseline: publication 1 taken
  // 30 more publishes with no take between them: the KeepLast(10) reader
  // keeps publications 22 .. 31 and replaced 2 .. 21.
  publish_range(2, 31);
  reported_ = 0;
  ASSERT_EQ(take(1), 22);
  EXPECT_EQ(reported_, 20u)
    << "31 samples published into a KeepLast(10) reader, 20 replaced before the take, "
    << "reported lost = " << reported_ << ": the loss is silent";
#ifdef ROS2_LF_READER_GAP
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.reader_lost, 20u);
  ASSERT_EQ(snap.topics.count(topic_), 1u);
  EXPECT_EQ(snap.topics.at(topic_).reader_lost, 20u);
  EXPECT_EQ(snap.topics.at(topic_).late_dropped, 0u);
#endif
  // The takes that follow are gap free: 23 .. 31, nothing more reported.
  reported_ = 0;
  for (int id = 23; id <= 31; ++id) {
    ASSERT_EQ(take(1), id);
  }
  EXPECT_EQ(reported_, 0u);
}

TEST_F(TakeDebtTest, ReaderOverflowCountsPerWriter)
{
  // Two writers interleaved: each one's publication numbers are contiguous
  // on their own, so a take order that alternates between them is no gap,
  // and a second writer's first sample sets its own baseline.
  make_pair("/take_debt/reader_overflow_two_writers", 10);
  auto other = second_publisher();
  wait_publishers(2);
  publish({1});
  ASSERT_EQ(take(1), 1);
  send(other, {101});
  ASSERT_EQ(take(1), 101);
  publish({2});
  send(other, {102});
  publish({3});
  ASSERT_EQ(take(1), 2);
  ASSERT_EQ(take(1), 102);
  ASSERT_EQ(take(1), 3);
  EXPECT_EQ(reported_, 0u);
#ifdef ROS2_LF_READER_GAP
  EXPECT_EQ(take_stats_snapshot().reader_lost, 0u);
#endif
}

// Review of the fix (gpt-6-sol): on a best-effort reader a sample lost
// on the wire is also a DDS message_lost event. Could the binding count a
// loss DDS already reported? In rcllf no physical QoS event is ever executed:
// the ingress takes every ready event of an admitted subscription and drops
// it (ingress.cpp take_qos_events), the legacy and tag take paths call
// note_sequence, the corr take paths do not (a corr edge's loss is the MISSING
// ordinal), so a lost sample reaches the user's message_lost handler from one
// source whatever the reliability. Run evidence: one radar datagram dropped on
// the wire of a per-node brake_fusion federation, one report.
// In-process the test shows the other half: for both reliabilities the
// replacement in a full KeepLast reader is silent in DDS (the event's count,
// taken the way the ingress takes it, stays 0 and its callback never ran) and
// the binding declares the replaced samples exactly once.
TEST_F(TakeDebtTest, ReaderOverflowIsDeclaredOnceForEitherReliability)
{
  for (const bool best_effort : {true, false}) {
    SCOPED_TRACE(best_effort ? "best effort reader" : "reliable reader");
    uint64_t dds_executed = 0;
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.message_lost_callback =
      [&dds_executed](rclcpp::QOSMessageLostInfo &) {++dds_executed;};
    make_pair(
      std::string("/take_debt/overflow_once_") + (best_effort ? "be" : "rel"), 10, best_effort,
      options);
    reported_ = 0;
    publish({1});
    ASSERT_EQ(take(1), 1);
    publish_range(2, 31);  // the KeepLast(10) reader keeps 22 .. 31
    // DDS's own account of the same reader, taken as the ingress takes it:
    // a replacement in the history is no SAMPLE_LOST. On the
    // best-effort pair a datagram lost on the local wire would be one, and
    // the gap would then be partly wire loss: that run proves nothing here.
    const auto & handlers = sub_->get_event_handlers();
    const auto it = handlers.find(RCL_SUBSCRIPTION_MESSAGE_LOST);
    ASSERT_NE(it, handlers.end()) << "no message_lost handler on the subscription";
    const auto data = it->second->take_data();
    ASSERT_TRUE(data) << "the message_lost event could not be taken";
    const auto & dds = *std::static_pointer_cast<rclcpp::QOSMessageLostInfo>(data);
    if (best_effort && dds.total_count != 0) {
      GTEST_SKIP() << "the best-effort pair lost " << dds.total_count
                   << " sample(s) on the local wire; the replacement case is inconclusive";
    }
    EXPECT_EQ(dds.total_count, 0)
      << "DDS reported " << dds.total_count << " lost sample(s) for the replacement: the "
      << "binding's count would be a second report if the event were ever executed";
    ASSERT_EQ(take(1), 22);
    EXPECT_EQ(reported_, 20u) << "the 20 replaced samples were not declared exactly once";
    for (int id = 23; id <= 31; ++id) {
      ASSERT_EQ(take(1), id);
    }
    EXPECT_EQ(reported_, 20u);
    EXPECT_EQ(dds_executed, 0u) << "a DDS message_lost event ran the user's handler";
  }
}

// The content filter can change after the first take (set_content_filter is
// public; a reaction may call it), so the filtered state is read at every
// take: while a filter is on, the holes it leaves in a writer's numbers are
// by design and nothing is counted; when it changes, the recorded numbers
// mean nothing any more and the next sample of each writer is a new baseline.
// rmw_fastrtps clears a filter created with the
// subscription on an empty expression; an rmw that cannot skips the case.
TEST_F(TakeDebtTest, ContentFilterChangeResetsTheGapBaseline)
{
  rclcpp::SubscriptionOptions options;
  options.content_filter_options.filter_expression = "clock.nanosec = 0";
  make_pair("/take_debt/cft_change", 10, false, options);
  ASSERT_TRUE(sub_->is_cft_enabled()) << "the rmw has no content filter";
  try {
    sub_->set_content_filter("");
  } catch (const std::exception & e) {
    GTEST_SKIP() << "this rmw cannot clear a content filter: " << e.what();
  }
  if (sub_->is_cft_enabled()) {
    GTEST_SKIP() << "this rmw keeps the content filter enabled after clearing it";
  }
  // Unfiltered: replaced samples are declared.
  publish({1});
  ASSERT_EQ(take(1), 1);
  publish_range(2, 31);
  ASSERT_EQ(take(1), 22);
  EXPECT_EQ(reported_, 20u);
  for (int id = 23; id <= 31; ++id) {
    ASSERT_EQ(take(1), id);
  }
  // Filter on after the first take: its holes are not losses.
  sub_->set_content_filter("clock.nanosec = 0");
  ASSERT_TRUE(sub_->is_cft_enabled());
  publish_filtered({32, 33});
  publish({34});
  ASSERT_EQ(take(1), 34);
  EXPECT_EQ(reported_, 20u) << "the holes of a filter set after the first take were declared lost";
  // Filter off again: the first sample is a baseline, not a gap after 31.
  sub_->set_content_filter("");
  ASSERT_FALSE(sub_->is_cft_enabled());
  publish({35});
  ASSERT_EQ(take(1), 35);
  publish({36});
  ASSERT_EQ(take(1), 36);
  EXPECT_EQ(reported_, 20u)
    << "the samples a filter rejected were declared lost after it was cleared";
#ifdef ROS2_LF_READER_GAP
  EXPECT_EQ(take_stats_snapshot().topics.at(topic_).reader_lost, 20u);
#endif
}

// ── Drop by tag ────────────────────────────────────────────────
//
// A sample that misses its take is never delivered later: the take finds its
// write's record (publisher GID, sequence number) in the topic's tag log and
// drops it as soon as it arrives at a later trigger, reported as lost. The
// tests feed the log directly, as the publication gate does for a write made
// inside a reaction (test_take_mode links no gate): a record per write, a
// claim per ros2_lf_claim_publish, and an off-reaction publish that fills the
// newest open claim. A DDS sequence number counts from 1 per publisher.

#ifdef ROS2_LF_TAKE_BY_TAG
namespace
{
namespace tl = ros2_lf_binding::tag_log;
using Pub = rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr;
constexpr int kRid = 7;

tl::Gid gid_of(const Pub & p)
{
  tl::Gid g{};
  const rmw_gid_t & r = p->get_gid();
  std::copy_n(r.data, g.size(), g.begin());
  return g;
}
}  // namespace

class DropByTagTest : public TakeDebtTest
{
protected:
  /// A write inside a reaction: recorded, then published.
  void write(const Pub & p, int id)
  {
    tl::on_publish(topic_, true, true, gid_of(p), ++seq_[p.get()]);
    send_one(p, id);
  }

  /// ros2_lf_claim_publish inside a reaction.
  void claim() {tl::on_claim(topic_);}

  /// A publish outside any reaction (a claimed sample, or an untagged one).
  void outside(const Pub & p, int id)
  {
    tl::on_publish(topic_, false, true, gid_of(p), ++seq_[p.get()]);
    send_one(p, id);
  }

  void send_one(const Pub & p, int id)
  {
    rosgraph_msgs::msg::Clock m;
    m.clock.sec = id;
    p->publish(m);
  }

  void settle() {std::this_thread::sleep_for(std::chrono::milliseconds(200));}

  void message_lost_schedule(const std::string & topic, int late_ms);

  /// One trigger of `k` writes: k invokes, the IDs they deliver in order.
  std::vector<int> trigger(int k, TakeMode mode = TakeMode::Skip, int64_t timeout_ms = 100)
  {
    std::vector<int> got;
    for (int i = k; i >= 1; --i) {
      auto tt = ros2_lf_binding::take_by_tag(kRid, *sub_, mode, timeout_ms, topic_, i);
      EXPECT_FALSE(tt.count_rule) << "the trigger fell back to the count rule";
      reported_ += tt.lost;
      for (const auto & s : tt.deliver) {
        rosgraph_msgs::msg::Clock m;
        EXPECT_TRUE(ros2_lf_binding::deserialize_taken(*sub_, s, &m));
        got.push_back(m.clock.sec);
      }
    }
    return got;
  }

  /// One trigger of `k` writes as reaction_dispatch.cpp runs it: take by tag,
  /// else the count rule (take_with_mode). `k` 0 is ros2_lf_invoke_sub, whose
  /// write count is unknown: one invoke.
  std::vector<int> dispatch(int k)
  {
    std::vector<int> got;
    for (int i = k; i >= std::min(k, 1); --i) {
      auto tt = ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, timeout_ms_, topic_, i);
      if (tt.count_rule) {
        const int v = take(i);
        if (v >= 0) {
          got.push_back(v);
        }
        continue;
      }
      reported_ += tt.lost;
      for (const auto & s : tt.deliver) {
        rosgraph_msgs::msg::Clock m;
        EXPECT_TRUE(ros2_lf_binding::deserialize_taken(*sub_, s, &m));
        got.push_back(m.clock.sec);
      }
    }
    return got;
  }

  /// kKeep later writes of a writer with no samples, each its own trigger
  /// that is passed: every record older than them is forgotten.
  void forget_all()
  {
    tl::Gid other{};
    other.fill(0xEE);
    for (uint64_t i = 1; i <= tl::kKeep; ++i) {
      tl::on_publish(topic_, true, true, other, i);
      ros2_lf_binding::tag_pass(kRid, 1);
    }
  }

  std::map<const void *, uint64_t> seq_;
};

using Ids = std::vector<int>;

TEST_F(DropByTagTest, FastModeWaitsForTheRecordedWritesSample)
{
  // The by-tag path: the write is recorded, its sample arrives
  // 30 ms after the trigger.
  make_pair("/drop_by_tag/fast_wait", 10);
  ros2_lf_binding::set_lf_fast_for_test(true);
  tl::on_publish(topic_, true, true, gid_of(pub_), ++seq_[pub_.get()]);
  std::thread late([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      send_one(pub_, 5);
    });
  const Ids got = trigger(1);
  late.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(got, Ids{5})
    << "fast mode: the recorded write's sample arrived 30 ms after the tag and the take "
    "skipped the reaction instead of waiting for it";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 0u);
  EXPECT_EQ(snap.waited, 1u);
  EXPECT_EQ(reported_, 0u);
}

// A burst above the reader depth at one tag. The reader keeps
// the newest 10 of 15 writes; the gap at the first take declares the 5
// replaced ones lost, and the takes of their records end at once
// instead of each polling to the take timeout (fast mode: 5 x 100 ms).
TEST_F(DropByTagTest, ReplacedSamplesOfOneTriggerCostNoPollEach)
{
  make_pair("/drop_by_tag/burst_overflow", 10);
  ros2_lf_binding::take_note_start_writers(*sub_, {gid_of(pub_)});
  for (int id = 1; id <= 15; ++id) {
    write(pub_, id);
  }
  settle();
  ros2_lf_binding::set_lf_fast_for_test(true);
  const auto t0 = std::chrono::steady_clock::now();
  Ids got;
  const auto warned = warnings_while([&] {got = trigger(15);});
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(got, Ids({6, 7, 8, 9, 10, 11, 12, 13, 14, 15}));
  EXPECT_EQ(reported_, 5u) << "the 5 replaced samples were not reported as lost at their tag";
  EXPECT_LT(ms, 100)
    << "the 5 replaced samples cost a poll to the take timeout each (" << ms
    << " ms for one trigger)";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 5u);
  EXPECT_EQ(snap.poll_timeouts, 0u) << "a replaced sample was polled for";
  EXPECT_EQ(snap.late_dropped, 0u);
  EXPECT_EQ(snap.topics.at(topic_).reader_lost, 5u);
  for (const auto & w : warned) {
    EXPECT_EQ(w.find("no sample for a trigger"), std::string::npos)
      << "a replaced sample was declared a miss: " << w;
  }
  EXPECT_EQ(warned.size(), 1u) << "the gap WARN alone declares the loss";
  // The next trigger delivers its own sample at once.
  tl::on_publish(topic_, true, true, gid_of(pub_), ++seq_[pub_.get()]);
  send_one(pub_, 16);
  settle();
  EXPECT_EQ(trigger(1), Ids{16});
  EXPECT_EQ(reported_, 5u);
}

// On the count rule (take_with_mode): the same burst, the same
// outcome, and no debt is owed for the replaced samples, so the next
// trigger's sample is neither waited for nor dropped as late.
TEST_F(TakeDebtTest, FastModeReplacedSamplesOweNoDebtAndNoPoll)
{
  make_pair("/take_mode_test/burst_overflow", 10);
  ros2_lf_binding::take_note_start_writers(*sub_, {gid_of(pub_)});
  publish_range(1, 15);
  ros2_lf_binding::set_lf_fast_for_test(true);
  const auto t0 = std::chrono::steady_clock::now();
  Ids got;
  const auto warned = warnings_while(
    [&] {
      for (int i = 15; i >= 1; --i) {
        got.push_back(take(i));
      }
    });
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(got, Ids({6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1}));
  EXPECT_EQ(reported_, 5u) << "the 5 replaced samples were not reported as lost at their tag";
  EXPECT_LT(ms, 100)
    << "the 5 replaced samples cost a poll to the take timeout each (" << ms
    << " ms for one trigger)";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 5u);
  EXPECT_EQ(snap.poll_timeouts, 0u) << "a replaced sample was polled for";
  for (const auto & w : warned) {
    EXPECT_EQ(w.find("no sample for a trigger"), std::string::npos)
      << "a replaced sample was declared a miss: " << w;
  }
  EXPECT_EQ(warned.size(), 1u) << "the gap WARN alone declares the loss";
  publish({16});
  EXPECT_EQ(take(1), 16) << "the replaced samples left a debt";
  EXPECT_EQ(take_stats_snapshot().late_dropped, 0u);
  EXPECT_EQ(reported_, 5u);
}

// A hole found at this tag may be an earlier
// tag's owed sample, never a credit for a sample of this tag still on its
// way. One miss owes a sample; the writer then bursts 11 into depth 10, so
// publication 1 is replaced: that hole settles the debt, and the trigger's
// 11th sample, published while its take polls, is delivered, not skipped.
TEST_F(TakeDebtTest, FastModeHoleSettlesDebtBeforeItCreditsTheTag)
{
  make_pair("/take_mode_test/hole_settles_debt", 10);
  ros2_lf_binding::take_note_start_writers(*sub_, {gid_of(pub_)});
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_EQ(take(1), -1) << "an earlier trigger's sample never came";
  EXPECT_EQ(take_stats_snapshot().poll_timeouts, 1u);
  publish_range(1, 11);
  Ids got;
  for (int i = 11; i >= 2; --i) {
    got.push_back(take(i));
  }
  EXPECT_EQ(got, Ids({2, 3, 4, 5, 6, 7, 8, 9, 10, 11}));
  EXPECT_EQ(reported_, 1u) << "the replaced publication 1 is the hole";
  std::thread late(
    [&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      rosgraph_msgs::msg::Clock m;
      m.clock.sec = 12;
      pub_->publish(m);
    });
  const auto t0 = std::chrono::steady_clock::now();
  const int last = take(1);
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  late.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(last, 12)
    << "an earlier tag's loss was credited to this tag and a sample on its "
    "way was skipped";
  EXPECT_LT(ms, 100) << "the take polled to the timeout instead of taking the sample";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.poll_timeouts, 1u) << "only the first miss polled to the timeout";
  EXPECT_EQ(snap.late_dropped, 0u) << "the hole settled the debt; nothing is dropped as late";
  EXPECT_EQ(reported_, 1u);
}

// The tag link: the record of a cross-federate write arrives
// over the LF link ahead of its DDS sample, so in real time too a recorded
// write's empty reader is a sample on its way, not a miss (k3assigned per-node
// brake_fusion, controller on /fused/range at tag 490 ms once the fusion
// federate waited for its own count-link samples).
TEST_F(DropByTagTest, RealTimeWaitsForTheRecordedWritesSample)
{
  make_pair("/drop_by_tag/real_time_wait", 10);
  ros2_lf_binding::set_lf_fast_for_test(false);
  tl::on_publish(topic_, true, true, gid_of(pub_), ++seq_[pub_.get()]);
  std::thread late([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      send_one(pub_, 5);
    });
  Ids got;
  const auto warned = warnings_while([&] {got = trigger(1);});
  late.join();
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  EXPECT_EQ(got, Ids{5})
    << "real time: the recorded write's sample arrived 30 ms after the tag and the take "
    "skipped the reaction instead of waiting for it"
    << (warned.empty() ? "" : "; logged: " + warned.front());
  EXPECT_TRUE(warned.empty()) << "the take warned on a sample it knew was owed: "
                              << warned.front();
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 0u);
  EXPECT_EQ(snap.waited, 1u);
  EXPECT_EQ(snap.poll_timeouts, 0u);
  EXPECT_EQ(reported_, 0u);
}

TEST_F(DropByTagTest, RealTimeMissAfterTheTimeoutStillDropsTheLateSample)
{
  make_pair("/drop_by_tag/real_time_timeout", 10);
  ros2_lf_binding::set_lf_fast_for_test(false);
  tl::on_publish(topic_, true, true, gid_of(pub_), ++seq_[pub_.get()]);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(trigger(1), Ids{});
  const auto waited_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();
  EXPECT_GE(waited_ms, 100)
    << "real time: the miss was declared before the take timeout";
  send_one(pub_, 0);  // the recorded write's sample, late
  write(pub_, 1);
  settle();
  EXPECT_EQ(trigger(1), Ids{1}) << "the late sample was delivered one tag late";
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(snap.poll_timeouts, 1u);
  EXPECT_EQ(reported_, 1u);
}

TEST_F(DropByTagTest, LoneLateSampleIsDroppedNotDeliveredLate)
{
  make_pair("/drop_by_tag/lone_late", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  claim();  // 1 is late
  settle();
  EXPECT_EQ(trigger(1), Ids{});
  outside(pub_, 1);
  claim();  // 2 is late too: 1 is alone at tag 2
  settle();
  EXPECT_EQ(trigger(1), Ids{}) << "the lone late ID 1 was delivered one tag late";
  outside(pub_, 2);
  write(pub_, 3);
  settle();
  EXPECT_EQ(trigger(1), Ids{3});
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 2u);
  EXPECT_EQ(snap.late_dropped, 2u);
  EXPECT_EQ(reported_, 2u);
  EXPECT_EQ(snap.untagged, 0u);
}

TEST_F(DropByTagTest, LateSampleBeforeAnOnTimeOneIsDropped)
{
  make_pair("/drop_by_tag/late_next", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  claim();
  settle();
  EXPECT_EQ(trigger(1), Ids{});
  outside(pub_, 1);
  write(pub_, 2);
  settle();
  EXPECT_EQ(trigger(1), Ids{2});
  EXPECT_EQ(reported_, 1u);
  EXPECT_EQ(take_stats_snapshot().late_dropped, 1u);
}

TEST_F(DropByTagTest, LostClaimCostsNoOnTimeSample)
{
  make_pair("/drop_by_tag/lost", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  claim();  // never published
  settle();
  EXPECT_EQ(trigger(1), Ids{});
  for (int id = 2; id <= 4; ++id) {
    write(pub_, id);
    settle();
    EXPECT_EQ(trigger(1), Ids{id}) << "tag " << id << " slipped behind the lost sample";
  }
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.late_dropped, 0u);
}

TEST_F(DropByTagTest, ClaimFilledBeforeTheTakeIsOnTime)
{
  make_pair("/drop_by_tag/claim_on_time", 10);
  claim();
  outside(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  EXPECT_EQ(take_stats_snapshot().first_try, 1u);
}

TEST_F(DropByTagTest, UntaggedSampleIsDeliveredInAdditionToTheTagsOwn)
{
  make_pair("/drop_by_tag/untagged", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  outside(pub_, 100);  // no open claim: no record
  write(pub_, 1);
  settle();
  EXPECT_EQ(trigger(1), (Ids{100, 1}));
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 1u);
  EXPECT_EQ(snap.topics.at(topic_).untagged, 1u);
  EXPECT_EQ(snap.late_dropped, 0u);
}

TEST_F(DropByTagTest, FanInLateWriterIsDroppedAndTheOthersDelivered)
{
  make_pair("/drop_by_tag/fan", 10);
  auto b = second_publisher();
  wait_publishers(2);
  write(pub_, 0);
  write(b, 1000);
  settle();
  EXPECT_EQ(trigger(2), (Ids{0, 1000}));
  write(pub_, 1);
  claim();  // 1001 is late
  settle();
  EXPECT_EQ(trigger(2), Ids{1});
  outside(b, 1001);
  write(pub_, 2);
  write(b, 1002);
  settle();
  EXPECT_EQ(trigger(2), (Ids{2, 1002}));
  EXPECT_EQ(reported_, 1u);
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.skipped, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
}

TEST_F(DropByTagTest, BlockModeWaitsForAClaimedSample)
{
  make_pair("/drop_by_tag/block_wait", 10);
  claim();
  std::thread late([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      outside(pub_, 0);
    });
  EXPECT_EQ(trigger(1, TakeMode::Block, 2000), Ids{0});
  late.join();
  EXPECT_EQ(take_stats_snapshot().waited, 1u);
}

TEST_F(DropByTagTest, BlockTimeoutThenLateSampleIsDropped)
{
  make_pair("/drop_by_tag/block_timeout", 10);
  claim();
  EXPECT_EQ(trigger(1, TakeMode::Block, 50), Ids{});
  outside(pub_, 0);
  write(pub_, 1);
  settle();
  EXPECT_EQ(trigger(1, TakeMode::Block, 50), Ids{1})
    << "block mode delivered the sample that missed its timeout";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.timeouts, 1u);
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(reported_, 1u);
}

// Block mode: past take_timeout_ms the late sample is dropped
// and reported once by tag, with or without a tardy-drop edge in the process.
class DropByTagWithTardyHook : public DropByTagTest
{
protected:
  void run(bool tardy_hook)
  {
    ros2_lf_binding::corr_rt::reset_for_test();
    if (tardy_hook) {
      ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {}, "/drop_by_tag/r3_xfed", "Sub", 0,
        "/drop_by_tag/r3_xfed", 5000000};
      s.role = ros2_lf_binding::CorrEdgeRole::kIngress;
      ros2_lf_binding::register_corr_edge(s);
      ros2_lf_binding::set_corr_edge_tardy_drop(0);
    }
    make_pair(tardy_hook ? "/drop_by_tag/r3_block_hook" : "/drop_by_tag/r3_block", 10);
    claim();
    EXPECT_EQ(trigger(1, TakeMode::Block, 50), Ids{});
    outside(pub_, 0);
    write(pub_, 1);
    settle();
    EXPECT_EQ(trigger(1, TakeMode::Block, 50), Ids{1});
    const auto snap = take_stats_snapshot();
    EXPECT_EQ(snap.timeouts, 1u);
    EXPECT_EQ(snap.late_dropped, 1u);
    EXPECT_EQ(reported_, 1u) << "the late block-mode sample was not reported exactly once";
    ros2_lf_binding::corr_rt::reset_for_test();
  }
};

TEST_F(DropByTagWithTardyHook, BlockWithoutTardyHook)
{
  run(false);
}

TEST_F(DropByTagWithTardyHook, BlockWithTardyHook)
{
  run(true);
}


TEST_F(DropByTagTest, UntakenTriggerStillUsesUpItsWrites)
{
  // An unbound or inactive subscription reaction takes nothing (tag_pass).
  make_pair("/drop_by_tag/pass", 10);
  write(pub_, 0);
  ros2_lf_binding::tag_pass(kRid, 1);
  write(pub_, 1);
  settle();
  EXPECT_EQ(trigger(1), Ids{1}) << "the untaken trigger's sample slipped into the next tag";
  EXPECT_EQ(reported_, 1u);
}

TEST_F(DropByTagTest, UnrecordedWritesKeepTheReaderOnTheCountRule)
{
  make_pair("/drop_by_tag/fallback", 10);
  publish({0});  // no record: a cross-federate write
  const auto tt = ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, 100, topic_, 1);
  EXPECT_TRUE(tt.count_rule);
  EXPECT_TRUE(tt.deliver.empty());
  EXPECT_EQ(take(1), 0);
  // A local write on the same topic later does not line up with this
  // reader's triggers any more: it stays on the count rule.
  write(pub_, 1);
  settle();
  EXPECT_TRUE(
    ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, 100, topic_, 1).count_rule);
  EXPECT_EQ(take(1), 1);
}

TEST_F(DropByTagTest, AnonymousWriteFallsBackForItsTriggerOnly)
{
  make_pair("/drop_by_tag/anonymous", 10);
  tl::on_publish(topic_, true, false, {}, 0);  // the gate read no GID
  publish({0});
  EXPECT_TRUE(
    ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, 100, topic_, 1).count_rule);
  EXPECT_EQ(take(1), 0);
  write(pub_, 1);
  settle();
  EXPECT_EQ(trigger(1), Ids{1}) << "the anonymous record was not used up";
}

TEST_F(DropByTagTest, PassedTriggerDropsAndReportsItsHeldSample)
{
  make_pair("/drop_by_tag/pass_held", 10);
  write(pub_, 0);
  write(pub_, 1);  // the writer is one tag ahead: 1 is held by the take of 0
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  EXPECT_EQ(ros2_lf_binding::tag_pass(kRid, 1), 1u) << "the held sample of the passed trigger";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(snap.topics.at(topic_).late_dropped, 1u);
  write(pub_, 2);
  settle();
  EXPECT_EQ(trigger(1), Ids{2});
}

// Review round 2, item 1: a late sample whose record the log already forgot
// (kKeep later records) is still known to be old, by its writer's highest
// forgotten sequence number, and is dropped, not delivered as untagged.
TEST_F(DropByTagTest, LateSampleOfAForgottenRecordIsDropped)
{
  make_pair("/drop_by_tag/forgotten_record", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  claim();
  settle();
  EXPECT_EQ(trigger(1), Ids{});
  outside(pub_, 1);  // fills the claim: a late sample with a record
  forget_all();
  write(pub_, 2);
  settle();
  EXPECT_EQ(trigger(1), Ids{2}) << "the late ID 1 was delivered after its record was forgotten";
  // A later unclaimed publish of the same writer is still untagged.
  outside(pub_, 100);
  write(pub_, 3);
  settle();
  EXPECT_EQ(trigger(1), (Ids{100, 3}));
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(reported_, 1u);
  EXPECT_EQ(snap.untagged, 1u);
}

// Review round 2, item 1: a claim forgotten before its sample came. The
// publish that fills it finds no open claim; it fills the forgotten one and
// is old, not untagged.
TEST_F(DropByTagTest, LateSampleOfAForgottenClaimIsDropped)
{
  make_pair("/drop_by_tag/forgotten_claim", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(trigger(1), Ids{0});
  claim();
  settle();
  EXPECT_EQ(trigger(1), Ids{});
  forget_all();
  outside(pub_, 1);
  write(pub_, 2);
  settle();
  EXPECT_EQ(trigger(1), Ids{2}) << "the late ID 1 of a forgotten claim was delivered";
  outside(pub_, 100);  // the forgotten claim is filled: this one is untagged
  write(pub_, 3);
  settle();
  EXPECT_EQ(trigger(1), (Ids{100, 3}));
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.late_dropped, 1u);
  EXPECT_EQ(reported_, 1u);
  EXPECT_EQ(snap.untagged, 1u);
}

// Review round 2, item 2, kept as a guard: a cross-federate reader. A fan-in
// with writers in two federates never pairs a remote writer with a local one
// here: the generator refuses it as legacy (P17h, fan_in_tag_errors;
// test_one_writer_local_one_remote_is_refused), and a fan-in group is admitted only
// when its writers are all local or all remote, never mixed. So the topic of
// a legacy reader carries no write of this process: no trigger is served by
// tag and no sample counts as untagged.
TEST_F(DropByTagTest, CrossFederateReaderStaysOnTheCountRule)
{
  make_pair("/drop_by_tag/remote_only", 10);
  for (int id = 0; id < 3; ++id) {
    publish({id});  // published in another process: no record
    EXPECT_TRUE(
      ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, 100, topic_, 1).count_rule);
    EXPECT_EQ(take(1), id);
  }
  EXPECT_EQ(take_stats_snapshot().untagged, 0u);
}

// A cross-federate take link. The writer is in another
// federate: its writes are recorded in the writer process's tag log (here a
// log of its own, `wlog_`), and each trigger carries the events packed since
// the last one, applied to the reader's log before its take. On a binding
// without the link (no RCLLF_XTAG_MAGIC) the trigger carries only the count,
// as on main before this change, and the reader keeps the count rule.
class XfedTagTest : public DropByTagTest
{
protected:
  void make_link(const std::string & topic)
  {
    make_pair(topic, 10);
    wlog_ = topic + "#writer_process";
    rwriter(pub_);
  }

  /// A publisher of the writer federate: the gate registered it at its
  /// creation, and every link message announces it.
  void rwriter(const Pub & p)
  {
#ifdef RCLLF_XTAG_WRITERS
    tl::on_writer(wlog_, gid_of(p));
#else
    (void)p;
#endif
  }

  void rwrite(const Pub & p, int id)
  {
    tl::on_publish(wlog_, true, true, gid_of(p), ++seq_[p.get()]);
    send_one(p, id);
  }

  void rclaim() {tl::on_claim(wlog_);}

  void routside(const Pub & p, int id)
  {
    tl::on_publish(wlog_, false, true, gid_of(p), ++seq_[p.get()]);
    send_one(p, id);
  }

#ifdef RCLLF_XTAG_MAGIC
  /// The writer federate's pack on `link` at a trigger of `k` writes.
  rcllf_xtag_t pack(int k, const char * link = "data")
  {
    rcllf_xtag_t x;
    ros2_lf_xtag_pack(wlog_.c_str(), link, k, &x);
    return x;
  }

  /// The reader federate's import, then the trigger as reaction_dispatch.cpp runs it.
  Ids arrive(const rcllf_xtag_t & x)
  {
    const int k = ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x);
    return dispatch(k);
  }

  Ids link(int k) {return arrive(pack(k));}
#else
  Ids link(int k) {return dispatch(k);}
#endif

  std::string wlog_;
};

// The cross-federate lone_late case: on the count rule the lone late ID 1 is
// delivered one tag late; with the link it is dropped and reported once.
TEST_F(XfedTagTest, CrossFederateLoneLateSampleIsDroppedNotDeliveredLate)
{
  make_link("/xfed_tag/lone_late");
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), Ids{0});
  rclaim();  // 1 is late
  settle();
  EXPECT_EQ(link(1), Ids{});
  routside(pub_, 1);
  rclaim();  // 2 is late too: 1 is alone at tag 2
  settle();
  EXPECT_EQ(link(1), Ids{}) << "the lone late ID 1 was delivered one tag late";
  routside(pub_, 2);
  rwrite(pub_, 3);
  settle();
  EXPECT_EQ(link(1), Ids{3}) << "tag 3 did not deliver only its own ID 3";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.late_dropped, 2u) << "late samples dropped";
  EXPECT_EQ(reported_, 2u) << "late samples reported to message_lost";
  EXPECT_EQ(snap.untagged, 0u);
}

// late_next_on_time across federates: the late 1 arrives with the on-time 2.
TEST_F(XfedTagTest, CrossFederateLateSampleBeforeAnOnTimeOneIsDropped)
{
  make_link("/xfed_tag/late_next");
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), Ids{0});
  rclaim();
  settle();
  EXPECT_EQ(link(1), Ids{});
  routside(pub_, 1);
  rwrite(pub_, 2);
  settle();
  EXPECT_EQ(link(1), Ids{2}) << "tag 2 delivered the late ID 1";
  EXPECT_EQ(reported_, 1u) << "late samples reported to message_lost";
  EXPECT_EQ(take_stats_snapshot().late_dropped, 1u);
}

#ifdef RCLLF_XTAG_MAGIC
// A sample of the next trigger can be taken before that trigger's events
// arrive: it is held, not delivered early as untagged.
TEST_F(XfedTagTest, SampleAheadOfItsEventsIsHeldForItsTrigger)
{
  make_link("/xfed_tag/ahead");
  rwrite(pub_, 0);
  const auto x0 = pack(1);
  rwrite(pub_, 1);
  const auto x1 = pack(1);
  settle();
  EXPECT_EQ(arrive(x0), Ids{0}) << "ID 1 of the next trigger was delivered early";
  EXPECT_EQ(arrive(x1), Ids{1});
  EXPECT_EQ(take_stats_snapshot().untagged, 0u);
  EXPECT_EQ(reported_, 0u);
}

// An unclaimed publish outside any reaction is untagged: delivered at the
// first take after its writer's events cover it, in addition.
TEST_F(XfedTagTest, UntaggedRemoteSampleIsDeliveredInAddition)
{
  make_link("/xfed_tag/untagged");
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), Ids{0});
  routside(pub_, 50);
  routside(pub_, 51);  // merged with 50 in the journal: both untagged
  rwrite(pub_, 1);
  settle();
  EXPECT_EQ(link(1), (Ids{50, 51, 1}));
  EXPECT_EQ(take_stats_snapshot().untagged, 2u);
  EXPECT_EQ(reported_, 0u);
}

// Two boundary ports of one link apply the same events once.
TEST_F(XfedTagTest, SameEventsImportedTwiceApplyOnce)
{
  make_link("/xfed_tag/twice");
  rwrite(pub_, 0);
  const auto x = pack(1);
  settle();
  EXPECT_EQ(ros2_lf_xtag_import(topic_.c_str(), "w.out -> r2.in", &x), 1);
  EXPECT_EQ(arrive(x), Ids{0});
  rwrite(pub_, 1);
  settle();
  EXPECT_EQ(link(1), Ids{1});
}

// The link's message carries at most RCLLF_XTAG_MAX events and
// RCLLF_XTAG_WRITERS writers (fixed sizes of an LF port struct). When the
// reader cannot apply the writer's events (more events than one message
// carries, a lost message, more writers than the message names, a restarted
// writer), tag-faithful delivery is no longer possible. Before the fix the
// reader warned once and fell back to the count rule, which may deliver a
// sample one tag late; now the run ends with exit code 3 and says why.
TEST_F(XfedTagTest, AnOverflowedLinkEndsTheRun)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
  {
    make_link("/xfed_tag/overflow");
    tl::Gid other{};
    other.fill(0xEE);
    for (int i = 0; i < RCLLF_XTAG_MAX + 1; ++i) {
      rclaim();
      tl::on_publish(wlog_, false, true, other, static_cast<uint64_t>(i + 1));  // fills it
    }
    rwrite(pub_, 0);
    const auto x = pack(RCLLF_XTAG_MAX + 2);
    if (x.overflow != 1u) {
      std::fprintf(stderr, "the message did not overflow\n");
      std::_Exit(4);
    }
    ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x);
    std::fprintf(stderr, "the reader went on with the count rule\n");
    std::_Exit(0);
  }, ::testing::ExitedWithCode(3),
    "tag fidelity lost.*more than " + std::to_string(RCLLF_XTAG_MAX) + " since the link's last");
}

TEST_F(XfedTagTest, ALostLinkMessageEndsTheRun)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
  {
    make_link("/xfed_tag/gap");
    const std::string gap_topic = "/xfed_tag/gap#reader";
    tl::on_publish(wlog_ + "2", true, true, gid_of(pub_), 1);
    rcllf_xtag_t y1;
    ros2_lf_xtag_pack((wlog_ + "2").c_str(), "data", 1, &y1);  // never arrives
    tl::on_publish(wlog_ + "2", true, true, gid_of(pub_), 2);
    rcllf_xtag_t y2;
    ros2_lf_xtag_pack((wlog_ + "2").c_str(), "data", 1, &y2);
    ros2_lf_xtag_import(gap_topic.c_str(), "w.out -> r.in", &y2);
    std::fprintf(stderr, "a gap in the events was not fatal\n");
    std::_Exit(0);
  }, ::testing::ExitedWithCode(3), "tag fidelity lost.*did not all arrive");
}

// The fatal end flushed only stderr before std::_Exit,
// so the lines a node had written to a redirected stdout (a file or a launch
// pipe: fully buffered) were lost, also the last deliveries before the fatal
// message. The child here sends stdout to a fully buffered file, prints a
// delivery line, then imports a link message after a lost one.
TEST_F(XfedTagTest, AFatalLinkEndKeepsTheBufferedStdoutLines)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  // One file per test process: the parent names it after its
  // pid and hands the name to the re-executed threadsafe child through the
  // environment, so concurrent runs of this binary never share it.
  const char * kPathVar = "RCLLF_TEST_XTAG_FATAL_STDOUT";
  if (std::getenv(kPathVar) == nullptr) {
    const std::string own = ::testing::TempDir() + "rcllf_xtag_fatal_stdout_" +
      std::to_string(::getpid()) + ".txt";
    ::setenv(kPathVar, own.c_str(), 0);
  }
  const std::string path = std::getenv(kPathVar);
  const std::string line = "TSR delivered v=41 before the fatal link end";
  std::remove(path.c_str());
  EXPECT_EXIT(
  {
    make_link("/xfed_tag/fatal_stdout");
    if (std::freopen(path.c_str(), "w", stdout) == nullptr) {
      std::fprintf(stderr, "cannot redirect stdout\n");
      std::_Exit(4);
    }
    std::setvbuf(stdout, nullptr, _IOFBF, 1 << 16);
    std::printf("%s\n", line.c_str());
    const std::string gap_topic = "/xfed_tag/fatal_stdout#reader";
    tl::on_publish(wlog_ + "2", true, true, gid_of(pub_), 1);
    rcllf_xtag_t y1;
    ros2_lf_xtag_pack((wlog_ + "2").c_str(), "data", 1, &y1);  // never arrives
    tl::on_publish(wlog_ + "2", true, true, gid_of(pub_), 2);
    rcllf_xtag_t y2;
    ros2_lf_xtag_pack((wlog_ + "2").c_str(), "data", 1, &y2);
    ros2_lf_xtag_import(gap_topic.c_str(), "w.out -> r.in", &y2);
    std::fprintf(stderr, "a gap in the events was not fatal\n");
    std::_Exit(0);
  }, ::testing::ExitedWithCode(3), "tag fidelity lost.*did not all arrive");
  std::ifstream in(path);
  const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  EXPECT_NE(text.find(line), std::string::npos) <<
    "the line written to the redirected stdout before the fatal link end is missing (the "
    "exit dropped the buffered stdout); the file holds '" << text << "'";
  std::remove(path.c_str());
}

TEST_F(XfedTagTest, MoreWritersThanTheLinkNamesEndTheRun)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
  {
    make_link("/xfed_tag/writers");
    for (int i = 0; i <= RCLLF_XTAG_WRITERS; ++i) {
      tl::Gid w{};
      w.fill(static_cast<uint8_t>(0x10 + i));
      tl::on_writer(wlog_, w);  // one more publisher than writer[] holds
    }
    const auto x = pack(RCLLF_XTAG_WRITERS + 1);
    if (x.writers_more != 1u) {
      std::fprintf(stderr, "the message named every writer\n");
      std::_Exit(4);
    }
    ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x);
    std::fprintf(stderr, "the reader went on with the count rule\n");
    std::_Exit(0);
  }, ::testing::ExitedWithCode(3),
    "tag fidelity lost.*more than " + std::to_string(RCLLF_XTAG_WRITERS) + " publishers");
}
// Regression: two publisher nodes of one federate leave on two
// links of one topic. Each link carries the events since its own last
// message, so the reader has them all whichever link it imports first, also
// after one link was silent for more events than one message carries.
TEST_F(XfedTagTest, TwoLinksOfOneTopicCarryTheirEventsInEitherOrder)
{
  make_link("/xfed_tag/two_links");
  auto b = second_publisher();
  wait_publishers(2);
  const auto sorted = [](Ids ids) {std::sort(ids.begin(), ids.end()); return ids;};
  rwrite(pub_, 0);
  rwrite(b, 1);
  const auto xa = pack(1, "data");
  const auto xb = pack(1, "pb_data");
  settle();
  ros2_lf_xtag_import(topic_.c_str(), "pb.data -> sub.data", &xb);
  ros2_lf_xtag_import(topic_.c_str(), "pa.data -> sub.data", &xa);
  EXPECT_FALSE(tl::broken(topic_)) << "the second link's message broke the reader's log";
  EXPECT_EQ(sorted(dispatch(2)), (Ids{0, 1}));
  for (int i = 2; i < RCLLF_XTAG_MAX + 4; ++i) {
    rwrite(pub_, i);
    settle();
    EXPECT_EQ(link(1), Ids{i});
  }
  rwrite(b, 100);
  const auto late = pack(1, "pb_data");
  EXPECT_EQ(late.overflow, 1u);
  settle();
  EXPECT_EQ(arrive(late), Ids{100});
  EXPECT_FALSE(tl::broken(topic_)) << "the silent link's message broke the reader's log";
  EXPECT_EQ(reported_, 0u);
  EXPECT_EQ(take_stats_snapshot().untagged, 0u);
}
// Review: the first sample of a remote writer can be taken before
// any event of that writer arrives. It is held for its trigger, not delivered
// as untagged at the take before.
TEST_F(XfedTagTest, FirstSampleOfAnUnseenRemoteWriterIsHeldForItsTrigger)
{
  make_link("/xfed_tag/unseen_first");
  rclaim();  // tag 1's write is late (a claim, no GID yet)
  const auto x1 = pack(1);
  rwrite(pub_, 1);  // the writer's first sample, of tag 2
  const auto x2 = pack(1);
  settle();
  EXPECT_EQ(arrive(x1), Ids{})
    << "the first remote sample (ID 1 of tag 2) was delivered at tag 1 as untagged";
  EXPECT_EQ(arrive(x2), Ids{1}) << "tag 2 did not deliver its own ID 1";
  EXPECT_EQ(take_stats_snapshot().untagged, 0u) << "the first remote sample counted as untagged";
}

// The same for a second publisher node of the writer federate: its first
// sample comes before the link brings its first event.
TEST_F(XfedTagTest, FirstSampleOfASecondRemotePublisherIsHeldForItsTrigger)
{
  make_link("/xfed_tag/unseen_second");
  auto b = second_publisher();
  rwriter(b);
  wait_publishers(2);
  rwrite(pub_, 0);
  const auto x0 = pack(1);
  rwrite(b, 1);  // b's first sample, of tag 2
  const auto x1 = pack(1);
  settle();
  EXPECT_EQ(arrive(x0), Ids{0})
    << "the second publisher's first sample (ID 1 of tag 2) was delivered at tag 1 as untagged";
  EXPECT_EQ(arrive(x1), Ids{1}) << "tag 2 did not deliver its own ID 1";
  EXPECT_EQ(take_stats_snapshot().untagged, 0u);
}

// Second review: a gated writer is told by its announced GID, not
// by how many triggers passed. The reader runs three triggers behind the
// writer, whose second publisher node's first sample (ID 100, of tag 4) is
// on the wire long before its events: it is held for tag 4, not delivered
// as untagged at the third trigger.
TEST_F(XfedTagTest, GatedWriterWhoseEventsComeAfterThreeTriggersIsPlacedByTag)
{
  make_link("/xfed_tag/gated_slow");
  auto b = second_publisher();
  rwriter(b);
  wait_publishers(2);
  const auto sorted = [](Ids ids) {std::sort(ids.begin(), ids.end()); return ids;};
  std::vector<rcllf_xtag_t> x;
  for (int i = 0; i < 3; ++i) {
    rwrite(pub_, i);
    x.push_back(pack(1));
  }
  rwrite(b, 100);
  rwrite(pub_, 3);
  x.push_back(pack(2));
  settle();
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(arrive(x[i]), Ids{i})
      << "trigger " << i + 1 << " did not deliver only its own ID " << i
      << " (the gated ID 100 of tag 4 delivered early as untagged)";
  }
  EXPECT_EQ(sorted(arrive(x[3])), (Ids{3, 100})) << "tag 4 did not deliver its own IDs 3, 100";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 0u) << "the gated ID 100 counted as untagged";
  EXPECT_EQ(snap.unsettled, 0u);
  EXPECT_EQ(reported_, 0u);
}

// A writer outside the publish gate (a plain ROS node) is in no announced
// set: once the writer federate announced, its sample goes out untagged at
// the first take, with no further trigger.
TEST_F(XfedTagTest, PlainWriterSampleIsDeliveredAtTheFirstTakeAfterTheAnnouncement)
{
  make_link("/xfed_tag/plain_after");
  auto b = second_publisher();  // never announced
  wait_publishers(2);
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), Ids{0});
  send_one(b, 7);  // no tag log event
  rwrite(pub_, 1);
  settle();
  EXPECT_EQ(link(1), (Ids{7, 1})) << "the plain writer's ID 7 was not delivered at the first take";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 1u);
  EXPECT_EQ(snap.unsettled, 0u);
  EXPECT_EQ(reported_, 0u);
}

// A plain sample that lands before the writer federate's announcement is
// held only until the announcement: the take of the announcing trigger
// delivers it.
TEST_F(XfedTagTest, PlainSampleBeforeTheAnnouncementIsDeliveredWhenItLands)
{
  make_link("/xfed_tag/plain_before");
  auto b = second_publisher();  // never announced
  wait_publishers(2);
#ifdef RCLLF_XTAG_WRITERS
  ros2_lf_xtag_expect(topic_.c_str(), 1);  // the receiving federate's startup
  EXPECT_EQ(tl::writer(topic_, gid_of(b)), tl::Writer::kUnknown)
    << "a writer was classified before any announcement arrived";
#endif
  send_one(b, 7);  // no tag log event
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), (Ids{7, 0}))
    << "the plain writer's ID 7 was not delivered with the announcing trigger";
#ifdef RCLLF_XTAG_WRITERS
  EXPECT_EQ(tl::writer(topic_, gid_of(b)), tl::Writer::kPlain);
  EXPECT_EQ(tl::writer(topic_, gid_of(pub_)), tl::Writer::kGated);
#endif
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 1u);
  EXPECT_EQ(snap.unsettled, 0u);
}

#ifdef RCLLF_XTAG_WRITERS
// A publisher the writer federate created after its earlier messages: the
// reader took its sample for a plain writer's. The later announcement makes
// that visible (one WARN at the import), and its samples are gated from then on.
TEST_F(XfedTagTest, PublisherAnnouncedAfterItsSampleWentOutPlainIsReported)
{
  make_link("/xfed_tag/late_writer");
  auto b = second_publisher();
  wait_publishers(2);
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), Ids{0});
  send_one(b, 7);  // b's write before the writer federate announced it
  ++seq_[b.get()];  // its DDS sequence number, as the gate would count it
  rwrite(pub_, 1);
  settle();
  EXPECT_EQ(link(1), (Ids{7, 1}));
  rwriter(b);
  rwrite(b, 2);
  const auto x = pack(1);
  EXPECT_EQ(x.nw, 2u);
  const std::vector<tl::Gid> writers = {gid_of(pub_), gid_of(b)};
  tl::import(topic_, x.session, x.first, nullptr, 0, writers.data(), writers.size());
  EXPECT_TRUE(tl::late_writer(topic_)) << "the late announcement of a plain writer was not seen";
  EXPECT_FALSE(tl::late_writer(topic_)) << "reported twice";
  EXPECT_EQ(tl::writer(topic_, gid_of(b)), tl::Writer::kGated);
  settle();
  EXPECT_EQ(arrive(x), Ids{2});
}

// Third review, item 1: one writer federate per crossing topic, so
// the expectation counts gated publishers, not writer federates. Two
// announced publishers, a third writer outside rcllf: its sample goes out
// untagged at the first take.
TEST_F(XfedTagTest, PlainWriterIsToldOnceEveryExpectedPublisherIsAnnounced)
{
  make_link("/xfed_tag/two_expected");
  auto b = second_publisher();
  auto c = second_publisher();  // never announced: a plain ROS writer
  rwriter(b);
  wait_publishers(3);
  ros2_lf_xtag_expect(topic_.c_str(), 2);  // the generator counted pub_ and b
  send_one(c, 7);
  rwrite(pub_, 0);
  settle();
  EXPECT_EQ(link(1), (Ids{7, 0}))
    << "the plain writer's ID 7 was not delivered at the first take once both expected "
    "publishers were announced";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 1u);
  EXPECT_EQ(snap.unsettled, 0u) << "the plain sample still waits for another writer federate";
}

// Third review, item 3: a gated publisher created after the
// writer federate's earlier messages (a lifecycle publisher made at
// configure) is in none of them. The generator counts it, so its first
// sample, taken by the earlier trigger, is held for its own tag instead of
// going out untagged a tag early.
// The generator's expectation for two publisher nodes of one federate
// (test_xtag_link.py, test_two_publishers_of_one_federate_pack_per_link).
constexpr int kLatePublisherExpect = 2;

TEST_F(XfedTagTest, PublisherCreatedAfterEarlierMessagesIsHeldNotTakenPlain)
{
  make_link("/xfed_tag/late_publisher");
  auto b = second_publisher();
  wait_publishers(2);
  ros2_lf_xtag_expect(topic_.c_str(), kLatePublisherExpect);
  rwrite(pub_, 0);
  const auto x1 = pack(1);  // announces pub_ only
  rwriter(b);  // b's publisher is created at tag 2
  rwrite(b, 100);
  const auto x2 = pack(1);
  settle();
  EXPECT_EQ(arrive(x1), Ids{0})
    << "the late publisher's ID 100 of tag 2 was delivered at tag 1 as untagged";
  EXPECT_EQ(arrive(x2), Ids{100}) << "tag 2 did not deliver its own ID 100";
  const auto snap = take_stats_snapshot();
  EXPECT_EQ(snap.untagged, 0u) << "the late gated publisher's sample counted as untagged";
  EXPECT_EQ(snap.unsettled, 0u);
  EXPECT_EQ(reported_, 0u);
}
#endif

// Third review, item 2: a take on a crossing topic before its
// link's first import (not generated: the import reaction triggers the take)
// holds its samples and warns once naming the topic. It does not put the
// topic on the count rule for good: once imported, the next trigger delivers
// its own sample by tag, and the early triggers' samples are dropped as late.
TEST_F(XfedTagTest, TakeBeforeTheFirstImportHoldsItsSamplesAndWarnsOnce)
{
  make_link("/xfed_tag/before_import");
  ros2_lf_xtag_expect(topic_.c_str(), 1);
  std::vector<rcllf_xtag_t> x;
  for (int i = 0; i < 3; ++i) {
    rwrite(pub_, i);
    x.push_back(pack(1));
  }
  settle();
  g_logged.clear();
  const auto previous = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capture_log);
  const Ids early1 = dispatch(1);  // the takes of tags 1 and 2, before any import
  const Ids early2 = dispatch(1);
  rcutils_logging_set_output_handler(previous);
  EXPECT_EQ(early1, Ids{}) << "a take before the first import delivered by the count rule";
  EXPECT_EQ(early2, Ids{}) << "a take before the first import delivered by the count rule";
  size_t warns = 0;
  for (const auto & l : g_logged) {
    if (l.find("before the cross-federate take link's first message") != std::string::npos &&
      l.find(topic_) != std::string::npos)
    {
      ++warns;
    }
  }
  EXPECT_EQ(warns, 1u) << "no single WARN named the topic taken before its first import";
  ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x[0]);
  ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x[1]);
  EXPECT_FALSE(tl::broken(topic_));
  EXPECT_EQ(arrive(x[2]), Ids{2})
    << "tag 3 did not deliver its own ID 2 by tag (the reader stayed on the count rule)";
  EXPECT_EQ(reported_, 2u) << "the early triggers' IDs 0 and 1 were not dropped as late";
  EXPECT_EQ(take_stats_snapshot().unsettled, 0u);
}

// Review: a restarted writer numbers its journal from 0 under a
// new session. Read as the old session its events would be skipped as
// applied already. The tag log breaks the topic, and the
// link import ends the run instead of going on with the count rule.
TEST_F(XfedTagTest, ChangedWriterSessionEndsTheRun)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
  {
    make_link("/xfed_tag/session");
    rwrite(pub_, 0);
    settle();
    if (link(1) != Ids{0}) {
      std::fprintf(stderr, "the first sample was not delivered\n");
      std::_Exit(4);
    }
    rwrite(pub_, 1);
    auto x = pack(1);
    x.session += 1;  // the writer process restarted
    x.first = 0;
    settle();
    ros2_lf_xtag_import(topic_.c_str(), "w.out -> r.in", &x);
    std::fprintf(stderr, "a changed writer session was read as duplicate events\n");
    std::_Exit(0);
  }, ::testing::ExitedWithCode(3), "tag fidelity lost.*session changed");
}
#endif  // RCLLF_XTAG_MAGIC

// Review round 2, item 3: a sample held for a later trigger is delivered when
// that trigger falls to the count rule, not stranded and later dropped.
TEST_F(DropByTagTest, HeldSampleIsDeliveredWhenItsTriggerFallsToTheCountRule)
{
  make_pair("/drop_by_tag/held_count_rule", 10);
  auto b = second_publisher();
  wait_publishers(2);
  write(pub_, 0);
  write(pub_, 1);  // one tag ahead: held by the take of 0
  settle();
  EXPECT_EQ(dispatch(1), Ids{0});
  tl::on_publish(topic_, true, false, {}, 0);  // tag 1's second write: no GID
  send_one(b, 100);
  settle();
  EXPECT_EQ(dispatch(2), (Ids{1, 100})) << "the held on-time ID 1 was not delivered";
  write(pub_, 2);
  settle();
  EXPECT_EQ(dispatch(1), Ids{2});
  EXPECT_EQ(reported_, 0u);
  EXPECT_EQ(take_stats_snapshot().late_dropped, 0u);
}

// A trigger whose writes are not all recorded (here a write
// without a GID) is served by the count rule, which may deliver a late sample
// one tag late. Before the fix the reader warned once and the run summary
// said nothing, so a run could change rules silently; now take_stats counts
// each such take per topic and the shutdown summary names them.
TEST_F(DropByTagTest, ACountRuleTakeIsNamedInTheRunSummary)
{
  make_pair("/drop_by_tag/count_rule_summary", 10);
  auto b = second_publisher();
  wait_publishers(2);
  write(pub_, 0);
  settle();
  EXPECT_EQ(dispatch(1), Ids{0});
  tl::on_publish(topic_, true, false, {}, 0);  // tag 1's second write: no GID
  write(pub_, 1);
  send_one(b, 100);
  settle();
  EXPECT_EQ(dispatch(2).size(), 2u);
  std::string topic_line;
  for (const auto & l : logged_take_stats()) {
    if (l.find("ros2_lf_take_stats_topic: topic=" + topic_ + " ") != std::string::npos) {
      topic_line = l;
    }
  }
  EXPECT_NE(topic_line.find(" count_rule=2"), std::string::npos)
    << "the run summary does not count the 2 takes the count rule served: " << topic_line;
  const auto warned = warnings_while([] {take_stats_log_summary("shutdown");});
  bool named = false;
  for (const auto & w : warned) {
    named = named || w.find("2 take(s) were served by the count rule") != std::string::npos;
  }
  EXPECT_TRUE(named) << "the shutdown summary does not warn that the run changed rules";
}

// Review round 2, item 4: a trigger of unknown write count (ros2_lf_invoke_sub)
// owns records the reader cannot count, so the reader stays on the count rule
// instead of reading a record of an earlier trigger as its own.
TEST_F(DropByTagTest, UnknownCountTriggerKeepsTheReaderOnTheCountRule)
{
  make_pair("/drop_by_tag/unknown_count", 10);
  write(pub_, 0);
  settle();
  EXPECT_EQ(dispatch(1), Ids{0});
  write(pub_, 1);
  settle();
  EXPECT_EQ(dispatch(0), Ids{1});
  for (int id = 2; id <= 3; ++id) {
    write(pub_, id);
    settle();
    EXPECT_EQ(dispatch(1), Ids{id}) << "tag " << id << " read the record of an earlier trigger";
  }
  EXPECT_EQ(reported_, 0u);
  EXPECT_EQ(take_stats_snapshot().late_dropped, 0u);
}

// Review round 2, items 3 and 4: the unknown-count trigger first gets the
// sample an earlier take held, as the count rule would.
TEST_F(DropByTagTest, UnknownCountTriggerTakesTheHeldSampleFirst)
{
  make_pair("/drop_by_tag/unknown_count_held", 10);
  write(pub_, 0);
  write(pub_, 1);  // one tag ahead: held by the take of 0
  settle();
  EXPECT_EQ(dispatch(1), Ids{0});
  EXPECT_EQ(dispatch(0), Ids{1}) << "the held ID 1 was not delivered at its own tag";
  write(pub_, 2);
  settle();
  EXPECT_EQ(dispatch(1), Ids{2});
  EXPECT_EQ(reported_, 0u);
  EXPECT_EQ(take_stats_snapshot().late_dropped, 0u);
}

// The message_lost schedule of the two tests below. `late_ms` delays the
// transport delivery of the late ID 1: its record is made at once (the claimed
// publish), its send waits `late_ms` on another thread.
void DropByTagTest::message_lost_schedule(const std::string & topic, int late_ms)
{
  topic_ = topic;
  const auto qos = rclcpp::QoS(10).reliable();
  std::vector<int> delivered;
  std::vector<std::pair<size_t, size_t>> lost;
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.message_lost_callback = [&lost](rclcpp::QOSMessageLostInfo & info) {
      lost.emplace_back(info.total_count, info.total_count_change);
    };
  pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic_, qos);
  sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    topic_, qos, [&delivered](rosgraph_msgs::msg::Clock::SharedPtr m) {
      delivered.push_back(m->clock.sec);
    }, options);
  wait_publishers(1);
  ros2_lf_binding::g_handle_table.clear();
  ros2_lf_binding::g_handle_table.resize(1);
  auto & e = ros2_lf_binding::g_handle_table[0];
  e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  e.subscription = sub_;
  e.topic = topic_;
  e.node_class = "TakeDebtNode";
  e.callback_name = "on_clock";
  e.take_mode = TakeMode::Skip;
  ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto invoke = [&]() {return ros2_lf_invoke_sub_n(nullptr, 0, topic_.c_str(), 1);};
  // Wait until the reader holds a sample instead of sleeping 200 ms. A
  // late sample still in flight at the take of step 3 is not dropped there,
  // so the handler saw both drops in one report. Every take empties the
  // reader, so ready means this step's first sample has landed; the writer's
  // later one is ordered after it (reliable, one writer). The bound only ends
  // a hung test: a tenth of the binary's ctest TIMEOUT of 300 s.
  rclcpp::WaitSet landed;
  landed.add_subscription(sub_);
  auto arrived = [&landed]() {
      return landed.wait(std::chrono::seconds(30)).kind() == rclcpp::WaitResultKind::Ready;
    };

  write(pub_, 0);
  ASSERT_TRUE(arrived()) << "ID 0 did not reach the reader within 30 s";
  EXPECT_EQ(invoke(), 1);
  claim();
  EXPECT_EQ(invoke(), 0);
  std::thread late_send;
  if (late_ms > 0) {
    tl::on_publish(topic_, false, true, gid_of(pub_), ++seq_[pub_.get()]);
    late_send = std::thread([this, late_ms]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(late_ms));
          send_one(pub_, 1);
        });
  } else {
    outside(pub_, 1);
  }
  claim();
  const bool late_landed = arrived();
  // Joined before the assertion: a failed ASSERT returns, and a joinable
  // std::thread destroyed then would terminate the whole binary.
  if (late_send.joinable()) {
    late_send.join();
  }
  ASSERT_TRUE(late_landed) << "the late ID 1 did not reach the reader within 30 s";
  EXPECT_EQ(invoke(), 0) << "the lone late ID 1 ran the reaction one tag late";
  outside(pub_, 2);
  write(pub_, 3);
  ASSERT_TRUE(arrived()) << "ID 2 did not reach the reader within 30 s";
  // Ready proves ID 2 landed, not ID 3: the take waits for the counted ID 3
  // up to the reader's take timeout (100 ms by default), so give it the same
  // 30 s bound; it returns as soon as ID 3 is taken.
  e.take_timeout_ms = 30000;
  EXPECT_EQ(invoke(), 1);
  landed.remove_subscription(sub_);
  ros2_lf_binding::g_handle_table.clear();

  EXPECT_EQ(delivered, (Ids{0, 3}));
  ASSERT_EQ(lost.size(), 2u) << "message_lost did not receive both dropped late samples";
  EXPECT_EQ(lost[0], (std::pair<size_t, size_t>{1u, 1u}));
  EXPECT_EQ(lost[1], (std::pair<size_t, size_t>{2u, 1u}));
}
TEST_F(DropByTagTest, LateDropReachesTheMessageLostHandlerThroughInvokeSub)
{
  message_lost_schedule("/drop_by_tag/message_lost", 0);
}

// The falsification of the wait. The late ID 1 lands 500 ms
// after its claimed publish, past the old fixed 200 ms sleep and the 100 ms
// take that followed it. With the sleep, the take of step 2 found nothing, ID 1
// landed afterwards, and step 3 dropped IDs 1 and 2 in one report: the handler
// saw one call (2, 2), "message_lost did not receive both dropped late
// samples". The wait for the reader holds step 2 until ID 1 has landed.
TEST_F(DropByTagTest, LateDropReachesTheMessageLostHandlerWhenDeliveryIsDelayed)
{
  message_lost_schedule("/drop_by_tag/message_lost_delayed", 500);
}
#endif  // ROS2_LF_TAKE_BY_TAG

// ── Differential test against a reference model of drop by tag
//
// Seeded schedules of 1 to 3 writers. At each tag every writer makes 0 to 2
// writes (one each in the tail): on time (recorded and published in the
// reaction), claimed and published outside the reaction before the take
// (on time), claimed and published 1 to 3 tags late, or claimed and lost.
// Now and then a writer publishes outside any reaction with no claim of its
// own. A publish outside a reaction fills the newest open claim, so a sample's
// tag is the tag of the claim it fills, or none (untagged). The reference
// model below states the rule per tag, not per take: a trigger delivers every
// arrived sample of its own tag and every untagged one, and drops and reports
// every arrived sample of an earlier tag. The take path must match it at every
// trigger and hold the three properties: no sample of an earlier tag
// delivered, no on-time sample dropped, every sample not delivered reported.
// Built without ROS2_LF_TAKE_BY_TAG the same schedules run through the count
// rule (take_with_mode, one take per write).

namespace
{
struct TagFuzzSample
{
  int id;
  int writer;
  int tag;     // tag of its write (its claim, or of the in-reaction publish)
  int bound;   // tag of the claim it fills, -1 untagged
  int due;     // tag before whose take it is published
};

enum class WriteKind {OnTime, ClaimOnTime, Late, Lost};
}  // namespace

TEST_F(TakeDebtTest, DifferentialAgainstADropByTagModel)
{
  constexpr uint32_t kSchedules = 1000;
  // Every sample of a slot is settled before its takes, so a miss is final:
  // poll 1 ms instead of 100 ms per miss (thousands of misses per run).
  timeout_ms_ = 1;
  constexpr int kTagTail = 4;
  const char * log_path = std::getenv("TAKE_TAG_FUZZ_LOG");
  std::ofstream log;
  if (log_path != nullptr) {
    log.open(log_path);
  }
  int writers3 = 0, lates = 0, losses = 0, extras = 0, claimed_on_time = 0, drops = 0;
  int earlier_tag = 0, on_time_dropped = 0, unreported = 0;
  std::vector<std::string> mismatches;
  for (uint32_t seed = 1; seed <= kSchedules; ++seed) {
    take_stats_reset();
    reported_ = 0;
    std::mt19937 rng(seed);
    auto pick = [&rng](uint32_t n) {return static_cast<int>(rng() % n);};
    const int nw = 1 + pick(3);
    const int main_tags = 8 + pick(7);
    const int tags = main_tags + kTagTail;
    writers3 += nw == 3;
    topic_ = "/drop_by_tag_fuzz/s" + std::to_string(seed);
    const auto qos = rclcpp::QoS(100).reliable();
    sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic_, qos, [](rosgraph_msgs::msg::Clock::SharedPtr) {});
    auto mirror = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      topic_, qos, [](rosgraph_msgs::msg::Clock::SharedPtr) {});
    std::vector<rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr> pubs(nw);
    for (auto & p : pubs) {
      p = node_->create_publisher<rosgraph_msgs::msg::Clock>(topic_, qos);
    }
    {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      bool ok = false;
      while (!ok && std::chrono::steady_clock::now() < deadline) {
        ok = sub_->get_publisher_count() == static_cast<size_t>(nw) &&
          mirror->get_publisher_count() == static_cast<size_t>(nw);
        for (const auto & p : pubs) {
          ok = ok && p->get_subscription_count() == 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      ASSERT_TRUE(ok) << "seed " << seed << ": pub/sub did not match";
    }
#ifdef ROS2_LF_TAKE_BY_TAG
    std::vector<uint64_t> seq(nw, 0);
#endif
    std::vector<TagFuzzSample> samples;
    std::vector<int> open_claims;  // tags of unfilled claims, oldest first
    std::vector<std::vector<int>> late_at(tags);  // ids published before the take of tag t
    std::vector<int> pending;  // published since the last trigger
    size_t published = 0, mirrored = 0;
    int next_id = 0;
    auto put = [&](int w, int id, bool in_reaction) {
        rosgraph_msgs::msg::Clock m;
        m.clock.sec = id;
#ifdef ROS2_LF_TAKE_BY_TAG
        ros2_lf_binding::tag_log::Gid g{};
        std::copy_n(pubs[w]->get_gid().data, g.size(), g.begin());
        ros2_lf_binding::tag_log::on_publish(topic_, in_reaction, true, g, ++seq[w]);
#endif
        if (!in_reaction) {
          int bound = -1;
          if (!open_claims.empty()) {
            bound = open_claims.back();
            open_claims.pop_back();
          }
          samples[id].bound = bound;
        }
        pubs[w]->publish(m);
        pending.push_back(id);
        ++published;
      };
    auto add = [&](int w, int tag, int bound, int due) {
        samples.push_back({next_id, w, tag, bound, due});
        return next_id++;
      };
    std::vector<int> got_all;
    std::string why;
    for (int t = 0; t < tags; ++t) {
      // Before the reactions: late samples now due, and an untagged extra.
      for (int id : late_at[t]) {
        put(samples[id].writer, id, false);
      }
      if (t < main_tags && pick(8) == 0) {
        const int w = pick(nw);
        put(w, add(w, -1, -1, t), false);
        ++extras;
      }
      // The reactions at tag t.
      int k = 0;
      std::vector<int> claimed_now;
      for (int w = 0; w < nw; ++w) {
        const int n = t < main_tags ? pick(3) : 1;
        for (int i = 0; i < n; ++i) {
          WriteKind kind = WriteKind::OnTime;
          if (t < main_tags) {
            const int r = pick(20);
            kind = r < 4 ? WriteKind::Late : r < 6 ? WriteKind::Lost :
              r < 8 ? WriteKind::ClaimOnTime : WriteKind::OnTime;
          }
          ++k;
          if (kind == WriteKind::OnTime) {
            put(w, add(w, t, t, t), true);
            continue;
          }
#ifdef ROS2_LF_TAKE_BY_TAG
          ros2_lf_binding::tag_log::on_claim(topic_);
#endif
          open_claims.push_back(t);
          if (kind == WriteKind::Lost) {
            ++losses;
          } else if (kind == WriteKind::Late) {
            const int due = t + 1 + pick(3);
            late_at[due].push_back(add(w, t, -1, due));
            ++lates;
          } else {
            claimed_now.push_back(add(w, t, -1, t));
            ++claimed_on_time;
          }
        }
      }
      for (int id : claimed_now) {
        put(samples[id].writer, id, false);
      }
      // Wait until the samples arrived (the mirror reads the same writers).
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (mirrored < published && std::chrono::steady_clock::now() < deadline) {
        rosgraph_msgs::msg::Clock m;
        rclcpp::MessageInfo info;
        if (mirror->take_type_erased(&m, info)) {
          ++mirrored;
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
      }
      ASSERT_EQ(mirrored, published) << "seed " << seed << ": a sample did not arrive";
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (k == 0) {
        continue;  // no write, no trigger
      }
      // The take path: one trigger of k writes.
      std::vector<int> got;
      for (int i = k; i >= 1; --i) {
#ifdef ROS2_LF_TAKE_BY_TAG
        auto tt = ros2_lf_binding::take_by_tag(kRid, *sub_, TakeMode::Skip, timeout_ms_, topic_, i);
        if (tt.count_rule) {
          why += " tag " + std::to_string(t) + " fell back to the count rule;";
        }
        reported_ += tt.lost;
        for (const auto & s : tt.deliver) {
          rosgraph_msgs::msg::Clock m;
          if (ros2_lf_binding::deserialize_taken(*sub_, s, &m)) {
            got.push_back(m.clock.sec);
          }
        }
#else
        const int v = take(i);
        if (v >= 0) {
          got.push_back(v);
        }
#endif
      }
      // The reference model.
      std::vector<int> want;
      for (int id : pending) {
        if (samples[id].bound == t || samples[id].bound < 0) {
          want.push_back(id);
        }
      }
      for (int id : got) {
        const auto & x = samples.at(id);
        if (x.bound >= 0 && x.bound < t) {
          ++earlier_tag;
          why += " tag " + std::to_string(t) + " delivered ID " + std::to_string(id) +
            " of the earlier tag " + std::to_string(x.bound) + ";";
        }
      }
      for (int id : pending) {
        if (samples[id].bound == t && std::find(got.begin(), got.end(), id) == got.end()) {
          ++on_time_dropped;
          why += " tag " + std::to_string(t) + " did not deliver its on-time ID " +
            std::to_string(id) + ";";
        }
      }
      std::sort(got.begin(), got.end());
      std::sort(want.begin(), want.end());
      if (got != want && why.find("model") == std::string::npos) {
        std::string g, m;
        for (int id : got) {
          g += " " + std::to_string(id);
        }
        for (int id : want) {
          m += " " + std::to_string(id);
        }
        why += " model first differs at tag " + std::to_string(t) + ": got [" + g + " ], want [" +
          m + " ];";
      }
      got_all.insert(got_all.end(), got.begin(), got.end());
      if (log.is_open()) {
        log << "seed " << seed << " tag " << t << " k " << k << " got";
        for (int id : got) {
          log << ' ' << id;
        }
        log << " want";
        for (int id : want) {
          log << ' ' << id;
        }
        log << '\n';
      }
      pending.clear();
    }
    ASSERT_TRUE(pending.empty()) << "seed " << seed << ": a sample arrived after the last tag";
    const auto snap = take_stats_snapshot();
    const auto it = snap.topics.find(topic_);
    const uint64_t late = it == snap.topics.end() ? 0 : it->second.late_dropped;
    drops += static_cast<int>(late);
    std::set<int> unique(got_all.begin(), got_all.end());
    if (unique.size() != got_all.size()) {
      why += " a sample was delivered twice;";
    }
    if (reported_ != late || published != got_all.size() + late) {
      ++unreported;
      why += " published " + std::to_string(published) + ", delivered " +
        std::to_string(got_all.size()) + ", late_dropped " + std::to_string(late) +
        ", reported " + std::to_string(reported_) + ": a sample was dropped without a report;";
    }
    if (!why.empty()) {
      mismatches.push_back("seed " + std::to_string(seed) + ":" + why);
    }
    sub_.reset();
    mirror.reset();
    pubs.clear();
  }
  std::printf(
    "[tag differential] schedules=%u (3 writers: %d; late: %d, lost: %d, claimed on time: %d, "
    "untagged extras: %d; late drops: %d)\n"
    "[tag differential] earlier-tag deliveries: %d, on-time samples dropped: %d, schedules "
    "with an unreported drop: %d, schedules that differ from the model: %zu\n",
    kSchedules, writers3, lates, losses, claimed_on_time, extras, drops, earlier_tag,
    on_time_dropped, unreported, mismatches.size());
  std::string first;
  for (size_t i = 0; i < mismatches.size() && i < 3; ++i) {
    first += "\n  " + mismatches[i];
  }
  EXPECT_TRUE(mismatches.empty())
    << mismatches.size() << " schedule(s) break drop by tag; first:" << first;
}

// A fast federate's table judges
// cutoffs on its logical tag. The clock the runtime installs reads the tag in a
// fast federate only, from the peer check on; real time and the one program
// keep the physical clock.
TEST(FastFederateClock, InstalledClockReadsTheLogicalTagInAFastFederate)
{
  namespace rt = ros2_lf_binding::corr_rt;
  constexpr int64_t kTag = 1793241232682377065;  // start + 5 ms, far from now
  rt::reset_for_test();
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(kTag, 0u));
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_FALSE(rt::fast_federate()) << "the one program in fast mode is not a federate";
  EXPECT_NE(rt::table_clock(), kTag) << "the one program keeps the physical clock";
  rt::set_federated(true);
  EXPECT_TRUE(rt::fast_federate());
  EXPECT_EQ(rt::table_clock(), kTag)
    << "a fast federate's table clock must be its logical tag, not lf_time_physical";
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{-1}, 0u));
  EXPECT_EQ(rt::table_clock(), 0) << "outside LF (tag -1) the logical clock reads 0";
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(kTag, 0u));
  ros2_lf_binding::set_lf_fast_for_test(false);
  EXPECT_FALSE(rt::fast_federate());
  EXPECT_NE(rt::table_clock(), kTag) << "a real-time federate keeps the physical clock";
  ros2_lf_binding::set_lf_fast_for_test(true);
  rt::reset_for_test();
  EXPECT_FALSE(rt::fast_federate()) << "reset_for_test clears the federate flag";
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
}

// The arming of the stalled
// sender watch (corr_runtime.cpp, stall_watch_start_impl) had no test that
// fails when it breaks. Each scenario runs in its own process (threadsafe death
// test style), because the watch is a once-only process singleton. The
// receiver has one ingress edge (d = 0, T = 2 ms, lag 2 T) and a 5 ms timer, so
// W = 5 + 0 + 2 + 4 = 11 ms; nothing dispatches after the start, so a healthy
// watch reports one stall about 11 ms after the start time.
extern "C" int64_t lf_time_physical(void);

namespace
{

enum class TimerBound {BeforeStart, AfterStart};

[[noreturn]] void stall_arming_scenario(TimerBound when)
{
  namespace rt = ros2_lf_binding::corr_rt;
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("stall_arming_test_node");
  auto timer = node->create_wall_timer(std::chrono::milliseconds(5), [] {});
  rt::reset_for_test();
  rt::set_federated(true);
  rt::table().add_edge(
    0, 0, 0, 2000000, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  auto bind_timer = [&] {
      ros2_lf_binding::g_handle_table.clear();
      ros2_lf_binding::g_handle_table.resize(1);
      auto & e = ros2_lf_binding::g_handle_table[0];
      e.kind = ros2_lf_binding::HandleKind::TIMER;
      e.timer = timer;
    };
  const int64_t start = lf_time_physical();
  if (when == TimerBound::BeforeStart) {
    bind_timer();  // a plain node binds its timers at loading, before the start
    rt::stall_watch_start(start);
  } else {
    // A lifecycle node binds its timers on activation, after the RTI start;
    // its first dispatch at (start, 0) arms the watch.
    rt::stall_watch_start(start);
    bind_timer();
    rt::stall_progress(start, 0);
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    if (rt::table().stalls() > 0) {
      std::_Exit(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::fprintf(
    stderr, "%s\n", when == TimerBound::BeforeStart ?
    "no stall reported 500 ms after the start time: stall_watch_start did not arm the "
    "watch of a federate whose timer was bound before the start" :
    "no stall reported 500 ms after the start time: a timer bound after stall_watch_start "
    "never armed the watch, the start used up the once-only arming without a timer "
    "(lifecycle guard)");
  std::_Exit(1);
}

}  // namespace

TEST(StallArming, WatchArmsFromTheStartTimeWithATimerBoundBeforeIt)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(stall_arming_scenario(TimerBound::BeforeStart), ::testing::ExitedWithCode(0), "");
}

TEST(StallArming, LifecycleTimerBoundAfterTheStartStillArmsTheWatch)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(stall_arming_scenario(TimerBound::AfterStart), ::testing::ExitedWithCode(0), "");
}

// The stop-due backstop of a
// timer-free federate (corr_runtime.cpp, stall_backstop) had only a runtime
// matrix as evidence. A federate without a timer has no stall bound, so only
// the backstop can report it, once physical time passes stop + W with LF still
// short of its stop tag. The receiver has one ingress edge (d = 0, T = 2 ms,
// L = 100 ms), so W = 2 + 100 = 102 ms; the stop tag is 100 ms after the start.
// The backstop polls every 10 ms, so a report can come late, never early.
extern "C" {
#include "core/environment.h"
}
extern environment_t * g_lf_environments_stub;  // lf_environments_stub.cpp

namespace
{

enum class StopReached {No, Yes};

[[noreturn]] void stall_backstop_scenario(StopReached reached)
{
  namespace rt = ros2_lf_binding::corr_rt;
  constexpr int64_t kStopAfterNs = 100000000;  // 100 ms
  constexpr int64_t kWindowNs = 102000000;  // T + L
  rt::reset_for_test();
  rt::set_federated(true);
  rt::table().add_edge(
    0, 0, 0, 2000000, 100000000, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  if (rt::table().stall_window_ns() != kWindowNs) {
    std::fprintf(
      stderr, "stall window %lld ns, expected %lld\n",
      static_cast<long long>(rt::table().stall_window_ns()),  // NOLINT(runtime/int)
      static_cast<long long>(kWindowNs));  // NOLINT(runtime/int)
    std::_Exit(4);
  }
  const int64_t start = lf_time_physical();
  static environment_t env{};
  env.start_tag.time = start;
  env.stop_tag.time = start + kStopAfterNs;
  // LF stands at its start tag (an upstream federate stopped granting), or
  // has reached its stop tag and is shutting down.
  env.current_tag.time = reached == StopReached::Yes ? env.stop_tag.time : start;
  g_lf_environments_stub = &env;
  rt::stall_watch_start(start);  // no timer bound: only the backstop can report
  const int64_t stop = env.stop_tag.time;
  // The stall came after the last poll that saw none (last_clean, read before
  // that poll's stalls(), ns past the stop tag) and before the time read after
  // the poll that sees it: a late poll alone cannot tell an early report from a
  // due one.
  int64_t last_clean = start - stop;
  while (lf_time_physical() <= stop) {
    const int64_t polled = lf_time_physical() - stop;
    if (rt::table().stalls() > 0) {
      std::fprintf(
        stderr, "a stall was reported before the stop tag: the backstop does not wait for "
        "stop + W\n");
      std::_Exit(2);
    }
    last_clean = polled;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    const int64_t polled = lf_time_physical() - stop;
    if (rt::table().stalls() > 0) {
      const int64_t past = lf_time_physical() - stop;  // after the read: an upper bound
      if (reached == StopReached::No && past <= kWindowNs) {
        std::fprintf(
          stderr, "a stall was reported %lld ns past the stop tag, inside the %lld ns "
          "window: the backstop does not wait for stop + W\n",
          static_cast<long long>(past), static_cast<long long>(kWindowNs));  // NOLINT(runtime/int)
        std::_Exit(2);
      }
      if (reached == StopReached::Yes) {
        std::fprintf(
          stderr, "a stall was reported after LF reached its stop tag: the backstop "
          "judged a federate that is shutting down\n");
        std::_Exit(3);
      }
      constexpr int64_t kPollGapNs = 20000000;  // 20 ms
      if (last_clean <= kWindowNs && past - last_clean > kPollGapNs) {
        std::fprintf(
          stderr, "inconclusive: no poll between %lld and %lld ns past the stop tag, "
          "across the %lld ns window: the stall may have come inside it (test thread "
          "descheduled)\n",
          static_cast<long long>(last_clean), static_cast<long long>(past),  // NOLINT(runtime/int)
          static_cast<long long>(kWindowNs));  // NOLINT(runtime/int)
        std::_Exit(5);
      }
      std::_Exit(0);
    }
    last_clean = polled;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (reached == StopReached::Yes) {
    std::_Exit(0);
  }
  std::fprintf(
    stderr, "no stall reported 500 ms past the stop tag of a timer-free federate with a "
    "102 ms window: the stop-due backstop did not run\n");
  std::_Exit(1);
}

// Exit 5 says the run could not judge the backstop: the test thread missed its
// polls for more than 20 ms across stop + W, so the stall may have come inside
// the window. A loaded runner causes that while the backstop is correct (the
// flake class), so only that outcome reruns, each time in a
// fresh child (threadsafe style re-executes the binary per EXPECT_EXIT). Every
// other exit code decides on the first try. Three tries: the tries are
// independent processes, so a per-try inconclusive rate p fails the test with
// p^3; 20 runs next to stress-ng --cpu 8 on 24 cores saw no inconclusive try,
// so even p = 0.1 would fail 1 run in 1000, while a backstop
// that always fires inside the window still fails every try with exit 2.
constexpr int kBackstopTries = 3;

void expect_backstop_outcome(StopReached reached)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  for (int attempt = 1; attempt <= kBackstopTries; ++attempt) {
    const bool last = attempt == kBackstopTries;
    int status = -1;
    auto decided_or_retry = [&status, last](int exit_status) {
        status = exit_status;
        if (!WIFEXITED(exit_status)) {
          return false;
        }
        const int code = WEXITSTATUS(exit_status);
        return code == 0 || (code == 5 && !last);
      };
    EXPECT_EXIT(stall_backstop_scenario(reached), decided_or_retry, "");
    if (status == -1) {
      // No child status: in a death-test child this try belongs to another
      // child and was skipped; in the parent gtest already reported the
      // failed spawn. Either way the next try runs.
      continue;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 5) {
      return;  // decided: 0 passed, anything else already failed above
    }
    if (last) {
      ADD_FAILURE() << "all " << kBackstopTries << " tries were inconclusive (exit 5): the "
        "test thread was descheduled across stop + W in every fresh child, so the backstop "
        "was never judged";
      return;
    }
    std::printf(
      "[ RETRY    ] StallBackstop try %d of %d inconclusive (exit 5, test thread "
      "descheduled across stop + W); rerunning in a fresh child\n", attempt, kBackstopTries);
    std::fflush(stdout);
  }
}

}  // namespace

TEST(StallBackstop, TimerFreeFederatePastItsStopTagIsAStall)
{
  expect_backstop_outcome(StopReached::No);
}

TEST(StallBackstop, FederateThatReachedItsStopTagIsNoStall)
{
  expect_backstop_outcome(StopReached::Yes);
}

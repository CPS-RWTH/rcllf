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

/// @file test_match_warmup.cpp
/// @brief The cross-federate match warm-up.
///
/// Two rclcpp contexts are two DDS participants, as two federates are. The
/// writer publishes its first sample right after the warm-up, as a fast
/// federate does right after the RTI handshake. With ROS2_LF_MATCH_WARMUP_MS=0
/// the warm-up does not wait and the best-effort sample is published before
/// the writer matched the reader, so FirstSampleArrivesAfterWarmup fails with
/// the bug's message.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "match_warmup.hpp"
#include "rmw/qos_profiles.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mw = ros2_lf_binding::match_warmup;
using std::chrono::milliseconds;

namespace
{

std::shared_ptr<rclcpp::Context> make_context()
{
  auto ctx = std::make_shared<rclcpp::Context>();
  ctx->init(0, nullptr);
  return ctx;
}

rclcpp::NodeOptions on(const std::shared_ptr<rclcpp::Context> & ctx)
{
  return rclcpp::NodeOptions().context(ctx).start_parameter_services(false)
         .start_parameter_event_publisher(false);
}

std::string unique_topic(const char * stem)
{
  static int n = 0;
  return std::string("/mw_") + stem + "_" + std::to_string(getpid()) + "_" + std::to_string(++n);
}

}  // namespace

// Two participants, a reliable writer and a best-effort reader (the admitted
// /brake_cmd edge of the R2 federation). The first sample is published right
// after the warm-up and must arrive.
TEST(MatchWarmup, FirstSampleArrivesAfterWarmup)
{
  auto ctx_pub = make_context();
  auto ctx_sub = make_context();
  {
    auto pub_node = std::make_shared<rclcpp::Node>("mw_writer", on(ctx_pub));
    auto sub_node = std::make_shared<rclcpp::Node>("mw_reader", on(ctx_sub));
    const std::string topic = unique_topic("first");
    auto sub = sub_node->create_subscription<std_msgs::msg::Float64>(
      topic, rclcpp::QoS(10).best_effort(), [](std_msgs::msg::Float64::ConstSharedPtr) {});
    auto pub = pub_node->create_publisher<std_msgs::msg::Float64>(topic, rclcpp::QoS(10));

    // The runtime passes its discovery budget (runtime.cpp discovery_timeout_ms,
    // 5000 ms unless ROS2_LF_DISCOVERY_TIMEOUT_MS).
    const int64_t timeout_ms = mw::timeout_ms_from_env(5000);
    std::vector<mw::Target> targets{
      {topic, true, 1, [&pub]() {return pub->get_subscription_count();}},
      {topic, false, 1, [&sub]() {return sub->get_publisher_count();}},
    };
    const auto out = mw::wait(targets, milliseconds(timeout_ms));
    EXPECT_TRUE(mw::log_outcomes(out, "test", timeout_ms));
    const size_t writer_matched_at_publish = pub->get_subscription_count();

    std_msgs::msg::Float64 first;
    first.data = 42.0;
    pub->publish(first);

    bool got = false;
    std_msgs::msg::Float64 msg;
    rclcpp::MessageInfo info;
    const auto deadline = std::chrono::steady_clock::now() + milliseconds(1000);
    while (!got && std::chrono::steady_clock::now() < deadline) {
      got = sub->take(msg, info);
      if (!got) {
        std::this_thread::sleep_for(milliseconds(1));
      }
    }
    ASSERT_TRUE(got) <<
      "the first sample was published before the reader matched and never arrived "
      "(writer matched " << writer_matched_at_publish << " reader(s) at the publish, warm-up " <<
      timeout_ms << " ms)";
    EXPECT_DOUBLE_EQ(msg.data, 42.0);
    EXPECT_TRUE(out[0].matched);
    EXPECT_TRUE(out[1].matched);
  }
  ctx_pub->shutdown("test done");
  ctx_sub->shutdown("test done");
}

// A reader that never comes: the wait ends at its bound and reports the
// target unmatched with its last count.
TEST(MatchWarmup, TimeoutIsBoundedAndReported)
{
  size_t polls = 0;
  std::chrono::steady_clock::time_point first_read;
  std::vector<mw::Target> targets{
    {"/never", true, 2, [&polls, &first_read]() {
        if (polls++ == 0) {
          first_read = std::chrono::steady_clock::now();
        }
        return size_t{1};
      }},
    {"/now", false, 1, []() {return size_t{1};}},
  };
  const auto start = std::chrono::steady_clock::now();
  const auto out = mw::wait(targets, milliseconds(50));
  const auto took = std::chrono::steady_clock::now() - start;
  // An unmatched target refuses the start. The old warm-up logged
  // "starting anyway" and the run went on with an uncounted loss.
  EXPECT_FALSE(mw::log_outcomes(out, "test", 50)) <<
    "an unmatched endpoint let the start go on: a sample published before the match "
    "is lost with no count";
  EXPECT_GE(took, milliseconds(50));
  // Bounded, with slack for a loaded CI runner.
  EXPECT_LT(took, milliseconds(5000));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_FALSE(out[0].matched);
  EXPECT_EQ(out[0].have, 1u);
  EXPECT_EQ(out[0].want, 2u);
  EXPECT_GT(polls, 1u);
  EXPECT_TRUE(out[1].matched);
  // match_us is a measured wall time (match_warmup.hpp, Outcome),
  // taken at the start of the pass that saw the match, so a match at the
  // first check is not 0 when the thread is preempted between the start of
  // the wait and its first pass. What the first check does fix: the time is
  // no later than the first read of the pass, which "/never" records, so a
  // match first seen on a later pass (one poll interval on) is caught.
  const auto first_read_us =
    std::chrono::duration_cast<std::chrono::microseconds>(first_read - start).count();
  EXPECT_GE(out[1].match_us, 0);
  EXPECT_LE(out[1].match_us, first_read_us) <<
    "\"/now\" matched at the first check but match_us is later than that check";
  EXPECT_EQ(out[1].lost, 0u);
}

// Matched at the first check: no wait at all, as in real time once matched.
TEST(MatchWarmup, MatchedTargetsReturnAtOnce)
{
  std::vector<mw::Target> targets{{"/t", true, 1, []() {return size_t{3};}}};
  const auto start = std::chrono::steady_clock::now();
  const auto out = mw::wait(targets, milliseconds(2000));
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds(1000));  // < the 2 s bound
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].matched);
  EXPECT_EQ(out[0].have, 3u);
}

// A match that is lost again does not
// count. Target A matches at the first read and loses its reader at the
// third, target B matches only at the fourth. The barrier must not lift on
// A's old match: the wait runs to its bound and A is reported unmatched.
TEST(MatchWarmup, MatchLostBeforeTheLastMatchRefuses)
{
  int reads_a = 0;
  int reads_b = 0;
  std::vector<mw::Target> targets{
    {"/a", true, 1, [&reads_a]() {return size_t{++reads_a < 3 ? 1u : 0u};}},
    {"/b", false, 1, [&reads_b]() {return size_t{++reads_b < 4 ? 0u : 1u};}},
  };
  const auto out = mw::wait(targets, milliseconds(50));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_FALSE(out[0].matched) <<
    "the barrier lifted on a remembered match of /a whose reader had gone away: a sample "
    "published then is lost with no count";
  EXPECT_EQ(out[0].have, 0u);
  EXPECT_EQ(out[0].lost, 1u);
  EXPECT_TRUE(out[1].matched);
  EXPECT_FALSE(mw::log_outcomes(out, "test", 50));
}

// The same with two DDS participants: writer A matches a remote reader, the
// reader goes away, then reader B matches a remote writer. Before the fix the
// barrier lifted as soon as B matched.
TEST(MatchWarmup, RealReaderLeavesBeforeTheOtherTopicMatches)
{
  auto ctx_local = make_context();
  auto ctx_peer = make_context();
  {
    auto local = std::make_shared<rclcpp::Node>("mw_local", on(ctx_local));
    const std::string topic_a = unique_topic("leave_a");
    const std::string topic_b = unique_topic("leave_b");
    auto pub_a = local->create_publisher<std_msgs::msg::Float64>(topic_a, rclcpp::QoS(10));
    auto sub_b = local->create_subscription<std_msgs::msg::Float64>(
      topic_b, rclcpp::QoS(10), [](std_msgs::msg::Float64::ConstSharedPtr) {});
    std::atomic<bool> a_seen{false};
    std::vector<mw::Target> targets{
      {topic_a, true, 1, [&pub_a, &a_seen]() {
          const size_t n = pub_a->get_subscription_count();
          if (n >= 1) {
            a_seen = true;
          }
          return n;
        }},
      {topic_b, false, 1, [&sub_b]() {return sub_b->get_publisher_count();}},
    };
    // 3000 ms: three times the 1000 ms each discovery step below gets, so
    // the steps fit on a loaded runner and the wait still ends at its bound.
    std::vector<mw::Outcome> out;
    std::thread waiter([&]() {out = mw::wait(targets, milliseconds(3000));});

    auto peer = std::make_shared<rclcpp::Node>("mw_peer", on(ctx_peer));
    auto reader_a = peer->create_subscription<std_msgs::msg::Float64>(
      topic_a, rclcpp::QoS(10), [](std_msgs::msg::Float64::ConstSharedPtr) {});
    const auto until = [](auto cond) {
        const auto deadline = std::chrono::steady_clock::now() + milliseconds(1000);
        while (!cond() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(milliseconds(1));
        }
        return cond();
      };
    // The waits are asserted only after the join: an ASSERT returns from the
    // test, and a joinable std::thread destroyed then calls std::terminate.
    const bool saw_a = until([&]() {return a_seen.load();});
    reader_a.reset();
    const bool a_gone = until([&]() {return pub_a->get_subscription_count() == 0;});
    auto writer_b = peer->create_publisher<std_msgs::msg::Float64>(topic_b, rclcpp::QoS(10));
    waiter.join();
    ASSERT_TRUE(saw_a) << "the wait never saw reader A";
    ASSERT_TRUE(a_gone) << "reader A did not go away";

    ASSERT_EQ(out.size(), 2u);
    EXPECT_FALSE(out[0].matched) <<
      "the barrier lifted on a remembered match of " << topic_a << " whose reader had gone "
      "away: a sample published then is lost with no count";
    EXPECT_GE(out[0].lost, 1u);
    EXPECT_TRUE(out[1].matched);
    EXPECT_FALSE(mw::log_outcomes(out, "test", 3000));
  }
  ctx_local->shutdown("test done");
  ctx_peer->shutdown("test done");
}

// A twin node holds two readers on one topic, and the second is
// made late. The generator counts both (cross_federate_topics lists one entry
// per endpoint), and the writer's DDS count counts endpoints too, so the
// barrier waits for the late twin.
TEST(MatchWarmup, WriterWaitsForTheLateTwinReader)
{
  auto ctx_local = make_context();
  auto ctx_peer = make_context();
  {
    auto local = std::make_shared<rclcpp::Node>("mw_twin_writer", on(ctx_local));
    auto peer = std::make_shared<rclcpp::Node>("mw_twin", on(ctx_peer));
    const std::string topic = unique_topic("twin");
    auto pub = local->create_publisher<std_msgs::msg::Float64>(topic, rclcpp::QoS(10));
    auto first = peer->create_subscription<std_msgs::msg::Float64>(
      topic, rclcpp::QoS(10), [](std_msgs::msg::Float64::ConstSharedPtr) {});
    std::vector<mw::Target> targets{
      {topic, true, 2, [&pub]() {return pub->get_subscription_count();}}};
    std::vector<mw::Outcome> out;
    const auto start = std::chrono::steady_clock::now();
    std::thread waiter([&]() {out = mw::wait(targets, milliseconds(3000));});
    // 300 ms: long after the first reader matched on one host (logged
    // match_us values are well below it), short against the 3000 ms bound.
    std::this_thread::sleep_for(milliseconds(300));
    auto second = peer->create_subscription<std_msgs::msg::Float64>(
      topic, rclcpp::QoS(10), [](std_msgs::msg::Float64::ConstSharedPtr) {});
    waiter.join();
    ASSERT_EQ(out.size(), 1u);
    EXPECT_TRUE(out[0].matched);
    EXPECT_EQ(out[0].have, 2u);
    EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds(300)) <<
      "the barrier lifted with the late twin reader unmatched";
  }
  ctx_local->shutdown("test done");
  ctx_peer->shutdown("test done");
}

// The bound is the caller's discovery budget, not a constant of
// its own, unless ROS2_LF_MATCH_WARMUP_MS sets it.
TEST(MatchWarmup, BoundDefaultsToTheCallersBudget)
{
  unsetenv("ROS2_LF_MATCH_WARMUP_MS");
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  EXPECT_EQ(mw::timeout_ms_from_env(30000), 30000);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "0", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 0);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "abc", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  unsetenv("ROS2_LF_MATCH_WARMUP_MS");
}

TEST(MatchWarmup, ParsesCrossFederateTopics)
{
  const auto doc = nlohmann::json::parse(
    R"({"cross_federate_topics": [
          {"topic": "/brake_cmd", "publishers": 1, "subscribers": 1},
          {"topic": "/radar/range", "publishers": 2, "subscribers": 1},
          {"publishers": 1},
          {"topic": "/odd", "publishers": -1}
        ]})");
  const auto t = mw::parse_cross_topics(doc);
  ASSERT_EQ(t.size(), 3u);
  EXPECT_EQ(t[0].topic, "/brake_cmd");
  EXPECT_EQ(t[1].publishers, 2u);
  EXPECT_EQ(t[1].subscribers, 1u);
  EXPECT_EQ(t[2].topic, "/odd");
  EXPECT_EQ(t[2].publishers, 0u);
  EXPECT_TRUE(mw::parse_cross_topics(nlohmann::json::parse(R"({"nodes": {}})")).empty());
  EXPECT_TRUE(mw::load_cross_topics("/nonexistent/system.json").empty());
}

TEST(MatchWarmup, TimeoutFromEnv)
{
  const char * saved = std::getenv("ROS2_LF_MATCH_WARMUP_MS");
  const std::string keep = saved ? saved : "";
  unsetenv("ROS2_LF_MATCH_WARMUP_MS");
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "0", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 0);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "350", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 350);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "-5", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "9223372036854775807", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  setenv("ROS2_LF_MATCH_WARMUP_MS", "2s", 1);
  EXPECT_EQ(mw::timeout_ms_from_env(5000), 5000);
  if (saved) {
    setenv("ROS2_LF_MATCH_WARMUP_MS", keep.c_str(), 1);
  } else {
    unsetenv("ROS2_LF_MATCH_WARMUP_MS");
  }
}

// Startup readiness covers the writer side and an
// unmatched endpoint refuses the start. On the unfixed code local_targets did
// not exist (wait_ready polled the readers only) and a timeout logged
// "starting anyway"; the checks below then fail with the messages they name.

TEST(StartupReadiness, AWriterWaitsForEveryLocalReader)
{
  // The brake_fusion case: two local readers of /sim/state see the writer,
  // the writer has seen only one of them.
  const std::vector<mw::LocalReader> readers = {
    {"/sim/state", []() {return size_t{1};}},
    {"/sim/state", []() {return size_t{1};}},
    {"/brake_cmd", []() {return size_t{1};}},
  };
  const auto targets = mw::local_targets(
    readers, [](const std::string & topic) {
      return topic == "/sim/state" ? std::vector<size_t>{1} : std::vector<size_t>{};
    });
  ASSERT_EQ(targets.size(), 4u) <<
    "the writer side was not a target: a publisher that sees 1 of 2 local readers let the "
    "start go ahead";
  EXPECT_TRUE(targets[3].writer);
  EXPECT_EQ(targets[3].topic, "/sim/state");
  EXPECT_EQ(targets[3].want, 2u);
  const auto out = mw::wait(targets, milliseconds(30));
  EXPECT_NE(
    mw::unready_message(out, "wait_ready", 30).find(
      "topic '/sim/state': the local publisher matched 1 of 2 readers"), std::string::npos)
    << mw::unready_message(out, "wait_ready", 30);
}

TEST(StartupReadinessDeathTest, AnUnmatchedEndpointRefusesTheStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const std::vector<mw::Target> targets = {
    {"/sim/state", true, 3, []() {return size_t{2};}}};
  const auto out = mw::wait(targets, milliseconds(20));
  EXPECT_EXIT(
    mw::enforce_ready(out, "wait_ready", 20), testing::ExitedWithCode(mw::kUnreadyExitCode),
    "refusing to start \\(wait_ready\\): topic '/sim/state': the local publisher matched 2 "
    "of 3 readers") << "an unmatched endpoint did not refuse the start: the run started anyway";
}

TEST(StartupReadiness, ADelayedMatchStartsOnceMatched)
{
  // Discovery delayed by 80 ms: the start waits for it and goes ahead.
  // The 80 ms window opens at the wait's first read, not at a time the
  // test takes before calling wait(). A test thread descheduled for 80 ms
  // between such a t0 and the first read saw the match on that first read and
  // failed the unmatched-read count, with the binding correct. match_us is
  // bounded by the reads the wait made: wait() stamps a pass before it reads,
  // so a correct match_us may come out a few us under 80 ms (79996 seen on
  // CI), but it cannot come out under (last read that saw 1) - (first read):
  // the wait started before its first read, and the pass that saw 2 was
  // stamped after the read before it. A match_us stamped too early, or never,
  // is under it.
  using steady = std::chrono::steady_clock;
  steady::time_point first_read{}, last_unmatched{};
  int unmatched_reads = 0;
  const std::vector<mw::Target> targets = {
    {"/sim/state", true, 2, [&]() {
        const auto now = steady::now();
        if (first_read == steady::time_point{}) {
          first_read = now;
        }
        if (now - first_read < milliseconds(80)) {
          last_unmatched = now;
          ++unmatched_reads;
          return size_t{1};
        }
        return size_t{2};
      }}};
  const auto out = mw::wait(targets, milliseconds(2000));
  ASSERT_TRUE(out[0].matched);
  ASSERT_GE(unmatched_reads, 2) << "the wait read the delayed match unmatched " <<
    unmatched_reads << " time(s) in 80 ms";
  const int64_t floor_us =
    std::chrono::duration_cast<std::chrono::microseconds>(last_unmatched - first_read).count();
  EXPECT_GE(out[0].match_us, floor_us) << "match_us " << out[0].match_us <<
    " is before the last read that saw the endpoint unmatched (" << floor_us << " us)";
  EXPECT_TRUE(mw::unready_message(out, "wait_ready", 2000).empty());
  mw::enforce_ready(out, "wait_ready", 2000);  // returns: no refusal
}

// A pair whose verdict cannot be told
// (nullopt: a null handle, an unreadable QoS, a failed check) counted as
// matched, so the counts alone started the run. Fail closed: it is unmatched
// and its reason is in the refusal.
TEST(StartupReadiness, AnUnknownPairVerdictRefusesTheStart)
{
  mw::LocalEdge e;
  e.topic = "/unknown_pair";
  int r = 0;
  int w = 0;
  e.reader = &r;
  e.writer = &w;
  e.reader_writers = []() {return size_t{1};};
  e.writer_readers = []() {return size_t{1};};
  e.paired = []() {
      return ros2_lf_binding::pair_match::Verdict{std::nullopt, "the writer is gone"};
    };
  const auto outcomes = mw::wait(mw::local_edge_targets({e}), std::chrono::milliseconds(0));
  ASSERT_EQ(outcomes.size(), 3u);
  EXPECT_FALSE(outcomes[2].matched) << "a pair whose verdict is unknown (nullopt) counted as "
    "matched: the warm-up must fail closed";
  const std::string why = mw::unready_message(outcomes, "local_edges", 0);
  EXPECT_NE(why.find("cannot be told: the writer is gone"), std::string::npos) << why;
}

// The pair proof, step by step.
namespace pm = ros2_lf_binding::pair_match;

pm::Facts proven_pair()
{
  pm::Facts f;
  f.same_context = true;
  f.writer_gid.fill(7);
  f.reader_type = "std_msgs/msg/Int32";
  pm::GraphWriter own;
  own.gid = f.writer_gid;
  own.type = "std_msgs/msg/Int32";
  f.graph_writers = {own};
  // Actual QoS as rmw reports it: every policy resolved. rmw_qos_profile_default
  // keeps liveliness at SYSTEM_DEFAULT, which the check answers with a warning.
  f.writer_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  f.reader_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return f;
}

TEST(PairMatch, AProvenPairIsMatched)
{
  const auto v = pm::judge(proven_pair());
  ASSERT_TRUE(v.paired.has_value()) << v.why;
  EXPECT_TRUE(*v.paired) << v.why;
}

TEST(PairMatch, DecoysOfAnotherTypeDoNotProveTheWriter)
{
  // The reader's graph holds decoys only: a writer of the same type with
  // another GID, and none of the edge's writer.
  auto f = proven_pair();
  f.graph_writers[0].gid.fill(9);
  const auto v = pm::judge(f);
  ASSERT_TRUE(v.paired.has_value());
  EXPECT_FALSE(*v.paired) << "a decoy writer's GID proved the edge's writer";
  EXPECT_NE(v.why.find("does not see the writer"), std::string::npos) << v.why;
}

TEST(PairMatch, AWriterOfAnotherTypeIsRefused)
{
  auto f = proven_pair();
  f.graph_writers[0].type = "std_msgs/msg/String";
  const auto v = pm::judge(f);
  ASSERT_TRUE(v.paired.has_value());
  EXPECT_FALSE(*v.paired) << "an Int32 reader was judged matched to a String writer";
  EXPECT_EQ(v.why, "the writer has type 'std_msgs/msg/String', the reader 'std_msgs/msg/Int32'");
}

TEST(PairMatch, APairInTwoContextsIsRefused)
{
  auto f = proven_pair();
  f.same_context = false;
  const auto v = pm::judge(f);
  ASSERT_TRUE(v.paired.has_value());
  EXPECT_FALSE(*v.paired);
  EXPECT_NE(v.why.find("different contexts"), std::string::npos) << v.why;
}

TEST(PairMatch, AnIncompatibleQosIsRefused)
{
  auto f = proven_pair();
  f.writer_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  f.reader_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  const auto v = pm::judge(f);
  ASSERT_TRUE(v.paired.has_value());
  EXPECT_FALSE(*v.paired);
  EXPECT_NE(v.why.find("incompatible"), std::string::npos) << v.why;
}

TEST(PairMatch, AQosWarningIsNotAProof)
{
  // A QoS check that ends in a warning has not
  // established compatibility (a policy is SYSTEM_DEFAULT or UNKNOWN), so the
  // pair is not proven. The verdict is unknown and the warm-up fails closed.
  auto f = proven_pair();
  f.writer_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  f.reader_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_compatibility_type_t c = RMW_QOS_COMPATIBILITY_OK;
  char reason[512] = {0};
  ASSERT_EQ(
    rmw_qos_profile_check_compatible(f.writer_qos, f.reader_qos, &c, reason, sizeof(reason)),
    RMW_RET_OK);
  ASSERT_EQ(c, RMW_QOS_COMPATIBILITY_WARNING) << "the fixture no longer yields a QoS warning";
  const auto v = pm::judge(f);
  EXPECT_FALSE(v.paired.has_value() && *v.paired) <<
    "a QoS warning was counted as a proven pair: " << v.why;
  EXPECT_FALSE(v.paired.has_value()) << "a QoS warning must give the unknown verdict";
  EXPECT_NE(v.why.find("could not be established"), std::string::npos) << v.why;
}

TEST(PairMatch, AnUnreadableTypeCannotBeTold)
{
  auto f = proven_pair();
  f.reader_type.clear();
  const auto v = pm::judge(f);
  EXPECT_FALSE(v.paired.has_value()) << v.why;
}

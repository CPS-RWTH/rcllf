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

/// @file test_local_now.cpp
/// @brief A local edge is decided when its publish
///        returns. The publication takes its sample from each local reader
///        under the delivery lock (rcllf_gate_pre/post), so the drain never
///        waits on a local edge, a missing sample ends LOST local_undelivered
///        at once, and only a planted one passes the reconcile. The
///        executable gate (src/publication_gate.cpp) is linked in.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "corr_runtime.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "match_warmup.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rid_table.hpp"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/publication_gate_api.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "semantic_trace.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
extern std::vector<HandleEntry> g_handle_table;
}

namespace rt = ros2_lf_binding::corr_rt;
namespace st = ros2_lf_binding::semtrace;
using ros2_lf_binding::corr::Outcome;

namespace
{

constexpr int kPubRid = 0;
constexpr int kSubRid = 1;
constexpr int kSubRid2 = 2;
constexpr int kPubRidB = 3;
constexpr int kAnyGroup = ros2_lf_binding::kGroupUnknown;

/// The dispatch_end events, with the join wait the fake sink of the other
/// suites does not keep.
class JoinWaitSink : public st::Sink
{
public:
  struct End
  {
    uint8_t outcome;
    int64_t join_wait_ns;
  };
#define JW_EV(ev, catv) \
  bool enabled_ ## ev() override {return true; \
  } \
  void emit_ ## ev(uint64_t, uint64_t, const rcllf_tp_ ## ev & e) override {keep(e);}
  RCLLF_TRACE_EVENTS(JW_EV)
#undef JW_EV
  std::vector<End> ends()
  {
    std::lock_guard<std::mutex> lk(mu_);
    return ends_;
  }

private:
  template<typename T>
  void keep(const T &) {}
  void keep(const rcllf_tp_dispatch_end & e)
  {
    std::lock_guard<std::mutex> lk(mu_);
    ends_.push_back({e.outcome, e.join_wait_ns});
  }
  std::mutex mu_;
  std::vector<End> ends_;
};

std::string outcomes_text(uint32_t edge)
{
  std::string out;
  for (const auto & kv : rt::table().counts(edge).outcomes) {
    out += std::string(out.empty() ? "" : ", ") + ros2_lf_binding::corr::outcome_name(kv.first) +
      "=" + std::to_string(kv.second);
  }
  return out.empty() ? "none" : out;
}

/// The reconcile problems, without the worker lag ones: the tests drive the
/// drain at tag 0 while physical time is the epoch clock, so every drain
/// measures a lag of decades. Not what these tests check.
std::string table_problems()
{
  std::string all;
  for (const auto & p : rt::table().reconcile().problems) {
    if (p.find("worker_lag_exceeded") == std::string::npos) {
      all += p + "; ";
    }
  }
  return all;
}

uint64_t outcome_count(uint32_t edge, Outcome o)
{
  const auto c = rt::table().counts(edge);
  return c.outcomes.count(o) ? c.outcomes.at(o) : 0;
}

class LocalNow : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_TAKE_MODE");
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    ros2_lf_binding::IngressRole::instance().stop_and_join(false);
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    setenv("RCLLF_ADMITTED_READER_RELIABLE", "1", 1);
    rt::reset_for_test();
    st::reset_for_test();
    st::set_sink_for_test(&sink);
    ros2_lf_binding::clear_rid_entries_for_test();
    ++counter;
    pub_node = std::make_shared<rclcpp::Node>("local_now_pub_" + std::to_string(counter));
    sub_node = std::make_shared<rclcpp::Node>("local_now_sub_" + std::to_string(counter));
    topic = "/local_now_" + std::to_string(getpid()) + "_" + std::to_string(counter);
    ros2_lf_binding::register_rid_entry({kSubRid, "Sub", "on_data", "sub", topic, 0, 0, kAnyGroup});
    ros2_lf_binding::g_handle_table.assign(kPubRidB + 1, ros2_lf_binding::HandleEntry{});
  }

  void TearDown() override
  {
    ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
    rt::set_local_after_publish_hook_for_test({});
    rt::set_local_delivery_lock_for_test(true);
    rt::set_local_take_skip_for_test({});
    rt::set_reader_depth_for_test(8);
    rt::reset_for_test();
    st::set_sink_for_test(nullptr);
    st::reset_for_test();
    unsetenv("RCLLF_ADMITTED_READER_RELIABLE");
    unsetenv("ROS2_LF_DISCOVERY_TIMEOUT_MS");
  }

  /// A subscription of node `node` (binding key `key`) admitted as RID `rid`,
  /// with T = `timeout_ms`. Its values land in heard[rid].
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr make_sub(
    const rclcpp::Node::SharedPtr & node, const std::string & key, int rid, size_t depth,
    int timeout_ms, const rclcpp::QoS * qos = nullptr)
  {
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
    {
      Ros2LfNodeConstructionScope scope(key.c_str());  // as the generated node factory does
      s = node->create_subscription<std_msgs::msg::Int32>(
        topic, qos ? *qos : rclcpp::QoS(depth), [this, rid](const std_msgs::msg::Int32 & m) {
          std::lock_guard<std::mutex> lk(mu);
          heard[rid].push_back(m.data);
        });
    }
    auto & e = ros2_lf_binding::g_handle_table[rid];
    e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
    e.subscription = s;
    e.take_timeout_ms = timeout_ms;
    e.node_class = key;
    e.callback_name = "on_data";
    return s;
  }

  std::optional<rt::StartupFailure> start()
  {
    auto f = rt::startup(
      [this](const std::string & key) {
        return key == "Pub" ? pub_node->get_node_base_interface() :
               key == "PubB" ? pub_node_b->get_node_base_interface() :
               key == "Sub2" ? sub_node_2->get_node_base_interface() :
               sub_node->get_node_base_interface();
      });
    if (!f) {
      rt::trace_begin();
    }
    return f;
  }

  static bool matched(const rclcpp::PublisherBase::SharedPtr & p, size_t readers = 1)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (p->get_subscription_count() < readers) {
      if (std::chrono::steady_clock::now() > until) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  /// An admitted publish of `v` by the writer of RID `rid`, as an LF reaction.
  void publish(const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr & p, int rid, int v)
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    rt::OriginScope o(nullptr, rid);
    std_msgs::msg::Int32 m;
    m.data = v;
    p->publish(m);
  }

  /// Emit + arrive + drain of `edge` into subscription RID `rid` (d = 0), as
  /// the generated reactions do. Returns the dispatch_end events it added.
  std::vector<JoinWaitSink::End> dispatch(int edge, int rid)
  {
    const size_t before = sink.ends().size();
    const uint64_t waits = rt::table().join_waits();
    {
      ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
      const uint32_t n = ros2_lf_corr_emit(nullptr, edge);
      int64_t w = n > 0 ? ros2_lf_corr_arrive(nullptr, edge, n) : -1;
      std::pair<int64_t, uint32_t> tag{0, 0};
      while (w >= 0) {
        tag = w == 0 ? std::make_pair(tag.first, tag.second + 1) :
          std::make_pair(tag.first + w, 0u);
        ros2_lf_binding::set_lf_tag_for_test(tag);
        w = ros2_lf_corr_drain(nullptr, edge, rid, topic.c_str());
      }
      ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    }
    last_join_waits = rt::table().join_waits() - waits;
    auto all = sink.ends();
    return std::vector<JoinWaitSink::End>(all.begin() + before, all.end());
  }

  /// Edge 0, Pub -> Sub on `topic`, d = 0, T = 5 ms, ingress parked, one
  /// admitted publish of 7, then a 50 ms worker stall before the drain: past
  /// any cutoff t + d + T.
  std::vector<JoinWaitSink::End> publish_and_stall_with_ingress_parked()
  {
    ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
    sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
    const auto f = start();
    if (f) {
      ADD_FAILURE() << "startup refused: " << f->check << " " << f->detail;
      return {};
    }
    if (!matched(pub)) {
      ADD_FAILURE() << "the publisher never matched the reader";
      return {};
    }
    auto & ingress = ros2_lf_binding::IngressRole::instance();
    ingress.pause_corr_takes_for_test(true);  // returns once no take loop is in flight
    publish(pub, kPubRid, 7);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return dispatch(0, kSubRid);
  }

  static int counter;
  /// Joins of the last dispatch() that entered the table's timed wait. The
  /// property "decided without a wait", not the wall time of the join, which
  /// measures the CI runner's scheduling.
  uint64_t last_join_waits = 0;
  JoinWaitSink sink;
  rclcpp::Node::SharedPtr pub_node, pub_node_b, sub_node, sub_node_2;
  std::string topic;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub, pub_b;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub, sub_2;
  std::mutex mu;
  std::map<int, std::vector<int>> heard;
};
int LocalNow::counter = 0;

std::vector<int> heard_of(std::mutex & mu, std::map<int, std::vector<int>> & heard, int rid)
{
  std::lock_guard<std::mutex> lk(mu);
  return heard[rid];
}

// --- (i) the drain never waits on a local edge --------------------------------

TEST_F(LocalNow, ALocalEdgeMatchesAtPublishWhileIngressIsParked)
{
  const auto ends = publish_and_stall_with_ingress_parked();
  ASSERT_FALSE(HasFailure());
  ASSERT_EQ(ends.size(), 1u);
  EXPECT_EQ(outcome_count(0, Outcome::MATCHED), 1u) << "edge 0 ended " << outcomes_text(0) <<
    ": the parked ingress thread never took the local sample, so the drain decided it at the "
    "cutoff t + d + T";
  EXPECT_EQ(ends[0].outcome, static_cast<uint8_t>(Outcome::MATCHED));
  EXPECT_EQ(last_join_waits, 0u) << "the drain waited (" << ends[0].join_wait_ns <<
    " ns) on a local edge decided at publish";
  EXPECT_EQ(heard_of(mu, heard, kSubRid), (std::vector<int>{7}));
}

TEST_F(LocalNow, APlantedDropOnALocalEdgeIsLostAtOnceAndPassesTheReconcile)
{
  // The table's F3 hook drops writer ordinal 1 on edge 0 in transport.
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  ASSERT_FALSE(start().has_value());
  ASSERT_TRUE(matched(pub));
  rt::table().set_drop_sample_for_test(0, 1);
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(true);
  publish(pub, kPubRid, 7);
  // Decided by the publish itself, before any drain or stall.
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 1u) << "edge 0 is " << outcomes_text(0) <<
    " right after the publish returned: the publish did not settle its dropped local sample";
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto ends = dispatch(0, kSubRid);
  ASSERT_EQ(ends.size(), 1u);
  EXPECT_EQ(ends[0].outcome, static_cast<uint8_t>(Outcome::LOST)) << "edge 0 ended " <<
    outcomes_text(0) << ", not LOST local_undelivered";
  EXPECT_EQ(last_join_waits, 0u) << "the drain waited (" << ends[0].join_wait_ns <<
    " ns) for a sample the publish already found missing";
  EXPECT_TRUE(heard_of(mu, heard, kSubRid).empty());
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
  EXPECT_EQ(table_problems(), "") << "a planted drop failed the reconcile";
}

TEST_F(LocalNow, AHookDroppedLocalSampleIsPlanted)
{
  // The env fault hook (-DRCLLF_CORR_TEST_HOOKS=ON) is read once per process:
  // run alone, RCLLF_TEST_CORR_DROP=0:1 ./test_local_now
  // --gtest_filter=LocalNow.AHookDroppedLocalSampleIsPlanted
  const char * drop = std::getenv("RCLLF_TEST_CORR_DROP");
  if (drop == nullptr || std::string(drop) != "0:1") {
    GTEST_SKIP() << "needs RCLLF_TEST_CORR_DROP=0:1 and a hooks-ON build";
  }
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  ASSERT_FALSE(start().has_value());
  ASSERT_TRUE(matched(pub));
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(true);
  publish(pub, kPubRid, 7);
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 1u) << "edge 0 is " << outcomes_text(0) <<
    " right after the publish: the hook's drop was not settled at publish";
  dispatch(0, kSubRid);
  EXPECT_TRUE(heard_of(mu, heard, kSubRid).empty());
  EXPECT_EQ(table_problems(), "") << "the env hook's drop counted as a real undelivered sample";
}

TEST_F(LocalNow, AnUnplantedUndeliveredLocalSampleFailsTheRun)
{
  // The planted flag forced off: the sample is not in the reader after the
  // publish and no hook took it, so the premise broke (exit 3).
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  ASSERT_FALSE(start().has_value());
  ASSERT_TRUE(matched(pub));
  rt::set_local_take_skip_for_test({0});
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(true);
  publish(pub, kPubRid, 7);
  const auto ends = dispatch(0, kSubRid);
  ASSERT_EQ(ends.size(), 1u);
  EXPECT_EQ(ends[0].outcome, static_cast<uint8_t>(Outcome::LOST)) << "edge 0 ended " <<
    outcomes_text(0);
  EXPECT_EQ(last_join_waits, 0u) << "the drain waited (" << ends[0].join_wait_ns <<
    " ns) for an undelivered local sample";
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
  EXPECT_FALSE(rt::close_and_reconcile()) << "an undelivered local sample passed the reconcile";
  EXPECT_TRUE(rt::reconcile_failed()) << "the run would exit 0, not 3";
  const std::string problems = table_problems();
  EXPECT_NE(problems.find("local_undelivered edge 0: 1 sample(s)"), std::string::npos) <<
    problems;
}

// --- Review of the local_now take ------------------------------------------

// Finding 1 (rejected): can a join see a committed local expectation while the
// publish-time take still runs? No. The token is released at (t, m + 1) or
// later (Table::release_micro_locked), and reactor-c starts no reaction of a
// later tag before every reaction of (t, m), the publishing one included,
// returned. This test pauses in the after-publish hook, between commit and
// the take, emits and arrives the token there at the publish tag (0, 0), and
// finds nothing to pop at that tag. The drain at (0, 1) then matches.
TEST_F(LocalNow, ACommittedLocalTokenIsNotReleasedAtItsPublishTag)
{
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  ASSERT_FALSE(start().has_value());
  ASSERT_TRUE(matched(pub));
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(true);
  bool hooked = false;
  bool visible = false;
  std::string at_hook;
  rt::set_local_after_publish_hook_for_test(
    [&]() {
      hooked = true;
      ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{0}, 0u));
      const uint32_t n = ros2_lf_corr_emit(nullptr, 0);
      if (n > 0) {
        ros2_lf_corr_arrive(nullptr, 0, n);
      }
      const auto t = rt::table().pop_released(0, 0, 0);
      visible = t.has_value();
      at_hook = outcomes_text(0);
      ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    });
  publish(pub, kPubRid, 7);
  rt::set_local_after_publish_hook_for_test({});
  ASSERT_TRUE(hooked) << "the after-publish hook never ran";
  EXPECT_FALSE(visible) << "a drain at the publish tag (0, 0) popped the token of a local "
    "expectation whose publish-time take had not run yet: its join would end it LOST "
    "local_undelivered, edge 0 at the hook: " << at_hook;
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{0}, 1u));
    ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
  }
  EXPECT_EQ(outcome_count(0, Outcome::MATCHED), 1u) << "edge 0 ended " << outcomes_text(0);
  EXPECT_EQ(heard_of(mu, heard, kSubRid), (std::vector<int>{7}));
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
}

// Finding 2 (fixed): a KEEP_ALL reader with more than one batch
// (kCorrDrainTakeMax) of older samples ahead of the publication's. The take
// went on past one batch only for a KEEP_LAST reader, so the sample stayed
// behind the old ones and settle_local ended it local_undelivered.
TEST_F(LocalNow, AKeepAllReaderWithAFullBatchQueuedStillYieldsThePublishedSample)
{
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  const rclcpp::QoS keep_all = rclcpp::QoS(rclcpp::KeepAll()).reliable();
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5, &keep_all);
  ASSERT_EQ(sub->get_actual_qos().history(), rclcpp::HistoryPolicy::KeepAll);
  // A writer no edge names (a stock publisher on the topic): its samples
  // stay in the reader while ingress is parked.
  auto other_node = std::make_shared<rclcpp::Node>("local_now_other_" + std::to_string(counter));
  auto other = other_node->create_publisher<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(rclcpp::KeepAll()).reliable());
  ASSERT_FALSE(start().has_value());
  ASSERT_TRUE(matched(pub));
  ASSERT_TRUE(matched(other));
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const size_t queued = ros2_lf_binding::IngressRole::kCorrDrainTakeMax + 100;
  for (size_t i = 0; i < queued; ++i) {
    std_msgs::msg::Int32 m;
    m.data = -1;
    other->publish(m);
  }
  publish(pub, kPubRid, 7);
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 0u) << "edge 0 is " << outcomes_text(0) <<
    " right after the publish: the publish-time take stopped after " <<
    ros2_lf_binding::IngressRole::kCorrDrainTakeMax << " of the " << queued <<
    " older samples in the KEEP_ALL reader and ended the delivered sample local_undelivered";
  const auto ends = dispatch(0, kSubRid);
  ASSERT_EQ(ends.size(), 1u);
  EXPECT_EQ(ends[0].outcome, static_cast<uint8_t>(Outcome::MATCHED)) << outcomes_text(0);
  EXPECT_EQ(heard_of(mu, heard, kSubRid), (std::vector<int>{7}));
  ingress.pause_corr_takes_for_test(false);
  EXPECT_EQ(table_problems(), "");
}

// Review of the KEEP_ALL fix: the KEEP_ALL take stops at its own reader's
// sample. Readers A (edge 0, best-effort KEEP_ALL) and B (edge 1, reliable)
// hear one publication; A is taken first. Between the publish and the take a
// best-effort writer no edge names (B, reliable, never matches it) puts two
// batches behind the sample in A. A stop rule over the whole publication
// keeps draining A while B is still open, so a writer that keeps A full would
// hold the publish forever; the per-reader rule stops after A's first batch.
TEST_F(LocalNow, AKeepAllTakeStopsAtItsOwnReadersSample)
{
  sub_node_2 = std::make_shared<rclcpp::Node>("local_now_sub2_" + std::to_string(counter));
  ros2_lf_binding::register_rid_entry(
    {kSubRid2, "Sub2", "on_data", "sub", topic, 0, 0, kAnyGroup});
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  ros2_lf_binding::register_corr_edge({1, "Pub", {kPubRid}, topic, "Sub2", kSubRid2, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  const rclcpp::QoS keep_all = rclcpp::QoS(rclcpp::KeepAll()).best_effort();
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5, &keep_all);
  sub_2 = make_sub(sub_node_2, "Sub2", kSubRid2, 10, 5);
  auto other_node = std::make_shared<rclcpp::Node>("local_now_other_" + std::to_string(counter));
  auto other = other_node->create_publisher<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(rclcpp::KeepAll()).best_effort());
  const auto f = start();
  ASSERT_FALSE(f.has_value()) << f->check << " " << f->detail;
  ASSERT_TRUE(matched(pub, 2));
  ASSERT_TRUE(matched(other, 1));
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const size_t behind = 2 * ros2_lf_binding::IngressRole::kCorrDrainTakeMax;
  rt::set_local_after_publish_hook_for_test(
    [&]() {
      for (size_t i = 0; i < behind; ++i) {
        std_msgs::msg::Int32 m;
        m.data = -1;
        other->publish(m);
      }
    });
  const uint64_t before = ingress.corr_publish_take_stats().samples;
  publish(pub, kPubRid, 7);
  rt::set_local_after_publish_hook_for_test({});
  const uint64_t taken = ingress.corr_publish_take_stats().samples - before;
  EXPECT_LE(taken, ros2_lf_binding::IngressRole::kCorrDrainTakeMax + 1) <<
    "the publish-time take drained " << taken << " samples: it went on taking reader A after "
    "A's own sample was in, because reader B of the same publication was still open (review "
    "of the KEEP_ALL fix: a writer that keeps A full holds the publish forever)";
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 0u) << "edge 0: " << outcomes_text(0);
  EXPECT_EQ(outcome_count(1, Outcome::LOST), 0u) << "edge 1: " << outcomes_text(1);
  const auto ends0 = dispatch(0, kSubRid);
  const auto ends1 = dispatch(1, kSubRid2);
  ASSERT_EQ(ends0.size(), 1u);
  ASSERT_EQ(ends1.size(), 1u);
  EXPECT_EQ(ends0[0].outcome, static_cast<uint8_t>(Outcome::MATCHED)) << outcomes_text(0);
  EXPECT_EQ(ends1[0].outcome, static_cast<uint8_t>(Outcome::MATCHED)) << outcomes_text(1);
  ingress.pause_corr_takes_for_test(false);
  EXPECT_EQ(table_problems(), "");
}

// Finding 3 (fixed): the edge's writer is best effort, its reader reliable,
// so the pair never matches. A reliable writer no edge names matches the
// reader and a best-effort reader no edge names matches the writer, so both
// matched counts reach their want. Only a per-pair check sees the edge
// unmatched; the counts alone started the run, whose first publish then
// found no sample (local_undelivered, exit 3).
TEST_F(LocalNow, CountsMadeUpByOtherEndpointsDoNotStartAnUnmatchedLocalPair)
{
  setenv("ROS2_LF_DISCOVERY_TIMEOUT_MS", "1500", 1);
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10).best_effort());
  const rclcpp::QoS reliable = rclcpp::QoS(10).reliable();
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5, &reliable);
  auto decoy_node = std::make_shared<rclcpp::Node>("local_now_decoy_" + std::to_string(counter));
  auto decoy_writer = decoy_node->create_publisher<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10).reliable());
  auto decoy_reader = decoy_node->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((sub->get_publisher_count() < 1 || pub->get_subscription_count() < 1) &&
    std::chrono::steady_clock::now() < until)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(sub->get_publisher_count(), 1u) << "the decoy writer never matched the reader";
  ASSERT_GE(pub->get_subscription_count(), 1u) << "the decoy reader never matched the writer";
  const auto f = start();
  ASSERT_TRUE(f.has_value()) << "startup admitted local edge 0 although its best-effort writer "
    "and reliable reader are not matched to each other: other endpoints on the topic made up "
    "both matched counts";
  EXPECT_EQ(f->check, "local_unmatched") << f->detail;
  EXPECT_NE(f->detail.find("not matched to each other"), std::string::npos) << f->detail;
}

// The QoS-only pair check let a pair
// of two types start. The edge's Int32 writer is in the default context, its
// String reader in a second context (another DDS participant), both with
// default QoS. An Int32 reader no edge names matches the writer, a String
// writer no edge names matches the reader, so both matched counts reach their
// want and the QoS of the pair is compatible. The pair itself never matches
// (two types), so a start would lose the first publish. The pair proof
// (pair_match.hpp: same context, the writer's GID in the reader node's graph,
// same type, compatible QoS) refuses it.
TEST_F(LocalNow, DecoysOfAnotherTypeDoNotStartAPairOfTwoTypes)
{
  setenv("ROS2_LF_DISCOVERY_TIMEOUT_MS", "1500", 1);
  auto ctx_b = std::make_shared<rclcpp::Context>();
  ctx_b->init(0, nullptr);
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub_node = std::make_shared<rclcpp::Node>(
    "local_now_sub_b_" + std::to_string(counter), rclcpp::NodeOptions().context(ctx_b));
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reader;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    reader = sub_node->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10), [](const std_msgs::msg::String &) {});
  }
  auto & h = ros2_lf_binding::g_handle_table[kSubRid];
  h.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  h.subscription = reader;
  h.take_timeout_ms = 5;
  h.node_class = "Sub";
  h.callback_name = "on_data";
  auto decoy_a = std::make_shared<rclcpp::Node>("local_now_decoy_a_" + std::to_string(counter));
  auto decoy_reader = decoy_a->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  auto decoy_b = std::make_shared<rclcpp::Node>(
    "local_now_decoy_b_" + std::to_string(counter), rclcpp::NodeOptions().context(ctx_b));
  auto decoy_writer = decoy_b->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((reader->get_publisher_count() < 1 || pub->get_subscription_count() < 1) &&
    std::chrono::steady_clock::now() < until)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(reader->get_publisher_count(), 1u) << "the decoy String writer never matched";
  ASSERT_GE(pub->get_subscription_count(), 1u) << "the decoy Int32 reader never matched";
  const auto f = start();
  const bool refused = f.has_value();
  const std::string detail = refused ? f->detail : "";
  const std::string check = refused ? f->check : "";
  h = ros2_lf_binding::HandleEntry{};
  reader.reset();
  decoy_writer.reset();
  decoy_b.reset();
  sub_node.reset();
  ctx_b->shutdown("test done");
  ASSERT_TRUE(refused) << "startup admitted local edge 0 although its Int32 writer and String "
    "reader are not matched to each other: decoys of the other type made up both matched "
    "counts and the pair check judged QoS only";
  EXPECT_EQ(check, "local_unmatched") << detail;
  EXPECT_NE(detail.find("not matched to each other"), std::string::npos) << detail;
  EXPECT_NE(detail.find("different contexts"), std::string::npos) << detail;
}

// Finding 4 (rejected): the warm-up names pubs.front() only, and that is the
// edge's only writer. A node with a second publisher on the edge's topic
// refuses startup before the warm-up (corr_runtime.cpp "endpoint" check).
// Review of the KEEP_ALL fix: the per-pair check judges QoS only, so could a
// local writer and reader of one topic differ in type and be counted matched
// by decoys? Not on Fast DDS: the nodes of one context share one participant,
// and rmw_fastrtps refuses an endpoint whose topic that participant already
// has with another type (find_and_check_topic_and_type,
// rmw_fastrtps_shared_cpp/utils.hpp), so such a pair is never created.
TEST_F(LocalNow, ALocalReaderOfAnotherTypeOnTheTopicCannotBeCreated)
{
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  bool created = false;
  try {
    auto s = sub_node->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10), [](const std_msgs::msg::String &) {});
    created = static_cast<bool>(s);
  } catch (const std::exception &) {
  }
  EXPECT_FALSE(created) << "a String reader was created on the Int32 topic " << topic <<
    " of the same context: a type-mismatched local pair would reach the QoS-only pair check";
}

TEST_F(LocalNow, ALocalEdgeWithTwoPublishersOfItsNodeRefusesStartup)
{
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  pub_b = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  const auto f = start();
  ASSERT_TRUE(f.has_value()) << "startup admitted local edge 0 with two publishers of its node: "
    "the warm-up waits for the first one only";
  EXPECT_EQ(f->check, "endpoint") << f->detail;
  EXPECT_NE(f->detail.find("2 publishers of"), std::string::npos) << f->detail;
}

// --- (iii) a planted drop on one subscriber hides nothing on another ----------

TEST_F(LocalNow, APlantedDropOnOneSubscriberDoesNotHideARealLossOnTheOther)
{
  // One writer, two local subscribers (edges 0 and 1). Edge 0's sample is
  // dropped by the F3 hook (planted), edge 1's is lost for real (test seam).
  sub_node_2 = std::make_shared<rclcpp::Node>("local_now_sub2_" + std::to_string(counter));
  ros2_lf_binding::register_rid_entry(
    {kSubRid2, "Sub2", "on_data", "sub", topic, 0, 0, kAnyGroup});
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  ros2_lf_binding::register_corr_edge({1, "Pub", {kPubRid}, topic, "Sub2", kSubRid2, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  sub_2 = make_sub(sub_node_2, "Sub2", kSubRid2, 10, 5);
  const auto f = start();
  ASSERT_FALSE(f.has_value()) << f->check << " " << f->detail;
  ASSERT_TRUE(matched(pub, 2));
  rt::table().set_drop_sample_for_test(0, 1);
  rt::set_local_take_skip_for_test({1});
  publish(pub, kPubRid, 7);
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 1u) << "edge 0: " << outcomes_text(0);
  EXPECT_EQ(outcome_count(1, Outcome::LOST), 1u) << "edge 1: " << outcomes_text(1);
  dispatch(0, kSubRid);
  dispatch(1, kSubRid2);
  const std::string all = table_problems();
  EXPECT_NE(all.find("local_undelivered edge 1: 1 sample(s)"), std::string::npos) <<
    "the planted drop on edge 0 excused the real undelivered sample on edge 1, problems: " << all;
  EXPECT_EQ(all.find("edge 0:"), std::string::npos) << "the planted drop was counted real: " <<
    all;
}

// --- (ii) the delivery lock ----------------------

class LocalNowRace : public LocalNow
{
protected:
  void TearDown() override
  {
    rt::set_local_after_publish_hook_for_test({});
    LocalNow::TearDown();
  }

  /// Writers Pub (edge 0) and PubB (edge 1) on their own nodes into one
  /// KEEP_LAST 1 reader of Sub, one fan-in group, ingress running. `retain`
  /// covers every publish: nothing is drained.
  bool two_writers_on_a_depth_one_reader(uint32_t retain)
  {
    rt::set_reader_depth_for_test(0);  // the user's depth 1 stays
    topic += "_race";
    ros2_lf_binding::clear_rid_entries_for_test();
    ros2_lf_binding::register_rid_entry(
      {kSubRid, "Sub", "on_data", "sub", topic, 0, 0, kAnyGroup});
    ros2_lf_binding::register_corr_edge(
      {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0, -1, -1, 0, retain});
    ros2_lf_binding::register_corr_edge(
      {1, "PubB", {kPubRidB}, topic, "Sub", kSubRid, topic, 0, -1, -1, 0, retain});
    ros2_lf_binding::set_corr_edge_fan_in(0, 0, 1);
    ros2_lf_binding::set_corr_edge_fan_in(1, 0, 0);
    pub_node_b = std::make_shared<rclcpp::Node>("local_now_pub_b_" + std::to_string(counter));
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(1));
    pub_b = pub_node_b->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(1));
    sub = make_sub(sub_node, "Sub", kSubRid, 1, 5);
    EXPECT_EQ(sub->get_actual_qos().depth(), 1u);
    const auto f = start();
    if (f) {
      ADD_FAILURE() << "startup refused: " << f->check << " " << f->detail;
      return false;
    }
    if (!matched(pub) || !matched(pub_b)) {
      ADD_FAILURE() << "a writer never matched the reader";
      return false;
    }
    return true;
  }

  /// Pub publishes 1. Between its rcl_publish and its publish-time take,
  /// PubB publishes 2 from another thread; Pub goes on once PubB returned
  /// or after 200 ms. Ingress is parked, so only the publish-time takes
  /// take. Returns whether PubB's publish returned inside the window.
  bool publish_with_the_other_writer_in_between()
  {
    ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(true);
    std::thread b;
    std::promise<void> b_done;
    auto b_returned = b_done.get_future();
    bool in_window = false;
    rt::set_local_after_publish_hook_for_test(
      [&]() {
        b = std::thread(
          [&]() {
            publish(pub_b, kPubRidB, 2);
            b_done.set_value();
          });
        in_window = b_returned.wait_for(std::chrono::milliseconds(200)) ==
        std::future_status::ready;
      });
    publish(pub, kPubRid, 1);
    if (b.joinable()) {
      b.join();
    } else {
      ADD_FAILURE() << "the after-publish hook never ran";
    }
    return in_window;
  }

  uint64_t race(int per_writer)
  {
    if (!two_writers_on_a_depth_one_reader(static_cast<uint32_t>(per_writer + 8))) {
      return 0;
    }
    std::atomic<bool> go{false};
    auto writer = [&](const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr & p, int rid) {
        while (!go.load()) {
          std::this_thread::yield();
        }
        for (int i = 0; i < per_writer; ++i) {
          publish(p, rid, i);
        }
      };
    std::thread a(writer, pub, kPubRid);
    std::thread b(writer, pub_b, kPubRidB);
    go.store(true);
    a.join();
    b.join();
    return outcome_count(0, Outcome::LOST) + outcome_count(1, Outcome::LOST);
  }
};

TEST_F(LocalNowRace, AnotherWritersPublishWaitsUntilTheSampleWasTaken)
{
  ASSERT_TRUE(two_writers_on_a_depth_one_reader(16));
  const bool in_window = publish_with_the_other_writer_in_between();
  EXPECT_FALSE(in_window) << "PubB's publish returned between Pub's rcl_publish and its "
    "publish-time take: no delivery lock held the reader";
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 0u) << "edge 0 is " << outcomes_text(0) <<
    ": PubB's sample overwrote Pub's in the depth-1 reader before the take";
  EXPECT_EQ(outcome_count(1, Outcome::LOST), 0u) << "edge 1 is " << outcomes_text(1);
  dispatch(0, kSubRid);
  dispatch(1, kSubRid);
  EXPECT_EQ(outcome_count(0, Outcome::MATCHED), 1u) << "edge 0: " << outcomes_text(0);
  EXPECT_EQ(outcome_count(1, Outcome::MATCHED), 1u) << "edge 1: " << outcomes_text(1);
  auto got = heard_of(mu, heard, kSubRid);
  std::sort(got.begin(), got.end());
  EXPECT_EQ(got, (std::vector<int>{1, 2}));
}

TEST_F(LocalNowRace, WithoutTheDeliveryLockTheOtherWriterOverwritesTheSample)
{
  // What the test above guards against: the take mutexes taken only after
  // rcl_publish returned, as before the fix.
  rt::set_local_delivery_lock_for_test(false);
  ASSERT_TRUE(two_writers_on_a_depth_one_reader(16));
  EXPECT_TRUE(publish_with_the_other_writer_in_between()) << "PubB blocked without the lock";
  EXPECT_EQ(outcome_count(0, Outcome::LOST), 1u) << "edge 0 is " << outcomes_text(0) <<
    ": the overwrite seam does not reach the depth-1 reader";
}

TEST_F(LocalNowRace, TwoWritersKeepEveryPublishOnADepthOneReader)
{
  const uint64_t lost = race(5000);
  ASSERT_FALSE(HasFailure());
  EXPECT_EQ(lost, 0u) << lost << " of 10000 publishes ended local_undelivered: the other "
    "writer overwrote the sample in the depth-1 reader, or the ingress thread held it, between "
    "rcl_publish and the publish-time take, edge 0: " <<
    outcomes_text(0) << ", edge 1: " << outcomes_text(1);
}

// --- R4: a local edge starts only matched ---------------------------------------

TEST(LocalEdgeTargets, EachReaderWantsItsWritersAndEachWriterItsReaders)
{
  namespace mw = ros2_lf_binding::match_warmup;
  int r1 = 0, r2 = 0, w1 = 0, w2 = 0;
  auto n = [](size_t v) {return [v]() {return v;};};
  // A fan-in reader r1 with two writers; writer w1 also feeds reader r2.
  const auto t = mw::local_edge_targets(
    {{"/a", &r1, &w1, n(1), n(2), {}}, {"/a", &r1, &w2, n(1), n(1), {}},
      {"/a", &r2, &w1, n(1), n(2), {}}});
  ASSERT_EQ(t.size(), 4u);
  EXPECT_FALSE(t[0].writer);
  EXPECT_EQ(t[0].want, 2u) << "the fan-in reader waits for one writer only";
  EXPECT_FALSE(t[1].writer);
  EXPECT_EQ(t[1].want, 1u);
  EXPECT_TRUE(t[2].writer);
  EXPECT_EQ(t[2].want, 2u) << "writer w1 waits for one of its two local readers only";
  EXPECT_TRUE(t[3].writer);
  EXPECT_EQ(t[3].want, 1u);
  const auto o = mw::wait(t, std::chrono::milliseconds(0));
  EXPECT_FALSE(o[0].matched) << "r1 matched 1 of 2 writers and counted as ready";
  EXPECT_TRUE(o[2].matched);
}

TEST_F(LocalNow, ALocalEdgeWhoseReaderNeverMatchesItsWriterRefusesStartup)
{
  // A best-effort writer never matches a reliable reader: a publish would
  // find no sample to take. Startup refuses before tag 0 instead.
  setenv("ROS2_LF_DISCOVERY_TIMEOUT_MS", "300", 1);
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10).best_effort());
  const rclcpp::QoS reliable = rclcpp::QoS(10).reliable();
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5, &reliable);
  const auto f = start();
  ASSERT_TRUE(f.has_value()) << "startup admitted local edge 0 whose reader never matched its "
    "writer: its first publish would end local_undelivered";
  EXPECT_EQ(f->check, "local_unmatched") << f->detail;
  EXPECT_EQ(f->edge, 0);
  EXPECT_NE(f->detail.find(topic), std::string::npos) << f->detail;
}

// --- The older fault hooks are test-build only ---------------------------------

std::vector<std::string> g_corr_errors;

void corr_error_capture(
  const rcutils_log_location_t *, int severity, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  if (severity != RCUTILS_LOG_SEVERITY_ERROR || std::string(name) != "ros2_lf_binding.corr") {
    return;
  }
  char buf[1024];
  va_list copy;
  va_copy(copy, *args);
  vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_corr_errors.emplace_back(buf);
}

// RCLLF_TEST_CORR_DROP_SAMPLE, _DROP_TOKEN and _DRAIN_DELAY are fault hooks
// like the newer trio: an ordinary build (no -DRCLLF_CORR_TEST_HOOKS=ON) must
// inject nothing and say so at startup, once per variable.
TEST_F(LocalNow, OlderFaultHooksInjectNothingInAnOrdinaryBuild)
{
#ifdef RCLLF_CORR_TEST_HOOKS
  GTEST_SKIP() << "hooks-ON build: the hooks are meant to act here";
#endif
  const char * vars[] = {
    "RCLLF_TEST_CORR_DROP_SAMPLE", "RCLLF_TEST_CORR_DROP_TOKEN", "RCLLF_TEST_CORR_DRAIN_DELAY"};
  setenv(vars[0], "0:1", 1);   // drop writer ordinal 1 on edge 0
  setenv(vars[1], "1", 1);     // drop the token of ordinal 1 between pop and dispatch
  setenv(vars[2], "1:20000", 1);  // sleep 20 s in the last drain of ordinal 1
  g_corr_errors.clear();
  rcutils_logging_set_output_handler(corr_error_capture);
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  sub = make_sub(sub_node, "Sub", kSubRid, 10, 5);
  const auto f = start();
  rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  for (const char * v : vars) {
    unsetenv(v);
  }
  ASSERT_FALSE(f.has_value()) << f->check << " " << f->detail;
  ASSERT_TRUE(matched(pub));
  publish(pub, kPubRid, 7);
  const auto t0 = std::chrono::steady_clock::now();
  dispatch(0, kSubRid);
  const auto drain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(outcome_count(0, Outcome::MATCHED), 1u) << "edge 0 ended " << outcomes_text(0) <<
    ": RCLLF_TEST_CORR_DROP_SAMPLE dropped a sample in a build without the test hooks";
  EXPECT_EQ(heard_of(mu, heard, kSubRid), (std::vector<int>{7})) <<
    "RCLLF_TEST_CORR_DROP_SAMPLE or _DROP_TOKEN lost a sample in a build without the test hooks";
  EXPECT_EQ(table_problems(), "") <<
    "a fault hook broke the reconcile in a build without the test hooks";
  // The hook would sleep 20 s; the 10 s bound sits far above any scheduling
  // delay a loaded CI runner adds to an undelayed drain, and far below the
  // delay, so it tells the two apart without flaking under load.
  EXPECT_LT(drain_ms, 10000) << "the drain took " << drain_ms <<
    " ms: RCLLF_TEST_CORR_DRAIN_DELAY slept in a build without the test hooks";
  for (const char * v : vars) {
    const std::string want = std::string("rcllf corr: ") + v +
      " is set but the test hooks are compiled out";
    EXPECT_EQ(
      std::count_if(
        g_corr_errors.begin(), g_corr_errors.end(),
        [&want](const std::string & e) {return e.find(want) != std::string::npos;}), 1) <<
      "no startup error says that " << v << " injects nothing";
  }
}

}  // namespace

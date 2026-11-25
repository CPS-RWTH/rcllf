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

/// @file test_semantic_producers.cpp
/// @brief rcllf producers of the correspondence runtime and the executable
///        gate, recorded through a fake sink (T3). Own executable:
///        the ingress thread is process-wide and cannot restart between suites.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "corr_runtime.hpp"
#include "fake_trace_sink.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rcutils/logging.h"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/publication_gate_api.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "rid_table.hpp"
#include "std_msgs/msg/int32.hpp"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
extern std::vector<HandleEntry> g_handle_table;
}

extern "C" {
#include "core/environment.h"
}
extern environment_t * g_lf_environments_stub;  // lf_environments_stub.cpp

namespace rt = ros2_lf_binding::corr_rt;
using ros2_lf_binding::corr::Outcome;

namespace
{

constexpr int kPubRid = 0;
constexpr int kSubRid = 1;

/// Drain edge 0 from tag `tag` on, each drain at the tag the generated
/// lf_schedule(drain, w) gives for the wake `w` before it: the next microstep
/// for w = 0, else (t + w, 0). A zero-delay token is due one microstep after
/// its publication. Stops at the first wake that is
/// not 0 when `until_nonzero`, else when none is left; returns that wake.
int64_t drain_wakes(
  std::pair<int64_t, uint32_t> tag, int64_t w, const std::string & topic, bool until_nonzero)
{
  while (until_nonzero ? w == 0 : w >= 0) {
    tag = w == 0 ? std::make_pair(tag.first, tag.second + 1) : std::make_pair(tag.first + w, 0u);
    ros2_lf_binding::set_lf_tag_for_test(tag);
    w = ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
  }
  return w;
}

class CorrRuntime : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite()
  {
    ros2_lf_binding::IngressRole::instance().stop_and_join(false);
    rclcpp::shutdown();
  }

  // Join ingress before the fixture's subscription, sink and table go away.
  void TearDown() override
  {
    rt::reset_for_test();
    unsetenv("RCLLF_ADMITTED_READER_RELIABLE");
  }

  void SetUp() override
  {
    // The fixtures build their subscriptions by hand, outside a
    // construction scope, so startup would refuse the RELIABLE readers. The
    // escape hatch keeps the user's QoS; the tests clear it themselves.
    setenv("RCLLF_ADMITTED_READER_RELIABLE", "1", 1);
    rt::reset_for_test();
    ros2_lf_binding::clear_rid_entries_for_test();  // the edge's creation rank
    pub_node = std::make_shared<rclcpp::Node>("corr_pub_" + std::to_string(++counter));
    sub_node = std::make_shared<rclcpp::Node>("corr_sub_" + std::to_string(counter));
    // One topic per process, so concurrent runs in one domain never meet.
    topic = "/corr_test_" + std::to_string(getpid()) + "_" + std::to_string(counter);
    // Creation index 0, any callback group, the first twin on the topic.
    ros2_lf_binding::register_rid_entry(
      {kSubRid, "Sub", "on_data", "sub", topic, 0, 0, ros2_lf_binding::kGroupUnknown});
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
    sub = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [this](const std_msgs::msg::Int32 & m) {
        std::lock_guard<std::mutex> lk(mu);
        heard.push_back(m.data);
      });
    ros2_lf_binding::register_corr_edge(
      {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
    ros2_lf_binding::g_handle_table.assign(2, ros2_lf_binding::HandleEntry{});
    auto & e = ros2_lf_binding::g_handle_table[kSubRid];
    e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
    e.subscription = sub;
    e.take_timeout_ms = 1000;
    e.node_class = "Sub";
    e.callback_name = "on_data";
  }

  rt::NodeResolver resolver()
  {
    return [this](const std::string & key) {
             return key == "Pub" ? pub_node->get_node_base_interface() :
                    sub_node->get_node_base_interface();
           };
  }

  void publish(int v)
  {
    std_msgs::msg::Int32 m;
    m.data = v;
    pub->publish(m);
  }

  /// Emit + arrive + drain all, as the generated reactions do (d = 0).
  std::vector<int> dispatch_all()
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    const uint32_t n = ros2_lf_corr_emit(nullptr, 0);
    if (n > 0) {
      // Outside an LF program ({-1, 0}, no override) the tag is restored to none.
      const auto start = ros2_lf_binding::lf_current_tag();
      drain_wakes(start, ros2_lf_corr_arrive(nullptr, 0, n), topic, false);
      ros2_lf_binding::set_lf_tag_for_test(
        start == std::pair<int64_t, uint32_t>(-1, 0) ?
        std::nullopt : std::make_optional(start));
    }
    std::lock_guard<std::mutex> lk(mu);
    return heard;
  }

  static int counter;
  rclcpp::Node::SharedPtr pub_node, sub_node;
  std::string topic;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
  std::mutex mu;
  std::vector<int> heard;
};
int CorrRuntime::counter = 0;

// --- rcllf producers, recorded through a fake sink. -------------------------------

namespace st = ros2_lf_binding::semtrace;

class SemanticProducers : public CorrRuntime
{
protected:
  void SetUp() override
  {
    CorrRuntime::SetUp();
    st::reset_for_test();
    st::set_sink_for_test(&sink);
  }
  void TearDown() override
  {
    CorrRuntime::TearDown();  // ingress emits into `sink` until joined
    st::set_sink_for_test(nullptr);
    st::reset_for_test();
  }
  void start()
  {
    ASSERT_FALSE(rt::startup(resolver()).has_value());
    rt::trace_begin();
  }
  const void * handle() {return pub->get_publisher_handle().get();}
  rcllf_test::FakeSink sink;
};

TEST_F(SemanticProducers, ZeroPublishesGiveBeginBacklogAndEndOnly)
{
  start();
  rt::close_and_reconcile();
  rt::trace_close();
  EXPECT_EQ(sink.count("run_begin"), 1u);
  EXPECT_EQ(sink.count("provider"), static_cast<size_t>(RCLLF_CAT_COUNT));
  EXPECT_EQ(sink.count("backlog"), 1u);
  EXPECT_EQ(sink.count("publication"), 0u);
  EXPECT_EQ(sink.count("run_end"), 1u);
  EXPECT_TRUE(sink.last_end.reconciled);
}

// run_begin precedes reactor-c's start time, so the start tag is
// traced once by lf_start at the first startup dispatch.
TEST_F(SemanticProducers, LfStartCarriesReactorStartTagExactlyOnce)
{
  environment_t env{};
  env.start_tag.time = 1700000000123456789LL;
  env.start_tag.microstep = 0;
  env.current_tag = env.start_tag;
  rt::trace_lf_start();  // before run_begin: nothing to trace into
  EXPECT_EQ(sink.count("lf_start"), 0u);
  start();
  g_lf_environments_stub = &env;
  rt::trace_lf_start();
  {
    rt::DispatchTrace dt(kPubRid, RCLLF_ENTRY_TIMER);
  }
  rt::trace_lf_start();
  g_lf_environments_stub = nullptr;
  auto starts = sink.of("lf_start");
  ASSERT_EQ(starts.size(), 1u);
  EXPECT_EQ(starts[0].lf_start_time_ns, 1700000000123456789LL);
  auto begins = sink.of("run_begin");
  ASSERT_EQ(begins.size(), 1u);
  EXPECT_GT(starts[0].seq, begins[0].seq);
}

TEST_F(SemanticProducers, LfStartIsAbsentOutsideAnLfProgram)
{
  start();
  {
    rt::DispatchTrace dt(kPubRid, RCLLF_ENTRY_TIMER);
  }
  EXPECT_EQ(sink.count("lf_start"), 0u);
}

TEST_F(SemanticProducers, ManyPublishesWalkPublicationToDispatchByKey)
{
  start();
  {
    rt::DispatchTrace dt(kPubRid, RCLLF_ENTRY_TIMER);
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
    publish(2);
    publish(3);
  }
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1, 2, 3}));
  // Ingress emits `sample` after it hands the sample to the table, so a
  // drain can dispatch it first. The walk is by key: wait for all three.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (sink.count("sample") < 3u && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto pubs = sink.of("publication");
  auto exps = sink.of("expectation");
  auto samples = sink.of("sample");
  auto begins = sink.of("dispatch_begin");
  auto ends = sink.of("dispatch_end");
  ASSERT_EQ(pubs.size(), 3u);
  ASSERT_EQ(exps.size(), 3u);
  ASSERT_EQ(samples.size(), 3u);
  // one timer dispatch (id 1) produced all three, then three corr dispatches
  ASSERT_EQ(begins.size(), 4u);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(pubs[i].ordinal, i + 1);
    EXPECT_EQ(pubs[i].origin, RCLLF_ORIGIN_ADMITTED);
    EXPECT_EQ(pubs[i].dispatch_id, begins[0].dispatch_id);
    EXPECT_EQ(exps[i].ordinal, i + 1);
    EXPECT_EQ(begins[i + 1].ordinal, samples[i].ordinal);
    EXPECT_EQ(ends[i + 1].retained_id, samples[i].retained_id);
    EXPECT_EQ(ends[i + 1].outcome, RCLLF_OUTCOME_MATCHED);
  }
  EXPECT_EQ(sink.count("terminal"), 3u);
  for (size_t i = 1; i < pubs.size(); ++i) {
    EXPECT_EQ(pubs[i].seq, pubs[i - 1].seq + 1);
                                                                                        }
}

TEST_F(SemanticProducers, ConditionalRefusedOriginEmitsPublicationWithoutExpectation)
{
  start();
  {
    rt::OriginScope o(nullptr, 5);  // undeclared effect
    publish(1);
  }
  EXPECT_EQ(sink.count("publication", RCLLF_ORIGIN_UNDECLARED_EFFECT), 1u);
  EXPECT_EQ(sink.count("expectation"), 0u);
}

TEST_F(SemanticProducers, FailedPublishEmitsPublicationFailed)
{
  start();
  rt::OriginScope o(nullptr, kPubRid);
  void * ctx = rcllf_gate_pre(handle(), RCLLF_GATE_PUBLISH);
  ASSERT_NE(ctx, nullptr);
  rcllf_gate_post(ctx, RCL_RET_ERROR);
  auto pubs = sink.of("publication");
  ASSERT_EQ(pubs.size(), 1u);
  EXPECT_EQ(pubs[0].result, RCLLF_PUB_FAILED);
  EXPECT_EQ(sink.count("expectation"), 0u);
}

TEST_F(SemanticProducers, NestedPublishEmitsExactlyOneNestedPublication)
{
  start();
  const size_t before = sink.recs.size();
  rcllf_gate_nested(handle());
  ASSERT_EQ(sink.recs.size(), before + 1);
  EXPECT_EQ(sink.recs.back().event, "publication");
  EXPECT_EQ(sink.recs.back().origin, RCLLF_ORIGIN_NESTED);
  EXPECT_EQ(sink.recs.back().ordinal, 1u);
}

// A write made inside another write
// counts toward the invoke's ros2_lf_topic_sent, as an outer one does. A
// nested write in an invoke on another thread counts there, never here.
TEST_F(SemanticProducers, NestedPublishCountsTowardTopicSent)
{
  start();
  rt::topic_sent_reset();
  {
    rt::OriginScope o(nullptr, kPubRid);
    void * ctx = rcllf_gate_pre(handle(), RCLLF_GATE_PUBLISH);
    ASSERT_NE(ctx, nullptr);
    rcllf_gate_nested(handle());
    rcllf_gate_post(ctx, RCL_RET_OK);
  }
  int other = -1;
  std::thread t([&]() {
      rt::OriginScope o(nullptr, kPubRid);
      rcllf_gate_nested(handle());
      other = ros2_lf_topic_sent(-1, topic.c_str());
    });
  t.join();
  EXPECT_EQ(other, 1) << "a nested publish in another thread's invoke was not counted there";
  EXPECT_EQ(ros2_lf_topic_sent(-1, topic.c_str()), 2) <<
    "nested publish not counted in ros2_lf_topic_sent: the stub port is raised by "
    "the outer publish only";
}

TEST_F(SemanticProducers, UntrackedPublishEmitsNothingAndIsCounted)
{
  start();
  auto other = pub_node->create_publisher<std_msgs::msg::Int32>(topic + "_untracked", 10);
  const size_t before = sink.count("publication");
  std_msgs::msg::Int32 m;
  other->publish(m);
  other->publish(m);
  EXPECT_EQ(sink.count("publication"), before);
  rt::close_and_reconcile();
  rt::trace_close();
  EXPECT_GE(sink.last_end.untracked_publishes, 2u);
}

TEST_F(SemanticProducers, PublishRacingRunBeginIsBacklogOrEventNeverBoth)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  std::atomic<bool> go{true};
  std::atomic<int> n{0};
  std::thread t([&] {  // a busy publisher through the real gate (slot mutex path)
      while ((go.load() || n.load() < 50) && n.load() < 2000) {
        publish(n.load());
        n.fetch_add(1);
      }
    });
  while (n.load() < 20) {std::this_thread::yield();}
  rt::trace_begin();
  go.store(false);
  t.join();
  auto backlog = sink.of("backlog");
  ASSERT_EQ(backlog.size(), 1u);
  const uint64_t last = backlog[0].last_ordinal;
  auto pubs = sink.of("publication");
  EXPECT_GE(last, 20u);
  for (const auto & p : pubs) {
    EXPECT_GT(p.ordinal, last) << "ordinal in backlog and event";
                                                                                            }
  for (size_t i = 0; i < pubs.size(); ++i) {
    EXPECT_EQ(pubs[i].ordinal, last + 1 + i);
                                                                                     }
  EXPECT_EQ(last + pubs.size(), static_cast<uint64_t>(n.load()));
}

TEST_F(SemanticProducers, EpochCloseEmitsTerminalAndCleanupAndRunEndMatchesReconcile)
{
  start();
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
    publish(2);
  }
  // not dispatched: both expectations end EPOCH_CLOSE at close
  rt::close_and_reconcile();
  rt::trace_close();
  auto terms = sink.of("terminal");
  ASSERT_EQ(terms.size(), 2u);
  for (const auto & t : terms) {
    EXPECT_EQ(t.outcome, RCLLF_OUTCOME_EPOCH_CLOSE);
                                                                                }
  EXPECT_EQ(sink.count("cleanup"), 1u);
  ASSERT_EQ(sink.end_edges.size(), 1u);
  const auto c = rt::table().counts(0);
  EXPECT_EQ(sink.end_edges[0].expectations, c.expectations);
  EXPECT_EQ(sink.end_edges[0].terminal_other, 2u);
  EXPECT_EQ(sink.last_end.totals[RCLLF_CAT_PUBLICATION], 2u);
  EXPECT_EQ(sink.last_end.totals[RCLLF_CAT_DELIVERY], 4u);  // 2 expectation + 2 terminal
  EXPECT_EQ(sink.recs.back().event, "run_end");
}

TEST_F(SemanticProducers, DisabledPublicationShowsAsAttemptsWithoutTotal)
{
  sink.disabled["publication"] = true;
  start();
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
  }
  rt::close_and_reconcile();
  rt::trace_close();
  EXPECT_EQ(sink.last_end.totals[RCLLF_CAT_PUBLICATION], 0u);
  EXPECT_EQ(sink.last_end.attempts[RCLLF_CAT_PUBLICATION], 1u);
  ASSERT_FALSE(sink.end_pubs.empty());
}


}  // namespace

// --- Checked edges detect an overwrite, they do not prevent it. ----------------

TEST_F(SemanticProducers, ALocalLossIsDeliveredMissingAndNoOverwriteBeyondTheReaderDepth)
{
  // Ten publishes into a depth 8 reader while ingress is parked. Each
  // publish takes its own sample, so DDS overwrites none (the overwrite check
  // is moot on a local edge). The table's F3 hook drops ordinals 1 and 2 in transport:
  // both end LOST local_undelivered (planted) at their publish, and each
  // dispatch must deliver them missing through the deadline and message_lost
  // callbacks.
  rt::reset_for_test();
  topic += "_overwrite";
  std::vector<size_t> lost_totals, deadline_totals;
  rclcpp::SubscriptionOptions so;
  so.event_callbacks.message_lost_callback = [&](rclcpp::QOSMessageLostInfo & i) {
      lost_totals.push_back(i.total_count);
    };
  so.event_callbacks.deadline_callback = [&](rclcpp::QOSDeadlineRequestedInfo & i) {
      deadline_totals.push_back(static_cast<size_t>(i.total_count));
    };
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(16));
  sub = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(8), [this](const std_msgs::msg::Int32 & m) {
      std::lock_guard<std::mutex> lk(mu);
      heard.push_back(m.data);
    }, so);
  ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
  // Carrier and retention hold all ten records, reader depth 8, burst unchecked.
  ros2_lf_binding::register_corr_edge(
    {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0, -1, -1, 0, 16u, 16u, 8u});
  start();
  ASSERT_EQ(sub->get_actual_qos().depth(), 8u);
  // Publish only once DDS matched the writer to the reader (5 s at most).
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub->get_subscription_count() < 1 && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(pub->get_subscription_count(), 1u) << "the publisher never matched the reader";
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  {
    rt::DispatchTrace dt(kPubRid, RCLLF_ENTRY_TIMER);
    rt::OriginScope o(nullptr, kPubRid);
    for (int v = 1; v <= 10; ++v) {
      // Ordinal v is publish v of this writer; ordinal 0 is never used.
      rt::table().set_drop_sample_for_test(0, v <= 2 ? static_cast<uint64_t>(v) : 0u);
      publish(v);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ingress.pause_corr_takes_for_test(false);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (sink.count("sample") < 8u && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(dispatch_all(), (std::vector<int>{3, 4, 5, 6, 7, 8, 9, 10}));
  std::vector<uint64_t> lost_ordinals;
  for (const auto & r : sink.of("terminal")) {
    if (r.outcome == RCLLF_OUTCOME_LOST) {
      lost_ordinals.push_back(r.ordinal);
    }
  }
  EXPECT_EQ(lost_ordinals, (std::vector<uint64_t>{1, 2}))
    << "the dropped local samples were not classified LOST at their publish";
  // The first dispatch_end is the publishing timer dispatch, the rest are corr dispatches.
  const auto ends = sink.of("dispatch_end");
  ASSERT_FALSE(ends.empty());
  std::vector<uint8_t> missing_outcomes;
  std::vector<uint8_t> missing_hooks;
  EXPECT_EQ(ends[0].missing_hook, RCLLF_MISSING_HOOK_NA);
  for (size_t i = 1; i < ends.size(); ++i) {
    // The edge has no timeout (T_e -1), so no dispatch carries a cutoff.
    EXPECT_EQ(ends[i].cutoff, -1) << "dispatch_end " << i;
    if (ends[i].outcome != RCLLF_OUTCOME_MATCHED) {
      missing_outcomes.push_back(ends[i].outcome);
      missing_hooks.push_back(ends[i].missing_hook);
    } else {
      EXPECT_EQ(ends[i].missing_hook, RCLLF_MISSING_HOOK_NA) << "dispatch_end " << i;
    }
  }
  // Both hooks ran, the first one (deadline) names the dispatch.
  EXPECT_EQ(
    missing_hooks,
    (std::vector<uint8_t>{RCLLF_MISSING_HOOK_DEADLINE, RCLLF_MISSING_HOOK_DEADLINE}));
  EXPECT_EQ(missing_outcomes, (std::vector<uint8_t>{RCLLF_OUTCOME_LOST, RCLLF_OUTCOME_LOST}))
    << "the lost samples were not dispatched as LOST";
  EXPECT_EQ(lost_totals, (std::vector<size_t>{1, 2}))
    << "message_lost did not run once per lost sample: the loss passed silently";
  EXPECT_EQ(deadline_totals, (std::vector<size_t>{1, 2}));
}

// --- The logical deadline of a silent periodic edge. ------------------------

namespace
{
constexpr int64_t kMs = 1000000;
constexpr int64_t kSilenceD = 25 * kMs;  // brake_fusion's /wheel/speed deadline
}  // namespace

class SilenceDeadline : public SemanticProducers
{
protected:
  /// Rebuild edge 0 as a periodic one: both ends declare deadline D, and the
  /// subscription registers a deadline handler that records its tag and count.
  void make_periodic_edge(const std::string & suffix)
  {
    rt::reset_for_test();
    topic += suffix;
    rclcpp::SubscriptionOptions so;
    so.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo & i) {
        calls.push_back({ros2_lf_binding::lf_current_tag().first, i.total_count,
            i.total_count_change});
      };
    const auto qos = rclcpp::QoS(10).deadline(rclcpp::Duration::from_nanoseconds(kSilenceD));
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, qos);
    sub = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, qos, [this](const std_msgs::msg::Int32 & m) {
        std::lock_guard<std::mutex> lk(mu);
        heard.push_back(m.data);
      }, so);
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
    ros2_lf_binding::register_corr_edge(
      {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  }

  /// The drain at tag (t, micro), as LF runs it; returns its wake.
  int64_t drain(int64_t t, uint32_t micro)
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, micro));
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    return ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
  }

  /// The writer publishes `v` at (t, 0) (ordinal `drop` lost in transport),
  /// its edge emits and arrives it, and ingress takes it (unless dropped).
  void publish_at(int v, int64_t t, uint64_t drop)
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, 0u));
    {
      rt::DispatchTrace dt(kPubRid, RCLLF_ENTRY_TIMER);
      rt::OriginScope o(nullptr, kPubRid);
      rt::table().set_drop_sample_for_test(0, drop);
      publish(v);
    }
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    const uint32_t n = ros2_lf_corr_emit(nullptr, 0);
    if (n > 0) {
      ros2_lf_corr_arrive(nullptr, 0, n);
    }
  }

  bool matched()
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (pub->get_subscription_count() < 1 && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pub->get_subscription_count() >= 1;
  }

  bool samples_taken(size_t n)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (sink.count("sample") < n && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return sink.count("sample") >= n;
  }

public:
  struct Call
  {
    int64_t tag_ns;
    int32_t total;
    int32_t change;
    bool operator==(const Call & o) const
    {
      return tag_ns == o.tag_ns && total == o.total && change == o.change;
    }
  };
  std::vector<Call> calls;
};

std::ostream & operator<<(std::ostream & os, const SilenceDeadline::Call & c)
{
  return os << "{tag " << c.tag_ns << " ns, total " << c.total << ", change " << c.change << "}";
}

TEST_F(SilenceDeadline, ASilentPeriodicEdgeRaisesTheDeadlineAtEveryWindowFromTheStartTag)
{
  // No publish at all. The startup arm drains at the start tag (0 outside an
  // LF program); each window end is a drain at the wake the last one gave.
  make_periodic_edge("_silent");
  start();
  int64_t t = 0;
  std::vector<int64_t> wakes;
  int64_t w = drain(t, 0);
  for (int k = 0; k < 3 && w > 0; ++k) {
    wakes.push_back(w);
    t += w;
    w = drain(t, 0);
  }
  wakes.push_back(w);
  ASSERT_EQ(calls.size(), 3u)
    << "no deadline event on a silent periodic edge after 3 D (" << calls.size()
    << " calls, first drain wake " << wakes.front() << ")";
  EXPECT_EQ(
    calls, (std::vector<Call>{{kSilenceD, 1, 1}, {2 * kSilenceD, 2, 1}, {3 * kSilenceD, 3, 1}}));
  EXPECT_EQ(wakes, (std::vector<int64_t>{kSilenceD, kSilenceD, kSilenceD, kSilenceD}));
  // A silence dispatch has no record: no edge, outcome NONE, no missing hook.
  size_t silent_dispatches = 0;
  for (const auto & r : sink.of("dispatch_end")) {
    EXPECT_EQ(r.missing_hook, RCLLF_MISSING_HOOK_NA);
    ++silent_dispatches;
  }
  EXPECT_EQ(silent_dispatches, 3u);
}

TEST_F(SilenceDeadline, OneLostSampleIsReportedOnceAndAStoppedProducerAtTLastPlusD)
{
  // Period 10 ms, D 25 ms: publishes at 10, 20 (lost in transport) and 30 ms,
  // then silence. Expected: one deadline call for the loss at its tag 20 ms
  // (its own call; the record restarts the window, so no window call
  // before 30 + D), then window calls at 30 + D = 55 ms and 80 ms.
  make_periodic_edge("_stopped");
  start();
  ASSERT_TRUE(matched()) << "the publisher never matched the reader";
  EXPECT_EQ(drain(0, 0), kSilenceD);  // the startup arm: next window end at D
  std::vector<int64_t> wakes;
  size_t taken = 0;
  for (int v = 1; v <= 3; ++v) {
    const int64_t t = 10 * kMs * v;
    if (t >= kSilenceD) {
      wakes.push_back(drain(kSilenceD, 0));  // the stale wake the arm scheduled
    }
    publish_at(v, t, v == 2 ? 2u : 0u);
    taken += v == 2 ? 0 : 1;
    ASSERT_TRUE(samples_taken(taken)) << "ingress never took sample " << v;
    wakes.push_back(drain(t, 1));
  }
  int64_t t = 30 * kMs;
  int64_t w = wakes.back();
  for (int k = 0; k < 2 && w > 0; ++k) {
    t += w;
    w = drain(t, 0);
  }
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(heard, (std::vector<int>{1, 3}));
  }
  ASSERT_FALSE(calls.empty()) << "the lost sample raised no deadline call";
  EXPECT_EQ(calls.front(), (Call{20 * kMs, 1, 1}));
  size_t before_last_plus_d = 0;
  for (const auto & c : calls) {
    before_last_plus_d += c.tag_ns < 30 * kMs + kSilenceD ? 1 : 0;
  }
  EXPECT_EQ(before_last_plus_d, 1u) << "the lost sample was reported twice";
  EXPECT_EQ(
    calls, (std::vector<Call>{{20 * kMs, 1, 1}, {55 * kMs, 2, 1}, {80 * kMs, 3, 1}}))
    << "no deadline event at t_last + D after the producer stopped";
  // Records at 10, 20 and 30 ms each restart the window: the wakes point at
  // record + D, and the stale wake at 25 ms (drained before the 30 ms
  // publish) re-points at 20 + D.
  EXPECT_EQ(
    wakes, (std::vector<int64_t>{kSilenceD, kSilenceD, 20 * kMs, kSilenceD}));
}

TEST_F(SilenceDeadline, AnEdgeWithoutADeclaredDeadlineNeverWakes)
{
  // The fixture's edge: no QoS deadline, no handler. Not periodic: no window.
  start();
  EXPECT_EQ(drain(0, 0), -1) << "an undeclared edge scheduled a silence wake";
  EXPECT_EQ(drain(10 * kSilenceD, 0), -1);
  EXPECT_TRUE(calls.empty());
}

// --- A fan-in group, two writers into one reader. ------------------------------

namespace
{

constexpr int kPubRidB = 2;

std::mutex g_json_mu;
std::vector<std::string> g_json;  ///< every log line, while capture_json is installed

void capture_json(
  const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  va_list copy;
  va_copy(copy, *args);
  char buf[4096];
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  std::lock_guard<std::mutex> lk(g_json_mu);
  g_json.emplace_back(buf);
}

/// The `"edge":N` of every captured JSON line of kind `kind`, in log order.
std::string json_edges(const std::string & kind)
{
  std::lock_guard<std::mutex> lk(g_json_mu);
  std::string out;
  for (const auto & line : g_json) {
    const auto at = line.find("\"kind\":\"" + kind + "\",\"edge\":");
    if (at != std::string::npos) {
      out += (out.empty() ? "" : ",") +
        std::to_string(std::atoi(line.c_str() + line.find("\"edge\":", at) + 7));
    }
  }
  return out;
}

template<typename T>
std::string joined(const std::vector<T> & v)
{
  std::string out;
  for (const auto & x : v) {
    out += (out.empty() ? "" : ",") + std::to_string(x);
  }
  return out;
}

/// Writers Pub (edge 0) and PubB (edge 1), each on its own node, into one
/// subscription of Sub. The edges form one fan-in group led by edge 0, and
/// PubB ranks first (rank 0) as the lower sanitized instance name would.
class FanInGroup : public SemanticProducers
{
protected:
  void SetUp() override
  {
    SemanticProducers::SetUp();
    rt::reset_for_test();  // the group replaces the base's lone edge 0
    ros2_lf_binding::clear_rid_entries_for_test();
    topic += "_fanin";
    ros2_lf_binding::register_rid_entry(
      {kSubRid, "Sub", "on_data", "sub", topic, 0, 0, ros2_lf_binding::kGroupUnknown});
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
    pub_node_b = std::make_shared<rclcpp::Node>("corr_pub_b_" + std::to_string(counter));
    pub_b = pub_node_b->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
    rclcpp::SubscriptionOptions so;
    so.event_callbacks.message_lost_callback = [this](rclcpp::QOSMessageLostInfo &) {
        rcllf_lost_ctx_t ctx{};
        if (ros2_lf_message_lost_context(&ctx) == 1) {
          lost.push_back(ctx);
        }
        if (lost_sleep_ms > 0) {  // a handler that takes physical time
          std::this_thread::sleep_for(std::chrono::milliseconds(lost_sleep_ms));
        }
      };
    sub = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [this](const std_msgs::msg::Int32 & m) {
        std::lock_guard<std::mutex> lk(mu);
        heard.push_back(m.data);
      }, so);
    ros2_lf_binding::g_handle_table.resize(kPubRidB + 1);
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
  }
  void TearDown() override
  {
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    SemanticProducers::TearDown();
  }

  /// Edge 0 (Pub, rank 1, d = 0) and edge 1 (PubB, rank 0, d = `d_b`). The
  /// generator refuses unequal delays in one group; a test may register them
  /// to tell the member's delay from the lead's.
  void register_group(int64_t d_b = 0, uint32_t burst = 0)
  {
    ros2_lf_binding::register_corr_edge(
      {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0, -1, -1, burst});
    ros2_lf_binding::register_corr_edge(
      {1, "PubB", {kPubRidB}, topic, "Sub", kSubRid, topic, d_b, -1, -1, burst});
    ros2_lf_binding::set_corr_edge_fan_in(0, 0, 1);
    ros2_lf_binding::set_corr_edge_fan_in(1, 0, 0);
  }

  std::optional<rt::StartupFailure> start_group()
  {
    auto f = rt::startup(
      [this](const std::string & key) {
        return key == "Pub" ? pub_node->get_node_base_interface() :
               key == "PubB" ? pub_node_b->get_node_base_interface() :
               sub_node->get_node_base_interface();
      });
    if (!f) {
      rt::trace_begin();
    }
    return f;
  }

  /// True once DDS matched both writers to the reader (5 s at most).
  bool matched()
  {
    auto both = [this] {
        return pub->get_subscription_count() >= 1 && pub_b->get_subscription_count() >= 1;
      };
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!both() && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return both();
  }

  /// The writer of `rid` publishes `values` at tag (t, micro), then its edge
  /// emits and arrives them, as the generated reactions do.
  void publish_at(int rid, const std::vector<int> & values, int64_t t, uint32_t micro)
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, micro));
    {
      rt::OriginScope o(nullptr, rid);
      for (int v : values) {
        std_msgs::msg::Int32 m;
        m.data = v;
        (rid == kPubRidB ? pub_b : pub)->publish(m);
      }
    }
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    const int edge = rid == kPubRidB ? 1 : 0;
    const uint32_t n = ros2_lf_corr_emit(nullptr, edge);
    if (n > 0) {
      ros2_lf_corr_arrive(nullptr, edge, n);
    }
  }

  /// The group's drain (its lead's) at tag (t, micro), then at each next
  /// microstep while it wakes at 0, as LF reschedules it. Returns the first
  /// wake that is not 0, -1 with nothing pending.
  int64_t drain_at(int64_t t, uint32_t micro)
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, micro));
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    const int64_t w = ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
    return drain_wakes(std::make_pair(t, micro), w, topic, true);
  }

  bool samples_taken(size_t n)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (sink.count("sample") < n && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return sink.count("sample") >= n;
  }

  std::vector<int> delivered()
  {
    std::lock_guard<std::mutex> lk(mu);
    return heard;
  }

  std::vector<uint32_t> edges_of(const std::string & event, int op = -1)
  {
    std::vector<uint32_t> out;
    for (const auto & r : sink.of(event)) {
      if (op < 0 || r.op == op) {
        out.push_back(r.edge);
      }
    }
    return out;
  }

  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  /// Child of NonLeadDispatchCarriesMemberEdge: dispatch one token of the
  /// lead and two of the member, the first of them lost, and print what the
  /// trace, the JSON lines and the message_lost handler named.
  void dispatch_and_lose_on_the_member()
  {
    static constexpr int64_t kDB = 5000000;
    setenv("RCLLF_TRACE_CALLBACKS", "1", 1);
    if (!ros2_lf_binding::trace_callbacks_enabled()) {
      std::fprintf(stderr, "RCLLF_TRACE_CALLBACKS was read before the test set it\n");
      std::_Exit(4);
    }
    rcutils_logging_set_output_handler(capture_json);
    register_group(kDB);
    if (start_group() || !matched()) {
      std::fprintf(stderr, "the group did not start or its writers never matched\n");
      std::_Exit(5);
    }
    // PubB's first sample is lost. A seam, not RCLLF_TEST_CORR_DROP_SAMPLE:
    // that hook exists only in a -DRCLLF_CORR_TEST_HOOKS=ON build.
    rt::set_drop_sample_for_test(1, 1);
    const int64_t t = now_ns();
    publish_at(kPubRid, {1}, t, 0);
    publish_at(kPubRidB, {2, 3}, t, 0);  // ordinals 1 (dropped) and 2
    const bool taken = samples_taken(2);  // PubB's ordinal 2 shows its ordinal 1 LOST
    drain_at(t, 0);
    drain_at(t + kDB, 0);
    const int64_t t0 = std::max<int64_t>(0, ros2_lf_binding::lf_start_tag().first);
    const int64_t intended = lost.empty() ? -1 : lost[0].intended_elapsed_ns - (t - t0);
    char line[512];
    std::snprintf(
      line, sizeof(line),
      "member edge: begins=%s dispatch=%s missing_hook=%s lost=%zu intended-publish=%lld "
      "delivered=%s", joined(edges_of("dispatch_begin")).c_str(),
      json_edges("dispatch").c_str(), json_edges("missing_hook").c_str(), lost.size(),
      static_cast<long long>(intended), joined(delivered()).c_str());  // NOLINT
    std::fprintf(stderr, "%s taken=%d\n", line, taken ? 1 : 0);
    std::_Exit(
      std::string(line) == "member edge: begins=0,1,1 dispatch=0,1,1 missing_hook=1 lost=1 "
      "intended-publish=5000000 delivered=1,3" ? 0 : 1);
  }

  /// Child of AMissingHandlersPhysicalTimeIsOnItsTraceLine: lose PubB's
  /// first sample, its message_lost handler sleeps 30 ms, and print the
  /// handler_ns of the missing_hook JSON line (-1 when it has none).
  /// `traced` false: RCLLF_TRACE_CALLBACKS is
  /// unset, the handler still runs once and no missing_hook line is written.
  void lose_with_a_slow_handler(bool traced = true)
  {
    if (traced) {
      setenv("RCLLF_TRACE_CALLBACKS", "1", 1);
    } else {
      unsetenv("RCLLF_TRACE_CALLBACKS");
    }
    if (ros2_lf_binding::trace_callbacks_enabled() != traced) {
      std::fprintf(stderr, "RCLLF_TRACE_CALLBACKS was read before the test set it\n");
      std::_Exit(4);
    }
    rcutils_logging_set_output_handler(capture_json);
    lost_sleep_ms = 30;
    static constexpr int64_t kDB = 5000000;
    register_group(kDB);
    if (start_group() || !matched()) {
      std::fprintf(stderr, "the group did not start or its writers never matched\n");
      std::_Exit(5);
    }
    // PubB's first sample is lost. A seam, not RCLLF_TEST_CORR_DROP_SAMPLE:
    // that hook exists only in a -DRCLLF_CORR_TEST_HOOKS=ON build.
    rt::set_drop_sample_for_test(1, 1);
    const int64_t t = now_ns();
    publish_at(kPubRid, {1}, t, 0);
    publish_at(kPubRidB, {2, 3}, t, 0);  // ordinals 1 (dropped) and 2
    samples_taken(2);
    drain_at(t, 0);
    drain_at(t + kDB, 0);
    long long handler_ns = -1;  // NOLINT(runtime/int)
    size_t hooks = 0;
    {
      std::lock_guard<std::mutex> lk(g_json_mu);
      for (const auto & line : g_json) {
        if (line.find("\"kind\":\"missing_hook\"") == std::string::npos) {
          continue;
        }
        ++hooks;
        const auto at = line.find("\"handler_ns\":");
        if (at != std::string::npos) {
          handler_ns = std::atoll(line.c_str() + at + 13);
        }
      }
    }
    std::fprintf(stderr, "missing_hook lines=%zu lost=%zu handler_ns=%lld\n", hooks,
      lost.size(), handler_ns);
    if (!traced) {
      std::_Exit(hooks == 0 && lost.size() == 1 ? 0 : 1);
    }
    std::_Exit(hooks == 1 && lost.size() == 1 && handler_ns >= 30000000 ? 0 : 1);
  }

  rclcpp::Node::SharedPtr pub_node_b;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_b;
  std::vector<rcllf_lost_ctx_t> lost;
  int lost_sleep_ms = 0;
};

TEST_F(FanInGroup, TwoWritersIntoOneReaderDispatchInRankOrderOnTheirOwnEdges)
{
  // Both writers publish at one tag. The group's one reader takes both
  // samples, each lands on its writer's member, and the one drain dispatches
  // them in rank order, PubB (rank 0) first, each on its own edge.
  register_group();
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  const size_t before = ingress.admitted_count();
  const auto f = start_group();
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  EXPECT_EQ(ingress.admitted_count() - before, 1u)
    << "the group's one reader must be admitted once, on its lead";
  ASSERT_TRUE(matched()) << "the publishers never matched the reader";
  const int64_t t = now_ns();
  publish_at(kPubRid, {1}, t, 0);
  publish_at(kPubRidB, {2}, t, 0);
  ASSERT_TRUE(samples_taken(2)) << "ingress did not take both samples";
  EXPECT_EQ(drain_at(t, 0), -1);
  EXPECT_EQ(delivered(), (std::vector<int>{2, 1}))
    << "the group did not dispatch in rank order (PubB, rank 0, first)";
  auto sampled = edges_of("sample");
  std::sort(sampled.begin(), sampled.end());
  EXPECT_EQ(sampled, (std::vector<uint32_t>{0, 1}))
    << "a sample was not routed to its writer's member";
  EXPECT_EQ(edges_of("dispatch_begin"), (std::vector<uint32_t>{1, 0}))
    << "a dispatch did not carry its token's own edge";
  EXPECT_EQ(edges_of("carrier", RCLLF_CARRIER_DRAIN), (std::vector<uint32_t>{1, 0}));
  EXPECT_EQ(rt::table().order_violations(0), 0u);
  EXPECT_TRUE(rt::close_and_reconcile());
}

TEST_F(FanInGroup, AtZeroDelayTheEarlierPublicationMicrostepGoesFirst)
{
  // Pub (rank 1) publishes at (t, 0), PubB (rank 0) at (t, 1). At d = 0 they
  // are released at (t, 1) and (t, 2), so the release tag orders them before
  // rank.
  register_group();
  const auto f = start_group();
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  ASSERT_TRUE(matched()) << "the publishers never matched the reader";
  const int64_t t = now_ns();
  publish_at(kPubRid, {1}, t, 0);
  publish_at(kPubRidB, {2}, t, 1);
  ASSERT_TRUE(samples_taken(2)) << "ingress did not take both samples";
  EXPECT_EQ(drain_at(t, 2), -1);
  EXPECT_EQ(delivered(), (std::vector<int>{1, 2}))
    << "rank overtook the earlier publication microstep at d = 0";
  EXPECT_EQ(rt::table().order_violations(0), 0u);
}

TEST_F(FanInGroup, AZeroDelayTokenWaitsForTheNextMicrostep)
{
  // A d = 0 token published at (t, 0) is due at
  // lf_delay_tag((t, 0), 0) = (t, 1). The drain at the publication tag
  // dispatches nothing and wakes at 0; the next microstep dispatches both.
  register_group();
  const auto f = start_group();
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  ASSERT_TRUE(matched()) << "the publishers never matched the reader";
  const int64_t t = now_ns();
  publish_at(kPubRid, {1}, t, 0);
  publish_at(kPubRidB, {2}, t, 0);
  ASSERT_TRUE(samples_taken(2)) << "ingress did not take both samples";
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, 0u));
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    EXPECT_EQ(ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str()), 0);
  }
  EXPECT_TRUE(delivered().empty())
    << "a d = 0 token was dispatched at its publication tag (t, 0), not at (t, 1)";
  EXPECT_EQ(drain_at(t, 1), -1);
  EXPECT_EQ(delivered(), (std::vector<int>{2, 1}));
  EXPECT_EQ(rt::table().order_violations(0), 0u);
}

TEST_F(FanInGroup, MembersBurstsAddUpAgainstTheOneReadersDepth)
{
  // Burst 5 fits a depth 8 reader alone; two members of 5 fill it with 10.
  register_group(0, 5);
  const auto f = start_group();
  ASSERT_TRUE(f.has_value()) << "startup admitted a group burst of 10 into a depth 8 reader";
  EXPECT_EQ(f->check, "reader_depth");
  EXPECT_NE(f->detail.find("fan-in group burst 10 above reader depth 8"), std::string::npos)
    << f->detail;
}

TEST_F(FanInGroup, ARefusedDrainEndsEachMembersCarrierOnItsOwnEdge)
{
  // The group's one drain action is its lead's. When LF refuses to schedule
  // it, both members' carriers end refused, each traced on its own edge.
  register_group();
  const auto f = start_group();
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  ASSERT_TRUE(matched()) << "the publishers never matched the reader";
  const int64_t t = now_ns();
  publish_at(kPubRid, {1}, t, 0);
  publish_at(kPubRidB, {2}, t, 0);
  ASSERT_TRUE(samples_taken(2)) << "ingress did not take both samples";
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, 0u));
  ros2_lf_corr_schedule_refused(0);
  auto refused = edges_of("carrier", RCLLF_CARRIER_REFUSED);
  std::sort(refused.begin(), refused.end());
  EXPECT_EQ(refused, (std::vector<uint32_t>{0, 1}))
    << "a member's carrier ended refused without a trace on its own edge";
  EXPECT_TRUE(delivered().empty());
}

TEST_F(FanInGroup, NonLeadDispatchCarriesMemberEdge)
{
  // The member's dispatch (dispatch_begin, the dispatch and missing_hook
  // JSON) names edge 1, not the lead whose drain runs it, and the lost
  // context's intended time adds the member's delay (5 ms), not the lead's.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    dispatch_and_lose_on_the_member(), ::testing::ExitedWithCode(0),
    "member edge: begins=0,1,1 dispatch=0,1,1 missing_hook=1 lost=1 intended-publish=5000000 "
    "delivered=1,3");
}

TEST_F(FanInGroup, AMissingHandlersPhysicalTimeIsOnItsTraceLine)
{
  // Owner requirement (d): the missing_hook trace line carries
  // the physical time its handlers ran. The message_lost handler sleeps
  // 30 ms, so handler_ns is at least 30 ms.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    lose_with_a_slow_handler(), ::testing::ExitedWithCode(0),
    "missing_hook lines=1 lost=1 handler_ns=[0-9]+");
}

TEST_F(FanInGroup, AMissingHandlerRunsUntimedWithoutTheCallbackTrace)
{
  // With RCLLF_TRACE_CALLBACKS unset the
  // handler still runs once, and no missing_hook line (and no handler timing)
  // is produced.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    lose_with_a_slow_handler(false), ::testing::ExitedWithCode(0),
    "missing_hook lines=0 lost=1 handler_ns=-1");
}

}  // namespace

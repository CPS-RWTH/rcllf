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

/// @file test_publication_gate.cpp
/// @brief Gate origins, startup checks, ingress classification and typed
///        dispatch of the correspondence runtime (T3-T6). The
///        executable gate (src/publication_gate.cpp) is linked into this test.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "corr_runtime.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "rcl/subscription.h"
#include "rid_table.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/publication_gate_api.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rclcpp/serialization.hpp"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "std_msgs/msg/int32.hpp"
#include "thread_role.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{
extern std::vector<HandleEntry> g_handle_table;
}

namespace rt = ros2_lf_binding::corr_rt;
using ros2_lf_binding::corr::Outcome;

namespace
{

constexpr int kPubRid = 0;
constexpr int kSubRid = 1;
// Group column of the RID entries: these tests bind no handles.
constexpr int kAnyGroup = ros2_lf_binding::kGroupUnknown;

/// Drain edge 0 from the wake `w` its arrive returned until none is left, each
/// drain at the tag the generated lf_schedule(drain, w) gives: the next
/// microstep for w = 0, else (t + w, 0). A zero-delay token is due one
/// microstep after its publication. The caller's tag
/// is restored; {-1, 0} is no LF program, so no override was set.
void drain_wakes(int64_t w, const std::string & topic)
{
  const auto start = ros2_lf_binding::lf_current_tag();
  auto tag = start;
  while (w >= 0) {
    tag = w == 0 ? std::make_pair(tag.first, tag.second + 1) : std::make_pair(tag.first + w, 0u);
    ros2_lf_binding::set_lf_tag_for_test(tag);
    w = ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
  }
  ros2_lf_binding::set_lf_tag_for_test(
    start == std::pair<int64_t, uint32_t>(-1, 0) ?
    std::nullopt : std::make_optional(start));
}

class CorrRuntime : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // T_e comes from the fixture's take_timeout_ms. The compose env sets
    // ROS2_LF_TAKE_TIMEOUT_MS=100, which overrides it and pushes the samples
    // of tests that sleep past their cutoff (LATE).
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_TAKE_MODE");
    rclcpp::init(0, nullptr);
  }
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
    ros2_lf_binding::clear_rid_entries_for_test();
    pub_node = std::make_shared<rclcpp::Node>("corr_pub_" + std::to_string(++counter));
    sub_node = std::make_shared<rclcpp::Node>("corr_sub_" + std::to_string(counter));
    // One topic per process, so concurrent runs in one domain never meet.
    topic = "/corr_test_" + std::to_string(getpid()) + "_" + std::to_string(counter);
    // The edge's subscription is the first the node creates on its topic.
    ros2_lf_binding::register_rid_entry({kSubRid, "Sub", "on_data", "sub", topic, 0, 0, kAnyGroup});
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

  /// Edge 0 on `t` whose publisher runs in another federate (the
  /// ingress side, no publishing RID here, d = 5 ms, peer named by node).
  ros2_lf_binding::CorrEdgeSpec cross_federate_spec(const std::string & t)
  {
    ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {}, t, "Sub", kSubRid, t, 5000000};
    s.role = ros2_lf_binding::CorrEdgeRole::kIngress;
    s.peer_node = pub_node->get_fully_qualified_name();
    return s;
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
      drain_wakes(ros2_lf_corr_arrive(nullptr, 0, n), topic);
    }
    std::lock_guard<std::mutex> lk(mu);
    return heard;
  }

  /// One writer publishes twice at one tag on a depth-1 edge while
  /// ingress takes nothing (preempted in the refsys runs, paused here), then
  /// both records are dispatched. Returns the reader depth DDS gave.
  size_t race_two_publishes_on_a_depth_one_edge()
  {
    rt::reset_for_test();
    topic += "_depth1";
    ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(1));
    {
      Ros2LfNodeConstructionScope scope("Sub");  // as the generated node factory does
      sub = sub_node->create_subscription<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(1), [this](const std_msgs::msg::Int32 & m) {
          std::lock_guard<std::mutex> lk(mu);
          heard.push_back(m.data);
        });
    }
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
    if (rt::startup(resolver()).has_value()) {
      return 0;
    }
    if (!publisher_matched(pub)) {
      ADD_FAILURE() << "the publisher never matched the reader, so no publish could race";
      return 0;
    }
    auto & ingress = ros2_lf_binding::IngressRole::instance();
    ingress.pause_corr_takes_for_test(true);  // returns once no take loop is in flight
    {
      ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
      rt::OriginScope o(nullptr, kPubRid);
      publish(1);
      publish(2);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // both reach the reader
    ingress.pause_corr_takes_for_test(false);
    dispatch_all();
    return sub->get_actual_qos().depth();
  }

  /// One publish of `v` while ingress takes nothing, as a starved
  /// ingress thread on one loaded CPU, then the dispatch. With `self` one node
  /// publishes and subscribes (a local self edge). T is 1 ms and the dispatch
  /// runs 50 ms after the publish, so past the cutoff tag + d + T.
  void dispatch_while_ingress_is_starved(bool self, int v)
  {
    publish_while_ingress_is_starved(self, v);
    if (::testing::Test::HasFatalFailure()) {
      return;
    }
    dispatch_all();
    ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
  }

  /// The publish of dispatch_while_ingress_is_starved, ingress left paused.
  /// `older` samples of a second writer reach the reader (depth `depth`) first.
  void publish_while_ingress_is_starved(bool self, int v, int older = 0, size_t depth = 10)
  {
    rt::reset_for_test();
    topic += self ? "_self" : "_other";
    ros2_lf_binding::register_corr_edge(
      {0, self ? "Sub" : "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
    pub = (self ? sub_node : pub_node)->create_publisher<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(depth));
    {
      Ros2LfNodeConstructionScope scope("Sub");  // as the generated node factory does
      sub = sub_node->create_subscription<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(depth), [this](const std_msgs::msg::Int32 & m) {
          std::lock_guard<std::mutex> lk(mu);
          heard.push_back(m.data);
        });
    }
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
    ros2_lf_binding::g_handle_table[kSubRid].take_timeout_ms = 1;
    ASSERT_FALSE(rt::startup(resolver()).has_value());
    ASSERT_TRUE(publisher_matched(pub)) << "no publish could reach the reader";
    auto & ingress = ros2_lf_binding::IngressRole::instance();
    ingress.pause_corr_takes_for_test(true);  // returns once no take loop is in flight
    if (older > 0) {
      // A second writer, kept alive, and every older sample acknowledged
      // before the self sample, so none can arrive after it.
      extra = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(depth));
      ASSERT_TRUE(publisher_matched(extra)) << "the older samples could not reach the reader";
      std_msgs::msg::Int32 m;
      for (int i = 0; i < older; ++i) {
        m.data = 1000 + i;
        extra->publish(m);
      }
      ASSERT_TRUE(extra->wait_for_all_acked(std::chrono::seconds(5))) <<
        "the older samples did not reach the reader";
    }
    {
      ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
      rt::OriginScope o(nullptr, kPubRid);
      publish(v);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  /// Waits up to 5 s for DDS to match the publisher to a reader, so a
  /// publish right after cannot be dropped for want of a reader.
  static bool publisher_matched(const rclcpp::PublisherBase::SharedPtr & p)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (p->get_subscription_count() < 1) {
      if (std::chrono::steady_clock::now() > until) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  /// An admitted edge whose reader really runs with `reader_qos`
  /// (a scenario overlay or a ROS parameter chose it) while the analysis saw
  /// `analyzed`. Returns the startup stderr and whether startup passed.
  std::pair<std::string, bool> start_with_reader_reliability(
    const std::string & suffix, const std::string & analyzed, const rclcpp::QoS & reader_qos,
    int analyzed_depth = -1)
  {
    rt::reset_for_test();
    const std::string t = topic + suffix;
    auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(10));
    auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
      t, reader_qos, [](const std_msgs::msg::Int32 &) {});
    ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, t, "Sub", kSubRid, t, 0};
    spec.sub_reliability = analyzed;
    spec.sub_depth = analyzed_depth;
    ros2_lf_binding::register_corr_edge(spec);
    ::testing::internal::CaptureStderr();
    const bool ok = !rt::startup(resolver()).has_value();
    return {::testing::internal::GetCapturedStderr(), ok};
  }

  static int counter;
  rclcpp::Node::SharedPtr pub_node, sub_node;
  std::string topic;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr extra;  ///< older samples
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
  std::mutex mu;
  std::vector<int> heard;
};
int CorrRuntime::counter = 0;

TEST_F(CorrRuntime, AdmittedOriginDispatchesTheExactSampleOnce)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(11);
    publish(12);
    publish(13);
  }
  EXPECT_EQ(rt::table().in_flight(0), 3u);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{11, 12, 13}));
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 3u);
}

TEST_F(CorrRuntime, RefusedOriginsCreateNoExpectationAndTheirSampleIsUnsupported)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  std::thread helper([this]() {publish(99);});  // no_context
  helper.join();
  {
    rt::OriginScope o(nullptr, 5);  // a RID without this topic: undeclared_effect
    publish(98);
  }
  {
    rt::OriginScope o(nullptr, kPubRid);  // serialized: unsupported_api
    rclcpp::Serialization<std_msgs::msg::Int32> ser;
    rclcpp::SerializedMessage raw;
    std_msgs::msg::Int32 m;
    m.data = 97;
    ser.serialize_message(&m, &raw);
    pub->publish(raw);
  }
  EXPECT_EQ(rt::table().in_flight(0), 0u);
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
  }
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));  // the stale samples never reach it
  const auto c = rt::table().counts(0);
  EXPECT_EQ(c.samples.at(ros2_lf_binding::corr::SampleOutcome::UNSUPPORTED), 3u);
}

/// An edge of a lifecycle publisher (`pub_rebind`) whose node
/// creates the publisher after startup, as `on_configure` of a node brought
/// up later does. The late publisher predicts its records like a startup one.
TEST_F(CorrRuntime, ALifecyclePublisherCreatedAfterStartupIsBound)
{
  rt::reset_for_test();
  pub.reset();  // the node has no publisher on the topic at startup
  ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
  spec.pub_rebind = true;
  ros2_lf_binding::register_corr_edge(spec);
  const auto f = rt::startup(resolver());
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  ASSERT_TRUE(publisher_matched(pub));
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(21);
    publish(22);
  }
  EXPECT_EQ(rt::table().in_flight(0), 2u) << "the late publisher predicted no record";
  EXPECT_EQ(dispatch_all(), (std::vector<int>{21, 22}));
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 2u);
}

/// A cleanup and configure cycle replaces the publisher. The new
/// one is bound, so its samples are joined at their own tags.
TEST_F(CorrRuntime, AReplacedLifecyclePublisherIsBoundAgain)
{
  rt::reset_for_test();
  ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
  spec.pub_rebind = true;
  ros2_lf_binding::register_corr_edge(spec);
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
  }
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));
  pub.reset();  // on_cleanup
  pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));  // on_configure
  ASSERT_TRUE(publisher_matched(pub));
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(2);
  }
  EXPECT_EQ(rt::table().in_flight(0), 1u) << "the replacing publisher predicted no record";
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1, 2}));
}

/// Fixture: a `sub_rebind` edge whose node has no subscription at
/// startup, as a lifecycle subscriber that creates it in on_configure.
class LifecycleReader : public CorrRuntime
{
protected:
  void SetUp() override
  {
    CorrRuntime::SetUp();
    rt::reset_for_test();
    sub.reset();
    ros2_lf_binding::g_handle_table[kSubRid].subscription.reset();
    ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
    spec.sub_rebind = true;
    ros2_lf_binding::register_corr_edge(spec);
  }

  void TearDown() override
  {
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    CorrRuntime::TearDown();
  }

  /// LF tag `us` microseconds after the fixture start, near physical time so
  /// the worker lag bound holds.
  void at(int64_t us)
  {
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t0 + us * 1000, 0u));
  }

  /// on_configure at tag `us`: create the reader and bind it to the RID,
  /// as bind_subscription does at the drain.
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr configure(int64_t us)
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(us);
    auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [this](const std_msgs::msg::Int32 & m) {
        std::lock_guard<std::mutex> lk(mu);
        heard.push_back(m.data);
      });
    ros2_lf_binding::g_handle_table[kSubRid].subscription = s;
    rt::on_subscription_bound(kSubRid, s);
    return s;
  }

  /// A publish of `v` at tag `us`.
  void publish_at(int64_t us, int v)
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(us);
    rt::OriginScope o(nullptr, kPubRid);
    publish(v);
  }

  const int64_t t0 = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
};

/// The reader does not exist at startup. Startup defers it, and the
/// reader created later is checked and admitted, so its samples are joined.
TEST_F(LifecycleReader, AReaderCreatedAfterStartupIsAdmittedAndItsSamplesMatch)
{
  const auto f = rt::startup(resolver());
  ASSERT_FALSE(f.has_value()) << "startup refused the deferred reader: " << f->check << ": " <<
    f->detail;
  sub = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(200, 21);
  publish_at(200, 22);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{21, 22}));
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 2u);
  EXPECT_TRUE(rt::close_and_reconcile());
}

/// Cleanup and configure replace the reader. Records the
/// old reader got at or before the new reader's creation tag end exactly once
/// READER_REPLACED and are never delivered. Later records match the new reader.
TEST_F(LifecycleReader, RecordsOfAReplacedReaderEndReaderReplacedOnce)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  sub = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(110, 1);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));
  publish_at(120, 2);  // between cleanup and the new reader
  publish_at(120, 3);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // ingress may take them
  sub.reset();  // on_cleanup
  ros2_lf_binding::g_handle_table[kSubRid].subscription.reset();
  sub = configure(130);  // on_configure: generation 2
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(140, 4);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1, 4})) << "a replaced record was delivered";
  const auto c = rt::table().counts(0);
  EXPECT_EQ(c.outcomes.at(Outcome::READER_REPLACED), 2u);
  EXPECT_EQ(c.outcomes.at(Outcome::MATCHED), 2u);
  EXPECT_EQ(rt::table().in_flight(0), 0u);
  EXPECT_TRUE(rt::close_and_reconcile()) << "READER_REPLACED did not balance";
}

/// Two live subscriptions of the node on the topic and no
/// proof which one the RID names. The binding refuses with a typed code and
/// never guesses; the records end UNBOUND and reconcile fails.
TEST_F(LifecycleReader, TwinReadersWithoutAProvenIdentityAreRefused)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto twin = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ::testing::internal::CaptureStderr();
  sub = configure(100);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("refused its new reader, rebind_identity"), std::string::npos) << err;
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(200, 5);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{})) << "a refused reader was delivered to";
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::UNBOUND], 1u);
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::close_and_reconcile());
  const std::string rec = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(rec.find("reader_rebind_refused"), std::string::npos) << rec;
}

/// `sub_ = create_subscription(...)` in on_configure makes
/// the new reader while the node still holds the old one. Its check waits
/// until the old one is released, then it is admitted (generation 2) at the
/// next reaction of the RID. Before the first fix both counted as live
/// subscriptions and the edge was refused.
TEST_F(LifecycleReader, AReaderCreatedBeforeTheOldOneIsReleasedIsAdmitted)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(110, 1);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));
  ::testing::internal::CaptureStderr();
  sub = configure(130);  // the node still holds `first`
  first.reset();  // the assignment releases it
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(135);  // the next reaction of the RID (lifecycle_admit)
    rt::retry_deferred_reader(kSubRid);
  }
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("refused its new reader"), std::string::npos) << err;
  EXPECT_NE(err.find("admitted reader generation 2"), std::string::npos) << err;
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(140, 4);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1, 4}));
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 2u);
  EXPECT_TRUE(rt::close_and_reconcile());
}

/// A hook creates an unrelated subscription on the topic
/// while the node keeps the old reader. Excluding the old reader from the
/// count made the twin look like its replacement and it was admitted. Now its
/// check waits for the old reader's release, and at a later tag the old
/// reader still lives, so the twin is refused with rebind_identity.
TEST_F(LifecycleReader, ATwinCreatedWhileTheOldReaderLivesIsRefused)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(110, 1);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));
  ::testing::internal::CaptureStderr();
  auto twin = configure(130);  // `first` stays the node's reader
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(140);  // the next reaction of the RID
    rt::retry_deferred_reader(kSubRid);
  }
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("admitted reader generation 2"), std::string::npos) << err;
  EXPECT_NE(err.find("refused its new reader, rebind_identity"), std::string::npos) << err;
  EXPECT_FALSE(ros2_lf_binding::IngressRole::instance().has_corr_reader_for_test(0))
    << "the refused edge kept taking through its old reader";
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::close_and_reconcile());
  const std::string rec = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(rec.find("reader_rebind_refused"), std::string::npos) << rec;
}

/// A sample published while the new reader waits
/// for the old one's release stays in the new reader's DDS queue. Ingress
/// takes it once the reader is admitted, and it matches; the old reader's copy
/// of it is STALE_READER.
TEST_F(LifecycleReader, APendingReadersSampleIsTakenAfterAdmission)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  publish_at(110, 1);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1}));
  sub = configure(130);  // deferred: the node still holds `first`
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub->get_subscription_count() < 2 && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(pub->get_subscription_count(), 2u);
  publish_at(132, 3);  // while the new reader is pending
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  first.reset();
  ::testing::internal::CaptureStderr();
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(135);
    rt::retry_deferred_reader(kSubRid);
  }
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("admitted reader generation 2"), std::string::npos) << err;
  EXPECT_EQ(dispatch_all(), (std::vector<int>{1, 3}))
    << "the sample queued while the reader was pending was not taken";
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 2u);
  EXPECT_TRUE(rt::close_and_reconcile());
}

/// The settle point is the next tag, microstep
/// included. A twin created at (130 ms, 0) next to a reader the node keeps is
/// refused at (130 ms, 1). Comparing only the time left it pending there.
TEST_F(LifecycleReader, ATwinIsRefusedAtTheNextMicrostep)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ::testing::internal::CaptureStderr();
  auto twin = configure(130);  // `first` stays the node's reader
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t0 + 130 * 1000, 1u));
    rt::retry_deferred_reader(kSubRid);
  }
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("refused its new reader, rebind_identity"), std::string::npos)
    << "the twin stayed pending at the next microstep: " << err;
  EXPECT_FALSE(rt::deferred_reader_pending());
  EXPECT_FALSE(rt::close_and_reconcile());
}

/// A second new reader B while A still waits for
/// the reader it replaces. Two candidates cannot be proven the RID's, so B
/// refuses the edge at once. Later B is dropped and the old reader released:
/// nothing is admitted and nothing stays pending. Before the fix B overwrote
/// A, the retry saw B expired and forgot the edge, leaving A neither admitted
/// nor refused.
TEST_F(LifecycleReader, ASecondReaderWhileOneIsDeferredIsRefused)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ::testing::internal::CaptureStderr();
  auto a = configure(130);  // deferred: `first` lives
  auto b = configure(130);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("refused its new reader, rebind_identity: a second new reader"),
    std::string::npos) << err;
  b.reset();
  first.reset();
  ::testing::internal::CaptureStderr();
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(135);
    rt::retry_deferred_reader(kSubRid);
  }
  const std::string later = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(later.find("admitted reader generation 2"), std::string::npos) << later;
  EXPECT_FALSE(rt::deferred_reader_pending()) << "a candidate was left pending";
  EXPECT_FALSE(ros2_lf_binding::IngressRole::instance().has_corr_reader_for_test(0));
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::close_and_reconcile());
  const std::string rec = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(rec.find("reader_rebind_refused"), std::string::npos) << rec;
}

/// Ingress cannot admit the new reader (no C++
/// introspection type support). The edge stops, and the reader it had is
/// retired with it, so nothing is taken through it any more. Before the fix
/// the edge was marked refused and ingress kept its old reader.
TEST_F(LifecycleReader, AReaderIngressCannotAdmitStopsTheEdgeAndRetiresTheOldReader)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  auto first = configure(100);
  ASSERT_TRUE(ingress.has_corr_reader_for_test(0));
  first.reset();  // on_cleanup
  ros2_lf_binding::g_handle_table[kSubRid].subscription.reset();
  ingress.refuse_corr_admit_for_test(true);
  ::testing::internal::CaptureStderr();
  sub = configure(130);
  const std::string err = ::testing::internal::GetCapturedStderr();
  ingress.refuse_corr_admit_for_test(false);
  EXPECT_NE(err.find("ingress could not admit reader generation 2"), std::string::npos) << err;
  EXPECT_FALSE(ingress.has_corr_reader_for_test(0)) << "the stopped edge kept its old reader";
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::close_and_reconcile());
  const std::string rec = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(rec.find("reader_rebind_refused"), std::string::npos) << rec;
}

/// The take race: the next reader is created at tag 130 and
/// bound only at a later drain, so until then ingress still takes from the old
/// reader. It takes record 7 of tag 140, which is the new reader's. That
/// sample is STALE_READER, whichever reader the table has bound. The new
/// reader missed the record (a discovery gap, simulated by taking its copy
/// away), so the record ends LOST through the ordinary missing path once the
/// new reader's next sample proves the gap, exactly once. Before the fix the
/// old reader's sample matched and record 7 was delivered.
TEST_F(LifecycleReader, AReplacedReadersSampleOfALaterRecordIsStaleAndTheRecordEndsLost)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto first = configure(100);
  ASSERT_TRUE(publisher_matched(pub));
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr second;
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(130);  // on_configure: created now, bound at a later drain
    second = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [this](const std_msgs::msg::Int32 & m) {
        std::lock_guard<std::mutex> lk(mu);
        heard.push_back(m.data);
      });
  }
  auto wait_for = [](const std::function<bool()> & ok) {
      const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      bool done = ok();  // once per poll: a take consumes
      while (!done && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        done = ok();
      }
      return done;
    };
  ASSERT_TRUE(wait_for([this] {return pub->get_subscription_count() == 2;}));
  publish_at(140, 7);
  ASSERT_TRUE(wait_for([] {return rt::table().counts(0).samples_taken == 1;}))
    << "the old reader never took record 7";
  {
    std_msgs::msg::Int32 m;
    rclcpp::MessageInfo info;
    ASSERT_TRUE(wait_for([&] {return second->take_type_erased(&m, info);}));
    ASSERT_EQ(m.data, 7);
  }
  first.reset();  // on_configure's assignment
  ros2_lf_binding::g_handle_table[kSubRid].subscription = second;
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(150);  // the drain that binds it
    rt::on_subscription_bound(kSubRid, second);
  }
  publish_at(160, 8);
  ASSERT_TRUE(wait_for([] {return rt::table().counts(0).samples_taken == 2;}));
  EXPECT_EQ(dispatch_all(), (std::vector<int>{8}))
    << "record 7 was delivered with the sample the replaced reader took";
  auto c = rt::table().counts(0);
  EXPECT_EQ(c.samples[ros2_lf_binding::corr::SampleOutcome::STALE_READER], 1u);
  EXPECT_EQ(c.outcomes[Outcome::LOST], 1u);
  EXPECT_EQ(c.outcomes[Outcome::MATCHED], 1u);
  EXPECT_EQ(c.outcomes[Outcome::READER_REPLACED], 0u);
  EXPECT_EQ(rt::table().in_flight(0), 0u);
  EXPECT_TRUE(rt::close_and_reconcile());
}

/// A reader created off an LF worker while LF runs has no
/// tag that orders it against the publishes, so its cut would fence nothing.
/// It is refused with a typed code and the reconcile fails.
TEST_F(LifecycleReader, AReaderCreatedOffAnLfWorkerWhileLfRunsIsRefused)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  rt::set_lf_running_for_test(true);
  sub = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
  ::testing::internal::CaptureStderr();
  rt::on_subscription_bound(kSubRid, sub);
  const std::string err = ::testing::internal::GetCapturedStderr();
  rt::set_lf_running_for_test(std::nullopt);
  EXPECT_NE(err.find("refused its new reader, rebind_untagged"), std::string::npos) << err;
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::close_and_reconcile());
  const std::string rec = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(rec.find("reader_rebind_refused"), std::string::npos) << rec;
}

/// A lone lifecycle reader keeps logical rank 0 at every
/// creation, so each re-created reader of depth 1 is deepened to the edge's
/// reader depth and passes the bind's depth check.
TEST_F(LifecycleReader, EveryRecreatedLoneReaderIsDeepened)
{
  {
    Ros2LfNodeConstructionScope scope("Sub");  // as the generated node factory does
    sub_node = std::make_shared<rclcpp::Node>("corr_rebind_depth_" + std::to_string(counter));
  }
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  for (int life = 0; life < 3; ++life) {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    at(100 + 30 * life);
    sub = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
    EXPECT_EQ(sub->get_actual_qos().depth(), 8u) << "reader " << life << " kept depth 1";
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
    ::testing::internal::CaptureStderr();
    rt::on_subscription_bound(kSubRid, sub);
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(err.find("refused its new reader"), std::string::npos) << life << ": " << err;
    sub.reset();  // on_cleanup
  }
  EXPECT_TRUE(rt::close_and_reconcile());
}

/// Retiring a reader that is not the last admitted entry.
/// remove_if left the tail moved-from, so the fence locked a null mutex (a
/// crash) and the retired reader's waitable stayed in the wait set. Edge 5's
/// reader is retired before edge 6's: the fence waits for edge 5's take only,
/// edge 6 keeps its reader, and edge 5's rcl handle is released.
TEST_F(CorrRuntime, RetiringAReaderBeforeAnotherEdgesReaderFencesAndRemovesItsOwn)
{
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  auto cb = [](const std_msgs::msg::Int32 &) {};
  auto a = sub_node->create_subscription<std_msgs::msg::Int32>(topic + "_a", 10, cb);
  auto b = sub_node->create_subscription<std_msgs::msg::Int32>(topic + "_b", 10, cb);
  ASSERT_TRUE(ingress.replace_corr_reader(5, a, 0));
  ASSERT_TRUE(ingress.replace_corr_reader(6, b, 0));
  std::weak_ptr<rcl_subscription_t> a_rcl = a->get_subscription_handle();
  std::promise<void> a_held;
  std::thread taker([&] {
      auto hold_a = ingress.hold_corr_take_for_test(5);  // edge 5's take in flight
      a_held.set_value();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    });
  a_held.get_future().wait();
  const auto t0 = std::chrono::steady_clock::now();
  ingress.retire_corr_reader(5);
  const auto waited = std::chrono::steady_clock::now() - t0;
  taker.join();
  EXPECT_GE(waited, std::chrono::milliseconds(200)) << "the fence did not wait for edge 5's take";
  EXPECT_FALSE(ingress.has_corr_reader_for_test(5));
  EXPECT_TRUE(ingress.has_corr_reader_for_test(6)) << "edge 6's reader was retired instead";
  a.reset();  // the node drops it
  EXPECT_TRUE(a_rcl.expired()) << "edge 5's waitable stayed in the wait set";
  ingress.retire_corr_reader(6);
}

/// The gate on_init writes a slot's node name and durability
/// under the slot mutex only, so the startup scan must read them under it too.
/// The hook holds that mutex in on_init, before the writes, until startup
/// returns or 2 s pass. A scan that reads without the mutex finishes inside
/// that window, which is the data race. The late publisher is bound either way.
TEST_F(CorrRuntime, StartupReadsAPublisherOnlyAfterItsGateInitWroteIt)
{
  rt::reset_for_test();
  pub.reset();
  ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
  spec.pub_rebind = true;
  ros2_lf_binding::register_corr_edge(spec);
  std::atomic<bool> in_hook{false}, started{false}, started_in_hook{false};
  rt::set_pub_meta_hook_for_test([&] {
      if (in_hook.exchange(true)) {
        return;
      }
      const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (!started.load() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      started_in_hook = started.load();
    });
  std::thread maker([this] {
      pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
    });
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!in_hook.load() && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool hooked = in_hook.load();
  const auto f = rt::startup(resolver());
  started = true;
  maker.join();
  rt::set_pub_meta_hook_for_test({});
  ASSERT_TRUE(hooked) << "the publisher init never reached the gate on_init";
  ASSERT_FALSE(f.has_value()) << f->check << ": " << f->detail;
  EXPECT_FALSE(started_in_hook.load())
    << "data race: startup scanned the publishers while gate on_init held the slot mutex "
    "to write node_fqn, so the scan read slot metadata without slot->m";
  ASSERT_TRUE(publisher_matched(pub));
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(31);
  }
  EXPECT_EQ(rt::table().in_flight(0), 1u) << "the concurrent late publisher was not bound";
  EXPECT_EQ(dispatch_all(), (std::vector<int>{31}));
}

/// Without `pub_rebind` a missing publisher still refuses startup.
TEST_F(CorrRuntime, APlainEdgeWithoutItsPublisherStillRefusesStartup)
{
  rt::reset_for_test();
  pub.reset();
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  const auto f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "endpoint");
}

TEST_F(CorrRuntime, LoanedPublishIsAdmittedAndMatched)
{
  // Without a real loan this would test the ordinary path again.
  if (!pub->can_loan_messages()) {
    GTEST_SKIP() << "middleware cannot loan std_msgs/Int32 here";
  }
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  {
    rt::OriginScope o(nullptr, kPubRid);
    for (int v : {21, 22}) {
      auto loaned = pub->borrow_loaned_message();
      loaned.get().data = v;
      pub->publish(std::move(loaned));
    }
    publish(23);  // an ordinary write between loans keeps the ordinal in step
  }
  EXPECT_EQ(rt::table().in_flight(0), 3u);
  EXPECT_EQ(dispatch_all(), (std::vector<int>{21, 22, 23}));
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::MATCHED], 3u);
}

TEST_F(CorrRuntime, LoggingInsideAnOriginScopeDoesNotDeadlock)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  rt::OriginScope o(nullptr, kPubRid);
  for (int i = 0; i < 5; ++i) {
    RCLCPP_INFO(pub_node->get_logger(), "log before publish %d", i);  // rosout through the gate
    publish(i);
  }
  EXPECT_EQ(rt::table().in_flight(0), 5u);
}

TEST_F(CorrRuntime, PublishAfterCloseIsCountedAndTouchesNothing)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  EXPECT_TRUE(rt::close_and_reconcile());
  const uint64_t before = rt::after_close_publishes();
  rt::OriginScope o(nullptr, kPubRid);
  publish(5);
  EXPECT_GT(rt::after_close_publishes(), before);
  EXPECT_EQ(rt::table().in_flight(0), 0u);
}

// LF orders the reactions of different reactors at one tag only by
// dependencies, and a close reaction has no effects. A reaction of another
// reactor may publish at the stop tag after the last edge closed, so the close
// only marks the edge and the one reconcile runs at runtime shutdown.
TEST_F(CorrRuntime, APublishAfterTheLastEdgeCloseIsStillReconciled)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  const uint64_t before = rt::after_close_publishes();
  ros2_lf_corr_close(0);  // the only admitted edge, so the last one
  {
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    rt::OriginScope o(nullptr, kPubRid);
    publish(21);  // another reactor's reaction at the stop tag
  }
  EXPECT_EQ(rt::after_close_publishes(), before) <<
    "the publish after the last edge close was counted only in after_close";
  EXPECT_EQ(rt::table().counts(0).expectations, 1u) <<
    "the publish after the last edge close is missing from the reconcile";
  EXPECT_TRUE(rt::close_and_reconcile());
}

TEST_F(CorrRuntime, UnboundSubscriptionRidEndsUnbound)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  ros2_lf_binding::g_handle_table[kSubRid] = ros2_lf_binding::HandleEntry{};
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(3);
  }
  EXPECT_TRUE(dispatch_all().empty());
  EXPECT_EQ(rt::table().counts(0).outcomes[Outcome::UNBOUND], 1u);
}

// Two publishing reactions of one reactor write one corr port at
// one tag. The second emit sees both reactions' records, so its value (the
// one LF keeps) is the whole count and arrive matches in_flight exactly.
TEST_F(CorrRuntime, TwoPublishingRidsShareOneEdgeInPublishOrder)
{
  constexpr int kPubRid2 = 2;
  rt::reset_for_test();
  ros2_lf_binding::register_corr_edge(
    {0, "Pub", {kPubRid, kPubRid2}, topic, "Sub", kSubRid, topic, 0});
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
  {
    rt::OriginScope o(nullptr, kPubRid);
    publish(1);
  }
  EXPECT_EQ(ros2_lf_corr_emit(nullptr, 0), 1u);
  {
    rt::OriginScope o(nullptr, kPubRid2);
    publish(2);
    publish(3);
  }
  {
    rt::OriginScope o(nullptr, 7);  // not a publishing RID of the edge: undeclared_effect
    publish(99);
  }
  const uint32_t n = ros2_lf_corr_emit(nullptr, 0);
  EXPECT_EQ(n, 3u);
  drain_wakes(ros2_lf_corr_arrive(nullptr, 0, n), topic);
  {
    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(heard, (std::vector<int>{1, 2, 3}));
  }
  const auto c = rt::table().counts(0);
  EXPECT_EQ(c.outcomes.at(Outcome::MATCHED), 3u);
  EXPECT_EQ(c.count_mismatch, 0u);
}

TEST_F(CorrRuntime, DepthOneAdmittedReaderKeepsEverySameTagPublish)
{
  const size_t depth = race_two_publishes_on_a_depth_one_edge();
  EXPECT_EQ(depth, 8u) << "the admitted edge's reader kept the user's depth 1";
  const auto c = rt::table().counts(0);
  const uint64_t lost = c.outcomes.count(Outcome::LOST) ? c.outcomes.at(Outcome::LOST) : 0;
  EXPECT_EQ(lost, 0u) << "edge 0 writer ordinal 1 ended LOST: the second publish at the same "
    "tag overwrote it in the depth-" << depth << " reader before ingress took it";
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_EQ(heard, (std::vector<int>{1, 2}));
}

TEST_F(CorrRuntime, WithoutDeepeningTheDepthOneLocalReaderStillKeepsEveryPublish)
{
  // The publish-time take
  // empties a local reader before rcl_publish's caller can publish again, so
  // the reader depth no longer bounds a local edge.
  rt::set_reader_depth_for_test(0);
  const size_t depth = race_two_publishes_on_a_depth_one_edge();
  rt::set_reader_depth_for_test(8);
  EXPECT_EQ(depth, 1u);
  const auto c = rt::table().counts(0);
  EXPECT_EQ(c.outcomes.count(Outcome::LOST) ? c.outcomes.at(Outcome::LOST) : 0, 0u);
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_EQ(heard, (std::vector<int>{1, 2}));
}

TEST_F(CorrRuntime, SelfEdgeDrainTakesThePresentSampleItsIngressHasNotTaken)
{
  // The write precedes the arrive, so the sample is in the reader when
  // the drain runs. Before the fix the join found no sample past the cutoff
  // and ended NOT_READY, and the sample was shed late at the next take.
  dispatch_while_ingress_is_starved(true, 7);
  const auto c = rt::table().counts(0);
  const std::string ended = c.outcomes.empty() ? std::string("no outcome") :
    std::string(ros2_lf_binding::corr::outcome_name(c.outcomes.begin()->first));
  EXPECT_EQ(c.outcomes.count(Outcome::MATCHED) ? c.outcomes.at(Outcome::MATCHED) : 0, 1u)
    << "self edge 0 ended " << ended << ": the drain did not take the present sample";
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_EQ(heard, (std::vector<int>{7}));
}

TEST_F(CorrRuntime, SelfEdgeDrainTakesPastMoreThanABatchOfOlderSamples)
{
  // 20 older samples (more than one take batch) sit ahead of the
  // self sample in its reader. The drain still takes up to it and matches it.
  publish_while_ingress_is_starved(true, 7, 20, 40);
  ASSERT_FALSE(HasFatalFailure());
  const std::vector<int> got = dispatch_all();
  ros2_lf_binding::IngressRole::instance().pause_corr_takes_for_test(false);
  const auto c = rt::table().counts(0);
  const std::string ended = c.outcomes.empty() ? std::string("no outcome") :
    std::string(ros2_lf_binding::corr::outcome_name(c.outcomes.begin()->first));
  EXPECT_EQ(c.outcomes.count(Outcome::MATCHED) ? c.outcomes.at(Outcome::MATCHED) : 0, 1u)
    << "self edge 0 ended " << ended
    << ": the drain missed self sample 7 behind 20 older samples";
  EXPECT_EQ(got, (std::vector<int>{7}));
}

TEST_F(CorrRuntime, SelfEdgeDrainIsNotBlockedByATakeOnAnotherEdge)
{
  // A take on edge 1 whose reader refills forever (held open
  // here) must not keep the drain of self edge 0 from its own take and join.
  publish_while_ingress_is_starved(true, 7);
  ASSERT_FALSE(HasFatalFailure());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  auto other = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic + "_other_edge", rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ASSERT_TRUE(ingress.admit_corr_subscription(1, other));
  auto hold = ingress.hold_corr_take_for_test(1);
  ASSERT_TRUE(hold.owns_lock()) << "edge 1 was not admitted";
  auto drained = std::async(std::launch::async, [this] {return dispatch_all();});
  const bool done = drained.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  hold.unlock();
  ingress.pause_corr_takes_for_test(false);
  EXPECT_TRUE(done) <<
    "the drain of self edge 0 blocked 2 s behind a take of correspondence samples on edge 1";
  EXPECT_EQ(drained.get(), (std::vector<int>{7}));
}

TEST_F(CorrRuntime, OneCorrespondenceTakeEmptiesAKeepLastReaderPastSixteen)
{
  // A KEEP_LAST reader holds at most its depth, so one take loop of
  // depth samples ends (a refilling writer cannot hold the lock past it) and
  // a backlog of 40 on a depth-64 reader is taken in one loop, not in 16s.
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const std::string t = topic + "_batch";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(64));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(64), [](const std_msgs::msg::Int32 &) {});
  ASSERT_TRUE(publisher_matched(p));
  ASSERT_TRUE(ingress.admit_corr_subscription(7, s));
  std_msgs::msg::Int32 m;
  for (int i = 0; i < 40; ++i) {
    m.data = i;
    p->publish(m);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // all reach the reader
  const auto before = ingress.corr_take_stats(false);
  const size_t taken = ingress.take_corr_batch_for_test(7);
  const auto after = ingress.corr_take_stats(false);
  ingress.pause_corr_takes_for_test(false);
  EXPECT_EQ(taken, 40u) << "one take loop took " << taken <<
    " of a backlog of 40 on a depth-64 reader";
  EXPECT_EQ(after.samples - before.samples, 40u) << "the thread counters missed taken samples";
  EXPECT_EQ(
    ros2_lf_binding::IngressRole::corr_batch(true, 64), 64u);
  EXPECT_EQ(
    ros2_lf_binding::IngressRole::corr_batch(false, 64),
    ros2_lf_binding::IngressRole::kCorrKeepAllTakeBatch);
}

TEST_F(CorrRuntime, WorkerTakeOfABacklogIsCounted)
{
  // A worker's take_corr_now of a backlog past 16 counts as one
  // worker call with every sample, apart from the ingress thread's loops.
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const std::string t = topic + "_worker_batch";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(64));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(64), [](const std_msgs::msg::Int32 &) {});
  ASSERT_TRUE(publisher_matched(p));
  ASSERT_TRUE(ingress.admit_corr_subscription(7, s));
  std_msgs::msg::Int32 m;
  for (int i = 0; i < 40; ++i) {
    m.data = i;
    p->publish(m);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // all reach the reader
  const auto wb = ingress.corr_take_stats(true);
  const auto tb = ingress.corr_take_stats(false);
  const size_t taken = ingress.take_corr_now(7);
  const auto wa = ingress.corr_take_stats(true);
  const auto ta = ingress.corr_take_stats(false);
  ingress.pause_corr_takes_for_test(false);
  EXPECT_EQ(taken, 40u) << "the worker took " << taken << " of a backlog of 40";
  EXPECT_EQ(wa.calls - wb.calls, 1u) << "the worker's take_corr_now call was not counted";
  EXPECT_EQ(wa.samples - wb.samples, 40u) << "the worker counters missed taken samples";
  EXPECT_GE(wa.max, 40u) << "the worker counters miss the call's size";
  EXPECT_EQ(ta.samples - tb.samples, 0u) << "a worker take was counted as a thread loop";
  EXPECT_EQ(wa.timed - wb.timed, 40u) << "the rmw gave no reception time to measure the delay";
}

TEST_F(CorrRuntime, BacklogOfFortyIntakeOnBothPaths)
{
  // Measurement: a controlled backlog of 40 (past the KEEP_ALL
  // batch of 16) on a depth-64 KEEP_LAST reader and on a KEEP_ALL reader,
  // taken by the running ingress thread and by a worker's take_corr_now.
  // Prints the loops, full loops, the span from the release to the last take
  // and the mean reception-to-take delay per repetition (intake lines);
  // the delay includes the 50 ms the backlog is held, the span is the intake.
  // Expected counts, derived from corr_batch: KEEP_LAST depth 64 takes all 40
  // in one loop that is not full (40 < 64). KEEP_ALL takes ceil(40 / 16) = 3
  // thread loops that take samples, 2 of them full, and one worker call (3
  // lock-held batches under the 1024 limit, so not full). The thread runs a
  // loop on every admitted reader at each wake and counts an empty loop too,
  // so in the KEEP_ALL phase (two readers admitted) its call count is at least
  // 3, not exactly 3. The full count is exact, an empty loop is never full.
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  constexpr int kBacklog = 40;
  constexpr int kReps = 10;
  uint32_t edge = 7;
  for (const bool keep_all : {false, true}) {
    const std::string t = topic + (keep_all ? "_intake_all" : "_intake");
    const rclcpp::QoS qos = keep_all ? rclcpp::QoS(rclcpp::KeepAll()).reliable() :
      rclcpp::QoS(64);
    auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, qos);
    auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
      t, qos, [](const std_msgs::msg::Int32 &) {});
    ASSERT_TRUE(publisher_matched(p));
    ASSERT_TRUE(ingress.admit_corr_subscription(edge, s));
    const uint64_t want_calls_thread = keep_all ? 3 : 1;
    const uint64_t want_full_thread = keep_all ? 2 : 0;
    std_msgs::msg::Int32 m;
    for (const bool worker : {false, true}) {
      // Force the interleaving the CI runner hit, with an
      // explicit handshake. The test arms the hook per rep. The thread's loop
      // that completes the backlog enters the hook after its samples are
      // counted and before its call is, says so (entered), and holds until
      // the takes are paused or the test has read the stats (read). The test
      // asserts that the hook was entered before its read and that the hook
      // left because of the pause or the read, never because of the bound.
      // So a read that does not wait out the loop sees the call missing in
      // every rep (the hook records the call count it saw on entry), and the
      // bound only keeps a broken test from hanging.
      struct Gate
      {
        std::atomic<bool> armed{false};
        std::atomic<uint64_t> counted{0};
        std::atomic<bool> entered{false};
        std::atomic<bool> read{false};
        std::atomic<bool> exited{false};
        std::atomic<bool> bound_hit{false};
        std::atomic<uint64_t> calls_at_entry{0};  // thread calls recorded before the hooked one
      } gate;
      ingress.set_corr_loop_end_hook_for_test(
        worker ? std::function<void(size_t)>() : [&ingress, &gate](size_t n) {
          if (n == 0 || !gate.armed.load()) {
            return;
          }
          if (gate.counted.fetch_add(n) + n < static_cast<uint64_t>(kBacklog)) {
            return;
          }
          gate.armed.store(false);
          gate.calls_at_entry.store(ingress.corr_take_stats(false).calls);
          gate.entered.store(true);
          const auto bound = std::chrono::steady_clock::now() + std::chrono::seconds(5);
          while (!ingress.corr_takes_paused_for_test() && !gate.read.load()) {
            if (std::chrono::steady_clock::now() >= bound) {
              gate.bound_hit.store(true);
              break;
            }
            std::this_thread::yield();
          }
          gate.exited.store(true);
        });
      struct HookReset  // also on an early ASSERT return, so no hook outlives the gate
      {
        ros2_lf_binding::IngressRole & ingress;
        ~HookReset() {ingress.set_corr_loop_end_hook_for_test(nullptr);}
      } hook_reset{ingress};
      for (int rep = 0; rep < kReps; ++rep) {
        ingress.pause_corr_takes_for_test(true);
        for (int i = 0; i < kBacklog; ++i) {
          m.data = i;
          p->publish(m);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // all reach the reader
        const auto before = ingress.corr_take_stats(worker);
        const auto t0 = std::chrono::steady_clock::now();
        size_t taken = 0;
        if (!worker) {
          gate.counted.store(0);
          gate.entered.store(false);
          gate.read.store(false);
          gate.exited.store(false);
          gate.bound_hit.store(false);
          gate.armed.store(true);
        }
        if (worker) {
          taken = ingress.take_corr_now(edge);
          ingress.pause_corr_takes_for_test(false);
        } else {
          ingress.pause_corr_takes_for_test(false);
          const auto give_up = t0 + std::chrono::seconds(2);
          while (ingress.corr_take_stats(false).samples - before.samples <
            static_cast<uint64_t>(kBacklog) && std::chrono::steady_clock::now() < give_up)
          {
          }
          taken = ingress.corr_take_stats(false).samples - before.samples;
          const auto entered_by = std::chrono::steady_clock::now() + std::chrono::seconds(2);
          while (!gate.entered.load() && std::chrono::steady_clock::now() < entered_by) {
            std::this_thread::yield();
          }
          ASSERT_TRUE(gate.entered.load()) <<
            "rep " << rep << ": the loop that took the backlog never entered the hook";
        }
        const auto span_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - t0).count();
        // A loop counts its samples as it takes them and its call when it
        // ends, so the thread's last loop may still be in flight here. The
        // pause waits it out (the CI runner read calls=0 without it).
        ingress.pause_corr_takes_for_test(true);
        const auto after = ingress.corr_take_stats(worker);
        if (!worker) {
          gate.read.store(true);
          const auto exited_by = std::chrono::steady_clock::now() + std::chrono::seconds(6);
          while (!gate.exited.load() && std::chrono::steady_clock::now() < exited_by) {
            std::this_thread::yield();
          }
          ASSERT_TRUE(gate.exited.load()) << "rep " << rep << ": the hook never left";
          ASSERT_FALSE(gate.bound_hit.load()) <<
            "rep " << rep << ": the hook left on its 5 s bound, not on the pause or the read";
          // The read must include the hooked loop's own call. An earlier empty
          // loop can satisfy the count below on its own, so this is the check
          // that fails whenever the read does not wait out the loop.
          EXPECT_GT(after.calls, gate.calls_at_entry.load()) <<
            "rep " << rep << ": the stats were read before the hooked loop recorded its call";
        }
        ingress.pause_corr_takes_for_test(false);
        const char * path = worker ? "worker" : "thread";
        ASSERT_EQ(taken, static_cast<size_t>(kBacklog)) <<
          path << " took " << taken << " of " << kBacklog;
        if (worker) {
          EXPECT_EQ(after.calls - before.calls, 1u) << path;
        } else {
          EXPECT_GE(after.calls - before.calls, want_calls_thread) << path;
        }
        EXPECT_EQ(after.full - before.full, worker ? 0u : want_full_thread) << path;
        const uint64_t timed = after.timed - before.timed;
        std::printf(
          "intake: history=%s path=%s rep=%d backlog=%d calls=%llu full=%llu "
          "span_us=%lld delay_mean_us=%.1f\n", keep_all ? "keep_all" : "keep_last_64", path,
          rep, kBacklog,
          static_cast<unsigned long long>(after.calls - before.calls),  // NOLINT(runtime/int)
          static_cast<unsigned long long>(after.full - before.full),  // NOLINT(runtime/int)
          static_cast<long long>(span_us),  // NOLINT(runtime/int)
          timed > 0 ? static_cast<double>(after.delay_sum_ns - before.delay_sum_ns) /
          static_cast<double>(timed) / 1e3 : -1.0);
      }
    }
    ++edge;
  }
}

TEST_F(CorrRuntime, FanInReaderServesEveryMemberEdge)
{
  // A fan-in group's one reader is admitted on its
  // lead 7; a take, and the take lock, for member edge 8 use that reader.
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const std::string t = topic + "_fan_in";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(8));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(8), [](const std_msgs::msg::Int32 &) {});
  ASSERT_TRUE(publisher_matched(p));
  ASSERT_TRUE(ingress.admit_corr_subscription(7, s, {7, 8}));
  EXPECT_TRUE(ingress.hold_corr_take_for_test(8).owns_lock()) <<
    "member edge 8 found no reader: the group's reader is keyed on its lead 7 only";
  std_msgs::msg::Int32 m;
  for (int i = 0; i < 3; ++i) {
    m.data = i;
    p->publish(m);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // all reach the reader
  const size_t taken = ingress.take_corr_batch_for_test(8);
  ingress.pause_corr_takes_for_test(false);
  EXPECT_EQ(taken, 3u) << "a take for member edge 8 did not use the reader of its lead 7";
}

TEST_F(CorrRuntime, DrainTakeEndsUnderASustainedPublisher)
{
  // A worker's take for a drain is bounded by what the reader
  // held at entry, so a writer that never stops cannot keep the worker. A
  // depth that is not a multiple of the batch checks the last batch is cut.
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.pause_corr_takes_for_test(true);
  const std::string t = topic + "_sustained";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(17));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(17), [](const std_msgs::msg::Int32 &) {});
  ASSERT_TRUE(publisher_matched(p));
  ASSERT_TRUE(ingress.admit_corr_subscription(7, s));
  std::atomic<bool> stop{false};
  std::thread writer([&] {
      std_msgs::msg::Int32 m;
      while (!stop.load()) {
        p->publish(m);
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // the reader is full
  auto taken = std::async(std::launch::async, [&] {return ingress.take_corr_now(7);});
  const bool done = taken.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  stop.store(true);
  writer.join();
  ingress.pause_corr_takes_for_test(false);
  ASSERT_TRUE(done) << "the drain take did not end while a writer kept publishing";
  EXPECT_LE(taken.get(), 17u) << "the drain take went past what the reader held at entry";
}

TEST_F(CorrRuntime, OtherLocalEdgeIsDecidedAtPublishWhileIngressIsStarved)
{
  // Before, on an edge between two nodes the drain took nothing itself
  // and the untaken sample ended missing at the cutoff. The publish now takes
  // it, so a starved ingress thread no longer matters on a local edge.
  dispatch_while_ingress_is_starved(false, 7);
  const auto c = rt::table().counts(0);
  EXPECT_EQ(c.outcomes.count(Outcome::MATCHED) ? c.outcomes.at(Outcome::MATCHED) : 0, 1u);
  uint64_t missing = 0;
  for (const auto & kv : c.outcomes) {
    missing += ros2_lf_binding::corr::is_missing(kv.first) ? kv.second : 0;
  }
  EXPECT_EQ(missing, 0u);
  std::lock_guard<std::mutex> lk(mu);
  EXPECT_EQ(heard, (std::vector<int>{7}));
}

// A sample published just below the stop tag is still in the
// reader when LF returns (the join ended NOT_READY, the sample missing). The
// shutdown reactions stop the ingress thread at the stop tag, and the exit
// backstop stops it before the runtime's atexit shutdown, so the stop drain
//  ran with no taker: it waited its full bound and the sample was
// counted as never arrived. The drain takes the sample itself, late.
TEST_F(CorrRuntime, ALocalSampleIsNeverInFlightAtTheStopDrain)
{
  // On a local edge: the stop drain had to take a sample the
  // stopped ingress thread left in the reader. The publish now takes it, so
  // the join matches, the drain owes nothing and nothing is late.
  publish_while_ingress_is_starved(false, 7);
  ASSERT_FALSE(HasFatalFailure());
  dispatch_all();
  auto & ingress = ros2_lf_binding::IngressRole::instance();
  ingress.stop_and_join();  // the last mailbox close at the stop tag
  ingress.pause_corr_takes_for_test(false);
  uint64_t missing = 0;
  for (const auto & kv : rt::table().counts(0).outcomes) {
    missing += ros2_lf_binding::corr::is_missing(kv.first) ? kv.second : 0;
  }
  EXPECT_EQ(missing, 0u) << "the local sample was not taken at publish";
  const auto t0 = std::chrono::steady_clock::now();
  const size_t owed = rt::drain_in_flight(750);
  const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(owed, 0u) << "the stop drain still owed a local sample";
  EXPECT_LT(took, 500) << "the stop drain ran to its bound";
  EXPECT_EQ(rt::table().counts(0).samples[ros2_lf_binding::corr::SampleOutcome::LATE], 0u);
}

TEST_F(CorrRuntime, OnlyTheAdmittedSubscriptionInScopeIsDeepened)
{
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr legacy, other_key, outside;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    legacy = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic + "_not_admitted", rclcpp::QoS(1), cb);
  }
  {
    Ros2LfNodeConstructionScope scope("Other");
    other_key = sub_node->create_subscription<std_msgs::msg::Int32>(topic, rclcpp::QoS(1), cb);
  }
  outside = sub_node->create_subscription<std_msgs::msg::Int32>(topic, rclcpp::QoS(1), cb);
  EXPECT_EQ(legacy->get_actual_qos().depth(), 1u);
  EXPECT_EQ(other_key->get_actual_qos().depth(), 1u);
  EXPECT_EQ(outside->get_actual_qos().depth(), 1u);
}

TEST_F(CorrRuntime, OfTwinSubscriptionsOnlyTheAdmittedRankIsDeepened)
{
  // The edge's subscription RID has creation rank 0 on the topic. A second,
  // legacy subscription on the same topic keeps the user's depth.
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr admitted, legacy;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    admitted = sub_node->create_subscription<std_msgs::msg::Int32>(topic, rclcpp::QoS(1), cb);
    legacy = sub_node->create_subscription<std_msgs::msg::Int32>(topic, rclcpp::QoS(1), cb);
  }
  EXPECT_EQ(admitted->get_actual_qos().depth(), 8u);
  EXPECT_EQ(legacy->get_actual_qos().depth(), 1u) << "the legacy twin was deepened too";
}

TEST_F(CorrRuntime, AFanInGroupsReaderTakesItsDeepestMembersDepth)
{
  // The members of a fan-in group name one reader, so the gate
  // deepens it to the deepest member's reader_depth, not the first edge's.
  ros2_lf_binding::register_corr_edge(
    {0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0, -1, -1, 0, 0u, 0u, 4u});
  ros2_lf_binding::register_corr_edge(
    {1, "PubB", {kPubRid + 2}, topic, "Sub", kSubRid, topic, 0, -1, -1, 0, 0u, 0u, 12u});
  ros2_lf_binding::set_corr_edge_fan_in(0, 0, 1);
  ros2_lf_binding::set_corr_edge_fan_in(1, 0, 0);
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reader;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    reader = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(reader->get_actual_qos().depth(), 12u)
    << "the group's one reader holds only its first member's depth";
}

/// Bind `sub` to subscription RID `rid` of node key `key`, then
/// run startup over the registered edges.
std::optional<rt::StartupFailure> bind_and_start(
  const std::string & key, int rid,
  const rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr & sub,
  const rt::NodeResolver & resolver)
{
  auto & table = ros2_lf_binding::g_handle_table;
  table.resize(std::max(table.size(), static_cast<size_t>(rid) + 1));
  table[rid].kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  table[rid].subscription = sub;
  table[rid].take_timeout_ms = 1000;
  table[rid].node_class = key;
  table[rid].callback_name = "on_twin";
  return rt::startup(resolver);
}

TEST_F(CorrRuntime, OfTwoSubscriptionRidsTheAdmittedSecondIsDeepened)
{
  // Two generated subscription RIDs on one topic. The edge names the one the
  // node creates second (creation rank 1), so that reader is deepened.
  rt::reset_for_test();
  const std::string twin_topic = topic + "_twins";
  ros2_lf_binding::register_rid_entry({7, "Twin", "on_a", "sub", twin_topic, 0, 0, kAnyGroup});
  ros2_lf_binding::register_rid_entry({8, "Twin", "on_b", "sub", twin_topic, 0, 1, kAnyGroup});
  auto twin_pub = pub_node->create_publisher<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1));
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, twin_topic, "Twin", 8, twin_topic, 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr first, second;
  {
    Ros2LfNodeConstructionScope scope("Twin");
    first = sub_node->create_subscription<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1), cb);
    second = sub_node->create_subscription<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1), cb);
  }
  EXPECT_EQ(first->get_actual_qos().depth(), 1u) << "the twin the edge does not name was deepened";
  EXPECT_EQ(second->get_actual_qos().depth(), 8u)
    << "the admitted twin, created second, kept depth 1, so startup refuses a valid edge";
  const auto f = bind_and_start("Twin", 8, second, resolver());
  EXPECT_FALSE(f.has_value()) << "startup refused: " << (f ? f->check + " " + f->detail : "");
}

TEST_F(CorrRuntime, ASubscriptionTheRidTableDoesNotRepresentFailsStartupClosed)
{
  // One generated subscription RID on the topic, but the node also creates a
  // subscription the analysis never saw. Which handle the RID names is then
  // unknown, so startup refuses instead of guessing.
  rt::reset_for_test();
  const std::string t = topic + "_unrepresented";
  ros2_lf_binding::register_rid_entry({9, "Mixed", "on_admitted", "sub", t, 0, 0, kAnyGroup});
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(1));
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, t, "Mixed", 9, t, 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr unseen, admitted;
  {
    Ros2LfNodeConstructionScope scope("Mixed");
    unseen = sub_node->create_subscription<std_msgs::msg::Int32>(t, rclcpp::QoS(1), cb);
    admitted = sub_node->create_subscription<std_msgs::msg::Int32>(t, rclcpp::QoS(1), cb);
  }
  const auto f = bind_and_start("Mixed", 9, admitted, resolver());
  ASSERT_TRUE(f.has_value()) << "startup picked a handle for a subscription the analysis lacks";
  EXPECT_NE(
    f->detail.find("the analysis found 1 subscriptions of the node on this topic and 2 match"),
    std::string::npos) << f->detail;
}

TEST_F(CorrRuntime, TwinsRemappedOntoOneTopicDeepenAndSelectTheAdmittedSecond)
{
  // The node declares subscriptions on 'a' and 'b' and the
  // launch remaps both to one topic. They are twins by the resolved name, so
  // the edge on 'b' (created second) names the second reader on the topic.
  // Falsify by comparing declared names in twin_order: 'a' is then no twin,
  // the edge's rank is 0 and the subscription on 'a' is deepened instead.
  rt::reset_for_test();
  const std::string common = topic + "_common";
  auto remapped = std::make_shared<rclcpp::Node>(
    "corr_remap_" + std::to_string(counter), rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r", "a:=" + common, "-r", "b:=" + common}));
  ros2_lf_binding::register_rid_entry({20, "Remap", "on_a", "sub", "a", 0, 0, kAnyGroup});
  ros2_lf_binding::register_rid_entry({21, "Remap", "on_b", "sub", "b", 0, 1, kAnyGroup});
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(common, rclcpp::QoS(1));
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, common, "Remap", 21, "b", 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr on_a, on_b;
  {
    Ros2LfNodeConstructionScope scope("Remap");
    on_a = remapped->create_subscription<std_msgs::msg::Int32>("a", rclcpp::QoS(1), cb);
    on_b = remapped->create_subscription<std_msgs::msg::Int32>("b", rclcpp::QoS(1), cb);
  }
  ASSERT_EQ(std::string(on_a->get_topic_name()), common);
  ASSERT_EQ(std::string(on_b->get_topic_name()), common);
  EXPECT_EQ(on_a->get_actual_qos().depth(), 1u)
    << "the subscription on 'a' was deepened in place of the admitted one on 'b'";
  EXPECT_EQ(on_b->get_actual_qos().depth(), 8u)
    << "the admitted subscription on 'b' kept depth 1";
  const rt::NodeResolver remap_resolver = [&](const std::string & key) {
      return key == "Pub" ? pub_node->get_node_base_interface() :
             remapped->get_node_base_interface();
    };
  const auto f = bind_and_start("Remap", 21, on_b, remap_resolver);
  ASSERT_FALSE(f.has_value()) << "startup refused: " << f->check << " " << f->detail;
  EXPECT_EQ(rt::edge_subscription_for_test(0), on_b.get())
    << "startup bound the edge on 'b' to the subscription on 'a'";
}

TEST_F(CorrRuntime, AFailedSubscriptionInitTakesNoCreationRank)
{
  // rcl_subscription_init fails (no type support) inside the scope, then the
  // admitted subscription is created: it is still creation rank 0.
  rcl_subscription_t failed = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t o = rcl_subscription_get_default_options();
  o.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  o.qos.depth = 1;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr admitted;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    const rcl_ret_t ret = rcl_subscription_init(
      &failed, sub_node->get_node_base_interface()->get_rcl_node_handle(), nullptr,
      topic.c_str(), &o);
    ASSERT_NE(ret, RCL_RET_OK);
    rcl_reset_error();
    admitted = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(admitted->get_actual_qos().depth(), 8u)
    << "the failed init consumed creation rank 0, so the admitted reader kept depth 1";
}

TEST_F(CorrRuntime, UnindexedTwinsDeepenNoneAndStartupRefuses)
{
  // Two subscriptions on one topic without a creation index (an older
  // generator or a shared callback): which one the edge names is unknown, so
  // the gate deepens neither and startup refuses instead of guessing. A lone
  // subscription without an index is unambiguous and is not covered here.
  rt::reset_for_test();
  const std::string t = topic + "_no_index";
  ros2_lf_binding::register_rid_entry({11, "NoIndex", "on_a", "sub", t, 0});
  ros2_lf_binding::register_rid_entry({12, "NoIndex", "on_b", "sub", t, 0});
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(10));
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, t, "NoIndex", 12, t, 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr first, second;
  {
    Ros2LfNodeConstructionScope scope("NoIndex");
    first = sub_node->create_subscription<std_msgs::msg::Int32>(t, rclcpp::QoS(1), cb);
    second = sub_node->create_subscription<std_msgs::msg::Int32>(t, rclcpp::QoS(1), cb);
  }
  EXPECT_EQ(first->get_actual_qos().depth(), 1u) << "an unindexed twin was deepened";
  EXPECT_EQ(second->get_actual_qos().depth(), 1u) << "an unindexed twin was deepened";
  const auto f = bind_and_start("NoIndex", 12, second, resolver());
  ASSERT_TRUE(f.has_value()) << "startup guessed which unindexed twin the edge names";
  EXPECT_NE(f->detail.find("does not single one out"), std::string::npos) << f->detail;
}

TEST_F(CorrRuntime, ASubscriptionOnAnotherThreadOfAScopedNodeIsDeepened)
{
  rclcpp::Node::SharedPtr scoped;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    scoped = std::make_shared<rclcpp::Node>("corr_scoped_" + std::to_string(counter));
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr worker_sub;
  std::thread worker([&] {
      worker_sub = scoped->create_subscription<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
    });
  worker.join();
  EXPECT_EQ(worker_sub->get_actual_qos().depth(), 8u)
    << "a subscription created off the construction thread kept depth 1";
}

TEST_F(CorrRuntime, ConcurrentSubscriptionInitsOnOneNodeAndTopicTakeDistinctRanks)
{
  // Two threads create a subscription on one scoped node and topic at once.
  // The edge names the rank 0 twin, so exactly one reader is deepened and the
  // ranks committed are 0 and 1. The hook holds each init between its real
  // rcl init and its rank commit until the other arrives or 500 ms pass, the
  // window in which an unserialized pair both select rank 0.
  rt::reset_for_test();
  const std::string t = topic + "_concurrent";
  ros2_lf_binding::register_rid_entry({30, "Conc", "on_a", "sub", t, 0, 0, kAnyGroup});
  ros2_lf_binding::register_rid_entry({31, "Conc", "on_b", "sub", t, 0, 1, kAnyGroup});
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, t, "Conc", 30, t, 0});
  rclcpp::Node::SharedPtr scoped;
  {
    Ros2LfNodeConstructionScope scope("Conc");
    scoped = std::make_shared<rclcpp::Node>("corr_conc_" + std::to_string(counter));
  }
  std::atomic<int> arrived{0};
  rt::set_sub_commit_hook_for_test([&arrived] {
      ++arrived;
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
      while (arrived.load() < 2 && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr a, b;
  auto make = [&](rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr & out) {
      out = scoped->create_subscription<std_msgs::msg::Int32>(
        t, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
    };
  std::thread ta([&] {make(a);});
  std::thread tb([&] {make(b);});
  ta.join();
  tb.join();
  rt::set_sub_commit_hook_for_test({});
  const size_t deep = (a->get_actual_qos().depth() == 8u) + (b->get_actual_qos().depth() == 8u);
  EXPECT_EQ(deep, 1u) << "two concurrent subscription inits both took rank 0 and both were "
    "deepened (or neither was)";
  EXPECT_EQ(rt::sub_inits_for_test("Conc", t), 2u)
    << "the two concurrent inits committed one rank between them";
}

TEST_F(CorrRuntime, ARecreatedNodeOfTheSameKeyStartsItsRanksAtZero)
{
  // A component is finalized and recreated under the same binding key. Its
  // first subscription is rank 0 again, so the edge naming rank 0 deepens it.
  rt::reset_for_test();
  const std::string t = topic + "_recreate";
  ros2_lf_binding::register_rid_entry({40, "Recr", "on_a", "sub", t, 0, 0, kAnyGroup});
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, t, "Recr", 40, t, 0});
  auto cb = [](const std_msgs::msg::Int32 &) {};
  for (int life = 0; life < 2; ++life) {
    rclcpp::Node::SharedPtr node;
    {
      Ros2LfNodeConstructionScope scope("Recr");
      node = std::make_shared<rclcpp::Node>(
        "corr_recr_" + std::to_string(counter) + "_" + std::to_string(life));
    }
    auto s = node->create_subscription<std_msgs::msg::Int32>(t, rclcpp::QoS(1), cb);
    EXPECT_EQ(s->get_actual_qos().depth(), 8u)
      << "life " << life << ": the recreated node's subscription inherited the finalized "
      "node's rank and was not deepened";
    EXPECT_EQ(rt::sub_inits_for_test("Recr", t), 1u) << "life " << life;
    s.reset();
    node.reset();  // rcl_node_fini
  }
  EXPECT_EQ(rt::sub_inits_for_test("Recr", t), 0u)
    << "the finalized node's ranks outlived it";
}

TEST_F(CorrRuntime, StartupRefusesAnAdmittedReaderShallowerThanTheRetainDepth)
{
  // A subscription created outside any scope and on a node no scope created
  // keeps depth 1: startup refuses rather than run with an overwrite risk.
  rt::reset_for_test();
  const std::string shallow = topic + "_shallow";
  auto shallow_pub = pub_node->create_publisher<std_msgs::msg::Int32>(shallow, rclcpp::QoS(1));
  auto shallow_sub = sub_node->create_subscription<std_msgs::msg::Int32>(
    shallow, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, shallow, "Sub", kSubRid, shallow, 0});
  ASSERT_EQ(shallow_sub->get_actual_qos().depth(), 1u);
  auto f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "reader_depth");
  EXPECT_EQ(f->edge, 0);
  EXPECT_EXIT(
    rt::startup_or_exit(resolver()), ::testing::ExitedWithCode(1),
    "keeps 1 sample\\(s\\), fewer than 8.*refusing to start");
}

TEST_F(CorrRuntime, ABestEffortReaderOnAnEdgeAnalyzedReliableWarnsH001)
{
  const auto [err, ok] = start_with_reader_reliability(
    "_be", "reliable", rclcpp::QoS(10).best_effort());
  EXPECT_TRUE(ok) << "RCLLF-H001 is a warning, startup must not refuse";
  const std::regex want(
    "\\[WARN\\].*RCLLF-H001.*edge 0 Pub -> Sub on '" + topic + "_be'.*node '/corr_sub_[0-9]+'"
    ".*runs best effort.*analysis saw 'reliable'");
  EXPECT_TRUE(std::regex_search(err, want)) << err;
}

TEST_F(CorrRuntime, NoH001WarningWhenTheAnalysisAlreadySawBestEffortOrTheReaderIsReliable)
{
  const auto [seen, ok1] = start_with_reader_reliability(
    "_be_seen", "best_effort", rclcpp::QoS(10).best_effort());
  EXPECT_TRUE(ok1);
  EXPECT_EQ(seen.find("RCLLF-H001"), std::string::npos) << seen;
  const auto [rel, ok2] = start_with_reader_reliability("_rel", "reliable", rclcpp::QoS(10));
  EXPECT_TRUE(ok2);
  EXPECT_EQ(rel.find("RCLLF-H001"), std::string::npos) << rel;
}

TEST_F(CorrRuntime, AnAdmittedReaderAskingDepthOneWarnsH002AndStillStarts)
{
  // A scenario overlay asks for depth 1 where the analysis saw 10.
  // The binding deepens the reader, the user's depth still earns the warning.
  rt::reset_for_test();
  topic += "_h002";
  ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
  spec.sub_depth = 10;
  ros2_lf_binding::register_corr_edge(spec);
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  }
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << "RCLLF-H002 is a warning, startup must not refuse: " << err;
  const std::regex want(
    "\\[WARN\\].*RCLLF-H002 admitted edge 0 Pub -> Sub on '" + topic +
    "': the reader of node '/corr_sub_[0-9]+' asks for depth 1, the analysis saw depth 10.*"
    "keeps 8 sample");
  EXPECT_TRUE(std::regex_search(err, want)) << err;
}

TEST_F(CorrRuntime, ACrossFederateAdmittedReaderRunsBestEffortAndH001JudgesTheUsersReliability)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // A RELIABLE reader withholds every sample behind a sequence
  // gap until the writer's KeepLast history drops the lost one, so one
  // unrepairable loss on a cross-federate edge ended as history-depth lost
  // calls. The binding creates the reader of an ingress
  // edge BEST_EFFORT; the records over the LF connection detect the loss.
  rt::reset_for_test();
  topic += "_be459";
  auto spec = cross_federate_spec(topic);
  spec.sub_reliability = "reliable";
  ros2_lf_binding::register_corr_edge(spec);
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::BestEffort)
    << "the admitted reader runs RELIABLE: after one unrepairable loss DDS withholds every "
    "later sample until the writer's history drops the lost one, and under centralized "
    "coordination one lost message becomes 10 lost calls (history depth), "
    "the cascade of an unreliable link";
  EXPECT_EQ(s->get_actual_qos().depth(), 10u) << "a reader already at depth 10 was changed";
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << err;
  // H001 judges the reliability the user asked for, not the one the binding set.
  EXPECT_EQ(err.find("RCLLF-H001"), std::string::npos) << err;
  EXPECT_NE(err.find("reader best effort, user reliable"), std::string::npos)
    << "the admitted line does not say what the binding changed: " << err;
}

TEST_F(CorrRuntime, ASameFederateAdmittedReaderKeepsTheUsersReliability)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // Review: the cascade needs the blocking join across the
  // federation link. Inside one federate one loss is one lost call (one
  // program arm), so the reader keeps the user's RELIABLE and DDS still
  // repairs a transient wire loss there.
  rt::reset_for_test();
  topic += "_rel_local459";
  ros2_lf_binding::CorrEdgeSpec spec{0, "Pub", {kPubRid}, topic, "Sub", kSubRid, topic, 0};
  spec.sub_reliability = "reliable";
  ros2_lf_binding::register_corr_edge(spec);
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::Reliable)
    << "a same-federate admitted reader was made best effort: it loses DDS retransmission "
    "for nothing, no join across the federation link holds its publisher";
  EXPECT_EQ(s->get_actual_qos().depth(), 8u) << "the reader deepening must stay";
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << err;
  EXPECT_EQ(err.find("RCLLF-H001"), std::string::npos) << err;
  EXPECT_NE(err.find("user 1, reader reliable"), std::string::npos) << err;
  EXPECT_EQ(err.find("user reliable"), std::string::npos) << err;
}

TEST_F(CorrRuntime, ASystemDefaultReaderOnACrossFederateEdgeIsNotReportedAsUserReliable)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // Review: only an explicit RELIABLE is "user reliable". A reader
  // the user left at SYSTEM_DEFAULT is still made best effort on a
  // cross-federate edge, the admitted line names the policy the user gave,
  // and H001 judges it like the best-effort reader DDS would have made of it.
  rt::reset_for_test();
  topic += "_sysdef459";
  auto spec = cross_federate_spec(topic);
  spec.sub_reliability = "reliable";
  ros2_lf_binding::register_corr_edge(spec);
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT),
      [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << err;
  EXPECT_NE(err.find("reader best effort, user system default"), std::string::npos) << err;
  EXPECT_EQ(err.find("user reliable"), std::string::npos)
    << "a SYSTEM_DEFAULT request was reported as user reliable: " << err;
  EXPECT_NE(err.find("RCLLF-H001"), std::string::npos)
    << "a SYSTEM_DEFAULT request was judged reliable for H001: " << err;
}

TEST_F(CorrRuntime, OfTwoSubscriptionRidsOnlyTheAdmittedReaderRunsBestEffort)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // The twin the edge does not name keeps the user's reliability.
  rt::reset_for_test();
  const std::string twin_topic = topic + "_be_twins";
  ros2_lf_binding::register_rid_entry({7, "Twin", "on_a", "sub", twin_topic, 0, 0, kAnyGroup});
  ros2_lf_binding::register_rid_entry({8, "Twin", "on_b", "sub", twin_topic, 0, 1, kAnyGroup});
  auto twin_pub = pub_node->create_publisher<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1));
  auto spec = cross_federate_spec(twin_topic);
  spec.sub_key = "Twin";
  spec.sub_rid = 8;
  ros2_lf_binding::register_corr_edge(spec);
  auto cb = [](const std_msgs::msg::Int32 &) {};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr first, second;
  {
    Ros2LfNodeConstructionScope scope("Twin");
    first = sub_node->create_subscription<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1), cb);
    second = sub_node->create_subscription<std_msgs::msg::Int32>(twin_topic, rclcpp::QoS(1), cb);
  }
  EXPECT_EQ(first->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::Reliable)
    << "the twin the edge does not name was made best effort";
  EXPECT_EQ(second->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::BestEffort)
    << "the admitted twin kept the user's RELIABLE reader";
  EXPECT_EQ(second->get_actual_qos().depth(), 8u);
}

TEST_F(CorrRuntime, RcllfAdmittedReaderReliableKeepsTheUsersReliability)
{
  // The escape hatch for comparison runs leaves the user's QoS
  // on a cross-federate edge too.
  rt::reset_for_test();
  topic += "_keep459";
  ros2_lf_binding::register_corr_edge(cross_federate_spec(topic));
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  setenv("RCLLF_ADMITTED_READER_RELIABLE", "1", 1);
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr s;
  {
    Ros2LfNodeConstructionScope scope("Sub");
    s = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(1), [](const std_msgs::msg::Int32 &) {});
  }
  EXPECT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(s->get_actual_qos().depth(), 8u) << "the escape hatch must not stop the deepening";
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << err;
  EXPECT_NE(err.find("user 1, reader reliable"), std::string::npos) << err;
}

TEST_F(CorrRuntime, StartupRefusesACrossFederateReaderLeftReliableOutsideTheConstructionScope)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // A subscription created outside any scope and on a node
  // no scope created keeps the user's RELIABLE. Its depth passes the depth check, so the
  // reliability check must refuse on a cross-federate edge, else the cascade
  // stays possible. (The same reader on a same-federate edge is admitted:
  // the test after this one.)
  rt::reset_for_test();
  const std::string t = topic + "_rel459";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(10));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ASSERT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  ASSERT_EQ(s->get_actual_qos().depth(), 10u);
  ros2_lf_binding::register_corr_edge(cross_federate_spec(t));
  auto f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value()) << "startup admitted a RELIABLE cross-federate reader";
  EXPECT_EQ(f->check, "reader_reliability");
  EXPECT_EQ(f->edge, 0);
  EXPECT_EXIT(
    rt::startup_or_exit(resolver()), ::testing::ExitedWithCode(1),
    "ingress edge 0 reader.*runs RELIABLE: one lost sample would withhold.*refusing to start");
  setenv("RCLLF_ADMITTED_READER_RELIABLE", "1", 1);
  const bool ok = !rt::startup(resolver()).has_value();
  EXPECT_TRUE(ok) << "the escape hatch must let the reader through";
}

TEST_F(CorrRuntime, StartupAdmitsASameFederateReaderLeftReliableOutsideTheConstructionScope)
{
  unsetenv("RCLLF_ADMITTED_READER_RELIABLE");  // the fixture sets the hatch
  // Review: the reliability refusal is for cross-federate edges
  // only. A hand-built RELIABLE reader of a same-federate edge is admitted
  // and keeps the user's reliability.
  rt::reset_for_test();
  const std::string t = topic + "_rel_local_hand459";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(t, rclcpp::QoS(10));
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ASSERT_EQ(s->get_actual_qos().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, t, "Sub", kSubRid, t, 0});
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << "a same-federate RELIABLE reader was refused: " << err;
  EXPECT_EQ(err.find("reader_reliability"), std::string::npos) << err;
  EXPECT_NE(err.find("user 10, reader reliable"), std::string::npos) << err;
}

TEST_F(CorrRuntime, NoStartupHazardOnAReliableAdmittedReaderAtLeastAsDeepAsTwo)
{
  const auto [seen, ok1] = start_with_reader_reliability(
    "_d10_seen", "", rclcpp::QoS(10), 10);
  EXPECT_TRUE(ok1);
  EXPECT_EQ(seen.find("RCLLF-H00"), std::string::npos) << seen;
  const auto [unknown, ok2] = start_with_reader_reliability("_d_unknown", "", rclcpp::QoS(10));
  EXPECT_TRUE(ok2);
  EXPECT_EQ(unknown.find("RCLLF-H00"), std::string::npos) << unknown;
}

TEST_F(CorrRuntime, ALegacyEdgeWithABestEffortDepthOneReaderWarnsH002AtStartupAndH001AtBind)
{
  // Legacy edges get the startup QoS check too, also when no edge
  // is admitted at all. H001 comes from the per-subscription check
  // at bind time, which names the legacy edge the subscription reads.
  rt::reset_for_test();
  const std::string t = topic + "_legacy";
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(1).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_legacy_edge(t, "Pub", "Sub", "transient_local", t, "reliable", 10);
  ::testing::internal::CaptureStderr();
  const bool ok = !rt::startup(resolver()).has_value();
  rt::check_subscription_qos({7, "Sub", "on_legacy", "sub", t, 0, 0, kAnyGroup, "reliable"}, *s);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(ok) << "a startup QoS hazard on a legacy edge never refuses: " << err;
  const std::regex h001(
    "\\[WARN\\].*RCLLF-H001 subscription RID 7 \\(Sub::on_legacy\\) on '" + t +
    "' runs best effort, the analysis saw 'reliable'\\. It reads the legacy edge from Pub, "
    "which reports a sample DDS drops only when a later take of an unfiltered reader sees the "
    "gap, never a trailing one");
  const std::regex h002(
    "\\[WARN\\].*RCLLF-H002 legacy edge Pub -> Sub on '" + t +
    "': the reader of node '/corr_sub_[0-9]+' keeps depth 1, the analysis saw depth 10");
  EXPECT_TRUE(std::regex_search(err, h001)) << err;
  EXPECT_TRUE(std::regex_search(err, h002)) << err;
  EXPECT_EQ(err.find("RCLLF-H001 legacy edge"), std::string::npos)
    << "the legacy edge check must leave H001 to the per-subscription check: " << err;
}

TEST_F(CorrRuntime, ABestEffortInputNoEdgeReadsWarnsH001ByItsRid)
{
  // A subscription no edge reads, such as an input from outside
  // the analyzed graph, the analysis saw reliable and a scenario made best effort.
  rt::reset_for_test();
  const std::string t = topic + "_ext";
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ::testing::internal::CaptureStderr();
  rt::check_subscription_qos({5, "Sub", "on_ext", "sub", t, 0, 0, kAnyGroup, "reliable"}, *s);
  const std::string err = ::testing::internal::GetCapturedStderr();
  const std::regex want(
    "\\[WARN\\].*RCLLF-H001 subscription RID 5 \\(Sub::on_ext\\) on '" + t +
    "' runs best effort, the analysis saw 'reliable'\\. It is no edge's reader \\(an input "
    "from outside the analyzed graph\\)");
  EXPECT_TRUE(std::regex_search(err, want)) << err;
  EXPECT_EQ(err.find("RCLLF-H000"), std::string::npos) << "the analysis had a value: " << err;
}

TEST_F(CorrRuntime, TheReaderFederateOfALegacyCrossFederateEdgeChecksItsReader)
{
  // A cross-federate edge left legacy is listed by the
  // publisher's federate. The reader's federate registers its ingress copy,
  // which names the publisher in H001 and gets H002, but is no legacy edge of
  // this process (no legacy line, no count). A twin on the topic adds a second
  // row from the same publisher, which H001 names once.
  rt::reset_for_test();
  const std::string t = topic + "_xfed_legacy";
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(1).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_legacy_ingress(
    t, "Pub", "Sub", "xfed_decentralized", t, "reliable", 10);
  ros2_lf_binding::register_corr_legacy_ingress(
    t, "Pub", "Sub", "xfed_decentralized", t, "best_effort", 10);
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  rt::check_subscription_qos({6, "Sub", "on_xfed", "sub", t, 0, 0, kAnyGroup, "reliable"}, *s);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(
    err.find(
      "RCLLF-H001 subscription RID 6 (Sub::on_xfed) on '" + t + "' runs best effort, the "
      "analysis saw 'reliable'. It reads the legacy edge from Pub, which reports a sample DDS "
      "drops only when a later take of an unfiltered reader sees the gap, never a "
      "trailing one"),
    std::string::npos) << err;
  EXPECT_NE(err.find("RCLLF-H002 legacy edge Pub -> Sub on '" + t + "'"), std::string::npos)
    << err;
  EXPECT_EQ(err.find("rcllf corr: legacy edge " + t), std::string::npos)
    << "the publisher's federate lists the edge, the reader's must not: " << err;
}

TEST_F(CorrRuntime, TwinSubscriptionsAreJudgedEachAgainstItsOwnAnalyzedReliability)
{
  // Twins on one topic, both best effort at run time. The
  // analysis saw RID 3 best effort and RID 4 reliable, so only RID 4 warns,
  // although its sibling was analyzed best effort.
  rt::reset_for_test();
  const std::string t = topic + "_twins";
  auto s1 = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  auto s2 = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_legacy_edge(
    t, "Pub", "Sub", "ambiguous_endpoint", t, "best_effort", 10);
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  rt::check_subscription_qos({3, "Sub", "on_a", "sub", t, 0, 0, kAnyGroup, "best_effort"}, *s1);
  rt::check_subscription_qos({4, "Sub", "on_b", "sub", t, 0, 1, kAnyGroup, "reliable"}, *s2);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("RCLLF-H001 subscription RID 4 (Sub::on_b) on '" + t + "' runs best effort"),
    std::string::npos) << "the reliable twin turned best effort got no line: " << err;
  EXPECT_EQ(err.find("RID 3"), std::string::npos)
    << "the twin the analysis saw best effort must not warn: " << err;
}

TEST_F(CorrRuntime, NoReliabilityAnalyzedLogsH000WithTheEffectiveOneAndH001WhenBestEffort)
{
  // RCLLF-H000 (reliability unknown to the analysis) gets
  // its startup counterpart, an INFO line with the effective reliability.
  rt::reset_for_test();
  const std::string rel_t = topic + "_h000_rel";
  const std::string be_t = topic + "_h000_be";
  auto rel = sub_node->create_subscription<std_msgs::msg::Int32>(
    rel_t, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  auto be = sub_node->create_subscription<std_msgs::msg::Int32>(
    be_t, rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ::testing::internal::CaptureStderr();
  rt::check_subscription_qos({8, "Sub", "on_rel", "sub", rel_t, 0, 0, kAnyGroup}, *rel);
  rt::check_subscription_qos({9, "Sub", "on_be", "sub", be_t, 0, 1, kAnyGroup}, *be);
  const std::string err = ::testing::internal::GetCapturedStderr();
  const std::regex h000_rel(
    "\\[INFO\\].*RCLLF-H000 subscription RID 8 \\(Sub::on_rel\\) on '" + rel_t +
    "': the analysis saw no reliability, the subscription runs reliable");
  const std::regex h000_be(
    "\\[INFO\\].*RCLLF-H000 subscription RID 9 \\(Sub::on_be\\) on '" + be_t +
    "': the analysis saw no reliability, the subscription runs best effort");
  const std::regex h001_be(
    "\\[WARN\\].*RCLLF-H001 subscription RID 9 .*the analysis saw no reliability\\.");
  EXPECT_TRUE(std::regex_search(err, h000_rel)) << err;
  EXPECT_TRUE(std::regex_search(err, h000_be)) << err;
  EXPECT_TRUE(std::regex_search(err, h001_be)) << err;
  EXPECT_EQ(err.find("RCLLF-H001 subscription RID 8"), std::string::npos) << err;
}

TEST_F(CorrRuntime, TheReaderOfAnAdmittedEdgeLogsH000AndLeavesH001ToTheEdgeLine)
{
  // The fixture registers admitted edge 0 read by kSubRid. Its H001 is the
  // Startup line, the per-subscription check must not repeat it, but an
  // unknown analyzed reliability still gets its H000 line.
  auto s = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic + "_adm", rclcpp::QoS(10).best_effort(), [](const std_msgs::msg::Int32 &) {});
  ::testing::internal::CaptureStderr();
  rt::check_subscription_qos({kSubRid, "Sub", "on_data", "sub", topic, 0, 0, kAnyGroup}, *s);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(
    err.find(
      "RCLLF-H000 subscription RID " + std::to_string(kSubRid) + " (Sub::on_data) on '" +
      topic + "_adm': the analysis saw no reliability, the subscription runs best effort"),
    std::string::npos) << err;
  EXPECT_EQ(err.find("RCLLF-H001"), std::string::npos) << err;
}

TEST_F(CorrRuntime, APublisherWithNoAnalyzedReliabilityLogsH000WithItsEffectiveOneAtStartup)
{
  // The startup counterpart of the static RCLLF-H000 on a
  // publisher, also one on no edge. A publisher the analysis saw gets no line.
  rt::reset_for_test();
  const std::string loose = topic + "_h000_loose";
  const std::string known = topic + "_h000_known";
  const std::string twin = topic + "_h000_twin";
  const std::string late = topic + "_h000_late";
  auto p_loose = pub_node->create_publisher<std_msgs::msg::Int32>(
    loose, rclcpp::QoS(10).best_effort());
  auto p_known = pub_node->create_publisher<std_msgs::msg::Int32>(known, rclcpp::QoS(10));
  auto p_twin_be = pub_node->create_publisher<std_msgs::msg::Int32>(
    twin, rclcpp::QoS(10).best_effort());
  auto p_twin_rel = pub_node->create_publisher<std_msgs::msg::Int32>(twin, rclcpp::QoS(10));
  ros2_lf_binding::register_publisher_qos("Pub", loose, "");
  ros2_lf_binding::register_publisher_qos("Pub", known, "reliable");
  ros2_lf_binding::register_publisher_qos("Pub", twin, "best_effort");
  ros2_lf_binding::register_publisher_qos("Pub", twin, "");
  ros2_lf_binding::register_publisher_qos("Pub", late, "");
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  const std::string node = pub_node->get_fully_qualified_name();
  auto line = [&](const std::string & t, const std::string & tail) {
      return "RCLLF-H000 publisher of node '" + node + "' on '" + t + "': " + tail;
    };
  const std::string twins = "the analysis saw no reliability on 1 of the 2 publishers of the "
    "node on this topic, the publisher runs ";
  EXPECT_NE(
    err.find(line(loose, "the analysis saw no reliability, the publisher runs best effort")),
    std::string::npos) << err;
  EXPECT_NE(err.find(line(twin, twins + "best effort")), std::string::npos) << err;
  EXPECT_NE(err.find(line(twin, twins + "reliable")), std::string::npos) << err;
  EXPECT_NE(
    err.find(
      line(
        late, "the analysis saw no reliability, and no such publisher exists at startup, so "
        "its effective reliability is not checked")), std::string::npos) << err;
  EXPECT_EQ(err.find("on '" + known + "'"), std::string::npos)
    << "the analysis saw this publisher's reliability: " << err;
  size_t count = 0;
  for (size_t at = err.find("RCLLF-H000 publisher"); at != std::string::npos;
    at = err.find("RCLLF-H000 publisher", at + 1))
  {
    ++count;
  }
  EXPECT_EQ(count, 4u) << err;
}

TEST_F(CorrRuntime, TheNodesOwnRosoutAndParameterEventsPublishersAreNotTakenForAnAnalyzedOne)
{
  // Every rclcpp node creates its own publishers on
  // /rosout and /parameter_events. An analyzed publisher on such a topic
  // without a reliability must not be judged by them.
  rt::reset_for_test();
  ros2_lf_binding::register_publisher_qos("Pub", "/rosout", "");
  ros2_lf_binding::register_publisher_qos("Pub", "/parameter_events", "");
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("the publisher runs"), std::string::npos)
    << "the node's own /rosout or /parameter_events publisher was taken for the analyzed one: "
    << err;
  for (const std::string t : {"/rosout", "/parameter_events"}) {
    EXPECT_NE(
      err.find(
        "on '" + t + "': the analysis saw no reliability, and no such publisher exists at "
        "startup"), std::string::npos) << t << ": " << err;
  }
}

TEST_F(CorrRuntime, AUserPublisherOnAnActionLikeTopicIsStillJudged)
{
  // Only the action feedback and status publishers rclcpp
  // creates, with their own message types, are the node's own. A user
  // publisher under `/_action/`, or on a status topic with a user type, is not.
  rt::reset_for_test();
  const std::string data = "/custom/_action/data";
  const std::string status = "/custom/_action/status";
  auto p_data = pub_node->create_publisher<std_msgs::msg::Int32>(
    data, rclcpp::QoS(10).best_effort());
  auto p_status = pub_node->create_publisher<std_msgs::msg::Int32>(status, rclcpp::QoS(10));
  ros2_lf_binding::register_publisher_qos("Pub", data, "");
  ros2_lf_binding::register_publisher_qos("Pub", status, "");
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  const std::string node = pub_node->get_fully_qualified_name();
  EXPECT_NE(
    err.find(
      "RCLLF-H000 publisher of node '" + node + "' on '" + data + "': the analysis saw no "
      "reliability, the publisher runs best effort"), std::string::npos)
    << "the user publisher on " << data << " was taken for the node's own: " << err;
  EXPECT_NE(
    err.find(
      "RCLLF-H000 publisher of node '" + node + "' on '" + status + "': the analysis saw no "
      "reliability, the publisher runs reliable"), std::string::npos)
    << "the user publisher on " << status << " was taken for the node's own: " << err;
}

namespace
{
rosidl_runtime_c__type_description__TypeDescription g_fake_description{};
const rosidl_runtime_c__type_description__TypeDescription * fake_description(
  const rosidl_message_type_support_t *)
{
  return &g_fake_description;
}
}  // namespace

TEST_F(CorrRuntime, AMsgPackageFeedbackTypeOnAnActionFeedbackTopicIsStillJudged)
{
  // An action feedback type is always
  // `<pkg>/action/<Name>_FeedbackMessage`. A message type of a msg package with
  // that suffix, on an action feedback topic, is a user publisher.
  rt::reset_for_test();
  const std::string feedback = "/custom/Foo/_action/feedback";
  auto p = pub_node->create_publisher<std_msgs::msg::Int32>(
    feedback, rclcpp::QoS(10).best_effort());
  // Replay the init hook with the type name such a message type carries.
  static char name[] = "custom_msgs/msg/Telemetry_FeedbackMessage";
  g_fake_description = {};
  g_fake_description.type_description.type_name.data = name;
  g_fake_description.type_description.type_name.size = sizeof(name) - 1;
  g_fake_description.type_description.type_name.capacity = sizeof(name);
  rosidl_message_type_support_t ts{};
  ts.get_type_description_func = &fake_description;
  rcllf_gate_on_init(
    p->get_publisher_handle().get(),
    pub_node->get_node_base_interface()->get_rcl_node_handle(), &ts, RCL_RET_OK);
  ros2_lf_binding::register_publisher_qos("Pub", feedback, "");
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(
    err.find(
      "RCLLF-H000 publisher of node '" + std::string(pub_node->get_fully_qualified_name()) +
      "' on '" + feedback + "': the analysis saw no reliability, the publisher runs best "
      "effort"), std::string::npos)
    << "the custom_msgs/msg/Telemetry_FeedbackMessage publisher on " << feedback
    << " was taken for the node's own action feedback publisher: " << err;
}

TEST_F(CorrRuntime, TheEgressDrainWaitsOnlyOnTrackedPublishers)
{
  // rcl initializes its rosout publisher in a stack temporary and
  // copies it into its own map, so the gate's key for it points nowhere once
  // the process stops; its fini never reaches the gate either. The egress
  // drain may dereference the keys of tracked publishers only (bound to an
  // admitted edge): one here, on `topic`, bound by startup.
  // An untracked reliable publisher of the node, and a rosout-like stray: a
  // copy of a live handle registered from a temporary, then invalidated.
  auto other = pub_node->create_publisher<std_msgs::msg::Int32>(
    topic + "_untracked", rclcpp::QoS(10));
  auto stray = std::make_unique<rcl_publisher_t>(*other->get_publisher_handle());
  rcllf_gate_on_init(
    stray.get(), pub_node->get_node_base_interface()->get_rcl_node_handle(), nullptr,
    RCL_RET_OK);
  *stray = rcl_get_zero_initialized_publisher();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const auto r = rt::drain_egress(50);
  EXPECT_EQ(r.waited, 1u)
    << "the egress drain waited on " << r.waited
    << " publisher(s), not only the one tracked publisher of edge 0";
  EXPECT_EQ(r.unacked, 0u);
}

TEST_F(CorrRuntime, TwinLegacyReadersAreJudgedAgainstEveryAnalyzedTwin)
{
  // One node, two subscriptions on the topic, one legacy edge
  // each. The analysis saw the best-effort depth-1 twin, so neither edge may
  // warn about it.
  rt::reset_for_test();
  const std::string t = topic + "_legacy_twins";
  auto s1 = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(1).best_effort(), [](const std_msgs::msg::Int32 &) {});
  auto s2 = sub_node->create_subscription<std_msgs::msg::Int32>(
    t, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_legacy_edge(
    t, "Pub", "Sub", "transient_local", t, "best_effort", 1);
  ros2_lf_binding::register_corr_legacy_edge(t, "Pub", "Sub", "transient_local", t, "reliable", 10);
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("RCLLF-H00"), std::string::npos)
    << "a twin's edge warned about QoS the analysis saw on the other twin: " << err;
}

TEST_F(CorrRuntime, NoLegacyHazardWhenTheAnalysisSawItOrTheReaderIsSafe)
{
  rt::reset_for_test();
  const std::string seen_t = topic + "_legacy_seen";
  const std::string safe_t = topic + "_legacy_safe";
  auto s1 = sub_node->create_subscription<std_msgs::msg::Int32>(
    seen_t, rclcpp::QoS(1).best_effort(), [](const std_msgs::msg::Int32 &) {});
  auto s2 = sub_node->create_subscription<std_msgs::msg::Int32>(
    safe_t, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::register_corr_legacy_edge(
    seen_t, "Pub", "Sub", "transient_local", seen_t, "best_effort", 1);
  ros2_lf_binding::register_corr_legacy_edge(
    safe_t, "Pub", "Sub", "transient_local", safe_t, "", -1);
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(rt::startup(resolver()).has_value());
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("RCLLF-H00"), std::string::npos) << err;
  EXPECT_NE(err.find("legacy edge " + safe_t), std::string::npos) << err;
}

TEST_F(CorrRuntime, RegisterRefusesAnEdgeWithoutOrWithDuplicatePublishingRids)
{
  rt::reset_for_test();
  ros2_lf_binding::register_corr_edge({0, "Pub", {}, topic, "Sub", kSubRid, topic, 0});
  auto f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "publisher_rids");
  rt::reset_for_test();
  ros2_lf_binding::register_corr_edge(
    {0, "Pub", {kPubRid, kPubRid}, topic, "Sub", kSubRid, topic, 0});
  f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "publisher_rids");
}

TEST_F(CorrRuntime, DrainOnIngressThreadIsRefused)
{
  ASSERT_FALSE(rt::startup(resolver()).has_value());
  ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::INGRESS);
  EXPECT_EQ(ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str()), -1);
}

TEST_F(CorrRuntime, StartupRefusesAsyncPublicationMode)
{
  setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", 1);
  auto f = rt::startup(resolver());
  unsetenv("RMW_FASTRTPS_PUBLICATION_MODE");
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "publication_mode");
}

TEST_F(CorrRuntime, StartupRefusesAnXmlProfile)
{
  setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/tmp/none.xml", 1);
  auto f = rt::startup(resolver());
  unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "xml_profile");
}

TEST_F(CorrRuntime, StartupRefusesTransientLocal)
{
  auto tl = pub_node->create_publisher<std_msgs::msg::Int32>(
    topic + "_tl", rclcpp::QoS(1).transient_local());
  auto tl_sub = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic + "_tl", rclcpp::QoS(1).transient_local(), [](const std_msgs::msg::Int32 &) {});
  rt::reset_for_test();
  ros2_lf_binding::register_corr_edge({0, "Pub", {kPubRid}, topic + "_tl", "Sub", kSubRid,
      topic + "_tl", 0});
  auto f = rt::startup(resolver());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->check, "durability");
  EXPECT_EQ(f->edge, 0);
}

TEST_F(CorrRuntime, StartupFailureExitsNonzeroNamingEdgeAndCheck)
{
  EXPECT_EXIT(
    {
      setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", 1);
      rt::startup_or_exit(resolver());
  }, ::testing::ExitedWithCode(1), "startup refused admitted edge 0 .*check publication_mode");
}

// The analysis lists two subscriptions of "Sub" on
// the topic (twins), but the node creates one at run time. Startup must
// refuse instead of handing that one handle to both twin edges. Falsify by
// testing the twin count only when more than one subscription matches: the
// startup then succeeds and the first assertion fails with the message below.
TEST_F(CorrRuntime, AnalyzedTwinsWithOneRuntimeSubscriptionRefuse)
{
  ros2_lf_binding::clear_rid_entries_for_test();
  ros2_lf_binding::register_rid_entry({kSubRid, "Sub", "on_first", "sub", topic, 0, 0, -1});
  ros2_lf_binding::register_rid_entry({kSubRid + 1, "Sub", "on_second", "sub", topic, 0, 1, -1});
  const auto f = rt::startup(resolver());
  ros2_lf_binding::clear_rid_entries_for_test();
  ASSERT_TRUE(f.has_value()) <<
    "startup admitted both twin edges with the one run-time subscription of the node";
  EXPECT_EQ(f->check, "endpoint");
  EXPECT_NE(
    f->detail.find("the analysis found 2 subscriptions of the node on this topic and 1 match"),
    std::string::npos) << f->detail;
}

// The matching case: two analyzed twins and two run-time subscriptions in the
// default group start, the rank picks one handle.
TEST_F(CorrRuntime, AnalyzedTwinsWithTwoRuntimeSubscriptionsStart)
{
  auto second = sub_node->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
  ros2_lf_binding::clear_rid_entries_for_test();
  ros2_lf_binding::register_rid_entry({kSubRid, "Sub", "on_first", "sub", topic, 0, 0, -1});
  ros2_lf_binding::register_rid_entry({kSubRid + 1, "Sub", "on_second", "sub", topic, 0, 1, -1});
  const auto f = rt::startup(resolver());
  ros2_lf_binding::clear_rid_entries_for_test();
  EXPECT_FALSE(f.has_value()) << f->check << ": " << f->detail;
}

// An egress edge whose node is gone before the peer
// check must not start unchecked. It is refused like the boot id check.
TEST_F(CorrRuntime, XfedPeerCheckRefusesAnEdgeWhoseNodeIsGone)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto drop_node_then_check = [this]() {
      ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {kPubRid}, topic, "", -1, topic, 5000000};
      s.role = ros2_lf_binding::CorrEdgeRole::kEgress;
      s.global_id = 0;
      s.peer_node = "/corr_peer";
      ros2_lf_binding::register_corr_edge(s);
      const auto f = rt::startup(resolver());
      if (f) {
        std::fprintf(stderr, "startup failed: %s: %s\n", f->check.c_str(), f->detail.c_str());
        std::_Exit(1);
      }
      pub.reset();
      pub_node.reset();
      setenv("ROS2_LF_XFED_CLOCK_SYNCHRONIZED", "1", 1);
      rt::xfed_peer_check_or_exit(0);
      std::_Exit(0);
    };
  EXPECT_EXIT(
    drop_node_then_check(), ::testing::ExitedWithCode(3),
    "startup refused egress edge 0 \\(global 0\\).*check peer");
}

// The peer check of a cross-federate edge whose lifecycle end makes its
// endpoint in a hook. Before tag 0 that endpoint does not exist yet, so the
// check is skipped for it, and neither federate of a sub_rebind edge exits.
class XfedRebindPeerCheck : public CorrRuntime
{
protected:
  /// Register `s` alone, start, run the peer check with no wait and exit 0.
  void check_only(ros2_lf_binding::CorrEdgeSpec s)
  {
    rt::reset_for_test();
    s.global_id = 0;
    ros2_lf_binding::register_corr_edge(s);
    const auto f = rt::startup(resolver());
    if (f) {
      std::fprintf(stderr, "startup failed: %s: %s\n", f->check.c_str(), f->detail.c_str());
      std::_Exit(1);
    }
    setenv("ROS2_LF_XFED_CLOCK_SYNCHRONIZED", "1", 1);
    rt::xfed_peer_check_or_exit(0);
    std::_Exit(0);
  }

  ros2_lf_binding::CorrEdgeSpec ingress_spec()
  {
    ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {}, topic, "Sub", kSubRid, topic, 5000000};
    s.role = ros2_lf_binding::CorrEdgeRole::kIngress;
    s.peer_node = pub_node->get_fully_qualified_name();
    return s;
  }

  ros2_lf_binding::CorrEdgeSpec egress_spec()
  {
    ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {kPubRid}, topic, "", -1, topic, 5000000};
    s.role = ros2_lf_binding::CorrEdgeRole::kEgress;
    s.peer_node = sub_node->get_fully_qualified_name();
    return s;
  }
};

TEST_F(XfedRebindPeerCheck, AnIngressReaderMadeInAHookSkipsTheCheck)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      sub.reset();  // on_configure has not run: no reader yet
      ros2_lf_binding::g_handle_table[kSubRid].subscription.reset();
      auto s = ingress_spec();
      s.sub_rebind = true;
      check_only(s);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(0),
    "ingress edge 0 skips its peer check.*its local reader is made in a lifecycle hook");
}

TEST_F(XfedRebindPeerCheck, AnEgressWhosePeerReaderIsMadeInAHookSkipsTheCheck)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      sub.reset();  // the peer federate's reader comes with its on_configure
      auto s = egress_spec();
      s.sub_rebind = true;
      check_only(s);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(0),
    "egress edge 0 skips its peer check.*the peer's reader is made in a lifecycle hook");
}

TEST_F(XfedRebindPeerCheck, AnEgressLifecyclePublisherNotYetActiveSkipsTheCheck)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      pub.reset();  // the publisher comes with the node's configure at tag 0
      auto s = egress_spec();
      s.pub_rebind = true;
      check_only(s);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(0),
    "egress edge 0 skips its peer check.*its local publisher is made in a lifecycle hook");
}

// The skip covers only an absent hook-made end. A missing peer that is not
// made in a hook, and a present peer, are still checked.
TEST_F(XfedRebindPeerCheck, AMissingPeerNotMadeInAHookIsStillRefused)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      sub.reset();
      check_only(egress_spec());
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(3),
    "startup refused egress edge 0 \\(global 0\\).*check peer_check: 0 endpoint\\(s\\) of the "
    "peer node");
}

TEST_F(XfedRebindPeerCheck, APresentPeerOfARebindEdgeIsStillChecked)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      // A second reader of the peer node: no designated peer to compare with.
      auto twin = sub_node->create_subscription<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
      ASSERT_TRUE(publisher_matched(pub));
      auto s = egress_spec();
      s.sub_rebind = true;
      check_only(s);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(3),
    "startup refused egress edge 0 \\(global 0\\).*check peer_check: 2 endpoint\\(s\\) of the "
    "peer node");
}

// An end that skipped the startup check meets its present peer
// when it is bound, with the comparison D7 makes (type, QoS).
TEST_F(XfedRebindPeerCheck, AnAbsentLocalEndWithATwinPeerIsRefusedAtStartup)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      pub.reset();
      auto twin = sub_node->create_subscription<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
      auto s = egress_spec();
      s.pub_rebind = true;
      check_only(s);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(3),
    "startup refused egress edge 0 \\(global 0\\).*check peer_check: 2 endpoint\\(s\\) of the "
    "peer node");
}

class XfedRebindBind : public XfedRebindPeerCheck
{
protected:
  /// Start `s` with its hook-made end absent, then make that end as a hook does.
  void start(ros2_lf_binding::CorrEdgeSpec s)
  {
    rt::reset_for_test();
    s.global_id = 0;
    ros2_lf_binding::register_corr_edge(s);
    const auto f = rt::startup(resolver());
    if (f) {
      std::fprintf(stderr, "startup failed: %s: %s\n", f->check.c_str(), f->detail.c_str());
      std::_Exit(1);
    }
  }

  /// The peer's reader is the fixture's reliable one: a publisher made with
  /// `qos` meets it when bound. (One participant holds one type per topic, so
  /// a peer of another type needs another process; QoS shows the comparison.)
  void publisher_with_peer(const rclcpp::QoS & qos)
  {
    pub.reset();
    auto s = egress_spec();
    s.pub_rebind = true;
    start(s);
    pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, qos);  // on_configure
    std::_Exit(0);
  }
};

TEST_F(XfedRebindBind, AHookMadePublisherIsRefusedByAPeerOfIncompatibleQos)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  // A best-effort publisher never serves the peer's reliable reader.
  EXPECT_EXIT(
    publisher_with_peer(rclcpp::QoS(10).best_effort()), ::testing::ExitedWithCode(3),
    "bind refused egress edge 0 on '.*': check peer_check: the peer endpoint's QoS is "
    "incompatible");
}

TEST_F(XfedRebindBind, AHookMadePublisherMatchingItsPeerIsBound)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    publisher_with_peer(rclcpp::QoS(10)), ::testing::ExitedWithCode(0),
    "bound .*'s publisher on '.*' to edge 0");
}

TEST_F(XfedRebindBind, AHookMadeReaderIsRefusedByAPeerOfIncompatibleQos)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  auto run = [this]() {
      // The peer publishes best effort; a reliable reader never hears it.
      pub = pub_node->create_publisher<std_msgs::msg::Int32>(
        topic, rclcpp::QoS(10).best_effort());
      sub.reset();
      ros2_lf_binding::g_handle_table[kSubRid].subscription.reset();
      auto s = ingress_spec();
      s.sub_rebind = true;
      start(s);
      ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
      const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      ros2_lf_binding::set_lf_tag_for_test(std::make_pair(now, 0u));
      auto r = sub_node->create_subscription<std_msgs::msg::Int32>(  // on_configure
        topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {});
      ros2_lf_binding::g_handle_table[kSubRid].subscription = r;
      rt::on_subscription_bound(kSubRid, r);
      std::_Exit(0);
    };
  EXPECT_EXIT(
    run(), ::testing::ExitedWithCode(0),
    "edge 0 refused its new reader, peer_check: the peer endpoint's QoS is incompatible");
}

// A cross-federate record that arrives after
// its intended tag publish time + d. On a decentralized ingress edge
// (set_corr_edge_tardy_drop) each of its messages ends LOST once, through the
// message_lost handler at the arrival tag, with its intended tag in the lost
// context. Without the flag (centralized) the same record exits 3.
class XfedTardy : public XfedRebindBind
{
protected:
  static constexpr int64_t kD = 5000000;
  static constexpr int64_t kLate = 1000000;

  /// Start an ingress edge, deliver two records of `pub_time` at
  /// pub_time + d + kLate, drain, and print what the message_lost handler saw.
  /// With `throws` the handler throws, the caller catches it, and the test
  /// prints whether the lost context still reads valid outside any handler.
  void deliver_late(bool tardy_drop, bool throws = false)
  {
    const int64_t pub_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::vector<rcllf_lost_ctx_t> seen;
    int calls = 0;
    // The edge's reader is the first its node makes on the topic: a fresh node.
    sub.reset();
    sub_node = std::make_shared<rclcpp::Node>("corr_sub_tardy_" + std::to_string(counter));
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.message_lost_callback =
      [&seen, &calls, throws](rclcpp::QOSMessageLostInfo &) {
        ++calls;
        rcllf_lost_ctx_t ctx{};
        if (ros2_lf_message_lost_context(&ctx) == 1) {
          seen.push_back(ctx);
        }
        if (throws) {
          throw std::runtime_error("user message_lost handler failed");
        }
      };
    sub = sub_node->create_subscription<std_msgs::msg::Int32>(
      topic, rclcpp::QoS(10), [](const std_msgs::msg::Int32 &) {}, options);
    ros2_lf_binding::g_handle_table[kSubRid].subscription = sub;
    start(ingress_spec());
    if (tardy_drop) {
      ros2_lf_binding::set_corr_edge_tardy_drop(0);
    }
    setenv("ROS2_LF_XFED_CLOCK_SYNCHRONIZED", "1", 1);
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    const int64_t now = pub_time + kD + kLate;
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(now, 0u));
    ros2_lf_binding::corr::Gid g{};
    g[0] = 7;
    const std::vector<ros2_lf_binding::corr::XRecord> recs{{g, 1, 0}, {g, 2, 0}};
    rcllf_xcorr_t wire{};
    rt::pack_xcorr(recs, &wire);
    wire.magic = RCLLF_XCORR_MAGIC;
    wire.pub_time = pub_time;
    int64_t w = ros2_lf_corr_arrive_remote(nullptr, 0, &wire);
    if (throws) {
      bool caught = false;
      try {
        ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
      } catch (const std::runtime_error &) {
        caught = true;
      }
      rcllf_lost_ctx_t after{};
      const int valid = ros2_lf_message_lost_context(&after);
      std::fprintf(
        stderr, "handler threw caught=%d calls=%d context outside a handler valid=%d\n",
        caught ? 1 : 0, calls, valid);
      std::_Exit(caught && calls == 1 && seen.size() == 1 && valid == 0 ? 0 : 1);
    }
    while (w >= 0) {
      w = ros2_lf_corr_drain(nullptr, 0, kSubRid, topic.c_str());
    }
    const int64_t start_time = std::max<int64_t>(0, ros2_lf_binding::lf_start_tag().first);
    int ok = 0;
    for (size_t i = 0; i < seen.size(); ++i) {
      ok += seen[i].tardy == 1 && seen[i].ordinal == i + 1 &&
        seen[i].intended_elapsed_ns == pub_time + kD - start_time && seen[i].gid[0] == 7;
    }
    std::fprintf(stderr, "tardy lost calls=%d context=%zu matching=%d\n", calls, seen.size(), ok);
    std::_Exit(calls == 2 && ok == 2 ? 0 : 1);
  }

  /// A fast federate's tag runs about 1 s ahead of physical time,
  /// and its publisher stamps the record with that tag. The receiver must
  /// judge the stamp on its own logical clock, not on physical time.
  void deliver_fast_logical_record()
  {
    const int64_t phys = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const int64_t pub_time = phys + 1000000000;
    start(ingress_spec());
    rt::set_federated(true);
    ros2_lf_binding::set_lf_fast_for_test(true);
    ros2_lf_binding::ScopedThreadRoleForTest role(ros2_lf_binding::ThreadRole::LF_WORKER);
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(pub_time + kD, 0u));
    ros2_lf_binding::corr::Gid g{};
    g[0] = 7;
    const std::vector<ros2_lf_binding::corr::XRecord> recs{{g, 1, pub_time}};
    rcllf_xcorr_t wire{};
    rt::pack_xcorr(recs, &wire);
    wire.magic = RCLLF_XCORR_MAGIC;
    wire.pub_time = pub_time;
    const auto boot = rt::read_boot_id(rt::kBootIdPath);  // one host, one clock
    ASSERT_TRUE(boot.has_value());
    std::copy(boot->begin(), boot->end(), wire.boot_id);
    ros2_lf_corr_arrive_remote(nullptr, 0, &wire);
    std::fprintf(stderr, "fast record accepted\n");
    std::_Exit(0);
  }
};

TEST_F(XfedTardy, ALateRecordOnADecentralizedEdgeIsLostOnceNotFatal)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    deliver_late(true), ::testing::ExitedWithCode(0),
    "tardy lost calls=2 context=2 matching=2");
}

TEST_F(XfedTardy, AThrowingLostHandlerLeavesNoStaleContext)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    deliver_late(true, true), ::testing::ExitedWithCode(0),
    "handler threw caught=1 calls=1 context outside a handler valid=0");
}

TEST_F(XfedTardy, ALateRecordOnACentralizedEdgeStaysFatal)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    deliver_late(false), ::testing::ExitedWithCode(3),
    "cross-federate records of time [0-9]+ arrived at [0-9]+, not at publish time \\+ 5000000 ns");
}

TEST_F(XfedTardy, AFastFederateAcceptsARecordStampedWithItsLogicalTag)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    deliver_fast_logical_record(), ::testing::ExitedWithCode(0), "fast record accepted");
}

TEST(UserCallbackGate, ClosingWaitsForTheRunningCallbackAndRefusesLaterOnes)
{
  // The pre-shutdown callback closes this gate before rcl_shutdown,
  // so no user callback (with a loaned publish in it) meets an invalid context.
  ros2_lf_binding::reopen_user_callbacks_for_test();
  std::atomic<bool> inside{false};
  std::atomic<bool> release{false};
  std::thread callback([&]() {
      ros2_lf_binding::UserCallbackScope scope;
      EXPECT_TRUE(scope.admitted());
      inside = true;
      while (!release) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  while (!inside) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::atomic<bool> closed{false};
  bool closed_in_time = false;
  std::thread closer([&]() {
      closed_in_time = ros2_lf_binding::close_user_callbacks(std::chrono::milliseconds(5000));
      closed = true;
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(closed) << "closing returned while a user callback was still running";
  release = true;
  callback.join();
  closer.join();
  EXPECT_TRUE(closed_in_time);
  const uint64_t skipped = ros2_lf_binding::g_user_callbacks_skipped.load();
  {
    ros2_lf_binding::UserCallbackScope late;
    EXPECT_FALSE(late.admitted()) << "a user callback was admitted after shutdown began";
  }
  EXPECT_EQ(ros2_lf_binding::g_user_callbacks_skipped.load(), skipped + 1);
  ros2_lf_binding::reopen_user_callbacks_for_test();
}

// The peer check names the designated peer node, not a match
// count, and refuses a wrong type or an incompatible QoS.
rt::PeerEndpoint peer_endpoint(const std::string & fqn, rmw_qos_reliability_policy_t rel)
{
  rt::PeerEndpoint p{fqn, "std_msgs/msg/Int32", rmw_qos_profile_default};
  p.qos.reliability = rel;
  return p;
}

TEST(XfedPeerCheck, DesignatedPeerWithMatchingTypeAndQosPasses)
{
  const auto local = peer_endpoint("/sub", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  const std::vector<rt::PeerEndpoint> peers{
    peer_endpoint("/other", RMW_QOS_POLICY_RELIABILITY_RELIABLE),
    peer_endpoint("/pub", RMW_QOS_POLICY_RELIABILITY_RELIABLE)};
  EXPECT_EQ(rt::xfed_peer_problem(peers, local, "/pub", false), "");
}

TEST(XfedPeerCheck, AnExternalEndpointIsNotThePeer)
{
  const auto local = peer_endpoint("/sub", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  const std::vector<rt::PeerEndpoint> peers{
    peer_endpoint("/rosbag", RMW_QOS_POLICY_RELIABILITY_RELIABLE)};
  EXPECT_EQ(
    rt::xfed_peer_problem(peers, local, "/pub", false),
    "0 endpoint(s) of the peer node '/pub' among 1 on the topic");
}

TEST(XfedPeerCheck, WrongTypeIsRefused)
{
  const auto local = peer_endpoint("/sub", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  auto peer = peer_endpoint("/pub", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  peer.type = "std_msgs/msg/String";
  EXPECT_EQ(
    rt::xfed_peer_problem({peer}, local, "/pub", false),
    "the peer endpoint has type 'std_msgs/msg/String', not 'std_msgs/msg/Int32'");
}

TEST(XfedPeerCheck, BestEffortPublisherToReliableReaderIsRefused)
{
  const auto local = peer_endpoint("/sub", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  const auto peer = peer_endpoint("/pub", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  const std::string problem = rt::xfed_peer_problem({peer}, local, "/pub", false);
  EXPECT_EQ(problem.rfind("the peer endpoint's QoS is incompatible", 0), 0u) << problem;
  // The same pair seen from the publisher side is refused too.
  EXPECT_NE(rt::xfed_peer_problem({local}, peer, "/sub", true), "");
}

std::string boot_id_file(const std::string & name, const std::string & text)
{
  const std::string path = ::testing::TempDir() + "rcllf_boot_id_" + name;
  std::ofstream(path) << text;
  return path;
}

std::string hex(const std::array<uint8_t, 16> & b)
{
  std::string out;
  char buf[3];
  for (uint8_t x : b) {
    std::snprintf(buf, sizeof(buf), "%02x", x);
    out += buf;
  }
  return out;
}

// An unreadable or short boot id must not become
// the all-zero id, two such hosts would look like one clock domain.
// Two records of one tag cross the wire, the second
// committed after the cutoff. Both keep the tag cutoff t + d + T: the commit
// stamp never moves the window (no publisher_late counter), a late
// sample is the receiver's LATE verdict at its join.
TEST(XfedWire, EveryRecordOfATagCutsAtThePublicationTag)
{
  constexpr int64_t kD = 5000000;
  constexpr int64_t kT = 100000000;
  constexpr int64_t kTag = 1000000000;
  constexpr int64_t kCut = kTag + kD + kT;
  ros2_lf_binding::corr::Table t{61};
  t.set_clock([] {return kTag + kD;});
  t.add_edge(0, 7, kD, kT, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  ros2_lf_binding::corr::Gid g{};
  g[0] = 9;
  // Both records come from one activation that started at its tag.
  const std::vector<ros2_lf_binding::corr::XRecord> sent{
    {g, 1, kTag, -1, kTag}, {g, 2, kCut + 1, -1, kTag}};
  rcllf_xcorr_t wire{};
  rt::pack_xcorr(sent, &wire);
  const auto recs = rt::unpack_xcorr(wire);
  ASSERT_EQ(recs.size(), 2u);
  EXPECT_EQ(recs[0].phys, kTag);
  EXPECT_EQ(recs[1].phys, kCut + 1);
  t.arrive_remote(0, recs, kTag, kTag + kD, 0);
  std::vector<int64_t> cuts;
  while (auto tok = t.pop_released(0, kTag + kD)) {
    cuts.push_back(t.cutoff(*tok));
    t.next_wake(0, kTag + kD);
  }
  EXPECT_EQ(cuts, (std::vector<int64_t>{kCut, kCut})) <<
    "expected both records cut at t + d + T = " << kCut << "";
  t.close_epoch();
  std::string all;
  for (const auto & p : t.reconcile().problems) {
    all += p + "\n";
  }
  EXPECT_EQ(all.find("publisher_late"), std::string::npos) << all;
}

TEST(XfedBootId, AWellFormedBootIdIsRead)
{
  const auto id = rt::read_boot_id(
    boot_id_file("ok", "0f1e2d3c-4b5a-6978-8796-a5b4c3d2e1f0\n"));
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(hex(*id), "0f1e2d3c4b5a69788796a5b4c3d2e1f0");
}

TEST(XfedBootId, AMissingFileGivesNoId)
{
  const auto id = rt::read_boot_id(::testing::TempDir() + "rcllf_boot_id_does_not_exist");
  EXPECT_FALSE(id.has_value()) << "accepted boot id " << hex(*id);
}

TEST(XfedBootId, AShortBootIdGivesNoId)
{
  const auto id = rt::read_boot_id(boot_id_file("short", "0f1e2d3c-4b5a\n"));
  EXPECT_FALSE(id.has_value()) << "accepted boot id " << hex(*id);
}

TEST(XfedBootId, ALongOrNonHexBootIdGivesNoId)
{
  const auto lng = rt::read_boot_id(
    boot_id_file("long", "0f1e2d3c-4b5a-6978-8796-a5b4c3d2e1f0aa\n"));
  EXPECT_FALSE(lng.has_value()) << "accepted boot id " << hex(*lng);
  const auto bad = rt::read_boot_id(
    boot_id_file("nonhex", "zz1e2d3c-4b5a-6978-8796-a5b4c3d2e1f0\n"));
  EXPECT_FALSE(bad.has_value()) << "accepted boot id " << hex(*bad);
}

TEST(XfedBootId, StartupRefusesWithoutABootIdUnlessSynchronized)
{
  const std::string missing = ::testing::TempDir() + "rcllf_boot_id_does_not_exist";
  const std::string problem = rt::xfed_clock_problem(missing, false);
  EXPECT_EQ(problem.rfind("no 16-byte boot id in '" + missing + "'", 0), 0u) << problem;
  EXPECT_EQ(rt::xfed_clock_problem(missing, true), "");
  EXPECT_EQ(
    rt::xfed_clock_problem(
      boot_id_file("ok2", "0f1e2d3c-4b5a-6978-8796-a5b4c3d2e1f0\n"), false), "");
}

// The declared clock offset bound. ROS2_LF_XFED_CLOCK_BOUND_NS gives eps,
// ROS2_LF_XFED_CLOCK_SYNCHRONIZED=1 alone declares eps 0, nothing is undeclared.
TEST(XfedClockBound, TheBoundIsReadFromTheEnvironmentText)
{
  const auto none = rt::xfed_clock_bound_from(nullptr, nullptr);
  EXPECT_FALSE(none.declared);
  EXPECT_EQ(none.problem, "");
  const auto eps = rt::xfed_clock_bound_from("2000000", nullptr);
  EXPECT_TRUE(eps.declared);
  EXPECT_EQ(eps.eps_ns, 2000000);
  EXPECT_EQ(eps.problem, "");
  const auto both = rt::xfed_clock_bound_from("30000000", "1");
  EXPECT_TRUE(both.declared);
  EXPECT_EQ(both.eps_ns, 30000000);
  const auto sync = rt::xfed_clock_bound_from(nullptr, "1");
  EXPECT_TRUE(sync.declared);
  EXPECT_EQ(sync.eps_ns, 0);
  EXPECT_FALSE(rt::xfed_clock_bound_from(nullptr, "0").declared);
  for (const char * bad : {"", "-5", "1e6", "12ms", " 5", "99999999999999999999"}) {
    const auto b = rt::xfed_clock_bound_from(bad, nullptr);
    EXPECT_NE(b.problem, "") << "accepted bound '" << bad << "'";
    EXPECT_FALSE(b.declared) << bad;
  }
}

TEST(XfedClockBound, AnotherHostIsRefusedWithoutADeclaredBound)
{
  const auto none = rt::xfed_clock_bound_from(nullptr, nullptr);
  const std::string problem = rt::xfed_host_problem(false, none);
  EXPECT_NE(problem.find("boot id differs"), std::string::npos) << problem;
  EXPECT_NE(problem.find("ROS2_LF_XFED_CLOCK_BOUND_NS"), std::string::npos) << problem;
  EXPECT_EQ(rt::xfed_host_problem(true, none), "");
  EXPECT_EQ(rt::xfed_host_problem(false, rt::xfed_clock_bound_from("1000", nullptr)), "");
}

TEST(XfedClockBound, ARecordStampedPastTheBoundBreaksIt)
{
  constexpr int64_t kNow = 5000000000;
  EXPECT_FALSE(rt::xfed_record_beyond_bound(kNow + 2000, kNow, 2000));
  EXPECT_TRUE(rt::xfed_record_beyond_bound(kNow + 2001, kNow, 2000));
  EXPECT_FALSE(rt::xfed_record_beyond_bound(kNow, kNow, 0));
  EXPECT_TRUE(rt::xfed_record_beyond_bound(kNow + 1, kNow, 0));
}

// Startup sets the declared bound on every ingress edge,
// but a record whose boot id equals the receiver's shares its clock, so its
// cutoff is t + d + T without the declared 2 ms bound (the receiver's
// cutoff, no publisher_late counter).
TEST(XfedClockBound, ASameHostRecordIsJudgedWithoutTheDeclaredBound)
{
  constexpr int64_t kD = 5000000;
  constexpr int64_t kT = 100000000;
  constexpr int64_t kTag = 1000000000;
  constexpr int64_t kCut = kTag + kD + kT;
  const auto bound = rt::xfed_clock_bound_from("2000000", nullptr);
  ros2_lf_binding::corr::Table t{62};
  t.set_clock([] {return kTag + kD;});
  t.add_edge(0, 7, kD, kT, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  t.set_clock_bound(0, bound.eps_ns);  // as at startup
  ros2_lf_binding::corr::Gid g{};
  g[0] = 9;
  const std::vector<ros2_lf_binding::corr::XRecord> recs{{g, 1, kCut + 1000000, -1, kTag}};
  EXPECT_EQ(rt::xfed_judge_bound(t, 0, true, bound), 0);
  t.arrive_remote(0, recs, kTag, kTag + kD, 0);
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(t.cutoff(*tok), kCut) << "expected the same-host cutoff t + d + T = " << kCut <<
    ", got " << t.cutoff(*tok) << " (t + d + T + eps is " << kCut + bound.eps_ns << ")";
}

// With a declared bound startup admits a host without a
// readable boot id, and such a host sends zeros. A zero id from the peer must
// not make an unreadable host judge it as its own clock with eps 0.
TEST(XfedClockBound, AHostWithoutABootIdNeverSharesTheClock)
{
  const uint8_t zeros[16] = {};
  std::array<uint8_t, 16> id{};
  id[0] = 7;
  EXPECT_FALSE(rt::xfed_same_boot(zeros, std::nullopt));
  EXPECT_FALSE(rt::xfed_same_boot(zeros, id));
  EXPECT_TRUE(rt::xfed_same_boot(id.data(), id));
}

TEST(XfedClockBound, ABoundAtOrAboveTheTimeoutIsRefused)
{
  EXPECT_EQ(rt::xfed_bound_problem(99999999, 100000000), "");
  EXPECT_NE(rt::xfed_bound_problem(100000000, 100000000), "");
  EXPECT_EQ(rt::xfed_bound_problem(0, 100000000), "");
}

// A legacy stub link with `after d` maps every microstep of one
// time to one tag. reactor-c keeps two (the second moves to microstep 1) and
// merges a third, so the guard counts the distinct microsteps per time.
TEST(XfedLegacyGuard, CountsDistinctMicrostepsPerTimeAndPort)
{
  rt::LegacyGuard g;
  EXPECT_EQ(g.note("b", 10, 0), 1u);
  EXPECT_EQ(g.note("b", 10, 0), 1u) << "the same tag again is one message";
  EXPECT_EQ(g.note("b", 10, 1), 2u);
  EXPECT_EQ(g.note("c", 10, 1), 1u) << "ports are independent";
  EXPECT_EQ(g.note("b", 10, 2), 3u);
  EXPECT_EQ(g.note("b", 20, 3), 1u) << "a new time starts over";
  EXPECT_LE(rt::LegacyGuard::kMaxPerTime, 2u);
  // Decentralized refuses the second microstep of one time.
  EXPECT_EQ(rt::LegacyGuard::kMaxPerTimeDecentralized, 1u);
}

}  // namespace

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


// AcquisitionLookup against a real tf2_ros::Buffer, with the rcllf runtime
// hooks defined here (strong definitions override the adapter's weak
// declarations), so the test plays the LF scheduler: it sets the tag, feeds
// the Buffer as an ingest reaction would and delivers the requested wakes.
//
// Corpus (hand-derived): sensor_link moves along x at 1 m/s, one /tf sample
// every 20 ms, so tf2's linear interpolation gives x(t) = t in seconds.
// wait_bound is 55 ms.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcllf_tf2/acquisition_lookup.hpp"

namespace
{
int64_t g_tag = -1;
bool g_constructing = true;
std::vector<rcllf_tf2_hooks> g_hooks;
std::vector<int64_t> g_wakes;
std::vector<std::string> g_trace;
}  // namespace

extern "C" {
int ros2_lf_tf2_active(void) {return 1;}
int ros2_lf_tf2_register(const rcllf_tf2_hooks * hooks)
{
  if (!g_constructing) {
    return RCLLF_TF2_NOT_CONSTRUCTING;
  }
  g_hooks.push_back(*hooks);
  return static_cast<int>(g_hooks.size() - 1);
}
void ros2_lf_tf2_unregister(int handle) {g_hooks[handle] = rcllf_tf2_hooks{};}
int ros2_lf_tf2_current(int64_t * time, uint32_t * micro)
{
  *time = g_tag;
  *micro = 0;
  return g_tag < 0 ? -1 : 0;
}
void ros2_lf_tf2_request_wake(int, int64_t deadline) {g_wakes.push_back(deadline);}
void ros2_lf_tf2_trace(const char * json) {g_trace.emplace_back(json);}
}

namespace
{
constexpr int64_t kMs = 1000000;

class AcquisitionLookupTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    g_tag = -1;
    g_constructing = true;
    g_hooks.clear();
    g_wakes.clear();
    g_trace.clear();
    node_ = std::make_shared<rclcpp::Node>("acq_test");
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  }

  /// An ingest reaction at tag `ms`: /tf sample stamped `ms`, x = ms / 1000.
  void ingest(int64_t ms)
  {
    g_tag = ms * kMs;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(ms * kMs);
    t.header.frame_id = "world";
    t.child_frame_id = "sensor_link";
    t.transform.translation.x = static_cast<double>(ms) / 1000.0;
    t.transform.rotation.w = 1.0;
    buffer_->setTransform(t, "test", false);
    for (auto & h : g_hooks) {
      if (h.ingested) {h.ingested(h.ctx, g_tag, 0);}
    }
  }

  void wake(int64_t ns)
  {
    g_tag = ns;
    for (auto & h : g_hooks) {
      if (h.wake) {h.wake(h.ctx, g_tag, 0);}
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::vector<rcllf_tf2::Result> results_;
  rcllf_tf2::AcquisitionLookup::Callback collect()
  {
    return [this](const rcllf_tf2::Result & r) {results_.push_back(r);};
  }
};

TEST_F(AcquisitionLookupTest, DecidesAtTheRequestTagAnIngestOrTheLogicalDeadline)
{
  rcllf_tf2::AcquisitionLookup lk(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  g_constructing = false;
  const int64_t samples_ms[] = {0, 20, 40};
  for (int64_t ms : samples_ms) {
    ingest(ms);
  }

  // T1: stamp 30 ms between the samples at 20 and 40: x = 0.030, ok at once.
  g_tag = 45 * kMs;
  lk.request("world", "sensor_link", rclcpp::Time(30 * kMs), collect());
  // T2: stamp 40 ms equals the newest sample: tf2 accepts the end point.
  lk.request("world", "sensor_link", rclcpp::Time(40 * kMs), collect());
  ASSERT_EQ(results_.size(), 2u);
  EXPECT_EQ(results_[0].name(), "ok");
  EXPECT_NEAR(results_[0].transform.transform.translation.x, 0.030, 1e-9);
  EXPECT_EQ(results_[0].decided_time, 45 * kMs);
  EXPECT_NEAR(results_[1].transform.transform.translation.x, 0.040, 1e-9);
  EXPECT_TRUE(g_wakes.empty());

  // T3: stamp 50 ms at tag 50 ms, newest 40: waits; the sample at 60 ms
  // makes it ok at the ingest tag, x = 0.050, 10 ms logical wait.
  g_tag = 50 * kMs;
  lk.request("world", "sensor_link", rclcpp::Time(50 * kMs), collect());
  EXPECT_EQ(g_wakes, (std::vector<int64_t>{105 * kMs}));  // 50 + 55
  EXPECT_EQ(lk.pending(), 1u);
  ingest(60);
  ASSERT_EQ(results_.size(), 3u);
  EXPECT_EQ(results_[2].name(), "ok");
  EXPECT_NEAR(results_[2].transform.transform.translation.x, 0.050, 1e-9);
  EXPECT_EQ(results_[2].decided_time - results_[2].request_time, 10 * kMs);
  wake(105 * kMs);  // the wake of a decided request decides nothing
  EXPECT_EQ(results_.size(), 3u);

  // T4: stamp 90 ms at tag 90 ms and no further /tf: reject at 90 + 55.
  // T7: frame never published: failure at the same deadline, not before.
  g_tag = 90 * kMs;
  lk.request("world", "sensor_link", rclcpp::Time(90 * kMs), collect());
  lk.request("world", "ghost_link", rclcpp::Time(90 * kMs), collect());
  EXPECT_EQ(results_.size(), 3u);
  wake(144 * kMs);  // before the deadline: nothing
  EXPECT_EQ(results_.size(), 3u);
  wake(145 * kMs);
  ASSERT_EQ(results_.size(), 5u);
  EXPECT_EQ(results_[3].name(), "reject");
  EXPECT_EQ(results_[3].decided_time, 145 * kMs);
  EXPECT_EQ(results_[4].name(), "failure");
  EXPECT_EQ(results_[4].decided_time, 145 * kMs);
  EXPECT_EQ(lk.pending(), 0u);
}

TEST_F(AcquisitionLookupTest, FallbackIsNamedAndTraced)
{
  rcllf_tf2::AcquisitionLookup lk(
    *buffer_, node_.get(), std::chrono::milliseconds(55),
    rcllf_tf2::OnTimeout::fallback("identity"));
  ingest(0);
  g_tag = 30 * kMs;
  lk.request("world", "sensor_link", rclcpp::Time(30 * kMs), collect());
  wake(85 * kMs);
  ASSERT_EQ(results_.size(), 1u);
  EXPECT_EQ(results_[0].name(), "fallback:identity");
  ASSERT_EQ(g_trace.size(), 2u);
  EXPECT_EQ(
    g_trace[0],
    "{\"event\":\"tf_request\",\"node\":\"/acq_test\",\"adapter\":0,\"id\":1,"
    "\"target\":\"world\",\"source\":\"sensor_link\",\"stamp\":30000000,\"time\":30000000,"
    "\"micro\":0,\"wait_bound_ns\":55000000,\"on_timeout\":\"fallback:identity\","
    "\"oldest_common\":\"unavailable\",\"latest_common\":\"unavailable\"}");
  EXPECT_NE(g_trace[1].find("\"outcome\":\"fallback:identity\""), std::string::npos);
  EXPECT_NE(g_trace[1].find("\"wait_logical_ns\":55000000"), std::string::npos);
  // One dynamic sample stamped 0: tf2's latest common time is 0, as for a
  // static link, but the link answers no positive stamp, so the interval is
  // unavailable, never the all-static [0, 0].
  EXPECT_NE(
    g_trace[1].find("\"oldest_common\":\"unavailable\",\"latest_common\":\"unavailable\","),
    std::string::npos) << g_trace[1];
}

/// `"<key>":<value>` of a flat JSON line, "<absent>" when the key is missing.
std::string field(const std::string & json, const std::string & key)
{
  const std::string k = "\"" + key + "\":";
  const auto at = json.find(k);
  if (at == std::string::npos) {
    return "<absent>";
  }
  const auto from = at + k.size();
  return json.substr(from, json.find_first_of(",}", from) - from);
}

TEST_F(AcquisitionLookupTest, TwoAdaptersOnOneNodeTraceDistinctKeys)
{
  // Each adapter numbers its requests from 1, so
  // two adapters of one node both trace id 1. Before the adapter field the
  // pairing key (node, id) of the two requests was the same.
  rcllf_tf2::AcquisitionLookup a(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  rcllf_tf2::AcquisitionLookup b(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::failure());
  g_constructing = false;
  ingest(0);
  ingest(20);
  g_tag = 30 * kMs;
  a.request("world", "sensor_link", rclcpp::Time(10 * kMs), collect());
  b.request("world", "sensor_link", rclcpp::Time(10 * kMs), collect());
  ASSERT_EQ(g_trace.size(), 4u);  // request + outcome per adapter
  std::vector<std::string> keys;
  for (const auto & line : g_trace) {
    if (field(line, "event") == "\"tf_request\"") {
      keys.push_back(
        field(line, "node") + "/adapter=" + field(line, "adapter") + "/id=" + field(line, "id"));
    }
  }
  ASSERT_EQ(keys.size(), 2u);
  EXPECT_NE(keys[0], keys[1]) << "colliding (node, adapter, id) keys: " << keys[0];
  EXPECT_EQ(keys[0], "\"/acq_test\"/adapter=0/id=1");
  EXPECT_EQ(keys[1], "\"/acq_test\"/adapter=1/id=1");
}

TEST_F(AcquisitionLookupTest, AdapterOrdinalRestartsOnANewNode)
{
  // The fixture made a new node: its first adapter is 0 again, also when the
  // node reuses the address of the previous test's node.
  rcllf_tf2::AcquisitionLookup a(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  rcllf_tf2::AcquisitionLookup b(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  EXPECT_EQ(a.adapter_index(), 0u);
  EXPECT_EQ(b.adapter_index(), 1u);
  auto other = std::make_shared<rclcpp::Node>("acq_test_other");
  rcllf_tf2::AcquisitionLookup c(
    *buffer_, other.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  EXPECT_EQ(c.adapter_index(), 0u);
  rcllf_tf2::AcquisitionLookup d(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  EXPECT_EQ(d.adapter_index(), 2u);
}

TEST_F(AcquisitionLookupTest, AvailableIntervalIsTheLatestPerLinkOldestStamp)
{
  // Hand-derived corpus, a Buffer that keeps
  // 100 ms. tf2 0.36 (Jazzy) prunes a link to its samples newer than
  // newest - 100 ms plus the newest sample at or before that time, which an
  // interpolation at newest - 100 ms needs (allFramesAsYAML of this corpus
  // prints base oldest_transform 0.080000, evidence impl-91r3/tf2_prune_probe):
  //   world -> base     dynamic, 0, 20, ..., 200 ms: kept 80..200, oldest 80 ms
  //   base -> sensor    dynamic, 130 ms + 7 ns, then every 20 ms to 190 ms + 7 ns
  //   sensor -> optical static
  //   world -> cam      dynamic, 60, 80, ..., 160 ms: oldest 60 ms
  //   world -> late     dynamic, 300, 320 ms
  // Expected [oldest, latest] in ns (latest = tf2's latest common time, the
  // minimum newest stamp along the path; oldest = maximum oldest stamp of
  // the dynamic links on the path):
  //   world <- optical : [130 ms + 7, 190 ms + 7]   (static link skipped)
  //   base  <- sensor  : [130 ms + 7, 190 ms + 7]
  //   cam   <- sensor  : [130 ms + 7, 160 ms]       (three dynamic links)
  //   world <- base    : [80 ms, 200 ms]            (pruned below 80 ms)
  //   sensor<- late    : [300 ms, 190 ms + 7]       (no overlap: oldest > latest)
  //   sensor<- optical : [0, 0]                     (static only)
  //   world <- ghost   : [-1, -1]                   (unknown frame)
  auto buffer = std::make_unique<tf2_ros::Buffer>(
    node_->get_clock(), tf2::durationFromSec(0.1));
  rcllf_tf2::AcquisitionLookup lk(
    *buffer, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  auto put = [&buffer](const char * parent, const char * child, int64_t ns, bool is_static) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = rclcpp::Time(ns);
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.rotation.w = 1.0;
      buffer->setTransform(t, "test", is_static);
    };
  for (int64_t ms = 0; ms <= 200; ms += 20) {
    put("world", "base", ms * kMs, false);
  }
  for (int64_t ms = 130; ms <= 190; ms += 20) {
    put("base", "sensor", ms * kMs + 7, false);
  }
  put("sensor", "optical", 0, true);
  for (int64_t ms = 60; ms <= 160; ms += 20) {
    put("world", "cam", ms * kMs, false);
  }
  put("world", "late", 300 * kMs, false);
  put("world", "late", 320 * kMs, false);

  struct Case
  {
    const char * target;
    const char * source;
    int64_t oldest;
    int64_t latest;
  };
  const Case cases[] = {
    {"world", "optical", 130 * kMs + 7, 190 * kMs + 7},
    {"base", "sensor", 130 * kMs + 7, 190 * kMs + 7},
    {"cam", "sensor", 130 * kMs + 7, 160 * kMs},
    {"world", "base", 80 * kMs, 200 * kMs},
    {"sensor", "late", 300 * kMs, 190 * kMs + 7},
    {"sensor", "optical", 0, 0},
    {"world", "ghost", -1, -1},
  };
  for (const auto & c : cases) {
    const auto iv = lk.available(c.target, c.source);
    EXPECT_EQ(iv.oldest, c.oldest) << c.target << " <- " << c.source;
    EXPECT_EQ(iv.latest, c.latest) << c.target << " <- " << c.source;
  }
  // tf2's own report of each dynamic link's oldest stamp (text, in s with
  // 6 decimals) agrees with the bisection.
  tf2::BufferCore & core = *buffer;
  const std::string yaml = core.allFramesAsYAML();
  for (const auto & [frame, want] : std::vector<std::pair<std::string, std::string>>{
      {"base", "0.080000"}, {"sensor", "0.130000"}, {"cam", "0.060000"}, {"late", "0.300000"}})
  {
    const auto at = yaml.find(frame + ": \n");
    ASSERT_NE(at, std::string::npos) << frame;
    const auto key = yaml.find("oldest_transform: ", at);
    ASSERT_NE(key, std::string::npos) << frame;
    EXPECT_EQ(yaml.substr(key + 18, want.size()), want) << frame;
  }
  // The oldest end is transformable and 1 ns before it is not (exactness).
  EXPECT_TRUE(
    core.canTransform(
      "world", "optical", tf2::TimePoint(std::chrono::nanoseconds(130 * kMs + 7)), nullptr));
  EXPECT_FALSE(
    core.canTransform(
      "world", "optical", tf2::TimePoint(std::chrono::nanoseconds(130 * kMs + 6)), nullptr));
}

TEST_F(AcquisitionLookupTest, AvailableIntervalIsUnavailableForAReparentedOrZeroStampedLink)
{
  // tf2 allows reparenting: child
  // under A at 10 ms, under B at 20 ms, under A again at 100 ms is answerable
  // from A at 10..20 ms and from 100 ms on. The bisection over canTransform
  // assumed one contiguous range and reported [100 ms, 100 ms] as exact.
  // A dynamic link sampled only at stamp 0 answers no positive stamp; it
  // reported [0, 0], the all-static value.
  auto buffer = std::make_unique<tf2_ros::Buffer>(
    node_->get_clock(), tf2::durationFromSec(10.0));
  rcllf_tf2::AcquisitionLookup lk(
    *buffer, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  auto put = [&buffer](const char * parent, const char * child, int64_t ns, bool is_static) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = rclcpp::Time(ns);
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.rotation.w = 1.0;
      buffer->setTransform(t, "test", is_static);
    };
  put("A", "child", 10 * kMs, false);
  put("B", "child", 20 * kMs, false);
  put("A", "child", 100 * kMs, false);
  put("world", "zero", 0, false);
  put("world", "fixed", 0, true);
  put("world", "steady", 10 * kMs, false);
  put("world", "steady", 100 * kMs, false);
  tf2::BufferCore & core = *buffer;
  // The premise, from tf2 itself: answerable at 15 ms, not at 50 ms, again at 100 ms.
  auto can = [&core](const char * target, const char * source, int64_t ns) {
      return core.canTransform(
        target, source, tf2::TimePoint(std::chrono::nanoseconds(ns)), nullptr);
    };
  EXPECT_TRUE(can("A", "child", 15 * kMs));
  EXPECT_FALSE(can("A", "child", 50 * kMs));
  EXPECT_TRUE(can("A", "child", 100 * kMs));
  EXPECT_FALSE(can("world", "zero", 1));
  EXPECT_TRUE(can("world", "fixed", 1));

  const int64_t u = rcllf_tf2::AcquisitionLookup::kUnavailable;
  struct Case
  {
    const char * target;
    const char * source;
    int64_t oldest;
    int64_t latest;
  };
  const Case cases[] = {
    {"A", "child", u, u},                       // reparented: no exact range
    {"world", "zero", u, u},                    // dynamic, stamp 0 only
    {"world", "fixed", 0, 0},                   // static: any stamp
    {"world", "steady", 10 * kMs, 100 * kMs},   // one parent throughout: exact
  };
  for (const auto & c : cases) {
    const auto iv = lk.available(c.target, c.source);
    EXPECT_EQ(iv.oldest, c.oldest) << c.target << " <- " << c.source;
    EXPECT_EQ(iv.latest, c.latest) << c.target << " <- " << c.source;
  }
}

TEST_F(AcquisitionLookupTest, ZeroStampFailsAtTheRequestInsteadOfAnsweringLatest)
{
  // tf2 reads stamp 0 as "latest"; before the guard this request was ok with
  // the newest sample (x = 0.040), a value that depends on ingest timing.
  rcllf_tf2::AcquisitionLookup lk(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  const int64_t samples_ms[] = {0, 20, 40};
  for (int64_t ms : samples_ms) {
    ingest(ms);
  }
  lk.request("world", "sensor_link", rclcpp::Time(0, 0, RCL_ROS_TIME), collect());
  ASSERT_EQ(results_.size(), 1u);
  EXPECT_EQ(results_[0].name(), "failure");
  EXPECT_EQ(results_[0].decided_time, 40 * kMs);
  EXPECT_NE(results_[0].error.find("tf2 reads 0 as latest"), std::string::npos);
  EXPECT_TRUE(g_wakes.empty());
  EXPECT_EQ(lk.pending(), 0u);
}

TEST_F(AcquisitionLookupTest, RequestOutsideAnLfReactionThrows)
{
  // Before the guard the request read the node clock and decided at once on
  // the calling thread (review finding F3).
  rcllf_tf2::AcquisitionLookup lk(
    *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());
  ingest(0);
  g_tag = -1;  // the fake's ros2_lf_tf2_current: not an LF worker
  EXPECT_THROW(
    lk.request("world", "sensor_link", rclcpp::Time(0, 1000, RCL_ROS_TIME), collect()),
    std::logic_error);
  EXPECT_TRUE(results_.empty());
  EXPECT_EQ(lk.pending(), 0u);
}

TEST_F(AcquisitionLookupTest, RefusesToRegisterOutsideTheNodeConstructor)
{
  g_constructing = false;
  EXPECT_THROW(
    rcllf_tf2::AcquisitionLookup(
      *buffer_, node_.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject()),
    std::runtime_error);
}

}  // namespace

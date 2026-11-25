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

// BNA policy in stock mode (no rcllf runtime).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/pass_through.h"
#include "message_filters/synchronizer.h"
#include "rcllf_message_filters/bna_policy.hpp"

namespace
{

using Msg = geometry_msgs::msg::PointStamped;
using Policy = rcllf_message_filters::sync_policies::BoundedNearestToAnchor<Msg, Msg>;
constexpr int64_t kMs = 1000000;

Msg::ConstSharedPtr msg(int64_t stamp_ns)
{
  auto m = std::make_shared<Msg>();
  m->header.stamp = rclcpp::Time(stamp_ns);
  return m;
}

struct Counter
{
  int * n;
  void cb(const Msg::ConstSharedPtr &, const Msg::ConstSharedPtr &) {++*n;}
};

struct Lines
{
  std::mutex mu;
  std::vector<std::string> v;
  void add(const std::string & s)
  {
    std::lock_guard<std::mutex> lk(mu);
    v.push_back(s);
  }
  std::vector<std::string> outputs()
  {
    std::lock_guard<std::mutex> lk(mu);
    std::vector<std::string> out;
    for (const auto & s : v) {
      if (s.rfind("{\"ev\":\"decision\"", 0) == 0 || s.rfind("{\"ev\":\"sample\"", 0) == 0) {
        out.push_back(s);
      }
    }
    return out;
  }
};

}  // namespace

// Fix 2: the timer is held back past an anchor's deadline. A candidate that
// would have matched arrives late, then samples that would complete the
// anchor. The anchor still ends REJECTED(timeout) at its deadline tag and no
// callback runs.
TEST(BnaPolicyStock, DelayedTimerNeverLetsAMatchPassItsDeadline)
{
  std::atomic<int64_t> now{0};
  Lines lines;
  int callbacks = 0;
  {
    Policy policy(10 * kMs, 0, 0, 8, 8, 1000 * kMs, 50 * kMs);
    policy.setClockForTest([&now] {return now.load();});
    policy.holdTimerForTest();
    policy.setTraceSinkForTest([&lines](const std::string & s) {lines.add(s);});
    message_filters::PassThrough<Msg> in0;
    message_filters::PassThrough<Msg> in1;
    message_filters::Synchronizer<Policy> sync(static_cast<const Policy &>(policy), in0, in1);
    Counter counter{&callbacks};
    sync.registerCallback(&Counter::cb, &counter);
    now = 10 * kMs;
    in0.add(msg(100 * kMs));            // deadline 60 ms
    now = 100 * kMs;                    // timer held: nothing ran at 60 ms
    in1.add(msg(100 * kMs));            // the perfect partner, too late
    in0.add(msg(300 * kMs));
    in1.add(msg(300 * kMs));            // both watermarks now pass 110 ms
    const auto out = lines.outputs();
    ASSERT_FALSE(out.empty());
    EXPECT_EQ(
      out[0],
      "{\"ev\":\"decision\",\"sync\":0,\"anchor\":\"0:a:1\",\"stamp\":100000000,"
      "\"outcome\":\"REJECTED\",\"reason\":\"timeout\",\"partner\":null,"
      "\"partner_stamp\":null,\"tag\":[60000000,0]}");
    EXPECT_EQ(callbacks, 0);
  }
  // Hand derivation: anchor 0:a:2 (stamp 300 ms) lifts W[0]
  // to 300 ms, so 0:b:1 (100 + 10 < 300) is SUPERSEDED at once. Neither 0:a:2
  // nor 0:b:2 completes (W must pass 310 ms), and destruction closes with
  // reason cancelled at the last clock reading, 100 ms.
  EXPECT_EQ(callbacks, 0);
  const auto out = lines.outputs();
  ASSERT_EQ(out.size(), 4u);
  EXPECT_EQ(
    out[1],
    "{\"ev\":\"sample\",\"sync\":0,\"input\":1,\"key\":\"0:b:1\",\"stamp\":100000000,"
    "\"outcome\":\"SUPERSEDED\",\"reason\":\"\",\"tag\":[100000000,0]}");
  EXPECT_EQ(
    out[2],
    "{\"ev\":\"decision\",\"sync\":0,\"anchor\":\"0:a:2\",\"stamp\":300000000,"
    "\"outcome\":\"REJECTED\",\"reason\":\"cancelled\",\"partner\":null,"
    "\"partner_stamp\":null,\"tag\":[100000000,0]}");
  EXPECT_EQ(
    out[3],
    "{\"ev\":\"sample\",\"sync\":0,\"input\":1,\"key\":\"0:b:2\",\"stamp\":300000000,"
    "\"outcome\":\"FLUSHED\",\"reason\":\"\",\"tag\":[100000000,0]}");
}

// Fix 4: a lone candidate with nothing else happening ends EXPIRED by the
// timer thread at its retention deadline, before shutdown.
TEST(BnaPolicyStock, LoneCandidateExpiresBeforeShutdown)
{
  Lines lines;
  {
    Policy policy(10 * kMs, 0, 0, 8, 8, 30 * kMs, 1000 * kMs);
    policy.setTraceSinkForTest([&lines](const std::string & s) {lines.add(s);});
    message_filters::PassThrough<Msg> in0;
    message_filters::PassThrough<Msg> in1;
    message_filters::Synchronizer<Policy> sync(static_cast<const Policy &>(policy), in0, in1);
    in1.add(msg(5 * kMs));
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (lines.outputs().empty() && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  const auto out = lines.outputs();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(out[0].find("\"key\":\"0:b:1\",\"stamp\":5000000,\"outcome\":\"EXPIRED\""),
    std::string::npos);
  // The EXPIRED tag is the admission tag plus retention.
  std::string admit;
  for (const auto & s : lines.v) {
    if (s.rfind("{\"ev\":\"admit\"", 0) == 0) {
      admit = s;
    }
  }
  const auto tag_of = [](const std::string & s) {
      const auto i = s.find("\"tag\":[") + 7;
      return std::stoll(s.substr(i, s.find(',', i) - i));
    };
  ASSERT_FALSE(admit.empty());
  EXPECT_EQ(tag_of(out[0]), tag_of(admit) + 30 * kMs);
  // Close came after the expiry, so nothing was left to flush.
  EXPECT_EQ(lines.v.back().rfind("{\"ev\":\"close\",\"sync\":0,\"reason\":\"cancelled\"", 0), 0u);
}

// The policy's own parameter check.
TEST(BnaPolicyStock, RefusesBadParameters)
{
  EXPECT_THROW(Policy(-1, 0, 0, 1, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW(Policy(0, 0, 0, 0, 1, 1, 1), std::invalid_argument);
}

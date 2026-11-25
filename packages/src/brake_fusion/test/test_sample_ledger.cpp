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

// The ledger accounts for every sample an unmodified
// message_filters ApproximateTime policy receives, names the evicted sample
// and the reason, and does not change the pairs, as an earlier Python observer did.

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "brake_fusion/sample_ledger.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/msg/range.hpp"

using brake_fusion::SampleLedger;
using Range = sensor_msgs::msg::Range;
using Policy = message_filters::sync_policies::ApproximateTime<Range, Range>;
using Sync = message_filters::Synchronizer<Policy>;

namespace
{

struct Rig
{
  explicit Rig(uint32_t q, bool with_ledger = true)
  : ledger(q), use_ledger(with_ledger), sync(Policy(q))
  {
    // Member function through std::bind, as the node registers it (Jazzy's
    // Synchronizer::registerCallback takes no two-argument lambda).
    sync.registerCallback(
      std::bind(&Rig::on_pair, this, std::placeholders::_1, std::placeholders::_2));
  }

  void on_pair(const Range::ConstSharedPtr & r, const Range::ConstSharedPtr & c)
  {
    pairs.emplace_back(
      rclcpp::Time(r->header.stamp).nanoseconds(),
      rclcpp::Time(c->header.stamp).nanoseconds());
    if (use_ledger) {
      ledger.paired(r.get(), c.get());
    }
  }

  // One arrival as the node sees it: the arrival callback settles the
  // previous add's discards, records this sample, then Synchronizer::add.
  // The message is released when this returns, as an executor releases it
  // after the Subscriber callbacks.
  int64_t add(int input, int64_t stamp_ns)
  {
    auto msg = std::make_shared<Range>();
    msg->header.stamp = rclcpp::Time(stamp_ns);
    Range::ConstSharedPtr cmsg = msg;
    const int64_t n = next[input]++;
    int64_t dup = -1;
    if (use_ledger) {
      finish();
      dup = ledger.arrive(input, cmsg, n, stamp_ns);
    }
    if (input == 0) {
      sync.add<0>(cmsg);
    } else {
      sync.add<1>(cmsg);
    }
    return dup;
  }

  // The node's shutdown settle.
  void finish()
  {
    for (const auto & f : ledger.settle()) {
      fates.emplace_back(f.input, f.n, std::string(f.reason));
    }
  }

  SampleLedger ledger;
  bool use_ledger;
  Sync sync;
  std::vector<std::pair<int64_t, int64_t>> pairs;
  std::vector<std::tuple<int, int64_t, std::string>> fates;
  int64_t next[2]{0, 0};
};

constexpr int64_t kMs = 1'000'000;

// The t2 rcllf pattern: two radars through the tracker give a track every
// 10 ms, the camera stamps every 30 ms and arrives 5 ms after its stamp.
void nominal(Rig & rig, int periods)
{
  for (int k = 0; k < periods * 3; ++k) {
    rig.add(0, k * 10 * kMs);
    if (k % 3 == 0 && k > 0) {
      rig.add(1, (k - 3) * 10 * kMs);
    }
  }
}

}  // namespace

TEST(SampleLedger, EverySampleHasExactlyOneFate)
{
  Rig rig(9);
  nominal(rig, 20);
  rig.finish();
  for (int i = 0; i < 2; ++i) {
    const auto & L = rig.ledger;
    EXPECT_EQ(L.arrivals(i), L.paired_count(i) + L.superseded(i) + L.overflowed(i) + L.held(i))
      << "input " << i;
  }
  EXPECT_EQ(rig.ledger.overflowed(0), 0);
  EXPECT_EQ(rig.ledger.overflowed(1), 0);
  // Every camera sample but the last pairs, and two of three tracks are
  // superseded by the one nearest the camera stamp.
  EXPECT_EQ(rig.ledger.paired_count(1), static_cast<int64_t>(rig.pairs.size()));
  EXPECT_GE(rig.ledger.superseded(0), 2 * static_cast<int64_t>(rig.pairs.size()) - 2);
  EXPECT_LE(rig.ledger.peak(0), 9);
}

TEST(SampleLedger, DoesNotChangeThePairs)
{
  Rig with(9, true);
  Rig without(9, false);
  nominal(with, 20);
  nominal(without, 20);
  ASSERT_FALSE(without.pairs.empty());
  EXPECT_EQ(with.pairs, without.pairs);
}

TEST(SampleLedger, NamesTheEvictedSampleOnOverflow)
{
  // A camera stall: queue 3, six tracks and no camera sample. The policy
  // evicts tracks 0, 1, 2 as tracks 3, 4, 5 arrive.
  Rig rig(3);
  for (int k = 0; k < 6; ++k) {
    rig.add(0, k * 10 * kMs);
  }
  rig.finish();
  std::vector<std::tuple<int, int64_t, std::string>> want{
    {0, 0, "queue_overflow"}, {0, 1, "queue_overflow"}, {0, 2, "queue_overflow"}};
  EXPECT_EQ(rig.fates, want) << "evictions must be named, sample by sample";
  EXPECT_EQ(rig.ledger.overflowed(0), 3);
  EXPECT_EQ(rig.ledger.peak(0), 4);
  EXPECT_EQ(rig.ledger.held_max(0), 3);
}

TEST(SampleLedger, ReportsDuplicateStamps)
{
  Rig rig(9);
  EXPECT_EQ(rig.add(0, 20 * kMs), -1);
  EXPECT_EQ(rig.add(0, 20 * kMs), 0) << "second track at the same stamp must name the first";
  EXPECT_EQ(rig.ledger.duplicates(0), 1);
}

TEST(SampleLedger, SupersededDuringTheAddIsNotAnOverflow)
{
  // Queue 2, tracks 0 and 10 ms, camera 10 ms,
  // then track 20 ms. The policy publishes the 10/10 pair inside the last add
  // and discards track 0 as the older candidate member, it evicts nothing.
  // The reference run at queue 1000 must give the same pairs and fates.
  auto run = [](uint32_t q) {
      Rig rig(q);
      rig.add(0, 0);
      rig.add(0, 10 * kMs);
      rig.add(1, 10 * kMs);
      rig.add(0, 20 * kMs);
      rig.finish();
      return std::make_pair(rig.pairs, rig.fates);
    };
  const auto small = run(2);
  const auto ref = run(1000);
  EXPECT_EQ(small.first, ref.first) << "queue 2 must pair like queue 1000 here";
  EXPECT_EQ(small.second, ref.second)
    << "a sample the policy discarded without evicting it must not be named queue_overflow";
}

TEST(SampleLedger, RefusesQueueSizeOne)
{
  // At queue size 1 the push-time flag is wrong: camera 15 ms, radar 10, 15,
  // camera 25, radar 30, camera 30, radar 45, 55, camera 45, 50 names radar 0
  // and camera 1 queue_overflow while the policy, with its eviction hooked,
  // evicts neither (the ledger-oracle script, queue size 1, ledger of 3aac5f56).
  EXPECT_THROW(SampleLedger(1), std::invalid_argument)
    << "queue size 1 would name superseded samples queue_overflow";
  EXPECT_THROW(SampleLedger(0), std::invalid_argument);
  EXPECT_NO_THROW(SampleLedger(2));
}

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

// Unit checks of the BNA engine API. The rule-by-rule agreement with the
// Python oracle is tools/lf_eval/tests/bna/test_replay.py.

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "rcllf_message_filters/bna_engine.hpp"

namespace bna = rcllf_message_filters::bna;

namespace
{

constexpr int64_t kMs = 1000000;

bna::Params params()
{
  bna::Params p;
  p.epoch = 1;
  p.delta = 10 * kMs;
  p.capacity[0] = 8;
  p.capacity[1] = 8;
  p.retention = 1000 * kMs;
  p.timeout = 47 * kMs;
  return p;
}

bna::Key key(const std::string & gid, uint64_t ordinal)
{
  return bna::Key{1, gid, ordinal};
}

bna::Tag at(int64_t ms) {return bna::Tag{ms * kMs, 0};}

struct Fixture
{
  bna::Engine<int> engine{params()};
  std::vector<std::string> records;
  std::vector<std::pair<int, int>> matches;
  std::vector<bna::Tag> deadlines;

  Fixture()
  {
    engine.on_record([this](const bna::Record & r) {records.push_back(bna::to_json(r));});
    engine.on_match([this](int & a, int & b) {matches.emplace_back(a, b);});
    engine.on_deadline([this](const bna::Tag & t) {deadlines.push_back(t);});
  }
};

}  // namespace

TEST(BnaEngine, MatchesNearestAndHandsBackPayloads)
{
  Fixture f;
  f.engine.admit(1, 95 * kMs, key("b", 1), at(100), 11);
  f.engine.admit(1, 108 * kMs, key("b", 2), at(101), 12);
  f.engine.admit(0, 100 * kMs, key("a", 1), at(102), 1);
  EXPECT_TRUE(f.matches.empty());
  f.engine.admit(0, 120 * kMs, key("a", 2), at(103), 2);
  f.engine.admit(1, 121 * kMs, key("b", 3), at(104), 13);
  ASSERT_EQ(f.matches.size(), 1u);
  EXPECT_EQ(f.matches[0], std::make_pair(1, 11));
  ASSERT_GE(f.records.size(), 2u);
  EXPECT_EQ(
    f.records[0],
    "{\"ev\":\"decision\",\"sync\":0,\"anchor\":\"1:a:1\",\"stamp\":100000000,"
    "\"outcome\":\"MATCHED\",\"reason\":\"\",\"partner\":\"1:b:1\","
    "\"partner_stamp\":95000000,\"tag\":[104000000,0]}");
  EXPECT_EQ(
    f.records[1],
    "{\"ev\":\"sample\",\"sync\":0,\"input\":1,\"key\":\"1:b:1\",\"stamp\":95000000,"
    "\"outcome\":\"SELECTED\",\"reason\":\"\",\"tag\":[104000000,0]}");
}

TEST(BnaEngine, ReportsEveryDeadlineAndTheEarliest)
{
  Fixture f;
  f.engine.admit(1, 0, key("b", 1), at(5));
  f.engine.admit(0, 0, key("a", 1), at(6));
  ASSERT_EQ(f.deadlines.size(), 2u);
  EXPECT_EQ(f.deadlines[0], at(1005));
  EXPECT_EQ(f.deadlines[1], at(53));
  ASSERT_TRUE(f.engine.next_deadline().has_value());
  EXPECT_EQ(*f.engine.next_deadline(), at(53));
}

TEST(BnaEngine, WakeAtTheDeadlineRejectsWithTimeout)
{
  Fixture f;
  f.engine.admit(0, 0, key("a", 1), at(6));
  f.engine.wake(at(52));
  EXPECT_TRUE(f.records.empty());
  f.engine.wake(at(53));
  ASSERT_EQ(f.records.size(), 1u);
  EXPECT_NE(f.records[0].find("\"reason\":\"timeout\",\"partner\":null"), std::string::npos);
  EXPECT_NE(f.records[0].find("\"tag\":[53000000,0]"), std::string::npos);
  EXPECT_FALSE(f.engine.next_deadline().has_value());
}

TEST(BnaEngine, CloseFlushesAndRejectsLaterAdmissions)
{
  Fixture f;
  f.engine.admit(1, 0, key("b", 1), at(5));
  f.engine.close("shutdown", at(6));
  f.engine.admit(0, 0, key("a", 1), at(7));
  ASSERT_EQ(f.records.size(), 2u);
  EXPECT_NE(f.records[0].find("\"outcome\":\"FLUSHED\""), std::string::npos);
  EXPECT_NE(f.records[1].find("\"reason\":\"closed\""), std::string::npos);
  EXPECT_TRUE(f.engine.closed());
}

TEST(BnaEngine, RefusesBackwardTagsAndBadParameters)
{
  Fixture f;
  f.engine.admit(0, 0, key("a", 1), at(5));
  EXPECT_THROW(f.engine.wake(at(4)), std::invalid_argument);
  bna::Params p = params();
  p.timeout = 0;
  EXPECT_THROW(bna::Engine<int>{p}, std::invalid_argument);
  EXPECT_THROW(f.engine.close("bogus", at(6)), std::invalid_argument);
}

TEST(BnaEngine, ParsesKeys)
{
  bna::Key k;
  ASSERT_TRUE(bna::Key::parse("3:01.0f.aa:42", &k));
  EXPECT_EQ(k.epoch, 3);
  EXPECT_EQ(k.gid, "01.0f.aa");
  EXPECT_EQ(k.ordinal, 42u);
  EXPECT_EQ(k.text(), "3:01.0f.aa:42");
  EXPECT_FALSE(bna::Key::parse("3:42", &k));
  EXPECT_FALSE(bna::Key::parse("x:a:1", &k));
}

namespace
{

// A relay-free fixture. Anchors every 100 ms, candidates every
// 50 ms, links after 3 ms, delta 20 ms, candidate lateness 30 ms. A lost
// message is reported at (stamp + 10 ms, 2) as the decentralized runs did.
bna::Params p20_params()
{
  bna::Params p;
  p.epoch = 1;
  p.delta = 20 * kMs;
  p.lateness[1] = 30 * kMs;
  p.capacity[0] = 4;
  p.capacity[1] = 4;
  p.retention = 350 * kMs;
  p.timeout = 300 * kMs;
  return p;
}

struct StampLagRun
{
  bna::Engine<int> engine{p20_params()};
  std::vector<bna::Record> decisions;

  StampLagRun()
  {
    engine.on_record([this](const bna::Record & r) {if (r.decision) {decisions.push_back(r);}});
  }

  // Seq s of input i, stamped s * period, delivered at its intended tag or
  // reported lost at (stamp + 10 ms, 2) with its stamp.
  void step(int i, int64_t seq, bool lost)
  {
    const int64_t stamp = seq * (i == 0 ? 100 : 50) * kMs;
    const bna::Key k = key(i == 0 ? "a" : "b", static_cast<uint64_t>(seq + 1));
    if (lost) {
      engine.missing(i, k, bna::Tag{stamp + 10 * kMs, 2}, stamp);
    } else {
      engine.admit(i, stamp, k, bna::Tag{stamp + 3 * kMs, 0}, 0);
    }
  }

  // Both streams in tag order up to `until_ms`, losing the candidate seqs in `lost_b`.
  void run(int64_t until_ms, int64_t lost_lo, int64_t lost_hi)
  {
    for (int64_t ms = 0; ms <= until_ms; ms += 50) {
      if (ms % 100 == 0) {
        step(0, ms / 100, false);
      }
      const int64_t b = ms / 50;
      if (b >= lost_lo && b <= lost_hi) {
        // The loss report comes 7 ms after the intended tag, after any admit at it.
        step(1, b, true);
      } else {
        step(1, b, false);
      }
    }
  }

  std::string decision_of(uint64_t anchor_ordinal) const
  {
    for (const auto & r : decisions) {
      if (r.key == "1:a:" + std::to_string(anchor_ordinal)) {
        return r.outcome + " " + r.reason + " " + r.partner + " " + bna::tag_json(r.tag);
      }
    }
    return "none";
  }
};

}  // namespace

TEST(BnaEngine, LostReportWithStampReleasesThePendingAnchorAtTheReportTag)
{
  // b20 lost: a9 (ordinal 10) pairs with b18 when the report arrives at
  // 1010 ms, not when b21 is admitted at 1053 ms (the old shifted tag).
  StampLagRun r;
  r.run(1050, 20, 20);
  EXPECT_EQ(r.decision_of(10), "MATCHED  1:b:19 [1010000000,2]");
}

TEST(BnaEngine, LostReportsKeepAnAnchorWithAPartnerFromTimingOut)
{
  // b20..b27 lost: a9 still pairs with b18 at 1010 ms instead of the old
  // REJECTED timeout at 1203 ms, and a10 ends missing at the b22 report.
  StampLagRun r;
  r.run(1400, 20, 27);
  EXPECT_EQ(r.decision_of(10), "MATCHED  1:b:19 [1010000000,2]");
  EXPECT_EQ(r.decision_of(11), "REJECTED missing  [1110000000,2]");
  EXPECT_EQ(r.decision_of(12), "REJECTED missing  [1210000000,2]");
  EXPECT_EQ(r.decision_of(13), "REJECTED missing  [1310000000,2]");
}

namespace
{

// The relay counterexample as run end to end. Anchors every
// 100 ms admitted at stamp + 3 ms. A relay candidate every 10 ms stamps 20 ms
// before it publishes, admitted at stamp + 23 ms. Delta 15 ms, lateness 0.
// b99 (stamp 990, published 1010) is lost and reported at (1016 ms, 2) with
// the stamp bound `report_stamp`.
std::string relay_lag_run(int64_t report_stamp)
{
  bna::Params p;
  p.epoch = 1;
  p.delta = 15 * kMs;
  p.capacity[0] = 4;
  p.capacity[1] = 16;
  p.retention = 350 * kMs;
  p.timeout = 300 * kMs;
  bna::Engine<int> engine{p};
  std::vector<bna::Record> decisions;
  engine.on_record([&](const bna::Record & r) {if (r.decision) {decisions.push_back(r);}});
  bool reported = false;
  for (int64_t ms = 0; ms <= 1200; ++ms) {
    if (!reported && ms == 1016) {
      engine.missing(1, key("b", 100), bna::Tag{1016 * kMs, 2}, report_stamp);
      reported = true;
    }
    if (ms >= 23 && (ms - 23) % 10 == 0 && (ms - 23) / 10 != 99) {
      const int64_t j = (ms - 23) / 10;
      engine.admit(1, j * 10 * kMs, key("b", static_cast<uint64_t>(j + 1)), {ms * kMs, 0}, 0);
    }
    if (ms % 100 == 3) {
      const int64_t k = ms / 100;
      engine.admit(0, k * 100 * kMs, key("a", static_cast<uint64_t>(k + 1)), {ms * kMs, 0}, 0);
    }
  }
  for (const auto & r : decisions) {
    if (r.key == "1:a:11") {
      return r.outcome + " " + r.reason + " " + r.partner + " " + bna::tag_json(r.tag);
    }
  }
  return "none";
}

}  // namespace

TEST(BnaEngine, LostReportBoundBelowThePublishTagKeepsTheInWindowPartner)
{
  // The binding passes publish tag minus the declared lag (990 ms): a1000
  // pairs with b100 (stamp 1000) as without the loss.
  EXPECT_EQ(relay_lag_run(990 * kMs), "MATCHED  1:b:101 [1103000000,0]");
}

TEST(BnaEngine, LostReportAtThePublishTagRejectsLaterInWindowSamples)
{
  // The first draft binding passed the publish tag (1010 ms) as the stamp. The
  // watermark then sits above b100 and b101, both are rejected late and a1000
  // ends missing. This is the wrong answer the declared lag removes.
  EXPECT_EQ(relay_lag_run(1010 * kMs), "REJECTED missing  [1103000000,0]");
}

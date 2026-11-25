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

/// @file test_correspondence_state.cpp
/// @brief Correspondence state machine (T2): D5 rows, D6 outcomes,
///        bounds, wake discipline and the D7 reconcile invariants.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "correspondence.hpp"
#include "rcutils/logging.h"

using ros2_lf_binding::corr::Gid;
using ros2_lf_binding::corr::kNoCutoff;
using ros2_lf_binding::corr::Origin;
using ros2_lf_binding::corr::Outcome;
using ros2_lf_binding::corr::Sample;
using ros2_lf_binding::corr::SampleOutcome;
using ros2_lf_binding::corr::StallWatch;
using ros2_lf_binding::corr::Table;
using ros2_lf_binding::corr::Token;
using std::chrono::milliseconds;

namespace
{

Gid gid(uint8_t b)
{
  Gid g{};
  g[0] = b;
  return g;
}

Sample sample(int payload)
{
  return Sample{std::make_shared<int>(payload), nullptr};
}

int payload_of(const Sample & s) {return *std::static_pointer_cast<int>(s.message);}

struct Fixture : ::testing::Test
{
  Table t{42};
  const Gid g = gid(1);
  uint64_t next = 0;
  void SetUp() override
  {
    t.add_edge(0, 7, 1000000);
    t.register_gid(g);
  }
  uint64_t publish(std::vector<uint32_t> edges = {0}, Origin o = Origin::ADMITTED, bool ok = true)
  {
    ++next;
    t.predict(g, next, o, edges);
    t.commit(g, next, ok);
    return next;
  }
  std::vector<Token> move_all(uint32_t edge = 0, int64_t now = 0)
  {
    t.arrive(edge, t.in_flight(edge), now);
    std::vector<Token> out;
    while (auto tok = t.pop_released(edge, now)) {
      out.push_back(*tok);
      t.next_wake(edge, now);
    }
    return out;
  }
};

TEST_F(Fixture, ZeroOneAndManyPublishesCreateThatManyTokensInOrder)
{
  EXPECT_EQ(t.in_flight(0), 0u);
  publish();
  EXPECT_EQ(t.in_flight(0), 1u);
  move_all();
  for (int i = 0; i < 3; ++i) {
    publish();
  }
  auto toks = move_all();
  ASSERT_EQ(toks.size(), 3u);
  EXPECT_EQ(toks[0].ordinal, 2u);
  EXPECT_EQ(toks[1].ordinal, 3u);
  EXPECT_EQ(toks[2].ordinal, 4u);
  EXPECT_EQ(toks[0].instance, 7u);
  EXPECT_EQ(toks[0].epoch, 42u);
}

TEST_F(Fixture, IdenticalPayloadsStayDistinctByOrdinal)
{
  publish();
  publish();
  EXPECT_EQ(t.on_sample(0, g, 1, sample(5)), SampleOutcome::RETAINED);
  EXPECT_EQ(t.on_sample(0, g, 2, sample(5)), SampleOutcome::RETAINED);
  auto toks = move_all();
  ASSERT_EQ(toks.size(), 2u);
  Sample a, b;
  EXPECT_EQ(t.join(toks[0], milliseconds(0), &a), Outcome::MATCHED);
  EXPECT_EQ(t.join(toks[1], milliseconds(0), &b), Outcome::MATCHED);
  EXPECT_NE(a.message.get(), b.message.get());
}

TEST_F(Fixture, EarlySampleBeforeCommitIsRetainedAndMatchedSameObject)
{
  t.predict(g, 1, Origin::ADMITTED, {0});
  Sample s = sample(9);
  void * addr = s.message.get();
  EXPECT_EQ(t.on_sample(0, g, 1, s), SampleOutcome::RETAINED);  // write still in progress
  t.commit(g, 1, true);
  next = 1;
  auto toks = move_all();
  Sample out;
  ASSERT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(out.message.get(), addr);
}

TEST_F(Fixture, D5Rows)
{
  // refused origin: zero expectations -> unsupported
  uint64_t k = publish({}, Origin::NO_CONTEXT);
  EXPECT_EQ(t.on_sample(0, g, k, sample(1)), SampleOutcome::UNSUPPORTED);
  // unknown GID -> unresolved
  EXPECT_EQ(t.on_sample(0, gid(9), 1, sample(1)), SampleOutcome::UNRESOLVED);
  // ordinal above the last assigned -> rejected(identity)
  EXPECT_EQ(t.on_sample(0, g, 99, sample(1)), SampleOutcome::REJECTED_IDENTITY);
  EXPECT_EQ(t.identity_faults(), 1u);
  // matched then duplicate
  k = publish();
  EXPECT_EQ(t.on_sample(0, g, k, sample(2)), SampleOutcome::RETAINED);
  EXPECT_EQ(t.on_sample(0, g, k, sample(2)), SampleOutcome::DUPLICATE);  // same key retained
  auto toks = move_all();
  Sample out;
  ASSERT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(t.on_sample(0, g, k, sample(2)), SampleOutcome::DUPLICATE);  // expectation MATCHED
  // terminal (not ready) then late
  k = publish();
  toks = move_all();
  EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::NOT_READY);
  EXPECT_EQ(t.on_sample(0, g, k, sample(3)), SampleOutcome::LATE);
}

TEST_F(Fixture, FailedPublishPoisonsAndItsOrdinalIsAnIdentityFault)
{
  uint64_t k = publish({0}, Origin::ADMITTED, false);
  EXPECT_TRUE(t.poisoned(g));
  EXPECT_EQ(t.in_flight(0), 0u);
  EXPECT_EQ(t.on_sample(0, g, k, sample(1)), SampleOutcome::REJECTED_IDENTITY);
  publish();  // poisoned: no expectation any more
  EXPECT_EQ(t.in_flight(0), 0u);
}

TEST_F(Fixture, FinalizedGidReuseFaults)
{
  t.finalize(g);
  t.register_gid(g);
  EXPECT_EQ(t.identity_faults(), 1u);
}

TEST_F(Fixture, RetentionOverflowAtNine)
{
  for (int i = 0; i < 9; ++i) {
    publish();
  }
  for (uint64_t k = 1; k <= 8; ++k) {
    EXPECT_EQ(t.on_sample(0, g, k, sample(static_cast<int>(k))), SampleOutcome::RETAINED);
  }
  EXPECT_EQ(t.on_sample(0, g, 9, sample(9)), SampleOutcome::OVERFLOW);
  EXPECT_EQ(t.retained(), 8u);
  EXPECT_EQ(t.counts(0).outcomes[Outcome::OVERFLOW], 1u);
}

TEST_F(Fixture, CarrierRejectsNewestAtNine)
{
  for (int i = 0; i < 9; ++i) {
    publish();
  }
  EXPECT_TRUE(t.arrive(0, 9, 0));
  EXPECT_EQ(t.carrier_size(0), 8u);
  EXPECT_EQ(t.counts(0).outcomes[Outcome::PUSH_OVERFLOW], 1u);
}

TEST_F(Fixture, AtMostOnePendingWakeAndReleaseAtTPlusD)
{
  publish();
  EXPECT_TRUE(t.arrive(0, 1, 1000));    // first record: schedule the wake
  publish();
  EXPECT_FALSE(t.arrive(0, 1, 1000));   // wake pending: no second schedule
  EXPECT_FALSE(t.pop_released(0, 999).has_value());  // not released before t+d
  auto a = t.pop_released(0, 1000);
  ASSERT_TRUE(a.has_value());
  EXPECT_EQ(t.next_wake(0, 1000), 0);   // a released head remains: next microstep
  auto b = t.pop_released(0, 1000);
  ASSERT_TRUE(b.has_value());
  EXPECT_LT(a->ordinal, b->ordinal);
  EXPECT_EQ(t.next_wake(0, 1000), -1);
  publish();
  EXPECT_TRUE(t.arrive(0, 1, 5000));
  EXPECT_EQ(t.next_wake(0, 2000), 3000);  // gap to the next head's release
}

TEST_F(Fixture, LossGapTerminalizesEarlierOrdinals)
{
  uint64_t k1 = publish();
  uint64_t k2 = publish();
  auto toks = move_all();
  EXPECT_EQ(t.on_sample(0, g, k2, sample(2)), SampleOutcome::RETAINED);
  Sample out;
  EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::LOST);
  EXPECT_EQ(toks.at(0).ordinal, k1);
  EXPECT_EQ(t.join(toks.at(1), milliseconds(0), &out), Outcome::MATCHED);
}

namespace
{
/// Every line that reaches rcutils' installed output handler. In a running
/// system that is rcl's handler, which also publishes on /rosout: an
/// rcl_publish the publication gate checks under the corr Table's lock.
std::vector<std::string> g_handled;

void handler_capture(
  const rcutils_log_location_t *, int, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  if (std::string(name) != "ros2_lf_binding.corr") {
    return;
  }
  char buf[1024];
  va_list copy;
  va_copy(copy, *args);
  vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_handled.emplace_back(buf);
}

/// Runs `f` with rcutils' output handler replaced; returns the corr lines
/// that went through it.
template<typename F>
std::vector<std::string> handler_lines_while(F && f)
{
  EXPECT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
  g_handled.clear();
  const auto previous = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(handler_capture);
  f();
  rcutils_logging_set_output_handler(previous);
  return g_handled;
}

/// Runs `f` with stderr redirected to a temporary file; returns the corr
/// WARN lines written there. The missing-sample line goes through rcutils'
/// console handler straight to stderr, never through the output handler
/// above, so a test of its text reads the console.
template<typename F>
std::vector<std::string> corr_warnings_while(F && f)
{
  EXPECT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
  std::fflush(stderr);
  FILE * tmp = std::tmpfile();
  if (tmp == nullptr) {
    ADD_FAILURE() << "tmpfile failed";
    return {};
  }
  const int saved = dup(STDERR_FILENO);
  dup2(fileno(tmp), STDERR_FILENO);
  f();
  std::fflush(stderr);
  dup2(saved, STDERR_FILENO);
  close(saved);
  std::vector<std::string> lines;
  std::rewind(tmp);
  char buf[2048];
  while (std::fgets(buf, sizeof(buf), tmp) != nullptr) {
    std::string line(buf);
    if (line.find("[WARN]") != std::string::npos &&
      line.find("[ros2_lf_binding.corr]") != std::string::npos)
    {
      lines.push_back(line);
    }
  }
  std::fclose(tmp);
  return lines;
}
}  // namespace

// The missing line is issued under
// the Table lock. Through rcl's output handler it would be published on
// /rosout, an rcl_publish the publication gate checks under the Table lock,
// so a node logging at the same moment deadlocked the process and the LOST
// record never reached the trace. The line goes to the console handler only.
TEST_F(Fixture, MissingSampleLineNeverReachesTheOutputHandler)
{
  publish();
  auto toks = move_all();
  Sample out;
  const auto handled = handler_lines_while(
    [&] {EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::NOT_READY);});
  for (const auto & line : handled) {
    ADD_FAILURE() << "the missing-sample line went through rcl's output handler, "
      "whose /rosout publish takes the publication gate and so the Table lock this line "
      "holds (a deadlock): " << line;
  }
  EXPECT_EQ(t.reconcile().missing_total, 1u);
}

// A missing sample is said at once, rate-limited per edge, and
// the record at stop counts it in missing_total beside reconciled.
TEST_F(Fixture, MissingSampleWarnsWhenDecidedAndTheRecordCountsIt)
{
  int64_t clock_ns = 0;
  t.set_clock([&clock_ns] {return clock_ns;});
  Sample out;
  auto miss = [&] {
      publish();
      auto toks = move_all();
      EXPECT_EQ(t.join(toks.at(0), milliseconds(10), &out), Outcome::TIMED_OUT);
    };
  // The first missing sample of the edge: one line at once.
  auto warned = corr_warnings_while(miss);
  ASSERT_EQ(warned.size(), 1u)
    << "a missing sample was not reported until the record at stop";
  EXPECT_NE(warned.at(0).find("edge 0"), std::string::npos) << warned.at(0);
  EXPECT_NE(warned.at(0).find("ordinal 1 TIMED_OUT"), std::string::npos) << warned.at(0);
  EXPECT_NE(warned.at(0).find("1 missing on this edge so far"), std::string::npos) << warned.at(0);
  // Two more within the period: no line.
  clock_ns += ros2_lf_binding::corr::kMissingWarnPeriodNs / 2;
  warned = corr_warnings_while([&] {miss(); miss();});
  EXPECT_TRUE(warned.empty()) << "the rate limit let a second line through: " << warned.at(0);
  // The period is over: one line, carrying the count since the last one.
  clock_ns += ros2_lf_binding::corr::kMissingWarnPeriodNs / 2;
  warned = corr_warnings_while(miss);
  ASSERT_EQ(warned.size(), 1u);
  EXPECT_NE(warned.at(0).find("ordinal 4 TIMED_OUT"), std::string::npos) << warned.at(0);
  EXPECT_NE(warned.at(0).find("4 missing on this edge so far, 3 since the last line"),
    std::string::npos) << warned.at(0);
  t.close_epoch();
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << "the accounting balanced; the loss is not an accounting fault";
  EXPECT_EQ(r.missing_total, 4u);
  EXPECT_NE(r.summary.find("\"missing_total\":4,\"reconciled\":true"), std::string::npos) <<
    "the record does not count the missing samples beside the accounting verdict: " <<
    r.summary;
}

// The other side: a clean run is reconciled with nothing missing.
TEST_F(Fixture, CleanRunIsReconciledWithNothingMissing)
{
  publish();
  auto toks = move_all();
  Sample out;
  t.on_sample(0, g, toks.at(0).ordinal, sample(1));
  EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::MATCHED);
  t.close_epoch();
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.missing_total, 0u);
  EXPECT_NE(r.summary.find("\"missing_total\":0,\"reconciled\":true"), std::string::npos) <<
    r.summary;
}

// A join with no wait gives the sample up for
// good, so it is said and counted like a timed-out one.
TEST_F(Fixture, ZeroBoundNotReadyIsSaidAndCountedMissing)
{
  publish();
  auto toks = move_all();
  Sample out;
  const auto warned = corr_warnings_while(
    [&] {EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::NOT_READY);});
  ASSERT_EQ(warned.size(), 1u)
    << "a join with no wait gave the sample up without a line";
  EXPECT_NE(warned.at(0).find("ordinal 1 NOT_READY"), std::string::npos) << warned.at(0);
  t.close_epoch();
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.missing_total, 1u) << "the give-up was not counted missing";
  EXPECT_NE(r.summary.find("\"missing_total\":1,\"reconciled\":true"), std::string::npos) <<
    r.summary;
}

TEST_F(Fixture, JoinWaitsForALateSampleAndTimesOut)
{
  uint64_t k = publish();
  auto toks = move_all();
  std::thread ingress([&]() {
      std::this_thread::sleep_for(milliseconds(20));
      t.on_sample(0, g, k, sample(4));
    });
  Sample out;
  EXPECT_EQ(t.join(toks.at(0), milliseconds(2000), &out), Outcome::MATCHED);
  ingress.join();
  EXPECT_EQ(payload_of(out), 4);
  publish();
  toks = move_all();
  EXPECT_EQ(t.join(toks.at(0), milliseconds(10), &out), Outcome::TIMED_OUT);
}

TEST_F(Fixture, RejectedDuplicateAndOrphanTokens)
{
  publish();
  auto toks = move_all();
  Token wrong = toks.at(0);
  wrong.generation = 1;
  Sample out;
  EXPECT_EQ(t.join(wrong, milliseconds(0), &out), Outcome::REJECTED);
  wrong = toks.at(0);
  wrong.epoch = 1;
  EXPECT_EQ(t.join(wrong, milliseconds(0), &out), Outcome::REJECTED);
  EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::NOT_READY);
  EXPECT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::DUPLICATE);
  Token orphan = toks.at(0);
  orphan.ordinal = 50;
  EXPECT_EQ(t.join(orphan, milliseconds(0), &out), Outcome::REJECTED);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_FALSE(r.ok);
  ASSERT_FALSE(r.problems.empty());
  EXPECT_NE(
    r.summary.find("\"reconciled\":false"), std::string::npos) << r.summary;
  bool named = false;
  for (const auto & p : r.problems) {
    named |= p == "false notification: 1 dispatch attempts without expectation on edge 0";
  }
  EXPECT_TRUE(named) << r.problems.front();
}

TEST_F(Fixture, EpochCloseBalancesAndMissingOutcomeIsNamed)
{
  publish();
  publish();
  // The token is lost, not the sample.
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  t.arrive(0, 2, 0);
  t.set_drop_token_for_test(1);
  auto tok = t.pop_released(0, 0);  // ordinal 1 is dropped, 2 is returned
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(tok->ordinal, 2u);
  Sample out;
  EXPECT_EQ(t.join(*tok, milliseconds(0), &out), Outcome::NOT_READY);
  publish();  // its sample arrives after the token closed: released at close
  // Stays in flight, closes as EPOCH_CLOSE.
  EXPECT_EQ(t.on_sample(0, g, 3, sample(3)), SampleOutcome::RETAINED);
  t.close_epoch();
  EXPECT_EQ(t.retained(), 0u);
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  const std::string want = "missing outcome: expectation (" +
    ros2_lf_binding::corr::gid_hex(g) + ", 1, edge 0) has no terminal outcome";
  bool named = false;
  for (const auto & p : r.problems) {
    named |= p == want;
  }
  EXPECT_TRUE(named) << r.problems.front();
}

// Every expectation close_epoch ends EPOCH_CLOSE is listed
// with the stop tag, and with the release tag its carrier entry holds. One never
// moved to the carrier has no recorded release tag (release_known false).
TEST_F(Fixture, EpochClosedEntriesCarryTheReleaseAndStopTags)
{
  publish();
  publish();
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  t.arrive(0, 1, 5000);  // ordinal 1 enters the carrier, ordinal 2 stays in flight
  std::vector<ros2_lf_binding::corr::Queued> terminal;
  const auto carrier = t.carrier_entries(0, &terminal);
  ASSERT_EQ(carrier.size(), 1u);
  EXPECT_TRUE(t.epoch_closed_entries().empty());
  t.close_epoch(4000, 2);
  const auto ec = t.epoch_closed_entries();
  ASSERT_EQ(ec.size(), 2u);
  EXPECT_EQ(t.epoch_close_terminals(), 2u);
  for (const auto & e : ec) {
    EXPECT_EQ(e.edge, 0u);
    EXPECT_EQ(e.q.token.gid, g);
    EXPECT_EQ(e.stop_time, 4000);
    EXPECT_EQ(e.stop_micro, 2u);
    if (e.q.token.ordinal == 1u) {
      EXPECT_TRUE(e.release_known);
      EXPECT_EQ(e.q.release_ns, carrier[0].release_ns);
      EXPECT_EQ(e.q.release_micro, carrier[0].release_micro);
    } else {
      EXPECT_EQ(e.q.token.ordinal, 2u);
      EXPECT_FALSE(e.release_known);
    }
  }
}

TEST_F(Fixture, RefusedDrainEndsTheSampleNeitherMissingNorPending)
{
  // LF refuses to queue a drain whose tag lies after the stop tag
  // (lf_schedule returns 0). The sample that drain would have delivered ends
  // schedule_refused: nothing is left to deliver, it is neither missing nor
  // pending, and the epoch still balances.
  publish();
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  EXPECT_TRUE(t.arrive(0, 1, 2000));      // first record: the drain is due at 2000
  EXPECT_EQ(t.next_wake(0, 1000), 1000);
  t.schedule_refused(0);                   // LF refused that drain
  EXPECT_FALSE(t.pop_released(0, 2000).has_value()) << "a refused sample stayed deliverable";
  EXPECT_EQ(t.next_wake(0, 2000), -1);
  t.close_epoch();
  const auto c = t.counts(0);
  const auto n = [&c](Outcome o) {
      const auto it = c.outcomes.find(o);
      return it == c.outcomes.end() ? uint64_t{0} : it->second;
    };
  EXPECT_EQ(c.expectations, 1u);
  EXPECT_EQ(n(Outcome::SCHEDULE_REFUSED), 1u);
  EXPECT_EQ(n(Outcome::MATCHED), 0u);
  EXPECT_EQ(c.missing, 0u);
  EXPECT_EQ(c.pending_at_stop, 0u);
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? std::string{} : r.problems.front());
}

namespace
{
uint64_t sample_count(const ros2_lf_binding::corr::EdgeCounts & c, SampleOutcome o)
{
  const auto it = c.samples.find(o);
  return it == c.samples.end() ? uint64_t{0} : it->second;
}
}  // namespace

TEST_F(Fixture, StopTagSampleAfterTheRefusalClosesTheEpochNotLate)
{
  // The stop-tag publication's drain is refused past the stop
  // tag and its intra-process sample is taken after that refusal (a recorded
  // two-host run, side a, edge 7, ordinal 4044). Its expectation ended at the stop
  // tag, so the sample closes with the epoch. It was counted late, and only
  // in the runs where the take lost the race, so the reconcile differed.
  // a d = 0 edge, as /sim/state: the release is (t, 1), the stop tag (t, 0)
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));    // release (stop, 1), past the stop tag (stop, 0)
  t.schedule_refused(1, stop, 0);
  const SampleOutcome o = t.on_sample(1, g, 1, sample(1));
  EXPECT_TRUE(o == SampleOutcome::EPOCH_CLOSE) <<
    "expected EPOCH_CLOSE, got " << ros2_lf_binding::corr::sample_outcome_name(o);
  t.close_epoch(stop, 0);
  const auto c = t.counts(1);
  EXPECT_EQ(sample_count(c, SampleOutcome::LATE), 0u) << "a no-loss run reported a late sample";
  EXPECT_EQ(sample_count(c, SampleOutcome::EPOCH_CLOSE), 1u);
  EXPECT_EQ(sample_count(c, SampleOutcome::RETAINED), 0u);
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? std::string{} : r.problems.front());
}

TEST_F(Fixture, StopTagSampleCountsTheSameWhicheverSideWinsTheRace)
{
  // The other order of the same race (another run, side a, edge 7):
  // the sample is retained, then the drain is refused past the stop tag. The
  // per-edge sample counts must equal the late-arrival order's above.
  // a d = 0 edge, as /sim/state: the release is (t, 1), the stop tag (t, 0)
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_EQ(t.on_sample(1, g, 1, sample(1)), SampleOutcome::RETAINED);
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));
  t.schedule_refused(1, stop, 0);
  t.close_epoch(stop, 0);
  const auto c = t.counts(1);
  EXPECT_EQ(sample_count(c, SampleOutcome::LATE), 0u);
  EXPECT_EQ(sample_count(c, SampleOutcome::EPOCH_CLOSE), 1u) <<
    "the retained stop-tag sample is counted retained, the late-arrival order epoch_close";
  EXPECT_EQ(sample_count(c, SampleOutcome::RETAINED), 0u);
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? std::string{} : r.problems.front());
}

namespace
{
/// The reconcile summary's epoch_close_ids entry of edge 1, or "".
std::string epoch_close_ids_of(const std::string & summary)
{
  const auto e = summary.find("{\"edge\":1,");
  if (e == std::string::npos) {
    return {};
  }
  const auto k = summary.find("\"epoch_close_ids\":[", e);
  const auto next = summary.find("{\"edge\":2,", e);
  if (k == std::string::npos || (next != std::string::npos && k > next)) {
    return {};
  }
  return summary.substr(k, summary.find(']', k) + 1 - k);
}
}  // namespace

TEST_F(Fixture, ReconcileNamesTheSampleTheStopRefusalClosed)
{
  // The stop-tag sample retained
  // before its refusal moves to epoch_close in schedule_refused with no trace
  // of its own (desk P03: /brake_cmd ordinals 159 to 162 traced 'retained',
  // reconcile epoch_close 4). The reconcile line must name it by identity,
  // so the gate can prove the epoch_close samples ARE the refused tokens.
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_EQ(t.on_sample(1, g, 1, sample(1)), SampleOutcome::RETAINED);
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));    // release (stop, 1), past the stop tag
  t.schedule_refused(1, stop, 0);
  t.close_epoch(stop, 0);
  const auto r = t.reconcile();
  const std::string want = "\"epoch_close_ids\":[{\"gid\":\"" +
    ros2_lf_binding::corr::gid_hex(g) + "\",\"ordinal\":1}]";
  EXPECT_EQ(epoch_close_ids_of(r.summary), want) <<
    "the reconcile JSON does not name the sample schedule_refused moved to epoch_close: " <<
    r.summary;
}

TEST_F(Fixture, ReconcileNamesTheStopTagSampleTakenAfterItsRefusal)
{
  // The other side of the race, the sample taken after the refusal
  // (desk P03 edge 9, ordinal 4044).
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));    // release (stop, 1), past the stop tag
  t.schedule_refused(1, stop, 0);
  EXPECT_TRUE(t.on_sample(1, g, 1, sample(1)) == SampleOutcome::EPOCH_CLOSE);
  t.close_epoch(stop, 0);
  const auto r = t.reconcile();
  const std::string want = "\"epoch_close_ids\":[{\"gid\":\"" +
    ros2_lf_binding::corr::gid_hex(g) + "\",\"ordinal\":1}]";
  EXPECT_EQ(epoch_close_ids_of(r.summary), want) <<
    "the reconcile JSON does not name the epoch_close sample taken after the refusal: " <<
    r.summary;
}

TEST_F(Fixture, StopDrainWaitsForTheStopTagSampleItsRefusalEnded)
{
  // Stop drain: the stop-tag publication's drain is refused past
  // the stop tag before its sample was taken. The stop drain must still owe
  // that sample, or it returns at once and the run ends with samples_taken
  // one short (a workstation-pair smoke: 4043 vs 4044 in 1 run of 5).
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));    // release (stop, 1), past the stop tag
  t.schedule_refused(1, stop, 0);
  EXPECT_EQ(t.drain_in_flight(0), 1u) <<
    "the stop drain owes nothing, so it never takes the stop-tag sample: "
    "samples_taken stays one short (4043 vs 4044)";
  EXPECT_TRUE(t.on_sample(1, g, 1, sample(1)) == SampleOutcome::EPOCH_CLOSE);
  EXPECT_EQ(t.drain_in_flight(0), 0u) << "the taken stop-tag sample is still owed";
  EXPECT_EQ(t.counts(1).samples_taken, 1u);
  t.close_epoch(stop, 0);
  const auto c = t.counts(1);
  EXPECT_EQ(sample_count(c, SampleOutcome::EPOCH_CLOSE), 1u);
  EXPECT_EQ(sample_count(c, SampleOutcome::LATE), 0u);
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? std::string{} : r.problems.front());
}

TEST_F(Fixture, StopTagSampleThatNeverArrivesStaysOwed)
{
  // Stop drain: a refused stop-tag sample that never comes keeps its
  // writer owing one after the bound, so G7 fails loudly ("1 writer(s) still
  // owe one"), never passes silently. A refusal before the stop tag owes
  // nothing new: that sample is late, not drained.
  t.add_edge(1, 8, 0);
  const int64_t stop = 2000;
  publish({1});
  EXPECT_TRUE(t.arrive(1, 1, stop, 0));
  t.schedule_refused(1, stop, 0);
  EXPECT_EQ(t.drain_in_flight(1000000), 1u) << "a missing stop-tag sample passed silently";
  const size_t owed = t.drain_in_flight(0);
  publish();
  EXPECT_TRUE(t.arrive(0, 1, 1000, 0));    // release (1000, 1), before the stop tag
  t.schedule_refused(0, stop, 0);
  EXPECT_EQ(t.drain_in_flight(0), owed) << "a refusal before the stop tag made the drain wait";
}

TEST_F(Fixture, SampleLateBeforeTheStopTagIsStillLate)
{
  // Only an expectation the stop tag closed makes its sample epoch
  // close. (a) A refusal whose release lies at or before the stop tag (a
  // same-tag conflict, reactor-c schedule.c returns 0 for both) is no stop.
  const int64_t stop = 5000;
  publish();
  EXPECT_TRUE(t.arrive(0, 1, 2000, 0));    // release (2000, 1), before the stop tag
  t.schedule_refused(0, stop, 0);
  SampleOutcome o = t.on_sample(0, g, 1, sample(1));
  EXPECT_TRUE(o == SampleOutcome::LATE) <<
    "expected late, got " << ros2_lf_binding::corr::sample_outcome_name(o);
  // (b) No stop tag known: a refusal is never the stop.
  publish();
  EXPECT_TRUE(t.arrive(0, 1, 2000, 0));
  t.schedule_refused(0);
  o = t.on_sample(0, g, 2, sample(2));
  EXPECT_TRUE(o == SampleOutcome::LATE) <<
    "expected late, got " << ros2_lf_binding::corr::sample_outcome_name(o);
  // (c) The expectation ended mid-run at its dispatch (NOT_READY).
  publish();
  auto toks = move_all();
  Sample out;
  ASSERT_EQ(t.join(toks.at(0), milliseconds(0), &out), Outcome::NOT_READY);
  o = t.on_sample(0, g, 3, sample(3));
  EXPECT_TRUE(o == SampleOutcome::LATE) <<
    "expected late, got " << ros2_lf_binding::corr::sample_outcome_name(o);
  EXPECT_EQ(sample_count(t.counts(0), SampleOutcome::LATE), 3u);
  EXPECT_EQ(sample_count(t.counts(0), SampleOutcome::EPOCH_CLOSE), 0u);
}

TEST_F(Fixture, TombstoneRingNeverErasesAQueuedToken)
{
  publish();
  publish();
  t.arrive(0, 2, 0);                                            // carrier [1, 2]
  EXPECT_EQ(t.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);  // 1 -> LOST, still queued
  for (int i = 0; i < 80; ++i) {
    publish();
  }
  t.arrive(0, t.in_flight(0), 0);  // 6 fit, 74 end PUSH_OVERFLOW: > 64 tombstones
  auto first = t.pop_released(0, 0);
  ASSERT_TRUE(first.has_value());
  Sample out;
  EXPECT_EQ(t.join(*first, milliseconds(0), &out), Outcome::LOST);  // not a false orphan
  EXPECT_EQ(t.counts(0).orphan_dispatches, 0u);
}

TEST_F(Fixture, UntrackedPublisherIsCountedApartFromRefusals)
{
  // A publisher on no admitted edge (/clock, /rosout) is not a refusal.
  const Gid other = gid(2);
  t.register_gid(other);
  t.predict(other, 1, Origin::NO_CONTEXT, {}, false);
  t.commit(other, 1, true);
  t.note_nested(other, 2, false);
  publish({}, Origin::NO_CONTEXT);  // tracked publisher outside a reaction: refused
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"refused_origin\":1,\"untracked\":2,"),
      std::string::npos) << r.summary;
  EXPECT_NE(r.summary.find("\"refused\":{\"no_context\":1}"), std::string::npos) << r.summary;
}

TEST_F(Fixture, CleanRunReconciles)
{
  for (int i = 0; i < 5; ++i) {
    uint64_t k = publish();
    t.on_sample(0, g, k, sample(i));
  }
  publish({}, Origin::NO_CONTEXT);
  auto toks = move_all();
  for (const auto & tok : toks) {
    Sample out;
    EXPECT_EQ(t.join(tok, milliseconds(0), &out), Outcome::MATCHED);
  }
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"MATCHED\":5"), std::string::npos) << r.summary;
  EXPECT_NE(r.summary.find("\"no_context\":1"), std::string::npos) << r.summary;
}


// --- missing-message semantics ----------------------------------------

TEST_F(Fixture, EarlyOverflowIsDispatchedOnceAsMissing)
{
  // Nine samples arrive before their publishes commit: the ninth overflows
  // retention. Its token must still reach the subscriber, once, as missing.
  std::vector<uint64_t> ks;
  for (int i = 0; i < 9; ++i) {
    ks.push_back(++next);
    t.predict(g, next, Origin::ADMITTED, {0});
  }
  for (uint64_t k : ks) {
    t.on_sample(0, g, k, sample(static_cast<int>(k)));
  }
  std::vector<Token> toks;
  for (size_t i = 0; i < ks.size(); ++i) {  // two arrives: the carrier holds 8
    t.commit(g, ks[i], true);
    if (i == 4 || i + 1 == ks.size()) {
      for (const auto & tok : move_all()) {
        toks.push_back(tok);
      }
    }
  }
  ASSERT_EQ(toks.size(), 9u) << "expected 9 tokens, the early overflow one included";
  for (size_t i = 0; i < 8; ++i) {
    Sample out;
    EXPECT_EQ(t.join(toks[i], milliseconds(0), &out), Outcome::MATCHED);
  }
  Sample out;
  EXPECT_EQ(t.join(toks[8], milliseconds(0), &out), Outcome::OVERFLOW);
  EXPECT_EQ(t.join(toks[8], milliseconds(0), &out), Outcome::DUPLICATE);
  EXPECT_EQ(t.counts(0).missing, 1u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"missing_total\":1,\"missing\":[{\"gid\":\"01"),
    std::string::npos) << r.summary;
  EXPECT_NE(r.summary.find("\"ordinal\":9}]"), std::string::npos) << r.summary;
}

TEST_F(Fixture, EarlyOverflowPastTheCarrierCapStillReachesOneMissingDispatch)
{
  // Review of b234d7b, finding 1: nine early samples, one arrive. The carrier
  // takes eight, the ninth token is already OVERFLOW and must still be drained.
  std::vector<uint64_t> ks;
  for (int i = 0; i < 9; ++i) {
    ks.push_back(++next);
    t.predict(g, next, Origin::ADMITTED, {0});
  }
  for (uint64_t k : ks) {
    t.on_sample(0, g, k, sample(static_cast<int>(k)));
  }
  for (uint64_t k : ks) {
    t.commit(g, k, true);
  }
  auto toks = move_all();
  ASSERT_EQ(toks.size(), 9u) << "expected 9 tokens, the early overflow one included, got " <<
    toks.size();
  for (size_t i = 0; i < 8; ++i) {
    Sample out;
    EXPECT_EQ(toks[i].ordinal, ks[i]);
    EXPECT_EQ(t.join(toks[i], milliseconds(0), &out), Outcome::MATCHED);
  }
  Sample out;
  EXPECT_EQ(toks[8].ordinal, 9u);
  EXPECT_EQ(t.join(toks[8], milliseconds(0), &out), Outcome::OVERFLOW);
  EXPECT_EQ(t.counts(0).missing, 1u) << "expected missing_total 1, got " << t.counts(0).missing;
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"ordinal\":9}]"), std::string::npos) << r.summary;
}

TEST_F(Fixture, CutoffPastInt64SaturatesToNoCutoff)
{
  // Review of b234d7b, finding 2: tag + d + T must not wrap negative.
  Table u{44};
  int64_t clock = 0;
  u.set_clock([&clock] {return clock;});
  u.add_edge(0, 7, 1000000, 100000000);
  u.register_gid(g);
  // A real-time cutoff counts from the publish moment (the clock at
  // predict), so the publish moment sits next to INT64_MAX here.
  clock = kNoCutoff - 50000000;
  u.predict(g, 1, Origin::ADMITTED, {0}, true, kNoCutoff - 50000000);
  u.commit(g, 1, true);
  u.arrive(0, u.in_flight(0), 0);
  auto tok = u.pop_released(0, 0);
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(u.cutoff(*tok), kNoCutoff) << "expected no cutoff (INT64_MAX), got " <<
    u.cutoff(*tok);
  SampleOutcome o = u.on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << "expected retained, got " <<
    ros2_lf_binding::corr::sample_outcome_name(o);
}

// Inside one federate the cutoff is fixed at predict. It counts
// from S(a), the activation start, not from the clock at the publish call.
TEST_F(Fixture, InProcessCutoffCountsFromTheActivationStart)
{
  constexpr int64_t kD = 1000000;  // d = 1 ms, the edge's after delay
  constexpr int64_t kT = 100000000;  // T = 100 ms, the edge's timeout
  constexpr int64_t kSa = 5000000000;  // S(a), 5 s into the run
  constexpr int64_t kRun = 60000000;  // 60 ms of user code before the publish
  Table u{45};
  int64_t clock = kSa + kRun;
  u.set_clock([&clock] {return clock;});
  u.add_edge(0, 7, kD, kT);
  u.register_gid(g);
  u.predict(g, 1, Origin::ADMITTED, {0}, true, kSa, kSa);
  u.commit(g, 1, true);
  u.arrive(0, u.in_flight(0), 0);
  auto tok = u.pop_released(0, 0);
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(u.cutoff(*tok), kSa + kD + kT) << "expected S(a) + d + T = " << kSa + kD + kT <<
    ", got " << u.cutoff(*tok) << " (publish moment + d + T is " << kSa + kRun + kD + kT << ")";
}

TEST_F(Fixture, DefaultLagLimitPastInt64SaturatesToNoCheck)
{
  // Review of b234d7b, finding 3: L = 2 * T must not wrap negative.
  Table u{45};
  u.add_edge(0, 7, 1000000, kNoCutoff / 2 + 1);
  EXPECT_EQ(u.max_lag_ns(0), kNoCutoff) << "expected L = INT64_MAX (no check), got " <<
    u.max_lag_ns(0);
  u.note_lag(0, 0, 1000000, "complete");
  u.close_epoch();
  auto r = u.reconcile();
  EXPECT_TRUE(r.ok) << "expected no lag problem, got " <<
    (r.problems.empty() ? "" : r.problems.front());
}

/// Edge 0 with d = 1 ms, T = 100 ms (so L = 200 ms) and a settable clock.
struct CutoffFixture : ::testing::Test
{
  static constexpr int64_t kTag = 1000000000;
  static constexpr int64_t kCutoff = kTag + 1000000 + 100000000;
  Table t{43};
  const Gid g = gid(1);
  uint64_t next = 0;
  int64_t clock = kTag;  // the publish moment, the cutoff counts from it
  void SetUp() override
  {
    t.set_clock([this] {return clock;});
    t.add_edge(0, 7, 1000000, 100000000);
    t.register_gid(g);
  }
  uint64_t predict()
  {
    clock = kTag;  // published at its tag, the cutoff counts from here
    t.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag);
    return next;
  }
  Token arrive_one()
  {
    t.arrive(0, t.in_flight(0), kTag + 1000000);
    auto tok = t.pop_released(0, kTag + 1000000);
    EXPECT_TRUE(tok.has_value());
    t.next_wake(0, kTag + 1000000);
    return tok.value_or(Token{});
  }
};

std::string late_msg(SampleOutcome o)
{
  return std::string("expected LATE, got ") + ros2_lf_binding::corr::sample_outcome_name(o);
}

TEST_F(CutoffFixture, StampedAfterCutoffIsLateBeforeBetweenAndAfterArrive)
{
  // Between predict and commit: the cutoff exists before the real publish.
  uint64_t k1 = predict();
  clock = kCutoff + 1;
  SampleOutcome o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  t.commit(g, k1, true);
  Sample out;
  Token tok1 = arrive_one();
  EXPECT_EQ(t.join(tok1, milliseconds(0), &out), Outcome::NOT_READY);

  // After commit, before arrive.
  uint64_t k2 = predict();
  clock = kCutoff + 1;
  t.commit(g, k2, true);
  o = t.on_sample(0, g, k2, sample(2));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  Token tok2 = arrive_one();
  EXPECT_EQ(t.join(tok2, milliseconds(0), &out), Outcome::NOT_READY);

  // After arrive, before the dispatch.
  uint64_t k3 = predict();
  clock = kCutoff + 1;
  t.commit(g, k3, true);
  Token tok3 = arrive_one();
  o = t.on_sample(0, g, k3, sample(3));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  EXPECT_EQ(t.join(tok3, milliseconds(0), &out), Outcome::NOT_READY);

  EXPECT_EQ(t.retained(), 0u);
  EXPECT_EQ(t.counts(0).missing, 3u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"missing_total\":3,"), std::string::npos) << r.summary;
  // Three keys, all listed.
  EXPECT_NE(r.summary.find("\"missing_truncated\":false"), std::string::npos) << r.summary;
}

TEST_F(CutoffFixture, JoinNeverMatchesASampleStampedPastTheCutoff)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff + 1;  // the sample is taken after the cutoff
  SampleOutcome o = t.on_sample(0, g, k, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  clock = kCutoff - 1000000;  // the join entered 1 ms before the cutoff
  Sample out;
  const Outcome j = t.join(tok, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::TIMED_OUT) <<
    "expected TIMED_OUT, got " << ros2_lf_binding::corr::outcome_name(j);
  EXPECT_EQ(out.message, nullptr);
}

TEST_F(CutoffFixture, StampAtTheCutoffMatchesEvenWhenTheJoinIsLater)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff;
  EXPECT_EQ(t.on_sample(0, g, k, sample(7)), SampleOutcome::RETAINED);
  clock = kCutoff + 50000000;  // a slow worker joins 50 ms after the cutoff
  Sample out;
  EXPECT_EQ(t.join(tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 7);
}

TEST_F(CutoffFixture, UnpredictedOrdinalStaysAnIdentityFaultPastTheCutoff)
{
  predict();
  t.commit(g, 1, true);
  clock = kCutoff + 1;
  EXPECT_EQ(t.on_sample(0, g, 5, sample(5)), SampleOutcome::REJECTED_IDENTITY);
  EXPECT_EQ(t.identity_faults(), 1u);
}

TEST_F(CutoffFixture, LagAboveLAtCompletionIsAReconcileProblem)
{
  EXPECT_EQ(t.max_lag_ns(0), 200000000);
  t.note_lag(0, kTag, 150000000, "complete");  // below L: no problem
  t.close_epoch();
  EXPECT_TRUE(t.reconcile().ok);
  t.note_lag(0, kTag, 250000000, "complete");
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  ASSERT_EQ(r.problems.size(), 1u);
  EXPECT_NE(r.problems[0].find("worker_lag_exceeded"), std::string::npos) << r.problems[0];
  EXPECT_NE(r.problems[0].find("point=complete"), std::string::npos) << r.problems[0];
}

TEST_F(CutoffFixture, UndrainedTokenPastReleasePlusLIsAReconcileProblem)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  t.arrive(0, t.in_flight(0), kTag + 1000000);
  clock = kTag + 1000000 + 200000001;
  t.close_epoch();
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.problems[0].find("undrained"), std::string::npos) << r.problems[0];
}

/// Edge 0 is a local self edge, the cutoff fixture otherwise unchanged.
struct SelfEdgeFixture : CutoffFixture
{
  void SetUp() override
  {
    CutoffFixture::SetUp();
    t.set_self_local(0);
  }
};

std::string retained_msg(SampleOutcome o)
{
  return std::string("expected RETAINED (a present sample on a self edge skips the "
           "cutoff), got ") + ros2_lf_binding::corr::sample_outcome_name(o);
}

std::string outcome_msg(Outcome want, Outcome got)
{
  return std::string("expected ") + ros2_lf_binding::corr::outcome_name(want) + ", got " +
         ros2_lf_binding::corr::outcome_name(got);
}

TEST_F(SelfEdgeFixture, PresentSamplePastTheCutoffIsMatchedBeforeBetweenAndAfterArrive)
{
  Sample out;
  // Between predict and commit: Fast DDS hands the sample over inside the write.
  uint64_t k1 = predict();
  clock = kCutoff + 1;
  SampleOutcome o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << retained_msg(o);
  t.commit(g, k1, true);
  Token tok1 = arrive_one();
  clock = kCutoff + 50000000;  // the drain runs 50 ms past the cutoff
  Outcome j = t.join(tok1, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 1);

  // After commit, before arrive.
  uint64_t k2 = predict();
  t.commit(g, k2, true);
  o = t.on_sample(0, g, k2, sample(2));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << retained_msg(o);
  Token tok2 = arrive_one();
  j = t.join(tok2, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 2);

  // After arrive, before the dispatch.
  uint64_t k3 = predict();
  t.commit(g, k3, true);
  Token tok3 = arrive_one();
  o = t.on_sample(0, g, k3, sample(3));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << retained_msg(o);
  j = t.join(tok3, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 3);

  EXPECT_EQ(t.retained(), 0u);
  EXPECT_EQ(t.counts(0).missing, 0u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"outcomes\":{\"MATCHED\":3}"), std::string::npos) << r.summary;
}

TEST_F(SelfEdgeFixture, AbsentSampleIsDecidedWithoutAWait)
{
  // A self edge is a local_now edge. Its sample is in the reader when
  // the publish returns, so an absent one never waits for a cutoff: the join
  // ends it LOST (local_undelivered), never TIMED_OUT after the 20 ms left.
  EXPECT_TRUE(t.local_now(0)) << "a self edge is not a local_now edge";
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  EXPECT_EQ(t.cutoff(tok), kNoCutoff);
  clock = kCutoff - 20000000;  // the join enters 20 ms before a timed edge's cutoff
  Sample out;
  const uint64_t waits = t.join_waits();
  Outcome j = t.join(tok, milliseconds(5000), &out);
  EXPECT_EQ(j, Outcome::LOST) << outcome_msg(Outcome::LOST, j);
  EXPECT_EQ(t.join_waits(), waits) << "the join waited: it entered the timed wait on a "
    "local_now edge";
  EXPECT_EQ(out.message, nullptr);
  // The sample shows up after its expectation ended: late, never matched.
  SampleOutcome o = t.on_sample(0, g, k, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  EXPECT_EQ(t.counts(0).missing, 1u);
  EXPECT_EQ(t.counts(0).local_undelivered, 1u);
  t.close_epoch();
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.problems[0].find("local_undelivered edge 0: 1"), std::string::npos) <<
    r.problems[0];
}

TEST_F(SelfEdgeFixture, IdentityDuplicateAndRetentionStillApply)
{
  predict();
  t.commit(g, 1, true);
  clock = kCutoff + 1;
  EXPECT_EQ(t.on_sample(0, g, 5, sample(5)), SampleOutcome::REJECTED_IDENTITY);
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::DUPLICATE);
  EXPECT_EQ(t.identity_faults(), 1u);

  Table u{47};
  u.set_clock([this] {return clock;});
  u.add_edge(0, 7, 1000000, 100000000, -1, 1, 3, 1);
  u.set_self_local(0);
  u.register_gid(g);
  for (uint64_t k = 1; k <= 4; ++k) {
    u.predict(g, k, Origin::ADMITTED, {0}, true, kTag);
    u.commit(g, k, true);
  }
  for (uint64_t k = 1; k <= 3; ++k) {
    EXPECT_EQ(u.on_sample(0, g, k, sample(static_cast<int>(k))), SampleOutcome::RETAINED);
  }
  EXPECT_EQ(u.on_sample(0, g, 4, sample(4)), SampleOutcome::OVERFLOW) <<
    "sample 4 exceeds retain 3, past the cutoff as well";
}

TEST(CorrSelfEdge, OtherLocalAndIngressEdgesKeepTheCutoff)
{
  constexpr int64_t kTag = CutoffFixture::kTag;
  constexpr int64_t kCut = CutoffFixture::kCutoff;
  int64_t clock = kTag;  // published at its tag
  Table t{48};
  t.set_clock([&clock] {return clock;});
  t.add_edge(0, 7, 1000000, 100000000);  // the self edge
  t.add_edge(1, 8, 1000000, 100000000);  // a local edge between two nodes
  t.set_self_local(0);
  t.set_self_local(5);  // no such edge: ignored
  const Gid g = gid(1);
  t.register_gid(g);
  t.predict(g, 1, Origin::ADMITTED, {0, 1}, true, kTag);
  t.commit(g, 1, true);
  clock = kCut + 1;
  EXPECT_EQ(t.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  SampleOutcome o = t.on_sample(1, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);

  // An ingress edge never becomes a self edge.
  Table x{49};
  x.set_clock([&clock] {return clock;});
  x.add_edge(0, 7, 1000000, 100000000, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  x.set_self_local(0);
  std::vector<ros2_lf_binding::corr::XRecord> rec{{g, 1, kTag, kTag}};
  x.arrive_remote(0, rec, kTag, kTag + 1000000, 0);
  o = x.on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
}

/// Edge 0 of the cutoff fixture (d = 1 ms, T = 100 ms) is a local edge
/// between two nodes, decided when its publish returns.
struct LocalNowFixture : CutoffFixture
{
  void SetUp() override
  {
    CutoffFixture::SetUp();
    t.set_local_now(0);
  }
  std::string summary()
  {
    t.close_epoch();
    return t.reconcile().summary;
  }
};

constexpr int64_t kTenSeconds = 10000000000;

TEST_F(LocalNowFixture, SampleTenSecondsPastATimedCutoffIsRetainedAndMatched)
{
  // (a) The worker that takes the sample runs 10 s behind a timed edge's
  // cutoff. A local edge is never late: retained, then matched.
  EXPECT_TRUE(t.local_now(0));
  Sample out;
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff + kTenSeconds;
  SampleOutcome o = t.on_sample(0, g, k, sample(9));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << "expected RETAINED (a local_now edge has no "
    "cutoff), got " << ros2_lf_binding::corr::sample_outcome_name(o);
  Outcome j = t.join(tok, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 9);
  // The same between predict and commit, the take inside rcl_publish.
  uint64_t k2 = predict();
  clock = kCutoff + kTenSeconds;
  o = t.on_sample(0, g, k2, sample(10));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << "expected RETAINED (a local_now edge has no "
    "cutoff), got " << ros2_lf_binding::corr::sample_outcome_name(o);
  t.commit(g, k2, true);
  EXPECT_EQ(t.settle_local(g, k2, {}), 0u) << "a retained sample was settled LOST";
  Token tok2 = arrive_one();
  j = t.join(tok2, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(t.counts(0).missing, 0u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  EXPECT_NE(r.summary.find("\"outcomes\":{\"MATCHED\":2}"), std::string::npos) << r.summary;
}

TEST_F(LocalNowFixture, SettledAbsentSampleIsLostWithoutAWait)
{
  // (b) No sample in the reader after the publish: settle_local ends it LOST,
  // and the drain's join returns at once, not after the 5 s bound or the
  // 101 ms a timed edge would wait for its cutoff.
  uint64_t k = predict();
  t.commit(g, k, true);
  EXPECT_EQ(t.local_now_edges(g, k), std::vector<uint32_t>{0});
  EXPECT_EQ(t.settle_local(g, k, {}), 1u);
  EXPECT_EQ(t.settle_local(g, k, {}), 0u) << "settled twice";
  Token tok = arrive_one();
  Sample out;
  const uint64_t waits = t.join_waits();
  Outcome j = t.join(tok, milliseconds(5000), &out);
  EXPECT_EQ(j, Outcome::LOST) << outcome_msg(Outcome::LOST, j);
  EXPECT_EQ(t.join_waits(), waits) << "the join waited: it entered the timed wait on a "
    "local_now edge";
  SampleOutcome o = t.on_sample(0, g, k, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  EXPECT_EQ(t.counts(0).missing, 1u);
  EXPECT_EQ(t.counts(0).local_undelivered, 1u);
  EXPECT_EQ(t.counts(0).local_undelivered_planted, 0u);
  t.close_epoch();
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok) << "an unplanted local_undelivered must fail the run (exit 3)";
  ASSERT_EQ(r.problems.size(), 1u) << r.problems.back();
  EXPECT_NE(r.problems[0].find("local_undelivered edge 0: 1"), std::string::npos) <<
    r.problems[0];
  EXPECT_NE(r.summary.find("\"local_undelivered\":1,\"local_undelivered_planted\":0"),
    std::string::npos) << r.summary;
  // Nothing of a settled local edge is in flight at stop.
  EXPECT_NE(r.summary.find("\"owed_at_close\":0"), std::string::npos) << r.summary;
}

TEST_F(LocalNowFixture, PlantedDropIsCountedPerEdgeAndOnlyItPasses)
{
  // A planted drop on one subscriber must not hide a
  // real undelivered sample on a second one.
  t.add_edge(1, 8, 1000000, 100000000);
  t.set_local_now(1);
  clock = kTag;
  t.predict(g, 1, Origin::ADMITTED, {0, 1}, true, kTag);
  t.commit(g, 1, true);
  EXPECT_EQ(t.local_now_edges(g, 1), (std::vector<uint32_t>{0, 1}));
  EXPECT_EQ(t.settle_local(g, 1, {0}), 2u);
  EXPECT_EQ(t.counts(0).local_undelivered_planted, 1u);
  EXPECT_EQ(t.counts(0).local_undelivered, 0u);
  EXPECT_EQ(t.counts(1).local_undelivered, 1u);
  for (uint32_t e : {0u, 1u}) {
    t.arrive(e, t.in_flight(e), kTag + 1000000);
    auto tok = t.pop_released(e, kTag + 1000000);
    ASSERT_TRUE(tok.has_value());
    Sample out;
    const Outcome j = t.join(*tok, milliseconds(5000), &out);
    EXPECT_EQ(j, Outcome::LOST) << "edge " << e << ": " << outcome_msg(Outcome::LOST, j);
  }
  t.close_epoch();
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  ASSERT_EQ(r.problems.size(), 1u) << r.problems.back();
  EXPECT_NE(r.problems[0].find("local_undelivered edge 1: 1"), std::string::npos) <<
    r.problems[0];

  // Only the planted drop: the run reconciles.
  Table u{60};
  u.add_edge(0, 7, 1000000, 100000000);
  u.set_local_now(0);
  u.register_gid(g);
  u.predict(g, 1, Origin::ADMITTED, {0}, true, kTag);
  u.commit(g, 1, true);
  EXPECT_EQ(u.settle_local(g, 1, {0}), 1u);
  u.arrive(0, u.in_flight(0), 0);
  auto tok = u.pop_released(0, 0);
  ASSERT_TRUE(tok.has_value());
  Sample out;
  const Outcome j = u.join(*tok, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::LOST) << outcome_msg(Outcome::LOST, j);
  u.close_epoch();
  auto ru = u.reconcile();
  EXPECT_TRUE(ru.ok) << (ru.problems.empty() ? "" : ru.problems.front());
  EXPECT_NE(ru.summary.find("\"local_undelivered\":0,\"local_undelivered_planted\":1"),
    std::string::npos) << ru.summary;
}

TEST_F(LocalNowFixture, UnsettledAbsentSampleEndsLostAtTheJoinWithoutAWait)
{
  // No settle_local (a wiring gap): the join still never waits on a local edge.
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  Sample out;
  const uint64_t waits = t.join_waits();
  Outcome j = t.join(tok, milliseconds(5000), &out);
  EXPECT_EQ(j, Outcome::LOST) << outcome_msg(Outcome::LOST, j);
  EXPECT_EQ(t.join_waits(), waits) << "the join waited: it entered the timed wait on a "
    "local_now edge";
  EXPECT_EQ(t.counts(0).local_undelivered, 1u);
}

TEST_F(LocalNowFixture, PredictedPublicationHasNoCutoff)
{
  // (d) the cutoff a timed edge would get is S(a) + d + T = kCutoff.
  uint64_t k = predict();
  t.commit(g, k, true);
  t.settle_local(g, k, {0});
  Token tok = arrive_one();
  EXPECT_EQ(t.cutoff(tok), kNoCutoff) << "expected no cutoff (INT64_MAX) on a local_now edge, "
    "got " << t.cutoff(tok) << " (S(a) + d + T = " << kCutoff << ")";
  Sample out;
  t.join(tok, milliseconds(0), &out);
  const std::string js = summary();
  EXPECT_NE(js.find("\"local_now\":true,\"cutoff\":\"none\""), std::string::npos) << js;
}

TEST(CorrLocalNow, CrossFederateRebindAndDeferredEdgesKeepTheirWindow)
{
  // (c) regression guard: only a plain local edge becomes local_now.
  constexpr int64_t kTag = CutoffFixture::kTag;
  constexpr int64_t kCut = CutoffFixture::kCutoff;
  using ros2_lf_binding::corr::Role;
  int64_t clock = kTag;
  const Gid g = gid(1);
  Table x{61};
  x.set_clock([&clock] {return clock;});
  x.add_edge(0, 7, 1000000, 100000000, -1, 0, 0, 0, Role::INGRESS);
  x.add_edge(1, 8, 1000000, 100000000, -1, 0, 0, 0, Role::EGRESS);
  x.add_edge(2, 9, 1000000, 100000000);  // sub_rebind, set first
  x.add_edge(3, 10, 1000000, 100000000);  // local_now, then sub_rebind
  x.add_edge(4, 11, 1000000, 100000000);  // deferred
  x.add_edge(5, 12, 1000000, 100000000);  // local_now
  x.set_sub_rebind(2);
  x.set_local_now(3);
  x.set_sub_rebind(3);
  EXPECT_TRUE(x.set_deferred(4));
  for (uint32_t e = 0; e < 6; ++e) {
    x.set_local_now(e);
  }
  x.set_local_now(9);  // no such edge: ignored
  EXPECT_FALSE(x.local_now(0)) << "an ingress edge became local_now";
  EXPECT_FALSE(x.local_now(1)) << "an egress edge became local_now";
  EXPECT_FALSE(x.local_now(2)) << "a sub_rebind edge became local_now";
  EXPECT_FALSE(x.local_now(3)) << "set_sub_rebind left local_now set";
  EXPECT_FALSE(x.local_now(4)) << "a deferred edge became local_now";
  EXPECT_TRUE(x.local_now(5));
  EXPECT_FALSE(x.set_deferred(5)) << "join: deferred accepted on a local_now edge";

  // Ingress keeps D = S(a) + d + T.
  std::vector<ros2_lf_binding::corr::XRecord> rec{{g, 1, kTag, kTag}};
  x.arrive_remote(0, rec, kTag, kTag + 1000000, 0);
  clock = kCut + 1;
  SampleOutcome o = x.on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);

  // The local edges 2, 3 and 4 keep the cutoff; edge 5 has none.
  const Gid w = gid(2);
  x.register_gid(w);
  clock = kTag;
  x.predict(w, 1, Origin::ADMITTED, {1, 2, 3, 4, 5}, true, kTag);
  x.commit(w, 1, true);
  EXPECT_EQ(x.local_now_edges(w, 1), std::vector<uint32_t>{5});
  clock = kCut + 1;
  for (uint32_t e : {2u, 3u, 4u}) {
    o = x.on_sample(e, w, 1, sample(1));
    EXPECT_EQ(o, SampleOutcome::LATE) << "edge " << e << ": " << late_msg(o);
  }
  o = x.on_sample(5, w, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << ros2_lf_binding::corr::sample_outcome_name(o);
}

TEST(CorrLocalNow, LocalNowEdgesNameEachFanInLeadOnceInEdgeOrder)
{
  const Gid g = gid(1);
  Table t{62};
  for (uint32_t e = 0; e < 5; ++e) {
    t.add_edge(e, 7 + e, 0, 100000000);
  }
  ASSERT_TRUE(t.set_fan_in(3, 3, 0));  // group led by 3: members 3 and 4
  ASSERT_TRUE(t.set_fan_in(4, 3, 1));
  for (uint32_t e : {0u, 3u, 4u}) {
    t.set_local_now(e);
  }
  t.register_gid(g);
  t.predict(g, 1, Origin::ADMITTED, {4, 1, 0, 3}, true, 0);
  EXPECT_EQ(t.local_now_edges(g, 1), (std::vector<uint32_t>{0, 3})) << "after predict";
  t.commit(g, 1, true);
  EXPECT_EQ(t.local_now_edges(g, 1), (std::vector<uint32_t>{0, 3}));
  // Planted is per edge: the lead's id does not cover member 4's real loss.
  EXPECT_EQ(t.settle_local(g, 1, {3}), 3u);
  EXPECT_EQ(t.counts(3).local_undelivered_planted, 1u);
  EXPECT_EQ(t.counts(4).local_undelivered_planted, 0u);
  EXPECT_EQ(t.counts(4).local_undelivered, 1u) << "a planted lead hid member 4's loss";
  EXPECT_EQ(t.counts(0).local_undelivered, 1u);
  EXPECT_EQ(t.counts(1).local_undelivered, 0u) << "a timed edge was settled";
  t.predict(g, 2, Origin::ADMITTED, {0}, true, 0);
  t.commit(g, 2, false);
  EXPECT_TRUE(t.local_now_edges(g, 2).empty()) << "a failed publish has no take";
  EXPECT_EQ(t.settle_local(g, 2, {}), 0u);
  EXPECT_TRUE(t.local_now_edges(gid(9), 1).empty()) << "unknown writer";
}

/// Edge 0 of the cutoff fixture with `join: deferred`.
struct DeferredFixture : CutoffFixture
{
  void SetUp() override
  {
    CutoffFixture::SetUp();
    t.set_deferred(0);
  }
};

TEST_F(DeferredFixture, ReleaseMovesToTheCutoffTag)
{
  EXPECT_TRUE(t.deferred(0));
  EXPECT_EQ(t.release_extra_ns(0), 100000000) << "the drain must wake at release + T";
  Table u{50};
  u.add_edge(0, 7, 1000000, 100000000);
  EXPECT_FALSE(u.deferred(0)) << "blocking stays the default";
  EXPECT_EQ(u.release_extra_ns(0), 0);
  u.set_self_local(0);
  EXPECT_FALSE(u.set_deferred(0)) << "a self edge's sample is present, the opt-in is refused";
  EXPECT_FALSE(u.deferred(0));
  EXPECT_FALSE(u.set_deferred(5)) << "no such edge";
  Table v{51};
  v.add_edge(0, 7, 1000000, 100000000, -1, 0, 0, 0, ros2_lf_binding::corr::Role::EGRESS);
  EXPECT_FALSE(v.set_deferred(0)) << "an egress edge has no join, the opt-in is refused";
  EXPECT_FALSE(v.deferred(0));
}

TEST_F(DeferredFixture, SampleTakenAfterTheCutoffBeforeALaggingJoinIsLate)
{
  // The race: the sample is taken in at or after the
  // cutoff, the LF worker has not run the join of the cutoff tag yet. It is
  // late at intake, and the lagging join never hands it out.
  Sample out;
  uint64_t k1 = predict();
  t.commit(g, k1, true);
  Token tok1 = arrive_one();
  clock = kCutoff;  // at the cutoff: late on a deferred edge
  SampleOutcome o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  clock = kCutoff + 30000000;  // the worker reaches the join 30 ms later
  Outcome j = t.join(tok1, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::TIMED_OUT) << outcome_msg(Outcome::TIMED_OUT, j);
  EXPECT_EQ(out.message, nullptr);

  uint64_t k2 = predict();
  t.commit(g, k2, true);
  Token tok2 = arrive_one();
  clock = kCutoff + 1;
  o = t.on_sample(0, g, k2, sample(2));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  clock = kCutoff + 30000000;
  j = t.join(tok2, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::TIMED_OUT) << outcome_msg(Outcome::TIMED_OUT, j);
  EXPECT_EQ(out.message, nullptr);

  EXPECT_EQ(t.retained(), 0u);
  EXPECT_EQ(t.counts(0).missing, 2u);
  // Exactly one terminal outcome per record: TIMED_OUT once each, no match,
  // and a second join of either token is a duplicate, not a new outcome.
  EXPECT_EQ(t.join(tok1, milliseconds(0), &out), Outcome::DUPLICATE);
  EXPECT_EQ(t.join(tok2, milliseconds(0), &out), Outcome::DUPLICATE);
  const auto & c = t.counts(0);
  EXPECT_EQ(c.expectations, 2u);
  uint64_t terminal = 0;
  for (const auto & kv : c.outcomes) {
    terminal += kv.second;
  }
  EXPECT_EQ(terminal, 2u) << "one terminal outcome per record";
  EXPECT_EQ(c.outcomes.count(Outcome::TIMED_OUT) ? c.outcomes.at(Outcome::TIMED_OUT) : 0u, 2u);
  EXPECT_EQ(c.outcomes.count(Outcome::MATCHED), 0u);
  EXPECT_EQ(c.samples.count(SampleOutcome::LATE) ? c.samples.at(SampleOutcome::LATE) : 0u, 2u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
}

TEST_F(DeferredFixture, PreCutoffSampleMatchesWhenTheJoinRunsLate)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff - 1;
  EXPECT_EQ(t.on_sample(0, g, k, sample(7)), SampleOutcome::RETAINED);
  clock = kCutoff + 50000000;  // a slow worker joins 50 ms after the cutoff
  Sample out;
  Outcome j = t.join(tok, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED) << outcome_msg(Outcome::MATCHED, j);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 7);
}

TEST_F(DeferredFixture, AbsentSampleTimesOutAtTheCutoffTagWithoutAWait)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff;  // LF runs the join at the cutoff tag
  Sample out;
  const uint64_t waits = t.join_waits();
  Outcome j = t.join(tok, milliseconds(5000), &out);
  EXPECT_EQ(j, Outcome::TIMED_OUT) << outcome_msg(Outcome::TIMED_OUT, j);
  EXPECT_EQ(t.join_waits(), waits) << "a deferred join waited for a lost sample";
  EXPECT_EQ(out.message, nullptr);

  EXPECT_EQ(t.counts(0).missing, 1u);
  t.close_epoch();
  EXPECT_TRUE(t.reconcile().ok);
}

TEST_F(DeferredFixture, JoinBeforeTheCutoffDecidesAtOnceAndALaterSampleIsLate)
{
  // The join decides from the table as it stands. With
  // the injected clock 20 ms before the cutoff and the sample absent, a
  // waiting join would block until the cutoff (or the 5 s bound).
  uint64_t k = predict();
  t.commit(g, k, true);
  Token tok = arrive_one();
  clock = kCutoff - 20000000;
  Sample out;
  const uint64_t waits = t.join_waits();
  Outcome j = t.join(tok, milliseconds(5000), &out);
  EXPECT_EQ(j, Outcome::TIMED_OUT) << outcome_msg(Outcome::TIMED_OUT, j);
  EXPECT_EQ(t.join_waits(), waits) << "a deferred join waited for its sample";
  EXPECT_EQ(out.message, nullptr);
  clock = kCutoff - 10000000;  // the sample comes after the join, before the cutoff
  SampleOutcome o = t.on_sample(0, g, k, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
  EXPECT_EQ(t.retained(), 0u);
  EXPECT_EQ(t.counts(0).missing, 1u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
}

TEST(CorrDeferred, IngressSampleAtTheCutoffIsLate)
{
  constexpr int64_t kTag = CutoffFixture::kTag;
  constexpr int64_t kCut = CutoffFixture::kCutoff;
  int64_t clock = kTag;
  const Gid g = gid(1);
  auto make = [&clock](uint32_t id, bool deferred) {
      auto x = std::make_unique<Table>(id);
      x->set_clock([&clock] {return clock;});
      x->add_edge(0, 7, 1000000, 100000000, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
      if (deferred) {
        x->set_deferred(0);
      }
      std::vector<ros2_lf_binding::corr::XRecord> rec{{gid(1), 1, kTag, kTag}};
      x->arrive_remote(0, rec, kTag, kTag + 1000000, 0);
      return x;
    };
  auto blocking = make(52, false);
  auto deferred = make(53, true);
  clock = kCut;
  SampleOutcome o = blocking->on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << "a blocking edge keeps stamp <= cutoff";
  o = deferred->on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << late_msg(o);
}

TEST(CorrDeferred, IngressRecordAfterTheCutoffAdoptsAPreCutoffSample)
{
  // The sample is taken in before the cutoff, its
  // record (the expectation) comes after it. A deferred edge decides by the
  // intake stamp alone and matches; a blocking edge refuses the late record.
  constexpr int64_t kTag = CutoffFixture::kTag;
  constexpr int64_t kCut = CutoffFixture::kCutoff;
  for (const bool deferred : {true, false}) {
    int64_t clock = kTag;
    const Gid g = gid(1);
    Table x{deferred ? 54u : 55u};
    x.set_clock([&clock] {return clock;});
    x.add_edge(0, 7, 1000000, 100000000, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
    if (deferred) {
      ASSERT_TRUE(x.set_deferred(0));
    }
    clock = kCut - 1;
    EXPECT_EQ(x.on_sample(0, g, 1, sample(9)), SampleOutcome::RETAINED);
    clock = kCut + 5000000;  // the record comes 5 ms after the cutoff
    std::vector<ros2_lf_binding::corr::XRecord> rec{{g, 1, kTag, kTag}};
    ASSERT_TRUE(x.arrive_remote(0, rec, kTag, clock, 0));
    auto tok = x.pop_released(0, clock);
    ASSERT_TRUE(tok.has_value());
    Sample out;
    Outcome j = x.join(*tok, milliseconds(0), &out);
    // The intake stamp decides on both join kinds. A blocking
    // edge no longer refuses an on-time sample because its record was
    // processed after the cutoff (the worker ran behind).
    EXPECT_EQ(j, Outcome::MATCHED) << (deferred ? "deferred: " : "blocking: ") <<
      outcome_msg(Outcome::MATCHED, j);
    EXPECT_EQ(out.message ? payload_of(out) : -1, 9);
    EXPECT_EQ(x.counts(0).missing, 0u);
    x.close_epoch();
    auto r = x.reconcile();
    EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  }
}

// A drain reaction runs its callback once per record due at its
// tag, and B_e bounds one invocation. Two invocations that publish one record
// each on a B = 1 edge are within the bound; the same two records from one
// invocation (no end_invocation between them) exceed it.
TEST(CorrBurst, BurstIsCountedPerCallbackInvocation)
{
  auto run = [](bool split) {
      Table t{45};
      const Gid g = gid(3);
      t.add_edge(0, 7, 1000000, -1, -1, 1);
      t.register_gid(g);
      for (uint64_t k = 1; k <= 2; ++k) {
        t.predict(g, k, Origin::ADMITTED, {0});
        t.commit(g, k, true);
        if (split) {
          t.end_invocation();
        }
      }
      uint32_t fresh = 0;
      EXPECT_EQ(t.emit(0, &fresh), 2u);
      EXPECT_EQ(fresh, 2u) << "the emit still carries both records";
      t.close_epoch();
      std::string all;
      for (const auto & p : t.reconcile().problems) {
        all += p + "\n";
      }
      return all;
    };
  const std::string split = run(true);
  EXPECT_EQ(split.find("burst_exceeded"), std::string::npos) << split;
  const std::string one = run(false);
  EXPECT_NE(one.find("burst_exceeded edge 0: 2 records in one reaction > B = 1"),
    std::string::npos) << one;
}

TEST(CorrBurst, MoreRecordsThanBInOneReactionIsAReconcileProblem)
{
  Table t{44};
  const Gid g = gid(2);
  t.add_edge(0, 7, 1000000, -1, -1, 2);
  t.register_gid(g);
  for (uint64_t k = 1; k <= 3; ++k) {
    t.predict(g, k, Origin::ADMITTED, {0});
    t.commit(g, k, true);
  }
  uint32_t fresh = 0;
  EXPECT_EQ(t.emit(0, &fresh), 3u);
  t.close_epoch();
  auto r = t.reconcile();
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.problems[0].find("burst_exceeded edge 0: 3 records"), std::string::npos) <<
    r.problems[0];
}

// The generated retain and carrier bounds replace the fixed caps.
// Fixture `one` emits retain 3, carrier 1; k 9 emits carrier 9 > kCarrierCap.
TEST(CorrBounds, RetainBoundOverflowsTheFourthSample)
{
  Table t{45};
  const Gid g = gid(3);
  t.add_edge(0, 7, 1000000, -1, -1, 1, 3, 1);
  t.register_gid(g);
  for (uint64_t k = 1; k <= 4; ++k) {
    t.predict(g, k, Origin::ADMITTED, {0});
    t.commit(g, k, true);
  }
  for (uint64_t k = 1; k <= 3; ++k) {
    EXPECT_EQ(t.on_sample(0, g, k, sample(static_cast<int>(k))), SampleOutcome::RETAINED) <<
      "sample " << k << " is within retain 3";
  }
  EXPECT_EQ(t.on_sample(0, g, 4, sample(4)), SampleOutcome::OVERFLOW) <<
    "sample 4 exceeds retain 3";
  EXPECT_EQ(t.retained(), 3u);
}

TEST(CorrBounds, CarrierBoundRejectsTheSecondToken)
{
  Table t{46};
  const Gid g = gid(4);
  t.add_edge(0, 7, 1000000, -1, -1, 1, 3, 1);
  t.register_gid(g);
  for (uint64_t k = 1; k <= 2; ++k) {
    t.predict(g, k, Origin::ADMITTED, {0});
    t.commit(g, k, true);
  }
  t.arrive(0, 2, 0);
  EXPECT_EQ(t.carrier_size(0), 1u) << "carrier 1 holds one token";
  EXPECT_EQ(t.counts(0).outcomes[Outcome::PUSH_OVERFLOW], 1u);
}

TEST(CorrBounds, CarrierBoundAboveTheDefaultCapHoldsNine)
{
  Table t{47};
  const Gid g = gid(5);
  t.add_edge(0, 7, 1000000, -1, -1, 9, 19, 9);
  t.register_gid(g);
  for (uint64_t k = 1; k <= 10; ++k) {
    t.predict(g, k, Origin::ADMITTED, {0});
    t.commit(g, k, true);
  }
  t.arrive(0, 10, 0);
  EXPECT_EQ(t.carrier_size(0), 9u) << "carrier 9 holds nine tokens, not kCarrierCap 8";
  EXPECT_EQ(t.counts(0).outcomes[Outcome::PUSH_OVERFLOW], 1u);
}

}  // namespace

// --- Cross-federate edges (egress and ingress tables) -------------------------

using ros2_lf_binding::corr::EmitStatus;
using ros2_lf_binding::corr::Role;
using ros2_lf_binding::corr::XRecord;

struct XFixture : ::testing::Test
{
  static constexpr int64_t kD = 5000000;
  static constexpr int64_t kT = 100000000;
  static constexpr int64_t kTag = 1000000000;
  static constexpr int64_t kCut = kTag + kD + kT;
  Table pub{50};
  Table sub{51};
  int64_t pclock = kTag;
  int64_t sclock = kTag;
  const Gid g = gid(9);
  uint64_t next = 0;
  void SetUp() override
  {
    pub.set_clock([this] {return pclock;});
    sub.set_clock([this] {return sclock;});
    pub.add_edge(0, 0, kD, kT, -1, 0, 0, 0, Role::EGRESS);
    sub.add_edge(0, 7, kD, kT, -1, 0, 0, 0, Role::INGRESS);
    pub.register_gid(g);
  }
  uint64_t publish(int64_t tag = kTag)
  {
    pclock = std::max(pclock, tag);  // a non-fast federate publishes at or after its tag
    pub.predict(g, ++next, Origin::ADMITTED, {0}, true, tag);
    pub.commit(g, next, true);
    return next;
  }
  std::vector<XRecord> emit(
    int64_t time = kTag, uint32_t micro = 0, EmitStatus * status = nullptr,
    uint32_t * fresh = nullptr)
  {
    std::vector<XRecord> out;
    uint32_t f = 0;
    const EmitStatus s = pub.emit_remote(0, time, micro, 8, &out, &f);
    if (status != nullptr) {
      *status = s;
    }
    if (fresh != nullptr) {
      *fresh = f;
    }
    return out;
  }
  /// The send at the publishing tag (send_xcorr, d = 0 and d > 0): the
  /// records of one message and its completion number, 0 for no message.
  std::vector<XRecord> send(int64_t time, uint32_t micro, uint64_t * seq)
  {
    std::vector<XRecord> out;
    uint32_t f = 0;
    EXPECT_EQ(pub.emit_remote(0, time, micro, 8, &out, &f, true, seq), EmitStatus::OK);
    EXPECT_EQ(out.empty(), *seq == 0) << "a message without a number, or a number without one";
    return out;
  }
  std::vector<Token> deliver(const std::vector<XRecord> & recs, int64_t pub_time = kTag)
  {
    sub.arrive_remote(0, recs, pub_time, pub_time + kD, 0);
    std::vector<Token> out;
    while (auto tok = sub.pop_released(0, pub_time + kD)) {
      out.push_back(*tok);
      sub.next_wake(0, pub_time + kD);
    }
    return out;
  }
  std::string problems(const Table & t)
  {
    std::string all;
    for (const auto & p : t.reconcile().problems) {
      all += p + "\n";
    }
    return all;
  }
};

// B1: one message per publishing time, cumulative within one microstep, and
// a second microstep of the same time is the D2 collapse.
TEST_F(XFixture, EgressEmitIsCumulativeAndRefusesASecondMicrostep)
{
  publish();
  uint32_t fresh = 0;
  auto first = emit(kTag, 0, nullptr, &fresh);
  ASSERT_EQ(first.size(), 1u);
  EXPECT_EQ(fresh, 1u);
  publish();
  auto second = emit(kTag, 0, nullptr, &fresh);
  ASSERT_EQ(second.size(), 2u) << "LF keeps the last lf_set, so it carries both records";
  EXPECT_EQ(second[0].ordinal, 1u);
  EXPECT_EQ(second[1].ordinal, 2u);
  EXPECT_EQ(fresh, 1u);
  publish();
  EmitStatus st = EmitStatus::OK;
  emit(kTag, 1, &st);
  EXPECT_EQ(st, EmitStatus::COLLAPSE) << "records at (t, 1) after a send at (t, 0)";
  pub.close_epoch();
  EXPECT_NE(problems(pub).find("egress_unsent edge 0: records 3 != sent 2, 1 queued"),
    std::string::npos) << problems(pub);
}

// A relay commits records at several microsteps of time t. One
// flush per time, at t + 1 ns, sends them all in one message.
TEST_F(XFixture, FlushIsDueOncePerTimeAndSendsEveryMicrostep)
{
  EXPECT_FALSE(pub.flush_due(0, kTag)) << "nothing queued";
  publish();
  EXPECT_TRUE(pub.flush_due(0, kTag));
  publish();
  EXPECT_FALSE(pub.flush_due(0, kTag)) << "the flush of this time is already scheduled";
  uint32_t fresh = 0;
  EmitStatus st = EmitStatus::CAP;
  auto out = emit(kTag, 0, &st, &fresh);
  EXPECT_EQ(st, EmitStatus::OK);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(fresh, 2u);
  publish(kTag + kD);
  EXPECT_TRUE(pub.flush_due(0, kTag + kD)) << "a new time needs its own flush";
}

// B_e bounds the records of one publishing reaction, counted at its
// flush_due call. The flush of a relay that publishes once at two microsteps
// sends two records and is within B_e = 1.
TEST(XfedBurst, BurstCountsRecordsPerReactionNotPerFlush)
{
  Table pub{50};
  pub.set_clock([] {return XFixture::kTag;});
  pub.add_edge(0, 0, XFixture::kD, XFixture::kT, -1, 1, 0, 0, Role::EGRESS);
  const Gid g = gid(9);
  pub.register_gid(g);
  uint64_t n = 0;
  auto publish = [&] {
      pub.predict(g, ++n, Origin::ADMITTED, {0}, true, XFixture::kTag);
      pub.commit(g, n, true);
    };
  publish();
  EXPECT_TRUE(pub.flush_due(0, XFixture::kTag));
  publish();
  EXPECT_FALSE(pub.flush_due(0, XFixture::kTag));
  std::vector<XRecord> out;
  uint32_t fresh = 0;
  ASSERT_EQ(pub.emit_remote(0, XFixture::kTag, 0, 8, &out, &fresh), EmitStatus::OK);
  EXPECT_EQ(out.size(), 2u);
  auto problems = [&pub] {
      std::string all;
      for (const auto & p : pub.reconcile().problems) {
        all += p + "\n";
      }
      return all;
    };
  EXPECT_EQ(problems().find("burst_exceeded"), std::string::npos) << problems();
  publish();
  publish();
  EXPECT_TRUE(pub.flush_due(0, XFixture::kTag + XFixture::kD));
  EXPECT_NE(problems().find("burst_exceeded edge 0: 2 records in one reaction > B = 1"),
    std::string::npos) << problems();
}

// A record of another time in the queue means a flush was lost or
// ran late. Sending it with time t would put it on the wrong tag.
TEST_F(XFixture, FlushRefusesARecordOfAnotherTime)
{
  publish(kTag);
  publish(kTag + 1);
  EmitStatus st = EmitStatus::OK;
  auto out = emit(kTag, 0, &st);
  EXPECT_EQ(st, EmitStatus::STALE);
  EXPECT_TRUE(out.empty());
  pub.close_epoch();
  EXPECT_NE(problems(pub).find("egress_unsent edge 0: records 2 != sent 0, 2 queued"),
    std::string::npos) << problems(pub);
}

TEST_F(XFixture, RefusedFlushIsAProblem)
{
  publish();
  ASSERT_TRUE(pub.flush_due(0, kTag));
  pub.flush_refused(0);
  pub.close_epoch();
  EXPECT_NE(problems(pub).find("flush_refused edge 0: 1 flush(es) not scheduled"),
    std::string::npos) << problems(pub);
}

TEST_F(XFixture, EgressEmitAboveTheMessageCapIsRefused)
{
  for (int k = 0; k < 9; ++k) {
    publish();
  }
  EmitStatus st = EmitStatus::OK;
  emit(kTag, 0, &st);
  EXPECT_EQ(st, EmitStatus::CAP);
}

// B2: early sample then record matches, the join hands out that sample.
TEST_F(XFixture, EarlySampleThenRecordIsMatched)
{
  publish();
  sclock = kTag + 1000000;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(11)), SampleOutcome::RETAINED);
  auto toks = deliver(emit());
  ASSERT_EQ(toks.size(), 1u);
  sclock = kTag + kD;
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 11);
  sub.close_epoch();
  pub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
  EXPECT_TRUE(pub.reconcile().ok) << problems(pub);
  EXPECT_NE(pub.reconcile().summary.find("\"egress_records\":1,\"egress_sent\":1"),
    std::string::npos) << pub.reconcile().summary;
}

// Records that arrived after their intended tag on a
// decentralized ingress edge end LOST once, and their sample arriving after
// does not revive them. On-time records are not tardy.
TEST_F(XFixture, TardyRecordsEndLostOnceAndTheirSampleDoesNotReviveThem)
{
  publish();
  publish();
  const auto recs = emit();
  ASSERT_EQ(recs.size(), 2u);
  const int64_t late = kTag + kD + 1000000;
  sclock = late;
  sub.arrive_remote(0, recs, kTag, late, 0);
  sub.terminate_tardy(0, recs);
  std::vector<Token> toks;
  while (auto tok = sub.pop_released(0, late)) {
    toks.push_back(*tok);
    sub.next_wake(0, late);
  }
  ASSERT_EQ(toks.size(), 2u);
  Sample out;
  for (const auto & t : toks) {
    EXPECT_TRUE(sub.is_tardy(t));
    EXPECT_EQ(sub.join(t, milliseconds(0), &out), Outcome::LOST);
  }
  EXPECT_NE(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  sub.close_epoch();
  EXPECT_EQ(sub.counts(0).outcomes[Outcome::LOST], 2u);
  EXPECT_EQ(sub.counts(0).missing, 2u);
}

TEST_F(XFixture, OnTimeRecordsAreNotTardy)
{
  publish();
  const auto toks = deliver(emit());
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_FALSE(sub.is_tardy(toks[0]));
}

// B2: a record whose sample never comes, while the writer's next sample does,
// is LOST (one writer delivers in order).
TEST_F(XFixture, RecordWithoutSampleIsLostWhenALaterSampleArrives)
{
  publish();
  publish();
  auto toks = deliver(emit());
  ASSERT_EQ(toks.size(), 2u);
  sclock = kTag + kD;
  EXPECT_EQ(sub.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::LOST);
  EXPECT_EQ(sub.join(toks[1], milliseconds(0), &out), Outcome::MATCHED);
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
  EXPECT_EQ(sub.counts(0).missing, 1u);
}

// B2: a duplicate record creates no second expectation.
TEST_F(XFixture, DuplicateRecordIsIgnored)
{
  publish();
  auto recs = emit();
  auto toks = deliver(recs);
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_TRUE(deliver(recs, kTag + 1).empty());
  EXPECT_EQ(sub.counts(0).expectations, 1u);
  sclock = kCut + 1;
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::NOT_READY);
  sub.close_epoch();
  EXPECT_NE(sub.reconcile().summary.find("\"duplicate_records\":1"), std::string::npos) <<
    sub.reconcile().summary;
}

// The ingress cutoff counts from the publication tag. The sender's
// activation of tag t started 60 ms late (it stalled), so S(a) = t + 60 ms. A
// sample taken 1 ms past t + d + T is late although it is 59 ms inside the
// removed sender_start window S(a) + d + T.
TEST_F(XFixture, DeadlineCountsFromThePublicationTag)
{
  constexpr int64_t kStall = 60000000;  // S(a) - t, the sender's stall
  const int64_t s_a = kTag + kStall;
  pclock = s_a;
  pub.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag, s_a);
  pub.commit(g, next, true);
  pub.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag, s_a);
  pub.commit(g, next, true);
  auto recs = emit();
  ASSERT_EQ(recs.size(), 2u);
  EXPECT_EQ(recs[0].act, s_a);
  sclock = kCut - 1;  // ordinal 1 on time against t + d + T
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  sclock = kCut + 1000000;  // ordinal 2 late against t + d + T, early against S(a) + d + T
  sub.on_sample(0, g, 2, sample(2));
  auto toks = deliver(recs);
  ASSERT_EQ(toks.size(), 2u);
  EXPECT_EQ(sub.cutoff(toks[0]), kCut) << "expected t + d + T = " << kCut << ", got " <<
    sub.cutoff(toks[0]) << ": the deadline counted from the activation start S(a) + d + T = " <<
    s_a + kD + kT << ", not from the publication tag";
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  const Outcome j2 = sub.join(toks[1], milliseconds(0), &out);
  EXPECT_NE(j2, Outcome::MATCHED) <<
    "a sample taken 1 ms past t + d + T matched: the deadline counted from the activation "
    "start S(a), not from the publication tag";
}

// The ingress deadline keeps the declared clock bound eps,
// D = t + max(0, d) + T + eps.
TEST_F(XFixture, DeadlineKeepsTheClockBound)
{
  constexpr int64_t kEps = 2000000;  // eps = 2 ms, a declared clock offset bound below T
  constexpr int64_t kStall = 60000000;  // S(a) = t + 60 ms
  sub.set_clock_bound(0, kEps);
  pclock = kTag + kStall;
  pub.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag, kTag + kStall);
  pub.commit(g, next, true);
  auto toks = deliver(emit());
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_EQ(sub.cutoff(toks[0]), kCut + kEps) << "expected t + d + T + eps = " << kCut + kEps <<
    ", got " << sub.cutoff(toks[0]) << " (S(a) + d + T + eps is " << kTag + kStall + kD + kT +
    kEps << ")";
}

// COMPLETE numbering, changed 2026-11-03: with d > 0 the k-th message of a
// time lands on (t + d, k). An activation at microstep 0 that published
// nothing sends no message, so the record sent at microstep 1 is the first
// message of kTag and lands on (kTag + d, 0), the stop tag: delivered.
TEST_F(XFixture, IdleActivationTakesNoReceiveSlotWithDelay)
{
  uint64_t seq = 0;
  EXPECT_TRUE(send(kTag, 0, &seq).empty());
  publish();
  ASSERT_EQ(send(kTag, 1, &seq).size(), 1u);
  EXPECT_EQ(seq, 1u);
  pub.close_epoch(kTag + kD, 0);
  const std::string js = pub.reconcile().summary;
  EXPECT_NE(js.find("\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":0"),
    std::string::npos) <<
    "the record message is the first of its time and lands on the stop tag, yet it counted "
    "at stop: the activation that published nothing took a receive slot\n" << js;
  EXPECT_NE(js.find("\"completes_sent\":1,\"completes_at_stop\":0"), std::string::npos) << js;
}

// Late COMPLETE: the sender counts the COMPLETE messages that land past the
// stop tag. d > 0: of kTag the first lands on (kTag + d, 0), the second on
// (kTag + d, 1), and the one of kTag + d on (kTag + 2d, 0). Stop (kTag + d, 0)
// leaves one delivered, two at stop.
TEST_F(XFixture, CompletesAtStopWithDelay)
{
  EXPECT_EQ(pub.complete_seq(0, kTag, 0), 1u);
  EXPECT_EQ(pub.complete_seq(0, kTag, 1), 2u);
  EXPECT_EQ(pub.complete_seq(0, kTag + kD, 0), 3u);
  pub.close_epoch(kTag + kD, 0);
  const std::string js = pub.reconcile().summary;
  EXPECT_NE(js.find("\"completes_sent\":3,\"completes_at_stop\":2"), std::string::npos) <<
    "the receiver can see one COMPLETE by the stop tag (late COMPLETE)\n" << js;
}

// Late COMPLETE, d = 0: a message sent at (t, m) lands on (t, m + 1). With
// stop (kTag, 1) the one of microstep 0 lands on the stop tag, the one of
// microstep 1 past it. Older times landed already.
TEST(CompleteAtStop, ZeroDelayCountsMicrostepsAtTheStopTime)
{
  Table t{52};
  t.add_edge(0, 0, 0, 100000000, -1, 0, 0, 0, Role::EGRESS);
  EXPECT_EQ(t.complete_seq(0, 500, 0), 1u);
  EXPECT_EQ(t.complete_seq(0, 1000, 0), 2u);
  EXPECT_EQ(t.complete_seq(0, 1000, 1), 3u);
  t.close_epoch(1000, 1);
  const std::string js = t.reconcile().summary;
  EXPECT_NE(js.find("\"completes_sent\":3,\"completes_at_stop\":1"), std::string::npos) <<
    "only the COMPLETE of (1000, 1) lands past the stop tag (1000, 1) (late COMPLETE)\n" << js;
}

// Late COMPLETE: the receiver reports the last completion number it saw.
TEST_F(XFixture, IngressReportsItsCompleteHigh)
{
  sub.note_complete(0, 1);
  sub.note_complete(0, 2);
  sub.close_epoch();
  EXPECT_NE(sub.reconcile().summary.find("\"complete_high\":2"), std::string::npos) <<
    sub.reconcile().summary;
}

// Stall watch: the stall watch fires once per episode, only past the bound,
// and a new tag starts a new episode.
TEST(StallWatch, FiresOncePerEpisodePastTheBound)
{
  StallWatch w;
  EXPECT_FALSE(w.check(1000, 10)) << "no dispatch yet, nothing stands still";
  w.progress(5, 0, 100);
  EXPECT_FALSE(w.check(110, 10)) << "exactly the bound is not past it";
  w.progress(5, 0, 105);  // another dispatch at the same tag is no progress
  EXPECT_TRUE(w.check(111, 10)) <<
    "logical time stood still 11 ns past a 10 ns bound and no stall was reported (stall watch)";
  EXPECT_FALSE(w.check(500, 10)) << "one report per episode";
  w.progress(6, 0, 600);
  EXPECT_FALSE(w.check(605, 10));
  EXPECT_TRUE(w.check(611, 10)) << "a new tag starts a new episode";
  EXPECT_FALSE(w.check(1u << 30, -1)) << "a negative bound disables the watch";
}

// A run of microsteps at one time is progress. A watch
// keyed on the time alone would report a stall while the tag still advances.
TEST(StallWatch, MicrostepIsProgress)
{
  StallWatch w;
  w.progress(5, 0, 100);
  w.progress(5, 1, 108);
  w.progress(5, 2, 116);
  EXPECT_FALSE(w.check(120, 10)) <<
    "the microstep advanced 4 ns ago, yet a stall was reported (time-only progress)";
  EXPECT_TRUE(w.check(127, 10));
}

// The watch starts at the negotiated start time, before
// any dispatch (corr_rt::stall_watch_start). An upstream that stops before
// the start tag leaves this federate without a single dispatch, so the start
// itself is the origin of the stall, and the first dispatch at (start, 0)
// does not move it.
TEST(StallWatch, StartTimeIsTheOriginBeforeAnyDispatch)
{
  StallWatch w;
  w.progress(1000, 0, 1000);  // armed at the handshake, start due at 1000
  EXPECT_FALSE(w.check(990, 10)) << "a stall reported before the start time was due";
  EXPECT_FALSE(w.check(1010, 10));
  w.progress(1000, 0, 1005);  // the start tag's own dispatch, late by 5 ns
  EXPECT_TRUE(w.check(1011, 10)) <<
    "the start tag stood still 11 ns from its start time, yet no stall was reported";
  w.progress(1001, 0, 1012);
  EXPECT_FALSE(w.check(1020, 10));
}

// Stall watch: W = P + sum over ingress edges of (d + T) + the lag limit
// (2 * the largest T without max_lag). Egress and local edges add nothing,
// and an infinite T or no timer disables the watch.
TEST(StallWatch, BoundAddsPeriodWindowsAndLag)
{
  Table t{53};
  t.add_edge(0, 0, 5, 100, -1, 0, 0, 0, Role::INGRESS);
  t.add_edge(1, 0, 0, 200, -1, 0, 0, 0, Role::INGRESS);
  t.add_edge(2, 0, 7, 300, -1, 0, 0, 0, Role::EGRESS);
  EXPECT_EQ(t.stall_bound_ns(1000), 1000 + 105 + 200 + 2 * 300);
  t.set_clock_bound(0, 40);
  t.set_clock_bound(2, 900);  // egress: its eps is the receiver's, adds nothing here
  EXPECT_EQ(t.stall_bound_ns(1000), 1000 + 105 + 40 + 200 + 2 * 300) <<
    "the ingress clock bound eps is missing from W";
  EXPECT_EQ(t.stall_bound_ns(0), -1) << "no timer, an idle federate may stand still";
  Table inf{54};
  inf.add_edge(0, 0, 5, -1, -1, 0, 0, 0, Role::INGRESS);
  EXPECT_EQ(inf.stall_bound_ns(1000), -1) << "infinite T: a healthy wait has no bound";
  Table none{55};
  none.add_edge(0, 0, 5, 100, -1, 0, 0, 0, Role::EGRESS);
  EXPECT_EQ(none.stall_bound_ns(1000), -1) << "no ingress edge, no upstream to stall";
}

// The timer-free receiver keeps a stop-due window,
// the bound without the timer period, so its stalled upstream is caught.
TEST(StallWatch, WindowCoversTimerFreeReceiver)
{
  Table t{57};
  t.add_edge(0, 0, 5, 100, -1, 0, 0, 0, Role::INGRESS);
  t.add_edge(1, 0, 0, 200, -1, 0, 0, 0, Role::INGRESS);
  t.set_clock_bound(0, 40);
  EXPECT_EQ(t.stall_bound_ns(0), -1) << "the timer watch stays off without a timer";
  EXPECT_EQ(t.stall_window_ns(), 105 + 40 + 200 + 2 * 200) <<
    "a timer-free receiver has no stop-due window";
  EXPECT_EQ(t.stall_window_ns() + 1000, t.stall_bound_ns(1000)) <<
    "the window is W without the timer period";
  Table inf{58};
  inf.add_edge(0, 0, 5, -1, -1, 0, 0, 0, Role::INGRESS);
  EXPECT_EQ(inf.stall_window_ns(), -1) << "infinite T: a healthy wait has no bound";
  Table none{59};
  none.add_edge(0, 0, 5, 100, -1, 0, 0, 0, Role::EGRESS);
  EXPECT_EQ(none.stall_window_ns(), -1) << "no ingress edge, no upstream to stall";
}

// Stall watch: a stall episode fails the reconcile as a CONTROL_FAILURE.
TEST(StallWatch, StallFailsTheReconcile)
{
  Table t{56};
  t.add_edge(0, 0, 5, 100, -1, 0, 0, 0, Role::INGRESS);
  t.note_stall();
  t.close_epoch();
  const auto r = t.reconcile();
  EXPECT_FALSE(r.ok);
  std::string all;
  for (const auto & p : r.problems) {
    all += p + "\n";
  }
  EXPECT_NE(all.find("control_failure stall"), std::string::npos) << all;
  EXPECT_NE(r.summary.find("\"stalls\":1"), std::string::npos) << r.summary;
}

// Regression (c), changed 2026-11-03: an activation that
// published nothing sends no COMPLETE and takes no number. The receiver sees
// 1 and 2 with no gap, takes the next record at its tag and reconciles clean.
TEST_F(XFixture, IdleActivationSendsNoCompleteAndTheReceiverAdvances)
{
  uint64_t seq = 0;
  publish(kTag);
  auto first = send(kTag, 0, &seq);
  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(seq, 1u);
  EXPECT_TRUE(sub.note_complete(0, seq).control_failures.empty());
  auto toks = deliver(first, kTag);
  ASSERT_EQ(toks.size(), 1u);
  sclock = kTag + 1;
  ASSERT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  const int64_t idle = kTag + kD;  // published nothing
  EXPECT_TRUE(send(idle, 0, &seq).empty()) << "an activation that published nothing sent";
  EXPECT_EQ(seq, 0u) << "an activation that published nothing took a completion number";
  const int64_t later = kTag + 2 * kD;
  publish(later);
  auto third = send(later, 0, &seq);
  ASSERT_EQ(third.size(), 1u);
  const auto res = sub.note_complete(0, seq);
  EXPECT_TRUE(res.control_failures.empty()) <<
    "CONTROL_FAILURE for completion number " <<
    (res.control_failures.empty() ? 0 : res.control_failures[0]) <<
    ": the idle activation took a number but sent no message";
  toks = deliver(third, later);
  ASSERT_EQ(toks.size(), 1u) << "the receiver did not take the next record after the idle tag";
  sclock = later + 1;
  ASSERT_EQ(sub.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  sub.expect_complete_end(0);
  pub.close_epoch();
  sub.note_complete_end(pub.complete_ends().at(0));
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
  EXPECT_NE(sub.reconcile().summary.find("\"completes\":2,\"control_failures\":0,"
    "\"complete_high\":2"), std::string::npos) << sub.reconcile().summary;
}

// Regression (a), changed 2026-11-03: an activation that published sends its COMPLETE
// with the record. The sample never comes in time, so at D = S(a) + d + T
// the join reports it missing, and a sample after D is never handed out.
TEST_F(XFixture, LostSampleOfACompletedActivationIsLostAtItsDeadline)
{
  constexpr int64_t kRun = 60000000;
  const int64_t s_a = kTag;
  pclock = s_a + kRun;
  pub.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag, s_a);
  pub.commit(g, next, true);
  uint64_t seq = 0;
  auto recs = send(kTag, 0, &seq);
  ASSERT_EQ(recs.size(), 1u) <<
    "the activation published, yet its COMPLETE carried no record: no expectation exists "
    "and a lost sample would never be reported";
  EXPECT_EQ(seq, 1u);
  EXPECT_TRUE(sub.note_complete(0, seq).control_failures.empty());
  auto toks = deliver(recs);
  ASSERT_EQ(toks.size(), 1u);
  sclock = kCut + 1;  // 1 ns past D = S(a) + d + T
  EXPECT_NE(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED) <<
    "a sample 1 ns past D = S(a) + d + T was retained for its join";
  Sample out;
  const Outcome o = sub.join(toks[0], milliseconds(0), &out);
  EXPECT_NE(o, Outcome::MATCHED) << "a sample past D = S(a) + d + T was handed out";
  EXPECT_EQ(o, Outcome::NOT_READY) << "the join at D did not report the sample missing";
  EXPECT_EQ(sub.counts(0).missing, 1u) << "the lost sample was not counted missing at D";
  sub.close_epoch();
  EXPECT_EQ(sub.counts(0).missing, 1u) <<
    "after the close the lost sample is not counted missing exactly once";
}

// Regression (b), changed 2026-11-03: with idle activations in between, a COMPLETE
// that never arrives is still a gap in the numbers (CONTROL_FAILURE), and a
// trailing one still fails the end-of-run check.
TEST_F(XFixture, DroppedCompleteIsAGapAndATrailingOneFailsTheEndCheck)
{
  uint64_t seq = 0;
  std::vector<uint64_t> sent;
  for (int i = 0; i < 6; ++i) {
    const int64_t t = kTag + i * kD;
    if (i % 2 == 0) {
      publish(t);  // even ticks publish, odd ticks publish nothing
    }
    send(t, 0, &seq);
    if (seq != 0) {
      sent.push_back(seq);
    }
  }
  ASSERT_EQ(sent, (std::vector<uint64_t>{1, 2, 3}));
  sub.expect_complete_end(0);
  EXPECT_TRUE(sub.note_complete(0, 1).control_failures.empty());
  // number 2 is dropped on the wire
  const auto res = sub.note_complete(0, 3);
  ASSERT_EQ(res.control_failures.size(), 1u) <<
    "a COMPLETE never arrived and no CONTROL_FAILURE was reported";
  EXPECT_EQ(res.control_failures[0], 2u);
  publish(kTag + 6 * kD);
  send(kTag + 6 * kD, 0, &seq);  // number 4 is dropped too, no later one exposes it
  EXPECT_EQ(seq, 4u);
  pub.close_epoch();
  sub.note_complete_end(pub.complete_ends().at(0));
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok) << "a dropped COMPLETE passed the reconcile";
  EXPECT_NE(problems(sub).find("control_failure edge 0: 1 activation(s) without a COMPLETE "
    "record (2)"), std::string::npos) << problems(sub);
  EXPECT_NE(problems(sub).find("control_failure edge 0: 1 trailing activation(s) without a "
    "COMPLETE record (sender numbered 4, 0 past the stop tag, receiver's last 3)"),
    std::string::npos) << "the trailing dropped COMPLETE passed the end check\n" << problems(sub);
}

// The flush (decentralized d > 0, RCLLF_XFED_FLUSH=1) sends
// one message per time at t + 1 ns. It is that time's COMPLETE record: the
// records of every microstep of t share one completion number, so a flushed
// message dropped on the wire is a gap the receiver reports as
// CONTROL_FAILURE, and a dropped last one fails the end-of-run check. As in
// send_xcorr, a message without a completion number is not accounted.
TEST_F(XFixture, DroppedFlushMessageIsAControlFailure)
{
  auto flush = [this](int64_t t) {
      std::vector<XRecord> out;
      uint32_t f = 0;
      uint64_t seq = 0;
      EXPECT_TRUE(pub.flush_due(0, t));
      EXPECT_EQ(pub.emit_remote(0, t, 0, 8, &out, &f, false, &seq), EmitStatus::OK);
      EXPECT_FALSE(out.empty());
      return seq;
    };
  publish(kTag);  // a relay commits at two microsteps of kTag, one flush sends both
  publish(kTag);
  std::vector<uint64_t> sent;
  sent.push_back(flush(kTag));
  for (int i = 1; i < 4; ++i) {
    publish(kTag + i * kD);
    sent.push_back(flush(kTag + i * kD));
  }
  sub.expect_complete_end(0);
  std::vector<uint64_t> failures;
  for (size_t i = 0; i < sent.size(); ++i) {
    if (i == 1 || i == 3 || sent[i] == 0) {
      continue;  // the 2nd and the last flushed message are dropped on the wire
    }
    const auto res = sub.note_complete(0, sent[i]);
    failures.insert(failures.end(), res.control_failures.begin(), res.control_failures.end());
  }
  ASSERT_EQ(failures, (std::vector<uint64_t>{2})) <<
    "a flushed message was dropped and no CONTROL_FAILURE was reported "
    "(completion numbers sent: " << sent[0] << ", " << sent[1] << ", " << sent[2] << ", " <<
    sent[3] << ")";
  EXPECT_EQ(sent, (std::vector<uint64_t>{1, 2, 3, 4}));
  pub.close_epoch();
  sub.note_complete_end(pub.complete_ends().at(0));
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok) << "a dropped flushed message passed the reconcile";
  EXPECT_NE(problems(sub).find("control_failure edge 0: 1 trailing activation(s) without a "
    "COMPLETE record (sender numbered 4, 0 past the stop tag, receiver's last 3)"),
    std::string::npos) << "the trailing dropped flush passed the end check\n" << problems(sub);
}

// A flush of time t sends at (t + 1 ns, 0) over `after d - 1 ns`,
// so it lands on (t + d, 0), the 0-th message of t. Stop (kTag + d, 0): the
// flush of kTag lands on the stop tag, the one of kTag + 1 ms past it.
TEST_F(XFixture, FlushCompletesAtStopCountOneMessagePerTime)
{
  for (const int64_t t : {kTag, kTag + 1000000}) {
    publish(t);
    std::vector<XRecord> out;
    uint32_t f = 0;
    uint64_t seq = 0;
    ASSERT_TRUE(pub.flush_due(0, t));
    ASSERT_EQ(pub.emit_remote(0, t, 0, 8, &out, &f, false, &seq), EmitStatus::OK);
    ASSERT_NE(seq, 0u) << "the flush of time " << t << " carries no completion number";
  }
  pub.close_epoch(kTag + kD, 0);
  const std::string js = pub.reconcile().summary;
  EXPECT_NE(js.find("\"completes_sent\":2,\"completes_at_stop\":1"), std::string::npos) <<
    "only the flush of kTag + 1 ms lands past the stop tag\n" << js;
}

// One sequence number per tag, so two sends at one tag (LF keeps the
// last lf_set) share it and a new microstep or time takes the next one.
TEST_F(XFixture, CompleteSeqCountsDistinctTags)
{
  EXPECT_EQ(pub.complete_seq(0, kTag, 0), 1u);
  EXPECT_EQ(pub.complete_seq(0, kTag, 0), 1u);
  EXPECT_EQ(pub.complete_seq(0, kTag, 1), 2u);
  EXPECT_EQ(pub.complete_seq(0, kTag + kD, 0), 3u);
}

// A reaction that published nothing after a
// record-carrying one at the same tag sets nothing, LF keeps the earlier
// lf_set. A third reaction that publishes again sends both records under the
// tag's one completion number.
TEST_F(XFixture, IdleReactionAtSameTagSendsNothingAndKeepsTheTagCumulative)
{
  uint64_t seq = 0;
  publish();
  ASSERT_EQ(send(kTag, 0, &seq).size(), 1u);
  EXPECT_EQ(seq, 1u);
  EXPECT_TRUE(send(kTag, 0, &seq).empty());
  publish();
  EXPECT_EQ(send(kTag, 0, &seq).size(), 2u) << "LF keeps the last lf_set, it carries both";
  EXPECT_EQ(seq, 1u) << "one tag, one completion number";
  EXPECT_TRUE(send(kTag, 1, &seq).empty());
}

// A sender activation whose COMPLETE never arrives is a
// CONTROL_FAILURE, reported once with its sequence number, and the run
// reconcile fails.
TEST_F(XFixture, MissingCompleteIsControlFailure)
{
  EXPECT_TRUE(sub.note_complete(0, 1).control_failures.empty());
  const auto res = sub.note_complete(0, 3);
  ASSERT_EQ(res.control_failures.size(), 1u);
  EXPECT_EQ(res.control_failures[0], 2u);
  EXPECT_EQ(res.control_failure_count, 1u);
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok);
  EXPECT_NE(problems(sub).find("control_failure edge 0: 1 activation(s) without a COMPLETE "
    "record (2)"), std::string::npos) << problems(sub);
}

// The sender numbered three activations, the
// receiver saw 1 and 2. No later number exposes the gap, so only the sender's
// end-of-run count does, and the receiver's own reconcile fails with it.
TEST_F(XFixture, TrailingMissingCompleteFailsTheReceiversReconcile)
{
  EXPECT_EQ(pub.complete_seq(0, kTag, 0), 1u);
  EXPECT_EQ(pub.complete_seq(0, kTag + kD, 0), 2u);
  EXPECT_EQ(pub.complete_seq(0, kTag + 2 * kD, 0), 3u);
  sub.expect_complete_end(0);
  EXPECT_TRUE(sub.note_complete(0, 1).control_failures.empty());
  EXPECT_TRUE(sub.note_complete(0, 2).control_failures.empty());
  pub.close_epoch();
  const auto ends = pub.complete_ends();
  ASSERT_EQ(ends.size(), 1u);
  EXPECT_EQ(ends[0].due, 3u);
  sub.note_complete_end(ends[0]);
  EXPECT_TRUE(sub.complete_ends_missing().empty());
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok) << "a trailing COMPLETE lost after the last one passed";
  EXPECT_NE(problems(sub).find("control_failure edge 0: 1 trailing activation(s) without a "
    "COMPLETE record (sender numbered 3, 0 past the stop tag, receiver's last 2)"),
    std::string::npos) << problems(sub);
}

// A sender whose end-of-run count never came (crashed, stalled,
// cut off) fails the receiver's reconcile, it never passes silently.
TEST_F(XFixture, MissingCompleteEndFailsTheReconcile)
{
  sub.expect_complete_end(0);
  EXPECT_TRUE(sub.note_complete(0, 1).control_failures.empty());
  ASSERT_EQ(sub.complete_ends_missing().size(), 1u);
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok) << "a missing end-of-run count passed";
  EXPECT_NE(problems(sub).find("complete_end_missing edge 0"), std::string::npos) <<
    problems(sub);
}

// Every number up to the count arrived, the reconcile passes.
TEST_F(XFixture, MatchingCompleteEndReconciles)
{
  EXPECT_EQ(pub.complete_seq(0, kTag, 0), 1u);
  EXPECT_EQ(pub.complete_seq(0, kTag + kD, 0), 2u);
  sub.expect_complete_end(0);
  EXPECT_TRUE(sub.note_complete(0, 1).control_failures.empty());
  EXPECT_TRUE(sub.note_complete(0, 2).control_failures.empty());
  pub.close_epoch();
  sub.note_complete_end(pub.complete_ends().at(0));
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

// A COMPLETE not above the last one is a fault, never a second arrival.
TEST_F(XFixture, DuplicateCompleteIsReported)
{
  EXPECT_FALSE(sub.note_complete(0, 1).duplicate);
  EXPECT_TRUE(sub.note_complete(0, 1).duplicate);
  sub.close_epoch();
  EXPECT_FALSE(sub.reconcile().ok);
  EXPECT_NE(problems(sub).find("duplicate_complete edge 0: 1"), std::string::npos) <<
    problems(sub);
}

// A corrupt huge sequence number lists a bounded number of gaps and
// keeps the exact count, it never loops over the whole range.
TEST_F(XFixture, HugeCompleteGapIsBounded)
{
  const uint64_t seq = uint64_t{1} << 40;
  const auto res = sub.note_complete(0, seq);
  EXPECT_EQ(res.control_failures.size(), 64u);
  EXPECT_EQ(res.control_failure_count, seq - 1);
}

// B3: an unrecorded early sample below the highest record never gets one
// (a refused-origin publication), an unknown writer stays UNRESOLVED.
TEST_F(XFixture, UnrecordedEarlySampleIsUnsupportedAndUnknownWriterUnresolved)
{
  sclock = kTag + 1000000;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  EXPECT_EQ(sub.on_sample(0, gid(77), 1, sample(9)), SampleOutcome::RETAINED);
  XRecord r;
  r.gid = g;
  r.ordinal = 2;
  auto toks = deliver({r});
  ASSERT_EQ(toks.size(), 1u);
  auto c = sub.counts(0);
  EXPECT_EQ(c.samples[SampleOutcome::UNSUPPORTED], 1u);
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::UNSUPPORTED) <<
    "below the highest record, never recorded";
  sclock = kCut + 1;
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::NOT_READY);
  sub.close_epoch();
  c = sub.counts(0);
  EXPECT_EQ(c.samples[SampleOutcome::UNRESOLVED], 1u);
  EXPECT_EQ(c.samples[SampleOutcome::RETAINED], 0u);
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

// B4 (R1-1): a slow receiver stamps a late sample late at intake; the join
// running later still does not match it.
TEST_F(XFixture, SlowReceiverLateSampleIsLateAndMissing)
{
  publish();
  auto toks = deliver(emit());
  sclock = kCut + 1;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::LATE);
  sclock = kCut + 50000000;
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::NOT_READY);
  EXPECT_EQ(sub.counts(0).missing, 1u);
}

// B4: an early sample stamped after the cutoff (the arrive ran late) is
// released LATE when the record comes, and the ID ends missing.
TEST_F(XFixture, EarlySampleStampedPastTheCutoffIsReleasedLate)
{
  publish();
  sclock = kCut + 10;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  auto toks = deliver(emit());
  EXPECT_EQ(sub.retained(), 0u);
  EXPECT_EQ(sub.counts(0).samples[SampleOutcome::LATE], 1u);
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::NOT_READY);
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

// B5 (R1-1): a slow publisher. The activation started 3 T after its
// tag and committed at once, the sample taken 1 ns after the commit. The window
// is t + d + T, so the sample is LATE and the record ends NOT_READY at its
// join: the stall is charged to this edge. The commit lag above L_e is the
// sender's publisher_lag_exceeded report, and the receiver reconciles clean
// (late = lost + reported, no publisher_late counter).
TEST_F(XFixture, SlowPublisherIsLateAtTheReceiver)
{
  pclock = kTag + 3 * kT;  // L_e = 2 T_e
  pub.predict(g, ++next, Origin::ADMITTED, {0}, true, kTag, pclock);
  pub.commit(g, next, true);
  auto recs = emit();
  ASSERT_EQ(recs.size(), 1u);
  sclock = recs[0].phys + 1;
  sub.on_sample(0, g, 1, sample(1));
  auto toks = deliver(recs, kTag);
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_EQ(sub.cutoff(toks[0]), kCut) << "expected t + d + T = " << kCut << ", got " <<
    sub.cutoff(toks[0]) << ": the cutoff counted from the activation start S(a) + d + T = " <<
    kTag + 3 * kT + kD + kT << ", not from the publication tag";
  Sample out;
  EXPECT_NE(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED) <<
    "a sample taken 2 T past t + d + T matched: the cutoff counted from the activation "
    "start, not from the publication tag";
  pub.close_epoch();
  sub.close_epoch();
  EXPECT_NE(problems(pub).find("publisher_lag_exceeded edge 0: 1 commit(s)"),
    std::string::npos) << problems(pub);
  EXPECT_TRUE(sub.reconcile().ok) << "a late sample failed the receiver's reconcile instead of "
    "being reported: " << problems(sub);
  EXPECT_EQ(sub.counts(0).missing, 1u) << "the late sample was not reported missing";
}

// T = 30 ms, L = 60 ms, the activation starts 40 ms after its tag
// (within L) and the sample is taken 2 ms after S(a). The window is
// t + T = t + 30 ms, so the sample, taken at t + 42 ms, is LATE: a start lag
// within L is not on time downstream, L stays a retention and watchdog bound
// only. Both sides reconcile clean, the loss is reported, not refused.
TEST(XfedActivationWindow, ALateStartWithinLIsLateDownstream)
{
  constexpr int64_t kMs = 1000000;
  constexpr int64_t kTag = 1000 * kMs;
  constexpr int64_t kT = 30 * kMs;
  constexpr int64_t kL = 60 * kMs;
  const int64_t s_a = kTag + 40 * kMs;
  int64_t pclock = s_a;
  int64_t sclock = s_a;
  Table pub{70};
  Table sub{71};
  pub.set_clock([&pclock] {return pclock;});
  sub.set_clock([&sclock] {return sclock;});
  pub.add_edge(0, 0, 0, kT, kL, 0, 0, 0, Role::EGRESS);
  sub.add_edge(0, 7, 0, kT, kL, 0, 0, 0, Role::INGRESS);
  const Gid g = gid(9);
  pub.register_gid(g);
  pub.predict(g, 1, Origin::ADMITTED, {0}, true, kTag, s_a);
  pub.commit(g, 1, true);
  std::vector<XRecord> recs;
  uint32_t fresh = 0;
  pub.emit_remote(0, kTag, 0, 8, &recs, &fresh);
  ASSERT_EQ(recs.size(), 1u);
  sclock = s_a + 2 * kMs;
  sub.on_sample(0, g, 1, sample(1));
  sub.arrive_remote(0, recs, kTag, kTag, 0);
  std::vector<Token> toks;
  while (auto tok = sub.pop_released(0, kTag)) {
    toks.push_back(*tok);
    sub.next_wake(0, kTag);
  }
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_EQ(sub.cutoff(toks[0]), kTag + kT) << "expected t + T = " << kTag + kT << ", got " <<
    sub.cutoff(toks[0]) << ": the cutoff counted from the activation start S(a) + T = " <<
    s_a + kT << ", not from the publication tag";
  Sample out;
  EXPECT_NE(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED) <<
    "a sample taken 12 ms past t + T matched: the cutoff counted from the activation start "
    "";
  pub.close_epoch();
  sub.close_epoch();
  EXPECT_TRUE(pub.reconcile().ok && sub.reconcile().ok) <<
    "a tag-anchored late sample failed reconciliation instead of being reported";
  EXPECT_EQ(sub.counts(0).missing, 1u);
}

// Slice item 2: a nonzero logical delay d on an ingress edge.
// The record of tag t is released at t + d, not before, and its cutoff is
// t + d + T. d = 30 ms stands for a modeled actuator delay (/brake_cmd),
// T = 50 ms is the micro system's timeout, S(a) = t + 10 ms a late start.
TEST(XfedActivationWindow, WithDelayReleasesAtTPlusDAndCutsAtTPlusDPlusT)
{
  constexpr int64_t kMs = 1000000;
  constexpr int64_t kTag = 1000 * kMs;
  constexpr int64_t kD = 30 * kMs;
  constexpr int64_t kT = 50 * kMs;
  const int64_t s_a = kTag + 10 * kMs;
  int64_t pclock = s_a;
  int64_t sclock = s_a;
  Table pub{72};
  Table sub{73};
  pub.set_clock([&pclock] {return pclock;});
  sub.set_clock([&sclock] {return sclock;});
  pub.add_edge(0, 0, kD, kT, -1, 0, 0, 0, Role::EGRESS);
  sub.add_edge(0, 7, kD, kT, -1, 0, 0, 0, Role::INGRESS);
  const Gid g = gid(9);
  pub.register_gid(g);
  pub.predict(g, 1, Origin::ADMITTED, {0}, true, kTag, s_a);
  pub.commit(g, 1, true);
  std::vector<XRecord> recs;
  uint32_t fresh = 0;
  pub.emit_remote(0, kTag, 0, 8, &recs, &fresh);
  ASSERT_EQ(recs.size(), 1u);
  sclock = kTag + kD + kT - 1;  // on time against t + d + T
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  sub.arrive_remote(0, recs, kTag, kTag + kD, 0);
  EXPECT_FALSE(sub.pop_released(0, kTag + kD - 1).has_value()) <<
    "the record of tag t was released before t + d";
  auto tok = sub.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value()) << "the record of tag t was not released at t + d";
  EXPECT_EQ(sub.cutoff(*tok), kTag + kD + kT) << "expected t + d + T = " << kTag + kD + kT <<
    ", got " << sub.cutoff(*tok) << " (S(a) + d + T is " << s_a + kD + kT << ")";
  Sample out;
  EXPECT_EQ(sub.join(*tok, milliseconds(0), &out), Outcome::MATCHED);
}

// B7 (R1-4): a sample after its ID ended missing is LATE, never retained as
// early, also once the key left the tombstone ring.
TEST_F(XFixture, SampleAfterItsIdEndedMissingIsLate)
{
  publish();
  auto toks = deliver(emit());
  sclock = kCut + 1;
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::NOT_READY);
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::LATE);
  for (int64_t k = 1; k <= 70; ++k) {  // push ordinal 1 out of the ring
    publish(kTag + k * 1000000);
    for (const auto & tok : deliver(emit(kTag + k * 1000000), kTag + k * 1000000)) {
      sclock = kCut + k * 1000000 + 1;
      EXPECT_EQ(sub.join(tok, milliseconds(0), &out), Outcome::NOT_READY);
    }
  }
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::LATE);
  EXPECT_EQ(sub.retained(), 0u);
}

// B8 (R1-5): the publisher reconcile is cumulative over the epoch.
TEST_F(XFixture, EgressReconcileCountsUnsentRecords)
{
  publish();
  emit(kTag);
  publish(kTag + 1000000);
  pub.close_epoch();
  auto r = pub.reconcile();
  EXPECT_FALSE(r.ok);
  EXPECT_NE(problems(pub).find("egress_unsent edge 0: records 2 != sent 1, 1 queued at close"),
    std::string::npos) << problems(pub);

  Table all{52};
  all.add_edge(0, 0, kD, kT, -1, 0, 0, 0, Role::EGRESS);
  all.register_gid(g);
  all.set_clock([] {return kTag + 3;});
  for (int64_t k = 1; k <= 3; ++k) {
    all.predict(g, k, Origin::ADMITTED, {0}, true, kTag + k);
    all.commit(g, k, true);
    std::vector<XRecord> out;
    uint32_t fresh = 0;
    EXPECT_EQ(all.emit_remote(0, kTag + k, 0, 8, &out, &fresh), EmitStatus::OK);
  }
  all.close_epoch();
  EXPECT_TRUE(all.reconcile().ok) << all.reconcile().summary;
}

// A record committed at the stop tag's time has its flush at
// (stop time + 1 ns, 0), after the stop tag, so LF never runs it. It is
// retained at stop and named in the summary, not an invariant break. SIGINT
// shape: the flush was scheduled and LF cleared it at stop.
TEST_F(XFixture, EgressRecordAtTheStopTimeIsRetainedAtStop)
{
  constexpr int64_t kStop = kTag + 2 * kD;  // the kTag record lands before the stop
  publish();
  ASSERT_TRUE(pub.flush_due(0, kTag));
  emit(kTag);
  publish(kStop);  // the last publishing reaction, at the stop tag
  publish(kStop);
  ASSERT_TRUE(pub.flush_due(0, kStop));
  pub.close_epoch(kStop);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":3,\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":2,"
      "\"flushes_at_stop\":0"),
    std::string::npos) << r.summary;
}

// A record of t < stop time is flushed at t + 1 ns and sent, but
// it lands on t + d. Past the stop tag LF drops it, so the receiver never
// expects it. It is at stop, not sent, and a record landing on the stop time
// itself is sent.
TEST_F(XFixture, SentRecordLandingPastTheStopTagIsAtStop)
{
  constexpr int64_t kStop = kTag + kD;
  publish(kTag);
  ASSERT_TRUE(pub.flush_due(0, kTag));
  emit(kTag);
  publish(kTag + 1);
  ASSERT_TRUE(pub.flush_due(0, kTag + 1));
  publish(kTag + 1);  // a second reaction at the same time, one flush
  EXPECT_FALSE(pub.flush_due(0, kTag + 1));
  EXPECT_EQ(emit(kTag + 1).size(), 2u);
  pub.close_epoch(kStop);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":3,\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":2,"),
    std::string::npos) << r.summary;
}

// An edge with d = 0. `after 0` maps (t, m) to
// (t, m + 1), so each microstep sends its own message and nothing collapses.
struct ZFixture : XFixture
{
  void SetUp() override
  {
    pub.set_clock([this] {return pclock;});
    sub.set_clock([this] {return sclock;});
    pub.add_edge(0, 0, 0, kT, -1, 0, 0, 0, Role::EGRESS);
    sub.add_edge(0, 7, 0, kT, -1, 0, 0, 0, Role::INGRESS);
    pub.register_gid(g);
  }
};

TEST_F(ZFixture, ZeroDelayEgressSendsEachMicrostepItsOwnMessage)
{
  publish();
  EmitStatus st = EmitStatus::OK;
  EXPECT_EQ(emit(kTag, 0, &st).size(), 1u);
  publish();
  auto second = emit(kTag, 1, &st);
  ASSERT_EQ(st, EmitStatus::OK) << "d = 0: (t, 1) is a new message, not a collapse "
    "(status " << static_cast<int>(st) << ")";
  ASSERT_EQ(second.size(), 1u) << "a new microstep starts a new message";
  EXPECT_EQ(second[0].ordinal, 2u);
  publish();
  EXPECT_EQ(emit(kTag, 1, &st).size(), 2u) << "cumulative within one (t, m)";
  // The receiver takes each message at its own time, pub_time + 0.
  sub.arrive_remote(0, second, kTag, kTag, 0);
  ASSERT_TRUE(sub.pop_released(0, kTag).has_value());
  pub.close_epoch();
  EXPECT_TRUE(pub.reconcile().ok) << problems(pub);
  EXPECT_NE(pub.reconcile().summary.find(
      "\"egress_records\":3,\"egress_sent\":3,\"egress_unsent\":0,\"egress_at_stop\":0,"),
    std::string::npos) << pub.reconcile().summary;
}

// A timeout stops at (T, 0). A record sent there lands on
// (T, 1), past the stop tag, so LF drops it: at stop, never sent. A record of
// an earlier time landed before the stop and counts as sent. Same rule as d > 0.
TEST_F(ZFixture, ZeroDelayRecordAtTheStopTagIsAtStop)
{
  constexpr int64_t kStop = kTag + 1000000;
  publish(kTag);
  emit(kTag);
  publish(kStop);
  emit(kStop);
  pub.close_epoch(kStop);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":2,\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":1,"),
    std::string::npos) << "d = 0 stop accounting: " << r.summary;
}

// A stop tag (T, 2), as a requested stop negotiates it. Records
// sent at (T, 0) and (T, 1) land on (T, 1) and (T, 2) and are delivered; the
// one sent at (T, 2) lands past the stop tag.
TEST_F(ZFixture, ZeroDelayStopTagMicrostepDecidesDelivery)
{
  constexpr int64_t kStop = kTag + 1000000;
  for (uint32_t m = 0; m < 3; ++m) {
    publish(kStop);
    emit(kStop, m);
  }
  pub.close_epoch(kStop, 2);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":3,\"egress_sent\":2,\"egress_unsent\":0,\"egress_at_stop\":1,"),
    std::string::npos) << r.summary;
}

// By default the publishing reaction sends over
// `after d` itself, so (t, 1) after a send at (t, 0) is a new message, not
// the D2 collapse. The receiver queues it at (t + d, 1) (reactor-c patch 0002).
TEST_F(XFixture, PubEgressSendsEachMicrostepItsOwnMessage)
{
  std::vector<XRecord> out;
  uint32_t fresh = 0;
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 0, 8, &out, &fresh, true), EmitStatus::OK);
  ASSERT_EQ(out.size(), 1u);
  publish();
  const EmitStatus st = pub.emit_remote(0, kTag, 1, 8, &out, &fresh, true);
  ASSERT_EQ(st, EmitStatus::OK) << "PUB: (t, 1) is a new message, not a collapse "
    "(status " << static_cast<int>(st) << ")";
  ASSERT_EQ(out.size(), 1u) << "a new microstep starts a new message";
  EXPECT_EQ(out[0].ordinal, 2u);
  EXPECT_EQ(fresh, 1u) << "each record is sent once";
  // Both messages carry time t, the receiver takes each at t + d.
  sub.arrive_remote(0, out, kTag, kTag + kD, 0);
  sclock = kTag + kD;
  EXPECT_TRUE(sub.pop_released(0, kTag + kD).has_value());
}

// Messages of time t sent at (t, 0) and (t, 1) land on (t + d, 0) and
// (t + d, 1). For t + d the stop time and a timeout stop tag (t + d, 0), the
// second is past the stop tag: LF drops it, so it is at stop, not sent.
TEST_F(XFixture, PubSecondMessageOfTheStopTimeIsAtStop)
{
  constexpr int64_t kStop = kTag + kD;
  std::vector<XRecord> out;
  uint32_t fresh = 0;
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 0, 8, &out, &fresh, true), EmitStatus::OK);
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 1, 8, &out, &fresh, true), EmitStatus::OK);
  pub.close_epoch(kStop, 0);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":2,\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":1,"),
    std::string::npos) << "PUB stop accounting, stop tag (t + d, 0): " << r.summary;
}

// A send at the stop time itself must not drop
// the history of time stop - d before close_epoch reads it. Messages of time
// t at (t, 0) and (t, 1), then one at (t + d, 0), timeout stop (t + d, 0):
// the second of t lands on (t + d, 1) and the one of t + d on (t + 2d, 0),
// both past the stop tag.
TEST_F(XFixture, PubSendAtTheStopTimeKeepsTheMessagesOfStopMinusD)
{
  constexpr int64_t kStop = kTag + kD;
  std::vector<XRecord> out;
  uint32_t fresh = 0;
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 0, 8, &out, &fresh, true), EmitStatus::OK);
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 1, 8, &out, &fresh, true), EmitStatus::OK);
  publish(kStop);
  ASSERT_EQ(pub.emit_remote(0, kStop, 0, 8, &out, &fresh, true), EmitStatus::OK);
  pub.close_epoch(kStop, 0);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":3,\"egress_sent\":1,\"egress_unsent\":0,\"egress_at_stop\":2,"),
    std::string::npos) << "PUB stop accounting after a send at the stop time: " << r.summary;
}

TEST_F(XFixture, PubBothMessagesLandBeforeAStopTagAtMicrostepOne)
{
  constexpr int64_t kStop = kTag + kD;
  std::vector<XRecord> out;
  uint32_t fresh = 0;
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 0, 8, &out, &fresh, true), EmitStatus::OK);
  publish();
  ASSERT_EQ(pub.emit_remote(0, kTag, 1, 8, &out, &fresh, true), EmitStatus::OK);
  pub.close_epoch(kStop, 1);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":2,\"egress_sent\":2,\"egress_unsent\":0,\"egress_at_stop\":0,"),
    std::string::npos) << "PUB stop accounting, stop tag (t + d, 1): " << r.summary;
}

// Self-stop shape: the publishing reaction requested the stop, so
// lf_schedule refused the flush. A refused flush of the stop time is at stop.
TEST_F(XFixture, RefusedFlushAtTheStopTimeIsRetainedAtStop)
{
  constexpr int64_t kStop = kTag + 1000000;
  publish(kStop);
  ASSERT_TRUE(pub.flush_due(0, kStop));
  pub.flush_refused(0);
  pub.close_epoch(kStop);
  const auto r = pub.reconcile();
  EXPECT_TRUE(r.ok) << problems(pub);
  EXPECT_NE(r.summary.find(
      "\"egress_records\":1,\"egress_sent\":0,\"egress_unsent\":0,\"egress_at_stop\":1,"
      "\"flushes_at_stop\":1"),
    std::string::npos) << r.summary;
}

// A record 1 ns before the stop time had its flush at the stop
// tag's time, which LF runs. Left queued it is a lost flush, and a refused
// flush of that time stays refused.
TEST_F(XFixture, EgressRecordBeforeTheStopTimeStaysUnsent)
{
  constexpr int64_t kStop = kTag + 1000000;
  publish(kStop - 1);
  ASSERT_TRUE(pub.flush_due(0, kStop - 1));
  pub.flush_refused(0);
  publish(kStop);
  pub.close_epoch(kStop);
  EXPECT_FALSE(pub.reconcile().ok);
  const std::string p = problems(pub);
  EXPECT_NE(p.find("egress_unsent edge 0: records 2 != sent 0 + 1 at stop, 1 queued at close"),
    std::string::npos) << p;
  EXPECT_NE(p.find("flush_refused edge 0: 1 flush(es) not scheduled"), std::string::npos) << p;
}

// No reaction runs at a time after the stop time, so a
// record or refused flush later than it means a wrong stop time. It is not
// retained at stop, reconcile reports it.
TEST_F(XFixture, EgressRecordAfterTheStopTimeStaysUnsent)
{
  constexpr int64_t kStop = kTag + 1000000;
  publish(kStop + 1);
  ASSERT_TRUE(pub.flush_due(0, kStop + 1));
  pub.flush_refused(0);
  pub.close_epoch(kStop);
  EXPECT_FALSE(pub.reconcile().ok);
  const std::string p = problems(pub);
  EXPECT_NE(p.find("egress_unsent edge 0: records 1 != sent 0, 1 queued at close"),
    std::string::npos) << p;
  EXPECT_NE(p.find("flush_refused edge 0: 1 flush(es) not scheduled"), std::string::npos) << p;
}

// Overload: the publisher federate keeps its period, the
// subscriber works 15 ms per matched sample every 10 ms. Each DDS sample
// lands 1 ms after its tag, long before the record reaches the subscriber's
// lagging logical time. Every sample was taken 1 ms after its
// publish moment, within T, so time never declares an ID lost however far the
// worker lags. The outstanding work is bounded by the retain cap N_e = 22: a sample
// that finds it full ends OVERFLOW, and the lag above L_e is reported.
TEST_F(XFixture, OverloadedIngressLosesOnlyToTheRetainCapNeverToTime)
{
  constexpr int64_t kP = 10000000;
  constexpr int64_t kWork = 15000000;
  constexpr int kN = 200;
  Table s{53};
  int64_t clock = kTag;
  s.set_clock([&clock] {return clock;});
  s.add_edge(0, 7, kD, kT, -1, 0, 22, 0, Role::INGRESS);
  int64_t now = kTag;  // the subscriber's physical time
  int fed = 0;         // samples handed to the ingress so far
  uint64_t matched = 0;
  uint64_t missing = 0;
  for (int k = 0; k < kN; ++k) {
    const int64_t tag = kTag + k * kP;
    const int64_t at = tag + kD;  // the record's logical arrival
    now = std::max(now, at);
    for (; fed < kN && kTag + fed * kP + 1000000 <= now; ++fed) {
      clock = kTag + fed * kP + 1000000;  // the ingress thread stamps on arrival
      s.on_sample(0, g, static_cast<uint64_t>(fed + 1), sample(fed));
    }
    clock = now;
    XRecord r;
    r.gid = g;
    r.ordinal = static_cast<uint64_t>(k + 1);
    r.phys = tag;  // the publisher keeps its period: it commits at the tag
    r.tag = tag;
    s.arrive_remote(0, {r}, tag, at, 0);
    while (auto tok = s.pop_released(0, at)) {
      s.note_lag(0, at, now - at, "start");
      Sample out;
      const Outcome o = s.join(*tok, milliseconds(0), &out);
      if (o == Outcome::MATCHED) {
        ++matched;
        now += kWork;
        clock = now;
      } else {
        ++missing;
      }
      s.note_lag(0, at, now - at, "complete");
      s.next_wake(0, at);
    }
  }
  s.close_epoch();
  auto c = s.counts(0);
  EXPECT_EQ(matched + missing, static_cast<uint64_t>(kN));
  EXPECT_EQ(c.samples[SampleOutcome::LATE], 0u)
    << "on-time sample(s) declared LATE because the worker lagged";
  EXPECT_EQ(c.missing, c.samples[SampleOutcome::OVERFLOW]) << "every miss is a retain overflow";
  EXPECT_GT(c.missing, 0u) << "the overload must hit the retain cap";
  EXPECT_EQ(c.samples[SampleOutcome::UNSUPPORTED], 0u);
  EXPECT_NE(problems(s).find("worker_lag_exceeded edge 0"), std::string::npos) << problems(s);
}

// The receiver runs eps behind the publisher at most. A sample stamped
// before its tag on the receiver's clock is still before its cutoff after
// stamp + d + T, so it must not be shed until stamp + eps + d + T.
TEST_F(XFixture, EarlySampleIsNotShedBeforeItsCutoffUnderAClockBound)
{
  constexpr int64_t kEps = 2000000;
  constexpr int64_t kTag2 = kTag + kT;
  sub.set_clock_bound(0, kEps);
  publish();
  auto rec1 = emit();
  sclock = kTag - 1000000;  // the receiver's clock is 1 ms behind the publisher's
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  publish(kTag2);
  auto rec2 = emit(kTag2);
  sclock = kCut - 500000;  // past stamp + d + T of ordinal 1, before its cutoff
  EXPECT_EQ(sub.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);
  EXPECT_EQ(sub.retained(), 2u) << "ordinal 1 shed before its cutoff";
  EXPECT_EQ(sub.counts(0).samples[SampleOutcome::LATE], 0u);
  auto toks = deliver(rec1);
  ASSERT_EQ(toks.size(), 1u);
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 1);
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

// The early sample was taken before its cutoff. Time going
// on past stamp + eps + d + T before its record comes changes nothing.
TEST_F(XFixture, EarlySampleStaysPastStampPlusTheClockBound)
{
  constexpr int64_t kEps = 2000000;
  constexpr int64_t kTag2 = kTag + kT;
  sub.set_clock_bound(0, kEps);
  publish();
  auto rec1 = emit();
  sclock = kTag - 1000000;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  publish(kTag2);
  emit(kTag2);
  sclock = kTag - 1000000 + kEps + kD + kT + 1;
  EXPECT_EQ(sub.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);
  EXPECT_EQ(sub.retained(), 2u) << "ordinal 1 shed at stamp + eps + d + T";
  EXPECT_EQ(sub.counts(0).samples[SampleOutcome::LATE], 0u);
  auto toks = deliver(rec1);
  ASSERT_EQ(toks.size(), 1u);
  Sample out;
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 1);
}

// The fed1host t30 idxradar run shape. The camera sample arrived
// 1 ms after its tag, but the receiving worker was blocked in the tracker
// drain until the dropped radar's cutoff, so the camera record was processed
// after the camera cutoff. The sample was taken in time and must match. Before
// The next intake shed it LATE (stamp + d + T passed) and the record then
// declared camera ordinal 20 lost next to the one radar drop.
TEST_F(XFixture, OnTimeEarlySampleMatchesWhenItsRecordIsProcessedPastTheCutoff)
{
  constexpr int64_t kTag2 = kTag + kT;
  publish();
  auto rec1 = emit();
  sclock = kTag + 1000000;
  EXPECT_EQ(sub.on_sample(0, g, 1, sample(1)), SampleOutcome::RETAINED);
  publish(kTag2);
  auto rec2 = emit(kTag2);
  sclock = kCut + 1000001;  // past stamp + d + T of ordinal 1, before the cutoff of 2
  EXPECT_EQ(sub.on_sample(0, g, 2, sample(2)), SampleOutcome::RETAINED);
  EXPECT_EQ(sub.retained(), 2u) << "ordinal 1 shed at the intake of ordinal 2";
  EXPECT_EQ(sub.counts(0).samples[SampleOutcome::LATE], 0u);
  auto toks = deliver(rec1);  // processed at kCut + 1000001, past ordinal 1's cutoff
  ASSERT_EQ(toks.size(), 1u);
  Sample out;
  const Outcome j1 = sub.join(toks[0], milliseconds(0), &out);
  EXPECT_EQ(j1, Outcome::MATCHED) << "the on-time ordinal 1 ended " <<
    outcome_msg(Outcome::MATCHED, j1) << ": declared lost because its record was processed "
    "after the cutoff although its sample was taken before it";
  EXPECT_EQ(out.message ? payload_of(out) : -1, 1);
  toks = deliver(rec2, kTag2);
  ASSERT_EQ(toks.size(), 1u);
  EXPECT_EQ(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 2);
  sub.close_epoch();
  auto c = sub.counts(0);
  EXPECT_EQ(c.missing, 0u);
  EXPECT_EQ(c.samples[SampleOutcome::UNSUPPORTED], 0u);
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

/// A replaced reader's sample of a later publish, taken
/// before commit() made the expectation. The prediction has the publish tag,
/// so the sample is STALE_READER at intake and never matches. Before the fix
/// the stale test needed the expectation: the sample was retained and the
/// record MATCHED with the old reader's copy.
TEST_F(Fixture, AReplacedReadersSampleTakenBeforeCommitIsStale)
{
  t.set_sub_rebind(0);
  t.note_reader_created(0, 100);
  t.note_reader_created(0, 130);
  t.predict(g, 1, Origin::ADMITTED, {0}, true, 140);
  EXPECT_EQ(
    t.on_sample(0, g, 1, sample(7), nullptr, nullptr, 100), SampleOutcome::STALE_READER);
  t.commit(g, 1, true);
  next = 1;
  auto toks = move_all();
  ASSERT_EQ(toks.size(), 1u);
  Sample out;
  EXPECT_NE(t.join(toks[0], milliseconds(0), &out), Outcome::MATCHED)
    << "the record matched with the replaced reader's sample";
  EXPECT_EQ(t.retained(), 0u);
}

/// A record the new reader missed (it had not matched the
/// writer yet) while the replaced reader's copy was STALE_READER ends LOST.
/// The reconcile names it lost_discovery_gap, apart from a network loss.
TEST_F(Fixture, ALostRecordWhoseOnlyCopyWasStaleIsADiscoveryGap)
{
  t.set_sub_rebind(0);
  t.note_reader_created(0, 100);
  t.note_reader_created(0, 130);
  auto pub_at = [&](int64_t tag) {
      t.predict(g, ++next, Origin::ADMITTED, {0}, true, tag);
      t.commit(g, next, true);
      return next;
    };
  const uint64_t k1 = pub_at(140);
  const uint64_t k2 = pub_at(150);
  auto toks = move_all();
  ASSERT_EQ(toks.size(), 2u);
  EXPECT_EQ(
    t.on_sample(0, g, k1, sample(1), nullptr, nullptr, 100), SampleOutcome::STALE_READER);
  EXPECT_EQ(t.on_sample(0, g, k2, sample(2), nullptr, nullptr, 130), SampleOutcome::RETAINED);
  Sample out;
  EXPECT_EQ(t.join(toks[0], milliseconds(0), &out), Outcome::LOST);
  EXPECT_EQ(t.join(toks[1], milliseconds(0), &out), Outcome::MATCHED);
  // A loss the new reader alone saw: LOST, not a discovery gap.
  pub_at(160);
  const uint64_t k4 = pub_at(170);
  toks = move_all();
  ASSERT_EQ(toks.size(), 2u);
  EXPECT_EQ(t.on_sample(0, g, k4, sample(4), nullptr, nullptr, 130), SampleOutcome::RETAINED);
  EXPECT_EQ(t.join(toks[0], milliseconds(0), &out), Outcome::LOST);
  EXPECT_EQ(t.join(toks[1], milliseconds(0), &out), Outcome::MATCHED);
  auto c = t.counts(0);
  EXPECT_EQ(c.outcomes[Outcome::LOST], 2u);
  EXPECT_EQ(c.lost_discovery_gap, 1u);
  t.close_epoch();
  const auto r = t.reconcile();
  EXPECT_TRUE(r.ok);
  EXPECT_NE(r.summary.find("\"lost_discovery_gap\":1"), std::string::npos) << r.summary;
}

/// An ingress edge's early sample from a replaced reader.
/// Its record brings the publish tag, so the sample is STALE_READER then and
/// the record stays open and ends missing. Before the fix the record adopted
/// the old reader's sample and MATCHED.
TEST_F(XFixture, AReplacedReadersEarlySampleIsStaleWhenItsRecordComes)
{
  sub.set_sub_rebind(0);
  sub.note_reader_created(0, kTag - 50);
  sub.note_reader_created(0, kTag - 10);
  publish();
  auto rec = emit();
  sclock = kTag + 1000000;
  EXPECT_EQ(
    sub.on_sample(0, g, 1, sample(1), nullptr, nullptr, kTag - 50), SampleOutcome::RETAINED)
    << "no record yet, so no publish tag";
  auto toks = deliver(rec);
  ASSERT_EQ(toks.size(), 1u);
  Sample out;
  EXPECT_NE(sub.join(toks[0], milliseconds(0), &out), Outcome::MATCHED)
    << "the record adopted the replaced reader's early sample";
  auto c = sub.counts(0);
  EXPECT_EQ(c.samples[SampleOutcome::STALE_READER], 1u);
  EXPECT_EQ(c.samples[SampleOutcome::RETAINED], 0u);
  sub.close_epoch();
  EXPECT_TRUE(sub.reconcile().ok) << problems(sub);
}

// --- Fan-in groups (several publishers into one subscription) -----------------

/// One drain's pops as (member edge, ordinal).
using Pops = std::vector<std::pair<uint32_t, uint64_t>>;

/// A group of two members into one subscription. Edge 0 leads (lowest
/// edge id); publisher `ga` sends on edge 0, `gb` on edge 1. The ranks follow
/// the publisher instance names, so a test can give edge 1 rank 0.
struct GroupFixture : ::testing::Test
{
  static constexpr int64_t kD = 5000000;
  static constexpr int64_t kT = 100000000;
  static constexpr int64_t kTag = 1000000000;
  Table t{60};
  int64_t clock = kTag;
  const Gid ga = gid(0xa);
  const Gid gb = gid(0xb);
  uint64_t na = 0;
  uint64_t nb = 0;
  void make(
    int64_t d = kD, uint32_t rank0 = 0, uint32_t rank1 = 1, Role role = Role::LOCAL,
    uint32_t carrier = 0)
  {
    t.set_clock([this] {return clock;});
    t.add_edge(0, 7, d, kT, -1, 0, 0, carrier, role);
    t.add_edge(1, 7, d, kT, -1, 0, 0, carrier, role);
    ASSERT_TRUE(t.set_fan_in(0, 0, rank0));
    ASSERT_TRUE(t.set_fan_in(1, 0, rank1));
    t.register_gid(ga);
    t.register_gid(gb);
  }
  /// One publication on member `edge` at tag (`time`, `micro`), then its
  /// arrive there. True when the arrive must wake the group's drain.
  bool send(uint32_t edge, int64_t time, uint32_t micro, int64_t d = kD)
  {
    const Gid & g = edge == 0 ? ga : gb;
    uint64_t & n = edge == 0 ? na : nb;
    t.predict(g, ++n, Origin::ADMITTED, {edge}, true, time);
    t.commit(g, n, true);
    return t.arrive(edge, t.in_flight(edge), time + d, micro);
  }
  /// The drain as the runtime runs it: called with the lead, one pop per wake.
  Pops drain(int64_t now)
  {
    Pops out;
    while (auto tok = t.pop_released(0, now)) {
      out.emplace_back(tok->edge, tok->ordinal);
      t.next_wake(0, now);
    }
    return out;
  }
  std::string problems()
  {
    std::string all;
    for (const auto & p : t.reconcile().problems) {
      all += p + "\n";
    }
    return all;
  }
};

TEST_F(GroupFixture, GroupDrainOrdersByRankAtEqualTag)
{
  make(kD, 1, 0);  // edge 1's publisher sorts first
  EXPECT_TRUE(send(0, kTag, 0)) << "the group's first arrive wakes its drain";
  EXPECT_FALSE(send(1, kTag, 0));
  EXPECT_EQ(drain(kTag + kD), (Pops{{1, 1}, {0, 1}}))
    << "equal delivery tags pop in rank order, not in arrival or edge order";
  EXPECT_EQ(t.group_members(0), (std::vector<uint32_t>{0, 1}));
}

// The call sequence of the combined ingress
// arrive. Lead edge 0 is rank 1, so edge id and rank disagree.
TEST_F(GroupFixture, CombinedIngressArriveDrainsRankZeroInlineThenReArms)
{
  make(kD, 1, 0, Role::INGRESS);
  const int64_t at = kTag + kD;
  // One reaction arrives every present member before it drains.
  EXPECT_TRUE(t.arrive_remote(0, {XRecord{ga, 1, kTag, kTag}}, kTag, at, 0));
  EXPECT_FALSE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, at, 0))
    << "the group has one wake: the combined wake is the first member's";
  auto first = t.pop_released(0, at, 0);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->edge, 1u) << "the inline drain at (t + d, 0) pops rank 0, not the lead";
  EXPECT_EQ(t.next_wake(0, at), 0) << "the rest re-arms the drain for (t + d, 1)";
  auto second = t.pop_released(0, at, 1);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->edge, 0u);
  EXPECT_EQ(t.next_wake(0, at), -1);
  EXPECT_EQ(t.order_violations(0), 0u);
}

TEST_F(GroupFixture, PerMemberInlineDrainsRunEdgeOrderAndBreachRank)
{
  // The INLINE sequence (rejected, not generated) on the same group: each
  // member's arrive drains its own record at (t + d, 0) before the next
  // member's arrive runs. Kept as the reason the MERGED drain exists.
  make(kD, 1, 0, Role::INGRESS);
  const int64_t at = kTag + kD;
  EXPECT_TRUE(t.arrive_remote(0, {XRecord{ga, 1, kTag, kTag}}, kTag, at, 0));
  auto first = t.pop_released(0, at, 0);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->edge, 0u);
  EXPECT_EQ(t.next_wake(0, at), -1);
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, at, 0));
  auto second = t.pop_released(0, at, 0);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->edge, 1u);
  EXPECT_EQ(t.order_violations(0), 1u) << "rank 0 popped after rank 1 at one key time";
}

TEST_F(GroupFixture, MicrostepsCollapseForPositiveDelay)
{
  make();
  send(1, kTag, 0);
  send(0, kTag, 1);
  EXPECT_EQ(drain(kTag + kD), (Pops{{0, 1}, {1, 1}}))
    << "after d > 0 both land on (t + d, 0): the rank decides, not the publication microstep";
}

TEST_F(GroupFixture, ZeroDelayOrdersByPublicationTag)
{
  make(0);
  send(1, kTag, 0, 0);
  send(0, kTag, 1, 0);
  EXPECT_EQ(drain(kTag), (Pops{{1, 1}, {0, 1}}))
    << "with d = 0 the release tag is lf_delay_tag(publication tag, 0): (t, 1) before (t, 2)";
}

TEST_F(GroupFixture, ZeroDelayReleasesAtTheNextMicrostep)
{
  // reactor-c's lf_delay_tag((t, m), 0) is (t, m + 1), the
  // tag an `after 0` connection delivers at.
  make(0);
  ASSERT_TRUE(send(0, kTag, 3, 0));
  EXPECT_FALSE(t.pop_released(0, kTag, 3).has_value())
    << "a d = 0 record published at (t, 3) is released at lf_delay_tag((t, 3), 0) = (t, 4), "
    "not at its publication tag";
  EXPECT_EQ(t.next_wake(0, kTag), 0) << "due at a later microstep of t";
  auto tok = t.pop_released(0, kTag, 4);
  ASSERT_TRUE(tok.has_value()) << "released at (t, 4)";
  EXPECT_EQ(tok->ordinal, 1u);
}

TEST_F(GroupFixture, ZeroDelayAtTheLastMicrostepIsNeverDueBeforeItsPublication)
{
  // lf_delay_tag((t, m), 0) wraps m + 1 to 0 at the last microstep (reactor-c
  // core/tag.c); the table saturates there, so the record stays at (t, max).
  constexpr uint32_t kLast = ros2_lf_binding::corr::kLastMicrostep;
  make(0);
  ASSERT_TRUE(send(0, kTag, kLast, 0));
  EXPECT_FALSE(t.pop_released(0, kTag, 0).has_value())
    << "a d = 0 record published at (t, max) came due at (t, 0), before its publication tag";
  EXPECT_TRUE(t.pop_released(0, kTag, kLast).has_value()) << "released at (t, max)";
}

TEST_F(GroupFixture, CrossHostZeroDelayKeyEqualsTheOneProcessKey)
{
  // Member A (rank 1) publishes at (t, 0), member B (rank 0) at
  // (t, 2), over `after 0`. A arrives and pops at (t, 1), B arrives at (t, 3).
  // The cross-host table must key both records as one process does.
  make(0, 1, 0);  // one process: edge 0 is A, edge 1 is B
  Table x{62};    // the receiving federate of the same group
  x.set_clock([this] {return clock;});
  x.add_edge(0, 7, 0, kT, -1, 0, 0, 0, Role::INGRESS);
  x.add_edge(1, 7, 0, kT, -1, 0, 0, 0, Role::INGRESS);
  ASSERT_TRUE(x.set_fan_in(0, 0, 1));
  ASSERT_TRUE(x.set_fan_in(1, 0, 0));
  struct Pub
  {
    uint32_t edge;
    uint32_t micro;
  };
  for (const Pub & p : {Pub{0, 0}, Pub{1, 2}}) {
    send(p.edge, kTag, p.micro, 0);
    const XRecord rec{p.edge == 0 ? ga : gb, 1, kTag, kTag};
    x.arrive_remote(p.edge, {rec}, kTag, kTag, p.micro);
    for (uint32_t m = 0; m <= p.micro; ++m) {
      EXPECT_FALSE(t.pop_released(0, kTag, m).has_value()) << "one process, edge " << p.edge;
      EXPECT_FALSE(x.pop_released(0, kTag, m).has_value())
        << "edge " << p.edge << ": a d = 0 record published at (t, " << p.micro
        << ") came due at (t, " << m << "), before its one-process key (t, " << p.micro + 1
        << ")";
    }
    auto local = t.pop_released(0, kTag, p.micro + 1);
    auto remote = x.pop_released(0, kTag, p.micro + 1);
    ASSERT_TRUE(local.has_value());
    ASSERT_TRUE(remote.has_value()) << "edge " << p.edge << " due at (t, " << p.micro + 1 << ")";
    EXPECT_EQ(remote->edge, local->edge);
  }
  EXPECT_EQ(t.order_violations(0), 0u);
  EXPECT_EQ(x.order_violations(0), t.order_violations(0))
    << "B's cross-host key went back below A's: keyed (t, 1) instead of (t, 3)";
  x.close_epoch();
  for (const auto & p : x.reconcile().problems) {
    EXPECT_EQ(p.find("order_violations"), std::string::npos) << p;
  }
}

// The PUB sender over `after d`, d > 0. One time t, member A
// (`rank_a`) publishes at (t, 0) and (t, 1), member B at (t, 0). One process
// keys all three records (t + d, 0, rank, ordinal) and the drain pops one per
// microstep. Two hosts: A's second message collides on A's port and lands at
// (t + d, 1) (reactor-c patch 0002), the others at (t + d, 0); the merged
// drain files what came at a microstep before it pops. The sender writes the
// publication microstep m, an earlier d > 0 path wrote 0: the key is the same for
// both, because a d > 0 record's release microstep is 0 whatever m.
TEST_F(GroupFixture, CrossHostPositiveDelayKeyEqualsTheOneProcessKey)
{
  for (uint32_t rank_a : {0u, 1u}) {
    for (bool write_m : {true, false}) {
      SCOPED_TRACE(
        "A rank " + std::to_string(rank_a) + (write_m ? ", sender writes m" : ", sender writes 0"));
      Table one{70};
      one.set_clock([this] {return clock;});
      Table x{72};
      x.set_clock([this] {return clock;});
      for (Table * tb : {&one, &x}) {
        const Role role = tb == &x ? Role::INGRESS : Role::LOCAL;
        tb->add_edge(0, 7, kD, kT, -1, 0, 0, 0, role);
        tb->add_edge(1, 7, kD, kT, -1, 0, 0, 0, role);
        ASSERT_TRUE(tb->set_fan_in(0, 0, rank_a));
        ASSERT_TRUE(tb->set_fan_in(1, 0, 1 - rank_a));
      }
      one.register_gid(ga);
      one.register_gid(gb);
      struct Pub
      {
        uint32_t edge;
        uint64_t ordinal;
        uint32_t pub_micro;
        uint32_t arrive_micro;  // two hosts: patch 0002 defers A's second
      };
      const Pub pubs[] = {{0, 1, 0, 0}, {1, 1, 0, 0}, {0, 2, 1, 1}};
      for (const Pub & p : pubs) {
        const Gid & g = p.edge == 0 ? ga : gb;
        one.predict(g, p.ordinal, Origin::ADMITTED, {p.edge}, true, kTag);
        one.commit(g, p.ordinal, true);
        one.arrive(p.edge, one.in_flight(p.edge), kTag + kD, p.pub_micro);
      }
      const int64_t at = kTag + kD;
      Pops local;
      Pops remote;
      for (uint32_t m = 0; m < 3; ++m) {
        for (const Pub & p : pubs) {
          if (p.arrive_micro == m) {
            const XRecord rec{p.edge == 0 ? ga : gb, p.ordinal, kTag, kTag};
            x.arrive_remote(p.edge, {rec}, kTag, at, write_m ? p.pub_micro : 0);
          }
        }
        if (auto tok = one.pop_released(0, at, m)) {
          local.emplace_back(tok->edge, tok->ordinal);
        }
        if (auto tok = x.pop_released(0, at, m)) {
          remote.emplace_back(tok->edge, tok->ordinal);
        }
      }
      const Pops expect = rank_a == 0 ? Pops{{0, 1}, {0, 2}, {1, 1}} : Pops{{1, 1}, {0, 1}, {0, 2}};
      EXPECT_EQ(local, expect) << "one process: (t + d, 0) keys pop in rank, ordinal order";
      EXPECT_EQ(remote, local)
        << "two hosts popped another sequence than one process at (t + d, 0..2)";
      EXPECT_EQ(x.order_violations(0), 0u);
    }
  }
}

TEST_F(GroupFixture, MissingRecordPopsAtItsReleaseTagAcrossQueues)
{
  // The pop keys every member's carrier and terminal queue
  // by (release tag, rank, ordinal). With d = 0 and a carrier of one, A2 is
  // already missing at its arrive and waits in the terminal queue; A4 enters
  // the emptied carrier one microstep later, so A2 is due first.
  make(0, 0, 1, Role::LOCAL, 1);
  auto pop = [&](uint32_t micro) -> uint64_t {
      auto tok = t.pop_released(0, kTag, micro);
      return tok ? tok->ordinal : 0;
    };
  uint32_t member = 99;
  send(0, kTag, 0, 0);  // A1, released at (t, 1)
  ASSERT_EQ(t.on_group_sample(0, ga, 1, sample(1), &member), SampleOutcome::RETAINED);
  for (uint64_t n = 2; n <= 3; ++n) {  // A2 and A3 at (t, 1)
    t.predict(ga, n, Origin::ADMITTED, {0}, true, kTag);
    t.commit(ga, n, true);
  }
  na = 3;
  // A3's sample proves A2 lost before their arrive.
  ASSERT_EQ(t.on_group_sample(0, ga, 3, sample(3), &member), SampleOutcome::RETAINED);
  t.arrive(0, t.in_flight(0), kTag, 1);  // A2 to the terminal queue, A3 rejected (full)
  EXPECT_EQ(pop(1), 1u);
  send(0, kTag, 2, 0);  // A4 to the emptied carrier, released at (t, 3)
  EXPECT_EQ(pop(2), 2u)
    << "A2's terminal entry (t, 2) precedes A4's carrier entry (t, 3) of the same time";
  EXPECT_EQ(pop(2), 0u) << "A4 is not due before (t, 3)";
  EXPECT_EQ(pop(3), 4u);
  EXPECT_EQ(t.order_violations(0), 0u);
}

TEST_F(GroupFixture, CarrierFirstOnAnEqualKey)
{
  // Carrier first when the keys tie. Two writers on edge 0
  // give equal (release tag, rank, ordinal): gc1 is already missing at its
  // arrive and waits in the terminal queue behind the full carrier.
  make(kD, 0, 1, Role::LOCAL, 1);
  const Gid gc = gid(0xc);
  t.register_gid(gc);
  uint32_t member = 99;
  send(0, kTag, 0);  // ga1 to the carrier
  for (uint64_t n = 1; n <= 2; ++n) {
    t.predict(gc, n, Origin::ADMITTED, {0}, true, kTag);
    t.commit(gc, n, true);
  }
  ASSERT_EQ(t.on_group_sample(0, gc, 2, sample(2), &member), SampleOutcome::RETAINED);
  t.arrive(0, t.in_flight(0), kTag + kD, 1);  // gc1 to the terminal queue
  auto first = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->gid, ga) << "equal key (t + d, 0, rank 0, ordinal 1): the carrier pops first";
  auto second = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->gid, gc);
}

TEST_F(GroupFixture, OneWakePerGroup)
{
  make();
  EXPECT_TRUE(send(0, kTag, 0));
  EXPECT_FALSE(send(1, kTag, 0)) << "the group's wake is pending, a member's arrive adds none";
  EXPECT_EQ(drain(kTag + kD).size(), 2u);
  EXPECT_EQ(t.next_wake(0, kTag + kD), -1) << "both carriers drained, the wake is clear";
  EXPECT_TRUE(send(1, kTag + kD, 0)) << "the next arrive of any member wakes the drain again";
}

TEST_F(GroupFixture, MissingRecordKeepsItsPosition)
{
  make();
  send(0, kTag, 0);       // A1
  send(1, kTag, 0);       // B1, its sample never comes
  send(0, kTag + kD, 0);  // A2, one release later
  uint32_t member = 99;
  EXPECT_EQ(t.on_group_sample(0, ga, 1, sample(1), &member), SampleOutcome::RETAINED);
  EXPECT_EQ(t.on_group_sample(0, ga, 2, sample(2), &member), SampleOutcome::RETAINED);
  EXPECT_EQ(drain(kTag + 2 * kD), (Pops{{0, 1}, {1, 1}, {0, 2}}))
    << "a record without its sample is dispatched at its key, not after the ready ones";
}

TEST_F(GroupFixture, LocalSampleRoutesByWriterGid)
{
  make();
  send(1, kTag, 0);
  uint32_t member = 99;
  EXPECT_EQ(t.on_group_sample(0, gb, 1, sample(3), &member), SampleOutcome::RETAINED)
    << "the group's one reader took gb's sample, and gb publishes on edge 1";
  EXPECT_EQ(member, 1u);
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  ASSERT_TRUE(t.sample_ready(*tok));
  Sample out;
  EXPECT_EQ(t.join(*tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 3);
  EXPECT_EQ(t.counts(1).samples_taken, 1u);
  EXPECT_EQ(t.counts(0).samples_taken, 0u);
}

TEST_F(GroupFixture, EarlyPoolRoutesByGid)
{
  make(kD, 0, 1, Role::INGRESS);
  clock = kTag + 1000000;
  uint32_t member = 99;
  const SampleOutcome early = t.on_group_sample(0, gb, 1, sample(7), &member);
  const uint32_t early_member = member;
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(tok->edge, 1u);
  ASSERT_TRUE(t.sample_ready(*tok))
    << "the pooled sample moved to edge 1 when edge 1's record named its writer";
  Sample out;
  EXPECT_EQ(t.join(*tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(payload_of(out), 7);
  EXPECT_EQ(early, SampleOutcome::POOLED) << "no member knew writer gb when its sample came";
  EXPECT_EQ(early_member, 0u);
  EXPECT_EQ(t.counts(1).samples_taken, 1u) << "counted on its member once routed";
  EXPECT_EQ(t.counts(0).samples_taken, 0u);
  // Edge 1 knows gb now: gb's next sample goes there without the pool.
  EXPECT_EQ(t.on_group_sample(0, gb, 2, sample(8), &member), SampleOutcome::RETAINED);
  EXPECT_EQ(member, 1u);
  t.close_epoch();
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, PooledSampleTracedOnItsMemberWithItsTakeStamp)
{
  struct PoolExits : ros2_lf_binding::corr::Observer
  {
    struct Exit
    {
      uint32_t edge;
      Gid gid;
      uint64_t ordinal;
      SampleOutcome outcome;
      uint64_t retained_id;
      int64_t stamp;
    };
    std::vector<Exit> exits;
    void on_expectation(const Token &) override {}
    void on_terminal(uint32_t, const Gid &, uint64_t, Outcome) override {}
    void on_pool_exit(
      uint32_t edge, const Gid & g, uint64_t ordinal, SampleOutcome o, uint64_t retained_id,
      int64_t stamp) override
    {
      exits.push_back({edge, g, ordinal, o, retained_id, stamp});
    }
  } log;
  make(kD, 0, 1, Role::INGRESS);
  t.set_observer(&log);
  const int64_t taken = kTag + 1000000;
  clock = taken;
  uint32_t member = 99;
  ASSERT_EQ(t.on_group_sample(0, gb, 1, sample(7), &member), SampleOutcome::POOLED);
  EXPECT_TRUE(log.exits.empty()) << "a pooled sample is traced when it leaves the pool";
  clock = kTag + kD;  // edge 1's record names writer gb later
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  ASSERT_EQ(log.exits.size(), 1u) << "routing the pooled sample traces it once";
  EXPECT_EQ(log.exits[0].edge, 1u) << "traced on its member edge once routed, not on the lead";
  EXPECT_EQ(log.exits[0].gid, gb);
  EXPECT_EQ(log.exits[0].ordinal, 1u);
  EXPECT_EQ(log.exits[0].outcome, SampleOutcome::RETAINED);
  EXPECT_NE(log.exits[0].retained_id, 0u) << "a RETAINED exit names its retained sample";
  EXPECT_EQ(log.exits[0].stamp, taken) << "the take stamp, not the time it was routed";
  t.set_observer(nullptr);
}

TEST_F(GroupFixture, PooledSampleIsAdoptedWhenItsRecordComesPastTheCutoff)
{
  // A recorded one-host run under CPU load (cpu16)
  // took the first sample of each radar before any record named its writer,
  // and the records came later than d + T. The samples were taken in time,
  // so the record that names the writer adopts the pooled sample and the ID
  // matches.
  make(kD, 0, 1, Role::INGRESS);
  clock = kTag + 1000000;
  uint32_t member = 99;
  ASSERT_EQ(t.on_group_sample(0, gb, 1, sample(7), &member), SampleOutcome::POOLED);
  clock = kTag + 1000000 + kD + kT + 2000000;
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  EXPECT_EQ(tok->edge, 1u);
  EXPECT_TRUE(t.sample_ready(*tok)) << "the pooled sample was shed before its record";
  Sample out;
  EXPECT_EQ(t.join(*tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(out.message ? payload_of(out) : -1, 7);
  t.close_epoch();
  EXPECT_EQ(t.counts(1).missing, 0u);
  EXPECT_EQ(t.counts(1).samples[SampleOutcome::LATE], 0u);
  EXPECT_EQ(t.counts(1).samples_taken, 1u) << "the sample is counted on its member";
  EXPECT_EQ(t.counts(0).samples_taken, 0u) << "nothing stays on the lead: gb was named";
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::UNRESOLVED], 0u);
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, TwoPooledSamplesOfOneWriterAreBothAdopted)
{
  // The radar period equals d + T, so the second sample of a
  // writer comes before any record named it. Both wait in the pool and both
  // move to the member when the records name the writer.
  make(kD, 0, 1, Role::INGRESS);
  clock = kTag + 1000000;
  uint32_t member = 99;
  ASSERT_EQ(t.on_group_sample(0, gb, 1, sample(7), &member), SampleOutcome::POOLED);
  clock = kTag + 1000000 + kD + kT + 1000000;
  ASSERT_EQ(t.on_group_sample(0, gb, 2, sample(8), &member), SampleOutcome::POOLED)
    << "gb is still unnamed: the second sample waits too";
  const int64_t tag2 = kTag + kD + kT;
  t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0);
  t.arrive_remote(1, {XRecord{gb, 2, tag2, tag2}}, tag2, tag2 + kD, 0);
  t.close_epoch();
  EXPECT_EQ(t.counts(1).samples[SampleOutcome::LATE], 0u)
    << "gb's first sample was shed by its second one before any record named gb";
  EXPECT_EQ(t.counts(1).samples_taken, 2u);
  EXPECT_EQ(t.counts(0).samples_taken, 0u);
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::UNRESOLVED], 0u);
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, PooledSampleOfAWriterNeverNamedIsUnresolvedAtClose)
{
  // A pooled sample whose writer no member ever names stays in
  // the pool (bounded by the pool cap, never by time) and ends UNRESOLVED on
  // the lead at close.
  make(kD, 0, 1, Role::INGRESS);
  clock = kTag + 1000000;
  uint32_t member = 99;
  ASSERT_EQ(t.on_group_sample(0, gid(0xc), 1, sample(9), &member), SampleOutcome::POOLED);
  clock = kTag + 1000000 + kD + kT + 1000000;
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  EXPECT_EQ(t.retained(), 1u) << "the pooled sample was dropped by time";
  t.close_epoch();
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::UNRESOLVED], 1u) << "no member named 0xc";
  EXPECT_EQ(t.counts(0).samples_taken, 1u);
  EXPECT_EQ(t.counts(1).samples_taken, 0u);
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, UnclaimedPooledSampleIsUnresolvedAtClose)
{
  make(kD, 0, 1, Role::INGRESS);
  uint32_t member = 99;
  EXPECT_EQ(t.on_group_sample(0, gid(0xc), 1, sample(9), &member), SampleOutcome::POOLED);
  EXPECT_EQ(t.retained(), 1u);
  t.close_epoch();
  EXPECT_EQ(t.retained(), 0u);
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::UNRESOLVED], 1u)
    << "no member ever named writer 0xc, as for an early sample of one edge";
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, EarlySampleReconcilesOnItsMember)
{
  // The table resolves a sample's member under its lock and
  // counts it there. Only a sample no member resolves stays on the lead, as
  // UNRESOLVED; per edge the taken samples equal the classified ones.
  make(kD, 0, 1, Role::INGRESS);
  clock = kTag + 1000000;
  uint32_t member = 99;
  ASSERT_EQ(t.on_group_sample(0, gb, 1, sample(7), &member), SampleOutcome::POOLED);
  EXPECT_EQ(t.on_group_sample(0, gb, 1, sample(7), &member), SampleOutcome::UNRESOLVED)
    << "a second take of a parked (gid, ordinal) resolves no member either";
  ASSERT_EQ(t.on_group_sample(0, gid(0xc), 1, sample(9), &member), SampleOutcome::POOLED);
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  Sample out;
  EXPECT_EQ(t.join(*tok, milliseconds(0), &out), Outcome::MATCHED);
  t.close_epoch();
  for (uint32_t e : {0u, 1u}) {
    const ros2_lf_binding::corr::EdgeCounts c = t.counts(e);
    uint64_t classified = 0;
    for (const auto & kv : c.samples) {
      classified += kv.second;
    }
    EXPECT_EQ(c.samples_taken, classified) << "edge " << e;
  }
  EXPECT_EQ(t.counts(1).samples_taken, 1u) << "gb's early sample is counted on its member";
  for (const auto & kv : t.counts(0).samples) {
    if (kv.second != 0) {
      EXPECT_EQ(kv.first, SampleOutcome::UNRESOLVED)
        << "only UNRESOLVED stays on the lead, found " <<
        ros2_lf_binding::corr::sample_outcome_name(kv.first);
    }
  }
  EXPECT_EQ(t.counts(0).samples_taken, 2u) << "the second take and writer 0xc";
  EXPECT_TRUE(t.reconcile().ok) << problems();
}

TEST_F(GroupFixture, UnownedWriterIsUnresolvedOnTheLead)
{
  // A writer the table knows on another edge and on no
  // member is not classified on the lead (UNSUPPORTED there): no member
  // resolves it.
  make();
  t.add_edge(2, 8, kD, kT);  // a lone edge into another subscriber
  const Gid gc = gid(0xc);
  t.register_gid(gc);
  t.predict(gc, 1, Origin::ADMITTED, {2}, true, kTag);
  t.commit(gc, 1, true);
  uint32_t member = 99;
  EXPECT_EQ(t.on_group_sample(0, gc, 1, sample(5), &member), SampleOutcome::UNRESOLVED)
    << "no member of group 0 publishes as writer 0xc";
  EXPECT_EQ(member, 0u);
  EXPECT_EQ(t.counts(0).samples_taken, 1u);
  EXPECT_EQ(t.counts(2).samples_taken, 0u);
}

TEST_F(GroupFixture, DropHookNamesTheResolvedMember)
{
  // The F3 hook `<member>:<ordinal>` runs after the table
  // resolved the member; the lead's reader took gb's sample for edge 1.
  make();
  send(0, kTag, 0);
  send(1, kTag, 0);
  t.set_drop_sample_for_test(1, 1);
  uint32_t member = 99;
  bool dropped = false;
  t.on_group_sample(0, ga, 1, sample(1), &member, nullptr, nullptr, std::nullopt, &dropped);
  EXPECT_FALSE(dropped) << "ordinal 1 of member 0 is not the hook's";
  t.on_group_sample(0, gb, 1, sample(2), &member, nullptr, nullptr, std::nullopt, &dropped);
  EXPECT_TRUE(dropped) << "1:1 names gb's sample on member 1, taken by the lead's reader";
  EXPECT_EQ(member, 1u);
  EXPECT_EQ(t.counts(1).samples_taken, 0u) << "a dropped sample never reached the table";
}

TEST_F(GroupFixture, DropHookRunsAtAdoption)
{
  // A pooled sample meets the hook when its member adopts it.
  make(kD, 0, 1, Role::INGRESS);
  t.set_drop_sample_for_test(1, 1);
  clock = kTag + 1000000;
  uint32_t member = 99;
  bool dropped = false;
  ASSERT_EQ(
    t.on_group_sample(0, gb, 1, sample(7), &member, nullptr, nullptr, std::nullopt, &dropped),
    SampleOutcome::POOLED);
  EXPECT_FALSE(dropped) << "no member knows gb yet: the sample waits in the pool";
  EXPECT_TRUE(t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0));
  EXPECT_EQ(t.retained(), 0u) << "edge 1 adopted gb's sample, and the hook 1:1 dropped it";
  EXPECT_EQ(t.counts(1).samples_taken, 0u);
  auto tok = t.pop_released(0, kTag + kD);
  ASSERT_TRUE(tok.has_value());
  EXPECT_FALSE(t.sample_ready(*tok));
}

TEST_F(GroupFixture, FullTableLeavesAnUnknownWriterUnresolvedOnTheLead)
{
  // With the process's retained samples at their bound and
  // nothing in the group's early pool to make room, a sample of a writer no
  // member knows is UNRESOLVED on the lead, not an OVERFLOW there.
  t.set_clock([this] {return clock;});
  t.add_edge(0, 7, kD, kT, -1, 0, 1, 0, Role::INGRESS);
  t.add_edge(1, 7, kD, kT, -1, 0, 1, 0, Role::INGRESS);
  t.add_edge(2, 8, kD, -1, -1, 0, 254, 0);  // 254 + 1 + 1 = the 256 process bound
  ASSERT_TRUE(t.set_fan_in(0, 0, 0));
  ASSERT_TRUE(t.set_fan_in(1, 0, 1));
  const Gid gc = gid(0xc);
  for (const Gid & g : {ga, gb, gc}) {
    t.register_gid(g);
  }
  for (uint64_t k = 1; k <= 254; ++k) {
    t.predict(gc, k, Origin::ADMITTED, {2}, true, kTag);
    t.commit(gc, k, true);
    ASSERT_EQ(t.on_sample(2, gc, k, sample(1)), SampleOutcome::RETAINED) << "sample " << k;
  }
  uint32_t member = 99;
  t.arrive_remote(0, {XRecord{ga, 1, kTag, kTag}}, kTag, kTag + kD, 0);  // true: a drain wake
  t.arrive_remote(1, {XRecord{gb, 1, kTag, kTag}}, kTag, kTag + kD, 0);
  ASSERT_EQ(t.on_group_sample(0, ga, 1, sample(2), &member), SampleOutcome::RETAINED);
  ASSERT_EQ(t.on_group_sample(0, gb, 1, sample(3), &member), SampleOutcome::RETAINED);
  ASSERT_EQ(t.retained(), 256u);
  EXPECT_EQ(t.on_group_sample(0, gid(0xd), 1, sample(4), &member), SampleOutcome::UNRESOLVED)
    << "the table is full of other samples and the pool holds no victim";
  EXPECT_EQ(member, 0u);
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::OVERFLOW], 0u)
    << "only UNRESOLVED stays on the lead";
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::UNRESOLVED], 1u);
}

TEST_F(GroupFixture, OrderViolationCounted)
{
  make();
  send(0, kTag + kD, 0);  // A1, released at kTag + 2 kD
  EXPECT_EQ(drain(kTag + 2 * kD), (Pops{{0, 1}}));
  // A premise breach: B's record of an earlier release arrives after A's pop.
  // The drain still delivers it, and counts the key that went back.
  EXPECT_TRUE(send(1, kTag, 0));
  EXPECT_EQ(drain(kTag + 2 * kD), (Pops{{1, 1}}));
  EXPECT_EQ(t.order_violations(0), 1u);
  t.close_epoch();
  EXPECT_NE(problems().find("order_violations group 0: 1"), std::string::npos) << problems();
}

TEST(GroupRoles, GroupRejectsMixedRoles)
{
  Table t{61};
  t.add_edge(0, 7, 5000000, -1, -1, 0, 0, 0, Role::LOCAL);
  t.add_edge(1, 7, 5000000, -1, -1, 0, 0, 0, Role::INGRESS);
  std::string why;
  EXPECT_TRUE(t.set_fan_in(0, 0, 0, &why)) << why;
  EXPECT_FALSE(t.set_fan_in(1, 0, 1, &why)) << "a group is all local or all ingress (P5)";
  EXPECT_NE(why.find("edge 1"), std::string::npos) << why;
  EXPECT_NE(why.find("edge 0"), std::string::npos) << why;
  EXPECT_EQ(t.group_members(0), (std::vector<uint32_t>{0}));
}

// --- Fast-mode federate, early sample -------------------------------------------

TEST(CorrFast, EarlySampleSurvivesAPhysicallyLateRecordInFastMode)
{
  // R2 brake_fusion, two federates on one host, `-f true` (WU runs T023350Z,
  // T023355Z, T023441Z). Fast mode skips the one-second start wait, so the
  // controller runs tag 0 and publishes /brake_cmd ordinal 1 about 27 ms of
  // physical time before the simulator's federate executes tag 0; the
  // simulator's ingress thread (alive since load) takes the sample in. The
  // record arrives at the drain tag start + 5 ms, which the simulator reaches
  // at once in fast mode: more than d + T = 21 ms of physical time after the
  // intake. The table judged the gap on the physical clock
  // (shed_early_locked), released the sample LATE, and the join then waited
  // for the cutoff start + 21 ms and ended TIMED_OUT (late:1, missing:1).
  // In fast mode the table must judge on the receiver's logical tag: the
  // record is at start + 5 ms, before the cutoff, and the sample was in the
  // table before the record.
  constexpr int64_t kStart = 1793241232677377065;  // simulator.jsonl "start"
  constexpr int64_t kDelay = 5000000;              // d_e
  constexpr int64_t kTimeout = 16000000;           // T_e
  constexpr int64_t kCutoff = kStart + kDelay + kTimeout;
  int64_t clock = 0;  // the receiver's tag before its start tag
  const Gid g = gid(1);
  Table x{56u};
  x.set_fast([] {return true;});
  x.set_clock([&clock] {return clock;});
  x.add_edge(
    0, 7, kDelay, kTimeout, 32000000, 2, 165, 0, ros2_lf_binding::corr::Role::INGRESS);
  ASSERT_EQ(x.on_sample(0, g, 1, sample(9)), SampleOutcome::RETAINED);
  clock = kStart + kDelay;  // the simulator drains /brake_cmd at start + 5 ms
  // The publisher's physical commit: one second (DELAY_START) and the 27 ms
  // startup skew before the receiver's start tag.
  std::vector<ros2_lf_binding::corr::XRecord> rec{{g, 1, kStart - 1027000000, kStart}};
  ASSERT_TRUE(x.arrive_remote(0, rec, kStart, clock, 0));
  const auto late = [&x] {
      const auto c = x.counts(0);
      const auto it = c.samples.find(SampleOutcome::LATE);
      return it == c.samples.end() ? 0u : it->second;
    };
  EXPECT_EQ(late(), 0u)
    << "fast mode: the early /brake_cmd sample was shed LATE when its record arrived, "
    "because the record came more than d + T = 21 ms later on the physical clock, "
    "although the receiver's logical tag start + 5 ms is before the cutoff start + 21 ms";
  auto tok = x.pop_released(0, clock);
  ASSERT_TRUE(tok.has_value());
  Sample out;
  const Outcome j = x.join(*tok, milliseconds(0), &out);
  EXPECT_EQ(j, Outcome::MATCHED)
    << "fast mode: the join ended " << outcome_msg(Outcome::MATCHED, j)
    << " at the cutoff start + 21 ms although the sample was in the table before its record";
  EXPECT_EQ(out.message ? payload_of(out) : -1, 9);
  EXPECT_EQ(x.counts(0).missing, 0u) << "the sample was not consumed: a missing outcome";
  EXPECT_LT(clock, kCutoff);
  x.close_epoch();
  auto r = x.reconcile();
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
}

// Controlled shutdown. A join waiting below the stop tag ends within the
// stop's join bound as EPOCH_CLOSE and is counted pending at the stop, not
// missing; a sample that lands before that deadline still matches; a sample
// still in flight for an expectation that already ended missing is taken late
// by the stop drain.
TEST_F(Fixture, AJoinPendingAtTheStopEndsPendingNotMissing)
{
  uint64_t k = publish();
  auto toks = move_all();
  std::vector<Table::PendingJoin> pending;
  std::thread stopper([&]() {
      std::this_thread::sleep_for(milliseconds(50));
      pending = t.note_stop_requested(200 * 1000000);  // the join bound of the stop
    });
  Sample out;
  const auto t0 = std::chrono::steady_clock::now();
  const Outcome j = t.join(toks.at(0), milliseconds(2000), &out);
  const auto took = std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - t0);
  stopper.join();
  EXPECT_EQ(j, Outcome::EPOCH_CLOSE)
    << "a join pending at the stop was counted " << static_cast<int>(j)
    << " (TIMED_OUT = missing) instead of EPOCH_CLOSE (pending at the stop)";
  EXPECT_LT(took.count(), 1500) << "the join did not end within the stop's join bound";
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_EQ(pending.at(0).edge, 0u);
  EXPECT_EQ(pending.at(0).ordinal, k);
  const auto c = t.counts(0);
  EXPECT_EQ(c.missing, 0u) << "a sample still in flight at the stop was counted missing";
  EXPECT_EQ(c.pending_at_stop, 1u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_NE(r.summary.find("\"pending_at_stop\":1"), std::string::npos) << r.summary;
  EXPECT_NE(r.summary.find("\"owed_at_close\":1"), std::string::npos) << r.summary;
}

TEST_F(Fixture, ASampleLandingBeforeTheStopDeadlineStillMatches)
{
  uint64_t k = publish();
  auto toks = move_all();
  std::thread stopper([&]() {
      std::this_thread::sleep_for(milliseconds(20));
      t.note_stop_requested(1000 * 1000000);
      std::this_thread::sleep_for(milliseconds(50));
      t.on_sample(0, g, k, sample(9));
    });
  Sample out;
  EXPECT_EQ(t.join(toks.at(0), milliseconds(2000), &out), Outcome::MATCHED);
  stopper.join();
  EXPECT_EQ(payload_of(out), 9);
  EXPECT_EQ(t.counts(0).pending_at_stop, 0u);
  EXPECT_EQ(t.counts(0).missing, 0u);
}

// The campaign path: an expectation with a cutoff that passed before the join
// (the program lagged) ends NOT_READY and missing; its sample is still in
// flight, so the stop drain waits for it and takes it late (fake clock: the
// drain's bound is real time, the cutoff is table time).
TEST_F(CutoffFixture, TheStopDrainTakesASampleStillInFlightLate)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  clock = kCutoff + 1;
  Token tok = arrive_one();
  Sample out;
  ASSERT_EQ(t.join(tok, milliseconds(0), &out), Outcome::NOT_READY);
  ASSERT_EQ(t.counts(0).missing, 1u);
  std::thread ingress([&]() {
      std::this_thread::sleep_for(milliseconds(50));
      t.on_sample(0, g, k, sample(5));
    });
  const auto t0 = std::chrono::steady_clock::now();
  const size_t owed = t.drain_in_flight(2000 * 1000000);
  const auto took = std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - t0);
  ingress.join();
  EXPECT_EQ(owed, 0u) << "the drain did not see the sample land";
  EXPECT_LT(took.count(), 1500) << "the drain waited past the sample";
  EXPECT_EQ(t.counts(0).samples[SampleOutcome::LATE], 1u);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_NE(r.summary.find("\"owed_at_close\":0"), std::string::npos) << r.summary;
}

TEST_F(CutoffFixture, TheStopDrainEndsAtItsBoundWhenTheSampleNeverComes)
{
  uint64_t k = predict();
  t.commit(g, k, true);
  clock = kCutoff + 1;
  Token tok = arrive_one();
  Sample out;
  ASSERT_EQ(t.join(tok, milliseconds(0), &out), Outcome::NOT_READY);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(t.drain_in_flight(100 * 1000000), 1u);
  const auto took = std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - t0);
  EXPECT_GE(took.count(), 90);
  EXPECT_LT(took.count(), 1000);
  t.close_epoch();
  auto r = t.reconcile();
  EXPECT_NE(r.summary.find("\"owed_at_close\":1"), std::string::npos) << r.summary;
}

// A same-tag edge releases its d = 0 record at the publication tag
// (t, m); a plain d = 0 edge keeps (t, m + 1).
TEST(CorrSameTag, ZeroDelayRecordReleasesAtItsPublicationTag)
{
  Table t{61};
  t.add_edge(0, 7, 0);  // same-tag
  t.add_edge(1, 8, 0);  // plain d = 0
  ASSERT_TRUE(t.set_same_tag(0));
  EXPECT_TRUE(t.same_tag(0));
  EXPECT_FALSE(t.same_tag(1));
  EXPECT_EQ(t.release_microstep(0, 3), 3u) << "same-tag key must be the publication microstep";
  EXPECT_EQ(t.release_microstep(1, 3), 4u) << "plain d = 0 key stays (t, m + 1)";
  const Gid g = gid(1);
  t.register_gid(g);
  t.predict(g, 1, Origin::ADMITTED, {0, 1});
  t.commit(g, 1, true);
  ASSERT_TRUE(t.arrive(0, 1, 1000, 3));
  ASSERT_TRUE(t.arrive(1, 1, 1000, 3));
  EXPECT_TRUE(t.pop_released(0, 1000, 3).has_value()) <<
    "same-tag record not due at its publication tag (1000, 3), it waits for (1000, 4)";
  EXPECT_FALSE(t.pop_released(1, 1000, 3).has_value()) << "plain d = 0 record due at (t, m)";
  EXPECT_TRUE(t.pop_released(1, 1000, 4).has_value());
}

TEST(CorrSameTag, RefusedOnSelfDelayedDeferredAndRemoteEdges)
{
  Table t{62};
  t.add_edge(0, 7, 0);
  t.add_edge(1, 8, 1000000);  // d > 0
  t.add_edge(2, 9, 0, 100000000);
  t.add_edge(3, 10, 0, -1, -1, 0, 0, 0, ros2_lf_binding::corr::Role::INGRESS);
  t.add_edge(4, 11, 0, -1, -1, 0, 0, 0, ros2_lf_binding::corr::Role::EGRESS);
  t.set_self_local(0);
  ASSERT_TRUE(t.set_deferred(2));
  for (uint32_t e = 0; e < 5; ++e) {
    EXPECT_FALSE(t.set_same_tag(e)) << "edge " << e << " accepted same-tag";
    EXPECT_FALSE(t.same_tag(e)) << "edge " << e;
  }
  EXPECT_FALSE(t.set_same_tag(9)) << "no such edge";
}

// A centralized d = 0 ingress runs over a plain
// zero-delay federated connection, so its records release at the publication
// tag (t, m); an `after 0` ingress keeps (t, m + 1).
TEST(CorrZeroLink, ZeroDelayIngressReleasesAtThePublicationTag)
{
  using ros2_lf_binding::corr::Role;
  Table t{63};
  t.add_edge(0, 7, 0, -1, -1, 0, 0, 0, Role::INGRESS);  // zero-delay link
  t.add_edge(1, 8, 0, -1, -1, 0, 0, 0, Role::INGRESS);  // `after 0` link
  ASSERT_TRUE(t.set_zero_link(0));
  EXPECT_TRUE(t.zero_link(0));
  EXPECT_FALSE(t.zero_link(1));
  EXPECT_EQ(t.release_microstep(0, 3), 3u) << "zero-link key must be the publication microstep";
  EXPECT_EQ(t.release_microstep(1, 3), 4u) << "an `after 0` key stays (t, m + 1)";
}

TEST(CorrZeroLink, RefusedOnLocalEgressAndDelayedEdges)
{
  using ros2_lf_binding::corr::Role;
  Table t{64};
  t.add_edge(0, 7, 0);  // local
  t.add_edge(1, 8, 0, -1, -1, 0, 0, 0, Role::EGRESS);
  t.add_edge(2, 9, 1000000, -1, -1, 0, 0, 0, Role::INGRESS);  // d > 0
  for (uint32_t e = 0; e < 3; ++e) {
    EXPECT_FALSE(t.set_zero_link(e)) << "edge " << e << " accepted a zero link";
    EXPECT_FALSE(t.zero_link(e)) << "edge " << e;
  }
  EXPECT_FALSE(t.set_zero_link(9)) << "no such edge";
}

// ── Batch A2: history and listing limits ─────────────────

// A local writer's history pruned the oldest publication once
// 1024 newer ones existed, whatever its state. A sample of a publication whose
// expectation was still open, inside its cutoff, then read LATE and the
// expectation ended missing. The history now keeps every publication with an
// open expectation and prunes the ended ones around it.
TEST_F(CutoffFixture, AnOpenExpectationKeepsItsPublicationPastTheWriterHistory)
{
  const uint64_t k1 = predict();  // edge 0, cutoff kCutoff
  t.commit(g, k1, true);
  Token tok = arrive_one();
  // 1100 later publications of the same writer: more than the 1024 kept.
  // They name no edge, so none of them holds an expectation.
  for (int i = 0; i < 1100; ++i) {
    t.predict(g, ++next, Origin::ADMITTED, {}, false, kTag);
    t.commit(g, next, true);
  }
  clock = kCutoff - 1;  // inside the cutoff of publication 1
  SampleOutcome o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) <<
    "a sample of a still open expectation, 1 ns inside its cutoff, after 1100 later "
    "publications of its writer, was counted " << ros2_lf_binding::corr::sample_outcome_name(o) <<
    " (the writer history dropped its publication)";
  Sample out;
  EXPECT_EQ(t.join(tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(t.counts(0).missing, 0u);
}

// The oldest open publication stopped the pruning, so
// one open publication at the head of the history (a lost sample, its
// expectation still open) pinned every later ended publication and the
// history grew with the commit count. The ended ones around an open one are
// now pruned: the history stays at the newest kWriterHistory ordinals plus
// the older publications still open (here: one).
TEST_F(CutoffFixture, AnOpenHeadPublicationDoesNotPinTheEndedHistory)
{
  const uint64_t k1 = predict();  // edge 0: its expectation stays open
  t.commit(g, k1, true);
  Token tok = arrive_one();
  constexpr uint64_t kEnded = 10 * ros2_lf_binding::corr::kWriterHistory;
  size_t worst = 0;
  for (uint64_t i = 0; i < kEnded; ++i) {
    t.predict(g, ++next, Origin::ADMITTED, {}, false, kTag);  // no edge: ended at commit
    t.commit(g, next, true);
    worst = std::max(worst, t.writer_history(g));
  }
  EXPECT_LE(worst, ros2_lf_binding::corr::kWriterHistory + 1) <<
    "one open publication at the head pinned the writer history: it held " << worst <<
    " publications after " << kEnded << " ended commits (bound kWriterHistory + 1 open = " <<
    ros2_lf_binding::corr::kWriterHistory + 1 << ")";
  // Ordinal 5 ended and was pruned above the open head (the floor cannot pass
  // the open publication): a stray sample of it still reads LATE.
  SampleOutcome o = t.on_sample(0, g, 5, sample(5));
  EXPECT_EQ(o, SampleOutcome::LATE) <<
    "a stray sample of pruned publication 5 above the open head read " <<
    ros2_lf_binding::corr::sample_outcome_name(o);
  // The open head still resolves.
  clock = kCutoff - 1;
  o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::RETAINED) << ros2_lf_binding::corr::sample_outcome_name(o);
  Sample out;
  EXPECT_EQ(t.join(tok, milliseconds(0), &out), Outcome::MATCHED);
  EXPECT_EQ(t.counts(0).missing, 0u);
  // Once it ended, the next sweep of the older publications (at most
  // max(kHeldRecheck, held) commits later) prunes it too.
  for (size_t i = 0; i < ros2_lf_binding::corr::kHeldRecheck; ++i) {
    t.predict(g, ++next, Origin::ADMITTED, {}, false, kTag);
    t.commit(g, next, true);
  }
  EXPECT_EQ(t.writer_history(g), ros2_lf_binding::corr::kWriterHistory);
  o = t.on_sample(0, g, k1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << ros2_lf_binding::corr::sample_outcome_name(o);
}

// `first + kWriterHistory < assigned` kept
// kWriterHistory + 1 ended publications (ordinals 1..1025 all kept). The
// history now keeps exactly the newest kWriterHistory.
TEST_F(CutoffFixture, TheWriterHistoryKeepsExactlyKWriterHistoryEndedPublications)
{
  constexpr uint64_t kKeep = ros2_lf_binding::corr::kWriterHistory;
  for (uint64_t i = 0; i < kKeep; ++i) {
    t.predict(g, ++next, Origin::ADMITTED, {}, false, kTag);
    t.commit(g, next, true);
  }
  EXPECT_EQ(t.writer_history(g), kKeep);
  SampleOutcome o = t.on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::UNSUPPORTED) << "ordinal 1 of " << kKeep << " pruned: " <<
    ros2_lf_binding::corr::sample_outcome_name(o);
  t.predict(g, ++next, Origin::ADMITTED, {}, false, kTag);
  t.commit(g, next, true);
  EXPECT_EQ(t.writer_history(g), kKeep) <<
    "after " << kKeep + 1 << " ended publications the writer history holds " <<
    t.writer_history(g) << ", not " << kKeep;
  o = t.on_sample(0, g, 1, sample(1));
  EXPECT_EQ(o, SampleOutcome::LATE) << "ordinal 1, outside the newest " << kKeep << ", read " <<
    ros2_lf_binding::corr::sample_outcome_name(o);
  o = t.on_sample(0, g, 2, sample(2));
  EXPECT_EQ(o, SampleOutcome::UNSUPPORTED) << ros2_lf_binding::corr::sample_outcome_name(o);
}

// A remote writer's record history kept the newest 1024
// recorded ordinals and took every older ordinal as recorded, so a sample of
// an old ordinal that never had a record (a refused origin) read LATE instead
// of UNSUPPORTED. The history is now exact for the whole run.
TEST_F(XFixture, AnOldOrdinalWithoutARecordStaysUnsupportedAfterManyRecords)
{
  constexpr uint64_t kN = 1100;  // more than the old 1024 kept ordinals
  constexpr uint64_t kGap = 5;   // the publication that made no record
  std::vector<XRecord> recs;
  for (uint64_t o = 1; o <= kN; ++o) {
    if (o != kGap) {
      recs.push_back(XRecord{g, o, kTag, kTag});
    }
  }
  deliver(recs);
  sclock = kCut + 1;
  SampleOutcome o = sub.on_sample(0, g, kGap, sample(5));
  EXPECT_EQ(o, SampleOutcome::UNSUPPORTED)
    << "ordinal " << kGap << " never had a record but is reported " <<
    ros2_lf_binding::corr::sample_outcome_name(o) << " after " << kN - 1 << " records";
  o = sub.on_sample(0, g, 3, sample(3));
  EXPECT_EQ(o, SampleOutcome::LATE) << "ordinal 3 had a record and its cutoff passed";
  o = sub.on_sample(0, g, kN + 1, sample(6));  // above the highest record: early
  EXPECT_EQ(o, SampleOutcome::RETAINED);
}

// The reconcile JSON names at most 64 missing keys per edge and
// gave no sign that the list was cut. It now says so next to missing_total.
TEST_F(XFixture, ReconcileMarksATruncatedMissingList)
{
  constexpr uint64_t kN = 100;  // above the 64 keys listed
  sclock = kCut + 1;  // past every cutoff, no sample ever came
  Sample out;
  size_t joined = 0;
  for (uint64_t o = 1; o <= kN; o += 4) {  // the records of one tag, four at a time
    std::vector<XRecord> recs;
    for (uint64_t k = o; k < o + 4; ++k) {
      recs.push_back(XRecord{g, k, kTag, kTag});
    }
    for (const auto & tok : deliver(recs)) {
      EXPECT_NE(sub.join(tok, milliseconds(0), &out), Outcome::MATCHED);
      ++joined;
    }
  }
  ASSERT_EQ(joined, kN);
  sub.close_epoch();
  ASSERT_EQ(sub.counts(0).missing, kN);
  const std::string js = sub.reconcile().summary;
  size_t listed = 0;
  for (size_t at = js.find("\"ordinal\":"); at != std::string::npos;
    at = js.find("\"ordinal\":", at + 1))
  {
    ++listed;
  }
  EXPECT_EQ(listed, 64u) << js;
  EXPECT_NE(js.find("\"missing_total\":100,"), std::string::npos) << js;
  EXPECT_NE(js.find("\"missing_truncated\":true"), std::string::npos) <<
    "the reconcile summary lists " << listed << " of " << kN <<
    " missing IDs and does not say the list is truncated: " << js;
  EXPECT_NE(js.find("\"pending_truncated\":false"), std::string::npos) << js;
}

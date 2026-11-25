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

// Emit layer of the rcllf provider with a fake sink (T2).

#include <gtest/gtest.h>

#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "semantic_trace.hpp"

namespace st = ros2_lf_binding::semtrace;

namespace
{

struct Rec
{
  std::string event;
  uint64_t epoch;
  uint64_t seq;
};

class FakeSink : public st::Sink
{
public:
  std::vector<Rec> recs;
  std::map<std::string, bool> disabled;
  rcllf_tp_run_end last_end{};
#define FAKE_EV(ev, catv) \
  bool enabled_ ## ev() override {return !disabled[#ev]; \
  } \
  void emit_ ## ev(uint64_t epoch, uint64_t seq, const rcllf_tp_ ## ev & e) override \
  { \
    recs.push_back({#ev, epoch, seq}); \
    keep(e); \
  }
  RCLLF_TRACE_EVENTS(FAKE_EV)
#undef FAKE_EV
  template<typename T>
  void keep(const T &) {}
  void keep(const rcllf_tp_run_end & e) {last_end = e;}

  size_t count(const std::string & ev) const
  {
    size_t n = 0;
    for (const auto & r : recs) {n += r.event == ev;}
    return n;
  }
};

class SemanticTrace : public ::testing::Test
{
protected:
  void SetUp() override
  {
    unsetenv("RCLLF_TEST_TRACE_DROP_SEQ");
    unsetenv("RCLLF_TEST_TRACE_SKIP_RUN_END");
    st::reset_for_test();
    st::set_sink_for_test(&sink);
  }
  void TearDown() override
  {
    st::set_sink_for_test(nullptr);
    st::reset_for_test();
  }
  FakeSink sink;
};

rcllf_tp_publication pub(uint64_t ordinal)
{
  rcllf_tp_publication p{};
  p.ordinal = ordinal;
  return p;
}

}  // namespace

TEST_F(SemanticTrace, NothingIsTracedBeforeBegin)
{
  EXPECT_FALSE(st::publication(pub(1)));
  EXPECT_TRUE(sink.recs.empty());
  EXPECT_EQ(st::attempts(RCLLF_CAT_PUBLICATION), 0u);
}

TEST_F(SemanticTrace, SeqAdvancesPerCategoryOnlyWhenEnabled)
{
  st::begin(42);
  EXPECT_TRUE(st::publication(pub(1)));
  EXPECT_TRUE(st::publication(pub(2)));
  EXPECT_TRUE(st::sample(rcllf_tp_sample{}));
  sink.disabled["publication"] = true;
  EXPECT_FALSE(st::publication(pub(3)));
  sink.disabled["publication"] = false;
  EXPECT_TRUE(st::publication(pub(4)));
  ASSERT_EQ(sink.recs.size(), 4u);
  EXPECT_EQ(sink.recs[0].seq, 1u);
  EXPECT_EQ(sink.recs[1].seq, 2u);
  EXPECT_EQ(sink.recs[2].event, "sample");
  EXPECT_EQ(sink.recs[2].seq, 1u);
  EXPECT_EQ(sink.recs[3].seq, 3u) << "a disabled event must not take a seq";
  for (const auto & r : sink.recs) {
    EXPECT_EQ(r.epoch, 42u);
                                                            }
}

TEST_F(SemanticTrace, SnapshotTotalsEqualEmittedCounts)
{
  st::begin(7);
  for (int i = 0; i < 5; ++i) {
    st::publication(pub(i + 1));
                                                           }
  for (int i = 0; i < 3; ++i) {
    st::dispatch_begin(rcllf_tp_dispatch_begin{});
                                                                             }
  st::Snapshot snap{};
  ASSERT_TRUE(st::close(&snap));
  EXPECT_EQ(snap.totals[RCLLF_CAT_PUBLICATION], sink.count("publication"));
  EXPECT_EQ(snap.totals[RCLLF_CAT_DISPATCH], sink.count("dispatch_begin"));
  EXPECT_EQ(snap.totals[RCLLF_CAT_SAMPLE], 0u);
}

TEST_F(SemanticTrace, DisabledEventHasZeroTotalButNonzeroAttempts)
{
  sink.disabled["publication"] = true;
  st::begin(7);
  for (int i = 0; i < 4; ++i) {
    st::publication(pub(i + 1));
                                                           }
  st::Snapshot snap{};
  ASSERT_TRUE(st::close(&snap));
  EXPECT_EQ(snap.totals[RCLLF_CAT_PUBLICATION], 0u);
  EXPECT_EQ(snap.attempts[RCLLF_CAT_PUBLICATION], 4u);
  EXPECT_EQ(sink.count("publication"), 0u);
}

TEST_F(SemanticTrace, EventAfterCloseTakesNoSeqAndEmitsNothing)
{
  st::begin(7);
  st::publication(pub(1));
  ASSERT_TRUE(st::close(nullptr));
  EXPECT_FALSE(st::publication(pub(2)));
  EXPECT_FALSE(st::cleanup(rcllf_tp_cleanup{}));
  EXPECT_EQ(st::total(RCLLF_CAT_PUBLICATION), 1u);
  EXPECT_EQ(sink.count("publication"), 1u);
  EXPECT_EQ(sink.count("cleanup"), 0u);
  EXPECT_FALSE(st::close(nullptr)) << "close is one-shot";
}

TEST_F(SemanticTrace, RunEndIsEmittedExactlyOnceWithTheSnapshot)
{
  st::begin(9);
  st::publication(pub(1));
  st::publication(pub(2));
  EXPECT_FALSE(st::close_run_end(rcllf_tp_run_end{})) << "run_end before close";
  ASSERT_TRUE(st::close(nullptr));
  st::close_publisher(rcllf_tp_run_end_publisher{});
  st::close_edge(rcllf_tp_run_end_edge{});
  EXPECT_TRUE(st::close_run_end(rcllf_tp_run_end{}));
  EXPECT_FALSE(st::close_run_end(rcllf_tp_run_end{}));
  st::close_publisher(rcllf_tp_run_end_publisher{});
  EXPECT_EQ(sink.count("run_end"), 1u);
  EXPECT_EQ(sink.count("run_end_publisher"), 1u) << "nothing after run_end";
  EXPECT_EQ(sink.recs.back().event, "run_end");
  EXPECT_EQ(sink.recs.back().seq, 0u);
  EXPECT_EQ(sink.last_end.totals[RCLLF_CAT_PUBLICATION], 2u);
}

TEST_F(SemanticTrace, DropSeqHookLeavesAGap)
{
  setenv("RCLLF_TEST_TRACE_DROP_SEQ", "publication:2", 1);
  st::begin(3);
  for (int i = 0; i < 3; ++i) {
    st::publication(pub(i + 1));
                                                           }
  ASSERT_EQ(sink.recs.size(), 2u);
  EXPECT_EQ(sink.recs[0].seq, 1u);
  EXPECT_EQ(sink.recs[1].seq, 3u);
  EXPECT_EQ(st::total(RCLLF_CAT_PUBLICATION), 3u);
}

TEST_F(SemanticTrace, SkipRunEndHookSuppressesRunEnd)
{
  setenv("RCLLF_TEST_TRACE_SKIP_RUN_END", "1", 1);
  st::begin(3);
  ASSERT_TRUE(st::close(nullptr));
  EXPECT_TRUE(st::close_run_end(rcllf_tp_run_end{}));
  EXPECT_EQ(sink.count("run_end"), 0u);
}

TEST_F(SemanticTrace, NoopSinkEmitsNothing)
{
  st::set_sink_for_test(&st::noop_sink());
  st::begin(1);
  EXPECT_FALSE(st::publication(pub(1)));
  EXPECT_EQ(st::total(RCLLF_CAT_PUBLICATION), 0u);
  EXPECT_EQ(st::attempts(RCLLF_CAT_PUBLICATION), 1u);
  EXPECT_TRUE(sink.recs.empty());
}

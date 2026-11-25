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

/// @file fake_trace_sink.hpp
/// @brief Recording rcllf sink for the binding's unit tests.
#ifndef FAKE_TRACE_SINK_HPP_
#define FAKE_TRACE_SINK_HPP_

#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "semantic_trace.hpp"

namespace rcllf_test
{

struct Rec
{
  std::string event;
  uint64_t epoch = 0;
  uint64_t seq = 0;
  uint64_t ordinal = 0;
  uint8_t origin = 0;
  uint8_t result = 0;
  uint8_t outcome = 0;
  uint32_t edge = 0;
  uint64_t dispatch_id = 0;
  uint64_t retained_id = 0;
  uint64_t last_ordinal = 0;
  int64_t lf_start_time_ns = 0;
  int64_t cutoff = 0;
  uint8_t missing_hook = 0;
  uint8_t op = 0;       ///< carrier: emit, arrive, drain or refused
  uint32_t count = 0;   ///< carrier: records moved
  int32_t rid = 0;           ///< actuation_late
  std::string topic;         ///< actuation_late
  int64_t tag_time = 0;      ///< actuation_late
  int64_t physical_ns = 0;   ///< actuation_late
  int64_t lateness_ns = 0;   ///< actuation_late
  int64_t bound_ns = 0;      ///< actuation_late
};

class FakeSink : public ros2_lf_binding::semtrace::Sink
{
public:
  std::vector<Rec> recs;
  std::map<std::string, bool> disabled;
  rcllf_tp_run_end last_end{};
  std::vector<rcllf_tp_run_end_edge> end_edges;
  std::vector<rcllf_tp_run_end_publisher> end_pubs;
  std::mutex mu;
#define FAKE_EV(ev, catv) \
  bool enabled_ ## ev() override {std::lock_guard<std::mutex> lk(mu); return !disabled[#ev]; \
  } \
  void emit_ ## ev(uint64_t epoch, uint64_t seq, const rcllf_tp_ ## ev & e) override \
  { \
    std::lock_guard<std::mutex> lk(mu); \
    Rec r; \
    r.event = #ev; \
    r.epoch = epoch; \
    r.seq = seq; \
    fill(r, e); \
    recs.push_back(r); \
  }
  RCLLF_TRACE_EVENTS(FAKE_EV)
#undef FAKE_EV

  size_t count(const std::string & ev, int origin = -1)
  {
    std::lock_guard<std::mutex> lk(mu);
    size_t n = 0;
    for (const auto & r : recs) {
      n += r.event == ev && (origin < 0 || r.origin == origin);
    }
    return n;
  }
  std::vector<Rec> of(const std::string & ev)
  {
    std::lock_guard<std::mutex> lk(mu);
    std::vector<Rec> out;
    for (const auto & r : recs) {
      if (r.event == ev) {out.push_back(r);}
    }
    return out;
  }

private:
  template<typename T>
  void fill(Rec &, const T &) {}
  void fill(Rec & r, const rcllf_tp_publication & e)
  {
    r.ordinal = e.ordinal;
    r.origin = e.origin;
    r.result = e.result;
    r.dispatch_id = e.dispatch_id;
  }
  void fill(Rec & r, const rcllf_tp_expectation & e) {r.ordinal = e.ordinal; r.edge = e.edge;}
  void fill(Rec & r, const rcllf_tp_terminal & e)
  {
    r.ordinal = e.ordinal;
    r.edge = e.edge;
    r.outcome = e.outcome;
  }
  void fill(Rec & r, const rcllf_tp_sample & e)
  {
    r.ordinal = e.ordinal;
    r.outcome = e.outcome;
    r.retained_id = e.retained_id;
    r.edge = e.edge;
  }
  void fill(Rec & r, const rcllf_tp_carrier & e)
  {
    r.edge = e.edge;
    r.op = e.op;
    r.count = e.count;
  }
  void fill(Rec & r, const rcllf_tp_dispatch_begin & e)
  {
    r.ordinal = e.ordinal;
    r.edge = e.edge;
    r.dispatch_id = e.dispatch_id;
  }
  void fill(Rec & r, const rcllf_tp_dispatch_end & e)
  {
    r.outcome = e.outcome;
    r.dispatch_id = e.dispatch_id;
    r.retained_id = e.retained_id;
    r.cutoff = e.cutoff;
    r.missing_hook = e.missing_hook;
  }
  void fill(Rec & r, const rcllf_tp_backlog & e) {r.last_ordinal = e.last_ordinal;}
  void fill(Rec & r, const rcllf_tp_lf_start & e) {r.lf_start_time_ns = e.lf_start_time_ns;}
  void fill(Rec & r, const rcllf_tp_actuation_late & e)
  {
    r.rid = e.rid;
    r.topic = e.topic ? e.topic : "";
    r.tag_time = e.tag_time;
    r.physical_ns = e.physical_ns;
    r.lateness_ns = e.lateness_ns;
    r.bound_ns = e.bound_ns;
  }
  void fill(Rec &, const rcllf_tp_run_end & e) {last_end = e;}
  void fill(Rec &, const rcllf_tp_run_end_edge & e) {end_edges.push_back(e);}
  void fill(Rec &, const rcllf_tp_run_end_publisher & e) {end_pubs.push_back(e);}
};

}  // namespace rcllf_test

#endif  // FAKE_TRACE_SINK_HPP_

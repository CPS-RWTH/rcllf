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

/// @file semantic_trace.hpp
/// @brief The one emit layer of the `rcllf` provider (D1/D2).
///
/// Producers fill an `rcllf_tp_<event>` struct and call the matching
/// function below. The layer takes the per-(epoch, category) seq only when the
/// sink reports the event enabled, the trace has begun and is not closed. The
/// hot path takes no lock and allocates nothing: atomics only, so the gate
/// hooks may call it under a slot mutex and from a nested publish.
#ifndef SEMANTIC_TRACE_HPP_
#define SEMANTIC_TRACE_HPP_

#include <cstdint>

#include "rcllf_tp_structs.h"  // NOLINT(build/include_subdir)

namespace ros2_lf_binding
{
namespace semtrace
{

/// Where events go: the LTTng provider, a no-op, or a test fake.
class Sink
{
public:
  virtual ~Sink() = default;
#define RCLLF_SINK_EV(ev, catv) \
  virtual bool enabled_ ## ev() = 0; \
  virtual void emit_ ## ev(uint64_t epoch, uint64_t seq, const rcllf_tp_ ## ev & e) = 0;
  RCLLF_TRACE_EVENTS(RCLLF_SINK_EV)
#undef RCLLF_SINK_EV
};

/// True when the binding was built with lttng-ust (the provider exists).
bool compiled_in();
/// A sink that reports every event disabled (the build without lttng-ust).
Sink & noop_sink();
/// Replace the sink (tests). nullptr restores the default sink.
void set_sink_for_test(Sink * sink);

/// Start tracing epoch `epoch`: counters reset, test hooks read from the
/// environment (RCLLF_TEST_TRACE_DROP_SEQ=<cat>:<n>, RCLLF_TEST_TRACE_SKIP_RUN_END).
/// Before begin() no event takes a seq.
void begin(uint64_t epoch);
bool begun();
bool closed();
uint64_t epoch();

/// Per-event emitters. Return true when the event took a seq. The run_end
/// group is not emitted through these (see close()).
#define RCLLF_EMIT_DECL(ev, catv) bool ev(const rcllf_tp_ ## ev & e);
RCLLF_TRACE_EVENTS(RCLLF_EMIT_DECL)
#undef RCLLF_EMIT_DECL

/// Seq and attempt counters at close, per category.
struct Snapshot
{
  uint64_t totals[RCLLF_CAT_COUNT];
  uint64_t attempts[RCLLF_CAT_COUNT];
};

/// Set the closed flag, wait for emitters already past the check, and
/// snapshot every category in one step. Later events take no seq. Returns
/// false (and leaves *out untouched) when already closed or never begun.
bool close(Snapshot * out);
/// run_end group, only after close(). run_end fills totals and attempts from
/// the snapshot and is emitted at most once; returns false on a second call.
void close_publisher(const rcllf_tp_run_end_publisher & e);
void close_edge(const rcllf_tp_run_end_edge & e);
bool close_run_end(rcllf_tp_run_end e);

/// Counters as of now (tests).
uint64_t total(uint8_t cat);
uint64_t attempts(uint8_t cat);

/// Test only: forget begin/close state.
void reset_for_test();

}  // namespace semtrace
}  // namespace ros2_lf_binding

#endif  // SEMANTIC_TRACE_HPP_

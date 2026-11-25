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

#include "semantic_trace.hpp"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

#ifdef RCLLF_HAVE_LTTNG
#include "rcllf_tp.h"  // NOLINT(build/include_subdir)
#endif

namespace ros2_lf_binding
{
namespace semtrace
{
namespace
{

class NoopSink final : public Sink
{
public:
#define RCLLF_NOOP_EV(ev, catv) \
  bool enabled_ ## ev() override {return false; \
  } \
  void emit_ ## ev(uint64_t, uint64_t, const rcllf_tp_ ## ev &) override {}
  RCLLF_TRACE_EVENTS(RCLLF_NOOP_EV)
#undef RCLLF_NOOP_EV
};

#ifdef RCLLF_HAVE_LTTNG
class LttngSink final : public Sink
{
public:
#define RCLLF_LTTNG_EV(ev, catv) \
  bool enabled_ ## ev() override {return tracepoint_enabled(rcllf, ev);} \
  void emit_ ## ev(uint64_t epoch, uint64_t seq, const rcllf_tp_ ## ev & e) override \
  { \
    do_tracepoint(rcllf, ev, epoch, seq, &e); \
  }
  RCLLF_TRACE_EVENTS(RCLLF_LTTNG_EV)
#undef RCLLF_LTTNG_EV
};
#endif

Sink & default_sink()
{
#ifdef RCLLF_HAVE_LTTNG
  static LttngSink sink;
#else
  static NoopSink sink;
#endif
  return sink;
}

struct Layer
{
  std::atomic<Sink *> sink{nullptr};
  std::atomic<uint64_t> epoch{0};
  std::atomic<bool> begun{false};
  std::atomic<bool> closed{false};
  std::atomic<bool> run_end_done{false};
  std::atomic<int> inflight{0};
  std::atomic<uint64_t> seq[RCLLF_CAT_COUNT]{};
  std::atomic<uint64_t> attempts[RCLLF_CAT_COUNT]{};
  Snapshot snap{};
  std::mutex close_mu;
  int drop_cat = -1;        // RCLLF_TEST_TRACE_DROP_SEQ
  uint64_t drop_seq = 0;
  bool skip_run_end = false;  // RCLLF_TEST_TRACE_SKIP_RUN_END
};

Layer & layer()
{
  static Layer * l = new Layer();  // never destroyed: static destruction must not race emitters
  return *l;
}

Sink & sink()
{
  Sink * s = layer().sink.load(std::memory_order_acquire);
  return s ? *s : default_sink();
}

/// Guard held across check, seq fetch-add and emit (Dekker pairing with close()).
struct Inflight
{
  Inflight() {layer().inflight.fetch_add(1, std::memory_order_seq_cst);}
  ~Inflight() {layer().inflight.fetch_sub(1, std::memory_order_release);}
};

template<typename T, typename Enabled, typename Emit>
bool emit_one(uint8_t cat, const T & e, Enabled enabled, Emit emit)
{
  Layer & l = layer();
  Inflight guard;
  if (!l.begun.load(std::memory_order_acquire) || l.closed.load(std::memory_order_seq_cst)) {
    return false;
  }
  l.attempts[cat].fetch_add(1, std::memory_order_relaxed);
  if (!enabled()) {
    return false;
  }
  const uint64_t seq = l.seq[cat].fetch_add(1, std::memory_order_relaxed) + 1;
  if (static_cast<int>(cat) == l.drop_cat && seq == l.drop_seq) {
    return true;  // test hook: the seq is taken, the event is lost
  }
  emit(l.epoch.load(std::memory_order_relaxed), seq, e);
  return true;
}

}  // namespace

bool compiled_in()
{
#ifdef RCLLF_HAVE_LTTNG
  return true;
#else
  return false;
#endif
}

Sink & noop_sink()
{
  static NoopSink s;
  return s;
}

void set_sink_for_test(Sink * s) {layer().sink.store(s, std::memory_order_release);}

void begin(uint64_t epoch)
{
  Layer & l = layer();
  std::lock_guard<std::mutex> lk(l.close_mu);
  for (int c = 0; c < RCLLF_CAT_COUNT; ++c) {
    l.seq[c].store(0);
    l.attempts[c].store(0);
  }
  l.drop_cat = -1;
  l.drop_seq = 0;
  if (const char * d = std::getenv("RCLLF_TEST_TRACE_DROP_SEQ")) {
    static const char * kNames[RCLLF_CAT_COUNT] = {
      "lifecycle", "publication", "delivery", "sample", "dispatch", "cleanup", "policy", "tuple"};
    const char * colon = std::strchr(d, ':');
    for (int c = 0; colon && c < RCLLF_CAT_COUNT; ++c) {
      if (std::strncmp(d, kNames[c], static_cast<size_t>(colon - d)) == 0 &&
        std::strlen(kNames[c]) == static_cast<size_t>(colon - d))
      {
        l.drop_cat = c;
        l.drop_seq = std::strtoull(colon + 1, nullptr, 10);
      }
    }
  }
  l.skip_run_end = std::getenv("RCLLF_TEST_TRACE_SKIP_RUN_END") != nullptr;
  l.epoch.store(epoch);
  l.closed.store(false);
  l.run_end_done.store(false);
  l.begun.store(true, std::memory_order_release);
}

bool begun() {return layer().begun.load(std::memory_order_acquire);}
bool closed() {return layer().closed.load(std::memory_order_acquire);}
uint64_t epoch() {return layer().epoch.load(std::memory_order_relaxed);}

#define RCLLF_EMIT_DEF(ev, catv) \
  bool ev(const rcllf_tp_ ## ev & e) \
  { \
    return emit_one( \
      static_cast<uint8_t>(catv), e, [] {return sink().enabled_ ## ev();}, \
      [](uint64_t ep, uint64_t sq, const rcllf_tp_ ## ev & x) {sink().emit_ ## ev(ep, sq, x);}); \
  }
RCLLF_TRACE_EVENTS(RCLLF_EMIT_DEF)
#undef RCLLF_EMIT_DEF

bool close(Snapshot * out)
{
  Layer & l = layer();
  std::lock_guard<std::mutex> lk(l.close_mu);
  if (!l.begun.load() || l.closed.load()) {
    return false;
  }
  l.closed.store(true, std::memory_order_seq_cst);
  while (l.inflight.load(std::memory_order_seq_cst) != 0) {
    std::this_thread::yield();
  }
  for (int c = 0; c < RCLLF_CAT_COUNT; ++c) {
    l.snap.totals[c] = l.seq[c].load();
    l.snap.attempts[c] = l.attempts[c].load();
  }
  if (out) {
    *out = l.snap;
  }
  return true;
}

void close_publisher(const rcllf_tp_run_end_publisher & e)
{
  Layer & l = layer();
  if (!l.closed.load() || l.run_end_done.load()) {return;}
  if (sink().enabled_run_end_publisher()) {sink().emit_run_end_publisher(l.epoch.load(), 0, e);}
}

void close_edge(const rcllf_tp_run_end_edge & e)
{
  Layer & l = layer();
  if (!l.closed.load() || l.run_end_done.load()) {return;}
  if (sink().enabled_run_end_edge()) {sink().emit_run_end_edge(l.epoch.load(), 0, e);}
}

bool close_run_end(rcllf_tp_run_end e)
{
  Layer & l = layer();
  if (!l.closed.load() || l.run_end_done.exchange(true)) {
    return false;
  }
  std::memcpy(e.totals, l.snap.totals, sizeof(e.totals));
  std::memcpy(e.attempts, l.snap.attempts, sizeof(e.attempts));
  if (!l.skip_run_end && sink().enabled_run_end()) {
    sink().emit_run_end(l.epoch.load(), 0, e);
  }
  return true;
}

uint64_t total(uint8_t cat) {return cat < RCLLF_CAT_COUNT ? layer().seq[cat].load() : 0;}
uint64_t attempts(uint8_t cat) {return cat < RCLLF_CAT_COUNT ? layer().attempts[cat].load() : 0;}

void reset_for_test()
{
  Layer & l = layer();
  std::lock_guard<std::mutex> lk(l.close_mu);
  l.begun.store(false);
  l.closed.store(false);
  l.run_end_done.store(false);
  for (int c = 0; c < RCLLF_CAT_COUNT; ++c) {
    l.seq[c].store(0);
    l.attempts[c].store(0);
  }
}

}  // namespace semtrace
}  // namespace ros2_lf_binding

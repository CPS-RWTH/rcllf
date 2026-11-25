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

/// @file correspondence.cpp
/// @brief Publication correspondence state machine (D3-D7).

#include "correspondence.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcutils/logging.h"

#include "console_log.hpp"

namespace ros2_lf_binding
{
namespace corr
{

namespace
{
// B_e bounds the records of one callback invocation (lf-analyze proves
// it from the publish call sites of one callback). A drain reaction now runs
// its callback once per record due at its tag, so the count is per invocation:
// the publishing thread tallies its records here, and the tally is checked and
// cleared at the end of each drain invocation and at the reaction's emit.
// Keyed by (table id, edge): ids are never reused, so a tally a dead table
// left on this thread never counts for another.
thread_local std::map<std::pair<uint64_t, uint32_t>, uint64_t> tl_burst;
std::atomic<uint64_t> next_burst_uid{1};
}  // namespace

std::string gid_hex(const Gid & gid)
{
  static const char * digits = "0123456789abcdef";
  std::string s;
  s.reserve(32);
  for (uint8_t b : gid) {
    s.push_back(digits[b >> 4]);
    s.push_back(digits[b & 0xf]);
  }
  return s;
}

const char * outcome_name(Outcome o)
{
  switch (o) {
    case Outcome::PENDING: return "PENDING";
    case Outcome::MATCHED: return "MATCHED";
    case Outcome::NOT_READY: return "NOT_READY";
    case Outcome::TIMED_OUT: return "TIMED_OUT";
    case Outcome::LOST: return "LOST";
    case Outcome::REJECTED: return "REJECTED";
    case Outcome::UNBOUND: return "UNBOUND";
    case Outcome::UNSUPPORTED: return "UNSUPPORTED";
    case Outcome::DUPLICATE: return "DUPLICATE";
    case Outcome::OVERFLOW: return "OVERFLOW";
    case Outcome::EPOCH_CLOSE: return "EPOCH_CLOSE";
    case Outcome::PUSH_OVERFLOW: return "push_overflow";
    case Outcome::SCHEDULE_REFUSED: return "schedule_refused";
    case Outcome::READER_REPLACED: return "READER_REPLACED";
  }
  return "?";
}

bool is_terminal(Outcome o) {return o != Outcome::PENDING;}

bool is_missing(Outcome o)
{
  return o == Outcome::TIMED_OUT || o == Outcome::LOST || o == Outcome::NOT_READY ||
         o == Outcome::OVERFLOW;
}

const char * sample_outcome_name(SampleOutcome o)
{
  switch (o) {
    case SampleOutcome::RETAINED: return "retained";
    case SampleOutcome::UNSUPPORTED: return "unsupported";
    case SampleOutcome::UNRESOLVED: return "unresolved";
    case SampleOutcome::LATE: return "late";
    case SampleOutcome::DUPLICATE: return "duplicate";
    case SampleOutcome::REJECTED_IDENTITY: return "rejected(identity)";
    case SampleOutcome::OVERFLOW: return "overflow";
    case SampleOutcome::EPOCH_CLOSE: return "epoch_close";
    case SampleOutcome::STALE_READER: return "stale_reader";
    case SampleOutcome::POOLED: return "pooled";
  }
  return "?";
}

const char * origin_name(Origin o)
{
  switch (o) {
    case Origin::ADMITTED: return "admitted";
    case Origin::NO_CONTEXT: return "no_context";
    case Origin::UNDECLARED_EFFECT: return "undeclared_effect";
    case Origin::EDGE_NOT_ADMITTED: return "edge_not_admitted";
    case Origin::UNSUPPORTED_API: return "unsupported_api";
    case Origin::NESTED: return "nested";
  }
  return "?";
}

const char * role_name(Role r)
{
  switch (r) {
    case Role::LOCAL: return "local";
    case Role::EGRESS: return "egress";
    case Role::INGRESS: return "ingress";
  }
  return "?";
}

namespace
{

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

uint64_t Table::new_burst_uid()
{
  return next_burst_uid.fetch_add(1);
}

Table::Table(uint64_t epoch)
: epoch_(epoch), clock_(steady_ns) {}

void Table::set_clock(std::function<int64_t()> clock)
{
  std::lock_guard<std::mutex> lk(mu_);
  clock_ = clock ? std::move(clock) : std::function<int64_t()>(steady_ns);
}

void Table::add_edge(
  uint32_t edge, uint32_t instance, int64_t delay_ns, int64_t timeout_ns, int64_t max_lag_ns,
  uint32_t burst, uint32_t retain, uint32_t carrier, Role role)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edges_.size() <= edge) {
    edges_.resize(edge + 1);
  }
  edges_[edge].role = role;
  edges_[edge].instance = instance;
  edges_[edge].delay_ns = delay_ns;
  edges_[edge].timeout_ns = timeout_ns;
  edges_[edge].max_lag_ns = max_lag_ns;
  edges_[edge].burst = burst;
  edges_[edge].retain_cap = retain > 0 ? retain : kRetainPerEdge;
  edges_[edge].carrier_cap = carrier > 0 ? carrier : kCarrierCap;
  size_t sum = 0;
  for (const Edge & x : edges_) {
    sum += x.retain_cap;
  }
  retain_total_cap_ = std::max(kRetainTotal, sum);
}

int64_t Table::lag_limit_locked(const Edge & e) const
{
  if (e.max_lag_ns >= 0) {
    return e.max_lag_ns;
  }
  int64_t t = -1;
  for (const auto & x : edges_) {
    t = std::max(t, x.timeout_ns);
  }
  if (t < 0 || t > kNoCutoff / 2) {
    return kNoCutoff;  // no timeout anywhere, or 2 * T_e past int64: no lag check
  }
  return 2 * t;
}

size_t Table::edge_count() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edges_.size();
}

namespace
{
/// a + b for b >= 0, saturated at kNoCutoff (a cutoff past int64 is no cutoff).
int64_t add_saturated(int64_t a, int64_t b)
{
  return a > kNoCutoff - b ? kNoCutoff : a + b;
}
}  // namespace

int64_t Table::cutoff_of(const Pub & p, uint32_t edge)
{
  for (size_t i = 0; i < p.edges.size() && i < p.cutoffs.size(); ++i) {
    if (p.edges[i] == edge) {
      return p.cutoffs[i];
    }
  }
  return kNoCutoff;
}

void Table::register_gid(const Gid & gid)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (finalized_.count(gid) != 0) {
    ++identity_faults_;  // a new publisher reuses a finalized GID
    finalized_.erase(gid);
  }
  writers_[gid];
}

void Table::predict(
  const Gid & gid, uint64_t ordinal, Origin origin, std::vector<uint32_t> edges, bool tracked,
  int64_t tag_ns, int64_t act_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  Writer & w = writers_[gid];
  w.assigned = std::max(w.assigned, ordinal);
  Pub & p = w.pubs[ordinal];
  p.state = PubState::PENDING;
  p.origin = origin;
  p.tracked = tracked;
  p.tag = tag_ns;
  p.act = act_ns;
  if (origin == Origin::ADMITTED && !w.poisoned && !closed_) {
    p.edges = std::move(edges);
    // the cutoff is fixed here, before the real publish, so that a
    // sample taken between the publish and commit() is already judged by it.
    // Causal window: it counts
    // from S(a), the physical start of the publishing activation (act_ns,
    // stamped after its inputs settled and before its user code), so every
    // publication of the activation shares one deadline S(a) + d + T and a
    // worker that ran behind its tags does not make its own on-time
    // publications late downstream (the lag cascade). Without an activation
    // scope the publish moment (this call) stands in. A fast federate keeps
    // the tag, its clock is logical.
    // The exec-config `cutoff_anchor` (tag only) is not consulted
    // here. It names the cross-federate (ingress) rule, see arrive_remote:
    // inside one federate a connection is never late (maxwait is a property
    // of network inputs), and a tag anchor on local edges cascades one loss
    // through the shared scheduler.
    // A local_now edge has no cutoff. Its sample is taken when the
    // publish returns, and T_e only sizes L and retention. The only local
    // edges that still get an S(a) window here are local sub_rebind edges,
    // which cannot be decided at publish.
    // Known limitation: a local sub_rebind edge keeps this S(a) + max(0, d) + T
    // window; whether it should become cutoff-free like local_now is open.
    p.cutoffs.clear();
    const int64_t base = fast_locked() ? tag_ns : act_ns >= 0 ? act_ns : clock_();
    for (uint32_t e : p.edges) {
      const bool timed = tag_ns >= 0 && e < edges_.size() && edges_[e].timeout_ns >= 0 &&
        !edges_[e].local_now;
      p.cutoffs.push_back(
        timed ? add_saturated(
          add_saturated(base, std::max<int64_t>(0, edges_[e].delay_ns)), edges_[e].timeout_ns) :
        kNoCutoff);
      claim_locked(e, gid);  // this writer's samples belong to member e of its group
    }
  }
  ++publications_;
}

void Table::commit(const Gid & gid, uint64_t ordinal, bool ok)
{
  std::lock_guard<std::mutex> lk(mu_);
  Writer & w = writers_[gid];
  auto it = w.pubs.find(ordinal);
  if (it == w.pubs.end()) {
    return;
  }
  Pub & p = it->second;
  if (!ok) {
    p.state = PubState::FAILED;
    ++failed_;
    w.failed.insert(ordinal);
    w.poisoned = true;
    for (uint32_t e : p.edges) {  // an early sample of a failed write: identity fault
      auto x = exps_.find(ExpKey{e, gid, ordinal});
      if (x != exps_.end() && !x->second.token) {
        if (x->second.has_sample) {
          --edges_[e].retained;
          --retained_total_;
          ++edges_[e].counts.samples[SampleOutcome::REJECTED_IDENTITY];
          --edges_[e].counts.samples[SampleOutcome::RETAINED];
        }
        exps_.erase(x);
        ++identity_faults_;
      }
    }
    p.edges.clear();
    return;
  }
  p.state = PubState::ACCEPTED;
  ++accepted_;
  if (p.origin == Origin::ADMITTED && !p.edges.empty()) {
    ++admitted_origin_;
    for (uint32_t e : p.edges) {
      if (e >= edges_.size()) {
        continue;
      }
      if (edges_[e].role == Role::EGRESS) {
        // The expectation lives in the subscriber's federate. Here
        // the publication becomes a record for the next emit.
        Edge & eg = edges_[e];
        const int64_t phys = clock_();
        eg.egress_queue.push_back(XRecord{gid, ordinal, phys, p.tag, p.act});
        ++eg.egress_records;
        ++tl_burst[{burst_uid_, e}];
        const int64_t lag = p.tag >= 0 ? phys - p.tag : 0;
        if (p.tag >= 0 && lag > lag_limit_locked(eg)) {
          ++eg.pub_lag_exceeded;
          if (lag > eg.pub_lag_worst_ns) {
            eg.pub_lag_worst_ns = lag;
            eg.pub_lag_worst_tag = p.tag;
          }
        }
        continue;
      }
      ExpKey key{e, gid, ordinal};
      Exp & x = exps_[key];
      x.token = true;
      x.cutoff = cutoff_of(p, e);
      x.pub_tag = p.tag;
      drop_stale_locked(key, x);  // an old reader's sample taken before this commit
      ++edges_[e].counts.expectations;
      if (observer_ != nullptr) {
        observer_->on_expectation(token_for(key));
      }
      if (is_terminal(x.outcome)) {
        // an early sample already overflowed. The token still goes
        // out so that the subscriber is told once, as a missing message, and
        // the key re-enters the ring (it left it while it had no token).
        edges_[e].tombstones.push_back(key);
      }
      edges_[e].in_flight.push_back(token_for(key));
      ++tl_burst[{burst_uid_, e}];
    }
  } else if (!p.tracked) {
    ++untracked_;
  } else {
    ++refused_origin_;
    ++refused_by_reason_[p.origin == Origin::ADMITTED ? Origin::EDGE_NOT_ADMITTED : p.origin];
  }
  prune_locked(gid, w);
}

void Table::note_nested(const Gid & gid, uint64_t ordinal, bool tracked)
{
  predict(gid, ordinal, Origin::NESTED, {}, tracked);
  commit(gid, ordinal, true);
}

void Table::finalize(const Gid & gid)
{
  std::lock_guard<std::mutex> lk(mu_);
  finalized_.insert(gid);
}

bool Table::poisoned(const Gid & gid) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = writers_.find(gid);
  return it != writers_.end() && it->second.poisoned;
}

Token Table::token_for(const ExpKey & key) const
{
  Token t;
  t.epoch = epoch_;
  t.gid = key.gid;
  t.ordinal = key.ordinal;
  t.edge = key.edge;
  t.instance = edges_[key.edge].instance;
  t.generation = edges_[key.edge].generation;
  return t;
}

bool Table::pub_open_locked(const Gid & gid, uint64_t ordinal, const Pub & p) const
{
  for (uint32_t e : p.edges) {
    auto x = exps_.find(ExpKey{e, gid, ordinal});
    if (x != exps_.end() && !is_terminal(x->second.outcome)) {
      return true;  // an open expectation, or an early sample waiting for its commit
    }
  }
  return false;
}

void Table::prune_locked(const Gid & gid, Writer & w)
{
  // The history keeps every publication with an open expectation.
  // Before, it kept the newest kWriterHistory publications whatever their
  // state, so with a timeout longer than kWriterHistory publication periods
  // a sample of a still open expectation found its publication gone and was
  // counted LATE, and its expectation ended missing.
  // The first fix stopped at the oldest open publication, so one
  // that stayed open (a lost sample on a local_now edge, which has no cutoff)
  // pinned every later ended publication and the history grew with the
  // commit count. Now the history is a window of the newest kWriterHistory
  // ordinals (ordinal > assigned - kWriterHistory) plus the older publications
  // still open (or still between predict and commit). An ordinal that leaves
  // the window is examined once: ended, it is erased at once; open, it is
  // held. The held ones are swept every max(kHeldRecheck, held) commits, so
  // an ended one leaves within that many commits, at amortized O(1) per
  // commit. Memory bound: kWriterHistory + O + E publications, O the open
  // publications older than the window, E <= the held count at the last
  // sweep (those that ended since).
  // A sample of an ordinal no longer held (at or below the floor, or erased
  // above it next to a held open one) belongs to a publication whose every
  // expectation had ended: late, as for a tombstoned one. The floor stays
  // below the oldest held publication.
  if (w.assigned <= kWriterHistory) {
    return;
  }
  const uint64_t edge = w.assigned - kWriterHistory;  // ordinals <= edge left the window
  ++w.since_sweep;
  const bool sweep = w.held != 0 &&
    w.since_sweep >= std::max<uint64_t>(kHeldRecheck, w.held);
  auto it = sweep ? w.pubs.begin() : w.pubs.upper_bound(w.swept);
  if (sweep) {
    w.held = 0;
    w.since_sweep = 0;
  }
  while (it != w.pubs.end() && it->first <= edge) {
    if (it->second.state == PubState::PENDING || pub_open_locked(gid, it->first, it->second)) {
      ++w.held;
      ++it;
    } else {
      it = w.pubs.erase(it);
    }
  }
  w.swept = std::max(w.swept, edge);
  uint64_t below = edge;
  if (!w.pubs.empty() && w.pubs.begin()->first <= edge) {
    below = w.pubs.begin()->first == 0 ? 0 : w.pubs.begin()->first - 1;
  }
  w.floor = std::max(w.floor, below);
}

uint32_t Table::in_flight(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() ? static_cast<uint32_t>(edges_[edge].in_flight.size()) : 0;
}

uint32_t Table::emit(uint32_t edge, uint32_t * fresh, std::vector<Token> * tokens)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    *fresh = 0;
    return 0;
  }
  Edge & e = edges_[edge];
  const size_t n = e.in_flight.size();
  *fresh = static_cast<uint32_t>(n > e.emitted ? n - e.emitted : 0);
  if (tokens != nullptr) {
    tokens->assign(e.in_flight.end() - *fresh, e.in_flight.end());
  }
  e.emitted = n;
  check_burst_locked(edge);  // B_e per callback invocation
  return static_cast<uint32_t>(n);
}

void Table::check_burst_locked(uint32_t edge)
{
  auto it = tl_burst.find({burst_uid_, edge});
  if (it == tl_burst.end()) {
    return;
  }
  Edge & e = edges_[edge];
  if (e.burst > 0 && it->second > e.burst) {
    e.burst_worst = std::max<uint64_t>(e.burst_worst, it->second);
  }
  tl_burst.erase(it);
}

void Table::end_invocation()
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = tl_burst.lower_bound({burst_uid_, 0});
  while (it != tl_burst.end() && it->first.first == burst_uid_) {
    const uint32_t edge = it->first.second;
    ++it;  // check_burst_locked erases the entry
    if (edge < edges_.size()) {
      check_burst_locked(edge);
    } else {
      tl_burst.erase(std::prev(it));
    }
  }
}

uint32_t Table::release_micro_locked(const Edge & e, uint32_t pub_microstep)
{
  // The release tag is reactor-c's
  // lf_delay_tag((t, m), offset), (t, m + 1) for offset 0, else (t + offset, 0).
  // lf_delay_tag wraps m + 1 to 0 at the last microstep; the table saturates
  // there, so a record never comes due before its publication tag.
  // A same-tag edge's drain is triggered by the zero-delay record port,
  // so its first record runs at the publication tag (t, m) itself.
  const int64_t offset = std::max<int64_t>(0, e.delay_ns) +
    (e.deferred ? std::max<int64_t>(0, e.timeout_ns) : 0);
  if (offset != 0) {
    return 0;
  }
  // A same-tag local edge and a zero-delay federated connection
  // deliver at (t, m) itself.
  if (e.same_tag || e.zero_link) {
    return pub_microstep;
  }
  return pub_microstep < kLastMicrostep ? pub_microstep + 1 : kLastMicrostep;
}

uint32_t Table::release_microstep(uint32_t edge, uint32_t pub_microstep) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() ? release_micro_locked(edges_[edge], pub_microstep) : 0;
}

bool Table::arrive(uint32_t edge, uint32_t count, int64_t release_ns, uint32_t pub_microstep)
{
  std::lock_guard<std::mutex> lk(mu_);
  return arrive_locked(edge, count, release_ns, pub_microstep);
}

bool Table::arrive_locked(
  uint32_t edge, uint32_t count, int64_t release_ns, uint32_t pub_microstep)
{
  if (edge >= edges_.size()) {
    return false;
  }
  Edge & e = edges_[edge];
  const uint32_t micro = release_micro_locked(e, pub_microstep);
  e.emitted = 0;
  if (count != e.in_flight.size()) {
    e.counts.count_mismatch += count > e.in_flight.size() ?
      count - e.in_flight.size() : e.in_flight.size() - count;
  }
  size_t n = std::min<size_t>(count, e.in_flight.size());
  for (size_t i = 0; i < n; ++i) {
    Token t = e.in_flight.front();
    e.in_flight.pop_front();
    ExpKey key{t.edge, t.gid, t.ordinal};
    auto it = exps_.find(key);
    if (it == exps_.end()) {
      continue;
    }
    if (closed_ || e.carrier.size() >= e.carrier_cap) {
      if (is_terminal(it->second.outcome)) {
        // An early OVERFLOW token is already missing: it still owes its one
        // missing dispatch (D-2), so it waits in the terminal queue.
        if (!closed_ && e.terminal.size() < kTerminalCap) {
          e.terminal.push_back(Queued{release_ns, micro, t});
        } else {
          it->second.dispatched = true;  // closed: shutdown, as for carrier tokens
          e.undelivered += closed_ ? 0 : 1;
        }
        continue;
      }
      // reject-newest
      terminate_locked(key, it->second, closed_ ? Outcome::EPOCH_CLOSE : Outcome::PUSH_OVERFLOW);
    } else {
      e.carrier.push_back(Queued{release_ns, micro, t});
    }
  }
  Edge & w = edges_[wake_edge_locked(edge)];  // one wake per group
  if ((!e.carrier.empty() || !e.terminal.empty()) && !w.wake_pending) {
    w.wake_pending = true;
    return true;
  }
  return false;
}

std::optional<Token> Table::pop_released(uint32_t edge, int64_t now_ns, uint32_t now_micro)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return std::nullopt;
  }
  Group * g = group_locked(edge);
  const uint32_t lone[] = {edge};
  const uint32_t * first = g != nullptr ? g->members.data() : lone;
  const uint32_t * last = g != nullptr ? first + g->members.size() : lone + 1;
  const std::pair<int64_t, uint32_t> now{now_ns, now_micro};
  while (true) {
    // The released entry of the smallest key
    // (release tag, rank, ordinal) over every member's carrier and terminal
    // queue, carrier first on ties. A member has one writer (R6), so each
    // queue is in key order and its head is its smallest entry.
    std::deque<Queued> * q = nullptr;
    std::tuple<int64_t, uint32_t, uint32_t, uint64_t, int> key{};
    for (const uint32_t * m = first; m != last; ++m) {
      Edge & e = edges_[*m];
      int kind = 0;  // 0 carrier, 1 terminal
      for (std::deque<Queued> * h : {&e.carrier, &e.terminal}) {
        const int k_kind = kind++;
        if (h->empty() ||
          std::make_pair(h->front().release_ns, h->front().release_micro) > now)
        {
          continue;
        }
        const Queued & f = h->front();
        const auto k =
          std::make_tuple(f.release_ns, f.release_micro, e.rank, f.token.ordinal, k_kind);
        if (q == nullptr || k < key) {
          q = h;
          key = k;
        }
      }
    }
    if (q == nullptr) {
      break;
    }
    Token t = q->front().token;
    q->pop_front();
    if (g != nullptr) {
      const auto popped =
        std::make_tuple(std::get<0>(key), std::get<1>(key), std::get<2>(key), std::get<3>(key));
      if (g->popped && popped < g->last) {
        ++g->order_violations;  // a smaller key came after a pop: a premise breach
      } else {
        g->last = popped;
      }
      g->popped = true;
    }
    auto it = exps_.find(ExpKey{t.edge, t.gid, t.ordinal});
    if (it != exps_.end()) {
      it->second.popped = true;
    }
    if (drop_token_ordinal_ != 0 && t.ordinal == drop_token_ordinal_) {
      continue;  // test hook F3: the token vanishes between pop and dispatch
    }
    return t;
  }
  return std::nullopt;
}

int64_t Table::next_wake(uint32_t edge, int64_t now_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return -1;
  }
  // A group wakes for the earliest head over its members.
  bool any = false;
  int64_t release = kNoCutoff;
  auto scan = [&](const Edge & e) {
      for (const auto * q : {&e.carrier, &e.terminal}) {
        if (!q->empty()) {
          any = true;
          release = std::min(release, q->front().release_ns);
        }
      }
    };
  if (const Group * g = group_locked(edge)) {
    for (uint32_t m : g->members) {
      scan(edges_[m]);
    }
  } else {
    scan(edges_[edge]);
  }
  Edge & w = edges_[wake_edge_locked(edge)];
  if (!any) {
    w.wake_pending = false;
    return -1;
  }
  w.wake_pending = true;
  return release <= now_ns ? 0 : release - now_ns;
}

void Table::schedule_refused(uint32_t edge, int64_t stop_time, uint32_t stop_micro)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return;
  }
  auto refuse = [this, stop_time, stop_micro](Edge & e) {
      for (auto & rt : e.carrier) {
        ExpKey key{rt.token.edge, rt.token.gid, rt.token.ordinal};
        auto it = exps_.find(key);
        if (it != exps_.end()) {
          // A release past the stop tag means the stop ended this
          // expectation. Its sample, whether taken before this refusal or
          // after it (a race at the stop tag), counts EPOCH_CLOSE, never late.
          const bool past = rt.release_ns > stop_time ||
            (rt.release_ns == stop_time && rt.release_micro > stop_micro);
          if (past && !is_terminal(it->second.outcome)) {
            it->second.stop_closed = true;
            release_early_locked(key, it->second, SampleOutcome::EPOCH_CLOSE);
          }
          if (past) {
            // Stop drain: its writer owes this sample, so the stop
            // drain takes it (counted epoch_close above) and samples_taken is
            // the same in every run. One that never comes leaves the writer
            // owing one at the drain bound, reported, never silent.
            Edge::Seen & s = edges_[key.edge].seen[key.gid];
            s.wanted = std::max(s.wanted, key.ordinal);
          }
          terminate_locked(key, it->second, Outcome::SCHEDULE_REFUSED);
        }
      }
      e.carrier.clear();
      for (auto & rt : e.terminal) {
        auto it = exps_.find(ExpKey{rt.token.edge, rt.token.gid, rt.token.ordinal});
        if (it != exps_.end()) {
          it->second.dispatched = true;
        }
        ++e.undelivered;
      }
      e.terminal.clear();
      e.wake_pending = false;
    };
  if (const Group * g = group_locked(edge)) {  // the group's one wake carried all members
    for (uint32_t m : g->members) {
      refuse(edges_[m]);
    }
  } else {
    refuse(edges_[edge]);
  }
}

EmitStatus Table::emit_remote(
  uint32_t edge, int64_t time, uint32_t microstep, size_t cap, std::vector<XRecord> * out,
  uint32_t * fresh, bool per_microstep, uint64_t * complete_seq)
{
  std::lock_guard<std::mutex> lk(mu_);
  out->clear();
  *fresh = 0;
  if (complete_seq != nullptr) {
    *complete_seq = 0;
  }
  if (edge >= edges_.size()) {
    return EmitStatus::OK;
  }
  Edge & e = edges_[edge];
  if (e.egress_queue.empty()) {
    // An activation that published nothing
    // on this edge sends no message and takes no completion number. Before,
    // it sent an empty COMPLETE per tag, and on two boards that traffic held
    // the federation at about 0.81 of real time. A reaction at a tag where
    // another one already sent records sets nothing either: LF keeps the
    // earlier lf_set, which already carries them.
    return EmitStatus::OK;
  }
  if (e.delay_ns > 0 && !per_microstep && e.sent_time == time && e.sent_micro != microstep) {
    // D2: `after d` maps every microstep of `time` to one tag, a second
    // message would be tardy at the receiver. `after 0` maps (t, m) to
    // (t, m + 1), so with d = 0 each microstep sends its own message.
    // a sender that sends at the publishing tag over `after d`
    // also sends one message per microstep; the receiver queues the k-th
    // message of one time at (t + d, k) (reactor-c patch 0002).
    return EmitStatus::COLLAPSE;
  }
  for (const XRecord & r : e.egress_queue) {
    if (r.tag != time) {
      // The flush sends the records of one time, the receiver puts
      // them on time + d. A record of another time would land on a wrong tag.
      return EmitStatus::STALE;
    }
  }
  if (e.sent_time == time && e.sent_micro == microstep) {
    *out = e.sent_now;  // cumulative: LF keeps the last lf_set of the tag
  }
  out->insert(out->end(), e.egress_queue.begin(), e.egress_queue.end());
  if (out->size() > cap) {
    return EmitStatus::CAP;
  }
  *fresh = static_cast<uint32_t>(e.egress_queue.size());
  e.egress_sent += e.egress_queue.size();
  if (e.delay_ns == 0) {
    // A record sent at an earlier time already landed (its time ran).
    if (e.sent_time != time) {
      e.zero_sent.clear();
    }
    if (!e.zero_sent.empty() && e.zero_sent.back().first == microstep) {
      e.zero_sent.back().second += e.egress_queue.size();
    } else {
      e.zero_sent.emplace_back(microstep, e.egress_queue.size());
    }
  } else if (!e.sent_window.empty() && e.sent_window.back().first == time &&
    e.sent_micro == microstep)
  {
    e.sent_window.back().second += e.egress_queue.size();
  } else {
    e.sent_window.emplace_back(time, e.egress_queue.size());
  }
  // A message of time t lands on (t + d, k): one of t < time - d lands before
  // `time`, which runs. The ones of t = time - d stay, the k-th of them
  // lands on (time, k), past a stop tag (time, s < k) (close_epoch).
  while (!e.sent_window.empty() && e.sent_window.front().first < time - e.delay_ns) {
    e.sent_window.pop_front();
  }
  e.egress_queue.clear();
  e.sent_now = *out;
  e.sent_time = time;
  e.sent_micro = microstep;
  if (complete_seq != nullptr) {
    // The flush sends one message per time, it lands on (time + d, 0);
    // close_epoch's completes_at_stop counts it as the 0-th message of time.
    *complete_seq = complete_seq_locked(e, time, per_microstep ? microstep : 0);
  }
  return EmitStatus::OK;
}

uint64_t Table::complete_seq(uint32_t edge, int64_t time, uint32_t microstep)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return 0;
  }
  return complete_seq_locked(edges_[edge], time, microstep);
}

uint64_t Table::complete_seq_locked(Edge & e, int64_t time, uint32_t microstep)
{
  if (e.complete_sent == 0 || e.complete_time != time || e.complete_micro != microstep) {
    ++e.complete_sent;
    e.complete_time = time;
    e.complete_micro = microstep;
    // Late COMPLETE: a message of an earlier time than `time` (d = 0) or than
    // time - d (d > 0) already landed, the federate runs `time` now.
    const int64_t landed_before = time - std::max<int64_t>(0, e.delay_ns);
    while (!e.complete_msgs.empty() && e.complete_msgs.front().first < landed_before) {
      e.complete_msgs.pop_front();
    }
    e.complete_msgs.emplace_back(time, microstep);
  }
  return e.complete_sent;
}

Table::CompleteResult Table::note_complete(uint32_t edge, uint64_t seq)
{
  std::lock_guard<std::mutex> lk(mu_);
  CompleteResult res;
  if (edge >= edges_.size() || edges_[edge].role != Role::INGRESS) {
    return res;
  }
  Edge & e = edges_[edge];
  if (seq <= e.complete_high) {
    // LF delivers each message once, so this is a layout or epoch fault.
    ++e.duplicate_completes;
    res.duplicate = true;
    return res;
  }
  // A sequence number is only listed up to this many per message and per
  // edge, so a corrupt seq cannot loop or grow memory without bound. 64 covers
  // every gap a run reports by hand (one per withheld activation) while one
  // report line stays readable. The counts stay exact.
  constexpr uint64_t kMaxListedControlFailures = 64;
  const uint64_t gap = seq - e.complete_high - 1;
  res.control_failure_count = gap;
  e.control_failure_count += gap;
  const uint64_t listed_end = e.complete_high + kMaxListedControlFailures;
  for (uint64_t s = e.complete_high + 1; s < seq && s <= listed_end; ++s) {
    res.control_failures.push_back(s);
    if (e.control_failures.size() < kMaxListedControlFailures) {
      e.control_failures.push_back(s);
    }
  }
  e.complete_high = seq;
  ++e.completes;
  return res;
}

std::vector<Table::CompleteEnd> Table::complete_ends() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<CompleteEnd> out;
  for (size_t i = 0; i < edges_.size(); ++i) {
    const Edge & e = edges_[i];
    if (e.role == Role::EGRESS) {
      out.push_back(
        CompleteEnd{static_cast<uint32_t>(i), e.complete_sent - e.completes_at_stop,
          e.complete_sent});
    }
  }
  return out;
}

void Table::expect_complete_end(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size() && edges_[edge].role == Role::INGRESS) {
    edges_[edge].complete_end_expected = true;
  }
}

void Table::note_complete_end(const CompleteEnd & end)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (end.edge >= edges_.size() || edges_[end.edge].role != Role::INGRESS) {
    return;
  }
  Edge & e = edges_[end.edge];
  if (e.complete_end_seen) {
    return;
  }
  e.complete_end_seen = true;
  e.complete_end_due = end.due;
  e.complete_end_sent = end.sent;
}

std::vector<uint32_t> Table::complete_ends_missing() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<uint32_t> out;
  for (size_t i = 0; i < edges_.size(); ++i) {
    if (edges_[i].complete_end_expected && !edges_[i].complete_end_seen) {
      out.push_back(static_cast<uint32_t>(i));
    }
  }
  return out;
}

void Table::release_early_locked(const ExpKey & key, Exp & exp, SampleOutcome outcome)
{
  Edge & e = edges_[key.edge];
  if (exp.has_sample) {
    exp.has_sample = false;
    exp.sample = Sample{};
    --e.retained;
    --retained_total_;
    ++released_retained_;
    --e.counts.samples[SampleOutcome::RETAINED];
    ++e.counts.samples[outcome];
    if (outcome == SampleOutcome::EPOCH_CLOSE) {
      note_epoch_close_locked(key.edge, key.gid, key.ordinal);
    }
  }
}

void Table::note_epoch_close_locked(uint32_t edge, const Gid & gid, uint64_t ordinal)
{
  EdgeCounts & c = edges_[edge].counts;
  if (c.epoch_close_ids.size() < kEpochCloseIdsCap) {
    c.epoch_close_ids.emplace_back(gid_hex(gid), ordinal);
  } else {
    c.epoch_close_ids_truncated = true;
  }
}

bool Table::stale_locked(
  const Edge & e, std::optional<int64_t> reader_created, int64_t pub_tag) const
{
  if (!e.sub_rebind || !reader_created || pub_tag == -1) {
    return false;
  }
  const auto later = std::upper_bound(
    e.reader_creations.begin(), e.reader_creations.end(), *reader_created);
  return later != e.reader_creations.end() && *later < pub_tag;
}

bool Table::drop_stale_locked(const ExpKey & key, Exp & exp)
{
  Edge & e = edges_[key.edge];
  if (!exp.has_sample || is_terminal(exp.outcome) ||
    !stale_locked(e, exp.reader_created, exp.pub_tag))
  {
    return false;
  }
  release_early_locked(key, exp, SampleOutcome::STALE_READER);
  e.stale_taken.insert(key);
  return true;
}

bool Table::arrive_remote(
  uint32_t edge, const std::vector<XRecord> & records, int64_t pub_time, int64_t release_ns,
  uint32_t pub_microstep)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size() || edges_[edge].role != Role::INGRESS) {
    return false;
  }
  Edge & e = edges_[edge];
  if (group_locked(edge) != nullptr) {
    // The records name their writers. The samples of a writer the
    // group's reader parked move to this member before the records adopt them.
    for (const XRecord & r : records) {
      claim_locked(edge, r.gid);
    }
  }
  // The window of every record is D = t + max(0, d) + T + eps,
  // t = pub_time the publication's logical tag (maxwait, Paladino et al.
  // Eq. 1b) and eps the declared clock bound, because the receiver compares
  // the shared tag to intake stamps on its own clock. A fast federate keeps
  // eps = 0, its stamps are logical tags. The moment a record is
  // processed and the receiver's logical lag never enter. A sender that runs
  // behind its tags makes its outputs late here: the receiver reports them
  // LATE or missing at its join (late = lost + reported), and the sender's
  // own lag stays its publisher_lag_exceeded check. Every record of one
  // publication shares D.
  const int64_t eps = fast_locked() ? 0 : e.clock_bound_ns;
  const int64_t cut = e.timeout_ns < 0 ? kNoCutoff : add_saturated(
    add_saturated(add_saturated(pub_time, std::max<int64_t>(0, e.delay_ns)), e.timeout_ns), eps);
  std::vector<ExpKey> adopted;
  std::set<Gid, GidLess> gids;
  for (const XRecord & r : records) {
    Edge::Remote & rm = e.remotes[r.gid];
    if (closed_ || (!rm.runs.empty() && r.ordinal <= rm.high)) {
      ++e.duplicate_records;  // D1 delivers each record once, in ordinal order
      continue;
    }
    rm.high = r.ordinal;
    rm.add(r.ordinal);
    gids.insert(r.gid);
    ExpKey key{edge, r.gid, r.ordinal};
    Exp & x = exps_[key];
    x.token = true;
    x.cutoff = cut;
    x.pub_tag = pub_time;
    ++e.counts.expectations;
    if (observer_ != nullptr) {
      observer_->on_expectation(token_for(key));
    }
    // An early sample of a replaced reader. The record stays open and
    // ends missing at its join, as when the sample came after the record.
    if (x.has_sample && !drop_stale_locked(key, x)) {
      // The intake stamp decides, whenever the record comes.
      if (late_for(e, x.stamp, cut)) {
        // Not taken by its cutoff: stays open and ends missing at its join.
        release_early_locked(key, x, SampleOutcome::LATE);
      } else {
        adopted.push_back(key);
      }
    }
    e.in_flight.push_back(token_for(key));
  }
  for (const ExpKey & k : adopted) {
    loss_gap_locked(edge, k.gid, k.ordinal);
  }
  // D5: records come in ordinal order per writer, so an unrecorded early
  // sample below the highest record never gets one (a refused origin).
  for (const Gid & g : gids) {
    const uint64_t high = e.remotes[g].high;
    std::vector<ExpKey> drop;
    for (auto it = exps_.lower_bound(ExpKey{edge, g, 0});
      it != exps_.end() && it->first.edge == edge && it->first.gid == g &&
      it->first.ordinal < high; ++it)
    {
      if (!it->second.token) {
        drop.push_back(it->first);
      }
    }
    for (const ExpKey & k : drop) {
      Exp & x = exps_[k];
      release_early_locked(k, x, SampleOutcome::UNSUPPORTED);
      exps_.erase(k);
    }
  }
  cv_.notify_all();
  // The publisher's microstep, not 0, so a d = 0 record gets the
  // key of its link, (t, m) over a zero-delay connection and (t, m + 1)
  // over `after 0`, and a fan-in group pops in publication order.
  return arrive_locked(
    edge, static_cast<uint32_t>(e.in_flight.size()), release_ns, pub_microstep);
}

bool Table::flush_due(uint32_t edge, int64_t time)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return false;
  }
  Edge & e = edges_[edge];
  // B_e bounds the records of one callback invocation. The flush sends
  // every record of a time; the count is this thread's tally.
  e.flush_counted = e.egress_records;
  check_burst_locked(edge);
  if (e.egress_queue.empty() || e.flush_time == time) {
    return false;
  }
  e.flush_time = time;
  return true;
}

void Table::flush_refused(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    ++edges_[edge].flushes_refused;
    edges_[edge].refused_times.push_back(edges_[edge].flush_time);
  }
}

void Table::note_clock_skew(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    ++edges_[edge].clock_skew;
  }
}

void Table::set_fast(std::function<bool()> fast)
{
  std::lock_guard<std::mutex> lk(mu_);
  fast_ = std::move(fast);
}

bool Table::fast_locked() const {return fast_ && fast_();}

void Table::set_clock_bound(uint32_t edge, int64_t eps_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    edges_[edge].clock_bound_ns = std::max<int64_t>(0, eps_ns);
  }
}

void Table::set_self_local(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size() && edges_[edge].role == Role::LOCAL) {
    edges_[edge].self_local = true;
    set_local_now_locked(edges_[edge]);  // a self edge is a local_now edge
  }
}

void Table::set_local_now_locked(Edge & e)
{
  if (e.role == Role::LOCAL && !e.sub_rebind && !e.deferred) {
    e.local_now = true;
  }
}

void Table::set_local_now(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    set_local_now_locked(edges_[edge]);
  }
}

bool Table::local_now(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() && edges_[edge].local_now;
}

std::vector<uint32_t> Table::local_now_edges(const Gid & gid, uint64_t ordinal) const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::set<uint32_t> readers;
  auto wi = writers_.find(gid);
  if (wi == writers_.end()) {
    return {};
  }
  auto pi = wi->second.pubs.find(ordinal);
  if (pi == wi->second.pubs.end() || pi->second.state == PubState::FAILED) {
    return {};
  }
  for (uint32_t e : pi->second.edges) {
    if (e < edges_.size() && edges_[e].local_now) {
      readers.insert(edges_[e].lead != kNoLead ? edges_[e].lead : e);  // the lead's reader
    }
  }
  return {readers.begin(), readers.end()};
}

void Table::lose_local_locked(const ExpKey & key, Exp & exp, bool planted)
{
  if (is_terminal(exp.outcome)) {
    return;
  }
  EdgeCounts & c = edges_[key.edge].counts;
  ++(planted ? c.local_undelivered_planted : c.local_undelivered);
  terminate_locked(key, exp, Outcome::LOST);
}

size_t Table::settle_local(
  const Gid & gid, uint64_t ordinal, const std::set<uint32_t> & planted)
{
  std::lock_guard<std::mutex> lk(mu_);
  auto wi = writers_.find(gid);
  if (wi == writers_.end()) {
    return 0;
  }
  auto pi = wi->second.pubs.find(ordinal);
  if (pi == wi->second.pubs.end() || pi->second.state != PubState::ACCEPTED) {
    return 0;
  }
  size_t ended = 0;
  for (uint32_t e : pi->second.edges) {
    if (e >= edges_.size() || !edges_[e].local_now) {
      continue;
    }
    const ExpKey key{e, gid, ordinal};
    auto x = exps_.find(key);
    if (x == exps_.end() || !x->second.token || x->second.has_sample ||
      is_terminal(x->second.outcome))
    {
      continue;  // retained, or already ended (an OVERFLOW at the take)
    }
    // Per edge, never per reader: a lead's id names the lead's own edge only,
    // so a planted drop on it cannot hide a real loss on a member.
    lose_local_locked(key, x->second, planted.count(e) != 0);
    ++ended;
  }
  return ended;
}

bool Table::local_open(const Gid & gid, uint64_t ordinal, uint32_t reader_edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto wi = writers_.find(gid);
  if (wi == writers_.end()) {
    return false;
  }
  auto pi = wi->second.pubs.find(ordinal);
  if (pi == wi->second.pubs.end() || pi->second.state != PubState::ACCEPTED) {
    return false;
  }
  for (uint32_t e : pi->second.edges) {
    if (e >= edges_.size() || !edges_[e].local_now) {
      continue;
    }
    if (reader_edge != kNoLead && wake_edge_locked(e) != reader_edge) {
      continue;
    }
    auto x = exps_.find(ExpKey{e, gid, ordinal});
    if (x != exps_.end() && x->second.token && !x->second.has_sample &&
      !is_terminal(x->second.outcome))
    {
      return true;
    }
  }
  return false;
}

bool Table::self_local(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() && edges_[edge].self_local;
}

bool Table::set_deferred(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size() || edges_[edge].role == Role::EGRESS || edges_[edge].self_local ||
    edges_[edge].local_now)
  {
    return false;  // a local_now edge is decided at publish, never at a cutoff tag
  }
  edges_[edge].deferred = true;
  return true;
}

bool Table::set_same_tag(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return false;
  }
  Edge & e = edges_[edge];
  if (e.role != Role::LOCAL || e.self_local || e.delay_ns != 0 || e.deferred) {
    return false;
  }
  e.same_tag = true;
  return true;
}

bool Table::same_tag(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() && edges_[edge].same_tag;
}

bool Table::deferred(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() && edges_[edge].deferred;
}

bool Table::set_zero_link(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return false;
  }
  Edge & e = edges_[edge];
  if (e.role != Role::INGRESS || e.delay_ns != 0) {
    return false;
  }
  e.zero_link = true;
  return true;
}

bool Table::zero_link(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() && edges_[edge].zero_link;
}

int64_t Table::release_extra_ns(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size() || !edges_[edge].deferred) {
    return 0;
  }
  return std::max<int64_t>(0, edges_[edge].timeout_ns);
}

bool Table::sample_ready(const Token & token) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (closed_ || token.epoch != epoch_ || token.edge >= edges_.size() ||
    token.instance != edges_[token.edge].instance ||
    token.generation != edges_[token.edge].generation)
  {
    return true;
  }
  auto it = exps_.find(ExpKey{token.edge, token.gid, token.ordinal});
  return it == exps_.end() || it->second.has_sample || is_terminal(it->second.outcome);
}

size_t Table::carrier_size(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() ? edges_[edge].carrier.size() + edges_[edge].terminal.size() : 0;
}

std::vector<Queued> Table::carrier_entries(uint32_t edge, std::vector<Queued> * terminal) const
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return {};
  }
  if (terminal != nullptr) {
    terminal->assign(edges_[edge].terminal.begin(), edges_[edge].terminal.end());
  }
  return {edges_[edge].carrier.begin(), edges_[edge].carrier.end()};
}

void Table::terminate_locked(const ExpKey & key, Exp & exp, Outcome outcome)
{
  if (is_terminal(exp.outcome)) {
    return;
  }
  exp.outcome = outcome;
  Edge & e = edges_[key.edge];
  ++e.counts.outcomes[outcome];
  if (observer_ != nullptr) {
    observer_->on_terminal(key.edge, key.gid, key.ordinal, outcome);
  }
  if (!e.stale_taken.empty()) {
    auto st = e.stale_taken.find(key);
    if (st != e.stale_taken.end()) {
      // The replaced reader's copy was the only one, the new reader had
      // not matched the writer yet. Named apart from a network loss.
      if (is_missing(outcome)) {
        ++e.counts.lost_discovery_gap;
      }
      e.stale_taken.erase(st);
    }
  }
  if (exp.has_sample && outcome != Outcome::MATCHED) {
    exp.has_sample = false;
    exp.sample = Sample{};
    --e.retained;
    --retained_total_;
    ++released_retained_;
  }
  e.tombstones.push_back(key);
  // Erase the oldest terminal keys beyond the ring, but never one whose token
  // is still queued for a dispatch (review m1: erasing it turned that dispatch
  // into a false orphan). Those rotate to the back until they are consumed.
  size_t budget = e.tombstones.size();
  while (e.tombstones.size() > kTombstones && budget-- > 0) {
    const ExpKey front = e.tombstones.front();
    e.tombstones.pop_front();
    auto old = exps_.find(front);
    if (old == exps_.end()) {
      continue;
    }
    const Exp & x = old->second;
    const bool queued = x.token && !x.dispatched &&
      x.outcome != Outcome::PUSH_OVERFLOW && x.outcome != Outcome::SCHEDULE_REFUSED &&
      x.outcome != Outcome::EPOCH_CLOSE;
    if (queued) {
      e.tombstones.push_back(front);
    } else if (x.token && is_terminal(x.outcome)) {
      exps_.erase(old);
    }
  }
  cv_.notify_all();
}

void Table::loss_gap_locked(uint32_t edge, const Gid & gid, uint64_t ordinal)
{
  // One writer delivers in order: a sample with ordinal k proves every earlier
  // ordinal of this writer on this edge still without a sample is lost.
  std::vector<ExpKey> lost;
  for (auto it = exps_.lower_bound(ExpKey{edge, gid, 0});
    it != exps_.end() && it->first.edge == edge && it->first.gid == gid &&
    it->first.ordinal < ordinal; ++it)
  {
    // A record published before the edge's current reader existed was
    // never this reader's to deliver; its drain settles it (rebind_settle).
    const bool before_reader = edges_[edge].sub_rebind && it->second.pub_tag != -1 &&
      it->second.pub_tag <= edges_[edge].reader_created;
    if (it->second.token && !it->second.has_sample && !is_terminal(it->second.outcome) &&
      !before_reader)
    {
      lost.push_back(it->first);
    }
  }
  for (const auto & k : lost) {
    if (edges_[edge].local_now) {
      lose_local_locked(k, exps_[k], false);  // settle_local should have ended it
    } else {
      terminate_locked(k, exps_[k], Outcome::LOST);
    }
  }
}

void Table::note_missing_locked(const ExpKey & key, Outcome outcome)
{
  if (!is_missing(outcome)) {
    return;
  }
  Edge & e = edges_[key.edge];
  ++e.counts.missing;
  if (e.missing.size() < kMissingListed) {
    e.missing.emplace_back(gid_hex(key.gid), key.ordinal);
  }
  // Say so when it is decided, whether or not the node registered a
  // deadline or message_lost handler (the runtime runs those after this
  // call); rate limit per edge, see kMissingWarnPeriodNs. Console only: this
  // runs under mu_, and rcl's output handler publishes on /rosout through the
  // publication gate, which takes mu_ (see console_log.hpp).
  ++e.warn_since;
  const int64_t now = clock_ ? clock_() : steady_ns();
  if (e.warn_last_ns == std::numeric_limits<int64_t>::min() ||
    now - e.warn_last_ns >= kMissingWarnPeriodNs)
  {
    const auto x = exps_.find(key);
    const int64_t pub_tag = x == exps_.end() ? -1 : x->second.pub_tag;
    console_log(
      __func__, RCUTILS_LOG_SEVERITY_WARN, "ros2_lf_binding.corr",
      "rcllf corr: edge %u: writer %s ordinal %" PRIu64 " %s (publish tag %" PRId64
      " ns): the sample never reached its reaction and is missing (%" PRIu64
      " missing on this edge so far, %" PRIu64 " since the last line; at most one line per "
      "edge per %" PRId64 " ms, the record at stop lists them all)",
      key.edge, gid_hex(key.gid).c_str(), key.ordinal, outcome_name(outcome), pub_tag,
      e.counts.missing, e.warn_since, kMissingWarnPeriodNs / 1000000);
    e.warn_last_ns = now;
    e.warn_since = 0;
  }
  if (e.local_now) {
    return;  // decided at publish, so no sample of it is in flight at stop
  }
  Edge::Seen & s = e.seen[key.gid];
  s.wanted = std::max(s.wanted, key.ordinal);
}

Outcome Table::join(
  const Token & token, std::chrono::milliseconds bound, Sample * out, uint64_t * retained_id)
{
  std::unique_lock<std::mutex> lk(mu_);
  if (token.epoch != epoch_ || token.edge >= edges_.size() ||
    token.instance != edges_[token.edge].instance ||
    token.generation != edges_[token.edge].generation)
  {
    ++rejected_tokens_;
    return Outcome::REJECTED;
  }
  ExpKey key{token.edge, token.gid, token.ordinal};
  auto it = exps_.find(key);
  if (it == exps_.end() || !it->second.token) {
    ++edges_[token.edge].counts.orphan_dispatches;
    return Outcome::REJECTED;
  }
  if (it->second.dispatched) {
    ++duplicate_tokens_;
    return Outcome::DUPLICATE;
  }
  it->second.dispatched = true;
  if (is_terminal(it->second.outcome)) {  // LOST or an early OVERFLOW
    note_missing_locked(key, it->second.outcome);
    return it->second.outcome;
  }
  auto ready = [&]() {
      auto x = exps_.find(key);
      return x == exps_.end() || x->second.has_sample || is_terminal(x->second.outcome) ||
             closed_;
    };
  const int64_t cutoff = it->second.cutoff;
  const bool deferred = edges_[token.edge].deferred;
  bool stop_hit = false;
  if (edges_[token.edge].local_now) {
    // Decided when the publish returned (settle_local), never a wait.
    // A sample neither retained nor settled here broke the premise as well.
    if (!ready()) {
      lose_local_locked(key, it->second, false);
      note_missing_locked(key, Outcome::LOST);
      return Outcome::LOST;
    }
  } else if (deferred) {
    // A deferred join never waits for its sample. LF runs it at the
    // cutoff tag and it decides from the table as it stands: a sample taken
    // in before the cutoff matches, also when the worker runs it late, and an
    // absent one times out. The expectation is then terminal, so a sample
    // that comes after the join is LATE, also should the join have run before
    // the cutoff on the table clock.
  } else if (!ready()) {
    const int64_t now = clock_();
    int64_t until = 0;
    if (cutoff != kNoCutoff) {
      // wait until the cutoff, never past it.
      // `now` is physical time in a real-time run (corr_rt::table_clock,
      // corr_runtime.cpp), so a join behind its tag by more than d + T
      // gives up here and reports the sample missing. In
      // the final brake_fusion configs (T 1000 ms, /radar/track 2000 ms) no
      // admitted edge comes near that in a no-loss run. The largest
      // physical lag of a matched sample was 11.2 ms idle and 55.4 ms under
      // stress-ng --cpu 4, one host and two hosts, on two x86 workstations
      // (final-design verification). On the Orins under the same load the
      // two-host layout fell behind its tags without bound (a profiling run): its
      // takes end here as NOT_READY and are reported, a capacity limit.
      // Only an ABSENT sample ends here. A sample taken before
      // its cutoff is ready whenever the join runs, and one not taken by now
      // can only be stamped after the cutoff, so it is lost either way.
      if (now > cutoff) {
        terminate_locked(key, it->second, Outcome::NOT_READY);
        note_missing_locked(key, Outcome::NOT_READY);
        return Outcome::NOT_READY;
      }
      until = cutoff + 1;
    } else {
      if (bound.count() <= 0) {
        // A join that does not wait gives the sample up for good (said
        // and counted like every other give-up).
        terminate_locked(key, it->second, Outcome::NOT_READY);
        note_missing_locked(key, Outcome::NOT_READY);
        return Outcome::NOT_READY;
      }
      until = now + std::chrono::duration_cast<std::chrono::nanoseconds>(bound).count();
    }
    // A stop request, also one made during the wait, bounds the wait
    // at the stop deadline. The join then ends pending at stop, not missing.
    joining_.insert(key);
    stop_hit = wait_until_locked(lk, until, ready);
    joining_.erase(key);
  }
  it = exps_.find(key);
  if (it == exps_.end()) {
    return Outcome::EPOCH_CLOSE;
  }
  Exp & x = it->second;
  // the stamp decides, not the moment of the join. A sample stamped
  // after the cutoff is never handed out (terminate_locked releases it),
  // unless a local self edge retained it as present.
  if (x.has_sample && (!late_for(edges_[key.edge], x.stamp, x.cutoff) || x.present)) {
    if (retained_id != nullptr) {
      *retained_id = x.retained_id;
    }
    *out = std::move(x.sample);
    x.sample = Sample{};
    x.has_sample = false;
    --edges_[key.edge].retained;
    --retained_total_;
    ++matched_samples_;
    terminate_locked(key, x, Outcome::MATCHED);
    return Outcome::MATCHED;
  }
  if (is_terminal(x.outcome)) {
    note_missing_locked(key, x.outcome);
    return x.outcome;
  }
  if (!closed_ && stop_hit) {
    // Below the stop tag, the sample still in flight at the stop
    // deadline. The program ended, not the cutoff: pending at stop, not missing.
    terminate_locked(key, x, Outcome::EPOCH_CLOSE);
    pending_at_stop_locked(key);
    return Outcome::EPOCH_CLOSE;
  }
  terminate_locked(key, x, closed_ ? Outcome::EPOCH_CLOSE : Outcome::TIMED_OUT);
  note_missing_locked(key, x.outcome);
  return x.outcome;
}

bool Table::wait_until_locked(
  std::unique_lock<std::mutex> & lk, int64_t until, const std::function<bool()> & ready)
{
  // One timed wait per deadline; a wait that times out ends the join on
  // that deadline even if the table clock (a test clock) did not advance.
  ++join_waits_;  // read by join_waits(): the join took the wait path
  for (;; ) {
    if (ready()) {
      return false;
    }
    const int64_t stop = stop_deadline_;
    const int64_t wake = std::min(until, stop);
    const int64_t now = clock_();
    if (now >= wake) {
      return stop < until;
    }
    const bool woke = cv_.wait_for(
      lk, std::chrono::nanoseconds(wake - now),
      [&]() {return ready() || stop_deadline_ != stop;});
    if (!woke) {
      return stop < until;  // timed out on the earlier of the two deadlines
    }
  }
}

void Table::pending_at_stop_locked(const ExpKey & key)
{
  Edge & e = edges_[key.edge];
  ++e.counts.pending_at_stop;
  if (e.pending.size() < kMissingListed) {
    e.pending.emplace_back(gid_hex(key.gid), key.ordinal);
  }
  Edge::Seen & s = e.seen[key.gid];
  s.wanted = std::max(s.wanted, key.ordinal);
}

std::vector<Table::PendingJoin> Table::note_stop_requested(int64_t bound_ns)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (stop_deadline_ == kNoCutoff) {
    stop_deadline_ = clock_() + bound_ns;
  }
  std::vector<PendingJoin> out;
  for (const auto & k : joining_) {
    auto it = exps_.find(k);
    out.push_back(
      PendingJoin{k.edge, gid_hex(k.gid), k.ordinal,
        it != exps_.end() ? it->second.pub_tag : -1});
  }
  cv_.notify_all();
  return out;
}

size_t Table::drain_in_flight(int64_t bound_ns)
{
  std::unique_lock<std::mutex> lk(mu_);
  auto owed = [this]() {
      size_t n = 0;
      for (const auto & e : edges_) {
        for (const auto & s : e.seen) {
          n += s.second.wanted > s.second.sample ? 1 : 0;
        }
      }
      return n;
    };
  if (!closed_ && owed() > 0) {
    // One timed wait: on_sample and close_epoch notify; a timeout ends it.
    cv_.wait_for(
      lk, std::chrono::nanoseconds(bound_ns), [&]() {return closed_ || owed() == 0;});
  }
  return owed();
}

int64_t Table::cutoff(const Token & token) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = exps_.find(ExpKey{token.edge, token.gid, token.ordinal});
  return it == exps_.end() ? kNoCutoff : it->second.cutoff;
}

void Table::note_lag(uint32_t edge, int64_t tag_ns, int64_t lag_ns, const char * point)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge >= edges_.size()) {
    return;
  }
  Edge & e = edges_[edge];
  if (lag_ns <= lag_limit_locked(e)) {
    return;
  }
  ++e.lag_exceeded;
  if (lag_ns > e.lag_worst_ns) {
    e.lag_worst_ns = lag_ns;
    e.lag_worst_tag = tag_ns;
    e.lag_worst_point = point;
  }
}

int64_t Table::max_lag_ns(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() ? lag_limit_locked(edges_[edge]) : kNoCutoff;
}

void Table::mark(const Token & token, Outcome outcome)
{
  std::lock_guard<std::mutex> lk(mu_);
  ExpKey key{token.edge, token.gid, token.ordinal};
  auto it = exps_.find(key);
  if (it != exps_.end()) {
    it->second.dispatched = true;
    terminate_locked(key, it->second, outcome);
  }
}

void Table::terminate_tardy(uint32_t edge, const std::vector<XRecord> & records)
{
  std::lock_guard<std::mutex> lk(mu_);
  for (const XRecord & r : records) {
    auto it = exps_.find(ExpKey{edge, r.gid, r.ordinal});
    if (it != exps_.end() && it->second.token) {
      it->second.tardy = true;
      terminate_locked(it->first, it->second, Outcome::LOST);
    }
  }
}

bool Table::is_tardy(const Token & token) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = exps_.find(ExpKey{token.edge, token.gid, token.ordinal});
  return it != exps_.end() && it->second.tardy;
}

int64_t Table::pub_tag(const Token & token) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = exps_.find(ExpKey{token.edge, token.gid, token.ordinal});
  return it == exps_.end() ? -1 : it->second.pub_tag;
}

void Table::note_reader_refused(
  uint32_t edge, const std::string & code, const std::string & detail)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    edges_[edge].reader_refused.push_back(code + ": " + detail);
  }
}

void Table::set_sub_rebind(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    edges_[edge].sub_rebind = true;
    // A pending reader's samples wait in DDS until it
    // is admitted, so a publish-time take is impossible. The edge keeps its
    // cutoff path.
    edges_[edge].local_now = false;
  }
}

void Table::note_reader_generation(uint32_t edge, uint32_t generation, int64_t created_tag)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    edges_[edge].reader_generation = generation;
    edges_[edge].reader_created = created_tag;
  }
}

void Table::note_reader_created(uint32_t edge, int64_t created_tag)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (edge < edges_.size()) {
    auto & c = edges_[edge].reader_creations;
    c.insert(std::upper_bound(c.begin(), c.end(), created_tag), created_tag);
  }
}

std::optional<Outcome> Table::rebind_settle(const Token & token, bool bound)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (token.edge >= edges_.size() || !edges_[token.edge].sub_rebind) {
    return std::nullopt;
  }
  const Edge & e = edges_[token.edge];
  const ExpKey key{token.edge, token.gid, token.ordinal};
  auto it = exps_.find(key);
  if (it == exps_.end()) {
    return std::nullopt;
  }
  Outcome o;
  if (!e.reader_refused.empty()) {
    o = Outcome::UNBOUND;  // the edge stopped, reconcile reports why
  } else if (!bound) {
    o = e.reader_generation >= 1 ? Outcome::READER_REPLACED : Outcome::UNBOUND;
  } else if (it->second.pub_tag != -1 && it->second.pub_tag <= e.reader_created) {
    // A lifecycle change at tag T takes effect after T, so a record of
    // tag <= the creation tag was published before this reader existed.
    o = e.reader_generation > 1 ? Outcome::READER_REPLACED : Outcome::UNBOUND;
  } else {
    return std::nullopt;
  }
  it->second.dispatched = true;
  terminate_locked(key, it->second, o);
  return it->second.outcome;
}

SampleOutcome Table::on_sample(
  uint32_t edge, const Gid & gid, uint64_t ordinal, Sample sample, uint64_t * retained_id,
  int64_t * stamp_ns, std::optional<int64_t> reader_created)
{
  std::lock_guard<std::mutex> lk(mu_);
  // stamped under the table lock, so the intake decision below and
  // a join deciding at the cutoff see one order.
  const int64_t stamp = clock_();
  if (stamp_ns != nullptr) {
    *stamp_ns = stamp;
  }
  const SampleOutcome o =
    on_sample_locked(edge, gid, ordinal, sample, stamp, retained_id, reader_created);
  cv_.notify_all();  // the drain at stop waits for a late sample too
  return o;
}

SampleOutcome Table::on_sample_locked(
  uint32_t edge, const Gid & gid, uint64_t ordinal, Sample & sample, int64_t stamp,
  uint64_t * retained_id, std::optional<int64_t> reader_created)
{
  if (edge >= edges_.size()) {
    return SampleOutcome::UNRESOLVED;
  }
  Edge & e = edges_[edge];
  ++e.counts.samples_taken;
  {
    // Whatever the sample's outcome, its writer has delivered this far.
    Edge::Seen & s = e.seen[gid];
    s.sample = std::max(s.sample, ordinal);
  }
  if (!e.sub_rebind) {
    reader_created.reset();
  }
  if (reader_created) {
    // The record belongs to the reader created last before its publish
    // tag. A reader created after this one and before that tag replaced this
    // one, whichever reader the table has bound yet, so the take order of the
    // two readers never decides the record. Before commit() made the
    // expectation the prediction has the tag; an ingress edge's early sample
    // has none yet and is judged when its record comes (drop_stale_locked).
    const ExpKey key{edge, gid, ordinal};
    auto x = exps_.find(key);
    int64_t pub_tag = x != exps_.end() ? x->second.pub_tag : -1;
    if (pub_tag == -1 && e.role != Role::INGRESS) {
      auto wi = writers_.find(gid);
      if (wi != writers_.end()) {
        auto pi = wi->second.pubs.find(ordinal);
        pub_tag = pi != wi->second.pubs.end() ? pi->second.tag : -1;
      }
    }
    if (stale_locked(e, reader_created, pub_tag)) {
      if (x == exps_.end() || !is_terminal(x->second.outcome)) {
        e.stale_taken.insert(key);
      }
      ++e.counts.samples[SampleOutcome::STALE_READER];
      return SampleOutcome::STALE_READER;
    }
  }
  if (e.role == Role::INGRESS) {
    SampleOutcome o = on_ingress_sample_locked(
      edge, gid, ordinal, sample, stamp, retained_id, reader_created);
    ++e.counts.samples[o];
    if (o == SampleOutcome::EPOCH_CLOSE) {
      note_epoch_close_locked(edge, gid, ordinal);
    }
    return o;
  }
  auto classify = [&]() -> SampleOutcome {
      if (closed_) {
        return SampleOutcome::EPOCH_CLOSE;
      }
      auto wi = writers_.find(gid);
      if (wi == writers_.end()) {
        return SampleOutcome::UNRESOLVED;
      }
      Writer & w = wi->second;
      if (ordinal > w.assigned || w.failed.count(ordinal) != 0) {
        ++identity_faults_;
        return SampleOutcome::REJECTED_IDENTITY;
      }
      if (ordinal <= w.floor) {
        return SampleOutcome::LATE;
      }
      auto pi = w.pubs.find(ordinal);
      if (pi == w.pubs.end()) {
        return SampleOutcome::LATE;
      }
      const Pub & p = pi->second;
      if (std::find(p.edges.begin(), p.edges.end(), edge) == p.edges.end()) {
        return SampleOutcome::UNSUPPORTED;
      }
      ExpKey key{edge, gid, ordinal};
      auto xi = exps_.find(key);
      if (xi == exps_.end() && p.state == PubState::ACCEPTED) {
        return SampleOutcome::LATE;  // committed, then tombstoned
      }
      if (xi != exps_.end()) {
        if (xi->second.outcome == Outcome::MATCHED || xi->second.has_sample) {
          return SampleOutcome::DUPLICATE;
        }
        if (is_terminal(xi->second.outcome)) {
          // Ended by the stop tag, not by a cutoff or a dispatch
          return xi->second.stop_closed ? SampleOutcome::EPOCH_CLOSE : SampleOutcome::LATE;
        }
      }
      // past the cutoff the sample is late. The expectation stays open
      // (or is created open by commit) and its dispatch delivers it missing.
      // Not on a local self edge. Its publication was predicted before
      // the write, so before this take, and the sample is here: the cutoff
      // bounds a wait for an absent sample, and this one needs no wait.
      const int64_t cut = cutoff_of(p, edge);
      const bool late = late_for(e, stamp, cut);  // strict on a deferred edge
      const bool present = late && (e.self_local || e.local_now);
      if (late && !present) {
        return SampleOutcome::LATE;
      }
      if (e.retained >= e.retain_cap || retained_total_ >= retain_total_cap_) {
        exps_[key].cutoff = cut;
        terminate_locked(key, exps_[key], Outcome::OVERFLOW);
        return SampleOutcome::OVERFLOW;
      }
      Exp & x = exps_[key];
      x.cutoff = cut;
      x.stamp = stamp;
      x.present = present;
      x.has_sample = true;
      x.reader_created = reader_created;
      x.retained_id = ++next_retained_id_;
      if (retained_id != nullptr) {
        *retained_id = x.retained_id;
      }
      x.sample = std::move(sample);
      ++e.retained;
      ++retained_total_;
      loss_gap_locked(edge, gid, ordinal);
      cv_.notify_all();
      return SampleOutcome::RETAINED;
    };
  SampleOutcome o = classify();
  ++e.counts.samples[o];
  if (o == SampleOutcome::EPOCH_CLOSE) {
    note_epoch_close_locked(edge, gid, ordinal);
  }
  return o;
}

SampleOutcome Table::on_ingress_sample_locked(
  uint32_t edge, const Gid & gid, uint64_t ordinal, Sample & sample, int64_t stamp,
  uint64_t * retained_id, std::optional<int64_t> reader_created)
{
  Edge & e = edges_[edge];
  if (closed_) {
    return SampleOutcome::EPOCH_CLOSE;
  }
  ExpKey key{edge, gid, ordinal};
  auto retain = [&](Exp & x) {
      x.stamp = stamp;
      x.has_sample = true;
      x.reader_created = reader_created;
      x.retained_id = ++next_retained_id_;
      if (retained_id != nullptr) {
        *retained_id = x.retained_id;
      }
      x.sample = std::move(sample);
      ++e.retained;
      ++retained_total_;
    };
  auto full = [&]() {
      return e.retained >= e.retain_cap || retained_total_ >= retain_total_cap_;
    };
  // R1-4: the expectation and the tombstones decide before the early path.
  auto xi = exps_.find(key);
  if (xi != exps_.end()) {
    Exp & x = xi->second;
    if (!x.token || x.outcome == Outcome::MATCHED || x.has_sample) {
      return SampleOutcome::DUPLICATE;
    }
    if (is_terminal(x.outcome)) {
      return x.stop_closed ? SampleOutcome::EPOCH_CLOSE : SampleOutcome::LATE;  // stop drain
    }
    if (late_for(e, stamp, x.cutoff)) {
      return SampleOutcome::LATE;  // the open expectation ends missing at its join
    }
    if (full()) {
      terminate_locked(key, x, Outcome::OVERFLOW);
      return SampleOutcome::OVERFLOW;
    }
    retain(x);
    loss_gap_locked(edge, gid, ordinal);
    cv_.notify_all();
    return SampleOutcome::RETAINED;
  }
  auto ri = e.remotes.find(gid);
  if (ri != e.remotes.end() && ordinal <= ri->second.high) {
    // Recorded and already gone from the ring: late. Below the highest
    // record and never recorded: a publication that made no record (refused
    // origin). The history is exact for the whole run, so an old gap
    // stays UNSUPPORTED however many records followed it.
    return ri->second.has(ordinal) ? SampleOutcome::LATE : SampleOutcome::UNSUPPORTED;
  }
  // D5: an early sample, its record is still on the way.
  if (full()) {
    // Make room by the oldest early sample of a writer that never sent a record.
    auto victim = exps_.end();
    for (auto it = exps_.lower_bound(ExpKey{edge, Gid{}, 0});
      it != exps_.end() && it->first.edge == edge; ++it)
    {
      if (!it->second.token && it->second.has_sample && e.remotes.count(it->first.gid) == 0 &&
        (victim == exps_.end() || it->second.retained_id < victim->second.retained_id))
      {
        victim = it;
      }
    }
    if (victim == exps_.end()) {
      return SampleOutcome::OVERFLOW;  // its record ends the expectation missing
    }
    release_early_locked(victim->first, victim->second, SampleOutcome::UNRESOLVED);
    exps_.erase(victim);
  }
  retain(exps_[key]);
  return SampleOutcome::RETAINED;
}

SampleOutcome Table::on_group_sample(
  uint32_t lead, const Gid & gid, uint64_t ordinal, Sample sample, uint32_t * member,
  uint64_t * retained_id, int64_t * stamp_ns, std::optional<int64_t> reader_created,
  bool * dropped)
{
  std::lock_guard<std::mutex> lk(mu_);
  const int64_t stamp = clock_();
  if (stamp_ns != nullptr) {
    *stamp_ns = stamp;
  }
  if (member != nullptr) {
    *member = lead;
  }
  if (dropped != nullptr) {
    *dropped = false;
  }
  uint32_t target = lead;
  Group * g = group_locked(lead);
  if (g != nullptr) {
    // The owner map holds each writer's member, from its
    // predicted edges (local) or the record that named it (ingress).
    auto oi = g->owner.find(gid);
    if (oi == g->owner.end()) {
      Edge & l = edges_[lead];
      auto unresolved = [&l] {
          ++l.counts.samples_taken;  // no member: the one outcome on the lead
          ++l.counts.samples[SampleOutcome::UNRESOLVED];
          return SampleOutcome::UNRESOLVED;
        };
      if (l.role != Role::INGRESS || closed_) {
        return unresolved();
      }
      // An ingress group learns a writer from its first record, which may
      // come after the sample: the sample waits in the group's early pool.
      auto & parked = g->pool[gid];
      if (parked.count(ordinal) != 0) {
        return unresolved();  // a second take of a parked sample
      }
      size_t cap = 0;
      for (uint32_t m : g->members) {
        cap += edges_[m].retain_cap;
      }
      if (g->pooled >= cap || retained_total_ >= retain_total_cap_) {
        // Make room by the oldest pooled sample, its writer unknown longest.
        const Gid * vg = nullptr;
        uint64_t vo = 0;
        int64_t vs = 0;
        for (const auto & [pg, by_ordinal] : g->pool) {
          for (const auto & [po, p] : by_ordinal) {
            if (vg == nullptr || p.stamp < vs) {
              vg = &pg;
              vo = po;
              vs = p.stamp;
            }
          }
        }
        if (vg == nullptr) {
          g->pool.erase(gid);
          return unresolved();  // the table is full of other edges' samples
        }
        const Gid victim = *vg;
        release_pooled_locked(lead, *g, victim, vo, SampleOutcome::UNRESOLVED);
      }
      Pooled & p = g->pool[gid][ordinal];
      p.sample = std::move(sample);
      p.stamp = stamp;
      p.reader_created = reader_created;
      ++g->pooled;
      ++retained_total_;
      return SampleOutcome::POOLED;
    }
    target = oi->second;
  }
  if (member != nullptr) {
    *member = target;
  }
  if (drop_sample_locked(target, ordinal)) {
    if (dropped != nullptr) {
      *dropped = true;
    }
    return SampleOutcome::UNRESOLVED;  // F3: lost in transport, never counted
  }
  return on_sample_locked(target, gid, ordinal, sample, stamp, retained_id, reader_created);
}

bool Table::drop_sample_locked(uint32_t edge, uint64_t ordinal) const
{
  return drop_sample_ordinal_ != 0 && ordinal == drop_sample_ordinal_ &&
         edge == drop_sample_edge_;
}

bool Table::set_fan_in(uint32_t edge, uint32_t lead, uint32_t rank, std::string * why)
{
  std::lock_guard<std::mutex> lk(mu_);
  auto refuse = [why](const std::string & m) {
      if (why != nullptr) {
        *why = m;
      }
      return false;
    };
  auto role = [](Role r) {
      return r == Role::LOCAL ? "local" : r == Role::INGRESS ? "ingress" : "egress";
    };
  const std::string G = "fan-in group " + std::to_string(lead) + ": edge " + std::to_string(edge);
  if (edge >= edges_.size() || lead >= edges_.size()) {
    return refuse(G + " or its lead is not registered");
  }
  Edge & e = edges_[edge];
  if (e.role == Role::EGRESS || e.role != edges_[lead].role) {
    return refuse(
      G + " is " + role(e.role) + ", edge " + std::to_string(lead) + " is " +
      role(edges_[lead].role) + "; a group is all local or all ingress");
  }
  if (e.lead != kNoLead && e.lead != lead) {
    return refuse(G + " is in the group of edge " + std::to_string(e.lead));
  }
  e.lead = lead;
  e.rank = rank;
  auto & members = groups_[lead].members;
  const auto at = std::lower_bound(members.begin(), members.end(), edge);
  if (at == members.end() || *at != edge) {
    members.insert(at, edge);
  }
  return true;
}

std::vector<uint32_t> Table::group_members(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  const Group * g = group_locked(edge);
  return g != nullptr ? g->members : std::vector<uint32_t>{edge};
}

uint64_t Table::order_violations(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  const Group * g = group_locked(edge);
  return g != nullptr ? g->order_violations : 0;
}

uint32_t Table::sample_edge(uint32_t edge, const Gid & gid) const
{
  std::lock_guard<std::mutex> lk(mu_);
  const Group * g = group_locked(edge);
  if (g == nullptr) {
    return edge;
  }
  auto it = g->owner.find(gid);
  return it != g->owner.end() ? it->second : edge;
}

Table::Group * Table::group_locked(uint32_t edge)
{
  return const_cast<Group *>(static_cast<const Table *>(this)->group_locked(edge));
}

const Table::Group * Table::group_locked(uint32_t edge) const
{
  if (edge >= edges_.size()) {
    return nullptr;
  }
  // The lead is a member too; it may be registered by its members alone.
  const uint32_t lead = edges_[edge].lead != kNoLead ? edges_[edge].lead : edge;
  auto it = groups_.find(lead);
  return it != groups_.end() && it->second.members.size() > 1 ? &it->second : nullptr;
}

uint32_t Table::wake_edge_locked(uint32_t edge) const
{
  if (group_locked(edge) == nullptr) {
    return edge;
  }
  return edges_[edge].lead != kNoLead ? edges_[edge].lead : edge;
}

void Table::claim_locked(uint32_t edge, const Gid & gid)
{
  Group * g = group_locked(edge);
  if (g == nullptr) {
    return;
  }
  const auto [oi, fresh] = g->owner.emplace(gid, edge);
  if (!fresh) {
    if (oi->second != edge) {
      ++identity_faults_;  // one writer on two members of one group
    }
    return;
  }
  auto pi = g->pool.find(gid);
  if (pi == g->pool.end()) {
    return;
  }
  std::map<uint64_t, Pooled> parked = std::move(pi->second);
  g->pool.erase(pi);
  for (auto & [ordinal, p] : parked) {
    --g->pooled;
    --retained_total_;
    if (drop_sample_locked(edge, ordinal)) {
      continue;  // F3: the hook meets a pooled sample at adoption
    }
    uint64_t rid = 0;
    const SampleOutcome o =
      on_sample_locked(edge, gid, ordinal, p.sample, p.stamp, &rid, p.reader_created);
    if (observer_ != nullptr) {
      observer_->on_pool_exit(
        edge, gid, ordinal, o, o == SampleOutcome::RETAINED ? rid : 0, p.stamp);
    }
  }
}

void Table::release_pooled_locked(
  uint32_t lead, Group & g, const Gid & gid, uint64_t ordinal, SampleOutcome outcome)
{
  auto pi = g.pool.find(gid);
  if (pi == g.pool.end()) {
    return;
  }
  auto oi = pi->second.find(ordinal);
  if (oi == pi->second.end()) {
    return;
  }
  const int64_t stamp = oi->second.stamp;
  pi->second.erase(oi);
  if (pi->second.empty()) {
    g.pool.erase(pi);
  }
  --g.pooled;
  --retained_total_;
  // Counted on the lead: no member ever named the writer.
  ++edges_[lead].counts.samples_taken;
  ++edges_[lead].counts.samples[outcome];
  if (outcome == SampleOutcome::EPOCH_CLOSE) {
    note_epoch_close_locked(lead, gid, ordinal);
  }
  if (observer_ != nullptr) {
    observer_->on_pool_exit(lead, gid, ordinal, outcome, 0, stamp);
  }
}

size_t Table::retained() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return retained_total_;
}

size_t Table::writer_history(const Gid & gid) const
{
  std::lock_guard<std::mutex> lk(mu_);
  auto it = writers_.find(gid);
  return it == writers_.end() ? 0 : it->second.pubs.size();
}

void Table::close_epoch(int64_t stop_time, uint32_t stop_micro)
{
  std::lock_guard<std::mutex> lk(mu_);
  if (closed_) {
    return;
  }
  closed_ = true;
  close_stop_time_ = stop_time;
  for (auto & e : edges_) {
    if (e.role != Role::EGRESS) {
      continue;
    }
    // The flush of time t runs at (t + 1 ns, 0). For t equal to the
    // stop time that is past the stop tag, so LF clears or refuses it. Its
    // records are retained at stop. The receiver could not take them either,
    // they would land at t + d, past the same stop tag. No reaction runs at a
    // time after the stop time, so a later record means a wrong stop time and
    // stays queued, where reconcile reports it.
    const auto cut = std::stable_partition(
      e.egress_queue.begin(), e.egress_queue.end(),
      [stop_time](const XRecord & r) {return r.tag != stop_time;});
    e.egress_at_stop += static_cast<uint64_t>(std::distance(cut, e.egress_queue.end()));
    e.egress_queue.erase(cut, e.egress_queue.end());
    // A record of t < stop time was flushed and sent, but it lands on
    // (t + d, 0). For t + d past the stop time LF drops it, on the sending
    // side (federate.c drops a message whose tag is after the stop tag) or
    // on the receiving side. The receiver never expects it, so it is at stop
    // and not sent.
    // One entry per message. Messages of one time t sent at different
    // microsteps land on (t + d, 0), (t + d, 1), ... in send order (the
    // receiver defers each colliding one a microstep, reactor-c patch 0002),
    // so the k-th of t = stop time - d is past a stop tag (stop time, s < k).
    int64_t prev = std::numeric_limits<int64_t>::min();
    uint32_t k = 0;
    for (const auto & [t, n] : e.sent_window) {
      k = t == prev ? k + 1 : 0;
      prev = t;
      if (t > stop_time - e.delay_ns || (t == stop_time - e.delay_ns && k > stop_micro)) {
        e.egress_sent -= n;
        e.egress_at_stop += n;
      }
    }
    // With d = 0 a record sent at (t, m) lands on (t, m + 1). It is
    // delivered when that is at or before the stop tag (stop_time, stop_micro),
    // else LF drops it and it is at stop. For a timeout stop (T, 0) every
    // record of the stop time T is at stop, as with d > 0.
    for (const auto & [m, n] : e.zero_sent) {
      if (e.sent_time > stop_time || (e.sent_time == stop_time && m >= stop_micro)) {
        e.egress_sent -= n;
        e.egress_at_stop += n;
      }
    }
    // Late COMPLETE: the COMPLETE messages that land past the stop tag. The
    // receiver never sees them, so its last completion number is
    // completes_sent - completes_at_stop; a lower one is a trailing
    // CONTROL_FAILURE (rq2b_oracle.merge_reconciles compares both sides).
    {
      int64_t prev_t = std::numeric_limits<int64_t>::min();
      uint32_t k = 0;
      for (const auto & [t, m] : e.complete_msgs) {
        k = t == prev_t ? k + 1 : 0;
        prev_t = t;
        const bool past = e.delay_ns <= 0 ?
          (t > stop_time || (t == stop_time && m >= stop_micro)) :
          (t > stop_time - e.delay_ns || (t == stop_time - e.delay_ns && k > stop_micro));
        e.completes_at_stop += past ? 1 : 0;
      }
    }
    for (int64_t t : e.refused_times) {
      if (t == stop_time) {
        --e.flushes_refused;
        ++e.flushes_at_stop;
      }
    }
  }
  for (auto & [lead, g] : groups_) {  // no member named these writers
    while (!g.pool.empty()) {
      const Gid gid = g.pool.begin()->first;
      release_pooled_locked(
        lead, g, gid, g.pool.begin()->second.begin()->first, SampleOutcome::UNRESOLVED);
    }
  }
  std::vector<ExpKey> keys;
  for (auto & kv : exps_) {
    keys.push_back(kv.first);
  }
  for (const auto & k : keys) {
    auto it = exps_.find(k);
    if (it == exps_.end()) {
      continue;
    }
    Exp & x = it->second;
    if (!x.token) {  // an early sample whose publish never committed
      if (x.has_sample) {
        Edge & ke = edges_[k.edge];
        // An ingress early sample of a writer that never sent a record.
        const bool unresolved = ke.role == Role::INGRESS && ke.remotes.count(k.gid) == 0;
        --edges_[k.edge].retained;
        --retained_total_;
        ++edges_[k.edge].counts.samples[
          unresolved ? SampleOutcome::UNRESOLVED : SampleOutcome::EPOCH_CLOSE];
        --edges_[k.edge].counts.samples[SampleOutcome::RETAINED];
        if (!unresolved) {
          note_epoch_close_locked(k.edge, k.gid, k.ordinal);
        }
      }
      exps_.erase(it);
      continue;
    }
    if (is_terminal(x.outcome)) {
      continue;
    }
    if (x.popped && !x.dispatched) {
      // Popped for a dispatch that never ran: the sample is released, the
      // outcome is left missing for reconcile to name.
      if (x.has_sample) {
        x.has_sample = false;
        x.sample = Sample{};
        --edges_[k.edge].retained;
        --retained_total_;
        ++edges_[k.edge].counts.samples[SampleOutcome::EPOCH_CLOSE];
        --edges_[k.edge].counts.samples[SampleOutcome::RETAINED];
        note_epoch_close_locked(k.edge, k.gid, k.ordinal);
      }
      continue;
    }
    if (x.has_sample) {
      ++edges_[k.edge].counts.samples[SampleOutcome::EPOCH_CLOSE];
      --edges_[k.edge].counts.samples[SampleOutcome::RETAINED];
      note_epoch_close_locked(k.edge, k.gid, k.ordinal);
    }
    {
      // Record the message time (the release tag the
      // carrier holds for this token) and the stop tag with the outcome.
      EpochClosed ec;
      ec.edge = k.edge;
      ec.q.token.gid = k.gid;
      ec.q.token.ordinal = k.ordinal;
      ec.q.token.edge = k.edge;
      ec.stop_time = stop_time;
      ec.stop_micro = stop_micro;
      for (const auto & q : edges_[k.edge].carrier) {
        if (q.token.gid == k.gid && q.token.ordinal == k.ordinal) {
          ec.q = q;
          ec.release_known = true;
          break;
        }
      }
      epoch_closed_.push_back(ec);
    }
    terminate_locked(k, x, Outcome::EPOCH_CLOSE);
  }
  const int64_t now = clock_();
  for (auto & e : edges_) {
    // a token whose release is more than L_e old was never drained in
    // time. Its outcome is EPOCH_CLOSE; the lag is a reconcile problem.
    const int64_t limit = lag_limit_locked(e);
    for (const auto & rt : e.carrier) {
      if (limit != kNoCutoff && now - rt.release_ns > limit) {
        ++e.undrained;
      }
    }
    for (const auto & rt : e.terminal) {  // already missing, judged like the carrier
      auto it = exps_.find(ExpKey{rt.token.edge, rt.token.gid, rt.token.ordinal});
      if (it != exps_.end()) {
        it->second.dispatched = true;
      }
      if (limit != kNoCutoff && now - rt.release_ns > limit) {
        ++e.undrained;
      }
    }
  }
  for (auto & e : edges_) {
    e.in_flight.clear();
    e.emitted = 0;
    e.carrier.clear();
    e.terminal.clear();
    e.wake_pending = false;
  }
  cv_.notify_all();
}

uint64_t Table::released_retained() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return released_retained_;
}

std::vector<Table::EpochClosed> Table::epoch_closed_entries() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return epoch_closed_;
}

uint64_t Table::epoch_close_terminals() const
{
  std::lock_guard<std::mutex> lk(mu_);
  uint64_t n = 0;
  for (const auto & e : edges_) {
    auto it = e.counts.outcomes.find(Outcome::EPOCH_CLOSE);
    n += it == e.counts.outcomes.end() ? 0 : it->second;
  }
  return n;
}

bool Table::closed() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return closed_;
}

EdgeCounts Table::counts(uint32_t edge) const
{
  std::lock_guard<std::mutex> lk(mu_);
  return edge < edges_.size() ? edges_[edge].counts : EdgeCounts{};
}

uint64_t Table::identity_faults() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return identity_faults_;
}

int64_t Table::stall_bound_ns(int64_t max_timer_period_ns) const
{
  if (max_timer_period_ns <= 0) {
    return -1;  // no timer: an idle federate stands still legitimately
  }
  const int64_t w = stall_window_ns();
  return w < 0 ? -1 : add_saturated(max_timer_period_ns, w);
}

int64_t Table::stall_window_ns() const
{
  std::lock_guard<std::mutex> lk(mu_);
  int64_t w = 0;
  int64_t lag = -1;
  bool any = false;
  for (const auto & e : edges_) {
    if (e.role != Role::INGRESS) {
      continue;
    }
    if (e.timeout_ns < 0) {
      return -1;  // infinite T: a healthy wait on this edge has no bound
    }
    any = true;
    // eps: the window D = S(a) + d + T + eps is counted on the sender's
    // clock, the watch on ours, so the declared offset bound widens it.
    w = add_saturated(
      w, add_saturated(
        add_saturated(std::max<int64_t>(0, e.delay_ns), e.timeout_ns),
        std::max<int64_t>(0, e.clock_bound_ns)));
    lag = std::max(lag, lag_limit_locked(e));
  }
  if (!any || lag >= kNoCutoff) {
    return -1;
  }
  return add_saturated(w, lag);
}

void Table::note_stall()
{
  std::lock_guard<std::mutex> lk(mu_);
  ++stalls_;
}

uint64_t Table::stalls() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return stalls_;
}

uint64_t Table::join_waits() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return join_waits_;
}

Report Table::reconcile() const
{
  std::lock_guard<std::mutex> lk(mu_);
  Report r;
  auto fail = [&r](const std::string & m) {
      r.ok = false;
      r.problems.push_back(m);
    };
  // The tags of the records still queued and the stop time they were cut
  // against, so a failing run names its cause.
  auto queued_tags = [this](const Edge & e) {
      if (e.egress_queue.empty()) {
        return std::string();
      }
      std::string t = " (tags";
      for (const XRecord & q : e.egress_queue) {
        t += " " + std::to_string(q.tag);
      }
      return t + ", stop time " + std::to_string(close_stop_time_) + ", last flush " +
             std::to_string(e.flush_time) + ", last sent " + std::to_string(e.sent_time) + ")";
    };
  if (publications_ != accepted_ + failed_) {
    fail(
      "publications " + std::to_string(publications_) + " != accepted " +
      std::to_string(accepted_) + " + failed " + std::to_string(failed_));
  }
  if (accepted_ != admitted_origin_ + refused_origin_ + untracked_) {
    fail(
      "accepted " + std::to_string(accepted_) + " != admitted origin " +
      std::to_string(admitted_origin_) + " + refused origin " + std::to_string(refused_origin_) +
      " + untracked " + std::to_string(untracked_));
  }
  for (const auto & kv : exps_) {
    if (kv.second.token && !is_terminal(kv.second.outcome)) {
      fail(
        "missing outcome: expectation (" + gid_hex(kv.first.gid) + ", " +
        std::to_string(kv.first.ordinal) + ", edge " + std::to_string(kv.first.edge) +
        ") has no terminal outcome");
    }
  }
  std::ostringstream js;
  js << "{\"event\":\"reconcile\",\"epoch\":" << epoch_ << ",\"publications\":" << publications_ <<
    ",\"accepted\":" << accepted_ << ",\"failed\":" << failed_ << ",\"admitted_origin\":" <<
    admitted_origin_ << ",\"refused_origin\":" << refused_origin_ << ",\"untracked\":" <<
    untracked_ << ",\"refused\":{";
  bool first = true;
  for (const auto & kv : refused_by_reason_) {
    js << (first ? "" : ",") << "\"" << origin_name(kv.first) << "\":" << kv.second;
    first = false;
  }
  js << "},\"edges\":[";
  size_t carrier = 0;
  for (size_t i = 0; i < edges_.size(); ++i) {
    const Edge & e = edges_[i];
    const std::string E = std::to_string(i);
    uint64_t terminal = 0;
    for (const auto & o : e.counts.outcomes) {
      terminal += o.second;
    }
    if (terminal != e.counts.expectations) {
      fail(
        "edge " + E + ": expectations " + std::to_string(e.counts.expectations) +
        " != terminal outcomes " + std::to_string(terminal));
    }
    if (e.counts.orphan_dispatches != 0) {
      fail(
        "false notification: " + std::to_string(e.counts.orphan_dispatches) +
        " dispatch attempts without expectation on edge " + E);
    }
    if (e.counts.count_mismatch != 0) {
      fail(
        "false notification: " + std::to_string(e.counts.count_mismatch) +
        " arrive records without expectation on edge " + E);
    }
    uint64_t sampled = 0;
    for (const auto & s : e.counts.samples) {
      sampled += s.second;
    }
    if (sampled != e.counts.samples_taken) {
      fail(
        "edge " + E + ": taken samples " + std::to_string(e.counts.samples_taken) +
        " != classified " + std::to_string(sampled));
    }
    if (e.lag_exceeded != 0) {
      fail(
        "worker_lag_exceeded edge " + E + ": " + std::to_string(e.lag_exceeded) +
        " drain measurement(s) above L = " + std::to_string(lag_limit_locked(e)) +
        " ns, worst lag " + std::to_string(e.lag_worst_ns) + " ns at tag " +
        std::to_string(e.lag_worst_tag) + " point=" + e.lag_worst_point);
    }
    if (e.undrained != 0) {
      fail(
        "worker_lag_exceeded edge " + E + ": " + std::to_string(e.undrained) +
        " undrained carrier token(s) older than release + L at close");
    }
    if (e.undelivered != 0) {
      fail(
        "missing_undelivered edge " + E + ": " + std::to_string(e.undelivered) +
        " missing token(s) never reached a dispatch");
    }
    if (e.burst_worst != 0) {
      fail(
        "burst_exceeded edge " + E + ": " + std::to_string(e.burst_worst) +
        " records in one reaction > B = " + std::to_string(e.burst));
    }
    if (e.flushes_refused != 0) {
      fail(
        "flush_refused edge " + E + ": " + std::to_string(e.flushes_refused) +
        " flush(es) not scheduled, their records were never sent");
    }
    if (e.role == Role::EGRESS &&
      (!e.egress_queue.empty() || e.egress_records != e.egress_sent + e.egress_at_stop))
    {
      const std::string at_stop =
        e.egress_at_stop ? " + " + std::to_string(e.egress_at_stop) + " at stop" : "";
      fail(
        "egress_unsent edge " + E + ": records " + std::to_string(e.egress_records) +
        " != sent " + std::to_string(e.egress_sent) + at_stop + ", " +
        std::to_string(e.egress_queue.size()) + " queued at close" + queued_tags(e));
    }
    if (e.pub_lag_exceeded != 0) {
      fail(
        "publisher_lag_exceeded edge " + E + ": " + std::to_string(e.pub_lag_exceeded) +
        " commit(s) above L = " + std::to_string(lag_limit_locked(e)) + " ns, worst lag " +
        std::to_string(e.pub_lag_worst_ns) + " ns at tag " + std::to_string(e.pub_lag_worst_tag));
    }
    if (!e.control_failures.empty()) {
      // A COMPLETE never arrived, so what that activation
      // published is unknown. Its records never arrived, so no expectation
      // claims that activation's samples.
      std::string seqs;
      for (size_t k = 0; k < e.control_failures.size() && k < 8; ++k) {
        seqs += (k ? "," : "") + std::to_string(e.control_failures[k]);
      }
      fail(
        "control_failure edge " + E + ": " + std::to_string(e.control_failure_count) +
        " activation(s) without a COMPLETE record (" + seqs + ")");
    }
    // The trailing COMPLETE check, in the process.
    // The sender's count of completion numbers landing by the stop tag
    // arrived after LF returned (note_complete_end); every one of them must
    // have arrived here, so a lower complete_high is a CONTROL_FAILURE at the
    // logical end. A missing count fails too, so a lost or stalled sender is
    // never a silent pass.
    if (e.complete_end_expected && !e.complete_end_seen) {
      fail(
        "complete_end_missing edge " + E + ": the sender's end-of-run COMPLETE count never "
        "arrived, so trailing activations without a COMPLETE record cannot be ruled out");
    } else if (e.complete_end_seen && e.complete_high < e.complete_end_due) {
      fail(
        "control_failure edge " + E + ": " +
        std::to_string(e.complete_end_due - e.complete_high) +
        " trailing activation(s) without a COMPLETE record (sender numbered " +
        std::to_string(e.complete_end_sent) + ", " +
        std::to_string(e.complete_end_sent - e.complete_end_due) +
        " past the stop tag, receiver's last " + std::to_string(e.complete_high) + ")");
    } else if (e.complete_end_seen && e.complete_high > e.complete_end_due) {
      fail(
        "complete_accounting edge " + E + ": receiver's last COMPLETE " +
        std::to_string(e.complete_high) + " above the " + std::to_string(e.complete_end_due) +
        " the sender says landed by the stop tag");
    }
    if (e.duplicate_completes != 0) {
      fail(
        "duplicate_complete edge " + E + ": " + std::to_string(e.duplicate_completes) +
        " COMPLETE record(s) not above the last one");
    }
    if (e.clock_skew != 0) {
      fail(
        "xfed_clock edge " + E + ": " + std::to_string(e.clock_skew) +
        " record(s) stamped after the receiver's physical time plus the clock bound");
    }
    for (const auto & why : e.reader_refused) {
      fail("reader_rebind_refused edge " + E + ": " + why);
    }
    if (e.counts.local_undelivered != 0) {
      fail(
        "local_undelivered edge " + E + ": " + std::to_string(e.counts.local_undelivered) +
        " sample(s) not in the local reader after their publish returned");
    }
    carrier += e.carrier.size() + e.terminal.size() + e.in_flight.size();
    r.missing_total += e.counts.missing;
    js << (i ? "," : "") << "{\"edge\":" << i << ",\"delay_ns\":" << e.delay_ns <<
      ",\"timeout_ns\":" << e.timeout_ns <<
      ",\"expectations\":" << e.counts.expectations << ",\"outcomes\":{";
    first = true;
    for (const auto & o : e.counts.outcomes) {
      js << (first ? "" : ",") << "\"" << outcome_name(o.first) << "\":" << o.second;
      first = false;
    }
    js << "},\"samples_taken\":" << e.counts.samples_taken << ",\"samples\":{";
    first = true;
    for (const auto & s : e.counts.samples) {
      js << (first ? "" : ",") << "\"" << sample_outcome_name(s.first) << "\":" << s.second;
      first = false;
    }
    // Every epoch_close sample by identity (see EdgeCounts)
    js << "},\"epoch_close_ids\":[";
    for (size_t m = 0; m < e.counts.epoch_close_ids.size(); ++m) {
      js << (m ? "," : "") << "{\"gid\":\"" << e.counts.epoch_close_ids[m].first <<
        "\",\"ordinal\":" << e.counts.epoch_close_ids[m].second << "}";
    }
    js << "]";
    if (e.counts.epoch_close_ids_truncated) {
      js << ",\"epoch_close_ids_truncated\":true";
    }
    js << ",\"orphan_dispatches\":" << e.counts.orphan_dispatches << ",\"missing_total\":" <<
      e.counts.missing << ",\"missing\":[";
    for (size_t m = 0; m < e.missing.size(); ++m) {
      js << (m ? "," : "") << "{\"gid\":\"" << e.missing[m].first << "\",\"ordinal\":" <<
        e.missing[m].second << "}";
    }
    // The list names at most kMissingListed keys, missing_total counts all.
    js << "],\"missing_truncated\":" << (e.counts.missing > e.missing.size() ? "true" : "false");
    // Joins the stop deadline ended (not missing), and the writers whose
    // newest missing or pending ordinal had no sample when the epoch closed.
    size_t owed = 0;
    for (const auto & s : e.seen) {
      owed += s.second.wanted > s.second.sample ? 1 : 0;
    }
    js << ",\"pending_at_stop\":" << e.counts.pending_at_stop << ",\"pending\":[";
    for (size_t m = 0; m < e.pending.size(); ++m) {
      js << (m ? "," : "") << "{\"gid\":\"" << e.pending[m].first << "\",\"ordinal\":" <<
        e.pending[m].second << "}";
    }
    js << "],\"pending_truncated\":" <<
      (e.counts.pending_at_stop > e.pending.size() ? "true" : "false") <<
      ",\"owed_at_close\":" << owed;
    if (e.local_now) {
      js << ",\"local_now\":true,\"cutoff\":\"none\",\"local_undelivered\":" <<
        e.counts.local_undelivered << ",\"local_undelivered_planted\":" <<
        e.counts.local_undelivered_planted;
    }
    if (e.sub_rebind || e.reader_generation != 0 || !e.reader_refused.empty()) {
      js << ",\"reader_generation\":" << e.reader_generation << ",\"readers_refused\":" <<
        e.reader_refused.size() << ",\"lost_discovery_gap\":" << e.counts.lost_discovery_gap;
    }
    if (e.role == Role::EGRESS) {
      js << ",\"role\":\"egress\",\"egress_records\":" << e.egress_records <<
        ",\"egress_sent\":" << e.egress_sent << ",\"egress_unsent\":" << e.egress_queue.size() <<
        ",\"egress_at_stop\":" << e.egress_at_stop <<
        ",\"flushes_at_stop\":" << e.flushes_at_stop <<
        ",\"publisher_lag_exceeded\":" << e.pub_lag_exceeded <<
        ",\"completes_sent\":" << e.complete_sent <<
        ",\"completes_at_stop\":" << e.completes_at_stop;
    } else if (e.role == Role::INGRESS) {
      js << ",\"role\":\"ingress\",\"duplicate_records\":" << e.duplicate_records <<
        ",\"clock_skew\":" << e.clock_skew <<
        ",\"completes\":" << e.completes <<
        ",\"control_failures\":" << e.control_failure_count <<
        ",\"complete_high\":" << e.complete_high;
      if (e.complete_end_expected) {
        js << ",\"complete_end\":" << (e.complete_end_seen ? "true" : "false") <<
          ",\"complete_end_due\":" << e.complete_end_due;
      }
    }
    js << "}";
  }
  if (retained_total_ != 0) {
    fail("retained " + std::to_string(retained_total_) + " samples at close");
  }
  if (stalls_ != 0) {
    fail(
      "control_failure stall: logical time stood still past the stall bound " +
      std::to_string(stalls_) + " time(s), an upstream federate stopped advancing");
  }
  if (carrier != 0) {
    fail("carrier occupancy " + std::to_string(carrier) + " at close");
  }
  if (identity_faults_ != 0) {
    fail("identity fault: " + std::to_string(identity_faults_));
  }
  for (const auto & [lead, g] : groups_) {
    if (g.order_violations != 0) {
      fail(
        "order_violations group " + std::to_string(lead) + ": " +
        std::to_string(g.order_violations) + " pop(s) below an earlier key of the group");
    }
  }
  js << "],\"stalls\":" << stalls_ << ",\"retained\":" << retained_total_ <<
    ",\"carrier\":" << carrier <<
    ",\"identity_faults\":" << identity_faults_ << ",\"duplicate_tokens\":" << duplicate_tokens_ <<
    ",\"rejected_tokens\":" << rejected_tokens_;
  // Only a run with a fan-in group carries the key.
  first = true;
  for (const auto & [lead, g] : groups_) {
    if (g.members.size() < 2) {
      continue;
    }
    js << (first ? ",\"groups\":[" : ",") << "{\"lead\":" << lead << ",\"members\":[";
    for (size_t m = 0; m < g.members.size(); ++m) {
      js << (m ? "," : "") << g.members[m];
    }
    js << "],\"order_violations\":" << g.order_violations << "}";
    first = false;
  }
  // `reconciled` is the accounting verdict (Report::ok) alone,
  // the same meaning as the exit code and the `run_end.reconciled` trace
  // field; `missing_total` is the separate fault count beside it.
  js << (first ? "" : "]") << ",\"missing_total\":" << r.missing_total <<
    ",\"reconciled\":" << (r.ok ? "true" : "false") << "}";
  r.summary = js.str();
  return r;
}

}  // namespace corr
}  // namespace ros2_lf_binding

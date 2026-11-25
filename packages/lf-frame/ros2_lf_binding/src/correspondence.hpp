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

/// @file correspondence.hpp
/// @brief Publication correspondence state (D3-D7), pure C++.
///
/// One table per process run (epoch). The publication gate predicts and
/// commits publications; ACCEPTED publications of an admitted origin create
/// one expectation per admitted edge and one in-flight token per expectation.
/// The subscriber's arrive reaction moves tokens into the edge's carrier, its
/// drain reaction pops released tokens and joins each with the sample ingress
/// retained for exactly that (GID, ordinal, edge). Every expectation ends in
/// exactly one terminal outcome; reconcile() checks the D7 invariants.

#ifndef CORRESPONDENCE_HPP_
#define CORRESPONDENCE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace ros2_lf_binding
{
namespace corr
{

constexpr size_t kCarrierCap = 8;      ///< carrier per edge when none is generated
constexpr size_t kTerminalCap = 64;    ///< per edge, already-missing tokens past the carrier cap
constexpr size_t kRetainPerEdge = 8;   ///< retain per edge when none is generated
constexpr size_t kRetainTotal = 256;   ///< floor of the process cap (Table::retain_total_cap_)
constexpr size_t kTombstones = 64;     ///< terminal keys remembered per edge
/// Missing (and pending) keys named per edge in the reconcile JSON.
/// CONVENTION, it bounds the one-line reconcile summary, not the evidence:
/// `missing_total` and `pending_at_stop` count every key, and
/// `missing_truncated` / `pending_truncated` say when the list is cut.
constexpr size_t kMissingListed = 64;
/// Ended publications a local writer's history keeps. CONVENTION, a
/// memory bound only: a publication with an open expectation is never pruned
/// (Table::prune_locked), so the value decides only whether a stray sample of
/// a publication whose every expectation ended reads DUPLICATE or
/// UNSUPPORTED (kept) or LATE (pruned).
constexpr uint64_t kWriterHistory = 1024;
/// Commits between two sweeps of a writer's open
/// publications older than its kWriterHistory window (at least; the period is
/// max(kHeldRecheck, held)). CONVENTION, it amortizes the sweep to O(1) per
/// commit and bounds how long an ended one of them stays in the history.
constexpr uint64_t kHeldRecheck = 64;
constexpr int64_t kNoCutoff = std::numeric_limits<int64_t>::max();
/// The last microstep of a time: every release at that time is due.
constexpr uint32_t kLastMicrostep = std::numeric_limits<uint32_t>::max();

using Gid = std::array<uint8_t, 16>;
/// Explicit comparator: rclcpp_action specializes std::less for this exact
/// array type (GoalUUID), so std::less<Gid> must never be instantiated here.
struct GidLess
{
  bool operator()(const Gid & a, const Gid & b) const {return a < b;}
};
std::string gid_hex(const Gid & gid);

/// Expectation outcomes: dispatch outcomes plus the carrier/close terminals.
enum class Outcome : uint8_t
{
  PENDING,
  MATCHED,
  NOT_READY,
  TIMED_OUT,
  LOST,
  REJECTED,
  UNBOUND,
  UNSUPPORTED,
  DUPLICATE,
  OVERFLOW,
  EPOCH_CLOSE,
  PUSH_OVERFLOW,
  SCHEDULE_REFUSED,
  /// Published to a reader of a lifecycle subscription that its node
  /// dropped or replaced before the record could be taken. Not a missing
  /// message (no hook runs) and never a successful take.
  READER_REPLACED,
};
const char * outcome_name(Outcome o);
bool is_terminal(Outcome o);
/// the outcomes a dispatch delivers as a missing message.
bool is_missing(Outcome o);

/// What ingress did with one taken sample (D5 table).
enum class SampleOutcome : uint8_t
{
  RETAINED,
  UNSUPPORTED,        ///< publication of a refused origin (zero expectations on this edge)
  UNRESOLVED,         ///< GID unknown to the gate (external writer, other process, previous run)
  LATE,               ///< expectation terminal, or older than the tombstone ring
  DUPLICATE,          ///< expectation MATCHED, or the same key already retained
  REJECTED_IDENTITY,  ///< ordinal above the last assigned, or a FAILED ordinal: epoch fault
  OVERFLOW,           ///< retention full: released, expectation OVERFLOW
  EPOCH_CLOSE,        ///< retained at close, released
  /// Taken by a reader of a `sub_rebind` edge for a record published
  /// after a later reader of the edge was created. That record is the later
  /// reader's, so this sample is released and never matched.
  STALE_READER,
  /// Taken by a fan-in group's reader for a writer no member knows yet.
  /// It waits in the group's early pool and is counted on its member once
  /// routed (or UNRESOLVED on the lead), never on an edge as POOLED.
  POOLED,
};
const char * sample_outcome_name(SampleOutcome o);

/// Why a publication created no expectation.
enum class Origin : uint8_t
{
  ADMITTED,
  NO_CONTEXT,
  UNDECLARED_EFFECT,
  EDGE_NOT_ADMITTED,
  UNSUPPORTED_API,
  NESTED,
};
const char * origin_name(Origin o);

/// A local edge has both ends in this process. An egress edge
/// has only the publisher here and sends records over the LF federated
/// connection, an ingress edge has only the subscriber and builds its
/// expectations from those records.
enum class Role : uint8_t
{
  LOCAL,
  EGRESS,
  INGRESS,
};
const char * role_name(Role r);

/// One publication record of a cross-federate edge.
struct XRecord
{
  Gid gid{};
  uint64_t ordinal = 0;
  int64_t phys = 0;  ///< physical commit stamp at the publisher
  int64_t tag = -1;  ///< logical time of the publication (egress only, not on the wire)
  /// Activation origin: S(a), the activation window start, -1 without one. Since
  /// The ingress cutoff counts from the publication tag, not from it.
  int64_t act = -1;
};

/// Result of an egress emit.
enum class EmitStatus : uint8_t
{
  OK,
  COLLAPSE,  ///< records of this time already sent at another microstep
  CAP,       ///< more records at one time than the message holds
  STALE,     ///< a queued record is of another time than the flush
};

struct Token
{
  uint64_t epoch = 0;
  Gid gid{};
  uint64_t ordinal = 0;
  uint32_t edge = 0;
  uint32_t instance = 0;
  uint32_t generation = 0;
};

/// A carrier or terminal entry, the token with its
/// release tag lf_delay_tag(publication tag, d), as reactor-c defines it:
/// (t, m + 1) for d = 0, (t + d, 0) for d > 0.
struct Queued
{
  int64_t release_ns = 0;
  uint32_t release_micro = 0;
  Token token;
};

/// A retained sample, type-erased: message and rclcpp::MessageInfo.
struct Sample
{
  std::shared_ptr<void> message;
  std::shared_ptr<void> info;
};

struct EdgeCounts
{
  uint64_t expectations = 0;
  std::map<Outcome, uint64_t> outcomes;
  uint64_t samples_taken = 0;
  std::map<SampleOutcome, uint64_t> samples;
  uint64_t orphan_dispatches = 0;  ///< dispatch attempts without expectation
  uint64_t count_mismatch = 0;     ///< arrive count != in-flight records
  uint64_t missing = 0;            ///< dispatches that delivered a missing outcome
  /// Records of which a replaced reader took a copy (STALE_READER) and
  /// that then ended missing: the new reader had not matched the writer yet.
  /// A discovery-gap loss with no old-reader copy is an ordinary missing outcome.
  uint64_t lost_discovery_gap = 0;
  /// Joins below the stop tag that the stop deadline ended with their
  /// sample still in flight. EPOCH_CLOSE, not missing.
  uint64_t pending_at_stop = 0;
  /// local_now expectations ended LOST because the sample was not in the
  /// reader after its publish returned (the Fast DDS premise broke). A
  /// reconcile problem. `_planted`: a test hook dropped it, not a problem.
  uint64_t local_undelivered = 0;
  uint64_t local_undelivered_planted = 0;
  /// The identity (gid hex, ordinal) of each sample counted
  /// EPOCH_CLOSE, in the order counted, at most kEpochCloseIdsCap of them.
  /// A sample retained before its stop refusal moves to EPOCH_CLOSE in
  /// schedule_refused with no trace of its own; this names it in the
  /// reconcile line so the gate can prove the epoch_close samples are the
  /// refused tokens, not a count that happens to match.
  std::vector<std::pair<std::string, uint64_t>> epoch_close_ids;
  bool epoch_close_ids_truncated = false;
};

constexpr size_t kEpochCloseIdsCap = 256;

/// A missing sample (NOT_READY, TIMED_OUT, LOST) is logged as a WARN
/// when it is decided, not only in the record at stop: the first one of an
/// edge at once, then at most one line per edge per this period, carrying
/// the count since the last line. One line a second keeps a 1 kHz edge with
/// a dead writer at one line per second instead of a thousand, and a single
/// loss is never held back.
constexpr int64_t kMissingWarnPeriodNs = 1000000000;

/// The record at stop (`summary`). `ok` is the accounting verdict: every
/// token, dispatch and record balanced; the JSON's `"reconciled"`, the
/// runtime's exit code and the `run_end.reconciled` trace field all carry
/// exactly it. `"missing_total"` is the separate fault count: the
/// missing samples of all edges, so a run can be reconciled and still have
/// lost samples, and a reader that wants "no fault" checks both.
struct Report
{
  bool ok = true;
  uint64_t missing_total = 0;  ///< NOT_READY + TIMED_OUT + LOST of all edges
  std::vector<std::string> problems;
  std::string summary;  ///< one JSON object
};

/// Semantic trace hooks, called under the table lock. They must
/// not call back into the table; the rcllf emit layer is lock-free.
class Observer
{
public:
  virtual ~Observer() = default;
  /// An ACCEPTED admitted publication created this expectation.
  virtual void on_expectation(const Token & token) = 0;
  /// The expectation (edge, gid, ordinal) reached its one terminal outcome.
  virtual void on_terminal(uint32_t edge, const Gid & gid, uint64_t ordinal, Outcome outcome) = 0;
  /// A sample left a fan-in group's early pool, routed to member `edge`
  /// with its intake `outcome` there (`retained_id` set when RETAINED), or
  /// released on the lead `edge` UNRESOLVED. `stamp` is its take stamp.
  virtual void on_pool_exit(
    uint32_t /*edge*/, const Gid & /*gid*/, uint64_t /*ordinal*/, SampleOutcome /*outcome*/,
    uint64_t /*retained_id*/, int64_t /*stamp*/) {}
};

/// Stall watch: a stalled upstream federate. Under centralized
/// coordination the RTI holds this federate's next tag until the upstream
/// advances, so a stalled sender never shows as a missing record, only as
/// logical time standing still. progress() is called at every dispatch,
/// check() by the watchdog: true once per episode, when the tag has not
/// changed for more than the bound. Not thread-safe, the caller locks.
class StallWatch
{
public:
  /// A dispatch at (`tag`, `microstep`) on physical time `now`. A new
  /// microstep at the same time is progress too.
  void progress(int64_t tag, uint32_t microstep, int64_t now)
  {
    if (!started_ || tag != tag_ || microstep != micro_) {
      started_ = true;
      tag_ = tag;
      micro_ = microstep;
      since_ = now;
      reported_ = false;
    }
  }
  /// True when the tag stood still past `bound_ns` and this episode was not
  /// reported yet. A bound below 0 disables the watch.
  bool check(int64_t now, int64_t bound_ns)
  {
    if (!started_ || bound_ns < 0 || reported_ || now - since_ <= bound_ns) {
      return false;
    }
    reported_ = true;
    return true;
  }
  int64_t tag() const {return tag_;}
  int64_t since() const {return since_;}

private:
  bool started_ = false;
  bool reported_ = false;
  int64_t tag_ = 0;
  uint32_t micro_ = 0;
  int64_t since_ = 0;
};

class Table
{
public:
  explicit Table(uint64_t epoch);

  /// Expectation and terminal events. nullptr detaches.
  void set_observer(Observer * observer) {observer_ = observer;}

  uint64_t epoch() const {return epoch_;}

  /// Physical clock in ns for the cutoff stamps and the lag check.
  /// The runtime installs lf_time_physical; the default is steady_clock.
  void set_clock(std::function<int64_t()> clock);
  /// true while the clock is the receiver's logical tag (a fast
  /// federate, `-f true`). The clock bound eps is 0 then (the ingress
  /// cutoff is t + max(0, d) + T + eps in both modes). Empty is false.
  void set_fast(std::function<bool()> fast);

  /// Edges are dense from 0; `instance` identifies the subscription.
  /// `timeout_ns` is T_e (cutoff = publish moment + d_e + T_e,
  /// the tag in a fast federate), -1 keeps
  /// the legacy bounded join. `max_lag_ns` is L_e, -1 means 2 * max T_e.
  /// `burst` is B_e, 0 means not checked. `retain` and `carrier` are the
  /// schedule-derived per-edge capacities (S2), 0 keeps kRetainPerEdge and
  /// kCarrierCap. The process retention cap grows to the sum of the edges'.
  void add_edge(
    uint32_t edge, uint32_t instance, int64_t delay_ns, int64_t timeout_ns = -1,
    int64_t max_lag_ns = -1, uint32_t burst = 0, uint32_t retain = 0, uint32_t carrier = 0,
    Role role = Role::LOCAL);
  size_t edge_count() const;

  // --- publisher side (gate) ------------------------------------------------
  /// A publisher the gate saw (init or first publish): its GID is "known".
  void register_gid(const Gid & gid);
  /// Before the real call: PENDING(ordinal). `edges` empty unless ADMITTED.
  /// `tracked` false: the publisher is on no admitted edge, so its
  /// publication counts as untracked, not as a refused origin.
  /// `tag_ns` (absolute LF tag of the publishing reaction, -1 unknown) fixes
  /// the cutoff tag + d_e + T_e of every edge with a timeout.
  void predict(
    const Gid & gid, uint64_t ordinal, Origin origin, std::vector<uint32_t> edges,
    bool tracked = true, int64_t tag_ns = -1, int64_t act_ns = -1);
  /// After the real call. ok: ACCEPTED, expectations and in-flight tokens for
  /// every predicted edge. !ok: FAILED, the GID is poisoned for the epoch.
  void commit(const Gid & gid, uint64_t ordinal, bool ok);
  /// Nested (re-entrant) publish: counted, ACCEPTED, refused origin
  /// (untracked when `tracked` is false).
  void note_nested(const Gid & gid, uint64_t ordinal, bool tracked = true);
  /// rcl_publisher_fini: the GID is finalized; reuse faults the epoch.
  void finalize(const Gid & gid);
  bool poisoned(const Gid & gid) const;

  // --- LF worker side ---------------------------------------------------------
  /// Records created on `edge` since the last arrive (the emit count).
  uint32_t in_flight(uint32_t edge) const;
  /// Emit side: in_flight(edge), and in `*fresh` the records
  /// added since the previous emit on `edge`. Several publishing reactions of
  /// one reactor write one port at a tag; LF keeps the last value, which is
  /// therefore the cumulative count, while `*fresh` is this reaction's share.
  /// `tokens`, when given, receives the fresh records' tokens, in publication
  /// order (the publication-keyed emit trace).
  uint32_t emit(uint32_t edge, uint32_t * fresh, std::vector<Token> * tokens = nullptr);
  /// End of one callback invocation in a drain reaction that runs its
  /// callback once per record: checks this thread's records per edge against
  /// B_e and clears the tally.
  void end_invocation();
  /// Move exactly `count` in-flight tokens into the carrier, released at
  /// `release_ns`. Returns true when the caller must schedule a wake (the
  /// carrier went nonempty with no wake pending; for a fan-in group, with no
  /// wake pending for the group). `pub_microstep` is the arrive's microstep,
  /// the publication's own (zero-delay link). The release microstep follows
  /// lf_delay_tag: m + 1 when the release is the publication time (d = 0),
  /// else 0, as `after d > 0` maps every microstep of t to (t + d, 0).
  /// m itself on a same-tag edge (set_same_tag).
  bool arrive(uint32_t edge, uint32_t count, int64_t release_ns, uint32_t pub_microstep = 0);
  /// The microstep of the release tag `arrive` files a record of
  /// `edge` published at microstep `pub_microstep` under (for the trace).
  uint32_t release_microstep(uint32_t edge, uint32_t pub_microstep) const;
  /// Pop the smallest released entry at the tag (`now_ns`, `now_micro`):
  /// key (release tag, rank, ordinal) over the carrier and terminal queue of
  /// every member, carrier first on ties. Called
  /// with any member of a fan-in group (the runtime uses the lead), a lone
  /// edge is a group of one; Token::edge names the member.
  std::optional<Token> pop_released(
    uint32_t edge, int64_t now_ns, uint32_t now_micro = kLastMicrostep);
  /// Next wake after a drain: 0 (a released head remains), the gap to the next
  /// head's release, or -1 (carrier empty, no wake pending). A group: over
  /// every member's carrier.
  int64_t next_wake(uint32_t edge, int64_t now_ns);
  /// lf_schedule refused: every carrier token ends SCHEDULE_REFUSED (every
  /// member's, for a group). `stop_time`/`stop_micro` is the stop tag
  /// (the default knows none). A token whose release tag lies past it ended
  /// at the stop: its sample, retained or taken later, counts EPOCH_CLOSE.
  void schedule_refused(
    uint32_t edge, int64_t stop_time = std::numeric_limits<int64_t>::max(),
    uint32_t stop_micro = 0);
  size_t carrier_size(uint32_t edge) const;
  /// The carrier and terminal entries of `edge`: what a
  /// schedule_refused on it would end, read before it does, for the trace.
  /// `terminal` receives the terminal queue (already lost, counted undelivered).
  std::vector<Queued> carrier_entries(uint32_t edge, std::vector<Queued> * terminal) const;

  // --- fan-in groups -----------------------------------------------------
  /// `edge` joins the fan-in group led by `lead` (the group's lowest edge id,
  /// registered with itself as lead) with `rank`, its publisher's position in
  /// the instance-name order. A group shares one drain wake, one reader and
  /// one early pool. Refused (false, reason in `*why`) for an egress edge, an
  /// edge already in another group, and a role other than the lead's: a
  /// group is all local or all ingress (premise P5).
  bool set_fan_in(uint32_t edge, uint32_t lead, uint32_t rank, std::string * why = nullptr);
  /// The members of `edge`'s group, ascending; {edge} outside a group.
  std::vector<uint32_t> group_members(uint32_t edge) const;
  /// Pops whose key was below an earlier pop of the group (premise breach).
  uint64_t order_violations(uint32_t edge) const;
  /// The member of `edge`'s group that owns writer `gid`, as far as the group
  /// knows it now; `edge` outside a group or for a writer no member knows.
  uint32_t sample_edge(uint32_t edge, const Gid & gid) const;
  /// A sample the group's one reader took (the table owns
  /// GID routing). Resolved under the table's lock to the member that owns
  /// writer `gid` (`*member`), and taken and classified there: a local group
  /// learns a writer's member at predict (its predicted edges among the
  /// members), an ingress group from the first record that names it. An
  /// ingress group parks a sample of a writer no member knows in its early
  /// pool (POOLED, `*member` the lead); it moves to its member with its take
  /// stamp when that member learns the GID, or ends UNRESOLVED on the lead
  /// when no member claims it within the members' widest early window, when
  /// the pool is full, or at close. A sample no member resolves otherwise
  /// (a second take of a parked one, a full pool, a closed table, a writer
  /// of no member) is UNRESOLVED on the lead, the one outcome counted there.
  /// The F3 hook (set_drop_sample_for_test) runs on the resolved member, and
  /// at adoption for a pooled sample: `*dropped` is set and nothing counted.
  /// Outside a group: the plain take of edge `lead`.
  SampleOutcome on_group_sample(
    uint32_t lead, const Gid & gid, uint64_t ordinal, Sample sample, uint32_t * member,
    uint64_t * retained_id = nullptr, int64_t * stamp_ns = nullptr,
    std::optional<int64_t> reader_created = std::nullopt, bool * dropped = nullptr);

  // --- cross-federate edges -----------------------------------------------
  /// Egress: the records of logical `time` for one message, cumulative over
  /// the reactions at (`time`, `microstep`), at most `cap`. `fresh` counts the
  /// records not handed out before. `per_microstep`: the
  /// caller sends at the publishing tag, so another microstep of `time` is a
  /// new message, not the D2 collapse. No record
  /// queued means no message, `out` stays empty. Every message is a COMPLETE
  /// record and `complete_seq` (when given) receives its completion number, 0
  /// when there is no message. The flush (`per_microstep` false) sends
  /// one message per time, numbered at microstep 0.
  EmitStatus emit_remote(
    uint32_t edge, int64_t time, uint32_t microstep, size_t cap, std::vector<XRecord> * out,
    uint32_t * fresh, bool per_microstep = false, uint64_t * complete_seq = nullptr);
  /// Ingress: one expectation and token per new record, cutoff
  /// `pub_time + d_e + T_e`, the tokens released at `release_ns`. Adopts early
  /// samples. Returns true when the drain needs a wake.
  /// The cutoff of every ingress record counts from the publication tag,
  /// pub_time + max(0, d_e) + T_e + eps (maxwait).
  /// `pub_microstep` is the publishing tag's microstep, so a d = 0
  /// record is keyed at (pub_time, pub_microstep + 1) as in one process.
  bool arrive_remote(
    uint32_t edge, const std::vector<XRecord> & records, int64_t pub_time, int64_t release_ns,
    uint32_t pub_microstep);
  /// Egress, the completion number of
  /// the message sent at (`time`, `microstep`). An activation that published
  /// on the edge sends one COMPLETE message (changed 2026-11-03: one that
  /// published nothing sends none, emit_remote numbers only real messages).
  /// A new tag takes the next number from 1. A second send at the same tag
  /// keeps it, because LF keeps the last lf_set of a tag and that message
  /// carries the records of both reactions.
  uint64_t complete_seq(uint32_t edge, int64_t time, uint32_t microstep);
  /// The outcome of one ingress COMPLETE message (note_complete).
  struct CompleteResult
  {
    /// The completion numbers between the last one received and this one.
    /// Their messages never arrived, so what they published is unknown:
    /// CONTROL_FAILURE per number, ascending.
    std::vector<uint64_t> control_failures;
    /// Every missing sequence number of this message, also past the listed ones.
    uint64_t control_failure_count = 0;
    bool duplicate = false;  ///< `seq` not above the last one received
  };
  /// Ingress: COMPLETE number `seq` of the publishing activation
  /// arrived (with at least one record, changed 2026-11-03). The expectation
  /// of every number below `seq` exists from the sequence alone, whether its
  /// message came or not.
  CompleteResult note_complete(uint32_t edge, uint64_t seq);
  /// The end-of-run count of a cross-federate edge.
  /// A COMPLETE lost after the receiver's last one leaves no sequence gap, so
  /// the sender reports, after LF returned, how many completion numbers land
  /// by the stop tag (`due`, completes_sent - completes_at_stop).
  struct CompleteEnd
  {
    uint32_t edge = 0;
    uint64_t due = 0;   ///< completion numbers that land at or before the stop tag
    uint64_t sent = 0;  ///< completion numbers the sender gave out
  };
  /// Egress, after close_epoch: one CompleteEnd per egress edge.
  std::vector<CompleteEnd> complete_ends() const;
  /// Ingress: `edge` is fed by a sender in another process, so the reconcile
  /// fails unless that sender's CompleteEnd arrived (note_complete_end).
  void expect_complete_end(uint32_t edge);
  /// Ingress: the sender's CompleteEnd for `edge`. The reconcile compares its
  /// `due` with complete_high: lower is a trailing CONTROL_FAILURE, higher a
  /// counting fault. A second one for the same edge is ignored.
  void note_complete_end(const CompleteEnd & end);
  /// Ingress edges whose CompleteEnd is expected and has not arrived.
  std::vector<uint32_t> complete_ends_missing() const;
  /// Ingress under a declared clock bound eps: a record stamped more than
  /// eps after the receiver's physical time, a reconcile problem.
  void note_clock_skew(uint32_t edge);
  /// Ingress `edge` judges records from a publisher whose physical clock
  /// is within `eps_ns` of this one (declared, not measured). The absence
  /// cutoff is t + max(0, d_e) + T_e + eps, t the publication tag.
  void set_clock_bound(uint32_t edge, int64_t eps_ns);
  /// Local `edge` is a self edge, one node publishes and subscribes it in
  /// this process. A self edge is a local_now edge (set_local_now), so
  /// it has no cutoff and its join never waits. On a `sub_rebind` edge, which
  /// cannot be local_now, a sample of a predicted publication is still
  /// retained and matched without the cutoff test.
  void set_self_local(uint32_t edge);
  /// Whether `edge` is a local self edge.
  bool self_local(uint32_t edge) const;
  /// Local `edge` is decided when its publish returns. Right after a
  /// successful rcl_publish the publishing thread takes the sample from the
  /// subscriber's reader (local_now_edges), then settle_local ends every
  /// expectation still without a sample LOST (local_undelivered). The edge has
  /// no cutoff (kNoCutoff at predict), so LATE, TIMED_OUT and NOT_READY cannot
  /// happen on it, and its join never waits. T_e only sizes L and retention.
  /// Ignored (local_now stays false) on an egress or ingress row, on a
  /// `sub_rebind` edge (a pending reader cannot be taken at publish
  /// time) and on a deferred edge. set_sub_rebind clears it.
  void set_local_now(uint32_t edge);
  /// Whether `edge` is decided at publish time.
  bool local_now(uint32_t edge) const;
  /// The readers to take from after commit(gid, ordinal, true): the
  /// local_now edges of that publication, a fan-in member replaced by
  /// its group lead, distinct and ascending (the take-mutex lock order). Also
  /// valid after predict. Empty for a failed, unknown or unadmitted publication.
  std::vector<uint32_t> local_now_edges(const Gid & gid, uint64_t ordinal) const;
  /// After the publish-time take, called with the readers' take mutexes
  /// held. Every local_now expectation of (gid, ordinal) that has a token, no
  /// sample and no terminal outcome ends LOST, reason local_undelivered. An
  /// edge in `planted` (its own edge id; a fan-in member is named itself, its
  /// lead's id covers only the lead's edge) had its sample
  /// dropped by a test hook: counted local_undelivered_planted, not a reconcile
  /// problem. Any other local_undelivered is a reconcile problem (exit 3).
  /// Returns how many expectations it ended.
  size_t settle_local(const Gid & gid, uint64_t ordinal, const std::set<uint32_t> & planted);
  /// Whether a local_now expectation of this
  /// accepted publication still has neither its sample nor an outcome, so the
  /// publish-time take must go on. False once every one is retained or ended.
  /// With `reader_edge` set, only the edges
  /// whose reader is admitted on `reader_edge` (its own id, or its fan-in
  /// group's lead) count, so one reader's take stops at its own sample.
  bool local_open(
    const Gid & gid, uint64_t ordinal, uint32_t reader_edge = kNoLead) const;
  /// The subscriber of `edge` joins deferred (exec-config `join: deferred`).
  /// Its drain runs at the cutoff tag, publish tag + d_e + T_e, and never waits
  /// for an absent sample. A sample is eligible only when its intake stamp is
  /// strictly below the cutoff. Refused (false) on an egress row, which has no
  /// subscriber here, on a local self edge, whose sample is present, and on a
  /// local_now edge, which is decided at publish.
  bool set_deferred(uint32_t edge);
  /// Whether `edge` joins deferred.
  bool deferred(uint32_t edge) const;
  /// `edge` is drained by a reaction its record port triggers, so a
  /// d = 0 record releases at its publication tag (t, m), not (t, m + 1).
  /// False (and unset) on a self, cross-federate, delayed or deferred edge.
  bool set_same_tag(uint32_t edge);
  /// Whether `edge` releases its d = 0 records at the publication tag.
  bool same_tag(uint32_t edge) const;
  /// Ingress `edge` runs over a plain zero-delay federated connection
  /// (centralized, d = 0), so its records arrive at their publication tag
  /// (t, m) and release there, not at (t, m + 1) as over `after 0`. False
  /// (and unset) on any other edge.
  bool set_zero_link(uint32_t edge);
  /// Whether ingress `edge` releases its records at the publication tag.
  bool zero_link(uint32_t edge) const;
  /// What the release tag of `edge` adds to its after-delay: T_e for a
  /// deferred edge with a timeout, else 0.
  int64_t release_extra_ns(uint32_t edge) const;
  /// The join of `token` would not wait, because its sample is retained,
  /// its expectation ended or the table closed. True for a token join rejects.
  bool sample_ready(const Token & token) const;
  /// Egress: true once per `time` when records are queued, the
  /// publishing reaction then schedules the flush at `time` + 1 ns. Called once
  /// per publishing reaction, so it also checks B_e on that reaction's records.
  bool flush_due(uint32_t edge, int64_t time);
  /// Egress: lf_schedule refused the flush (past the stop tag), so the
  /// queued records stay unsent. Reported at reconcile.
  void flush_refused(uint32_t edge);

  /// Join (D6). With a cutoff the join waits until the cutoff and
  /// matches only a sample stamped at or before it: NOT_READY when the cutoff
  /// passed at entry, TIMED_OUT when it passed during the wait. Without one,
  /// bound is the max wait. On MATCHED the retained sample is moved into
  /// *out exactly once.
  /// `retained_id` (optional) receives the id of the consumed sample on MATCHED.
  Outcome join(
    const Token & token, std::chrono::milliseconds bound, Sample * out,
    uint64_t * retained_id = nullptr);
  /// A join waiting when the stop was requested.
  struct PendingJoin
  {
    uint32_t edge;
    std::string gid;
    uint64_t ordinal;
    int64_t pub_tag;
  };
  /// The LF stop was requested. A join pending for
  /// a tag below the stop tag keeps waiting to its cutoff, but no longer than
  /// `bound_ns` from now: past that it ends EPOCH_CLOSE and is counted
  /// pending_at_stop on its edge, not missing. Later joins get the same
  /// deadline. Returns the joins waiting now. The first call sets the deadline.
  std::vector<PendingJoin> note_stop_requested(int64_t bound_ns);
  /// After LF returned and before the epoch closes, wait up to
  /// `bound_ns` for the samples of expectations that ended missing or pending
  /// at stop and whose writer has delivered nothing newer: they were in flight
  /// at the stop and are recorded late, not never. Returns how many writers
  /// still owe a sample.
  size_t drain_in_flight(int64_t bound_ns);
  /// The subscription's RID is unbound or expired: the token ends UNBOUND.
  void mark(const Token & token, Outcome outcome);
  /// The publish tag of a token's expectation, -1 when unknown.
  int64_t pub_tag(const Token & token) const;
  /// End the expectations of `records`, which arrived after
  /// their intended tag on a decentralized ingress edge, LOST, so the drain
  /// reports each through the subscription's missing hooks. Call after
  /// arrive_remote of the same records.
  void terminate_tardy(uint32_t edge, const std::vector<XRecord> & records);
  /// Whether the token's record arrived after its intended tag.
  bool is_tardy(const Token & token) const;
  /// A new reader of `edge`'s lifecycle subscription was refused
  /// (`code`: rebind_identity, intra_process, durability, reader_depth). A
  /// reconcile problem, so the run cannot claim the edge deterministic.
  void note_reader_refused(uint32_t edge, const std::string & code, const std::string & detail);
  /// `edge`'s subscription is a lifecycle node's, made or reset in a
  /// hook, so its reader is bound per generation (rebind_settle applies).
  void set_sub_rebind(uint32_t edge);
  /// Generation `generation` of `edge`'s reader was admitted; the
  /// reader was created at tag `created_tag` (INT64_MIN before LF ran).
  void note_reader_generation(uint32_t edge, uint32_t generation, int64_t created_tag);
  /// A subscription of `edge`'s node on its topic was created at tag
  /// `created_tag`, on an LF worker. Noted at creation, before any publish of
  /// a later tag, so a sample's reader is judged against it (STALE_READER)
  /// before the new reader is bound.
  void note_reader_created(uint32_t edge, int64_t created_tag);
  /// Drain rule for a `sub_rebind` edge, `bound`: the subscription RID
  /// holds a live handle now. Ends the token and returns its outcome when the
  /// bound reader cannot hold the record: the edge's reader was refused
  /// (UNBOUND); no reader now, or the record was published at or before the
  /// bound reader's creation tag (READER_REPLACED after an earlier reader,
  /// else UNBOUND). A retained sample of it is released. Otherwise nullopt,
  /// and the drain joins as usual.
  std::optional<Outcome> rebind_settle(const Token & token, bool bound);
  /// The cutoff of a token's expectation, kNoCutoff when none (trace only).
  int64_t cutoff(const Token & token) const;
  /// Lag check: physical minus logical time `lag_ns` of a drain on `edge`
  /// at tag `tag_ns`, measured at `point` ("start", "join", "complete").
  /// A lag above L_e is a reconcile problem (worker_lag_exceeded).
  void note_lag(uint32_t edge, int64_t tag_ns, int64_t lag_ns, const char * point);
  int64_t max_lag_ns(uint32_t edge) const;

  // --- ingress side -------------------------------------------------------------
  /// `retained_id` (optional) receives the per-epoch id of a RETAINED sample.
  /// The sample is stamped with the table clock under the table lock: a
  /// stamp past the expectation's cutoff is LATE and never matched.
  /// `stamp_ns` (optional) receives that stamp.
  /// `reader_created`: the creation tag of the reader that took the
  /// sample, for a `sub_rebind` edge's reader.
  SampleOutcome on_sample(
    uint32_t edge, const Gid & gid, uint64_t ordinal, Sample sample,
    uint64_t * retained_id = nullptr, int64_t * stamp_ns = nullptr,
    std::optional<int64_t> reader_created = std::nullopt);
  size_t retained() const;
  /// Publications `gid`'s writer history holds (0 for an unknown
  /// writer). A diagnostic of the memory bound stated at Table::prune_locked.
  size_t writer_history(const Gid & gid) const;

  // --- epoch close (D7) ---------------------------------------------------------
  /// `stop_time` is the stop tag's time. An egress record of a
  /// time at or after it has its flush at (time + 1 ns, 0), past the stop tag,
  /// so it is retained at stop, not unsent. The default knows no stop tag.
  /// `stop_micro` is the stop tag's microstep, used by d = 0 edges.
  void close_epoch(
    int64_t stop_time = std::numeric_limits<int64_t>::max(), uint32_t stop_micro = 0);
  bool closed() const;
  Report reconcile() const;
  /// Stall watch: the stall bound W of this federate, or -1 (no watch).
  /// W = P + sum over ingress edges of (d + T) + the lag limit, P the
  /// longest timer period here. -1 without a timer, without an ingress edge,
  /// or with an ingress edge of infinite T (a wait there has no bound).
  int64_t stall_bound_ns(int64_t max_timer_period_ns) const;
  /// The part of W without the timer period, the
  /// sum over ingress edges of (d + T + eps) plus the lag limit, or -1 under
  /// the same conditions as stall_bound_ns minus the timer one. A timer-free
  /// receiver past its stop tag by more than this has a stalled upstream.
  int64_t stall_window_ns() const;
  /// Stall watch: one stall episode, reported as a CONTROL_FAILURE.
  void note_stall();
  uint64_t stalls() const;
  /// Joins that entered the timed wait (wait_until_locked), all edges. A join
  /// decided from the table as it stands (local_now, deferred, ready) adds
  /// none; tests check this instead of a wall-clock bound (CI flakes).
  uint64_t join_waits() const;
  EdgeCounts counts(uint32_t edge) const;
  uint64_t identity_faults() const;
  /// Retained samples released without a dispatch.
  uint64_t released_retained() const;
  /// Expectations that ended EPOCH_CLOSE, all edges.
  uint64_t epoch_close_terminals() const;
  /// One entry per expectation close_epoch ended EPOCH_CLOSE,
  /// with its release tag (from the edge's carrier) and the stop tag close_epoch
  /// was given. `release_known` is false when the token was in no carrier, so
  /// the run records no message time for it (a checker counts it lost).
  struct EpochClosed
  {
    uint32_t edge = 0;
    Queued q;
    bool release_known = false;
    int64_t stop_time = 0;
    uint32_t stop_micro = 0;
  };
  std::vector<EpochClosed> epoch_closed_entries() const;

  /// Test hook (F3): drop the token of this ordinal in pop_released.
  void set_drop_token_for_test(uint64_t ordinal) {drop_token_ordinal_ = ordinal;}
  /// Test hook (F3, RCLLF_TEST_CORR_DROP_SAMPLE): the sample of this ordinal on
  /// this (member) edge is lost in transport; 0 disables (on_group_sample).
  void set_drop_sample_for_test(uint32_t edge, uint64_t ordinal)
  {
    std::lock_guard<std::mutex> lk(mu_);
    drop_sample_edge_ = edge;
    drop_sample_ordinal_ = ordinal;
  }

private:
  static constexpr uint32_t kNoLead = std::numeric_limits<uint32_t>::max();
  struct ExpKey
  {
    uint32_t edge;
    Gid gid;
    uint64_t ordinal;
    bool operator<(const ExpKey & o) const
    {
      if (edge != o.edge) {return edge < o.edge;}
      if (gid != o.gid) {return gid < o.gid;}
      return ordinal < o.ordinal;
    }
  };
  struct Exp
  {
    Outcome outcome = Outcome::PENDING;
    bool token = false;        ///< publication ACCEPTED, token created
    bool popped = false;       ///< token left the carrier for a dispatch
    bool dispatched = false;   ///< a dispatch consumed the token
    bool has_sample = false;
    uint64_t retained_id = 0;
    int64_t cutoff = kNoCutoff;  ///< publish tag + d_e + T_e
    int64_t pub_tag = -1;        ///< publish tag, -1 unknown
    int64_t stamp = 0;           ///< physical intake time of the retained sample
    bool present = false;        ///< retained past the cutoff on a local self edge
    bool tardy = false;          ///< the record arrived after its intended tag
    bool stop_closed = false;    ///< its drain was refused past the stop tag
    /// Creation tag of the reader that took the retained sample, judged
    /// again once the publish tag is known (an early sample has none yet).
    std::optional<int64_t> reader_created;
    Sample sample;
  };
  enum class PubState : uint8_t {PENDING, ACCEPTED, FAILED};
  struct Pub
  {
    PubState state = PubState::PENDING;
    Origin origin = Origin::NO_CONTEXT;
    bool tracked = true;
    std::vector<uint32_t> edges;
    std::vector<int64_t> cutoffs;  ///< parallel to `edges`
    int64_t tag = -1;              ///< publishing tag, the egress lag reference
    int64_t act = -1;              ///< S(a), the origin of the publishing activation
  };
  struct Writer
  {
    uint64_t assigned = 0;  ///< highest predicted ordinal
    uint64_t floor = 0;     ///< ordinals <= floor were pruned (late)
    /// Ordinals <= swept left the kWriterHistory window
    /// and were examined; `held` of them are kept (open when last examined).
    uint64_t swept = 0;
    uint64_t held = 0;
    uint64_t since_sweep = 0;  ///< commits since the last sweep of the held ones
    bool poisoned = false;
    std::map<uint64_t, Pub> pubs;
    std::set<uint64_t> failed;
  };
  struct Edge
  {
    uint32_t instance = 0;
    uint32_t generation = 0;
    int64_t delay_ns = 0;
    int64_t timeout_ns = -1;  ///< T_e, -1 legacy bounded join
    int64_t max_lag_ns = -1;  ///< L_e, -1 means 2 * max T_e
    uint32_t burst = 0;       ///< B_e, 0 not checked
    size_t retain_cap = kRetainPerEdge;  ///< retain bound, N_e(d_e + L) + 1
    size_t carrier_cap = kCarrierCap;    ///< carrier bound, N_e(d_e)
    std::deque<Token> in_flight;
    size_t emitted = 0;  ///< in_flight records already reported by an emit
    std::deque<Queued> carrier;
    /// tokens already missing (early OVERFLOW) that the carrier cap
    /// rejected. Drained with the carrier so the missing hook still runs once.
    std::deque<Queued> terminal;
    bool wake_pending = false;
    size_t retained = 0;
    std::deque<ExpKey> tombstones;
    EdgeCounts counts;
    std::vector<std::pair<std::string, uint64_t>> missing;  ///< (gid hex, ordinal), capped
    /// (gid hex, ordinal) of the joins pending at stop, capped like `missing`.
    std::vector<std::pair<std::string, uint64_t>> pending;
    /// Per writer, the highest ordinal with a sample seen and the highest
    /// that ended missing or pending at stop. The drain at stop waits while
    /// wanted > sample: that writer's newest missing sample may still be in flight.
    struct Seen
    {
      uint64_t sample = 0;
      uint64_t wanted = 0;
    };
    std::map<Gid, Seen, GidLess> seen;
    /// The missing WARN rate limit, clock_ ns of the last line and the
    /// missing samples since it (this one included when a line prints).
    int64_t warn_last_ns = std::numeric_limits<int64_t>::min();
    uint64_t warn_since = 0;
    uint64_t burst_worst = 0;     ///< largest fresh count above `burst`
    int64_t lag_worst_ns = 0;     ///< largest lag above L_e
    int64_t lag_worst_tag = 0;
    std::string lag_worst_point;
    uint64_t lag_exceeded = 0;
    uint64_t undrained = 0;       ///< carrier tokens past release + L_e at close
    uint64_t undelivered = 0;     ///< missing tokens that never reached a dispatch
    // Cross-federate state.
    Role role = Role::LOCAL;
    std::vector<XRecord> egress_queue;  ///< committed, not yet emitted
    std::vector<XRecord> sent_now;      ///< records of the last emitted (time, microstep)
    int64_t sent_time = std::numeric_limits<int64_t>::min();
    uint32_t sent_micro = 0;
    uint64_t egress_records = 0;
    uint64_t egress_sent = 0;
    uint64_t egress_at_stop = 0;  ///< records LF drops at the stop tag, see close_epoch
    /// (time, records) of the flushes sent within d of the last one. A
    /// record of time t lands on t + d, and LF drops it if that is past the stop tag.
    /// One entry per message; several entries of one time are the
    /// messages sent at different microsteps of it.
    std::deque<std::pair<int64_t, uint64_t>> sent_window;
    /// d = 0, (microstep, records) of the messages sent at sent_time. A
    /// record sent at (t, m) lands on (t, m + 1), past a stop tag (t, s) with s <= m.
    std::vector<std::pair<uint32_t, uint64_t>> zero_sent;
    uint64_t flushes_at_stop = 0;  ///< refused flushes of a time at or after the stop
    std::vector<int64_t> refused_times;  ///< time of every refused flush
    int64_t flush_time = std::numeric_limits<int64_t>::min();  ///< last flush_due time
    uint64_t flush_counted = 0;  ///< egress_records at the last flush_due call
    uint64_t flushes_refused = 0;
    uint64_t pub_lag_exceeded = 0;
    int64_t pub_lag_worst_ns = 0;
    int64_t pub_lag_worst_tag = 0;
    struct Remote
    {
      uint64_t high = 0;  ///< highest recorded ordinal of this writer
      /// Every recorded ordinal of the run as ascending closed runs
      /// [first, last], exact for the whole run (it replaced the newest 1024
      /// ordinals, which took every older ordinal as recorded). Records come
      /// in ordinal order (D1), so a record extends the last run or opens one
      /// after a gap: one run (16 bytes) per gap, and a gap is a publication
      /// that made no record (a refused origin, counted by the sender).
      std::vector<std::pair<uint64_t, uint64_t>> runs;
      void add(uint64_t ordinal)
      {
        if (!runs.empty() && runs.back().second + 1 == ordinal) {
          runs.back().second = ordinal;
        } else {
          runs.emplace_back(ordinal, ordinal);
        }
      }
      bool has(uint64_t ordinal) const
      {
        auto it = std::upper_bound(
          runs.begin(), runs.end(), ordinal,
          [](uint64_t o, const std::pair<uint64_t, uint64_t> & r) {return o < r.first;});
        return it != runs.begin() && ordinal <= std::prev(it)->second;
      }
    };
    std::map<Gid, Remote, GidLess> remotes;
    uint64_t duplicate_records = 0;
    /// Egress, the last completion number and its tag.
    uint64_t complete_sent = 0;
    int64_t complete_time = std::numeric_limits<int64_t>::min();
    uint32_t complete_micro = 0;
    /// Late COMPLETE: (time, microstep) of the COMPLETE messages that may still
    /// land past a stop tag, in send order. d = 0 lands (t, m + 1), d > 0 the
    /// k-th of time t lands (t + d, k). close_epoch counts those past the stop.
    std::deque<std::pair<int64_t, uint32_t>> complete_msgs;
    uint64_t completes_at_stop = 0;
    /// Ingress, the last completion number received, the
    /// COMPLETE messages and the numbers that never came.
    uint64_t complete_high = 0;
    uint64_t completes = 0;
    uint64_t duplicate_completes = 0;
    /// The first kMaxListedControlFailures missing sequence numbers.
    std::vector<uint64_t> control_failures;
    uint64_t control_failure_count = 0;
    /// Ingress, the sender's end-of-run count (note_complete_end).
    bool complete_end_expected = false;
    bool complete_end_seen = false;
    uint64_t complete_end_due = 0;
    uint64_t complete_end_sent = 0;
    uint64_t clock_skew = 0;
    int64_t clock_bound_ns = 0;  ///< eps, the declared offset bound of an ingress edge
    bool self_local = false;     ///< a local self edge, see set_self_local
    bool local_now = false;      ///< decided at publish, see set_local_now
    bool sub_rebind = false;         ///< see set_sub_rebind
    uint32_t reader_generation = 0;  ///< last reader generation admitted
    int64_t reader_created = std::numeric_limits<int64_t>::min();  ///< its creation tag
    std::vector<std::string> reader_refused;  ///< refused readers, "code: detail"
    std::vector<int64_t> reader_creations;    ///< creation tags, ascending
    std::set<ExpKey> stale_taken;  ///< records with a STALE_READER sample, open
    bool deferred = false;       ///< a deferred join, see set_deferred
    bool same_tag = false;       ///< release at the publication tag, see set_same_tag
    bool zero_link = false;      ///< zero-delay federated link, see set_zero_link
    uint32_t lead = kNoLead;     ///< lead of its fan-in group, kNoLead outside one
    uint32_t rank = 0;           ///< fan-in rank, orders equal releases of a group
  };
  /// A sample of a writer no member of an ingress group knows yet.
  struct Pooled
  {
    Sample sample;
    int64_t stamp = 0;
    std::optional<int64_t> reader_created;
  };
  /// A fan-in group, keyed by its lead in groups_.
  struct Group
  {
    std::vector<uint32_t> members;           ///< ascending edge ids
    std::map<Gid, uint32_t, GidLess> owner;  ///< writer GID -> its member edge
    std::map<Gid, std::map<uint64_t, Pooled>, GidLess> pool;  ///< (gid, ordinal) -> sample
    size_t pooled = 0;
    bool popped = false;  ///< `last` holds a popped key
    /// largest popped key (release time, release microstep, rank, ordinal)
    std::tuple<int64_t, uint32_t, uint32_t, uint64_t> last{};
    uint64_t order_violations = 0;
  };

  /// A sample stamped `stamp` missed `cutoff` on `e`. A deferred edge's
  /// deadline is strict, a blocking edge's join still matches at the cutoff.
  static bool late_for(const Edge & e, int64_t stamp, int64_t cutoff)
  {
    return e.deferred ? stamp >= cutoff : stamp > cutoff;
  }

  void terminate_locked(const ExpKey & key, Exp & exp, Outcome outcome);
  /// Set local_now unless the edge's role or flags refuse it.
  void set_local_now_locked(Edge & e);
  /// End a local_now expectation LOST, reason local_undelivered.
  void lose_local_locked(const ExpKey & key, Exp & exp, bool planted);
  void check_burst_locked(uint32_t edge);
  void note_missing_locked(const ExpKey & key, Outcome outcome);
  static int64_t cutoff_of(const Pub & p, uint32_t edge);
  void loss_gap_locked(uint32_t edge, const Gid & gid, uint64_t ordinal);
  void prune_locked(const Gid & gid, Writer & w);
  /// An expectation of publication (`gid`, `ordinal`) is still open.
  bool pub_open_locked(const Gid & gid, uint64_t ordinal, const Pub & p) const;
  Token token_for(const ExpKey & key) const;
  int64_t lag_limit_locked(const Edge & e) const;
  /// The next completion number of `e` for a message at (`time`, `microstep`).
  static uint64_t complete_seq_locked(Edge & e, int64_t time, uint32_t microstep);
  bool arrive_locked(uint32_t edge, uint32_t count, int64_t release_ns, uint32_t pub_microstep);
  static uint32_t release_micro_locked(const Edge & e, uint32_t pub_microstep);
  void release_early_locked(const ExpKey & key, Exp & exp, SampleOutcome outcome);
  /// Count one sample EPOCH_CLOSE identity on `edge` (no count change).
  void note_epoch_close_locked(uint32_t edge, const Gid & gid, uint64_t ordinal);
  bool fast_locked() const;  ///< the installed fast predicate
  SampleOutcome on_ingress_sample_locked(
    uint32_t edge, const Gid & gid, uint64_t ordinal, Sample & sample, int64_t stamp,
    uint64_t * retained_id, std::optional<int64_t> reader_created);
  SampleOutcome on_sample_locked(
    uint32_t edge, const Gid & gid, uint64_t ordinal, Sample & sample, int64_t stamp,
    uint64_t * retained_id, std::optional<int64_t> reader_created);
  /// The group of `edge` (two or more members), else nullptr.
  Group * group_locked(uint32_t edge);
  const Group * group_locked(uint32_t edge) const;
  /// The edge that holds the drain wake of `edge`: its group's lead, or itself.
  uint32_t wake_edge_locked(uint32_t edge) const;
  /// Member `edge` publishes or records for writer `gid`; its pooled
  /// samples move to `edge` with their take stamps.
  void claim_locked(uint32_t edge, const Gid & gid);
  bool drop_sample_locked(uint32_t edge, uint64_t ordinal) const;
  void release_pooled_locked(
    uint32_t lead, Group & g, const Gid & gid, uint64_t ordinal, SampleOutcome outcome);
  /// A reader created at `reader_created` was replaced before `pub_tag`.
  bool stale_locked(const Edge & e, std::optional<int64_t> reader_created, int64_t pub_tag) const;
  /// A retained sample whose reader `stale_locked` names goes STALE_READER.
  bool drop_stale_locked(const ExpKey & key, Exp & exp);
  /// Wait under `lk` until `ready`, `until` (table clock) or the stop
  /// deadline, whichever comes first. True when the stop deadline ended the wait.
  bool wait_until_locked(
    std::unique_lock<std::mutex> & lk, int64_t until, const std::function<bool()> & ready);
  /// Count and list a join the stop deadline ended.
  void pending_at_stop_locked(const ExpKey & key);

  const uint64_t epoch_;
  Observer * observer_ = nullptr;
  uint64_t next_retained_id_ = 0;
  std::function<int64_t()> clock_;
  std::function<bool()> fast_;
  mutable std::mutex mu_;
  /// Key of this table in the per-thread burst tally (never reused).
  static uint64_t new_burst_uid();
  const uint64_t burst_uid_ = new_burst_uid();
  std::condition_variable cv_;
  std::vector<Edge> edges_;
  std::map<Gid, Writer, GidLess> writers_;
  std::set<Gid, GidLess> finalized_;
  std::map<ExpKey, Exp> exps_;
  size_t retained_total_ = 0;
  size_t retain_total_cap_ = kRetainTotal;  ///< max(kRetainTotal, sum of retain caps)
  bool closed_ = false;
  int64_t close_stop_time_ = std::numeric_limits<int64_t>::max();  ///< close_epoch's stop time
  std::vector<EpochClosed> epoch_closed_;  ///< filled by close_epoch
  int64_t stop_deadline_ = kNoCutoff;  ///< table clock, set by note_stop_requested
  std::set<ExpKey> joining_;           ///< the joins waiting now
  uint64_t identity_faults_ = 0;
  uint64_t stalls_ = 0;  ///< stall-watch episodes
  uint64_t join_waits_ = 0;  ///< calls of wait_until_locked
  uint64_t publications_ = 0;
  uint64_t accepted_ = 0;
  uint64_t failed_ = 0;
  uint64_t admitted_origin_ = 0;
  uint64_t refused_origin_ = 0;
  uint64_t untracked_ = 0;  ///< accepted publications of publishers on no admitted edge
  std::map<Origin, uint64_t> refused_by_reason_;
  uint64_t drop_token_ordinal_ = 0;
  uint32_t drop_sample_edge_ = 0;
  uint64_t drop_sample_ordinal_ = 0;
  uint64_t released_retained_ = 0;
  uint64_t matched_samples_ = 0;
  uint64_t duplicate_tokens_ = 0;
  uint64_t rejected_tokens_ = 0;
  std::map<uint32_t, Group> groups_;  ///< fan-in groups by lead
};

}  // namespace corr
}  // namespace ros2_lf_binding

#endif  // CORRESPONDENCE_HPP_

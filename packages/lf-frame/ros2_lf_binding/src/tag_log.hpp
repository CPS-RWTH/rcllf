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

/// @file tag_log.hpp
/// @brief Per-topic record of the writes made inside LF reactions.
///
/// The publish gate records every publish made inside an LF reaction with its
/// writer GID and sequence number (the gate's ordinal, which is the DDS
/// publication sequence number, decision publication-identity-initial-path),
/// and every `ros2_lf_claim_publish` as an unfilled record. These are exactly
/// the writes a stub trigger counts, so records numbered in publish order
/// line up with the triggers: a trigger carrying k writes owns the next k
/// records of its topic. A publish outside any reaction fills the newest
/// unfilled claim of its topic, or stays unrecorded (an untagged sample).
/// The take path (take_mode.cpp, take_by_tag) looks a taken sample up here to
/// learn which trigger it belongs to.
///
/// Across federates the log is replayed: every call that
/// changes a topic's log is also kept as an event in the topic's journal, the
/// publishing federate packs the events since its last trigger into the
/// trigger it sends (`pack`), and the receiving federate applies them to its
/// own log of the topic (`import`) before its take runs. The publishers of
/// such a topic sit in one federate (the generator refuses more), so the
/// receiver's log equals the writer's, index for index. Each publisher node
/// leaves on its own link, and every link carries the events since its own
/// last message, so the receiver has them all whichever link it imports first.

#ifndef TAG_LOG_HPP_
#define TAG_LOG_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ros2_lf_binding
{
namespace tag_log
{

using Gid = std::array<uint8_t, 16>;

/// Records kept per topic; an older record is forgotten, but its sample is
/// still known to be old (`old`). Provenance: CONVENTION, a memory bound. A
/// trigger whose records were forgotten before its take is served by the
/// count rule, which take_stats counts (`count_rule=`) and the shutdown
/// summary warns about, so the bound never changes a rule
/// silently.
constexpr uint64_t kKeep = 4096;

/// One publish through the gate. `in_reaction`: made inside an LF reaction, so
/// counted in its trigger's write count. Without a GID (`gid_ok` false) the
/// record has no identity and its trigger falls back to the count rule.
void on_publish(
  const std::string & topic, bool in_reaction, bool gid_ok, const Gid & gid, uint64_t seq);

/// `ros2_lf_claim_publish` inside an LF reaction: one write whose sample comes later.
void on_claim(const std::string & topic);

/// Number of records ever made on `topic`: the index the next one gets.
uint64_t end(const std::string & topic);

/// Whether records [begin, end) of `topic` exist, are still kept and none is
/// without identity, so a trigger owning them can be served by tag.
bool exact(const std::string & topic, uint64_t begin, uint64_t end);

/// Index of the record of sample (gid, seq) on `topic`, if it has one.
std::optional<uint64_t> find(const std::string & topic, const Gid & gid, uint64_t seq);

/// Whether sample (gid, seq) of `topic` belongs to a forgotten record: its
/// writer's sequence number is at or below the highest one of a forgotten
/// record or a forgotten claim it filled. Such a
/// sample is of an earlier trigger. An unclaimed publish of that writer made
/// before the forgotten one and still untaken counts as old as well.
bool old(const std::string & topic, const Gid & gid, uint64_t seq);

/// The highest sequence number of writer `gid` the log of `topic` has seen,
/// recorded or not. A sample above it was published after the last event this
/// process knows of: across federates its events may still be on the way.
/// Only a topic that import was called for has it; in one process every
/// publish is recorded before its sample is sent.
std::optional<uint64_t> seen(const std::string & topic, const Gid & gid);

/// Whether import or expect was called for `topic`: its writers publish in
/// another process, so a sample of a writer not `seen` yet may be ahead of its events.
bool remote(const std::string & topic);

/// Writer side: `gid` is a publisher of `topic` in this process. Every
/// publisher goes through the publish gate, so its writes are journaled; pack
/// announces the set with each message.
void on_writer(const std::string & topic, const Gid & gid);

/// Reader side: one writer federate publishes `topic` to this one over take
/// links, with `publishers` gated publishers of it (the generator counts them
/// from the analysis, third review). The topic is remote from now
/// on, and a writer not announced is unknown until an import arrived and the
/// announced set holds `publishers` GIDs. One writer federate per topic: the
/// generator refuses publishers in two (`xtag_publisher_errors`), and a
/// second writer session breaks the topic at its first import (kSession).
void expect(const std::string & topic, size_t publishers);

/// Whether an import was applied to `topic` at all.
bool imported(const std::string & topic);

/// What the reader knows of the writer of a sample on a remote topic.
enum class Writer
{
  kGated,    ///< announced, or seen in an event: its events tell the sample's trigger
  kPlain,    ///< not announced, and every expected publisher was: outside rcllf, untagged
  kUnknown,  ///< not announced, and no import yet or an expected publisher not announced yet
};

/// Classify writer `gid` of `topic`. A topic not remote has no announcement
/// to wait for: kPlain.
Writer writer(const std::string & topic, const Gid & gid);

/// The take delivered a sample of `gid` as kPlain: a later announcement of it
/// (a publisher the generator did not count, created after the writer's
/// earlier messages) is a misclassification that `late_writer` reports.
void note_plain(const std::string & topic, const Gid & gid);

/// Whether an import announced a writer whose samples were delivered as
/// plain before; reported once per topic.
bool late_writer(const std::string & topic);

/// One event of a topic's journal: a call of on_publish or on_claim.
struct Event
{
  enum Kind : uint8_t { kPublish = 1, kClaim = 2 };
  Kind kind = kPublish;
  bool in_reaction = false;
  bool gid_ok = false;
  Gid gid{};
  uint64_t seq = 0;
};

/// Events kept per topic for `pack`; an older one is forgotten. Provenance:
/// CONVENTION, a memory bound. Forgotten events that a link still had to
/// send reach the reader as a gap, and the reader ends the run
/// (ros2_lf_xtag_import), so the bound never changes a rule silently.
constexpr uint64_t kJournalKeep = 4096;

/// What `pack` hands to the link.
struct Packed
{
  uint64_t session = 0;   ///< `session()` of the packing process
  uint64_t first = 0;     ///< journal ordinal of events[0]
  std::vector<Event> events;
  /// Events since the link's last pack were left out (more than `max`, or forgotten).
  bool overflow = false;
  std::vector<Gid> writers;  ///< the process's publishers of the topic (`on_writer`)
};

/// The events of `topic` since the last pack on `link`, the newest `max` of
/// them, and moves `link` past all of them. Consecutive publishes outside any
/// reaction by one writer are merged into the last one while no link sent it:
/// they change nothing but `seen`.
Packed pack(const std::string & topic, const std::string & link, size_t max);

/// This process's journal session: random, new at start and at `reset`. The
/// journal ordinals of a restarted writer start again under a new session.
uint64_t session();

/// What `import` found.
enum class Import { kOk, kGap, kSession, kWriters, kBroken };

/// Apply events packed by another process to the log of `topic`, skipping the
/// ones already applied (another link of the topic brought them). The topic's
/// log is broken from then on when events are missing (`kGap`: `first` past
/// the next expected one) or the writer's session changed (`kSession`: a
/// restarted writer numbers its journal from 0 again, review), or
/// the writer federate announced only part of its publishers (`kWriters`:
/// `writers_more`, so a writer outside the set may still be gated).
/// `writers`: the publishers the writer federate announces. `kBroken`: broken
/// by an earlier import.
Import import(
  const std::string & topic, uint64_t session, uint64_t first, const Event * events, size_t n,
  const Gid * writers = nullptr, size_t nw = 0, bool writers_more = false);

/// Whether an import of `topic` found events missing: its log no longer
/// equals the writer's, so no trigger on it can be served by tag.
bool broken(const std::string & topic);

/// Forget every record (tests only).
void reset();

}  // namespace tag_log
}  // namespace ros2_lf_binding

#endif  // TAG_LOG_HPP_

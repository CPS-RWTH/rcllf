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

/// @file match_warmup.hpp
/// @brief Wait for DDS to match the cross-federate endpoints before LF starts.
///
/// A best-effort sample published before DDS matched the reader to the writer
/// is lost, and matching is not symmetric in time: a reader can see the writer
/// before the writer sees the reader. A federate's first publication can come
/// right after the RTI handshake (a fast federate does not wait for the start
/// time), so each federate waits until every local writer of a topic that
/// crosses federates has matched as many readers as the whole system has, and
/// every local reader as many writers. The counts come from the federate's
/// system.json key `cross_federate_topics` (written by colcon lf-generate).
/// A checked barrier: an endpoint still unmatched when the bound
/// passes refuses the start, since a sample published before its match would
/// be lost with no count. The bound is the discovery budget the caller passes,
/// unless ROS2_LF_MATCH_WARMUP_MS overrides it.

#ifndef MATCH_WARMUP_HPP_
#define MATCH_WARMUP_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "pair_match.hpp"

namespace ros2_lf_binding
{
namespace match_warmup
{

/// One entry of `cross_federate_topics`: system-wide endpoint node counts.
struct CrossTopic
{
  std::string topic;
  size_t publishers = 0;
  size_t subscribers = 0;
};

/// The `cross_federate_topics` of a system.json document; empty when the key
/// is absent (the one program, or a federate generated without it). An entry
/// without a string `topic` is skipped.
std::vector<CrossTopic> parse_cross_topics(const nlohmann::json & doc);

/// parse_cross_topics() of the file at `path`; empty if it cannot be read.
std::vector<CrossTopic> load_cross_topics(const std::string & path);

/// ROS2_LF_MATCH_WARMUP_MS, else `default_ms` (the runtime passes its discovery
/// budget). 0 checks once and does not wait; an unparsable,
/// negative or over one day (86400000) value keeps the default.
int64_t timeout_ms_from_env(int64_t default_ms);

/// One local endpoint to wait for: `have()` is its current matched count.
struct Target
{
  std::string topic;
  bool writer = false;  ///< a publisher counting readers, else a reader counting writers
  size_t want = 0;
  std::function<size_t()> have;
  /// One writer-reader pair, `have()` 1 when the two are proven
  /// matched to each other (pair_match.hpp), else 0. `writer` is false.
  bool pair = false;
  /// The reason of the pair's last verdict, written by `have()`.
  std::shared_ptr<std::string> reason{};
};

struct Outcome
{
  std::string topic;
  bool writer = false;
  size_t want = 0;
  size_t have = 0;  ///< the count at the last read
  bool matched = false;  ///< `have >= want` at the last read
  int64_t match_us = 0;  ///< time from the start of the wait to the last match
  size_t lost = 0;  ///< times the count fell below `want` again after a match
  bool pair = false;  ///< a writer-reader pair target (Target::pair)
  std::string reason;  ///< a pair's reason at the last read (Target::reason)
};

/// Reads every target each millisecond until a read finds all of them at
/// `want` at once, or `timeout` passed. A target that matched and lost the
/// match again counts as unmatched. One outcome per target, in
/// order, from the last read.
std::vector<Outcome> wait(const std::vector<Target> & targets, std::chrono::milliseconds timeout);

/// One INFO stats line per outcome and a summary line, each prefixed
/// `ros2_lf_match_warmup:`, and a FATAL "startup refused" line per target that
/// timed out. True when every target matched; the caller refuses otherwise.
bool log_outcomes(const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms);

/// One reader of this process on `topic`; `writers()` is its matched writer count.
struct LocalReader
{
  std::string topic;
  std::function<size_t()> writers;
};

/// The startup targets of one process. Each reader
/// waits for at least one writer, and each local writer of a topic for every
/// local reader of it (`writer_counts(topic)`: the matched reader count of
/// every local writer on the topic, as corr_rt::publisher_match_counts). A
/// reader can see a writer before the writer sees the reader, so the reader
/// side alone left a node that gates on its publisher's subscription count
/// (brake_fusion simulator) to start a tick late.
std::vector<Target> local_targets(
  const std::vector<LocalReader> & readers,
  const std::function<std::vector<size_t>(const std::string &)> & writer_counts);

/// The binding's exit code for a start it refuses (as the cross-federate peer
/// check and the BNA runtime): a defect of the run, not of the program.
/// One local_now edge for the startup match. A publish
/// before its reader matched the writer leaves no sample in the reader, which
/// the publish-time take reports as local_undelivered (exit 3).
struct LocalEdge
{
  std::string topic;
  const void * reader = nullptr;  ///< identity of the edge's reader (a group's one)
  const void * writer = nullptr;  ///< identity of the edge's writer
  std::function<size_t()> reader_writers;  ///< writers the reader has matched
  std::function<size_t()> writer_readers;  ///< readers the writer has matched
  /// Whether this edge's writer and reader
  /// are proven matched to each other (pair_match::check). Unset: no pair
  /// target.
  std::function<pair_match::Verdict()> paired;
};

/// Each local reader has matched at least as many writers as
/// local_now edges name for it, and each local writer at least as many
/// readers. Pure, a reader or writer named by several edges is one target.
/// A count says how many, not which. A writer or
/// reader no edge names (another process, a QoS-incompatible endpoint's
/// compatible neighbour) makes up a count while the edge's own pair is
/// unmatched. So each distinct (reader, writer) pair with `paired` set is a
/// target of its own, want 1, met while `paired` holds. Fail closed: a pair
/// whose `paired` is nullopt (cannot be told) is unmatched, and its reason goes into the refusal.
std::vector<Target> local_edge_targets(const std::vector<LocalEdge> & edges);

constexpr int kUnreadyExitCode = 3;

/// The refusal text of the first unmatched outcome, empty when all matched.
std::string unready_message(
  const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms);

/// log_outcomes(), then, if a target did not match, a FATAL with
/// unready_message() and _Exit(kUnreadyExitCode) (it was once a WARN and
/// "starting anyway"). Returns only when every target matched.
void enforce_ready(const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms);

}  // namespace match_warmup
}  // namespace ros2_lf_binding

#endif  // MATCH_WARMUP_HPP_

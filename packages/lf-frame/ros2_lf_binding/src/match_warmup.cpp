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

/// @file match_warmup.cpp
/// @brief Cross-federate DDS match warm-up, see match_warmup.hpp.

#include "match_warmup.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{
namespace match_warmup
{

namespace
{
// Upper limit of ROS2_LF_MATCH_WARMUP_MS: steady_clock::now() + timeout must
// not overflow. One day is far below that range and far above any discovery
// time, so a larger value can only be a typo, which keeps the default.
constexpr int64_t kMaxTimeoutMs = 24LL * 3600 * 1000;
// The poll interval cannot change the verdict. Every pass reads
// every count, and the barrier lifts only on a pass where all of them have
// reached their target at once, so a match that is lost again (a peer that
// went away) is seen on the next pass whatever the interval. The interval
// sets two things only: how late after the last match the barrier lifts (at
// most one interval, before the RTI handshake, so no tag moves) and how often
// the counts are read. 1 ms is below every match time logged
// (ros2_lf_match_warmup ... match_us) and costs one count read per
// target per millisecond, which does not compete with DDS discovery. A peer
// that matches and leaves again within one interval is not seen, which is
// the limit of any count based check: the check reads counts, not events.
constexpr auto kPollInterval = std::chrono::milliseconds(1);

rclcpp::Logger logger() {return rclcpp::get_logger("ros2_lf_binding");}
}  // namespace

std::vector<CrossTopic> parse_cross_topics(const nlohmann::json & doc)
{
  std::vector<CrossTopic> out;
  if (!doc.is_object() || !doc.contains("cross_federate_topics") ||
    !doc.at("cross_federate_topics").is_array())
  {
    return out;
  }
  for (const auto & e : doc.at("cross_federate_topics")) {
    if (!e.is_object() || !e.contains("topic") || !e.at("topic").is_string()) {
      continue;
    }
    CrossTopic t;
    t.topic = e.at("topic").get<std::string>();
    if (e.contains("publishers") && e.at("publishers").is_number_unsigned()) {
      t.publishers = e.at("publishers").get<size_t>();
    }
    if (e.contains("subscribers") && e.at("subscribers").is_number_unsigned()) {
      t.subscribers = e.at("subscribers").get<size_t>();
    }
    out.push_back(std::move(t));
  }
  return out;
}

std::vector<CrossTopic> load_cross_topics(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
  if (doc.is_discarded()) {
    return {};
  }
  return parse_cross_topics(doc);
}

int64_t timeout_ms_from_env(int64_t default_ms)
{
  const char * v = std::getenv("ROS2_LF_MATCH_WARMUP_MS");
  if (v == nullptr || *v == '\0') {
    return default_ms;
  }
  char * end = nullptr;
  const long long ms = std::strtoll(v, &end, 10);  // NOLINT(runtime/int)
  if (end == v || *end != '\0' || ms < 0 || ms > kMaxTimeoutMs) {
    RCLCPP_WARN(
      logger(), "ROS2_LF_MATCH_WARMUP_MS='%s' is not a duration in ms; using %" PRId64,
      v, default_ms);
    return default_ms;
  }
  return static_cast<int64_t>(ms);
}

std::vector<Outcome> wait(const std::vector<Target> & targets, std::chrono::milliseconds timeout)
{
  std::vector<Outcome> out;
  out.reserve(targets.size());
  for (const auto & t : targets) {
    out.push_back({t.topic, t.writer, t.want, 0, false, 0, 0, t.pair, ""});
  }
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    bool all = true;
    // Every target is read on every pass,
    // also one that matched before. A writer whose reader went away after it
    // matched is unmatched again, and the barrier must not lift on the old
    // count: a sample published then is lost with no count.
    for (size_t i = 0; i < targets.size(); ++i) {
      out[i].have = targets[i].have();
      if (targets[i].reason) {
        out[i].reason = *targets[i].reason;
      }
      const bool now_matched = out[i].have >= out[i].want;
      if (now_matched && !out[i].matched) {
        out[i].match_us =
          std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
      } else if (!now_matched && out[i].matched) {
        ++out[i].lost;
      }
      out[i].matched = now_matched;
      all = all && now_matched;
    }
    if (all || now >= deadline) {
      return out;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
}

bool log_outcomes(const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms)
{
  size_t matched = 0;
  int64_t longest_us = 0;
  for (const auto & o : outcomes) {
    RCLCPP_INFO(
      logger(),
      "ros2_lf_match_warmup: stage=%s topic=%s side=%s have=%zu want=%zu matched=%d "
      "match_us=%" PRId64 " lost=%zu", stage, o.topic.c_str(),
      o.pair ? "pair" : o.writer ? "writer" : "reader",
      o.have, o.want, o.matched ? 1 : 0, o.match_us, o.lost);
    if (o.matched) {
      ++matched;
      longest_us = std::max(longest_us, o.match_us);
      continue;
    }
    // A sample published before the match would be lost with no
    // count, so an unmatched endpoint refuses the start instead.
    if (o.pair) {
      RCLCPP_FATAL(
        logger(),
        "match warm-up (%s): startup refused: topic '%s': a local edge's publisher and reader "
        "are not matched to each other within %" PRId64 " ms, although their matched counts "
        "may be complete: %s", stage, o.topic.c_str(), timeout_ms, o.reason.c_str());
      continue;
    }
    RCLCPP_FATAL(
      logger(),
      "match warm-up (%s): startup refused: topic '%s': the local %s matched %zu of %zu %s "
      "within %" PRId64 " ms (%zu match(es) lost again), a sample published before the "
      "match would be lost uncounted (ROS2_LF_MATCH_WARMUP_MS)", stage,
      o.topic.c_str(),
      o.writer ? "publisher" : "reader", o.have, o.want, o.writer ? "readers" : "publishers",
      timeout_ms, o.lost);
  }
  RCLCPP_INFO(
    logger(),
    "ros2_lf_match_warmup: stage=%s targets=%zu matched=%zu timed_out=%zu longest_match_us=%"
    PRId64 " timeout_ms=%" PRId64, stage, outcomes.size(), matched, outcomes.size() - matched,
    longest_us, timeout_ms);
  return matched == outcomes.size();
}

std::vector<Target> local_targets(
  const std::vector<LocalReader> & readers,
  const std::function<std::vector<size_t>(const std::string &)> & writer_counts)
{
  std::vector<Target> out;
  std::map<std::string, size_t> per_topic;
  for (const auto & r : readers) {
    out.push_back({r.topic, false, 1, r.writers});
    ++per_topic[r.topic];
  }
  for (const auto & [topic, n] : per_topic) {
    if (writer_counts(topic).empty()) {
      continue;  // no writer in this process: the reader target covers the topic
    }
    out.push_back(
      {topic, true, n, [topic = topic, writer_counts]() {
          const auto counts = writer_counts(topic);
          return counts.empty() ? size_t{0} : *std::min_element(counts.begin(), counts.end());
        }});
  }
  return out;
}

std::vector<Target> local_edge_targets(const std::vector<LocalEdge> & edges)
{
  std::vector<Target> out;
  // Distinct counterparts per endpoint, in first-seen order of the endpoint.
  std::vector<const void *> readers;
  std::vector<const void *> writers;
  std::map<const void *, std::set<const void *>> reader_wants;
  std::map<const void *, std::set<const void *>> writer_wants;
  std::map<const void *, const LocalEdge *> first;
  for (const auto & e : edges) {
    if (reader_wants.find(e.reader) == reader_wants.end()) {
      readers.push_back(e.reader);
      first[e.reader] = &e;
    }
    if (writer_wants.find(e.writer) == writer_wants.end()) {
      writers.push_back(e.writer);
      first[e.writer] = &e;
    }
    reader_wants[e.reader].insert(e.writer);
    writer_wants[e.writer].insert(e.reader);
  }
  for (const void * r : readers) {
    const LocalEdge & e = *first[r];
    out.push_back({e.topic, false, reader_wants[r].size(), e.reader_writers});
  }
  for (const void * w : writers) {
    const LocalEdge & e = *first[w];
    out.push_back({e.topic, true, writer_wants[w].size(), e.writer_readers});
  }
  std::set<std::pair<const void *, const void *>> pairs;
  for (const auto & e : edges) {
    if (!e.paired || !pairs.insert({e.reader, e.writer}).second) {
      continue;
    }
    // Fail closed. Only a pair proven
    // matched counts; an unknown verdict (nullopt) is unmatched with its reason.
    auto reason = std::make_shared<std::string>();
    out.push_back(
      {e.topic, false, 1, [paired = e.paired, reason]() {
          const pair_match::Verdict v = paired();
          const bool matched = v.paired.has_value() && *v.paired;
          *reason = matched ? std::string() :
          v.paired.has_value() ? v.why : "cannot be told: " + v.why;
          return matched ? size_t{1} : size_t{0};
        }, true, reason});
  }
  return out;
}

std::string unready_message(
  const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms)
{
  for (const auto & o : outcomes) {
    if (o.matched) {
      continue;
    }
    if (o.pair) {
      return std::string("refusing to start (") + stage + "): topic '" + o.topic +
             "': a local edge's publisher and reader are not matched to each other within " +
             std::to_string(timeout_ms) + " ms: " + o.reason + ". A publish now would leave no "
             "sample in the reader. Raise ROS2_LF_DISCOVERY_TIMEOUT_MS / "
             "ROS2_LF_MATCH_WARMUP_MS, or fix the endpoints' QoS";
    }
    return std::string("refusing to start (") + stage + "): topic '" + o.topic +
           "': the local " + (o.writer ? "publisher" : "reader") + " matched " +
           std::to_string(o.have) + " of " + std::to_string(o.want) + " " +
           (o.writer ? "readers" : "publishers") + " within " + std::to_string(timeout_ms) +
           " ms. A start before the match would make the first tags depend on when DDS "
           "discovery finished. Start every process of the system, or raise "
           "ROS2_LF_DISCOVERY_TIMEOUT_MS / ROS2_LF_MATCH_WARMUP_MS";
  }
  return {};
}

void enforce_ready(const std::vector<Outcome> & outcomes, const char * stage, int64_t timeout_ms)
{
  // The verdict is log_outcomes' (every target matched on the last
  // read, a match lost again counts as unmatched); the message names the first
  // unmatched target and the budget to raise.
  if (log_outcomes(outcomes, stage, timeout_ms)) {
    return;
  }
  const std::string why = unready_message(outcomes, stage, timeout_ms);
  RCLCPP_FATAL(logger(), "%s", why.c_str());
  std::fflush(nullptr);
  std::_Exit(kUnreadyExitCode);
}

}  // namespace match_warmup
}  // namespace ros2_lf_binding

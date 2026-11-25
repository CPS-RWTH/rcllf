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

/// @file tag_log.cpp
/// @brief Per-topic record of the writes made inside LF reactions.

#include "tag_log.hpp"

#include <algorithm>
#include <deque>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ros2_lf_binding
{
namespace tag_log
{
namespace
{

struct Rec
{
  bool anonymous = false;  // published without a GID
  bool known = false;      // has (gid, seq); false for an unfilled claim
  Gid gid{};
  uint64_t seq = 0;
};

struct TopicLog
{
  uint64_t base = 0;  // index of recs.front()
  std::deque<Rec> recs;
  std::map<std::pair<Gid, uint64_t>, uint64_t> by_id;
  std::vector<uint64_t> open_claims;  // indices of unfilled claims, oldest first
  // Forgotten records: per writer the highest sequence number among them, and
  // the claims forgotten unfilled, which the next publishes outside a reaction
  // fill when no kept claim is open.
  std::map<Gid, uint64_t> old_seq;
  uint64_t old_claims = 0;
  // Highest sequence number of each writer seen on the topic.
  std::map<Gid, uint64_t> seen;
  // Journal of the calls that changed this log, for pack (ordinals from
  // journal_base); per link the next ordinal its pack sends, the end of the
  // journal any link sent, and the next ordinal import expects.
  uint64_t journal_base = 0;
  std::deque<Event> journal;
  std::map<std::string, uint64_t> packed;
  uint64_t sent = 0;
  uint64_t imported = 0;
  uint64_t session = 0;  // the writer's session of the imported events
  bool broken = false;
  bool remote = false;  // imported into or expected: its writers publish in another process
  // Writer side: this process's publishers of the topic.
  std::set<Gid> writers;
  // Reader side: the publishers the writer federate announced, whether an
  // import arrived at all, how many gated publishers the generator expects,
  // and the writers whose samples went out as plain.
  // One writer federate per crossing topic (the generator refuses more): a
  // second one has another session and breaks the topic at its first import.
  std::set<Gid> announced;
  bool imported_once = false;
  size_t expected = 0;
  std::set<Gid> plain;
  bool late_writer = false;
  bool last_untagged = false;  // the journal's last event is an untagged publish
};

// Locked inside the gate's slot mutex; never the other way round.
std::mutex g_mutex;
std::unordered_map<std::string, TopicLog> g_logs;
uint64_t g_session = 0;

uint64_t new_session()
{
  std::random_device rd;
  uint64_t s = 0;
  while (s == 0) {
    s = (static_cast<uint64_t>(rd()) << 32) ^ rd();
  }
  return s;
}

void raise_old(TopicLog & log, const Gid & gid, uint64_t seq)
{
  uint64_t & hw = log.old_seq[gid];
  hw = std::max(hw, seq);
}

void append(TopicLog & log, const Rec & rec)
{
  const uint64_t idx = log.base + log.recs.size();
  log.recs.push_back(rec);
  if (rec.known) {
    log.by_id[{rec.gid, rec.seq}] = idx;
  }
  while (log.recs.size() > kKeep) {
    const Rec & front = log.recs.front();
    if (front.known) {
      log.by_id.erase({front.gid, front.seq});
      raise_old(log, front.gid, front.seq);
    }
    log.recs.pop_front();
    ++log.base;
  }
  while (!log.open_claims.empty() && log.open_claims.front() < log.base) {
    log.open_claims.erase(log.open_claims.begin());
    ++log.old_claims;
  }
}

void note(TopicLog & log, const Event & ev)
{
  // An untagged publish (outside any reaction, no claim to fill) changes
  // nothing but `seen`, so one after another of one writer merge into one.
  const bool untagged = ev.kind == Event::kPublish && !ev.in_reaction && ev.gid_ok &&
    log.open_claims.empty() && log.old_claims == 0;
  const bool unsent = log.journal_base + log.journal.size() > log.sent;
  if (untagged && log.last_untagged && unsent && log.journal.back().gid == ev.gid) {
    log.journal.back().seq = ev.seq;
    return;
  }
  log.last_untagged = untagged;
  log.journal.push_back(ev);
  while (log.journal.size() > kJournalKeep) {
    log.journal.pop_front();
    ++log.journal_base;
  }
}

void apply_publish(
  TopicLog & log, bool in_reaction, bool gid_ok, const Gid & gid, uint64_t seq)
{
  if (gid_ok) {
    uint64_t & s = log.seen[gid];
    s = std::max(s, seq);
  }
  if (in_reaction) {
    Rec rec;
    rec.anonymous = !gid_ok;
    rec.known = gid_ok;
    rec.gid = gid;
    rec.seq = seq;
    append(log, rec);
    return;
  }
  // Outside any reaction: the sample of the newest open claim, if any. A
  // claim never published (a lost sample) stays open and costs nothing.
  if (!gid_ok) {
    return;
  }
  if (log.open_claims.empty()) {
    if (log.old_claims > 0) {
      // Fills a forgotten claim: a sample of an earlier trigger.
      --log.old_claims;
      raise_old(log, gid, seq);
    }
    return;
  }
  const uint64_t idx = log.open_claims.back();
  log.open_claims.pop_back();
  Rec & rec = log.recs[idx - log.base];
  rec.known = true;
  rec.gid = gid;
  rec.seq = seq;
  log.by_id[{gid, seq}] = idx;
}

void apply_claim(TopicLog & log)
{
  log.open_claims.push_back(log.base + log.recs.size());
  append(log, Rec{});
}

}  // namespace

void on_publish(
  const std::string & topic, bool in_reaction, bool gid_ok, const Gid & gid, uint64_t seq)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  TopicLog & log = g_logs[topic];
  if (in_reaction || gid_ok) {
    Event ev;
    ev.kind = Event::kPublish;
    ev.in_reaction = in_reaction;
    ev.gid_ok = gid_ok;
    ev.gid = gid;
    ev.seq = seq;
    note(log, ev);  // before the apply: the merge looks at the claims it fills
  }
  apply_publish(log, in_reaction, gid_ok, gid, seq);
}

void on_claim(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  TopicLog & log = g_logs[topic];
  Event ev;
  ev.kind = Event::kClaim;
  note(log, ev);
  apply_claim(log);
}

uint64_t end(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  return it == g_logs.end() ? 0 : it->second.base + it->second.recs.size();
}

bool exact(const std::string & topic, uint64_t begin, uint64_t end)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  if (it == g_logs.end()) {
    return begin >= end;
  }
  const TopicLog & log = it->second;
  if (log.broken) {
    return false;
  }
  if (begin < log.base || end > log.base + log.recs.size()) {
    return false;
  }
  for (uint64_t i = begin; i < end; ++i) {
    if (log.recs[i - log.base].anonymous) {
      return false;
    }
  }
  return true;
}

std::optional<uint64_t> find(const std::string & topic, const Gid & gid, uint64_t seq)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  if (it == g_logs.end()) {
    return std::nullopt;
  }
  const auto rec = it->second.by_id.find({gid, seq});
  if (rec == it->second.by_id.end()) {
    return std::nullopt;
  }
  return rec->second;
}

bool old(const std::string & topic, const Gid & gid, uint64_t seq)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  if (it == g_logs.end()) {
    return false;
  }
  const auto hw = it->second.old_seq.find(gid);
  return hw != it->second.old_seq.end() && seq <= hw->second;
}

std::optional<uint64_t> seen(const std::string & topic, const Gid & gid)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  if (it == g_logs.end() || !it->second.remote) {
    return std::nullopt;
  }
  const auto s = it->second.seen.find(gid);
  if (s == it->second.seen.end()) {
    return std::nullopt;
  }
  return s->second;
}

bool remote(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  return it != g_logs.end() && it->second.remote;
}

void on_writer(const std::string & topic, const Gid & gid)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_logs[topic].writers.insert(gid);
}

void expect(const std::string & topic, size_t publishers)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  TopicLog & log = g_logs[topic];
  log.remote = true;
  log.expected = std::max(log.expected, publishers);
}

bool imported(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  return it != g_logs.end() && it->second.imported_once;
}

Writer writer(const std::string & topic, const Gid & gid)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  if (it == g_logs.end() || !it->second.remote) {
    return Writer::kPlain;
  }
  const TopicLog & log = it->second;
  if (log.announced.count(gid) != 0 || log.seen.count(gid) != 0) {
    return Writer::kGated;
  }
  // Plain only once the writer federate announced every gated publisher the
  // generator knows of: one created after the federate's earlier messages (a
  // lifecycle publisher made at configure) is unannounced until its own
  // message, and its sample is held, not taken for a plain writer's.
  return log.imported_once && log.announced.size() >= log.expected ?
         Writer::kPlain : Writer::kUnknown;
}

void note_plain(const std::string & topic, const Gid & gid)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_logs[topic].plain.insert(gid);
}

bool late_writer(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  return it != g_logs.end() && std::exchange(it->second.late_writer, false);
}

uint64_t session()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_session == 0) {
    g_session = new_session();
  }
  return g_session;
}

Packed pack(const std::string & topic, const std::string & link, size_t max)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  TopicLog & log = g_logs[topic];
  Packed out;
  if (g_session == 0) {
    g_session = new_session();
  }
  out.session = g_session;
  const uint64_t end = log.journal_base + log.journal.size();
  uint64_t & cursor = log.packed[link];
  // The newest `max` events still kept: an older one the receiver may have
  // from another link of the topic, or else it sees the gap.
  out.first = std::max({cursor, log.journal_base, end > max ? end - max : 0});
  out.overflow = out.first > cursor;
  out.writers.assign(log.writers.begin(), log.writers.end());
  for (uint64_t i = out.first; i < end; ++i) {
    out.events.push_back(log.journal[i - log.journal_base]);
  }
  cursor = end;
  log.sent = std::max(log.sent, end);
  return out;
}

Import import(
  const std::string & topic, uint64_t session, uint64_t first, const Event * events, size_t n,
  const Gid * writers, size_t nw, bool writers_more)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  TopicLog & log = g_logs[topic];
  log.remote = true;
  if (log.broken) {
    return Import::kBroken;
  }
  if (log.session != 0 && session != log.session) {
    // A restarted writer: its ordinals count from 0 again and would be
    // skipped as applied already.
    log.broken = true;
    return Import::kSession;
  }
  log.session = session;
  if (writers_more) {
    // A writer outside the announced part may be gated: no sample can be
    // told plain, so the topic keeps the count rule.
    log.broken = true;
    return Import::kWriters;
  }
  for (size_t i = 0; i < nw; ++i) {
    if (log.announced.insert(writers[i]).second && log.plain.count(writers[i]) != 0) {
      log.late_writer = true;
    }
  }
  log.imported_once = true;
  if (first > log.imported) {
    log.broken = true;
    return Import::kGap;
  }
  for (size_t i = 0; i < n; ++i) {
    if (first + i < log.imported) {
      continue;  // applied already, from another link of the topic
    }
    const Event & ev = events[i];
    if (ev.kind == Event::kClaim) {
      apply_claim(log);
    } else {
      apply_publish(log, ev.in_reaction, ev.gid_ok, ev.gid, ev.seq);
    }
    log.imported = first + i + 1;
  }
  return Import::kOk;
}

bool broken(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_logs.find(topic);
  return it != g_logs.end() && it->second.broken;
}

void reset()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_logs.clear();
  g_session = 0;
}

}  // namespace tag_log
}  // namespace ros2_lf_binding

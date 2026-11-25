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

#ifndef SYNC_ORACLE_SHOWCASE__CORPUS_HPP_
#define SYNC_ORACLE_SHOWCASE__CORPUS_HPP_

// Sync corpus: the file format both the converted showcase and the
// oracle O1 read, the lf_arrival_order and the canonical event record.
//
// A corpus is a strict YAML subset (also read by lf_eval.sync_oracle.corpus):
//
//   name: B1
//   policy: exact            # the policy the expect block was derived for
//   queue: 3                 # the queue size the expect block was derived for
//   kind: forkjoin           # optional: expect describes the P3 join
//   rows:
//     - {tick: 0, input: 0, stamp_ms: 5, payload: 1}
//   expect:
//     - signal 1 2           # rows in the input-0 and input-1 slot
//     - drop - 3             # '-' is an empty slot
//
// Row r is the r-th entry of rows (0-based), its stamp is stamp_ms * 1e6 ns.
// Header-only and inline: the converted build compiles the node sources into
// one target (lf_generated_*), so every definition here must be ODR-safe.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sync_oracle_showcase
{

inline constexpr const char * kDomain = "mf-sync/1";

struct Row
{
  uint64_t row = 0;
  int64_t tick = 0;
  uint32_t input = 0;
  int64_t stamp_ns = 0;
  uint64_t payload = 0;
};

struct Corpus
{
  std::string name;
  std::string policy;
  std::string kind;  // "forkjoin": expect is the P3 join, not O1 on the rows
  uint32_t queue = 0;
  std::vector<Row> rows;
  std::vector<std::string> expect;
  std::string digest;  // "fnv1a64:<16 hex>" over the file bytes
};

// FNV-1a 64 over the corpus bytes: an identity check that both sides read the
// same file, not a security property, so no crypto dependency.
inline std::string fnv1a64(const std::string & bytes)
{
  uint64_t h = 1469598103934665603ULL;
  for (unsigned char c : bytes) {
    h ^= c;
    h *= 1099511628211ULL;
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "fnv1a64:%016llx", static_cast<unsigned long long>(h));  // NOLINT
  return buf;
}

inline std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

inline std::string strip_comment(const std::string & line)
{
  for (size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '#' && (i == 0 || line[i - 1] == ' ' || line[i - 1] == '\t')) {
      return line.substr(0, i);
    }
  }
  return line;
}

// Value of `key:` inside a flow mapping such as "{tick: 0, input: 1}".
inline int64_t flow_int(const std::string & map, const std::string & key)
{
  const std::string needle = key + ":";
  size_t at = 0;
  while ((at = map.find(needle, at)) != std::string::npos) {
    const char before = at == 0 ? '{' : map[at - 1];
    if (before == '{' || before == ' ' || before == ',') {
      return std::stoll(map.substr(at + needle.size()));
    }
    at += needle.size();
  }
  throw std::runtime_error("corpus row without '" + key + "': " + map);
}

inline Corpus parse_corpus_text(const std::string & text)
{
  Corpus c;
  c.digest = fnv1a64(text);
  std::istringstream in(text);
  std::string raw;
  std::string section;
  while (std::getline(in, raw)) {
    const std::string line = trim(strip_comment(raw));
    if (line.empty()) {
      continue;
    }
    if (line[0] == '-') {
      std::string item = trim(line.substr(1));
      if (section == "rows") {
        Row r;
        r.row = c.rows.size();
        r.tick = flow_int(item, "tick");
        r.input = static_cast<uint32_t>(flow_int(item, "input"));
        r.stamp_ns = flow_int(item, "stamp_ms") * 1000000LL;
        r.payload = static_cast<uint64_t>(flow_int(item, "payload"));
        if (r.input > 1 || r.stamp_ns <= 0 || r.tick < 0) {
          throw std::runtime_error("corpus row out of range: " + item);
        }
        c.rows.push_back(r);
      } else if (section == "expect") {
        if (item.size() >= 2 && item.front() == '"' && item.back() == '"') {
          item = item.substr(1, item.size() - 2);
        }
        c.expect.push_back(item);
      } else {
        throw std::runtime_error("list item outside rows/expect: " + line);
      }
      continue;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("corpus line without ':': " + line);
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    section = value.empty() ? key : "";
    if (key == "name") {
      c.name = value;
    } else if (key == "policy") {
      c.policy = value;
    } else if (key == "kind") {
      c.kind = value;
    } else if (key == "queue") {
      c.queue = static_cast<uint32_t>(std::stoul(value));
    }
  }
  return c;
}

inline Corpus load_corpus(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot read corpus " + path);
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  return parse_corpus_text(ss.str());
}

// lf_arrival_order: (tag + d, input index, FIFO position). The player
// publishes all rows of tick k at one tag; a d = 0 in-process drain
// takes every record due at the tag in one firing (decision
// publication-correspondence, receiver side), and input 0's drain reaction is
// declared before input 1's. So all of input 0's rows of a tick arrive before
// any of input 1's, each input in publish order (measured on B8: every drain at
// microstep 0, input 0's two records first).
inline std::vector<Row> arrival_order(const std::vector<Row> & rows)
{
  std::vector<Row> out = rows;
  std::stable_sort(
    out.begin(), out.end(), [](const Row & a, const Row & b) {
      if (a.tick != b.tick) {
        return a.tick < b.tick;
      }
      return a.input < b.input;
    });
  return out;
}

// One slot of a tuple: absent (an empty slot of a drop) or a delivered row.
struct Slot
{
  bool present = false;
  uint32_t input = 0;
  uint64_t row = 0;
  uint64_t payload = 0;
  int64_t stamp_ns = 0;
};

struct Event
{
  bool signal = false;  // false: drop
  Slot slots[2];
};

// Canonical record body shared by the converted nodes and O1 (no node field):
// {"ev":"signal","seq":0,"stamp_ns":5000000,"slots":[{...},null]}
inline std::string event_json(const Event & e, uint64_t seq)
{
  int64_t stamp = 0;
  for (const auto & s : e.slots) {
    if (s.present) {
      stamp = s.stamp_ns;
      break;
    }
  }
  std::ostringstream o;
  o << "{\"ev\":\"" << (e.signal ? "signal" : "drop") << "\",\"seq\":" << seq <<
    ",\"stamp_ns\":" << stamp << ",\"slots\":[";
  for (int i = 0; i < 2; ++i) {
    if (i) {
      o << ",";
    }
    const Slot & s = e.slots[i];
    if (!s.present) {
      o << "null";
    } else {
      o << "{\"input\":" << s.input << ",\"row\":" << s.row << ",\"payload\":" << s.payload << "}";
    }
  }
  o << "]}";
  return o.str();
}

// Short form of the corpus expect block: "signal 1 2", "drop - 3".
inline std::string event_short(const Event & e)
{
  std::string out = e.signal ? "signal" : "drop";
  for (const auto & s : e.slots) {
    out += " " + (s.present ? std::to_string(s.row) : std::string("-"));
  }
  return out;
}

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__CORPUS_HPP_

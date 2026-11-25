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


/// @file input_period.cpp
/// @brief Runtime check of a declared minimum input period.

#include "input_period.hpp"

#include <cinttypes>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/rid_table_api.h"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
namespace input_period
{
namespace
{
struct Watch
{
  Summary s;
  bool seen = false;  // a run before this one exists
  int64_t prev_time = 0;
  uint32_t prev_microstep = 0;
  std::string prev_identity;
};

std::mutex g_mu;
// (binding key, analyzed topic) -> declared period in ns.
std::map<std::pair<std::string, std::string>, int64_t> g_declared;
// reaction id -> index into g_watches; attach order kept for the summary.
std::map<int, size_t> g_by_rid;
std::vector<Watch> g_watches;

rclcpp::Logger logger() {return rclcpp::get_logger("ros2_lf_binding");}
}  // namespace

void attach(int reaction_id, const std::string & node_class, const std::string & topic)
{
  std::lock_guard<std::mutex> lk(g_mu);
  const auto d = g_declared.find({node_class, topic});
  if (d == g_declared.end() || g_by_rid.count(reaction_id) != 0) {
    return;  // no declaration, or a late rebind of a handle already watched
  }
  Watch w;
  w.s.reaction_id = reaction_id;
  w.s.node_class = node_class;
  w.s.topic = topic;
  w.s.period_ns = d->second;
  g_by_rid[reaction_id] = g_watches.size();
  g_watches.push_back(std::move(w));
}

void observe(int reaction_id, int64_t time, uint32_t microstep, const std::string & identity)
{
  std::lock_guard<std::mutex> lk(g_mu);
  const auto it = g_by_rid.find(reaction_id);
  if (it == g_by_rid.end()) {
    return;
  }
  Watch & w = g_watches[it->second];
  ++w.s.runs;
  if (w.seen) {
    const int64_t gap = time - w.prev_time;
    if (w.s.min_gap_ns < 0 || gap < w.s.min_gap_ns) {
      w.s.min_gap_ns = gap;
    }
    if (gap < w.s.period_ns) {
      if (w.s.violations++ == 0) {
        w.s.first_time = time;
        w.s.first_microstep = microstep;
        w.s.first_identity = identity;
        w.s.first_prev_time = w.prev_time;
        w.s.first_prev_microstep = w.prev_microstep;
        w.s.first_prev_identity = w.prev_identity;
        RCLCPP_WARN(
          logger(),
          "ros2_lf_input_period: first violation: %s on '%s' (RID %d) ran at tag (%" PRId64
          ", %" PRIu32 ") for sample %s, %" PRId64 " ns after its run at tag (%" PRId64 ", %"
          PRIu32 ") for sample %s, below the declared min_input_period of %" PRId64 " ns "
          "that sizes the correspondence bounds; every violation is counted "
          "in the shutdown summary",
          w.s.node_class.c_str(), w.s.topic.c_str(), reaction_id, time, microstep,
          identity.c_str(), gap, w.prev_time, w.prev_microstep, w.prev_identity.c_str(),
          w.s.period_ns);
      }
    }
  }
  w.seen = true;
  w.prev_time = time;
  w.prev_microstep = microstep;
  w.prev_identity = identity;
}

void observe_message(int reaction_id, const rclcpp::MessageInfo & info)
{
  {
    std::lock_guard<std::mutex> lk(g_mu);
    if (g_by_rid.count(reaction_id) == 0) {
      return;  // the common case: no declaration, no tag read, no string built
    }
  }
  const auto & m = info.get_rmw_message_info();
  const auto tag = lf_current_tag();
  observe(
    reaction_id, tag.first, tag.second,
    identity(m.publisher_gid.data, sizeof(m.publisher_gid.data), m.publication_sequence_number));
}

std::string identity(const uint8_t * gid, size_t gid_len, uint64_t sequence)
{
  std::string out;
  char b[3];
  for (size_t i = 0; i < gid_len; ++i) {
    std::snprintf(b, sizeof(b), "%02x", gid[i]);
    out += b;
  }
  out += ":" + std::to_string(sequence);
  return out;
}

std::vector<Summary> summaries()
{
  std::lock_guard<std::mutex> lk(g_mu);
  std::vector<Summary> out;
  out.reserve(g_watches.size());
  for (const auto & w : g_watches) {
    out.push_back(w.s);
  }
  return out;
}

void log_summary()
{
  for (const auto & s : summaries()) {
    char line[1024];
    std::snprintf(
      line, sizeof(line),
      "ros2_lf_input_period: node=%s topic=%s rid=%d declared_ns=%" PRId64 " runs=%" PRIu64
      " violations=%" PRIu64 " min_gap_ns=%" PRId64 " first_violation=(%" PRId64 ",%" PRIu32
      ") %s after (%" PRId64 ",%" PRIu32 ") %s",
      s.node_class.c_str(), s.topic.c_str(), s.reaction_id, s.period_ns, s.runs, s.violations,
      s.min_gap_ns, s.first_time, s.first_microstep,
      s.violations > 0 ? s.first_identity.c_str() : "-", s.first_prev_time,
      s.first_prev_microstep, s.violations > 0 ? s.first_prev_identity.c_str() : "-");
    if (s.violations > 0) {
      RCLCPP_WARN(logger(), "%s", line);
    } else {
      RCLCPP_INFO(logger(), "%s", line);
    }
  }
}

void reset_for_test()
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_declared.clear();
  g_by_rid.clear();
  g_watches.clear();
}

}  // namespace input_period

void register_min_input_period(
  const std::string & node_class, const std::string & topic, int64_t period_ns)
{
  std::lock_guard<std::mutex> lk(input_period::g_mu);
  input_period::g_declared[{node_class, topic}] = period_ns;
}

}  // namespace ros2_lf_binding

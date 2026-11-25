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

/// @file xtag_link.cpp
/// @brief The cross-federate take link: tag log events on a stub link.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "tag_log.hpp"

namespace tl = ros2_lf_binding::tag_log;

void ros2_lf_xtag_pack(const char * topic, const char * link, int32_t count, rcllf_xtag_t * out)
{
  std::memset(out, 0, sizeof(*out));
  out->magic = RCLLF_XTAG_MAGIC;
  out->count = count;
  const tl::Packed p = tl::pack(topic, link, RCLLF_XTAG_MAX);
  out->session = p.session;
  out->first = p.first;
  out->overflow = p.overflow ? 1u : 0u;
  out->n = static_cast<uint32_t>(p.events.size());
  out->nw = static_cast<uint32_t>(std::min<size_t>(p.writers.size(), RCLLF_XTAG_WRITERS));
  out->writers_more = p.writers.size() > RCLLF_XTAG_WRITERS ? 1u : 0u;
  for (uint32_t i = 0; i < out->nw; ++i) {
    std::copy(p.writers[i].begin(), p.writers[i].end(), out->writer[i]);
  }
  for (size_t i = 0; i < p.events.size(); ++i) {
    const tl::Event & ev = p.events[i];
    rcllf_xtag_ev_t & w = out->ev[i];
    std::copy(ev.gid.begin(), ev.gid.end(), w.gid);
    w.seq = ev.seq;
    w.kind = ev.kind == tl::Event::kClaim ? RCLLF_XTAG_CLAIM : RCLLF_XTAG_PUBLISH;
    w.flags = (ev.in_reaction ? RCLLF_XTAG_IN_REACTION : 0u) | (ev.gid_ok ? RCLLF_XTAG_GID_OK : 0u);
  }
}

int32_t ros2_lf_xtag_import(const char * topic, const char * edge, const rcllf_xtag_t * in)
{
  if (in->magic != RCLLF_XTAG_MAGIC || in->n > RCLLF_XTAG_MAX || in->nw > RCLLF_XTAG_WRITERS) {
    std::fprintf(
      stderr, "rcllf: cross-federate take link %s on '%s': message with magic %u and %u events "
      "(layout or byte-order mismatch)\n", edge, topic, in->magic, in->n);
    // std::_Exit, not exit(): this runs inside an LF reaction while the
    // runtime's other threads (LF workers, the federate's network threads,
    // the ROS executor) are live, and exit() would run static destructors
    // under them. _Exit flushes no stdio stream, so flush them all first: a
    // redirected stdout (a file or a launch pipe) is fully buffered and held
    // the last deliveries.
    std::fflush(nullptr);
    std::_Exit(3);
  }
  std::vector<tl::Event> events(in->n);
  for (uint32_t i = 0; i < in->n; ++i) {
    const rcllf_xtag_ev_t & w = in->ev[i];
    tl::Event & ev = events[i];
    ev.kind = w.kind == RCLLF_XTAG_CLAIM ? tl::Event::kClaim : tl::Event::kPublish;
    ev.in_reaction = (w.flags & RCLLF_XTAG_IN_REACTION) != 0;
    ev.gid_ok = (w.flags & RCLLF_XTAG_GID_OK) != 0;
    std::copy(w.gid, w.gid + ev.gid.size(), ev.gid.begin());
    ev.seq = w.seq;
  }
  std::vector<tl::Gid> writers(in->nw);
  for (uint32_t i = 0; i < in->nw; ++i) {
    std::copy(in->writer[i], in->writer[i] + writers[i].size(), writers[i].begin());
  }
  const tl::Import res = tl::import(
    topic, in->session, in->first, events.data(), events.size(), writers.data(), writers.size(),
    in->writers_more != 0);
  if (tl::late_writer(topic)) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "cross-federate take link %s on topic '%s': the writer federate announced a publisher "
      "created after its earlier messages, and samples of it taken before were delivered as "
      "untagged, possibly at the wrong tag", edge, topic);
  }
  if (res != tl::Import::kOk) {
    // The receiver can no longer drop a late sample by its tag,
    // so tag-faithful delivery is lost. The count rule could deliver a sample
    // one tag late without a trace in the output, so the run ends instead.
    // RCLLF_XTAG_MAX and RCLLF_XTAG_WRITERS are the fixed sizes of the wire
    // struct (see ros2_lf_binding.h): a run that needs more must say so.
    std::string why;
    if (res == tl::Import::kWriters) {
      why = "the writer federate has more than " + std::to_string(RCLLF_XTAG_WRITERS) +
        " publishers of the topic, so a sample of a writer outside rcllf cannot be told from "
        "its own";
    } else if (res == tl::Import::kSession) {
      why = "the writer's tag log session changed (the writer process restarted), so its "
        "events no longer continue the ones applied here";
    } else if (in->overflow != 0) {
      why = "the writer's tag log events did not all arrive (more than " +
        std::to_string(RCLLF_XTAG_MAX) + " since the link's last message, and no other link "
        "of the topic brought them)";
    } else {
      why = "the writer's tag log events did not all arrive (a link message was lost, or "
        "the writer forgot the events before sending)";
    }
    std::fprintf(
      stderr, "rcllf: cross-federate take link %s on topic '%s': tag fidelity lost: %s. "
      "A late sample could be delivered one tag late, so the run ends\n",
      edge, topic, why.c_str());
    // std::_Exit, not exit(): this runs inside an LF reaction while the
    // runtime's other threads (LF workers, the federate's network threads,
    // the ROS executor) are live, and exit() would run static destructors
    // under them. _Exit flushes no stdio stream, so flush them all first: a
    // redirected stdout (a file or a launch pipe) is fully buffered and held
    // the last deliveries.
    std::fflush(nullptr);
    std::_Exit(3);
  }
  return in->count;
}

void ros2_lf_xtag_expect(const char * topic, int32_t publishers)
{
  tl::expect(topic, publishers > 0 ? static_cast<size_t>(publishers) : 1);
}

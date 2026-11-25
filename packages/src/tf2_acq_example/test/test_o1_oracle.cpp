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


// O1 oracle of the tf2 acquisition showcase. It replays the
// corpus events through a plain tf2::BufferCore in LF delivery order and
// decides each request by its own rule, written without the adapter's code:
//   - at its request tag and after every later ingest: ok when the buffer can
//     give world <- frame at the requested stamp (linear interpolation);
//   - at request tag + 55 ms (the wait bound) if still open: failure when the
//     frame was never received, reject otherwise (on_timeout reject).
// The decisions must equal the hand table corpus/t1_t7_expected.tsv, which is
// what the converted run is compared against (test_runtime_tf2_acq.py).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"

namespace
{

constexpr int64_t kWaitBoundMs = 55;  // plan constant, showcase choice
constexpr int64_t kBaseNs = 1000LL * 1000000000LL;  // keeps every stamp off 0 ("latest")

struct Decision
{
  std::string outcome;
  int64_t decided_ms = 0;
  double x = 0.0;
  double y = 0.0;
};

std::vector<std::vector<std::string>> read_tsv(const std::string & path)
{
  std::vector<std::vector<std::string>> rows;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, '\t')) {
      cells.push_back(cell);
    }
    rows.push_back(cells);
  }
  return rows;
}

tf2::TimePoint at_ms(int64_t ms)
{
  return tf2::TimePoint(std::chrono::nanoseconds(kBaseNs + ms * 1000000LL));
}

struct Open
{
  std::string name;
  std::string frame;
  int64_t stamp_ms;
  int64_t deadline_ms;
};

std::map<std::string, Decision> replay(const std::string & events_path)
{
  tf2::BufferCore core;
  std::map<std::string, Decision> out;
  std::vector<Open> open;
  // tag (ms, microstep) -> events at it, in file order
  std::map<std::pair<int64_t, int>, std::vector<std::vector<std::string>>> tags;
  for (const auto & r : read_tsv(events_path)) {
    tags[{std::stoll(r[1]), std::stoi(r[2])}].push_back(r);
  }
  auto ready = [&core](const Open & o, Decision * d, int64_t now) {
      if (!core._frameExists(o.frame) ||
        !core.canTransform("world", o.frame, at_ms(o.stamp_ms), nullptr))
      {
        return false;
      }
      const auto t = core.lookupTransform("world", o.frame, at_ms(o.stamp_ms));
      *d = {"ok", now, t.transform.translation.x, t.transform.translation.y};
      return true;
    };
  int64_t last_deadline = 0;
  auto decide_due = [&](int64_t now) {
      for (auto it = open.begin(); it != open.end(); ) {
        if (it->deadline_ms <= now) {
          Decision d;
          if (!ready(*it, &d, now)) {
            d = {core._frameExists(it->frame) ? "reject" : "failure", now, 0.0, 0.0};
          }
          out[it->name] = d;
          it = open.erase(it);
        } else {
          ++it;
        }
      }
    };
  for (const auto & [tag, rows] : tags) {
    // wake tags that fall between event tags: (deadline, 0)
    for (const auto & o : std::vector<Open>(open)) {
      if (o.deadline_ms < tag.first) {
        decide_due(o.deadline_ms);
      }
    }
    bool ingested = false;
    for (const auto & r : rows) {
      if (r[0] == "req") {
        const auto colon = r[3].find(':');
        Open o{r[3].substr(0, colon), r[3].substr(colon + 1), std::stoll(r[4]),
          tag.first + kWaitBoundMs};
        Decision d;
        if (ready(o, &d, tag.first)) {
          out[o.name] = d;
        } else {
          open.push_back(o);
          last_deadline = std::max(last_deadline, o.deadline_ms);
        }
      } else {
        geometry_msgs::msg::TransformStamped t;
        const int64_t ns = kBaseNs + std::stoll(r[4]) * 1000000LL;
        t.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
        t.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        t.header.frame_id = "world";
        t.child_frame_id = r[3];
        t.transform.translation.x = std::stod(r[5]);
        t.transform.translation.y = std::stod(r[6]);
        t.transform.rotation.w = 1.0;
        core.setTransform(t, "corpus", r[0] == "static");
        ingested = true;
      }
    }
    if (ingested) {
      for (auto it = open.begin(); it != open.end(); ) {
        Decision d;
        if (ready(*it, &d, tag.first)) {
          out[it->name] = d;
          it = open.erase(it);
        } else {
          ++it;
        }
      }
    }
    decide_due(tag.first);
  }
  decide_due(last_deadline);
  return out;
}

TEST(O1Oracle, ReplayEqualsTheHandTable)
{
  const auto got = replay(std::string(CORPUS_DIR) + "/t1_t7_events.tsv");
  const auto want = read_tsv(std::string(CORPUS_DIR) + "/t1_t7_expected.tsv");
  ASSERT_EQ(want.size(), 7u);
  ASSERT_EQ(got.size(), want.size());
  for (const auto & w : want) {
    SCOPED_TRACE(w[0]);
    ASSERT_EQ(got.count(w[0]), 1u);
    const Decision & d = got.at(w[0]);
    EXPECT_EQ(d.outcome, w[1]);
    EXPECT_EQ(d.decided_ms, std::stoll(w[2]));
    EXPECT_NEAR(d.x, std::stod(w[3]), 1e-9);  // plan constant: translation exact to 1e-9 m
    EXPECT_NEAR(d.y, std::stod(w[4]), 1e-9);
  }
}

}  // namespace

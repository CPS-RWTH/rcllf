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

#ifndef BNA_EXAMPLE__SCHEDULE_HPP_
#define BNA_EXAMPLE__SCHEDULE_HPP_

#include <cstdint>
#include <vector>

namespace bna_example
{

/// Both sensors tick every kPeriodMs (their timers repeat it as a literal,
/// which the analyzer reads). The anchor publishes ticks 0 to
/// kAnchorTicks - 1, stamped with now() (the LF tag under rcllf).
constexpr int64_t kPeriodMs = 100;
constexpr int kAnchorTicks = 12;

/// Candidate stamps, in ms after the tick's now(), one row per tick. An empty
/// row publishes nothing. tools/lf_eval/lf_eval/bna/schedule.py repeats this
/// table to derive the expected outcomes, see README.md.
inline const std::vector<std::vector<int64_t>> & candidate_offsets_ms()
{
  static const std::vector<std::vector<int64_t>> table = {
    {0},          // 0: exact match
    {20},         // 1: at the delta bound, matched
    {21},         // 2: one ms past delta, the anchor is rejected (missing)
    {-10, 10},    // 3: a tie, the earlier stamp wins
    {0, -40},     // 4: the second sample is late
    {}, {},       // 5 and 6: silent, anchor 5 waits for its timeout
    {-290},       // 7: stamp 410 admitted at anchor 5's timeout deadline
    {0}, {0}, {0}, {0},
    {},           // 12
    {0},          // 13: a lone candidate after the last anchor, expires
  };
  return table;
}

}  // namespace bna_example

#endif  // BNA_EXAMPLE__SCHEDULE_HPP_

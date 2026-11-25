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

/// @file plant_replay.hpp
/// @brief Admitted-only tag replay: `RCLLF_PLANT_REPLAY=<dir>` names a recording written by
/// `RCLLF_PLANT_RECORD` (plant_record.hpp, format version 1).
///
/// This file only reads the recording. external_input.cpp does the
/// scheduling: in a replay no stamp-anchored subscription is admitted to the
/// ingress thread; at the start tag every record whose final disposition is
/// `admitted` is inserted at (start + its elapsed tag, 0) on the topic's own
/// physical action, through the same insert and the same record-id lookup the
/// live run uses, and the stop tag is lowered to the recorded `lf_stop` tag.

#ifndef PLANT_REPLAY_HPP_
#define PLANT_REPLAY_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ros2_lf_binding
{
namespace plant_replay
{

/// Whether replay is on (RCLLF_PLANT_REPLAY set and non-empty, read once).
bool enabled();

/// The recording directory ("" when off).
std::string dir();

/// One record of a topic file, as its last entry per id leaves it.
struct Record
{
  int rid = -1;
  int64_t id = 0;
  int64_t stamp_ns = 0;
  int64_t arrival_ns = 0;
  std::optional<int64_t> tag_ns;  ///< elapsed computed tag (null when late)
  std::string disposition;        ///< final: admitted, late, overrun, collision, after_stop
  std::vector<uint8_t> bytes;     ///< rmw CDR bytes of the message
};

/// A topic file `ext.<topic>.jsonl`.
struct TopicFile
{
  std::string topic;   ///< resolved, leading slash
  std::string type;    ///< e.g. geometry_msgs/msg/PointStamped
  int64_t start_ns = 0;
  int64_t delay_ns = 0;
  int64_t reserve_ns = 0;
  int64_t grid_ns = 0;
  /// In the order of each id's first entry; disposition is the final one.
  std::vector<Record> records;
};

/// Read `path`. False with `why` naming the line on any malformed line, a
/// missing or second header, a correction without its entry, a format or
/// version other than rcllf_plant_record 1, or an admitted record with no
/// tag or no bytes (it could not be replayed as recorded), an unknown
/// disposition, or a field of the wrong json type or range.
bool load_topic(const std::string & path, TopicFile * out, std::string * why);

/// The run's termination, from run.jsonl's `lf_stop` line.
struct Stop
{
  int64_t start_ns = 0;    ///< the run header's lf_time_start (absolute)
  int64_t tag_ns = 0;      ///< elapsed
  uint32_t microstep = 0;
  std::string condition;   ///< timeout, request_stop, other
};

/// Read run.jsonl at `path`. False with `why` unless it has a run header and
/// exactly one lf_stop line.
bool load_stop(const std::string & path, Stop * out, std::string * why);

/// The topic files (`ext.*.jsonl`) in `dir`, sorted by name.
std::vector<std::string> topic_files(const std::string & dir);

/// RFC 4648 base64 with padding; false on any other character or length.
bool base64_decode(const std::string & in, std::vector<uint8_t> * out);

/// Test only: replay from `dir` (nullopt: back to the environment variable).
void set_dir_for_test(const std::optional<std::string> & dir);

}  // namespace plant_replay
}  // namespace ros2_lf_binding

#endif  // PLANT_REPLAY_HPP_

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

/// @file plant_record.hpp
/// @brief Recording with dispositions: `RCLLF_PLANT_RECORD=<dir>`.
///
/// Off unless the variable is set and non-empty; then every call below only
/// formats one JSON line and appends it to an in-memory queue under a short
/// mutex. A writer thread of its own does every file operation (directory,
/// open, write, flush), so no caller ever waits on I/O, and no caller holds
/// reactor-c's environment mutex when it records (the ingress thread records
/// after it left the critical section). The writer never logs through rclcpp
/// (a /rosout publish from it would pass the publication gate).
///
/// Files, JSON lines, one object per line, each with "ev":
/// - `ext.<topic>.jsonl`, one per stamp-anchored external topic (`/a/b` ->
///   `ext.a.b.jsonl`): a `header` line (format, version, topic, type name,
///   serialization, `lf_time_start`, delay_ns, ingress_reserve_ns, grid_ns),
///   then one `external` line per record when its disposition is first known
///   and a `correction` line when a later collision changes an admitted
///   record. A reader takes the last line per record id.
/// - `run.jsonl`: a `header` line with `lf_time_start`, the bounded
///   publishers' `publish` lines (tag, payload hash), every `actuation_late`
///   line, and one `lf_stop` line (elapsed stop tag and condition).
/// Tags are elapsed (tag - lf_time_start); stamps, arrival and insert times
/// are absolute CLOCK_REALTIME ns.

#ifndef PLANT_RECORD_HPP_
#define PLANT_RECORD_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rmw/serialized_message.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace ros2_lf_binding
{
namespace plant_record
{

/// Format version of every header line.
constexpr int kFormatVersion = 1;

/// Whether recording is on (RCLLF_PLANT_RECORD, read once).
bool enabled();

/// The recording directory ("" when off).
std::string dir();

/// File name of a topic's record: leading slash dropped, '/' -> '.'.
std::string topic_file(const std::string & resolved_topic);

/// One stamp-anchored topic: write its header line. `start_ns` is
/// lf_time_start(). Idempotent per topic.
void open_topic(
  const std::string & resolved_topic, const std::string & type_name, const std::string & rmw,
  int64_t start_ns, int64_t delay_ns, int64_t reserve_ns, int64_t grid_ns);

/// One external record whose disposition is now known.
struct External
{
  int rid = -1;
  int64_t id = 0;
  int64_t stamp_ns = 0;
  int64_t arrival_ns = 0;
  std::optional<int64_t> insert_ns;  ///< after the critical section, if entered
  std::optional<int64_t> tag_ns;     ///< elapsed computed tag, if computed
  const char * disposition = "";
  int64_t serialize_ns = -1;         ///< ingress-thread cost of serializing, -1 none
  std::vector<uint8_t> bytes;        ///< rmw serialized message (CDR)
  /// In a replay (RCLLF_PLANT_REPLAY), the live record's id this
  /// record re-inserts; written as "replay_of" only when set.
  std::optional<int64_t> replay_of;
};
void external(const std::string & resolved_topic, External e);

/// A later disposition of record `id` (collision): the reader's last entry.
void correction(
  const std::string & resolved_topic, int rid, int64_t id, const char * disposition);

/// Publication gate: remember the type support of `publisher` (record on).
void note_publisher(const void * publisher, const void * type_support);
/// Publication gate: `publisher` is finalized; forget it.
void forget_publisher(const void * publisher);

/// Publication gate, just before the real publish of `payload` (a ROS
/// message, or an rcl_serialized_message_t when `serialized`) by reaction
/// `rid` on a bounded topic (output_deadline_ns): one `publish` line with
/// the elapsed tag and the FNV-1a 64 hash of the serialized payload.
void publication(
  const void * publisher, const void * payload, bool serialized, int rid,
  const std::string & resolved_topic);

/// Output deadline check: one `actuation_late` line (absolute tag time in,
/// written elapsed).
void actuation_late(
  int rid, const std::string & resolved_topic, int64_t tag_time, uint32_t microstep,
  int64_t physical_ns, int64_t lateness_ns, int64_t bound_ns);

/// The binding's shutdown hook (a reaction(shutdown), at the stop tag): one
/// `lf_stop` line, once per run. `source` names the hook.
void lf_stop(const char * source);

/// Teardown: write what is queued, close every file, join the writer, log a
/// one-line summary. Nothing when off.
void close();

/// "timeout" when (stop_time, stop_microstep) is (start + duration, 0) with a
/// duration set, "request_stop" for any later microstep, else "other".
const char * stop_condition(
  int64_t start_ns, int64_t duration_ns, int64_t stop_time, uint32_t stop_microstep);

/// The CDR bytes the publish lines hash: `msg` serialized into `buf` with
/// every alignment padding byte zero, whatever `buf` held before.
bool serialize_zero_padded(
  const void * msg, const rosidl_message_type_support_t * type, rmw_serialized_message_t * buf);

/// FNV-1a 64 of `n` bytes (the publish lines' hash).
uint64_t fnv1a64(const uint8_t * data, size_t n);

/// RFC 4648 base64 with padding.
std::string base64(const uint8_t * data, size_t n);

/// Test only: record into `dir` (nullopt: back to the environment variable),
/// after close() of a previous recording.
void set_dir_for_test(const std::optional<std::string> & dir);

}  // namespace plant_record
}  // namespace ros2_lf_binding

#endif  // PLANT_RECORD_HPP_

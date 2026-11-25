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

/// @file policy_table.hpp
/// @brief Immutable-after-init execution-policy table, keyed by
///        (node_class, topic-as-in-model).
///
/// The generator (`colcon lf-configure` / `lf-generate`) resolves the sparse
/// user-owned `execution.yaml` against the extracted system model and emits a
/// fully-resolved `exec_config.json` next to the staged `system.json`:
///
/// @code
/// { "schema_version": 1,
///   "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
///   "subscriptions": [
///     {"node_class": "SubscriberNode", "topic": "sensor_data",
///      "take_mode": "block", "take_timeout_ms": 250} ] }
/// @endcode
///
/// All per-key merging (built-in default < defaults: < nodes: <
/// subscriptions:) happens in Python; node scope is flattened away, so this
/// side does a dumb lookup.  `topic` is the model form and may be relative
/// (e.g. `sensor_data` under namespace `/robot_1`), which is why
/// resolve_policy() compares topics with the same three-way slash-tolerant
/// comparison node_binding.cpp uses when matching RID entries against live
/// subscriptions.
///
/// Contract (frozen, design §2.4):
///   - absent file            -> built-in defaults, silently
///   - corrupt / unparseable  -> loud init FAILURE (fail-closed)
///   - wrong schema_version   -> loud init FAILURE (fail-closed)
///
/// Lifecycle: the table is built exactly once during ros2_lf_runtime_init()
/// (before any ros2_lf_bind_node startup reaction runs) and is read-only
/// afterwards.  resolve_policy() is called once per handle at registration and
/// the result is cached in HandleEntry, so the dispatch hot path never touches
/// this table.
///
/// Runtime reconfiguration (NOT implemented; interface shape only).
/// policy_table_load() / policy_table_set_for_test() are the *single writer
/// point*: every mutation of the table goes through one of them.  A future
/// runtime-reconfig service would (a) guard the table with a reader-writer
/// lock or swap an immutable shared_ptr snapshot atomically, (b) call the same
/// single writer to install the new table, and (c) re-run resolve_policy() over
/// every SUBSCRIPTION row of g_handle_table to refresh the cached
/// HandleEntry::take_mode / take_timeout_ms values (which is why those values
/// are cached per handle rather than looked up per dispatch).  Nothing here is
/// thread-safe today because nothing writes after init; adding the service is
/// therefore a change confined to the writer point plus a re-resolve pass.

#ifndef POLICY_TABLE_HPP_
#define POLICY_TABLE_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "take_mode.hpp"

namespace ros2_lf_binding
{

/// The resolved policy for one subscription (or the table-wide defaults).
struct TakePolicy
{
  TakeMode take_mode = TakeMode::Skip;
  int64_t take_timeout_ms = 100;
};

/// One resolved row of exec_config.json's `subscriptions` array.
struct PolicyRow
{
  std::string node_class;
  std::string topic;      ///< Model form; may be relative (no leading slash).
  TakePolicy policy;
};

/// Result of parsing an exec_config.json.
struct PolicyTableLoadResult
{
  bool ok = true;
  std::string error;      ///< Human-readable reason when `ok` is false.
  bool file_present = false;
};

/// The exec_config.json schema version this binding understands.
constexpr int kExecConfigSchemaVersion = 1;

/// Parse `path` and install it as the process-wide policy table.
///
/// Single writer point (see the reconfiguration note above).  An empty `path`
/// or a non-existent file leaves the built-in defaults in place and reports
/// `ok == true`, `file_present == false`.  Malformed JSON, a missing/mismatched
/// `schema_version`, or a bad policy value reports `ok == false` with `error`
/// set and leaves the previously installed table untouched.
PolicyTableLoadResult policy_table_load(const std::string & path);

/// Look up the policy for one bound subscription.
///
/// Matches `node_class` exactly (RIDs are per class, not per node instance —
/// design §1 non-goals) and the topic slash-tolerantly against both the
/// declared topic and the runtime-resolved topic name, mirroring
/// node_binding.cpp's three-way comparison.  Falls back to the table defaults
/// and increments the miss counter when no row matches.
TakePolicy resolve_policy(
  const std::string & node_class, const std::string & declared_topic,
  const std::string & resolved_topic);

/// Convenience overload: declared topic only.
TakePolicy resolve_policy(const std::string & node_class, const std::string & topic);

/// The table-wide defaults (built-in, or `defaults` from exec_config.json).
TakePolicy policy_table_defaults();

/// Number of resolved subscription rows currently installed.
size_t policy_table_row_count();

/// Number of resolve_policy() calls that found no matching row.  A miss means
/// a real key-form bug: the generator emits one row per model subscription.
uint64_t policy_table_miss_count();

/// Up to `kMaxMissExamples` (node_class, topic) pairs from the earliest
/// resolve_policy() misses, in first-seen order.  Used by the one-shot
/// post-bind summary in runtime.cpp; bounded so a system with many misses
/// does not grow this without limit.
constexpr size_t kMaxMissExamples = 5;
const std::vector<std::pair<std::string, std::string>> & policy_table_miss_examples();

/// Rows currently installed (tests and diagnostics).
const std::vector<PolicyRow> & policy_table_rows();

/// Install a table directly, bypassing JSON parsing (tests only).
/// Second half of the single writer point.
void policy_table_set_for_test(const TakePolicy & defaults, const std::vector<PolicyRow> & rows);

/// Restore built-in defaults and clear all rows and counters (tests only).
void policy_table_reset();

/// Log one WARN per env var that overrides resolved entries, naming the count
/// of resolved rows whose value the override actually changes (design §2.2).
/// Called once from ros2_lf_runtime_init() after the table is
/// installed.  The env vars do not apply to a subscription on a declared
/// `external_topics` topic: it dispatches on arrival and never takes,
/// so those rows are left out and the WARN names how many.
void policy_table_log_env_overrides();

/// Text of the ROS2_LF_TAKE_MODE override WARN, empty when the var is unset.
std::string policy_table_take_mode_override_message();

/// Text of the ROS2_LF_MAXWAIT_MS (alias ROS2_LF_TAKE_TIMEOUT_MS) override
/// WARN, empty when both are unset; names both when they disagree.
std::string policy_table_take_timeout_override_message();

/// Log a single one-shot WARN summarizing lookup misses across the whole
/// bind, if any occurred: total miss count plus up to `kMaxMissExamples`
/// example (node_class, topic) pairs, pointing at `colcon lf-analyze &&
/// colcon lf-configure` as the likely fix (key-form drift between the model
/// and exec_config.json).  No-op when `policy_table_miss_count() == 0`.
/// resolve_policy() is only ever called during per-handle binding. The one
/// caller, ros2_lf_runtime_wait_ready() in runtime.cpp, runs before any bind,
/// so it currently sees no misses (see the comment there).
void policy_table_log_miss_summary();

}  // namespace ros2_lf_binding

#endif  // POLICY_TABLE_HPP_

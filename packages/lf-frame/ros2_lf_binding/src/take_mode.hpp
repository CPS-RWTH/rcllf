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

/// @file take_mode.hpp
/// @brief Subscription take policy (skip vs bounded block) and take statistics.
///
/// The LF reaction for a subscription fires at the logical tag; the DDS
/// payload may not have arrived yet.  The take policy selects what happens
/// on a missed take:
///   - skip  (default): drop the reaction invocation; a trigger that knows
///            a sample is owed (a recorded write, or a known write count)
///            first polls for it up to the timeout, one
///            with an unknown write count skips at once
///   - block: poll until the payload arrives or the timeout (default 100 ms)
///            elapses, then warn and skip
///
/// Values are resolved per (node_class, topic) from exec_config.json at
/// registration time (see policy_table.hpp) and cached in HandleEntry.
/// ROS2_LF_TAKE_MODE / ROS2_LF_MAXWAIT_MS remain available as *per-key*
/// process-wide overrides: each var overrides only its own key, and an unset
/// var leaves the per-subscription value intact.  Hence the accessors return
/// std::optional (an unset var must be distinguishable from a var set to the
/// built-in default) and are not permanently cached (so tests can re-read them
/// across cases).
///
/// Skip keeps a per-subscription debt (probe P17e): a missed take owes one
/// sample. A later take with `remaining` writes left at this tag takes up to
/// remaining + debt samples, drops the oldest extras (at most debt) as late and
/// delivers the rest in order, so one late payload costs one reported sample
/// instead of a silent permanent one-tick slip. A lone sample held while debt
/// is open cannot be told from an on-time one: it is delivered and may be one
/// tag late (WARN once), and the debt stays open until two samples are held at
/// one take. No writer is charged a miss: the exact per-writer drop of a lone
/// late sample was removed because it could turn a lost sample into a lasting
/// slip. A trigger whose write count is unknown
/// (`remaining` 0) owes nothing when it misses.
///
/// Drop by tag replaces the count rule when the trigger's writes
/// were published in this process: tag_log.hpp records each write's identity,
/// and a trigger carrying k writes owns the next k records of its topic. A
/// sample of the trigger is delivered, a sample of an earlier trigger (it
/// missed its take) is dropped and reported, a sample of a later trigger is
/// held for it, and a sample with no record (published outside any reaction
/// and not claimed) is delivered as an extra and counted as `untagged`. A
/// sample whose record was forgotten (more than 4096 later writes on its
/// topic) is of an earlier trigger and dropped all the same. A trigger whose
/// writes are not recorded here (a cross-federate link, or an unknown write
/// count) falls back to the count rule above, and so does the reader from
/// then on: its tag cursor can no longer be aligned. A sample held for that
/// trigger is delivered first.
///
/// Both modes record statistics (global atomics; per-topic map touched only
/// on the failed-first-take path).  Summaries are logged at shutdown and
/// periodically from the dispatch path, because SIGTERM/SIGKILL container
/// stops never reach the atexit shutdown hook.

#ifndef TAKE_MODE_HPP_
#define TAKE_MODE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{

enum class TakeMode
{
  Skip,
  Block,
};

/// Per-topic failure-path counters (durations in nanoseconds).
struct TopicTakeStats
{
  uint64_t skipped = 0;
  uint64_t waited = 0;
  uint64_t wait_total_ns = 0;
  uint64_t wait_max_ns = 0;
  uint64_t timeouts = 0;
  uint64_t late_dropped = 0;  // owed samples that came late and were dropped
  uint64_t untagged = 0;      // samples with no record, delivered as extras
  uint64_t poll_timeouts = 0;    // count-rule polls that ran to the take timeout
  uint64_t poll_timeout_ns = 0;  // time those polls spent
  uint64_t reader_lost = 0;      // samples the reader replaced before a take
  uint64_t after_deadline = 0;   // samples received after tag + T, dropped as lost
  uint64_t unknown_missed = 0;   // unknown-count federate takes that found nothing by tag + T
  uint64_t count_rule = 0;       // drop-by-tag takes served by the count rule
  uint64_t debt_capped = 0;      // skip misses not owed because the debt was at its cap
};

/// Copy of all counters, for tests and the summary logger.
struct TakeStatsSnapshot
{
  uint64_t first_try = 0;
  uint64_t skipped = 0;
  uint64_t waited = 0;
  uint64_t wait_total_ns = 0;
  uint64_t wait_max_ns = 0;
  uint64_t timeouts = 0;
  uint64_t late_dropped = 0;
  uint64_t untagged = 0;
  uint64_t unsettled = 0;  // samples held ahead of their writer's events right now
  uint64_t poll_timeouts = 0;
  uint64_t poll_timeout_ns = 0;
  uint64_t reader_lost = 0;  // samples a reader replaced before a take, all topics
  uint64_t after_deadline = 0;  // samples received after tag + T, all topics
  uint64_t unknown_missed = 0;  // unknown-count federate misses, all topics
  uint64_t count_rule = 0;  // drop-by-tag takes served by the count rule, all topics
  uint64_t debt_capped = 0;  // skip misses past the debt cap, all topics
  std::map<std::string, TopicTakeStats> topics;
};

/// Pure parser: nullptr/""/"skip" -> Skip, "block" -> Block,
/// anything else -> warn once at call time and Skip.
TakeMode parse_take_mode(const char * val);

/// The maxwait override variable and its alias. The resolved
/// key (exec_config.json, PolicyRow, HandleEntry) keeps the name
/// take_timeout_ms; only the user-facing variable is renamed.
inline constexpr const char * kMaxwaitEnv = "ROS2_LF_MAXWAIT_MS";
inline constexpr const char * kMaxwaitEnvAlias = "ROS2_LF_TAKE_TIMEOUT_MS";

/// Pure parser: nullptr/"" -> 100; invalid or negative -> warn (naming
/// `var`) and 100.
int64_t parse_take_timeout_ms(const char * val, const char * var = kMaxwaitEnv);

/// The maxwait override as read from ROS2_LF_MAXWAIT_MS and its alias
/// ROS2_LF_TAKE_TIMEOUT_MS. An empty variable counts as unset.
/// Both set to the same value: that value. Both set to different values: the
/// override is refused (`value` empty, `conflict` names both), so every
/// subscription keeps its resolved value, the way the binding answers any
/// bad tuning variable with a WARN instead of a refused start.
struct MaxwaitEnv
{
  std::optional<int64_t> value;
  /// The variable(s) the value came from, e.g. "ROS2_LF_MAXWAIT_MS=250" or
  /// "ROS2_LF_TAKE_TIMEOUT_MS=250 (alias of ROS2_LF_MAXWAIT_MS)".
  std::string source;
  /// Non-empty when both are set with different values.
  std::string conflict;
};
MaxwaitEnv maxwait_env();

/// Per-key env overrides.  std::nullopt means "the variable is unset (or
/// empty)", i.e. the per-subscription resolved value stands.  Read on every
/// call — never permanently cached — so that a variable set to the built-in
/// default is still distinguishable from an unset one and so gtest cases can
/// setenv/unsetenv between assertions.  take_timeout_ms_env() is
/// maxwait_env().value: also nullopt when the two maxwait variables disagree.
std::optional<TakeMode> take_mode_env();
std::optional<int64_t> take_timeout_ms_env();

/// The skip debt rule below exists (tests use it to build on older trees).
#define ROS2_LF_TAKE_DEBT 1

/// Reader overflow is detected and reported (tests use it to
/// build on older trees). Every legacy and tag take site checks the
/// publication sequence number of a taken sample against the last one taken
/// from the same writer; the samples in between never reached a take
/// (replaced in a full KeepLast reader, which the middleware never reports,
/// or lost on the wire of a best-effort reader, whose DDS event rcllf takes
/// and drops), and they are counted in `reader_lost` and handed to the
/// message_lost handler exactly once, whatever the reliability. A reader
/// with a content filter on is not checked; a filter change resets the count.
///
/// At a known write count the replaced samples also stand in for
/// that many empty takes: the count rule credits the tag that found the gap,
/// drop by tag the records the replaced samples belong to. Those takes end at
/// once, in fast mode as in real time, owe no debt, poll nothing and raise no
/// miss WARN (the gap WARN and `reader_lost` already declared the loss). On
/// the count rule a gap first settles open debt (the oldest unfilled
/// expectation is the oldest hole) and only the rest credits the tag. A
/// writer first taken from at that very take has no baseline, so its gap is
/// unknown and its replaced samples still cost a poll each (known limit).
#define ROS2_LF_READER_GAP 1

/// Take one message honoring `mode`.  Returns true if a message was taken
/// into `message`.  Records statistics globally and per `topic`.
/// `remaining` is the number of writes of the trigger left at this tag,
/// this one included, or 0 when the caller does not know it (then a skip-mode
/// miss adds no debt, probe P17f). `reported`, when
/// given, receives the number of owed samples this call dropped as
/// late plus the samples the reader replaced before this take, for
/// the subscription's message_lost handler.
bool take_with_mode(
  rclcpp::SubscriptionBase & sub, void * message, rclcpp::MessageInfo & message_info,
  TakeMode mode, int64_t timeout_ms, const std::string & topic, int remaining = 1,
  uint64_t * reported = nullptr);

/// Drop by tag exists (tests use it to build on older trees).
#define ROS2_LF_TAKE_BY_TAG 1

/// A sample taken serialized, to be delivered by the caller.
struct TakenSample
{
  std::shared_ptr<rclcpp::SerializedMessage> data;
  rclcpp::MessageInfo info;
};

/// What one invoke of a subscription reaction delivers under drop by tag.
struct TagTake
{
  bool count_rule = false;           ///< not served by tag: use take_with_mode
  std::vector<TakenSample> deliver;  ///< in take order
  uint64_t lost = 0;                 ///< late drops and reader overflow, for message_lost
};

/// One invoke of subscription reaction `rid` whose trigger has `remaining`
/// writes left at this tag, this one included. Delivers one
/// sample of the trigger, and on the last invoke every one already there;
/// samples with no record ride along. `count_rule` when the trigger's writes
/// are not recorded in this process or `remaining` is 0 (unknown count); for
/// a cross-federate link or an unknown count this holds for the reader from
/// then on. A sample already held for such a trigger is delivered instead.
TagTake take_by_tag(
  int rid, rclcpp::SubscriptionBase & sub, TakeMode mode, int64_t timeout_ms,
  const std::string & topic, int remaining);

/// An invoke of `rid` that takes nothing (no subscription yet, or its node is
/// inactive): the trigger's writes are used up all the same. Returns the
/// samples of the trigger an earlier take already held, now dropped and
/// counted in late_dropped. A reader on the count rule for good uses up
/// nothing and returns 0.
uint64_t tag_pass(int rid, int remaining);

/// Deserialize `sample` into `message`, a message of `sub`'s type.
bool deserialize_taken(
  rclcpp::SubscriptionBase & sub, const TakenSample & sample, void * message);

TakeStatsSnapshot take_stats_snapshot();

/// Log the summary (one global line, one line per topic with failures) at INFO.
void take_stats_log_summary(const char * reason);

/// Throttled summary for the dispatch hot path: logs at most every 10 s.
void take_stats_maybe_periodic_log();

/// Zero all counters (tests only).
void take_stats_reset();

/// The writers `sub` already sees before the first tag, by
/// GID. Their publications are numbered from 1, so a gap before the first
/// sample taken from one of them is a reader overflow and is reported
/// (reader_lost), not taken as the baseline. A writer not named here keeps the
/// baseline rule: a volatile reader that joined late never saw its earlier
/// samples, and that is no loss. Called by ros2_lf_runtime_wait_ready().
void take_note_start_writers(
  rclcpp::SubscriptionBase & sub, const std::vector<std::array<uint8_t, 16>> & gids);

}  // namespace ros2_lf_binding

#endif  // TAKE_MODE_HPP_

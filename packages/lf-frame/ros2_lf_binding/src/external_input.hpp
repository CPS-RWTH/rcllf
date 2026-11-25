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

/// @file external_input.hpp
/// @brief External topic input: subscriptions on topics declared
///        in execution.yaml's `external_topics` enter LF time as a physical
///        action that the ingress thread schedules at message arrival.
///
/// Per subscription RID the module keeps the LF action, its wake thunk and a
/// FIFO of taken messages. The ingress thread appends each message with a
/// process-wide record id and schedules the action with that id as payload.
/// The reaction runs the user callback once per record with an id up to its
/// payload, so two arrivals that LF folds into one tag still run twice.
#ifndef EXTERNAL_INPUT_HPP_
#define EXTERNAL_INPUT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace ros2_lf_binding
{
namespace external_input
{

/// The generated thunk calls lf_schedule_copy(action, 0, &record, 1) on an int64_t
/// action. <= 0 means refused.
using WakeFn = int (*)(void * action, int64_t record);

/// Startup reaction of the subscription's reactor: remember the action.
/// For a RID whose topic is stamp-anchored, the action must be a
/// plain `physical action` (no min delay, no min spacing); else fatal.
void register_action(int rid, WakeFn wake, void * action);

/// ros2_lf_bind_node or a late bind bound `rid` to a subscription. Together
/// with register_action (either order) this admits that subscription to
/// ingress, once per subscription.
/// No-op for a RID that never registers (not an external subscription).
void on_bound(int rid);

/// ros2_lf_bind_node: `rid` is a subscription entry of a lifecycle node, so it
/// binds late when the node creates its subscription.
void on_lifecycle_entry(int rid);

/// After a change_state request ran, and when ingress retires an expired
/// external admission: schedule the action of every external
/// lifecycle RID without a live admitted subscription. Its reaction runs
/// bind_late_handle, which admits a subscription the transition created.
/// Returns the number scheduled.
size_t poke_lifecycle();

/// Ingress thread: one message taken from `source`, the subscription of `rid`.
void on_sample(
  int rid, const rclcpp::SubscriptionBase::SharedPtr & source, std::shared_ptr<void> message,
  const rclcpp::MessageInfo & info);

/// Reaction: run the callback once per record with id <= `record`, in
/// arrival order. A record taken from a subscription that is no longer the
/// RID's binding is dropped instead (logged, `external_lifecycle_drop` in the
/// JSON trace, dispatch_end outcome LOST). Returns the number of callbacks run.
size_t dispatch(void * self, int rid, int64_t record);

/// Startup: `topic` is declared in `external_topics` (leading slash optional).
void declare_topic(const std::string & topic);

/// The constants of a topic whose
/// messages are anchored at `header.stamp`. The tag of a message is
/// `start + ceil(stamp + delay_ns + reserve_ns - start, grid_ns)` at
/// microstep 0; a message arriving after `stamp + delay_ns` is late.
struct StampAnchor
{
  int64_t delay_ns = 0;    ///< T: arrival deadline after header.stamp
  int64_t reserve_ns = 0;  ///< the binding's own arrival-to-insert budget
  int64_t grid_ns = 1;     ///< tags land on this grid, anchored at start
};

/// Startup, right after declare_topic: anchor `topic` at its stamp.
void declare_stamp_anchor(const std::string & topic, const StampAnchor & anchor);

/// The anchor declared for the resolved `topic`, or nullptr (arrival anchor).
const StampAnchor * stamp_anchor_of(const std::string & topic);

/// What became of one message (wire values: rcllf_trace_ext_disposition).
enum class Disposition : uint8_t
{
  ARRIVAL = 0,     ///< arrival anchor: scheduled at arrival (lf_schedule_copy)
  ADMITTED = 1,    ///< stamp anchor: inserted at its tag
  LATE = 2,        ///< arrived after stamp + delay_ns
  OVERRUN = 3,     ///< on time, but its tag was at or before the current tag
  COLLISION = 4,   ///< two on-time messages of the topic on one grid cell
  AFTER_STOP = 5,  ///< its tag is after the stop tag
  REFUSED = 6,     ///< reactor-c refused the schedule (stopping)
  TF_MIXED_STAMPS = 7,  ///< a TFMessage with no single transforms[*].header.stamp
};

/// Per-RID counts of every disposition (summary, tests).
struct Counters
{
  uint64_t admitted = 0;
  uint64_t late = 0;
  uint64_t overrun = 0;
  uint64_t collision = 0;
  uint64_t after_stop = 0;
  uint64_t refused = 0;
  uint64_t tf_mixed_stamps = 0;
};
Counters counters(int rid);

/// Shutdown: one line per stamp-anchored RID with its counters, next to
/// take_stats. Nothing for a run without stamp anchors.
void log_summary();

/// The scheduler of the running program, as the stamp anchor needs it.
/// Production resolves the executable's threaded reactor-c with
/// dlsym (the binding links its own single-threaded copy, see runtime.cpp);
/// a test installs a fake to drive the tag deterministically.
struct SchedulerOps
{
  /// The first environment, or nullptr outside an LF program.
  void * (*env)();
  /// lf_time_start(): physical time of the start tag.
  int64_t (*start_ns)();
  /// lf_critical_section_enter / exit: env->mutex, which the runtime holds
  /// while it advances the tag.
  int (*enter)(void * env);
  int (*exit)(void * env);
  /// lf_is_tag_after_stop_tag for (time, microstep 0).
  bool (*after_stop)(void * env, int64_t time);
  /// env->current_tag; call under enter.
  void (*current_tag)(void * env, int64_t * time, uint32_t * microstep);
  /// Under enter: insert `action`'s event at (time, 0) with `record` as the
  /// int64_t payload, unless an event of the action is queued at that tag
  /// already, in which case that event is removed instead (collision).
  /// Returns INSERT_OK, INSERT_COLLIDED, INSERT_PAST when the tag is at or
  /// before the current tag, or INSERT_REFUSED when reactor-c's
  /// _lf_schedule_at_tag refused a future tag (stopping, or no event left).
  int (*insert)(void * env, void * action, int64_t time, int64_t record);
  /// Empty when every scheduler symbol above resolves to the executable;
  /// otherwise why not, naming the symbol. Checked at attach: a missing or
  /// foreign symbol refuses the run, so no call above can silently skip the
  /// lock or lock the wrong environment.
  std::string (*symbol_problem)();
  /// Replay only (nullptr refuses a replay). Under enter:
  /// lf_set_stop_tag(env, (time, microstep)), which only ever lowers it.
  void (*set_stop_tag)(void * env, int64_t time, uint32_t microstep);
  /// Under enter: env->stop_tag.
  void (*stop_tag)(void * env, int64_t * time, uint32_t * microstep);
  /// reactor-c's keepalive_specified: an empty event queue then advances to
  /// the stop tag instead of ending the run one microstep later.
  bool (*keepalive)();
};
constexpr int INSERT_OK = 1;
constexpr int INSERT_COLLIDED = 2;
constexpr int INSERT_PAST = -1;
constexpr int INSERT_REFUSED = -2;

/// Where one scheduler symbol resolved.
struct SymbolOrigin
{
  void * address = nullptr;       ///< nullptr: the name does not resolve
  const void * object = nullptr;  ///< identity of the defining object (its link_map)
  std::string path;               ///< the defining object's file name, for the log
};
using SymbolResolver = SymbolOrigin (*)(const char * name);

/// The production resolver: dlsym(RTLD_DEFAULT) plus the defining object
/// dladdr1 reports.
SymbolOrigin resolve_symbol(const char * name);

/// The main program's link_map, the identity resolve_symbol reports for a
/// symbol the executable defines (nullptr if it cannot be determined).
const void * main_program_object();

/// Resolves every scheduler symbol the stamp anchor calls with `resolve` and
/// requires each to be defined by the object `executable`: the binding is a
/// shared library carrying its own reactor-c, and a symbol the executable
/// does not export resolves to that copy, whose critical section does not
/// lock the executable's environment. Returns empty when all pass, else why
/// not for the first offending symbol. `origins`, when given, gets one line
/// per symbol naming the object it resolved to.
std::string check_scheduler_symbols(
  SymbolResolver resolve, const void * executable, std::vector<std::string> * origins);

/// Byte offsets of header.stamp.sec / .nanosec in a message described by
/// `top` (C++ introspection). Requires `header` to be std_msgs/msg/Header and
/// `header.stamp` builtin_interfaces/msg/Time by namespace and name, not
/// just by shape. False with `why` set otherwise.
bool locate_header_stamp_in(
  const rosidl_typesupport_introspection_cpp::MessageMembers * top, size_t * sec_off,
  size_t * nsec_off, std::string * why);

/// tf2_msgs/msg/TFMessage has no top-level header; each of its
/// `transforms` carries its own header.stamp. For a `top` that is
/// tf2_msgs::msg::TFMessage by namespace and name, sets `seq` to the
/// `transforms` sequence and the offsets to header.stamp.sec / .nanosec
/// inside one element. False with `why` set otherwise.
bool locate_tf_stamps_in(
  const rosidl_typesupport_introspection_cpp::MessageMembers * top,
  const rosidl_typesupport_introspection_cpp::MessageMember ** seq, size_t * sec_off,
  size_t * nsec_off, std::string * why);

/// The anchor rule of a TFMessage: 1 with `stamp` set
/// when every transform carries the same header.stamp, 0 when there is no
/// transform, -1 when two stamps differ. A message without 1 is ended as
/// TF_MIXED_STAMPS, never split and never anchored at a guessed stamp.
int common_tf_stamp(
  const rosidl_typesupport_introspection_cpp::MessageMember * seq, const void * msg,
  size_t sec_off, size_t nsec_off, int64_t * stamp);

/// Largest delay_ns, ingress_reserve_ns or grid_ns a stamp anchor accepts
/// (2^60 ns, about 36 years): with header.stamp below 2^31 s and the start
/// tag below 2^61 ns, every step of the tag arithmetic then fits int64. The
/// execution.yaml validator applies the same bound.
constexpr int64_t kAnchorMaxNs = int64_t{1} << 60;

/// Replay counters of one RID (RCLLF_PLANT_REPLAY).
struct ReplayCounters
{
  uint64_t inserted = 0;    ///< admitted records inserted at their live tag
  uint64_t delivered = 0;   ///< of those, handed to the callback's reaction
  uint64_t past_stop = 0;   ///< admitted live but tagged after the live stop tag
  uint64_t not_admitted = 0;  ///< late, overrun, collision, after_stop: listed, never replayed
};
ReplayCounters replay_counters(int rid);

/// log_summary reported a replay as FAILED (a recorded topic or
/// RID never replayed, an inserted record not delivered, or a stop tag other
/// than the live run's); the runtime then exits 3.
bool replay_failed();

/// Test only: use `ops` (nullptr restores the dlsym-resolved production set).
void set_scheduler_ops_for_test(const SchedulerOps * ops);

/// Whether the resolved `topic` was declared external. The readiness wait
/// skips such subscriptions, nothing in the system publishes them.
bool is_declared_topic(const std::string & topic);

/// Records taken but not yet dispatched (tests).
size_t pending(int rid);

/// Test only. The next record id is `id` + 1.
void set_last_record_id_for_test(int64_t id);

/// Test only: forget every registration and record.
void reset_for_test();

}  // namespace external_input
}  // namespace ros2_lf_binding

#endif  // EXTERNAL_INPUT_HPP_

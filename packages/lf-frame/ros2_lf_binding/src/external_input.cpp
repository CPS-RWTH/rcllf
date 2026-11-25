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

/// @file external_input.cpp
/// @brief External topic input, see external_input.hpp.

#include "external_input.hpp"

#include <dlfcn.h>
#include <link.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rcutils/allocator.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"

extern "C" {
#include "core/environment.h"
#include "core/lf_token.h"
#include "core/lf_types.h"
#include "core/utils/pqueue_tag.h"
}

#include "corr_runtime.hpp"
#include "correspondence.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "input_period.hpp"
#include "plant_record.hpp"
#include "plant_replay.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "semantic_trace.hpp"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

extern "C" int64_t lf_time_physical(void);

namespace ros2_lf_binding
{
extern std::vector<HandleEntry> g_handle_table;

namespace external_input
{
namespace
{

rclcpp::Logger logger() {return rclcpp::get_logger("ros2_lf_binding");}

struct Record
{
  int64_t id = 0;
  std::shared_ptr<void> message;
  std::shared_ptr<rclcpp::MessageInfo> info;
  corr::Gid gid{};
  uint64_t ordinal = 0;
  /// The subscription the message was taken from. A record whose source is no
  /// longer the RID's live binding at dispatch is dropped, as stock drops the
  /// queue of a subscription the node destroyed.
  std::weak_ptr<rclcpp::SubscriptionBase> source;
  /// Stamp anchors only: kept so a record removed by a later
  /// collision can be reported with its own arrival and stamp.
  int64_t arrival = 0;
  int64_t stamp = 0;
};

/// A grid cell of a stamp-anchored RID with a queued or collided event.
struct Cell
{
  int64_t record = 0;  ///< the queued record, meaningless once collided
  bool collided = false;
};

struct Slot
{
  WakeFn wake = nullptr;
  void * action = nullptr;
  bool bound = false;
  /// Set once the RID's subscription is known to be on a
  /// stamp-anchored topic; records then live in `by_id`, not `fifo`.
  const StampAnchor * anchor = nullptr;
  std::string topic;
  size_t stamp_sec_off = 0;   ///< byte offset of header.stamp.sec in the message
  size_t stamp_nsec_off = 0;  ///< byte offset of header.stamp.nanosec
  /// A TFMessage's `transforms`; the stamp offsets are then
  /// inside one element, and the anchor is their common stamp.
  const rosidl_typesupport_introspection_cpp::MessageMember * tf_seq = nullptr;
  bool warned_mixed = false;
  /// The subscription's type support, to serialize a record.
  const rosidl_message_type_support_t * type_support = nullptr;
  std::map<int64_t, Record> by_id;
  std::map<int64_t, Cell> cells;  ///< tag time -> cell, future tags only
  Counters counters;
  /// The subscription admitted to ingress. Expired once a lifecycle node
  /// dropped it, and replaced by the one its next late bind finds.
  std::weak_ptr<rclcpp::SubscriptionBase> admitted;
  bool warned_refused = false;
  bool lifecycle = false;  ///< entry of a lifecycle node, bound late
  std::deque<Record> fifo;
  bool replaying = false;  ///< inputs come from RCLLF_PLANT_REPLAY
  ReplayCounters replay;
};

std::mutex g_mu;
std::map<int, Slot> g_slots;
std::set<std::string> g_declared_topics;  // guarded by g_mu, leading slash
// Resolved topic (leading slash) -> its stamp anchor. Written at
// startup before any RID is admitted, read by pointer afterwards (a std::map
// never moves its values).
std::map<std::string, StampAnchor> g_stamp_anchors;  // guarded by g_mu

std::string with_slash(const std::string & topic)
{
  return !topic.empty() && topic[0] == '/' ? topic : "/" + topic;
}
// The record id is the int64_t payload of the LF action, end to end.
std::atomic<int64_t> g_next_id{0};

/// A record from a subscription that is no longer the RID's binding: one log
/// line, a JSON trace event, and no user callback.
void lifecycle_drop(int rid, int64_t id, const std::string & callback)
{
  RCLCPP_INFO(
    logger(), "rcllf external: RID %d (%s) dropped record %lld, taken from a subscription "
    "its lifecycle node has since destroyed", rid, callback.c_str(),
    static_cast<long long>(id));  // NOLINT(runtime/int)
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"external_lifecycle_drop\",\"rid\":%d,\"record_id\":%lld}", rid,
      static_cast<long long>(id));  // NOLINT(runtime/int)
  }
}

// ---------------------------------------------------------------------------
// The executable's scheduler, resolved with dlsym. The binding
// links its own single-threaded reactor-c (runtime.cpp, lf_request_stop), so
// every symbol here must come from the executable, whose copy owns the
// environment, its mutex and its event queue.
// ---------------------------------------------------------------------------

/// The executable's own definition of `name`, or nullptr. A definition in
/// any other object (the binding's own reactor-c above all) is never used.
template<typename Fn>
Fn executable_symbol(const char * name)
{
  const SymbolOrigin o = resolve_symbol(name);
  const void * exe = main_program_object();
  return exe != nullptr && o.object == exe ? reinterpret_cast<Fn>(o.address) : nullptr;
}

void * prod_env()
{
  using EnvsFn = int (*)(environment_t **);
  static EnvsFn fn = executable_symbol<EnvsFn>("_lf_get_environments");
  if (fn == nullptr) {
    return nullptr;
  }
  environment_t * envs = nullptr;
  return fn(&envs) > 0 ? envs : nullptr;
}

int64_t prod_start_ns() {return lf_start_tag().first;}

int prod_enter(void * env)
{
  using Fn = int (*)(environment_t *);
  static Fn fn = executable_symbol<Fn>("lf_critical_section_enter");
  return fn != nullptr ? fn(static_cast<environment_t *>(env)) : -1;
}

int prod_exit(void * env)
{
  using Fn = int (*)(environment_t *);
  static Fn fn = executable_symbol<Fn>("lf_critical_section_exit");
  return fn != nullptr ? fn(static_cast<environment_t *>(env)) : -1;
}

bool prod_after_stop(void * env, int64_t time)
{
  using Fn = bool (*)(environment_t *, tag_t);
  static Fn fn = executable_symbol<Fn>("lf_is_tag_after_stop_tag");
  tag_t tag;
  tag.time = time;
  tag.microstep = 0;
  return fn != nullptr && fn(static_cast<environment_t *>(env), tag);
}

void prod_current_tag(void * env, int64_t * time, uint32_t * microstep)
{
  const auto * e = static_cast<const environment_t *>(env);
  *time = e->current_tag.time;
  *microstep = e->current_tag.microstep;
}

/// Every executable symbol the stamp anchor calls, checked once at attach
/// so a missing one refuses the run instead of skipping the critical section
/// (enter/exit returned -1 unchecked) or exiting under the env mutex, and a
/// foreign one (resolved from another object) refuses it instead of locking
/// an environment the executable does not run.
std::string prod_symbol_problem()
{
  std::vector<std::string> origins;
  std::string problem = check_scheduler_symbols(&resolve_symbol, main_program_object(), &origins);
  static std::once_flag logged;
  std::call_once(
    logged, [&origins] {
      for (const std::string & line : origins) {
        RCLCPP_INFO(logger(), "rcllf external: scheduler symbol %s", line.c_str());
      }
    });
  return problem;
}

/// Under env->mutex. The bracket lf_schedule_token uses (lib/schedule.c),
/// with the collision rule in front of the insert:
/// an event of the same trigger at the same tag is removed and nothing is
/// inserted, so the admitted set of a cell is independent of arrival order.
int prod_insert(void * env, void * action, int64_t time, int64_t record)
{
  using FindFn = pqueue_tag_element_t * (*)(pqueue_tag_t *, pqueue_tag_element_t *);
  using RemoveFn = void (*)(pqueue_tag_t *, pqueue_tag_element_t *);
  using DoneFn = void (*)(lf_token_t *);
  using RecycleFn = void (*)(environment_t *, event_t *);
  using InitTokenFn = lf_token_t * (*)(token_template_t *, size_t);
  using ScheduleFn = int (*)(environment_t *, trigger_t *, tag_t, lf_token_t *);
  using NotifyFn = void (*)(environment_t *);
  static FindFn find = executable_symbol<FindFn>("pqueue_tag_find_equal_same_tag");
  static RemoveFn remove = executable_symbol<RemoveFn>("pqueue_tag_remove");
  static DoneFn done = executable_symbol<DoneFn>("_lf_done_using");
  static RecycleFn recycle = executable_symbol<RecycleFn>("lf_recycle_event");
  static InitTokenFn init_token = executable_symbol<InitTokenFn>("_lf_initialize_token");
  static ScheduleFn schedule = executable_symbol<ScheduleFn>("_lf_schedule_at_tag");
  static NotifyFn notify = executable_symbol<NotifyFn>("lf_notify_of_event");
  if (find == nullptr || remove == nullptr || done == nullptr || recycle == nullptr ||
    init_token == nullptr || schedule == nullptr || notify == nullptr)
  {
    RCLCPP_FATAL(
      logger(), "rcllf external: the executable does not export reactor-c's scheduling "
      "symbols (_lf_schedule_at_tag and friends); a stamp anchor cannot insert events. "
      "Refusing to run instead of dropping messages silently.");
    std::_Exit(EXIT_FAILURE);  // under env->mutex: no destructors, no unlock
  }
  auto * e = static_cast<environment_t *>(env);
  auto * a = static_cast<lf_action_base_t *>(action);
  tag_t tag;
  tag.time = time;
  tag.microstep = 0;
  // _lf_schedule_at_tag refuses a tag at or before the current one only once
  // execution started; test it here too so the decision never depends on that.
  // Microstep 0 at the current tag's time is at or before the current tag.
  if (time <= e->current_tag.time) {
    return INSERT_PAST;
  }
  event_t probe{};
  probe.base.tag = tag;
  probe.trigger = a->trigger;
  if (pqueue_tag_element_t * found = find(e->event_q, &probe.base)) {
    remove(e->event_q, found);
    auto * ev = reinterpret_cast<event_t *>(found);
    if (ev->token != nullptr) {
      done(ev->token);  // the queue's reference, taken by _lf_schedule_at_tag
    }
    recycle(e, ev);
    return INSERT_COLLIDED;
  }
  lf_token_t * token = init_token(&a->tmplt, 1);
  std::memcpy(token->value, &record, sizeof(record));
  if (schedule(e, a->trigger, tag, token) <= 0) {
    // A future tag refused by reactor-c itself (stopping, no event left), not
    // an overrun: the caller reports it as refused.
    return INSERT_REFUSED;  // _lf_schedule_at_tag released the token
  }
  notify(e);
  return INSERT_OK;
}

void prod_set_stop_tag(void * env, int64_t time, uint32_t microstep)
{
  using Fn = void (*)(environment_t *, tag_t);
  static Fn fn = executable_symbol<Fn>("lf_set_stop_tag");
  if (fn == nullptr) {
    return;  // the caller reads the stop tag back and refuses the replay
  }
  tag_t tag;
  tag.time = time;
  tag.microstep = microstep;
  fn(static_cast<environment_t *>(env), tag);
}

void prod_stop_tag(void * env, int64_t * time, uint32_t * microstep)
{
  const auto * e = static_cast<const environment_t *>(env);
  *time = e->stop_tag.time;
  *microstep = e->stop_tag.microstep;
}

bool prod_keepalive()
{
  static const bool * flag = executable_symbol<const bool *>("keepalive_specified");
  return flag != nullptr && *flag;
}

const SchedulerOps g_prod_ops{
  &prod_env, &prod_start_ns, &prod_enter, &prod_exit, &prod_after_stop, &prod_current_tag,
  &prod_insert, &prod_symbol_problem, &prod_set_stop_tag, &prod_stop_tag, &prod_keepalive};
std::atomic<const SchedulerOps *> g_ops{&g_prod_ops};

const char * disposition_name(Disposition d)
{
  switch (d) {
    case Disposition::ARRIVAL: return "arrival";
    case Disposition::ADMITTED: return "admitted";
    case Disposition::LATE: return "late";
    case Disposition::OVERRUN: return "overrun";
    case Disposition::COLLISION: return "collision";
    case Disposition::AFTER_STOP: return "after_stop";
    case Disposition::REFUSED: return "refused";
    case Disposition::TF_MIXED_STAMPS: return "tf_mixed_stamps";
  }
  return "?";
}

/// ceil(rel / grid) * grid for any sign of rel (grid >= 1). False when the
/// result does not fit int64.
bool ceil_to_grid(int64_t rel, int64_t grid, int64_t * out)
{
  int64_t q = rel / grid;
  if (rel % grid > 0) {
    ++q;
  }
  return !__builtin_mul_overflow(q, grid, out);
}

/// tag = start + ceil(stamp + delay + reserve - start, grid), every step
/// overflow-checked. The declared constants are bounded (kAnchorMaxNs), so
/// this fails only for a clock far outside what the bound
/// assumes.
bool anchor_tag(int64_t start, int64_t stamp, const StampAnchor & a, int64_t * tag_time)
{
  int64_t rel = 0;
  int64_t rounded = 0;
  return !__builtin_add_overflow(stamp, a.delay_ns, &rel) &&
         !__builtin_add_overflow(rel, a.reserve_ns, &rel) &&
         !__builtin_sub_overflow(rel, start, &rel) &&
         ceil_to_grid(rel, a.grid_ns, &rounded) &&
         !__builtin_add_overflow(start, rounded, tag_time);
}

/// Byte offsets of header.stamp.sec and header.stamp.nanosec in a message
/// of `sub`'s type, through C++ introspection. False when the type has no
/// `header` of type std_msgs/Header (or no introspection support).
/// `tf_seq` is set when the type is tf2_msgs/msg/TFMessage.
bool locate_header_stamp(
  const rclcpp::SubscriptionBase & sub, size_t * sec_off, size_t * nsec_off,
  const rosidl_typesupport_introspection_cpp::MessageMember ** tf_seq, std::string * why)
{
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    &sub.get_message_type_support_handle(),
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts == nullptr || ts->data == nullptr) {
    rcutils_reset_error();
    *why = "no C++ introspection type support";
    return false;
  }
  const auto * top =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  if (locate_header_stamp_in(top, sec_off, nsec_off, why)) {
    return true;
  }
  std::string tf_why;
  if (locate_tf_stamps_in(top, tf_seq, sec_off, nsec_off, &tf_why)) {
    return true;
  }
  if (tf_why.rfind("not a ", 0) != 0) {
    *why = tf_why;  // a TFMessage whose elements are not what tf2 sends
  }
  return false;
}

/// `pkg/msg/Type` of `sub`'s messages from C++ introspection ("?" without).
std::string message_type_name(const rclcpp::SubscriptionBase & sub)
{
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    &sub.get_message_type_support_handle(),
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts == nullptr || ts->data == nullptr) {
    rcutils_reset_error();
    return "?";
  }
  const auto * m =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  std::string ns = m->message_namespace_ != nullptr ? m->message_namespace_ : "?";
  for (size_t pos = ns.find("::"); pos != std::string::npos; pos = ns.find("::", pos + 1)) {
    ns.replace(pos, 2, "/");
  }
  return ns + "/" + (m->message_name_ != nullptr ? m->message_name_ : "?");
}

/// `message` serialized with `ts` (rmw_serialize), empty on failure.
std::vector<uint8_t> serialize_record(
  const void * message, const rosidl_message_type_support_t * ts)
{
  std::vector<uint8_t> out;
  if (message == nullptr || ts == nullptr) {
    return out;
  }
  rmw_serialized_message_t buf = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  if (rmw_serialized_message_init(&buf, 0, &alloc) != RMW_RET_OK) {
    rcutils_reset_error();
    return out;
  }
  if (rmw_serialize(message, ts, &buf) == RMW_RET_OK) {
    out.assign(buf.buffer, buf.buffer + buf.buffer_length);
  } else {
    rcutils_reset_error();
  }
  (void)rmw_serialized_message_fini(&buf);
  return out;
}

/// The subscription of `rid` is known and its topic has a stamp anchor:
/// refuse fast mode, a message type without header.stamp, and a physical
/// action with a min delay or min spacing; else attach the anchor to the slot.
/// Idempotent per RID. Returns false when the slot already carries it.
void attach_stamp_anchor(int rid, const rclcpp::SubscriptionBase & sub)
{
  const StampAnchor * anchor = nullptr;
  std::string topic;
  void * action = nullptr;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    if (s.anchor != nullptr) {
      return;
    }
    topic = with_slash(sub.get_topic_name());
    auto it = g_stamp_anchors.find(topic);
    if (it == g_stamp_anchors.end()) {
      return;
    }
    anchor = &it->second;
    action = s.action;
  }
  if (lf_fast() && !plant_replay::enabled()) {
    // A replay inserts recorded tags at the start tag and runs fast.
    // Fast mode advances logical time without waiting for physical time, so
    // a tag computed from header.stamp is in the past when the message
    // arrives and every message would be refused. Same code path as the
    // generator's --fast refusal; this catches a launch-time -f true.
    RCLCPP_FATAL(
      logger(), "rcllf external: RID %d topic '%s' is anchored at header.stamp "
      "(anchor: stamp), which fast mode cannot honour: logical time runs ahead of the "
      "stamps and every message would be refused. Run without fast mode, or anchor the "
      "topic at arrival. Exiting.", rid, topic.c_str());
    if (trace_callbacks_enabled()) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding.trace"),
        "{\"event\":\"external_fast_refused\",\"rid\":%d,\"topic\":\"%s\"}", rid, topic.c_str());
    }
    // _Exit, as bind_check: this runs inside the startup reaction on an LF
    // worker; exit() would run static destructors under the other threads.
    std::_Exit(3);
  }
  size_t sec_off = 0;
  size_t nsec_off = 0;
  const rosidl_typesupport_introspection_cpp::MessageMember * tf_seq = nullptr;
  std::string why;
  if (!locate_header_stamp(sub, &sec_off, &nsec_off, &tf_seq, &why)) {
    RCLCPP_FATAL(
      logger(), "rcllf external: RID %d topic '%s' is anchored at header.stamp, but its "
      "message type %s: %s. Refusing to run instead of tagging its messages at a time "
      "they do not carry.", rid, topic.c_str(),
      sub.get_message_type_support_handle().typesupport_identifier, why.c_str());
    std::_Exit(EXIT_FAILURE);
  }
  if (action != nullptr) {
    // _lf_schedule_at_tag reads neither offset nor period, so a declaration
    // with a min delay or min spacing would be ignored silently.
    const trigger_t * trigger = static_cast<lf_action_base_t *>(action)->trigger;
    if (trigger == nullptr || trigger->offset != 0 || trigger->period >= 0) {
      RCLCPP_FATAL(
        logger(), "rcllf external: RID %d topic '%s': the generated physical action has "
        "min delay %lld and min spacing %lld; a stamp anchor needs a plain physical action "
        "(offset 0, no spacing). Refusing to run.", rid, topic.c_str(),
        trigger == nullptr ? -1LL : static_cast<long long>(trigger->offset),  // NOLINT(runtime/int)
        trigger == nullptr ? -1LL :
        static_cast<long long>(trigger->period));  // NOLINT(runtime/int)
      std::_Exit(EXIT_FAILURE);
    }
  }
  const std::string symbol_problem = g_ops.load()->symbol_problem();
  if (!symbol_problem.empty()) {
    RCLCPP_FATAL(
      logger(), "rcllf external: RID %d topic '%s' is anchored at header.stamp, but %s; a "
      "stamp anchor cannot decide admission under the scheduler's lock without the "
      "executable's own copy. Refusing to run instead of dropping messages silently.",
      rid, topic.c_str(), symbol_problem.c_str());
    std::_Exit(EXIT_FAILURE);
  }
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    s.anchor = anchor;
    s.topic = topic;
    s.stamp_sec_off = sec_off;
    s.stamp_nsec_off = nsec_off;
    s.tf_seq = tf_seq;
    s.type_support = &sub.get_message_type_support_handle();
  }
  const int64_t start = g_ops.load()->start_ns();
  if (plant_record::enabled()) {
    // The topic's record file starts with its header (type name
    // for the reader's deserializer, lf_time_start to read tags as elapsed).
    const char * rmw = rmw_get_implementation_identifier();
    plant_record::open_topic(
      topic, message_type_name(sub), rmw != nullptr ? rmw : "?", start, anchor->delay_ns,
      anchor->reserve_ns, anchor->grid_ns);
    // Serialize one default message here, at attach, so the rmw type-support
    // lookup is not paid by the first record on the ingress thread (measured
    // 61 us cold against 2 us warm).
    // create_message() is not const in rclcpp; it allocates, it changes nothing.
    const std::shared_ptr<void> warm =
      const_cast<rclcpp::SubscriptionBase &>(sub).create_message();
    (void)serialize_record(warm.get(), &sub.get_message_type_support_handle());
  }
  if (tf_seq != nullptr) {
    RCLCPP_INFO(
      logger(), "rcllf external: RID %d topic '%s' is a tf2_msgs/TFMessage: anchored at the "
      "one header.stamp all its transforms carry; a message with differing stamps or none "
      "is not admitted (tf_mixed_stamps)", rid, topic.c_str());
  }
  RCLCPP_INFO(
    logger(), "rcllf external: RID %d topic '%s' anchored at header.stamp: delay_ns=%lld "
    "ingress_reserve_ns=%lld grid_ns=%lld start_ns=%lld", rid, topic.c_str(),
    static_cast<long long>(anchor->delay_ns),  // NOLINT(runtime/int)
    static_cast<long long>(anchor->reserve_ns),  // NOLINT(runtime/int)
    static_cast<long long>(anchor->grid_ns),  // NOLINT(runtime/int)
    static_cast<long long>(start));  // NOLINT(runtime/int)
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"external_stamp_anchor\",\"rid\":%d,\"topic\":\"%s\",\"delay_ns\":%lld,"
      "\"ingress_reserve_ns\":%lld,\"grid_ns\":%lld,\"start_ns\":%lld}", rid, topic.c_str(),
      static_cast<long long>(anchor->delay_ns),  // NOLINT(runtime/int)
      static_cast<long long>(anchor->reserve_ns),  // NOLINT(runtime/int)
      static_cast<long long>(anchor->grid_ns),  // NOLINT(runtime/int)
      static_cast<long long>(start));  // NOLINT(runtime/int)
  }
}

/// One external_arrival in both trace sinks.
void emit_arrival(
  int rid, const Record & r, int64_t arrival, int64_t stamp, int64_t tag_ns, int64_t insert,
  Disposition d)
{
  {
    rcllf_tp_external_arrival ev{};
    ev.rid = rid;
    std::memcpy(ev.gid, r.gid.data(), RCLLF_TRACE_GID_LEN);
    ev.ordinal = r.ordinal;
    ev.record_id = static_cast<uint64_t>(r.id);
    ev.arrival_physical_ns = arrival;
    ev.stamp_ns = stamp;
    ev.tag_ns = tag_ns;
    ev.insert_ns = insert;
    ev.disposition = static_cast<uint8_t>(d);
    semtrace::external_arrival(ev);
  }
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"external_arrival\",\"rid\":%d,\"record_id\":%lld,\"ordinal\":%llu,"
      "\"arrival_physical_ns\":%lld,\"stamp_ns\":%lld,\"tag_ns\":%lld,\"insert_ns\":%lld,"
      "\"disposition\":\"%s\"}",
      rid, static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(r.ordinal),  // NOLINT(runtime/int)
      static_cast<long long>(arrival),  // NOLINT(runtime/int)
      static_cast<long long>(stamp),  // NOLINT(runtime/int)
      static_cast<long long>(tag_ns),  // NOLINT(runtime/int)
      static_cast<long long>(insert),  // NOLINT(runtime/int)
      disposition_name(d));
  }
}

/// Plan 5.3 items 1 and 2: one message of a stamp-anchored RID.
/// `arrival` was read before any of this. Two decisions in order: late on the
/// ingress thread against T, then admission under the scheduler's mutex.
void on_stamped_sample(int rid, Record r, int64_t arrival)
{
  const StampAnchor * anchor = nullptr;
  void * action = nullptr;
  size_t sec_off = 0;
  size_t nsec_off = 0;
  const rosidl_typesupport_introspection_cpp::MessageMember * tf_seq = nullptr;
  const rosidl_message_type_support_t * type_support = nullptr;
  std::string topic;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    anchor = s.anchor;
    action = s.action;
    sec_off = s.stamp_sec_off;
    nsec_off = s.stamp_nsec_off;
    tf_seq = s.tf_seq;
    type_support = s.type_support;
    topic = s.topic;
  }
  // Serialized here, before the record is registered or
  // inserted, because once it is a worker may hand the message to a callback
  // that takes it by non-const shared_ptr. The cost is the recording's only
  // addition to the arrival-to-insert path and is recorded per record; the
  // file I/O happens on the recorder's own thread.
  const bool recording = plant_record::enabled();
  std::vector<uint8_t> rec_bytes;
  int64_t rec_serialize_ns = -1;
  if (recording) {
    const auto t0 = std::chrono::steady_clock::now();
    rec_bytes = serialize_record(r.message.get(), type_support);
    rec_serialize_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - t0).count();
  }
  std::optional<int64_t> rec_insert;  // set once the critical section was left
  auto record = [&](Disposition d, int64_t tag_ns) {
      if (!recording) {
        return;
      }
      plant_record::External e;
      e.rid = rid;
      e.id = r.id;
      e.stamp_ns = r.stamp;
      e.arrival_ns = arrival;
      e.insert_ns = rec_insert;
      if (tag_ns >= 0) {
        e.tag_ns = tag_ns;
      }
      e.disposition = disposition_name(d);
      e.serialize_ns = rec_serialize_ns;
      e.bytes = std::move(rec_bytes);
      plant_record::external(topic, std::move(e));
    };
  int64_t stamp = 0;
  bool no_common_stamp = false;
  if (tf_seq != nullptr) {
    no_common_stamp = common_tf_stamp(tf_seq, r.message.get(), sec_off, nsec_off, &stamp) != 1;
  } else {
    const auto * bytes = static_cast<const uint8_t *>(r.message.get());
    int32_t sec = 0;
    uint32_t nanosec = 0;
    std::memcpy(&sec, bytes + sec_off, sizeof(sec));
    std::memcpy(&nanosec, bytes + nsec_off, sizeof(nanosec));
    stamp = static_cast<int64_t>(sec) * 1000000000LL + nanosec;
  }
  r.arrival = arrival;
  r.stamp = stamp;

  auto refuse = [&](Disposition d, int64_t tag_ns) {
      {
        std::lock_guard<std::mutex> lk(g_mu);
        Counters & c = g_slots[rid].counters;
        switch (d) {
          case Disposition::LATE: ++c.late; break;
          case Disposition::OVERRUN: ++c.overrun; break;
          case Disposition::COLLISION: ++c.collision; break;
          case Disposition::AFTER_STOP: ++c.after_stop; break;
          case Disposition::TF_MIXED_STAMPS: ++c.tf_mixed_stamps; break;
          default: ++c.refused; break;
        }
      }
      emit_arrival(rid, r, arrival, stamp, tag_ns, 0, d);
      record(d, tag_ns);
    };

  if (no_common_stamp) {
    // Plan S2: no single stamp to anchor at. Ended unadmitted and
    // traced; never split into per-stamp messages, never anchored at a guess.
    bool warn = false;
    {
      std::lock_guard<std::mutex> lk(g_mu);
      warn = !g_slots[rid].warned_mixed;
      g_slots[rid].warned_mixed = true;
    }
    refuse(Disposition::TF_MIXED_STAMPS, -1);
    if (warn) {
      RCLCPP_WARN(
        logger(), "rcllf external: RID %d topic '%s': record %lld not admitted "
        "(tf_mixed_stamps): its transforms do not all carry one header.stamp, so it has no "
        "stamp to anchor at; publish one stamp per TFMessage. Further ones on this RID are "
        "counted in the summary, not logged.", rid, topic.c_str(),
        static_cast<long long>(r.id));  // NOLINT(runtime/int)
    }
    return;
  }

  // First decision: the owner's T test, independent of grid, reserve and of
  // whether the tag would still be schedulable.
  int64_t deadline = 0;
  if (__builtin_add_overflow(stamp, anchor->delay_ns, &deadline)) {
    refuse(Disposition::REFUSED, -1);
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: record %lld refused: header.stamp %lld ns plus "
      "delay_ns does not fit a 64-bit time.", rid,
      static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<long long>(stamp));  // NOLINT(runtime/int)
    return;
  }
  if (arrival > deadline) {
    refuse(Disposition::LATE, -1);
    return;
  }
  const SchedulerOps * ops = g_ops.load();
  void * env = ops->env();
  if (env == nullptr) {
    refuse(Disposition::REFUSED, -1);
    return;
  }
  const int64_t start = ops->start_ns();
  int64_t tag_time = 0;
  if (!anchor_tag(start, stamp, *anchor, &tag_time)) {
    refuse(Disposition::REFUSED, -1);
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: record %lld refused: its tag from header.stamp "
      "%lld ns and start %lld ns does not fit a 64-bit time.", rid,
      static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<long long>(stamp),  // NOLINT(runtime/int)
      static_cast<long long>(start));  // NOLINT(runtime/int)
    return;
  }
  const int64_t tag_ns = tag_time - start;

  // A cell that collided already (its event removed) is refused without
  // touching the queue, so the cell does not look empty again after the
  // removal. Whether that marker is stale (the scheduler passed the cell,
  // an overrun) is decided under the lock below, like every other refusal.
  bool cell_collided = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    auto it = s.cells.find(tag_time);
    cell_collided = it != s.cells.end() && it->second.collided;
    if (!cell_collided) {
      // Registered before its event exists: once the insert is done and the
      // lock released, a worker may run the event before this thread takes
      // g_mu again, and dispatch finds the record by id. Taken back below
      // unless the insert succeeded.
      s.by_id[r.id] = r;
    }
  }

  // Second decision, under the mutex the runtime holds to advance the tag:
  // after the stop tag, collision with the queued event of this cell, past
  // tag (overrun), or inserted. No binding lock is held here (R16).
  int64_t current = 0;
  uint32_t current_microstep = 0;
  int result = INSERT_PAST;
  bool after_stop = false;
  if (ops->enter(env) != 0) {
    {
      std::lock_guard<std::mutex> lk(g_mu);
      g_slots[rid].by_id.erase(r.id);
    }
    refuse(Disposition::REFUSED, tag_ns);
    RCLCPP_ERROR(
      logger(), "rcllf external: RID %d: record %lld refused: the scheduler's critical "
      "section could not be entered, so its admission cannot be decided atomically.",
      rid, static_cast<long long>(r.id));  // NOLINT(runtime/int)
    return;
  }
  after_stop = ops->after_stop(env, tag_time);
  ops->current_tag(env, &current, &current_microstep);
  if (!after_stop && !cell_collided) {
    result = ops->insert(env, action, tag_time, r.id);
  }
  if (ops->exit(env) != 0) {
    RCLCPP_FATAL(
      logger(), "rcllf external: RID %d: the scheduler's critical section could not be "
      "left; the runtime would wait on it forever. Exiting.", rid);
    std::_Exit(EXIT_FAILURE);
  }
  const int64_t insert_ns = lf_time_physical();
  rec_insert = insert_ns;
  // The marker was read before the lock; under it the cell may have passed.
  const bool stale_marker = cell_collided && tag_time <= current;

  int64_t collided_record = 0;
  bool have_collided = false;
  Record collided;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    // Cells at or before the current tag can only be overruns from now on.
    for (auto it = s.cells.begin(); it != s.cells.end() && it->first <= current; ) {
      it = s.cells.erase(it);
    }
    if (cell_collided) {
      // nothing was registered or inserted for this record
    } else if (!after_stop && result == INSERT_OK) {
      s.cells[tag_time] = Cell{r.id, false};
      ++s.counters.admitted;  // by_id holds the record, unless a worker ran it already
    } else {
      s.by_id.erase(r.id);  // not inserted: the registration above goes
      if (!after_stop && result == INSERT_COLLIDED) {
        auto it = s.cells.find(tag_time);
        if (it != s.cells.end()) {
          collided_record = it->second.record;
          auto rec = s.by_id.find(collided_record);
          if (rec != s.by_id.end()) {
            collided = std::move(rec->second);
            have_collided = true;
            s.by_id.erase(rec);
            if (s.counters.admitted > 0) {
              --s.counters.admitted;  // its event left the queue: not admitted after all
            }
            ++s.counters.collision;
          }
          it->second.collided = true;
        } else {
          s.cells[tag_time] = Cell{0, true};
        }
      }
    }
  }
  if (after_stop) {
    refuse(Disposition::AFTER_STOP, tag_ns);
    return;
  }
  if (cell_collided && !stale_marker) {
    refuse(Disposition::COLLISION, tag_ns);  // the cell's WARN was given at its collision
    return;
  }
  if (result == INSERT_COLLIDED) {
    if (have_collided) {
      emit_arrival(
        rid, collided, collided.arrival, collided.stamp, tag_ns, 0, Disposition::COLLISION);
      // The earlier record was recorded admitted at its insert;
      // this correction is its last entry, so every reader sees collision.
      plant_record::correction(topic, rid, collided.id, disposition_name(Disposition::COLLISION));
    }
    refuse(Disposition::COLLISION, tag_ns);
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: records %lld and %lld are on time but share grid "
      "cell %lld ns; both refused (collision). grid_ns is above the topic's period "
      "minus twice its stamp jitter, or the publisher sent two stamps in one cell.", rid,
      static_cast<long long>(collided_record),  // NOLINT(runtime/int)
      static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<long long>(tag_ns));  // NOLINT(runtime/int)
    return;
  }
  if (!stale_marker && result == INSERT_REFUSED) {
    refuse(Disposition::REFUSED, tag_ns);
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: record %lld refused: reactor-c would not schedule "
      "its future tag %lld ns (stopping, or no event available).", rid,
      static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<long long>(tag_ns));  // NOLINT(runtime/int)
    return;
  }
  if (stale_marker || result != INSERT_OK) {
    refuse(Disposition::OVERRUN, tag_ns);
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: record %lld arrived on time but its tag %lld ns is "
      "at or before the current tag %lld ns (overrun): the ingress path took longer "
      "than ingress_reserve_ns. The reserve's basis is falsified for this run.", rid,
      static_cast<long long>(r.id),  // NOLINT(runtime/int)
      static_cast<long long>(tag_ns),  // NOLINT(runtime/int)
      static_cast<long long>(current - start));  // NOLINT(runtime/int)
    return;
  }
  emit_arrival(rid, r, arrival, stamp, tag_ns, insert_ns, Disposition::ADMITTED);
  record(Disposition::ADMITTED, tag_ns);
}

// ---------------------------------------------------------------------------
// Plan 5.3 item 5: admitted-only tag replay (RCLLF_PLANT_REPLAY).
// ---------------------------------------------------------------------------

std::string json_escape(const std::string & in)
{
  std::string out;
  for (char c : in) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

/// A replay that cannot reproduce the live run's admitted inputs and stop tag
/// is refused before it runs (exit 3, like the fast-mode refusal): a replay
/// with a different input set is a failed replay, never a result.
[[noreturn]] void replay_refuse(const std::string & why)
{
  RCLCPP_FATAL(
    logger(), "rcllf replay: FAILED: %s. Refusing to replay instead of running with "
    "an input set or stop tag other than the live run's. Exiting.", why.c_str());
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"), "{\"event\":\"replay_failed\",\"why\":\"%s\"}",
      json_escape(why).c_str());
  }
  // _Exit, as the fast-mode refusal: this runs in a startup reaction on an LF
  // worker, possibly holding the env mutex; exit() would run destructors.
  std::_Exit(3);
}

struct ReplayRun
{
  bool started = false;
  int64_t start = 0;
  int64_t stop_time = 0;   ///< absolute
  plant_replay::Stop stop;  ///< elapsed, as recorded
  std::map<std::string, plant_replay::TopicFile> files;  ///< path -> file
  std::map<std::string, std::set<int>> recorded_rids;    ///< path -> RIDs in it
  std::map<std::string, std::set<int>> replayed_rids;    ///< path -> RIDs replayed
  std::set<std::string> matched;  ///< paths a subscription of the program replayed
};
std::mutex g_replay_mu;  // lock order: g_replay_mu, then g_mu
ReplayRun g_replay;      // guarded by g_replay_mu
std::atomic<bool> g_replay_failed{false};  // replay_log_summary found a failure

/// Under g_replay_mu, once per run, at the start tag: read the recording, and
/// lower the stop tag to the live run's recorded lf_stop tag.
void replay_start_locked()
{
  if (g_replay.started) {
    return;
  }
  const std::string d = plant_replay::dir();
  std::string why;
  if (!plant_replay::load_stop(d + "/run.jsonl", &g_replay.stop, &why)) {
    replay_refuse(why);
  }
  const SchedulerOps * ops = g_ops.load();
  if (ops->set_stop_tag == nullptr || ops->stop_tag == nullptr || ops->keepalive == nullptr) {
    replay_refuse("the scheduler offers no stop-tag access (lf_set_stop_tag, keepalive)");
  }
  if (!ops->keepalive()) {
    // Without keepalive an empty event queue ends the run at the current tag
    // plus one microstep (reactor_threaded.c:305-311), before the live stop.
    replay_refuse(
      "the program runs without keepalive, so it would end one microstep after its last "
      "replayed input instead of at the live run's stop tag");
  }
  void * env = ops->env();
  if (env == nullptr) {
    replay_refuse("no LF environment");
  }
  const int64_t start = ops->start_ns();
  int64_t stop_time = 0;
  if (g_replay.stop.tag_ns < 0 ||
    __builtin_add_overflow(start, g_replay.stop.tag_ns, &stop_time))
  {
    replay_refuse("the recorded lf_stop tag " + std::to_string(g_replay.stop.tag_ns) +
      " ns is negative or does not fit");
  }
  if (ops->enter(env) != 0) {
    replay_refuse("lf_critical_section_enter failed");
  }
  int64_t cur = 0;
  uint32_t cur_m = 0;
  ops->current_tag(env, &cur, &cur_m);
  const bool at_start = cur == start && cur_m == 0;
  if (at_start) {
    ops->set_stop_tag(env, stop_time, g_replay.stop.microstep);
  }
  int64_t got = 0;
  uint32_t got_m = 0;
  ops->stop_tag(env, &got, &got_m);
  ops->exit(env);
  if (!at_start) {
    replay_refuse(
      "the replay reached the scheduler at elapsed tag (" + std::to_string(cur - start) + ", " +
      std::to_string(cur_m) + "), not at the start tag; records and stop tag are set at the "
      "start tag only");
  }
  if (got != stop_time || got_m != g_replay.stop.microstep) {
    replay_refuse(
      "the program's stop tag is elapsed (" + std::to_string(got - start) + ", " +
      std::to_string(got_m) + "), not the live run's (" +
      std::to_string(g_replay.stop.tag_ns) + ", " + std::to_string(g_replay.stop.microstep) +
      "); lf_set_stop_tag only lowers it, so a replay with a shorter timeout cannot reach it");
  }
  for (const std::string & path : plant_replay::topic_files(d)) {
    plant_replay::TopicFile tf;
    if (!plant_replay::load_topic(path, &tf, &why)) {
      replay_refuse(why);
    }
    if (tf.start_ns != g_replay.stop.start_ns) {
      replay_refuse(
        path + " was recorded by a run that started at " + std::to_string(tf.start_ns) +
        ", run.jsonl by one that started at " + std::to_string(g_replay.stop.start_ns));
    }
    for (const auto & r : tf.records) {
      g_replay.recorded_rids[path].insert(r.rid);
    }
    g_replay.files.emplace(path, std::move(tf));
  }
  g_replay.start = start;
  g_replay.stop_time = stop_time;
  g_replay.started = true;
  RCLCPP_INFO(
    logger(), "rcllf replay: from '%s': %zu topic file(s); stop at the live run's elapsed tag "
    "(%lld, %u), condition %s", d.c_str(), g_replay.files.size(),
    static_cast<long long>(g_replay.stop.tag_ns),  // NOLINT(runtime/int)
    g_replay.stop.microstep, g_replay.stop.condition.c_str());
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"replay_start\",\"start_ns\":%lld,\"stop_tag_ns\":%lld,\"stop_microstep\":%u,"
      "\"condition\":\"%s\",\"topic_files\":%zu}",
      static_cast<long long>(start),  // NOLINT(runtime/int)
      static_cast<long long>(g_replay.stop.tag_ns),  // NOLINT(runtime/int)
      g_replay.stop.microstep, json_escape(g_replay.stop.condition).c_str(),
      g_replay.files.size());
  }
}

/// Replay mode instead of admission to ingress: insert every admitted record
/// of `rid` at (start + its live elapsed tag, 0) on the RID's own physical
/// action, through the live insert, so the live reaction and record lookup
/// run it. Called at the start tag, from the startup reaction that registers
/// the action. Once per RID.
void replay_rid(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  const StampAnchor * anchor = nullptr;
  void * action = nullptr;
  std::string topic;
  size_t sec_off = 0;
  size_t nsec_off = 0;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    if (s.replaying) {
      return;
    }
    s.replaying = true;
    anchor = s.anchor;
    action = s.action;
    topic = s.anchor != nullptr ? s.topic : with_slash(sub->get_topic_name());
    sec_off = s.stamp_sec_off;
    nsec_off = s.stamp_nsec_off;
  }
  if (anchor == nullptr) {
    replay_refuse(
      "RID " + std::to_string(rid) + " topic '" + topic + "' is external but anchored at "
      "arrival; only stamp-anchored topics are recorded, so its inputs cannot be replayed");
  }
  std::lock_guard<std::mutex> rl(g_replay_mu);
  replay_start_locked();
  const std::string path = plant_replay::dir() + "/" + plant_record::topic_file(topic);
  auto fit = g_replay.files.find(path);
  if (fit == g_replay.files.end()) {
    replay_refuse("RID " + std::to_string(rid) + " topic '" + topic + "': no recording " + path);
  }
  g_replay.matched.insert(path);
  const plant_replay::TopicFile & tf = fit->second;
  const std::string type = message_type_name(*sub);
  if (tf.topic != topic || tf.type != type) {
    replay_refuse(
      path + " records topic '" + tf.topic + "' of type " + tf.type + ", the program "
      "subscribes '" + topic + "' of type " + type);
  }
  if (tf.delay_ns != anchor->delay_ns || tf.reserve_ns != anchor->reserve_ns ||
    tf.grid_ns != anchor->grid_ns)
  {
    replay_refuse(
      path + " was recorded with delay_ns/ingress_reserve_ns/grid_ns " +
      std::to_string(tf.delay_ns) + "/" + std::to_string(tf.reserve_ns) + "/" +
      std::to_string(tf.grid_ns) + ", the program declares " + std::to_string(anchor->delay_ns) +
      "/" + std::to_string(anchor->reserve_ns) + "/" + std::to_string(anchor->grid_ns));
  }
  const SchedulerOps * ops = g_ops.load();
  void * env = ops->env();
  std::map<std::string, uint64_t> skipped;  // disposition -> count
  ReplayCounters rc;
  for (const plant_replay::Record & rec : tf.records) {
    if (rec.rid != rid) {
      continue;
    }
    if (rec.disposition != "admitted") {
      ++skipped[rec.disposition];
      ++rc.not_admitted;
      continue;
    }
    int64_t tag_time = 0;
    if (*rec.tag_ns <= 0 || __builtin_add_overflow(g_replay.start, *rec.tag_ns, &tag_time)) {
      replay_refuse(
        "record " + std::to_string(rec.id) + ": elapsed tag " + std::to_string(*rec.tag_ns) +
        " ns is not after the start tag");
    }
    if (tag_time > g_replay.stop_time) {
      // Admitted live, but a request_stop lowered the stop tag below it, so
      // its reaction never ran live either.
      ++rc.past_stop;
      continue;
    }
    std::shared_ptr<void> msg = sub->create_message();
    rmw_serialized_message_t view = rmw_get_zero_initialized_serialized_message();
    view.buffer = const_cast<uint8_t *>(rec.bytes.data());
    view.buffer_length = rec.bytes.size();
    view.buffer_capacity = rec.bytes.size();
    view.allocator = rcutils_get_default_allocator();
    if (rmw_deserialize(&view, &sub->get_message_type_support_handle(), msg.get()) !=
      RMW_RET_OK)
    {
      rcutils_reset_error();
      replay_refuse("record " + std::to_string(rec.id) + ": its bytes do not deserialize as " +
        type);
    }
    int32_t sec = 0;
    uint32_t nanosec = 0;
    std::memcpy(&sec, static_cast<const uint8_t *>(msg.get()) + sec_off, sizeof(sec));
    std::memcpy(&nanosec, static_cast<const uint8_t *>(msg.get()) + nsec_off, sizeof(nanosec));
    const int64_t stamp = static_cast<int64_t>(sec) * 1000000000LL + nanosec;
    if (stamp != rec.stamp_ns) {
      replay_refuse(
        "record " + std::to_string(rec.id) + ": its bytes carry header.stamp " +
        std::to_string(stamp) + ", the entry says " + std::to_string(rec.stamp_ns));
    }
    Record r;
    r.id = g_next_id.fetch_add(1) + 1;
    r.message = msg;
    rmw_message_info_t mi{};
    mi.publication_sequence_number = static_cast<uint64_t>(rec.id);  // the live record id
    r.info = std::make_shared<rclcpp::MessageInfo>(mi);
    r.source = sub;
    r.arrival = rec.arrival_ns;
    r.stamp = stamp;
    {
      std::lock_guard<std::mutex> lk(g_mu);
      g_slots[rid].by_id[r.id] = r;
    }
    if (ops->enter(env) != 0) {
      replay_refuse("lf_critical_section_enter failed");
    }
    const int res = ops->insert(env, action, tag_time, r.id);
    ops->exit(env);
    if (res != INSERT_OK) {
      replay_refuse(
        "record " + std::to_string(rec.id) + " at elapsed tag " + std::to_string(*rec.tag_ns) +
        " ns: the insert returned " +
        (res == INSERT_COLLIDED ? std::string("collided (two records in one cell)") :
        res == INSERT_PAST ? std::string("past") : std::string("refused")));
    }
    ++rc.inserted;
    if (plant_record::enabled()) {
      plant_record::External e;
      e.rid = rid;
      e.id = r.id;
      e.stamp_ns = stamp;
      e.arrival_ns = rec.arrival_ns;
      e.tag_ns = *rec.tag_ns;
      e.disposition = "admitted";
      e.bytes = rec.bytes;
      e.replay_of = rec.id;
      plant_record::external(topic, std::move(e));
    }
    if (trace_callbacks_enabled()) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding.trace"),
        "{\"event\":\"replay_insert\",\"rid\":%d,\"topic\":\"%s\",\"record_id\":%lld,"
        "\"replay_of\":%lld,\"tag_ns\":%lld,\"stamp_ns\":%lld}", rid, topic.c_str(),
        static_cast<long long>(r.id),  // NOLINT(runtime/int)
        static_cast<long long>(rec.id),  // NOLINT(runtime/int)
        static_cast<long long>(*rec.tag_ns),  // NOLINT(runtime/int)
        static_cast<long long>(stamp));  // NOLINT(runtime/int)
    }
  }
  g_replay.replayed_rids[path].insert(rid);
  {
    std::lock_guard<std::mutex> lk(g_mu);
    g_slots[rid].replay = rc;
  }
  auto n = [&skipped](const char * d) {
      auto it = skipped.find(d);
      return static_cast<unsigned long long>(it == skipped.end() ? 0 : it->second);  // NOLINT
    };
  RCLCPP_INFO(
    logger(), "rcllf replay: RID %d topic '%s': %llu admitted record(s) inserted at their live "
    "tags; not replayed: late=%llu overrun=%llu collision=%llu after_stop=%llu "
    "admitted_past_stop=%llu", rid, topic.c_str(),
    static_cast<unsigned long long>(rc.inserted),  // NOLINT(runtime/int)
    n("late"), n("overrun"), n("collision"), n("after_stop"),
    static_cast<unsigned long long>(rc.past_stop));  // NOLINT(runtime/int)
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.trace"),
      "{\"event\":\"replay_topic\",\"rid\":%d,\"topic\":\"%s\",\"inserted\":%llu,\"late\":%llu,"
      "\"overrun\":%llu,\"collision\":%llu,\"after_stop\":%llu,\"admitted_past_stop\":%llu}",
      rid, topic.c_str(), static_cast<unsigned long long>(rc.inserted),  // NOLINT(runtime/int)
      n("late"), n("overrun"), n("collision"), n("after_stop"),
      static_cast<unsigned long long>(rc.past_stop));  // NOLINT(runtime/int)
  }
}

/// Shutdown, replay only: every recorded RID was replayed and every inserted
/// record ran; otherwise the replay is reported FAILED.
void replay_log_summary()
{
  if (!plant_replay::enabled()) {
    return;
  }
  std::vector<std::string> failures;
  std::map<std::string, std::set<int>> missing;
  {
    std::lock_guard<std::mutex> rl(g_replay_mu);
    if (!g_replay.started) {
      failures.push_back("no stamp-anchored RID was replayed");
    }
    // A topic file with no entries (the
    // live run subscribed, nothing arrived) needs its subscription too.
    for (const auto & kv : g_replay.files) {
      if (g_replay.matched.count(kv.first) == 0) {
        failures.push_back(
          kv.first + ": recorded topic '" + kv.second.topic + "' has no subscription in this "
          "program");
      }
    }
    // The replay ends at the live run's
    // stop tag, not at an earlier stop request. Read after LF returned (all
    // workers joined), like plant_record's teardown lf_stop, so not under
    // enter: reactor-c may have released the environment's mutex by now.
    const SchedulerOps * ops = g_ops.load();
    void * env = g_replay.started && ops->stop_tag != nullptr ? ops->env() : nullptr;
    if (env != nullptr) {
      int64_t got = 0;
      uint32_t got_m = 0;
      ops->stop_tag(env, &got, &got_m);
      if (got != g_replay.stop_time || got_m != g_replay.stop.microstep) {
        failures.push_back(
          "the replay stopped at elapsed tag (" + std::to_string(got - g_replay.start) + ", " +
          std::to_string(got_m) + "), not at the live run's (" +
          std::to_string(g_replay.stop.tag_ns) + ", " + std::to_string(g_replay.stop.microstep) +
          ")");
      }
    }
    for (const auto & kv : g_replay.recorded_rids) {
      for (int rid : kv.second) {
        if (g_replay.replayed_rids[kv.first].count(rid) == 0) {
          failures.push_back(
            kv.first + ": recorded RID " + std::to_string(rid) + " was never replayed");
        }
      }
    }
  }
  std::lock_guard<std::mutex> lk(g_mu);
  for (const auto & kv : g_slots) {
    if (!kv.second.replaying) {
      continue;
    }
    const ReplayCounters & rc = kv.second.replay;
    RCLCPP_INFO(
      logger(), "rcllf replay summary: RID %d topic '%s': inserted=%llu delivered=%llu "
      "admitted_past_stop=%llu not_admitted=%llu", kv.first, kv.second.topic.c_str(),
      static_cast<unsigned long long>(rc.inserted),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(rc.delivered),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(rc.past_stop),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(rc.not_admitted));  // NOLINT(runtime/int)
    if (trace_callbacks_enabled()) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding.trace"),
        "{\"event\":\"replay_summary\",\"rid\":%d,\"topic\":\"%s\",\"inserted\":%llu,"
        "\"delivered\":%llu,\"admitted_past_stop\":%llu,\"not_admitted\":%llu}", kv.first,
        kv.second.topic.c_str(),
        static_cast<unsigned long long>(rc.inserted),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(rc.delivered),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(rc.past_stop),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(rc.not_admitted));  // NOLINT(runtime/int)
    }
    if (rc.delivered != rc.inserted) {
      failures.push_back(
        "RID " + std::to_string(kv.first) + " topic '" + kv.second.topic + "': " +
        std::to_string(rc.inserted) + " records inserted, " + std::to_string(rc.delivered) +
        " delivered before the stop tag");
    }
  }
  if (!failures.empty()) {
    // The runtime exits 3 (runtime.cpp).
    g_replay_failed.store(true);
  }
  for (const std::string & f : failures) {
    RCLCPP_ERROR(logger(), "rcllf replay: FAILED: %s", f.c_str());
    if (trace_callbacks_enabled()) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding.trace"), "{\"event\":\"replay_failed\",\"why\":\"%s\"}",
        json_escape(f).c_str());
    }
  }
}

/// Admit the subscription `rid` is bound to, once both the action and a
/// subscription are known, and again for each new subscription a lifecycle
/// node creates after dropping the old one.
void maybe_admit(int rid)
{
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_slots.find(rid);
    if (it == g_slots.end() || it->second.wake == nullptr || !it->second.bound) {
      return;
    }
  }
  // Outside g_mu: submit_admit waits for the ingress thread, which takes
  // g_mu in on_sample.
  rclcpp::SubscriptionBase::SharedPtr sub;
  std::string topic = "?";
  if (rid >= 0 && rid < static_cast<int>(g_handle_table.size())) {
    sub = g_handle_table[rid].subscription.lock();
    topic = g_handle_table[rid].resolved_topic;
  }
  if (sub) {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    if (s.admitted.lock() == sub) {
      return;  // this subscription is admitted already
    }
    s.admitted = sub;
  }
  if (sub) {
    attach_stamp_anchor(rid, *sub);  // before the first message can arrive
  }
  if (sub && plant_replay::enabled()) {
    replay_rid(rid, sub);  // recorded inputs instead of live ones
    return;
  }
  if (sub && IngressRole::instance().admit_external_subscription(rid, sub)) {
    RCLCPP_INFO(
      logger(), "rcllf external: RID %d topic '%s' admitted to ingress (external_topics)",
      rid, topic.c_str());
    return;
  }
  if (!rclcpp::ok()) {
    return;  // shutting down: nothing will arrive anyway
  }
  RCLCPP_FATAL(
    logger(), "rcllf external: RID %d subscription on external topic '%s' could not be "
    "admitted to the ingress thread and would never be dispatched. Refusing to run instead "
    "of dropping its messages silently.", rid, topic.c_str());
  std::exit(EXIT_FAILURE);
}

}  // namespace

void register_action(int rid, WakeFn wake, void * action)
{
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    s.wake = wake;
    s.action = action;
  }
  maybe_admit(rid);
}

void on_lifecycle_entry(int rid)
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_slots[rid].lifecycle = true;
}

size_t poke_lifecycle()
{
  // The payload is the last record id handed out, so a poke that LF folds
  // with an arrival at the same tag never hides a record (dispatch runs every
  // record up to the payload).
  std::vector<std::pair<WakeFn, void *>> pokes;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    for (const auto & kv : g_slots) {
      const Slot & s = kv.second;
      if (s.lifecycle && s.wake != nullptr && s.admitted.expired()) {
        pokes.emplace_back(s.wake, s.action);
      }
    }
  }
  const int64_t last = g_next_id.load();
  size_t scheduled = 0;
  for (const auto & p : pokes) {  // no lock held (R16)
    if (p.first(p.second, last) > 0) {
      ++scheduled;
    }
  }
  return scheduled;
}

void on_bound(int rid)
{
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_slots.find(rid);
    if (it == g_slots.end()) {
      // Bind can run before the registering startup reaction; remember it.
      g_slots[rid].bound = true;
      return;
    }
    it->second.bound = true;
  }
  maybe_admit(rid);
}

void on_sample(
  int rid, const rclcpp::SubscriptionBase::SharedPtr & source, std::shared_ptr<void> message,
  const rclcpp::MessageInfo & info)
{
  const int64_t arrival = lf_time_physical();  // before any work on the message
  Record r;
  r.source = source;
  const rmw_message_info_t & mi = info.get_rmw_message_info();
  std::memcpy(
    r.gid.data(), mi.publisher_gid.data, std::min(sizeof(mi.publisher_gid.data), r.gid.size()));
  r.ordinal = mi.publication_sequence_number;
  r.message = std::move(message);
  r.info = std::make_shared<rclcpp::MessageInfo>(info);
  r.id = g_next_id.fetch_add(1) + 1;

  WakeFn wake = nullptr;
  void * action = nullptr;
  bool stamped = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_slots.find(rid);
    if (it == g_slots.end() || it->second.wake == nullptr) {
      return;  // not admitted through register_action, cannot happen
    }
    if (it->second.anchor != nullptr) {
      stamped = true;
    } else {
      wake = it->second.wake;
      action = it->second.action;
      it->second.fifo.push_back(r);
    }
  }
  if (stamped) {
    on_stamped_sample(rid, std::move(r), arrival);
    return;
  }

  // Schedule first, so the physical tag follows `arrival` and a slow trace
  // sink or logger cannot delay it. No lock held (R16): reactor-c may exit()
  // holding the env mutex that the schedule waits for.
  const bool scheduled = wake(action, r.id) > 0;

  emit_arrival(rid, r, arrival, 0, -1, 0, Disposition::ARRIVAL);
  if (scheduled) {
    return;
  }
  bool warn = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    Slot & s = g_slots[rid];
    auto & q = s.fifo;
    q.erase(
      std::remove_if(q.begin(), q.end(), [&](const Record & x) {return x.id == r.id;}), q.end());
    warn = !s.warned_refused;
    s.warned_refused = true;
  }
  if (warn && rclcpp::ok()) {
    RCLCPP_WARN(
      logger(), "rcllf external: RID %d: LF refused to schedule record %lld (stopping?); "
      "message dropped. Further refusals on this RID are not logged.", rid,
      static_cast<long long>(r.id));  // NOLINT(runtime/int)
  }
}

size_t dispatch(void * self, int rid, int64_t record)
{
  if (!require_lf_worker("ros2_lf_invoke_external_sub", rid)) {
    return 0;
  }
  lf_time_advance_ros_clock();

  if (rid < 0 || rid >= static_cast<int>(g_handle_table.size())) {
    RCLCPP_WARN(logger(), "ros2_lf_invoke_external_sub: reaction_id %d out of range", rid);
    return 0;
  }
  // Records for a lifecycle node that is not active at this tag
  // are popped and dropped, never delivered at a later tag.
  const bool active = lifecycle_admit(rid);
  // A lifecycle node's subscription may appear on a later transition.
  // The bind admits it to ingress through on_bound.
  bind_late_handle(rid);
  auto & entry = g_handle_table[rid];
  if (entry.kind == HandleKind::UNBOUND && entry.late) {
    RCLCPP_DEBUG(
      logger(), "ros2_lf_invoke_external_sub: RID %d waits for its lifecycle node to create "
      "the subscription", rid);
    return 0;
  }
  if (entry.kind == HandleKind::UNBOUND) {
    trace_unbound("ros2_lf_invoke_external_sub", rid);
    return 0;
  }
  if (entry.kind != HandleKind::SUBSCRIPTION) {
    RCLCPP_WARN(logger(), "ros2_lf_invoke_external_sub: RID %d not bound to a subscription", rid);
    return 0;
  }
  auto sub = entry.subscription.lock();
  if (!sub && !entry.late) {
    trace_entity_expired(
      "ros2_lf_invoke_external_sub", rid, entry.node_class, entry.callback_name);
    return 0;
  }

  size_t ran = 0;
  for (;; ) {
    Record r;
    {
      std::lock_guard<std::mutex> lk(g_mu);
      auto it = g_slots.find(rid);
      if (it == g_slots.end()) {
        break;
      }
      Slot & s = it->second;
      if (s.anchor != nullptr) {
        // One event per record, found by id; a collided record was
        // erased when its event was removed, so there is nothing to run.
        if (ran > 0) {
          break;
        }
        auto rec = s.by_id.find(record);
        if (rec == s.by_id.end()) {
          break;
        }
        r = std::move(rec->second);
        s.by_id.erase(rec);
        if (s.replaying) {
          ++s.replay.delivered;
        }
      } else {
        if (s.fifo.empty() || s.fifo.front().id > record) {
          break;
        }
        r = std::move(s.fifo.front());
        s.fifo.pop_front();
      }
    }
    corr_rt::DispatchTrace dt(
      rid, RCLLF_ENTRY_EXTERNAL_SUB, RCLLF_TRACE_NO_EDGE, &r.gid, r.ordinal);
    dt.set_retained(static_cast<uint64_t>(r.id));
    if (!sub || r.source.lock() != sub) {
      // Taken from a subscription the lifecycle node has since dropped. Stock
      // destroys its queue with it, so no other callback may run it.
      lifecycle_drop(rid, r.id, entry.callback_name);
      dt.set_outcome(RCLLF_OUTCOME_LOST);
      continue;
    }
    if (!active) {
      trace_lifecycle_gated(
        "ros2_lf_invoke_external_sub", rid, entry.node_class, entry.callback_name, true);
      dt.set_outcome(RCLLF_OUTCOME_LOST);
      continue;
    }
    trace_callback_start(
      "ros2_lf_invoke_external_sub", rid, entry.node_class, entry.callback_name,
      r.info->get_rmw_message_info(), r.id);
    UserCallbackScope user;  // none after shutdown began
    if (!user.admitted()) {
      return ran;
    }
    dt.set_ran(true);
    corr_rt::OriginScope origin(self, rid);
    input_period::observe_message(rid, *r.info);
    sub->handle_message(r.message, *r.info);
    ++ran;
  }
  return ran;
}

void declare_topic(const std::string & topic)
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_declared_topics.insert(with_slash(topic));
}

bool is_declared_topic(const std::string & topic)
{
  std::lock_guard<std::mutex> lk(g_mu);
  return g_declared_topics.count(with_slash(topic)) > 0;
}

void declare_stamp_anchor(const std::string & topic, const StampAnchor & anchor)
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_declared_topics.insert(with_slash(topic));
  g_stamp_anchors[with_slash(topic)] = anchor;
}

const StampAnchor * stamp_anchor_of(const std::string & topic)
{
  std::lock_guard<std::mutex> lk(g_mu);
  auto it = g_stamp_anchors.find(with_slash(topic));
  return it == g_stamp_anchors.end() ? nullptr : &it->second;
}

Counters counters(int rid)
{
  std::lock_guard<std::mutex> lk(g_mu);
  auto it = g_slots.find(rid);
  return it == g_slots.end() ? Counters{} : it->second.counters;
}

bool replay_failed() {return g_replay_failed.load();}

ReplayCounters replay_counters(int rid)
{
  std::lock_guard<std::mutex> lk(g_mu);
  auto it = g_slots.find(rid);
  return it == g_slots.end() ? ReplayCounters{} : it->second.replay;
}

void log_summary()
{
  replay_log_summary();  // outside g_mu (lock order g_replay_mu, g_mu)
  std::vector<std::pair<int, Slot *>> stamped;
  std::lock_guard<std::mutex> lk(g_mu);
  for (auto & kv : g_slots) {
    if (kv.second.anchor == nullptr) {
      continue;
    }
    const Counters & c = kv.second.counters;
    const bool violated = c.overrun > 0 || c.collision > 0;
    RCLCPP_INFO(
      logger(), "rcllf external summary: RID %d topic '%s' (anchor: stamp): admitted=%llu "
      "late=%llu overrun=%llu collision=%llu after_stop=%llu refused=%llu queued=%zu%s",
      kv.first, kv.second.topic.c_str(),
      static_cast<unsigned long long>(c.admitted),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(c.late),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(c.overrun),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(c.collision),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(c.after_stop),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(c.refused),  // NOLINT(runtime/int)
      kv.second.by_id.size(),
      violated ? " ASSUMPTION VIOLATED (overrun: ingress_reserve_ns too small; collision: "
      "grid_ns too large for the stamps)" : "");
    if (trace_callbacks_enabled()) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding.trace"),
        "{\"event\":\"external_summary\",\"rid\":%d,\"topic\":\"%s\",\"admitted\":%llu,"
        "\"late\":%llu,\"overrun\":%llu,\"collision\":%llu,\"after_stop\":%llu,"
        "\"refused\":%llu,\"queued\":%zu}", kv.first, kv.second.topic.c_str(),
        static_cast<unsigned long long>(c.admitted),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(c.late),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(c.overrun),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(c.collision),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(c.after_stop),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(c.refused),  // NOLINT(runtime/int)
        kv.second.by_id.size());
    }
    if (kv.second.tf_seq != nullptr) {
      RCLCPP_INFO(
        logger(), "rcllf external summary: RID %d topic '%s' tf_mixed_stamps=%llu",
        kv.first, kv.second.topic.c_str(),
        static_cast<unsigned long long>(c.tf_mixed_stamps));  // NOLINT(runtime/int)
    }
  }
}

const void * main_program_object()
{
  static const void * object = [] {
      void * handle = dlopen(nullptr, RTLD_LAZY | RTLD_NOLOAD);
      struct link_map * map = nullptr;
      if (handle == nullptr || dlinfo(handle, RTLD_DI_LINKMAP, &map) != 0) {
        map = nullptr;
      }
      return static_cast<const void *>(map);
    }();
  return object;
}

SymbolOrigin resolve_symbol(const char * name)
{
  SymbolOrigin o;
  o.address = dlsym(RTLD_DEFAULT, name);
  if (o.address == nullptr) {
    return o;
  }
  Dl_info info{};
  struct link_map * map = nullptr;
  if (dladdr1(o.address, &info, reinterpret_cast<void **>(&map), RTLD_DL_LINKMAP) != 0) {
    o.object = map;
    o.path = info.dli_fname != nullptr ? info.dli_fname : "";
  }
  return o;
}

std::string check_scheduler_symbols(
  SymbolResolver resolve, const void * executable, std::vector<std::string> * origins)
{
  static const char * const names[] = {
    "_lf_get_environments", "lf_critical_section_enter", "lf_critical_section_exit",
    "lf_is_tag_after_stop_tag", "pqueue_tag_find_equal_same_tag", "pqueue_tag_remove",
    "_lf_done_using", "lf_recycle_event", "_lf_initialize_token", "_lf_schedule_at_tag",
    "lf_notify_of_event"};
  std::string problem;
  for (const char * name : names) {
    const SymbolOrigin o = resolve(name);
    const bool own = o.address != nullptr && executable != nullptr && o.object == executable;
    if (origins != nullptr) {
      origins->push_back(
        std::string(name) + " -> " + (o.address == nullptr ? "(unresolved)" :
        "'" + o.path + "'" + (own ? " (executable)" : " (NOT the executable)")));
    }
    if (!problem.empty() || own) {
      continue;
    }
    if (o.address == nullptr) {
      problem = std::string("the executable does not export reactor-c's ") + name;
    } else if (executable == nullptr) {
      problem = std::string("the executable's link map is unknown, so reactor-c's ") + name +
        " cannot be shown to be the executable's";
    } else {
      problem = std::string("reactor-c's ") + name + " resolves to '" + o.path +
        "', not to the executable (the executable does not export it, so the lookup "
        "found another object's copy)";
    }
  }
  return problem;
}

bool locate_header_stamp_in(
  const rosidl_typesupport_introspection_cpp::MessageMembers * top, size_t * sec_off,
  size_t * nsec_off, std::string * why)
{
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  using rosidl_typesupport_introspection_cpp::MessageMember;
  auto find = [](const MessageMembers * m, const char * name) -> const MessageMember * {
      for (uint32_t i = 0; i < m->member_count_; ++i) {
        if (std::strcmp(m->members_[i].name_, name) == 0 && !m->members_[i].is_array_) {
          return &m->members_[i];
        }
      }
      return nullptr;
    };
  auto is_type = [](const MessageMembers * m, const char * ns, const char * name) {
      return m->message_namespace_ != nullptr && m->message_name_ != nullptr &&
             std::strcmp(m->message_namespace_, ns) == 0 &&
             std::strcmp(m->message_name_, name) == 0;
    };
  auto type_name = [](const MessageMembers * m) {
      return std::string(m->message_namespace_ ? m->message_namespace_ : "?") + "::" +
             (m->message_name_ ? m->message_name_ : "?");
    };
  const MessageMember * header = find(top, "header");
  if (header == nullptr ||
    header->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
    header->members_ == nullptr || header->members_->data == nullptr)
  {
    *why = type_name(top) + " has no 'header' field of a message type";
    return false;
  }
  const auto * hdr = static_cast<const MessageMembers *>(header->members_->data);
  // Shape alone would accept any look-alike with a stamp of (sec, nanosec);
  // the anchor means std_msgs/Header's stamp.
  if (!is_type(hdr, "std_msgs::msg", "Header")) {
    *why = "'header' is a " + type_name(hdr) + ", not a std_msgs::msg::Header";
    return false;
  }
  const MessageMember * stamp = find(hdr, "stamp");
  if (stamp == nullptr ||
    stamp->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
    stamp->members_ == nullptr || stamp->members_->data == nullptr)
  {
    *why = "'header' has no 'stamp' field of a message type (not a std_msgs/Header)";
    return false;
  }
  const auto * time = static_cast<const MessageMembers *>(stamp->members_->data);
  if (!is_type(time, "builtin_interfaces::msg", "Time")) {
    *why = "'header.stamp' is a " + type_name(time) + ", not a builtin_interfaces::msg::Time";
    return false;
  }
  const MessageMember * sec = find(time, "sec");
  const MessageMember * nanosec = find(time, "nanosec");
  if (sec == nullptr || sec->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32 ||
    nanosec == nullptr ||
    nanosec->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32)
  {
    *why = "'header.stamp' is not a builtin_interfaces/Time (int32 sec, uint32 nanosec)";
    return false;
  }
  *sec_off = header->offset_ + stamp->offset_ + sec->offset_;
  *nsec_off = header->offset_ + stamp->offset_ + nanosec->offset_;
  return true;
}

bool locate_tf_stamps_in(
  const rosidl_typesupport_introspection_cpp::MessageMembers * top,
  const rosidl_typesupport_introspection_cpp::MessageMember ** seq, size_t * sec_off,
  size_t * nsec_off, std::string * why)
{
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  if (top == nullptr || top->message_namespace_ == nullptr || top->message_name_ == nullptr ||
    std::strcmp(top->message_namespace_, "tf2_msgs::msg") != 0 ||
    std::strcmp(top->message_name_, "TFMessage") != 0)
  {
    *why = "not a tf2_msgs::msg::TFMessage";
    return false;
  }
  for (uint32_t i = 0; i < top->member_count_; ++i) {
    const auto & m = top->members_[i];
    if (std::strcmp(m.name_, "transforms") != 0) {
      continue;
    }
    if (!m.is_array_ ||
      m.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
      m.members_ == nullptr || m.members_->data == nullptr || m.size_function == nullptr ||
      m.get_const_function == nullptr)
    {
      *why = "tf2_msgs::msg::TFMessage 'transforms' is not a sequence of messages";
      return false;
    }
    std::string elem_why;
    if (!locate_header_stamp_in(
        static_cast<const MessageMembers *>(m.members_->data), sec_off, nsec_off, &elem_why))
    {
      *why = "tf2_msgs::msg::TFMessage transforms[]: " + elem_why;
      return false;
    }
    *seq = &m;
    return true;
  }
  *why = "tf2_msgs::msg::TFMessage has no 'transforms'";
  return false;
}

int common_tf_stamp(
  const rosidl_typesupport_introspection_cpp::MessageMember * seq, const void * msg,
  size_t sec_off, size_t nsec_off, int64_t * stamp)
{
  const void * field = static_cast<const uint8_t *>(msg) + seq->offset_;
  const size_t n = seq->size_function(field);
  int64_t first = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto * e = static_cast<const uint8_t *>(seq->get_const_function(field, i));
    int32_t sec = 0;
    uint32_t nanosec = 0;
    std::memcpy(&sec, e + sec_off, sizeof(sec));
    std::memcpy(&nanosec, e + nsec_off, sizeof(nanosec));
    const int64_t v = static_cast<int64_t>(sec) * 1000000000LL + nanosec;
    if (i == 0) {
      first = v;
    } else if (v != first) {
      return -1;
    }
  }
  if (n == 0) {
    return 0;
  }
  *stamp = first;
  return 1;
}

void set_scheduler_ops_for_test(const SchedulerOps * ops)
{
  g_ops.store(ops != nullptr ? ops : &g_prod_ops);
}

size_t pending(int rid)
{
  std::lock_guard<std::mutex> lk(g_mu);
  auto it = g_slots.find(rid);
  return it == g_slots.end() ? 0 : it->second.fifo.size() + it->second.by_id.size();
}

void set_last_record_id_for_test(int64_t id)
{
  g_next_id.store(id);
}

void reset_for_test()
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_slots.clear();
  g_declared_topics.clear();
  g_stamp_anchors.clear();
  g_next_id.store(0);
  g_ops.store(&g_prod_ops);
  std::lock_guard<std::mutex> rl(g_replay_mu);
  g_replay = ReplayRun{};
  g_replay_failed.store(false);
}

}  // namespace external_input
}  // namespace ros2_lf_binding

extern "C" {

void ros2_lf_external_register(
  void * /*self*/, int reaction_id, int (*wake)(void *, int64_t), void * action)
{
  ros2_lf_binding::external_input::register_action(reaction_id, wake, action);
}

void ros2_lf_runtime_declare_external_topic(const char * topic)
{
  if (topic != nullptr) {
    ros2_lf_binding::external_input::declare_topic(topic);
  }
}

void ros2_lf_runtime_declare_external_stamp_anchor(
  const char * topic, int64_t delay_ns, int64_t ingress_reserve_ns, int64_t grid_ns)
{
  using ros2_lf_binding::external_input::kAnchorMaxNs;
  if (topic == nullptr || delay_ns < 0 || ingress_reserve_ns < 0 || grid_ns < 1 ||
    delay_ns > kAnchorMaxNs || ingress_reserve_ns > kAnchorMaxNs || grid_ns > kAnchorMaxNs)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf external: invalid stamp anchor for topic '%s' (delay_ns=%lld, "
      "ingress_reserve_ns=%lld, grid_ns=%lld; each must be in [0, 2^60], grid_ns >= 1); "
      "the generator validates these, so the generated startup is corrupt. Refusing to run.",
      topic == nullptr ? "(null)" : topic,
      static_cast<long long>(delay_ns),  // NOLINT(runtime/int)
      static_cast<long long>(ingress_reserve_ns),  // NOLINT(runtime/int)
      static_cast<long long>(grid_ns));  // NOLINT(runtime/int)
    std::_Exit(EXIT_FAILURE);
  }
  ros2_lf_binding::external_input::StampAnchor a;
  a.delay_ns = delay_ns;
  a.reserve_ns = ingress_reserve_ns;
  a.grid_ns = grid_ns;
  ros2_lf_binding::external_input::declare_stamp_anchor(topic, a);
}

int ros2_lf_invoke_external_sub(void * self, int reaction_id, int64_t record)
{
  const size_t ran = ros2_lf_binding::external_input::dispatch(self, reaction_id, record);
  if (ran > static_cast<size_t>(INT_MAX)) {
    // The count becomes the stub port's write count; capping
    // it would drop messages for the subscribers silently.
    RCLCPP_FATAL(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf external: RID %d ran %zu callbacks at one tag, more than the int write count "
      "of its stub port holds. Refusing to run instead of dropping messages.",
      reaction_id, ran);
    std::exit(EXIT_FAILURE);
  }
  return static_cast<int>(ran);
}

}  // extern "C"

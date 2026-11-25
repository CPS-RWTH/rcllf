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

/// @file reaction_dispatch.cpp
/// @brief Implements ros2_lf_invoke_timer, ros2_lf_invoke_sub, service dispatch,
///        and action dispatch.
///
/// Direct ports of on_lf_timer_fire / on_lf_subscription_fire from
/// lf_executor.cpp, keyed off the reaction_id integer into g_handle_table
/// instead of a callback-function-pointer lookup.

#include <rcl/timer.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "action_channels.hpp"
#include "corr_runtime.hpp"
#include "handle_table.hpp"
#include "input_period.hpp"
#include "per_subscription.hpp"
#include "service_pairing.hpp"
#include "take_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{
extern std::vector<HandleEntry> g_handle_table;
}

extern "C" {

void ros2_lf_invoke_timer(void * self, int reaction_id)
{
  if (!ros2_lf_binding::require_lf_worker("ros2_lf_invoke_timer", reaction_id)) {
    return;
  }
  ros2_lf_binding::corr_rt::topic_sent_reset();
  ros2_lf_binding::lf_time_advance_ros_clock();
  ros2_lf_binding::take_stats_maybe_periodic_log();

  if (reaction_id < 0 || reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"), "ros2_lf_invoke_timer: reaction_id %d out of range",
      reaction_id);
    return;
  }

  // A lifecycle node that is not active at this tag runs
  // no callback.
  if (!ros2_lf_binding::lifecycle_admit(reaction_id)) {
    const auto & gated = ros2_lf_binding::g_handle_table[reaction_id];
    ros2_lf_binding::trace_lifecycle_gated(
      "ros2_lf_invoke_timer", reaction_id, gated.node_class, gated.callback_name, false);
    return;
  }
  // A lifecycle node's handle may appear on a later transition.
  ros2_lf_binding::bind_late_handle(reaction_id);
  auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  if (entry.kind == ros2_lf_binding::HandleKind::UNBOUND && entry.late) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_timer: RID %d waits for its lifecycle node to create the handle",
        reaction_id);
    return;
  }
  if (entry.kind == ros2_lf_binding::HandleKind::UNBOUND) {
    ros2_lf_binding::trace_unbound("ros2_lf_invoke_timer", reaction_id);
    return;
  }
  if (entry.kind != ros2_lf_binding::HandleKind::TIMER) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"), "ros2_lf_invoke_timer: RID %d not bound to a timer",
      reaction_id);
    return;
  }
  auto timer_sp = entry.timer.lock();
  if (!timer_sp) {
    ros2_lf_binding::trace_entity_expired(
      "ros2_lf_invoke_timer", reaction_id, entry.node_class, entry.callback_name);
    return;
  }
  // A user cancel() parks the timer until reset(). reset() does not
  // shift the LF tick grid, the next tick after it runs the callback.
  if (timer_sp->is_canceled()) {
    return;
  }
  ros2_lf_binding::trace_callback_start(
    "ros2_lf_invoke_timer", reaction_id, entry.node_class, entry.callback_name);

  // Replicate on_lf_timer_fire from lf_executor.cpp
  auto data = std::make_shared<rcl_timer_call_info_t>();
  data->expected_call_time = 0;
  data->actual_call_time = 0;
  std::shared_ptr<void> void_data = data;
  ros2_lf_binding::corr_rt::DispatchTrace dt(reaction_id, RCLLF_ENTRY_TIMER);
  ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
  if (!user.admitted()) {
    return;
  }
  dt.set_ran(true);
  ros2_lf_binding::corr_rt::OriginScope origin(self, reaction_id);
  timer_sp->execute_callback(void_data);
}

int ros2_lf_invoke_sub(void * self, int reaction_id, const char * topic)
{
  // P17f: the caller's write count is unknown, so a miss owes nothing.
  return ros2_lf_invoke_sub_n(self, reaction_id, topic, 0);
}

namespace
{
/// Probe P17e: hand `n` owed samples the take dropped as late
/// to the subscription's message_lost handler, if one is registered.
void report_owed_lost(rclcpp::SubscriptionBase & sub, void * self, int reaction_id, uint64_t n)
{
  const auto & handlers = sub.get_event_handlers();
  const auto it = handlers.find(RCL_SUBSCRIPTION_MESSAGE_LOST);
  if (n == 0 || it == handlers.end() || !it->second) {
    return;
  }
  static std::mutex mu;
  // The running total is per live subscription, so a subscription
  // at a reused address starts from zero.
  static ros2_lf_binding::PerSubscription<size_t> totals;
  size_t total = 0;
  {
    std::lock_guard<std::mutex> lk(mu);
    total = (totals.at(sub) += n);
  }
  ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
  if (!user.admitted()) {
    return;
  }
  ros2_lf_binding::corr_rt::OriginScope origin(self, reaction_id);
  rclcpp::QOSMessageLostInfo info{};
  info.total_count = total;
  info.total_count_change = n;
  it->second->execute(std::make_shared<rclcpp::QOSMessageLostInfo>(info));
}
}  // namespace

int ros2_lf_invoke_sub_n(void * self, int reaction_id, const char * /*topic*/, int remaining)
{
  if (!ros2_lf_binding::require_lf_worker("ros2_lf_invoke_sub", reaction_id)) {
    return 0;
  }
  ros2_lf_binding::corr_rt::topic_sent_reset();
  ros2_lf_binding::lf_time_advance_ros_clock();
  ros2_lf_binding::take_stats_maybe_periodic_log();

  if (reaction_id < 0 || reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"), "ros2_lf_invoke_sub: reaction_id %d out of range",
      reaction_id);
    return 0;
  }

  // A lifecycle node that is not active at this tag runs
  // no callback.
  const bool active = ros2_lf_binding::lifecycle_admit(reaction_id);
  // A lifecycle node's handle may appear on a later transition.
  ros2_lf_binding::bind_late_handle(reaction_id);
  auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  if (entry.kind == ros2_lf_binding::HandleKind::UNBOUND && entry.late) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_sub: RID %d waits for its lifecycle node to create the handle", reaction_id);
    ros2_lf_binding::tag_pass(reaction_id, remaining);  // its writes are used up
    return 0;
  }
  if (entry.kind == ros2_lf_binding::HandleKind::UNBOUND) {
    ros2_lf_binding::tag_pass(reaction_id, remaining);
    ros2_lf_binding::trace_unbound("ros2_lf_invoke_sub", reaction_id);
    return 0;
  }
  if (entry.kind != ros2_lf_binding::HandleKind::SUBSCRIPTION) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_sub: RID %d not bound to a subscription", reaction_id);
    return 0;
  }
  auto sub_sp = entry.subscription.lock();
  if (!sub_sp) {
    ros2_lf_binding::tag_pass(reaction_id, remaining);
    ros2_lf_binding::trace_entity_expired(
      "ros2_lf_invoke_sub", reaction_id, entry.node_class, entry.callback_name);
    return 0;
  }

  // Replicate on_lf_subscription_fire from lf_executor.cpp
  auto * sub = sub_sp.get();
  rclcpp::MessageInfo message_info;
  auto message = sub->create_message();
  // Per-entry policy resolved at registration, with each key independently
  // overridden by its env var when that var is set (design §2.2, per-key).
  const ros2_lf_binding::TakeMode mode =
    ros2_lf_binding::take_mode_env().value_or(entry.take_mode);
  const int64_t timeout_ms =
    ros2_lf_binding::take_timeout_ms_env().value_or(entry.take_timeout_ms);
  if (active) {
    // Drop by tag when the trigger's writes are recorded here.
    auto tt = ros2_lf_binding::take_by_tag(
      reaction_id, *sub, mode, timeout_ms, entry.topic, remaining);
    if (!tt.count_rule) {
      report_owed_lost(*sub, self, reaction_id, tt.lost);
      int ran = 0;
      for (const auto & sample : tt.deliver) {
        auto msg = sub->create_message();
        if (!ros2_lf_binding::deserialize_taken(*sub, sample, msg.get())) {
          continue;
        }
        ros2_lf_binding::trace_callback_start(
          "ros2_lf_invoke_sub", reaction_id, entry.node_class, entry.callback_name,
          sample.info.get_rmw_message_info());
        ros2_lf_binding::corr_rt::DispatchTrace dt(reaction_id, RCLLF_ENTRY_LEGACY_SUB);
        ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
        if (!user.admitted()) {
          break;
        }
        dt.set_ran(true);
        ros2_lf_binding::corr_rt::OriginScope origin(self, reaction_id);
        ros2_lf_binding::input_period::observe_message(reaction_id, sample.info);
        sub->handle_message(msg, sample.info);
        ++ran;
      }
      if (tt.deliver.empty()) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("ros2_lf_binding"),
          "ros2_lf_invoke_sub: RID %d topic '%s': no message available", reaction_id,
          entry.topic.c_str());
      }
      return ran > 0 ? 1 : 0;
    }
  } else {
    report_owed_lost(*sub, self, reaction_id, ros2_lf_binding::tag_pass(reaction_id, remaining));
  }
  // An inactive node only drops what is already there: a blocking take would
  // hold an LF worker for a message nobody runs.
  uint64_t owed_lost = 0;
  const bool taken = active ?
    ros2_lf_binding::take_with_mode(
    *sub, message.get(), message_info, mode, timeout_ms, entry.topic, remaining, &owed_lost) :
    sub->take_type_erased(message.get(), message_info);
  report_owed_lost(*sub, self, reaction_id, owed_lost);
  if (taken && !active) {
    // Taken and dropped, as a node without the subscription would lose it.
    ros2_lf_binding::trace_lifecycle_gated(
      "ros2_lf_invoke_sub", reaction_id, entry.node_class, entry.callback_name, true);
  } else if (taken) {
    ros2_lf_binding::trace_callback_start(
      "ros2_lf_invoke_sub", reaction_id, entry.node_class, entry.callback_name,
      message_info.get_rmw_message_info());
    ros2_lf_binding::corr_rt::DispatchTrace dt(reaction_id, RCLLF_ENTRY_LEGACY_SUB);
    ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
    if (!user.admitted()) {
      return 0;
    }
    dt.set_ran(true);
    ros2_lf_binding::corr_rt::OriginScope origin(self, reaction_id);
    ros2_lf_binding::input_period::observe_message(reaction_id, message_info);
    sub->handle_message(message, message_info);
    return 1;
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_sub: RID %d topic '%s': no message available", reaction_id,
      entry.topic.c_str());
  }
  return 0;
}

void ros2_lf_invoke_srv_send(void * /*self*/, int reaction_id, const char * /*service_name*/)
{
  // The user's timer/sub callback that calls async_send_request is already
  // bound as a TIMER handle.  Re-use the exact same dispatch path.
  ros2_lf_invoke_timer(nullptr, reaction_id);
}

int ros2_lf_invoke_srv_server(void * /*self*/, int reaction_id, const char * /*service_name*/)
{
  if (!ros2_lf_binding::require_lf_worker("ros2_lf_invoke_srv_server", reaction_id)) {
    return 0;
  }
  ros2_lf_binding::lf_time_advance_ros_clock();

  if (reaction_id < 0 || reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_srv_server: reaction_id %d out of range", reaction_id);
    return 0;
  }

  // A lifecycle node that is not active at this tag runs
  // no callback.
  const bool active = ros2_lf_binding::lifecycle_admit(reaction_id);
  auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  if (entry.kind != ros2_lf_binding::HandleKind::SERVICE_SERVER) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_srv_server: RID %d not bound to a service server", reaction_id);
    return 0;
  }
  auto svc_sp = entry.service_server.lock();
  if (!svc_sp) {
    ros2_lf_binding::trace_entity_expired(
      "ros2_lf_invoke_srv_server", reaction_id, entry.node_class, entry.callback_name);
    return 0;
  }

  // Mirror rclcpp::Executor::execute_service using the public ServiceBase API.
  // One counted write stands for one request of an in-system client. A
  // request of a client outside the system is served at once and counted; it
  // never takes the place of the in-system one (review round 5, M2, D1).
  auto * svc = svc_sp.get();
  const std::string service = svc->get_service_name();
  namespace sp = ros2_lf_binding::service_pairing;
  sp::check_single_server(reaction_id, false);
  const auto wait_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(sp::kInFlightWaitMs);
  for (;; ) {
    auto request = svc->create_request();
    auto header = svc->create_request_header();
    if (!svc->take_type_erased_request(request.get(), *header)) {
      // An in-system request that was sent is in flight: wait for it, so
      // a late delivery is not a shortfall.
      if (sp::has_unserved(service) && std::chrono::steady_clock::now() < wait_end) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }
      break;
    }
    const bool outside = sp::is_outside_request(service, *header);
    if (!active) {
      ros2_lf_binding::trace_lifecycle_gated(
        "ros2_lf_invoke_srv_server", reaction_id, entry.node_class, entry.callback_name, true);
      if (outside) {
        continue;
      }
      return 0;
    }
    ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
    if (!user.admitted()) {
      return 0;
    }
    if (outside) {
      sp::note_outside_request(reaction_id, service);
    } else {
      ros2_lf_binding::trace_callback_start(
        "ros2_lf_invoke_srv_server", reaction_id, entry.node_class, entry.callback_name);
    }
    const int mark = sp::thread_sends_mark();
    svc->handle_request(header, request);
    sp::discard_thread_sends("ros2_lf_invoke_srv_server", reaction_id, mark);
    if (!outside) {
      return 1;
    }
  }
  // Each invoke stands for one counted write of the stub port:
  // a missing request is a shortfall, not an idle poll.
  sp::note_shortfall(reaction_id, "ros2_lf_invoke_srv_server", true);
  return 0;
}

int ros2_lf_invoke_srv_client_cb(void * /*self*/, int reaction_id, const char * /*service_name*/)
{
  if (!ros2_lf_binding::require_lf_worker("ros2_lf_invoke_srv_client_cb", reaction_id)) {
    return 0;
  }
  ros2_lf_binding::lf_time_advance_ros_clock();

  if (reaction_id < 0 || reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_srv_client_cb: reaction_id %d out of range", reaction_id);
    return 0;
  }

  // A lifecycle node that is not active at this tag runs
  // no callback.
  const bool active = ros2_lf_binding::lifecycle_admit(reaction_id);
  auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  if (entry.kind != ros2_lf_binding::HandleKind::SERVICE_CLIENT) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_invoke_srv_client_cb: RID %d not bound to a service client", reaction_id);
    return 0;
  }
  auto cli_sp = entry.service_client.lock();
  if (!cli_sp) {
    ros2_lf_binding::trace_entity_expired(
      "ros2_lf_invoke_srv_client_cb", reaction_id, entry.node_class, entry.callback_name);
    return 0;
  }

  // Mirror rclcpp::Executor::execute_client using the public ClientBase API.
  // Only a response to a request this client sent (its sequence number, seen
  // by the rcl_send_request gate) is delivered; any other is dropped and
  // counted (review round 5, M2).
  auto * cli = cli_sp.get();
  const void * rcl_cli = cli->get_client_handle().get();
  namespace sp = ros2_lf_binding::service_pairing;
  const auto wait_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(sp::kInFlightWaitMs);
  for (;; ) {
    auto response = cli->create_response();
    auto header = cli->create_request_header();
    if (!cli->take_type_erased_response(response.get(), *header)) {
      // A response to a request this client sent is in flight (its server
      // already ran): wait for it, so a late delivery is not a shortfall.
      if (sp::has_pending(rcl_cli) && std::chrono::steady_clock::now() < wait_end) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }
      break;
    }
    if (!sp::take_expected(rcl_cli, header->sequence_number)) {
      sp::note_foreign_response(reaction_id, cli->get_service_name(), header->sequence_number);
      continue;
    }
    if (!active) {
      ros2_lf_binding::trace_lifecycle_gated(
        "ros2_lf_invoke_srv_client_cb", reaction_id, entry.node_class, entry.callback_name, true);
      return 0;
    }
    ros2_lf_binding::UserCallbackScope user;  // none after shutdown began
    if (!user.admitted()) {
      return 0;
    }
    ros2_lf_binding::trace_callback_start(
      "ros2_lf_invoke_srv_client_cb", reaction_id, entry.node_class, entry.callback_name);
    const int mark = sp::thread_sends_mark();
    cli->handle_response(header, response);
    sp::discard_thread_sends("ros2_lf_invoke_srv_client_cb", reaction_id, mark);
    return 1;
  }
  // Each invoke stands for one counted write of the stub port:
  // a missing response is a shortfall, not an idle poll.
  sp::note_shortfall(reaction_id, "ros2_lf_invoke_srv_client_cb", false);
  return 0;
}

void ros2_lf_invoke_action_goal_send(
  void * self, int reaction_id, const char * /*action_name*/)
{
  // The user's timer callback that calls async_send_goal is already bound as a
  // TIMER handle.  Re-use the exact same dispatch path.
  ros2_lf_invoke_timer(self, reaction_id);
}

}  // extern "C"

namespace
{

/// Shared body of the counted action invokes: `count` samples
/// of `channel`, taken from entity `entity_id` of the action handle `kind`.
void invoke_action(
  const char * entry, void * self, int reaction_id, ros2_lf_binding::HandleKind kind,
  int entity_id, int channel, int count)
{
  if (!ros2_lf_binding::require_lf_worker(entry, reaction_id)) {
    return;
  }
  ros2_lf_binding::action_sends_begin(entry, reaction_id);
  ros2_lf_binding::lf_time_advance_ros_clock();

  if (reaction_id < 0 || reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"), "%s: reaction_id %d out of range", entry,
      reaction_id);
    return;
  }

  // A lifecycle node that is not active at this tag runs
  // no callback, and the samples this reaction takes are dropped.
  const bool active = ros2_lf_binding::lifecycle_admit(reaction_id);
  auto & handle_entry = ros2_lf_binding::g_handle_table[reaction_id];
  if (handle_entry.kind != kind) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"), "%s: RID %d not bound to an action %s", entry,
      reaction_id, kind == ros2_lf_binding::HandleKind::ACTION_SERVER ? "server" : "client");
    return;
  }
  if (entity_id < 0) {
    entity_id = static_cast<int>(handle_entry.action_entity_id);
  }
  ros2_lf_binding::action_counted_take(
    entry, self, reaction_id, static_cast<size_t>(entity_id), channel, count, active);
}

/// Client entity ids (GoalClient=0, ResultClient=1, CancelClient=2,
/// FeedbackSubscription=3) to the channel the entity receives.
int client_channel(size_t entity_id)
{
  switch (entity_id) {
    case 0: return RCLLF_ACTION_GOAL_RESP;
    case 1: return RCLLF_ACTION_RESULT_RESP;
    case 2: return RCLLF_ACTION_CANCEL_RESP;
    default: return RCLLF_ACTION_FEEDBACK;
  }
}

}  // namespace

extern "C" {

// Server entity ids: GoalService=0, ResultService=1, CancelService=2. A
// failed take is harmless in rclcpp_action (execute ignores TAKE_FAILED and
// keeps no ready flag), so the counted take may retry it until the sample
// arrives.

void ros2_lf_invoke_action_goal_server(
  void * self, int reaction_id, const char * /*action_name*/, int count)
{
  invoke_action(
    "ros2_lf_invoke_action_goal_server", self, reaction_id,
    ros2_lf_binding::HandleKind::ACTION_SERVER, 0, RCLLF_ACTION_GOAL_REQ, count);
}

void ros2_lf_invoke_action_client_cb(
  void * self, int reaction_id, const char * /*action_name*/, int count)
{
  int channel = RCLLF_ACTION_FEEDBACK;
  if (reaction_id >= 0 && reaction_id < static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
    channel = client_channel(ros2_lf_binding::g_handle_table[reaction_id].action_entity_id);
  }
  invoke_action(
    "ros2_lf_invoke_action_client_cb", self, reaction_id,
    ros2_lf_binding::HandleKind::ACTION_CLIENT, -1, channel, count);
}

void ros2_lf_invoke_action_cancel_server(
  void * self, int reaction_id, const char * /*action_name*/, int count)
{
  invoke_action(
    "ros2_lf_invoke_action_cancel_server", self, reaction_id,
    ros2_lf_binding::HandleKind::ACTION_SERVER, 2, RCLLF_ACTION_CANCEL_REQ, count);
}

void ros2_lf_invoke_action_result_server(
  void * self, int reaction_id, const char * /*action_name*/, int count)
{
  // The client sends GetResult once its goal is accepted. A terminal goal is
  // answered here, a running one later from succeed/canceled/abort.
  invoke_action(
    "ros2_lf_invoke_action_result_server", self, reaction_id,
    ros2_lf_binding::HandleKind::ACTION_SERVER, 1, RCLLF_ACTION_RESULT_REQ, count);
}

}  // extern "C"

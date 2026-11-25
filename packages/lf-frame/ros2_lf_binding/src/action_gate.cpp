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

/// @file action_gate.cpp
/// @brief Executable-side rcl_action gate.
///
/// Compiled into every generated executable next to publication_gate.cpp
/// (installed to share/ros2_lf_binding/src). Defining these rcl_action
/// symbols in the executable preempts librcl_action for librclcpp_action's
/// calls, and each forwards to the next definition via dlsym(RTLD_NEXT).
///
/// Sends and feedback publishes are reported to the binding, which counts
/// them per thread so the generated reaction raises its port once per sample
/// it really sent, and which may refuse one that has no LF origin. Takes
/// report their return code, so a counted take knows whether it took a
/// sample. rclcpp_action ignores a failed take and hands back a tuple either
/// way. The handles are passed through as opaque pointers, so this file needs
/// no rcl_action header.
///
/// Item 6: rclcpp::Context's pre-shutdown registration is wrapped too. A
/// callback that user code registers while a node that serves an action is
/// constructed is kept by the binding, not the context, so the node's
/// generated reaction(shutdown) runs it with an LF origin. rclcpp's own
/// registrations stay with the context.
///
/// rclcpp_action::ClientBase's send_goal_request,
/// send_result_request and send_cancel_request are wrapped too (virtual and
/// exported, so the client vtables bind to the executable's definitions).
/// Each hands its response callback to the binding, which guards it against
/// a response taken while the client's lifecycle node is inactive. They are
/// defined as free functions under the members' mangled names, `this` first,
/// so this file needs no rclcpp_action header either.
///
/// rcl_send_request (plain services) is wrapped as well: rclcpp's
/// Client<T>::async_send_request is a header template, so its call lands
/// here. A sent request's sequence number is reported to the binding, which
/// delivers only responses to sequence numbers its in-system clients sent and
/// counts the requests each thread sent (review round 5, M2 and O1-2).

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>

#include "rcl/client.h"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "ros2_lf_binding/action_gate_api.h"

namespace
{
using rcl_ret = int32_t;  // rcl_ret_t

template<typename Fn>
Fn next_symbol(const char * name)
{
  return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

using SendFn = rcl_ret (*)(const void *, const void *, int64_t *);
using RespondFn = rcl_ret (*)(const void *, void *, void *);
using FeedbackFn = rcl_ret (*)(const void *, void *);
using TakeFn = rcl_ret (*)(const void *, void *, void *);
using TakeFeedbackFn = rcl_ret (*)(const void *, void *);

rcl_ret send(int channel, SendFn next, const void * client, const void * request, int64_t * seq)
{
  if (!rcllf_action_gate_pre(channel, client)) {
    return 0;  // RCL_RET_OK: refused, not sent
  }
  const rcl_ret ret = next(client, request, seq);
  rcllf_action_gate_post(channel, client, ret);
  return ret;
}

rcl_ret respond(int channel, RespondFn next, const void * server, void * header, void * response)
{
  if (!rcllf_action_gate_pre(channel, server)) {
    return 0;
  }
  const rcl_ret ret = next(server, header, response);
  rcllf_action_gate_post(channel, server, ret);
  return ret;
}

rcl_ret take(int channel, TakeFn next, const void * handle, void * header, void * msg)
{
  const rcl_ret ret = next(handle, header, msg);
  rcllf_action_gate_take(channel, handle, ret);
  return ret;
}

/// `addr` lies in the shared object that defines `sym`.
bool same_object(const void * addr, const void * sym)
{
  Dl_info a{};
  Dl_info b{};
  return dladdr(addr, &a) != 0 && dladdr(sym, &b) != 0 && a.dli_fbase == b.dli_fbase;
}
}  // namespace

namespace rclcpp
{

// A member function takes `this` as its first argument in the Itanium ABI.
PreShutdownCallbackHandle Context::add_pre_shutdown_callback(PreShutdownCallback callback)
{
  using Fn = PreShutdownCallbackHandle (*)(Context *, PreShutdownCallback);
  static Fn next = next_symbol<Fn>(
    "_ZN6rclcpp7Context25add_pre_shutdown_callbackESt8functionIFvvEE");
  // Only the default context gets the binding's own pre-shutdown hook that
  // stops LF first, so a callback on any other context stays with it.
  if (this == contexts::get_global_default_context().get() &&
    !same_object(__builtin_return_address(0), reinterpret_cast<const void *>(next)))
  {
    if (auto kept = ros2_lf_binding::action_keep_pre_shutdown(callback)) {
      PreShutdownCallbackHandle handle;
      handle.callback = kept;
      return handle;
    }
  }
  return next(this, std::move(callback));
}

bool Context::remove_pre_shutdown_callback(const PreShutdownCallbackHandle & callback_handle)
{
  using Fn = bool (*)(Context *, const PreShutdownCallbackHandle &);
  static Fn next = next_symbol<Fn>(
    "_ZN6rclcpp7Context28remove_pre_shutdown_callbackERKNS_22ShutdownCallbackHandleE");
  if (auto kept = callback_handle.callback.lock()) {
    if (ros2_lf_binding::action_drop_pre_shutdown(kept)) {
      return true;
    }
  }
  return next(this, callback_handle);
}

}  // namespace rclcpp

namespace
{
using ResponseCallback = ros2_lf_binding::ActionResponseCallback;
using ClientSendFn = void (*)(void *, std::shared_ptr<void>, ResponseCallback);

#define RCLLF_CLIENT_SEND_SYMBOL(member) \
  "_ZN13rclcpp_action10ClientBase" member "ESt10shared_ptrIvESt8functionIFvS2_EE"
}  // namespace

// rclcpp_action::ClientBase::send_*_request(std::shared_ptr<void>, ResponseCallback).
void rcllf_client_send_goal_request(void *, std::shared_ptr<void>, ResponseCallback)
__asm__ (RCLLF_CLIENT_SEND_SYMBOL("17send_goal_request"));
void rcllf_client_send_result_request(void *, std::shared_ptr<void>, ResponseCallback)
__asm__ (RCLLF_CLIENT_SEND_SYMBOL("19send_result_request"));
void rcllf_client_send_cancel_request(void *, std::shared_ptr<void>, ResponseCallback)
__asm__ (RCLLF_CLIENT_SEND_SYMBOL("19send_cancel_request"));

void rcllf_client_send_goal_request(
  void * client, std::shared_ptr<void> request, ResponseCallback callback)
{
  static ClientSendFn next =
    next_symbol<ClientSendFn>(RCLLF_CLIENT_SEND_SYMBOL("17send_goal_request"));
  auto guarded = ros2_lf_binding::action_client_guard(
    client, RCLLF_ACTION_GOAL_REQ, request, std::move(callback),
    &rcllf_client_send_cancel_request);
  next(client, std::move(request), std::move(guarded));
}

void rcllf_client_send_result_request(
  void * client, std::shared_ptr<void> request, ResponseCallback callback)
{
  static ClientSendFn next =
    next_symbol<ClientSendFn>(RCLLF_CLIENT_SEND_SYMBOL("19send_result_request"));
  auto guarded = ros2_lf_binding::action_client_guard(
    client, RCLLF_ACTION_RESULT_REQ, request, std::move(callback),
    &rcllf_client_send_cancel_request);
  next(client, std::move(request), std::move(guarded));
}

void rcllf_client_send_cancel_request(
  void * client, std::shared_ptr<void> request, ResponseCallback callback)
{
  static ClientSendFn next =
    next_symbol<ClientSendFn>(RCLLF_CLIENT_SEND_SYMBOL("19send_cancel_request"));
  auto guarded = ros2_lf_binding::action_client_guard(
    client, RCLLF_ACTION_CANCEL_REQ, request, std::move(callback),
    &rcllf_client_send_cancel_request);
  next(client, std::move(request), std::move(guarded));
}

extern "C" {

rcl_ret_t rcl_send_request(const rcl_client_t * client, const void * request, int64_t * seq)
{
  using RclSendFn = rcl_ret_t (*)(const rcl_client_t *, const void *, int64_t *);
  static RclSendFn next = next_symbol<RclSendFn>("rcl_send_request");
  const rcl_ret ret = next(client, request, seq);
  if (ret == 0 && seq != nullptr) {
    rcllf_service_gate_sent(client, *seq);
  }
  return ret;
}

rcl_ret rcl_action_send_goal_request(const void * client, const void * request, int64_t * seq)
{
  static SendFn next = next_symbol<SendFn>("rcl_action_send_goal_request");
  return send(RCLLF_ACTION_GOAL_REQ, next, client, request, seq);
}

rcl_ret rcl_action_send_result_request(const void * client, const void * request, int64_t * seq)
{
  static SendFn next = next_symbol<SendFn>("rcl_action_send_result_request");
  return send(RCLLF_ACTION_RESULT_REQ, next, client, request, seq);
}

rcl_ret rcl_action_send_cancel_request(const void * client, const void * request, int64_t * seq)
{
  static SendFn next = next_symbol<SendFn>("rcl_action_send_cancel_request");
  return send(RCLLF_ACTION_CANCEL_REQ, next, client, request, seq);
}

rcl_ret rcl_action_send_goal_response(const void * server, void * header, void * response)
{
  static RespondFn next = next_symbol<RespondFn>("rcl_action_send_goal_response");
  return respond(RCLLF_ACTION_GOAL_RESP, next, server, header, response);
}

rcl_ret rcl_action_send_result_response(const void * server, void * header, void * response)
{
  static RespondFn next = next_symbol<RespondFn>("rcl_action_send_result_response");
  return respond(RCLLF_ACTION_RESULT_RESP, next, server, header, response);
}

rcl_ret rcl_action_send_cancel_response(const void * server, void * header, void * response)
{
  static RespondFn next = next_symbol<RespondFn>("rcl_action_send_cancel_response");
  return respond(RCLLF_ACTION_CANCEL_RESP, next, server, header, response);
}

rcl_ret rcl_action_publish_feedback(const void * server, void * feedback)
{
  static FeedbackFn next = next_symbol<FeedbackFn>("rcl_action_publish_feedback");
  if (!rcllf_action_gate_pre(RCLLF_ACTION_FEEDBACK, server)) {
    return 0;
  }
  const rcl_ret ret = next(server, feedback);
  rcllf_action_gate_post(RCLLF_ACTION_FEEDBACK, server, ret);
  return ret;
}

rcl_ret rcl_action_take_goal_request(const void * server, void * header, void * request)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_goal_request");
  return take(RCLLF_ACTION_GOAL_REQ, next, server, header, request);
}

rcl_ret rcl_action_take_result_request(const void * server, void * header, void * request)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_result_request");
  return take(RCLLF_ACTION_RESULT_REQ, next, server, header, request);
}

rcl_ret rcl_action_take_cancel_request(const void * server, void * header, void * request)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_cancel_request");
  return take(RCLLF_ACTION_CANCEL_REQ, next, server, header, request);
}

rcl_ret rcl_action_take_goal_response(const void * client, void * header, void * response)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_goal_response");
  return take(RCLLF_ACTION_GOAL_RESP, next, client, header, response);
}

rcl_ret rcl_action_take_result_response(const void * client, void * header, void * response)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_result_response");
  return take(RCLLF_ACTION_RESULT_RESP, next, client, header, response);
}

rcl_ret rcl_action_take_cancel_response(const void * client, void * header, void * response)
{
  static TakeFn next = next_symbol<TakeFn>("rcl_action_take_cancel_response");
  return take(RCLLF_ACTION_CANCEL_RESP, next, client, header, response);
}

rcl_ret rcl_action_take_feedback(const void * client, void * feedback)
{
  static TakeFeedbackFn next = next_symbol<TakeFeedbackFn>("rcl_action_take_feedback");
  const rcl_ret ret = next(client, feedback);
  rcllf_action_gate_take(RCLLF_ACTION_FEEDBACK, client, ret);
  return ret;
}

void * rcllf_action_gate_symbol(const char * name)
{
  static const struct
  {
    const char * name;
    void * fn;
  } table[] = {
    {"rcl_send_request", reinterpret_cast<void *>(&rcl_send_request)},
    {"rcl_action_send_goal_request", reinterpret_cast<void *>(&rcl_action_send_goal_request)},
    {"rcl_action_send_result_request", reinterpret_cast<void *>(&rcl_action_send_result_request)},
    {"rcl_action_send_cancel_request", reinterpret_cast<void *>(&rcl_action_send_cancel_request)},
    {"rcl_action_send_goal_response", reinterpret_cast<void *>(&rcl_action_send_goal_response)},
    {"rcl_action_send_result_response",
      reinterpret_cast<void *>(&rcl_action_send_result_response)},
    {"rcl_action_send_cancel_response",
      reinterpret_cast<void *>(&rcl_action_send_cancel_response)},
    {"rcl_action_publish_feedback", reinterpret_cast<void *>(&rcl_action_publish_feedback)},
    {"rcl_action_take_goal_request", reinterpret_cast<void *>(&rcl_action_take_goal_request)},
    {"rcl_action_take_result_request", reinterpret_cast<void *>(&rcl_action_take_result_request)},
    {"rcl_action_take_cancel_request", reinterpret_cast<void *>(&rcl_action_take_cancel_request)},
    {"rcl_action_take_goal_response", reinterpret_cast<void *>(&rcl_action_take_goal_response)},
    {"rcl_action_take_result_response",
      reinterpret_cast<void *>(&rcl_action_take_result_response)},
    {"rcl_action_take_cancel_response",
      reinterpret_cast<void *>(&rcl_action_take_cancel_response)},
    {"rcl_action_take_feedback", reinterpret_cast<void *>(&rcl_action_take_feedback)},
    {"rclcpp_action::ClientBase::send_goal_request",
      reinterpret_cast<void *>(&rcllf_client_send_goal_request)},
    {"rclcpp_action::ClientBase::send_result_request",
      reinterpret_cast<void *>(&rcllf_client_send_result_request)},
    {"rclcpp_action::ClientBase::send_cancel_request",
      reinterpret_cast<void *>(&rcllf_client_send_cancel_request)},
  };
  for (const auto & e : table) {
    if (std::strcmp(name, e.name) == 0) {
      return e.fn;
    }
  }
  return nullptr;
}

}  // extern "C"

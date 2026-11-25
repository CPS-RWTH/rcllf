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

/// @file action_gate_api.h
/// @brief Hooks between the executable-side action gate and the binding.
///
/// The gate (src/action_gate.cpp, compiled into every generated executable)
/// wraps the rcl_action send, publish and take calls. Sends and feedback
/// publishes are counted per thread so the generated reaction raises its port
/// once per sample it really sent. Takes report their return code so a counted
/// take knows whether it took a sample. The gate also wraps
/// rclcpp::Context's pre-shutdown registration for item 6, and
/// rclcpp_action::ClientBase's send members for the guard.

#ifndef ROS2_LF_BINDING__ACTION_GATE_API_H_
#define ROS2_LF_BINDING__ACTION_GATE_API_H_

#ifdef __cplusplus
extern "C" {
#endif

/** One channel of an action. Requests flow client to server, the rest back. */
enum rcllf_action_channel
{
  RCLLF_ACTION_GOAL_REQ = 0,
  RCLLF_ACTION_RESULT_REQ = 1,
  RCLLF_ACTION_CANCEL_REQ = 2,
  RCLLF_ACTION_GOAL_RESP = 3,
  RCLLF_ACTION_RESULT_RESP = 4,
  RCLLF_ACTION_CANCEL_RESP = 5,
  RCLLF_ACTION_FEEDBACK = 6,
  RCLLF_ACTION_CHANNELS = 7,
};

/**
 * Before a send or a feedback publish on `handle` (an rcl_action_client_t for
 * a request, an rcl_action_server_t otherwise). Returns 0 to refuse it, so it
 * is not forwarded and the gate reports success to the caller.
 */
int rcllf_action_gate_pre(int channel, const void * handle);
/** After the real send or publish returned `ret`. */
void rcllf_action_gate_post(int channel, const void * handle, int ret);
/** After a take on `handle` returned `ret`. */
void rcllf_action_gate_take(int channel, const void * handle, int ret);
/** A plain service request `client` (an rcl_client_t) sent with `seq`. */
void rcllf_service_gate_sent(const void * client, int64_t seq);
/** Self-check: the gate's own address for each gated symbol, or NULL. */
void * rcllf_action_gate_symbol(const char * name);

#ifdef __cplusplus
}

#include <functional>
#include <memory>

namespace ros2_lf_binding
{

/**
 * Item 6: a pre-shutdown callback that user code registers on its context
 * while a node that serves an action is constructed. The binding keeps it
 * instead of the context and returns the pointer it keeps, so the node's
 * generated reaction(shutdown) runs it with an LF origin. Returns null for any
 * other callback, which the caller forwards to rclcpp.
 */
  std::shared_ptr < std::function <
  void() >> action_keep_pre_shutdown(std::function < void() > &callback);

/** Drops a kept callback. Returns false when `callback` is not one. */
  bool action_drop_pre_shutdown(const std::shared_ptr < std::function < void() >> &callback);

/** rclcpp_action::ClientBase::ResponseCallback. */
  using ActionResponseCallback = std::function < void(std::shared_ptr < void >) >;
/** The gate's ClientBase::send_cancel_request, `client` its `this`. */
  using ActionSendFn = void (*)(void * client, std::shared_ptr < void > request,
    ActionResponseCallback callback);

/**
 * The response callback of a request `client` (an
 * rclcpp_action::ClientBase) sends on `channel` (GOAL_REQ, RESULT_REQ or
 * CANCEL_REQ), wrapped so it does not reach user code while the client's
 * lifecycle node is inactive. rclcpp still erases its pending entry. A goal
 * accepted while inactive is cancelled once through `send_cancel`.
 */
  ActionResponseCallback action_client_guard(
    void * client, int channel, const std::shared_ptr < void > &request,
    ActionResponseCallback callback, ActionSendFn send_cancel);

}  // namespace ros2_lf_binding
#endif

#endif  // ROS2_LF_BINDING__ACTION_GATE_API_H_

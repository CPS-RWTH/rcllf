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

/// @file clock_guard.cpp
/// @brief Clock uses rcllf cannot drive from LF logical time.
///        See clock_guard.hpp and clock_gate_api.h.

#include "clock_guard.hpp"

#include <dlfcn.h>

#include <atomic>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "corr_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/clock_gate_api.h"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{

namespace
{
thread_local std::string t_callback_node;
thread_local std::string t_callback_name;

std::mutex g_once_mutex;
std::set<std::string> g_once_keys;
std::atomic<uint64_t> g_diagnostics{0};
std::atomic<bool> g_disabled{false};

// Clocks the binding attached by its own calls:
// its g_lf_clock and every node clock ros2_lf_attach_node_clock attaches. A
// component's clocks are not in here. The binding feeds the component's
// TimeSource and never holds its clocks, so it cannot name them reliably.
std::mutex g_clocks_mutex;
std::unordered_set<const rcl_clock_t *> g_lf_clocks;

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));
std::string format(const char * fmt, ...)
{
  char buf[1024];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return buf;
}

bool registered_lf_clock(const rcl_clock_t * clock)
{
  std::lock_guard<std::mutex> lock(g_clocks_mutex);
  return g_lf_clocks.count(clock) != 0;
}

/// True for an RCL_ROS_TIME clock whose ROS time override is on: its time
/// moves only when someone sets it.
bool override_enabled(rcl_clock_t * clock)
{
  if (clock == nullptr || clock->type != RCL_ROS_TIME) {
    return false;
  }
  bool enabled = false;
  if (rcl_is_enabled_ros_time_override(clock, &enabled) != RCL_RET_OK) {
    rcl_reset_error();
    return false;
  }
  return enabled;
}

/// True for a clock that follows LF logical time: the binding attached it and
/// its override is still on.
bool follows_lf_time(rcl_clock_t * clock)
{
  return registered_lf_clock(clock) && override_enabled(clock);
}

/// True for a jump callback that lives in librcl: the bookkeeping callback
/// rcl_timer_init adds to every timer on a ROS clock. It only resets the
/// timer's next call time and runs no user code.
bool rcl_internal_callback(const void * callback)
{
  Dl_info info{};
  return callback != nullptr && dladdr(callback, &info) != 0 && info.dli_fname != nullptr &&
         std::strstr(info.dli_fname, "/librcl.so") != nullptr;
}

bool in_user_callback() {return t_user_callback_depth > 0;}

/// Node and callback of the calling thread, for messages.
std::pair<std::string, std::string> where()
{
  if (in_user_callback() && !t_callback_node.empty()) {
    return {t_callback_node, t_callback_name};
  }
  if (!corr_rt::construct_key().empty()) {
    return {corr_rt::construct_key(), "constructor"};
  }
  if (is_lf_worker_thread()) {
    return {"<unknown node>", "<LF reaction>"};
  }
  return {"<unknown node>", "<no callback>"};
}

/// "Node 'KEY' (ROS node '/ns/name')": the binding key and the ROS nodes
/// created under it. A key may own several rcl nodes, a node
/// class that builds helper nodes, and then all are listed.
std::string node_text(const std::string & node, const std::vector<std::string> & ros_nodes)
{
  std::string text = "Node '" + node + "' (";
  if (ros_nodes.empty()) {
    return text + "ROS node unknown)";
  }
  text += ros_nodes.size() == 1 ? "ROS node " : "ROS nodes ";
  for (size_t i = 0; i < ros_nodes.size(); ++i) {
    text += (i == 0 ? "'" : ", '") + ros_nodes[i] + "'";
  }
  return text + ")";
}

bool first_time(const std::string & key)
{
  std::lock_guard<std::mutex> lock(g_once_mutex);
  return g_once_keys.insert(key).second;
}

/// True for a read rclcpp makes for itself: the caller lies in librclcpp and
/// is not rclcpp::Node::now(). Every Node constructor stamps the parameter
/// events of its declared parameters with the node clock, for example.
bool rclcpp_internal_read(const void * caller)
{
  Dl_info info{};
  if (caller == nullptr || dladdr(caller, &info) == 0 || info.dli_fname == nullptr) {
    return false;
  }
  if (std::strstr(info.dli_fname, "/librclcpp.so") == nullptr) {
    return false;
  }
  return info.dli_sname == nullptr || std::strcmp(info.dli_sname, "_ZNK6rclcpp4Node3nowEv") != 0;
}

}  // namespace

std::string ros_sleep_refusal_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback, int64_t until_ns, int64_t now_ns)
{
  return format(
    "%s callback '%s' sleeps on ROS time until %" PRId64 " ns (now %" PRId64
    " ns) through rclcpp::Clock::sleep_for or sleep_until. The clock follows LF "
    "logical time, which does not advance until the reaction ends, so the sleep would "
    "never return. Use a timer for a delay, or an RCL_STEADY_TIME clock "
    "for a wall-clock wait. rcllf stops the program.",
    node_text(node, ros_nodes).c_str(), callback.c_str(), until_ns, now_ns);
}

std::string jump_handler_refusal_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback, int64_t min_forward_ns)
{
  return format(
    "%s (%s) adds a forward clock jump handler (min_forward %" PRId64
    " ns) to a clock that follows LF logical time. LF time jumps forward at every tag, "
    "so the handler would run on an LF worker outside the node's reactions. "
    "React to time in a timer instead. Backward and clock-change handlers stay "
    "allowed. rcllf stops the program.",
    node_text(node, ros_nodes).c_str(), callback.c_str(), min_forward_ns);
}

std::string multi_env_refusal_message(int n_envs)
{
  return format(
    "The LF program has %d environments (LF enclaves). rcllf drives ROS time from one "
    "logical timeline, so node clocks cannot follow several environments. "
    "Build the system without enclaves. rcllf stops the program.",
    n_envs);
}

std::string own_clock_read_message(
  const std::string & node, const std::vector<std::string> & ros_nodes,
  const std::string & callback)
{
  return format(
    "%s callback '%s' reads an RCL_ROS_TIME clock that does not follow LF "
    "logical time (a clock the node built itself, not get_clock()). It returns physical "
    "time, not the logical tag. Use this->now() or this->get_clock(). "
    "Logged once per callback.",
    node_text(node, ros_nodes).c_str(), callback.c_str());
}

std::string pre_tag_read_message(
  const std::string & node, const std::vector<std::string> & ros_nodes, int64_t value_ns)
{
  return format(
    "%s reads ROS time while it is constructed, before the first LF tag. It got "
    "%" PRId64 " ns (%s), not a logical time. Read time in a reaction, "
    "for example in the first timer tick. Logged once per node.",
    node_text(node, ros_nodes).c_str(), value_ns,
    value_ns == 0 ? "0, the clock waits for the first tag" :
    "physical time, the clock follows LF time once the node is attached");
}

void refuse_clock_use(const std::string & message)
{
  RCLCPP_FATAL(rclcpp::get_logger("ros2_lf_binding"), "%s", message.c_str());
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(EXIT_FAILURE);
}

void check_single_environment(int n_envs)
{
  if (n_envs > 1) {
    refuse_clock_use(multi_env_refusal_message(n_envs));
  }
}

void register_lf_clock(const rcl_clock_t * clock)
{
  if (clock == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_clocks_mutex);
  g_lf_clocks.insert(clock);
}

void set_current_callback(const std::string & node, const std::string & callback)
{
  t_callback_node = node;
  t_callback_name = callback;
}

uint64_t clock_diagnostic_count() {return g_diagnostics.load();}

void set_clock_guard_disabled_for_test(bool disabled) {g_disabled.store(disabled);}

void clock_guard_reset_for_test()
{
  {
    std::lock_guard<std::mutex> lock(g_once_mutex);
    g_once_keys.clear();
    g_diagnostics.store(0);
  }
  std::lock_guard<std::mutex> lock(g_clocks_mutex);
  g_lf_clocks.clear();
}

}  // namespace ros2_lf_binding

namespace rb = ros2_lf_binding;

extern "C" void rcllf_clock_gate_on_sleep(rcl_clock_t * clock, int64_t until_ns)
{
  // Only a reaction is checked: no tag advances until it ends. A constructor or
  // a plain user thread fails open.
  if (rb::g_disabled.load() || !(rb::in_user_callback() || rb::is_lf_worker_thread()) ||
    !rb::follows_lf_time(clock))
  {
    return;  // outside a reaction, or a clock the binding did not attach
  }
  rcl_time_point_value_t now_ns = 0;
  if (rcl_clock_get_now(clock, &now_ns) != RCL_RET_OK) {
    rcl_reset_error();
    return;
  }
  if (until_ns <= now_ns) {
    return;  // returns at once
  }
  // The clock follows LF time, which is frozen in the reaction. Any other
  // clock, a component's included, fails open: a clock that
  // stands still for a while may still move later.
  const auto [node, callback] = rb::where();
  rb::refuse_clock_use(
    rb::ros_sleep_refusal_message(
      node, rb::corr_rt::ros_node_names(node), callback, until_ns, now_ns));
}

extern "C" void rcllf_clock_gate_on_jump_handler(
  rcl_clock_t * clock, const rcl_jump_threshold_t * threshold, const void * jump_callback,
  int in_sleep)
{
  // A wait_until handler in a reaction stays refused: the wait would hang
  // like a sleep, and the gate cannot read the condition variable's clock.
  const bool waiter_handler =
    in_sleep == 1 || (in_sleep == 2 && !(rb::in_user_callback() || rb::is_lf_worker_thread()));
  if (rb::g_disabled.load() || waiter_handler || threshold == nullptr ||
    threshold->min_forward.nanoseconds <= 0 || rb::rcl_internal_callback(jump_callback) ||
    !rb::follows_lf_time(clock))
  {
    return;  // a handler added before the clock is attached fails open
  }
  const auto [node, callback] = rb::where();
  rb::refuse_clock_use(
    rb::jump_handler_refusal_message(
      node, rb::corr_rt::ros_node_names(node), callback, threshold->min_forward.nanoseconds));
}

extern "C" void rcllf_clock_gate_on_ros_read(
  rcl_clock_t * clock, int64_t value, const void * caller)
{
  if (rb::g_disabled.load()) {
    return;
  }
  const std::string & key = rb::corr_rt::construct_key();
  if (!key.empty()) {
    if (!rb::lf_time_started() && !rb::rclcpp_internal_read(caller) &&
      rb::first_time("pre_tag/" + key))
    {
      rb::g_diagnostics.fetch_add(1);
      RCLCPP_WARN(
        rclcpp::get_logger("ros2_lf_binding"), "%s",
        rb::pre_tag_read_message(key, rb::corr_rt::ros_node_names(key), value).c_str());
    }
    return;
  }
  // Only a clock without override is reported: it returns physical time. A
  // clock whose override is on may be a component's, which LF time drives.
  if (rb::in_user_callback() && clock != nullptr && clock->type == RCL_ROS_TIME &&
    !rb::override_enabled(clock))
  {
    const auto [node, callback] = rb::where();
    if (rb::first_time("own/" + node + "/" + callback)) {
      rb::g_diagnostics.fetch_add(1);
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_lf_binding"), "%s",
        rb::own_clock_read_message(node, rb::corr_rt::ros_node_names(node), callback).c_str());
    }
  }
}

extern "C" void rcllf_clock_gate_on_clock_fini(rcl_clock_t * clock)
{
  // A later clock at this address is a different clock (a reused address).
  std::lock_guard<std::mutex> lock(rb::g_clocks_mutex);
  rb::g_lf_clocks.erase(clock);
}

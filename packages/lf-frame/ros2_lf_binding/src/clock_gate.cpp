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

/// @file clock_gate.cpp
/// @brief Executable-side clock gate.
///
/// Compiled into every generated executable (installed to
/// share/ros2_lf_binding/src), like publication_gate.cpp. Defining these
/// symbols in the executable preempts librcl and librclcpp for their PLT
/// calls: Node::now() reaches rclcpp::Clock::now(),
/// Clock::create_jump_callback and rcl_timer_init reach
/// rcl_clock_add_jump_callback, rclcpp::Clock's destructor reaches
/// rcl_clock_fini, Clock::sleep_for reaches Clock::sleep_until through the
/// PLT, and librclcpp_action reaches ClockConditionalVariable::wait_until.
/// Reads are
/// gated at rclcpp::Clock::now() and not at rcl_clock_get_now, so rcl's own
/// reads (rcl_timer_init in a constructor) stay out of the diagnostics. Each
/// definition calls its hook in clock_gate_api.h, then forwards to the next
/// definition via dlsym(RTLD_NEXT). A thread-local flag keeps a hook's own
/// clock reads and logging out of the gate.

#include <dlfcn.h>

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/time.hpp"
#include "ros2_lf_binding/clock_gate_api.h"

namespace
{
thread_local bool t_in_clock_gate = false;
// Depth of rclcpp::Clock::sleep_until on this thread: the jump handler it adds
// for itself is not a user handler.
thread_local int t_sleep_depth = 0;
// Depth of rclcpp::ClockConditionalVariable::wait_until on this thread, for
// the same reason (rclcpp_action's goal expire thread).
thread_local int t_cv_wait_depth = 0;

template<typename Fn>
Fn next_symbol(const char * name)
{
  return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

struct InClockGate
{
  InClockGate() {t_in_clock_gate = true;}
  ~InClockGate() {t_in_clock_gate = false;}
};

struct InSleep
{
  InSleep() {++t_sleep_depth;}
  ~InSleep() {--t_sleep_depth;}
};

struct InCvWait
{
  InCvWait() {++t_cv_wait_depth;}
  ~InCvWait() {--t_cv_wait_depth;}
};

/// rclcpp's rcl jump callback, the one every rclcpp::JumpHandler goes through.
void * rclcpp_jump_trampoline()
{
  static void * fn = dlsym(RTLD_DEFAULT, "_ZN6rclcpp5Clock12on_time_jumpEPK15rcl_time_jump_sbPv");
  return fn;
}
}  // namespace

extern "C" {

rcl_ret_t rcl_clock_add_jump_callback(
  rcl_clock_t * clock, rcl_jump_threshold_t threshold, rcl_jump_callback_t callback,
  void * user_data)
{
  using Fn = rcl_ret_t (*)(rcl_clock_t *, rcl_jump_threshold_t, rcl_jump_callback_t, void *);
  static Fn next = next_symbol<Fn>("rcl_clock_add_jump_callback");
  if (clock != nullptr && clock->type == RCL_ROS_TIME && !t_in_clock_gate && callback != nullptr) {
    InClockGate in;
    // Only rclcpp's trampoline can carry the handler sleep_until or
    // wait_until adds.
    const bool trampoline = reinterpret_cast<void *>(callback) == rclcpp_jump_trampoline();
    const int in_sleep =
      !trampoline ? 0 : t_sleep_depth > 0 ? 1 : t_cv_wait_depth > 0 ? 2 : 0;
    rcllf_clock_gate_on_jump_handler(
      clock, &threshold, reinterpret_cast<const void *>(callback), in_sleep);
  }
  return next(clock, threshold, callback, user_data);
}

rcl_ret_t rcl_clock_fini(rcl_clock_t * clock)
{
  using Fn = rcl_ret_t (*)(rcl_clock_t *);
  static Fn next = next_symbol<Fn>("rcl_clock_fini");
  if (clock != nullptr && !t_in_clock_gate) {
    InClockGate in;
    rcllf_clock_gate_on_clock_fini(clock);
  }
  return next(clock);
}

}  // extern "C"

namespace rclcpp
{

Time Clock::now() const
{
  using Fn = Time (*)(const Clock *);
  static Fn next = next_symbol<Fn>("_ZNK6rclcpp5Clock3nowEv");
  Time value = next(this);
  if (get_clock_type() == RCL_ROS_TIME && !t_in_clock_gate) {
    InClockGate in;
    rcllf_clock_gate_on_ros_read(
      const_cast<Clock *>(this)->get_clock_handle(), value.nanoseconds(),
      __builtin_return_address(0));
  }
  return value;
}

bool Clock::sleep_until(Time until, Context::SharedPtr context)
{
  // A member function takes `this` as its first argument in the Itanium ABI.
  using Fn = bool (*)(Clock *, Time, Context::SharedPtr);
  static Fn next = next_symbol<Fn>(
    "_ZN6rclcpp5Clock11sleep_untilENS_4TimeESt10shared_ptrINS_7ContextEE");
  if (get_clock_type() == RCL_ROS_TIME && !t_in_clock_gate) {
    InClockGate in;
    rcllf_clock_gate_on_sleep(get_clock_handle(), until.nanoseconds());
  }
  InSleep sleeping;
  return next(this, std::move(until), std::move(context));
}

bool ClockConditionalVariable::wait_until(
  std::unique_lock<std::mutex> & lock, Time until, const std::function<bool()> & pred)
{
  using Fn = bool (*)(
    ClockConditionalVariable *, std::unique_lock<std::mutex> &, Time,
    const std::function<bool()> &);
  static Fn next = next_symbol<Fn>(
    "_ZN6rclcpp24ClockConditionalVariable10wait_untilERSt11unique_lockISt5mutexENS_4TimeERKSt8"
    "functionIFbvEE");
  InCvWait waiting;
  return next(this, lock, std::move(until), pred);
}

}  // namespace rclcpp

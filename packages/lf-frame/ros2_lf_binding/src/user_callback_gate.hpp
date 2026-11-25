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

/// @file user_callback_gate.hpp
/// @brief No user callback runs once rclcpp shutdown has begun.
///
/// On SIGINT, rclcpp shuts the context down while LF reactions still run until
/// the LF stop takes effect. A user callback in that window ran against an
/// invalid context. With loaned messages that corrupts memory: rclcpp's
/// LoanedMessage sees can_loan_messages() == false once the context is
/// invalid, never returns the loan, and frees the middleware's sample with
/// the local allocator. Fast DDS then refuses to delete the writer and the
/// participant crashes in the factory's exit-time destructor.
///
/// A pre-shutdown callback (runtime.cpp) closes the gate and waits for the
/// callbacks already running; every dispatch site holds a UserCallbackScope
/// around the user callback and skips it when the gate is closed.
///
/// The outermost scope on a thread also runs the lifecycle boundary step of the
/// tag before the callback, and records the lifecycle transitions the callback
/// made at the current tag when it returns (lifecycle_gate.hpp).
/// Its token tells the callback that changed a LifecyclePublisher's flag
/// from the other reactions of the tag.

#ifndef USER_CALLBACK_GATE_HPP_
#define USER_CALLBACK_GATE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "lifecycle_gate.hpp"

namespace ros2_lf_binding
{

inline std::atomic<bool> g_user_callbacks_closed{false};
inline std::atomic<int> g_user_callbacks_in_flight{0};
inline std::atomic<uint64_t> g_user_callbacks_skipped{0};  ///< refused after close
inline thread_local int t_user_callback_depth = 0;
inline std::atomic<uint64_t> g_user_callback_count{0};
/// Names the outermost user callback running on this thread.
inline thread_local uint64_t t_user_callback_token = 0;

/// Admits one user callback unless the gate is closed. Sequentially
/// consistent: either close_user_callbacks() sees this scope in flight, or
/// this scope sees the gate closed.
class UserCallbackScope
{
public:
  UserCallbackScope()
  {
    g_user_callbacks_in_flight.fetch_add(1);
    if (g_user_callbacks_closed.load()) {
      g_user_callbacks_in_flight.fetch_sub(1);
      g_user_callbacks_skipped.fetch_add(1);
      admitted_ = false;
      return;
    }
    if (t_user_callback_depth == 0) {
      t_user_callback_token = g_user_callback_count.fetch_add(1) + 1;
      try {
        lifecycle_gate_boundary();
      } catch (...) {
        // A state probe that throws records nothing. The callback still runs.
      }
    }
    ++t_user_callback_depth;
  }
  ~UserCallbackScope()
  {
    if (admitted_) {
      if (t_user_callback_depth == 1) {
        try {
          lifecycle_gate_observe_all();
        } catch (...) {
          // A state probe that throws records nothing. The callback is done.
        }
      }
      --t_user_callback_depth;
      g_user_callbacks_in_flight.fetch_sub(1);
    }
  }
  UserCallbackScope(const UserCallbackScope &) = delete;
  UserCallbackScope & operator=(const UserCallbackScope &) = delete;

  bool admitted() const {return admitted_;}

private:
  bool admitted_ = true;
};

/// Close the gate and wait until no other thread runs a user callback.
/// A callback that itself triggers shutdown does not wait for itself.
/// Returns false when `bound` expired first.
inline bool close_user_callbacks(std::chrono::milliseconds bound)
{
  g_user_callbacks_closed.store(true);
  const int own = t_user_callback_depth;
  const auto deadline = std::chrono::steady_clock::now() + bound;
  while (g_user_callbacks_in_flight.load() > own) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

/// Test support: reopen the gate.
inline void reopen_user_callbacks_for_test()
{
  g_user_callbacks_closed.store(false);
}

}  // namespace ros2_lf_binding

#endif  // USER_CALLBACK_GATE_HPP_

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

#ifndef BRAKE_FUSION__BRAKE_ACTUATOR_HPP_
#define BRAKE_FUSION__BRAKE_ACTUATOR_HPP_

#include <cstddef>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>

namespace brake_fusion
{

// One received brake command, as the actuator queues it.
struct QueuedBrake
{
  double level{0.0};  // clamped to [0, 1] by the simulator
  uint64_t seq{0};  // BrakeCmd identity
  int64_t tick{-1};
  int64_t stamp_ns{0};  // BrakeCmd stamp_ns, the controller's decision time t
  int64_t n{0};  // 1-based receipt index at the simulator (brake_rx n)
  int64_t rx_plant_ns{0};  // plant time when it was received (latest state stamp)
  // Plant time of the last step that ran before it was received (-1: none
  // yet). The command is late when this is already strictly after t + d_b:
  // the step it had to act in ran before it arrived.
  int64_t rx_step_ns{-1};
};

// Modeled brake actuation latency d_b: a command stamped t
// acts on the plant from the first simulator step strictly after t + d_b.
// Commands are ordered and released by their stamp, never by their receipt
// time, so stock ROS 2 (which receives the command a few ms after t) and
// rcllf (which receives it at tag t, or at tag t + d_b once /brake_cmd
// carries the logical delay d_b) apply it in the same step whenever it
// arrives before that step runs. A command that arrives later acts at the
// first step after its receipt (it cannot act in the past); the simulator
// logs it as late.
//
// The rule, one for both arms: the
// step at plant time s (the stamp of the state it publishes; its tag in
// rcllf, now() in stock) applies a command when t + d_b < s. In rcllf the
// steps run on the 1 ms tag grid, so a command stamped t acts in the step at
// tag t + d_b + 1 ms. With `after d_b` on /brake_cmd the command reaches the
// timers_first simulator at tag t + d_b, after that tag's step, and acts in
// the step at t + d_b + 1 ms: the edge delay and this queue release at the
// same tag because the queue keys on the stamp t, so d_b is never counted
// twice (keyed on the arrival t + d_b it would act at t + 2 d_b + 1 ms).
// Under the old rule (interval start, the previous state stamp, at or after
// t + d_b) a stock step whose own time had passed t + d_b waited one more
// step; on the rcllf grid both rules pick the same step.
class BrakeActuator
{
public:
  explicit BrakeActuator(int64_t delay_ns)
  : delay_ns_(delay_ns)
  {
    if (delay_ns < 0) {
      throw std::invalid_argument("brake actuation delay must be >= 0");
    }
  }

  int64_t delay_ns() const {return delay_ns_;}

  // Queue a received command in stamp order, whatever its receipt order.
  // Returns false (and drops it) when its stamp is not newer than the command
  // already acting (applying it would move the brake back in time) or equals
  // the stamp of a queued command (a duplicate).
  bool push(const QueuedBrake & cmd)
  {
    if (have_acting_ && cmd.stamp_ns <= acting_stamp_ns_) {
      return false;
    }
    const auto at = std::lower_bound(
      queue_.begin(), queue_.end(), cmd.stamp_ns,
      [](const QueuedBrake & q, int64_t stamp) {return q.stamp_ns < stamp;});
    if (at != queue_.end() && at->stamp_ns == cmd.stamp_ns) {
      return false;
    }
    queue_.insert(at, cmd);
    return true;
  }

  // The command that acts in the step at plant time `step_ns`: every queued
  // command with t + d_b < step_ns (strictly) is released in stamp order, and
  // the newest of them acts (the older ones were overwritten before any step
  // used them). `released` counts them.
  std::optional<QueuedBrake> release(int64_t step_ns, int64_t * released = nullptr)
  {
    std::optional<QueuedBrake> acting;
    int64_t count = 0;
    while (!queue_.empty() && queue_.front().stamp_ns + delay_ns_ < step_ns) {
      acting = queue_.front();
      queue_.pop_front();
      ++count;
    }
    if (released != nullptr) {
      *released = count;
    }
    if (acting) {
      have_acting_ = true;
      acting_stamp_ns_ = acting->stamp_ns;
    }
    return acting;
  }

  std::size_t pending() const {return queue_.size();}

private:
  int64_t delay_ns_;
  std::deque<QueuedBrake> queue_;
  bool have_acting_{false};
  int64_t acting_stamp_ns_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__BRAKE_ACTUATOR_HPP_

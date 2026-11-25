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

#ifndef BRAKE_FUSION__GRID_HPP_
#define BRAKE_FUSION__GRID_HPP_

#include <cstdint>
#include <deque>
#include <stdexcept>

#include "nav_msgs/msg/odometry.hpp"
#include "rcl/timer.h"
#include "rclcpp/rclcpp.hpp"

namespace brake_fusion
{

// Grid sampling: every timer fires on the shared grid origin + k * period.
//
// Re-arms `timer` so that its next call is the first grid instant after
// `now_ns` (the origin itself while now is before it). Called once after
// create_timer and again at the start of every callback: the next call is
// always computed from the absolute grid, so neither the node's creation
// time nor a late callback shifts the phase, and a missed period is skipped
// rather than replayed. Returns the instant it armed for.
inline int64_t arm_on_grid(
  rclcpp::TimerBase & timer, int64_t now_ns, int64_t origin_ns, int64_t period_ns)
{
  int64_t next = origin_ns;
  if (now_ns >= origin_ns) {
    next = origin_ns + ((now_ns - origin_ns) / period_ns + 1) * period_ns;
  }
  int64_t old_period = 0;
  if (rcl_timer_exchange_period(timer.get_timer_handle().get(), next - now_ns, &old_period) !=
    RCL_RET_OK)
  {
    throw std::runtime_error("rcl_timer_exchange_period failed");
  }
  timer.reset();
  return next;
}

// Grid index of `t_ns`: the k with origin + k * period <= t < origin + (k+1) * period.
inline int64_t grid_floor(int64_t t_ns, int64_t origin_ns, int64_t period_ns)
{
  const int64_t since = t_ns - origin_ns;
  return since >= 0 ? since / period_ns : -((-since + period_ns - 1) / period_ns);
}

// Plant states received from the simulator, keyed by their step stamp, so a
// sensor reads the state for its own grid stamp instead of the latest one.
struct PlantSample
{
  int64_t stamp_ns{-1};
  double position{0.0};
  double velocity{0.0};
  int64_t rx_ns{-1};  // when the state message reached this node
};

class GridSampler
{
public:
  void add(const nav_msgs::msg::Odometry & msg, int64_t rx_ns)
  {
    PlantSample s;
    s.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    s.position = msg.pose.pose.position.x;
    s.velocity = msg.twist.twist.linear.x;
    s.rx_ns = rx_ns;
    buffer_.push_back(s);
    while (buffer_.size() > kKeep) {
      buffer_.pop_front();
    }
  }

  enum class Result {kWaiting, kFound, kLost};

  // Looks up the state stamped exactly `stamp_ns`. kLost once a newer state
  // arrived without it (the simulator publishes in stamp order, reliably).
  Result find(int64_t stamp_ns, PlantSample * out) const
  {
    for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it) {
      if (it->stamp_ns == stamp_ns) {
        *out = *it;
        return Result::kFound;
      }
    }
    if (!buffer_.empty() && buffer_.back().stamp_ns > stamp_ns) {
      return Result::kLost;
    }
    return Result::kWaiting;
  }

private:
  static constexpr size_t kKeep = 200;  // 200 ms of 1 ms plant steps
  std::deque<PlantSample> buffer_;
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__GRID_HPP_

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

#ifndef BRAKE_FUSION__SIM_STATE_HPP_
#define BRAKE_FUSION__SIM_STATE_HPP_

#include <cstddef>

#include "rclcpp/rclcpp.hpp"

namespace brake_fusion
{

/// Reader depth of the sensors' /sim/state subscription.
///
/// The simulator publishes one plant state per step (1 ms, the edge's
/// min_input_period_ms in config/r2). /sim/state is a legacy edge, so the
/// binding takes whatever the reader holds when the sensor's reaction runs,
/// and a reliable KeepLast reader that is full replaces its oldest sample
/// without a message_lost event. The reader therefore has to hold every state
/// published while the sensor's executor is away: the longest stall seen in
/// the pilots is about 45 ms (link delay 5 ms + take timeout 25 ms + the
/// 15 ms stall shim of the loaded depth-10 stall runs), and the depth 10 reader
/// dropped states in 10 of 10 of those runs while depth 100 dropped none in
/// 5 of 5.
/// 100 steps is that bound with a 2x margin. The literal in each sensor's
/// KeepLast() call is what the analyzer reads; a static_assert keeps the two
/// equal.
constexpr std::size_t kSimStateDepth = 100;

/// Startup check: warn when the middleware gave the /sim/state reader
/// less history than the bound needs.
inline void check_sim_state_depth(
  const rclcpp::SubscriptionBase & sub, const rclcpp::Logger & logger)
{
  const auto actual = sub.get_actual_qos();
  if (actual.history() != rclcpp::HistoryPolicy::KeepLast || actual.depth() < kSimStateDepth) {
    RCLCPP_WARN(
      logger, "/sim/state reader holds %zu sample(s); the bound needs KeepLast(%zu)"
      ": states may be replaced unseen when this executor stalls",
      actual.depth(), kSimStateDepth);
  }
}

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__SIM_STATE_HPP_

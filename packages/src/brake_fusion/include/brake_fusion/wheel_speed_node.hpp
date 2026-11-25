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

#ifndef BRAKE_FUSION__WHEEL_SPEED_NODE_HPP_
#define BRAKE_FUSION__WHEEL_SPEED_NODE_HPP_

#include <cstdint>
#include <memory>

#include "brake_fusion/grid.hpp"
#include "brake_fusion/jsonl_log.hpp"
#include "brake_fusion/stamper.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace brake_fusion
{

// A sensor node: keeps the latest plant state from /sim/state and publishes
// on its own wall timer, stamped by its Stamper (`stamp_mode`,
// `stamp_origin_ns`). A sensor publishes nothing before the first state has
// arrived; its tick count k still advances.

// Wheel speed: 10 ms, /wheel/speed twist.linear.x = velocity.
class WheelSpeedNode : public rclcpp::Node
{
public:
  explicit WheelSpeedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void state_callback(const nav_msgs::msg::Odometry & msg);
  void timer_callback();
  // The callbacks publish themselves: the analyzer attributes a publish to
  // the timer or subscription callback whose body holds the call, so a
  // helper that published would leave the topic's edge legacy.
  bool serve_pending(geometry_msgs::msg::TwistStamped & msg);
  // Fault injection: true from silent_from_tick_ on (logs the first one).
  bool silent(int64_t tick);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Stamper> stamper_;
  JsonlLog log_;
  bool have_state_{false};
  int64_t state_stamp_ns_{0};
  double velocity_{0.0};
  int64_t seq_{0};
  // Grid sampling (sample_mode grid): the grid tick waiting for its plant state.
  bool grid_{false};
  int64_t period_ns_{0};
  GridSampler sampler_;
  int64_t pending_tick_{-1};
  int64_t pending_stamp_ns_{0};
  int64_t release_ns_{0};
  // Fault injection: the first tick that publishes nothing, -1 never.
  int64_t silent_from_tick_{-1};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__WHEEL_SPEED_NODE_HPP_

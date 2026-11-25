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

#ifndef BRAKE_FUSION__CONTROLLER_NODE_HPP_
#define BRAKE_FUSION__CONTROLLER_NODE_HPP_

#include <cstdint>

#include "brake_fusion/jsonl_log.hpp"
#include "custom_interfaces/msg/brake_cmd.hpp"
#include "brake_fusion/publish_drop.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace brake_fusion
{

// Emergency braking controller with the TTC rule of aeb_controller.
//
// The subscriptions store the last two fused ranges and the latest wheel
// speed. Every 25 ms the timer estimates the closing speed from the last two
// fused ranges as (range difference) / (difference of their header stamps),
// computes TTC = latest fused range / closing speed and publishes a brake
// level: `partial_brake` below `ttc_partial`, full brake below `ttc_full`, a
// stage once reached held. The wheel speed is the fallback before two fused
// ranges exist and replaces an estimate that differs from it by more than
// `plausibility_tolerance` m/s. Only the timer publishes, once per call.
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void fused_callback(const sensor_msgs::msg::Range & msg);
  void wheel_callback(const geometry_msgs::msg::TwistStamped & msg);
  void control_callback();

  // Inputs of one control step and the decision taken from them.
  struct Step
  {
    bool have_fused{false};
    double range{0.0};
    int64_t stamp_ns{-1};
    bool have_prev{false};
    double prev_range{0.0};
    int64_t prev_stamp_ns{-1};
    bool have_wheel{false};
    double wheel_speed{0.0};
    int64_t wheel_stamp_ns{-1};
    bool have_estimate{false};
    double estimate{0.0};
    double closing_speed{0.0};
    const char * source{"none"};
    double ttc{-1.0};
    double brake{0.0};
    double raw_brake{0.0};  // this tick's decision before the hold
  };
  void decide(Step & s);
  custom_interfaces::msg::BrakeCmd make_cmd(int64_t tick, const Step & s);
  void log_brake_pub(const custom_interfaces::msg::BrakeCmd & cmd, const Step & s);
  void send(int64_t tick, const Step & s);

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr fused_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
  rclcpp::Publisher<custom_interfaces::msg::BrakeCmd>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  JsonlLog log_;

  double stop_margin_m_;
  int64_t brake_actuation_delay_ms_{0};  // d_b, ms, see the constructor
  double ttc_full_;
  double ttc_partial_;
  double partial_brake_;
  bool partial_stage_{true};
  double plausibility_tolerance_;

  // Last two fused ranges; index 1 is the latest. Stamps in ns, -1 = none.
  double fused_range_[2]{0.0, 0.0};
  int64_t fused_stamp_ns_[2]{-1, -1};
  int64_t fused_count_{0};
  double wheel_speed_{0.0};
  int64_t wheel_stamp_ns_{-1};
  int64_t wheel_count_{0};
  // The stock /wheel/speed startup timeout (wheel_startup_check).
  int64_t wheel_wait_start_ns_{-1};  // node time of the first tick, -1 = none yet
  int64_t wheel_startup_windows_{0};  // kWheelDeadline windows reported so far
  void wheel_startup_check();
  bool log_inputs_{false};
  double held_brake_{0.0};
  int64_t tick_{0};
  int64_t brake_pubs_{0};  // published brake commands
  uint64_t cmd_seq_{0};  // decided brake commands, drops included: BrakeCmd.seq
  // seq of the first command of the current run of one published level whose
  // own decision was that level: the command a held level comes from.
  uint64_t level_seq_{0};
  double level_{-1.0};  // the level of level_seq_ (-1: none yet, levels are >= 0)
  // `drop_first_brake_ge` (>= 0): the one brake command not sent.
  PublishDrop drop_{-1, -1.0};
  bool dropped(const custom_interfaces::msg::BrakeCmd & cmd);
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__CONTROLLER_NODE_HPP_

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

#ifndef BRAKE_FUSION__SIMULATOR_NODE_HPP_
#define BRAKE_FUSION__SIMULATOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "brake_fusion/brake_actuator.hpp"
#include "brake_fusion/jsonl_log.hpp"
#include "custom_interfaces/msg/brake_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcllf_plant/transactor.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace brake_fusion
{

// The plant: a 1D vehicle (MuJoCo) driving toward a static obstacle, with the
// vehicle parameters of aeb_sim (40 m, 15 m/s, 8 m/s^2).
//
// Every 1 ms tick integrates exactly one fixed 1 ms step with the latched
// brake command and publishes the true state on /sim/state
// (pose.position.x = position, twist.linear.x = velocity). The sensors sample
// this state on their own timers. The stopping position depends only on the
// step at which each brake command arrives.
//
// With `publish_clock` the simulator also publishes /clock = step * 1 ms after
// every step, for the use_sim_time configuration.
//
// With `real_time_factor` f (default 1.0) the tick period is 1 ms / f of system
// time instead of 1 ms, so simulation time (/clock) runs f times faster than
// real time; f = 0 runs the ticks back to back. Ticks the executor misses are
// skipped, not replayed, so the achieved rate can stay below f.
//
// Each step is one request to the rcllf_plant Transactor with the MuJoCo
// backend: one mj_step per request, a plant_step log
// line with the state and output hashes, and a plant that times out or
// mismatches ends the process with exit code 3 (plant_timeout / plant_fault).
// `plant_step_timeout_ms` (1000) and `plant_seed` (0) are logged. `plant` (default:
// $BF_PLANT, else "mujoco") accepts only "mujoco": the kinematic update was
// deleted and any other value throws at startup.
//
// With `brake_actuation_delay_ms` d_b > 0 a brake command
// stamped t acts from the first step strictly after t + d_b (BrakeActuator),
// queued by its stamp,
// and every acting command logs a brake_act record. d_b = 0 (default) keeps the
// receipt latch: a command acts from the step after its receipt, as before.
class SimulatorNode : public rclcpp::Node
{
public:
  explicit SimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Plant steps run so far, and timer ticks after start that found
  // step < max_steps. The two are equal by construction: one tick,
  // one step, no skip and no catch-up.
  int64_t steps() const {return step_;}
  int64_t step_ticks() const {return step_ticks_;}

private:
  void step_callback();
  void integrate_step();
  void after_step();
  rosgraph_msgs::msg::Clock clock_msg() const;
  nav_msgs::msg::Odometry state_msg(int64_t stamp_ns);
  void brake_callback(const custom_interfaces::msg::BrakeCmd & msg);
  void release_delayed_brake();
  [[noreturn]] void plant_failed(const rcllf_plant::CompletionRecord & done, int64_t step);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<custom_interfaces::msg::BrakeCmd>::SharedPtr brake_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  JsonlLog log_;
  std::unique_ptr<rcllf_plant::Transactor> transactor_;
  BrakeActuator actuator_{0};

  double obstacle_distance_;
  double max_decel_;
  bool publish_clock_;
  double real_time_factor_{1.0};
  int64_t expected_state_subscribers_;
  int64_t start_at_ns_{0};
  int64_t max_steps_;
  std::string plant_;

  double position_{0.0};
  double velocity_;
  double brake_{0.0};
  double applied_brake_{0.0};
  int64_t brake_msgs_{0};
  // receipt index n of the command that acts now: brake_msgs_ under the
  // receipt latch (d_b = 0), the released command's n under a delay
  int64_t acting_msgs_{0};
  int64_t taken_msgs_{0};  // acting_msgs_ at the last brake_take record
  // stamp of the last published state, the start of the next step's plant
  // interval; -1 before the first step
  int64_t last_state_stamp_ns_{-1};
  // Plant time of the last step run with d_b > 0 (-1: none yet), the step
  // time BrakeActuator::release compared.
  int64_t last_step_ns_{-1};
  // identity of the last received command (BrakeCmd seq, tick), for brake_take
  uint64_t brake_seq_{0};
  int64_t brake_tick_{-1};
  int64_t step_{0};
  int64_t step_ticks_{0};  // ticks after start below the horizon
  bool started_{false};
  // Ticks the start gate held at step 0 (discovery not done). An rcllf run
  // must log 0: its binding refuses to start before every endpoint matched
  // at startup, so a nonzero count means the start followed discovery.
  int64_t held_ticks_{0};
  bool stopped_{false};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__SIMULATOR_NODE_HPP_

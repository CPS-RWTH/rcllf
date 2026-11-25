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

#include "brake_fusion/simulator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <rclcpp_components/register_node_macro.hpp>

#include "brake_fusion/loss_events.hpp"
#include "rcllf_plant/mujoco_backend.hpp"

namespace brake_fusion
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{
// Integration step. Equals the timer period below, which stays a literal
// because the rcllf analyzer reads timer periods from the source.
constexpr double kStepSeconds = 0.001;
constexpr int64_t kStepNs = 1'000'000;
// One episode per process: the simulator never resets after construction.
constexpr uint64_t kEpisode = 1;

// System-time period of one timer tick at `real_time_factor`: 1 ms / factor,
// and 0 (a timer that is always ready) for factor 0.
int64_t tick_period_ns(double real_time_factor)
{
  if (!std::isfinite(real_time_factor) || real_time_factor < 0.0) {
    throw std::invalid_argument("real_time_factor must be finite and >= 0");
  }
  return real_time_factor == 0.0 ? 0 : std::llround(kStepNs / real_time_factor);
}
}  // namespace

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions & options)
: Node("simulator", options)
{
  obstacle_distance_ = this->declare_parameter<double>("obstacle_distance", 40.0);
  velocity_ = this->declare_parameter<double>("initial_velocity", 15.0);
  max_decel_ = this->declare_parameter<double>("max_decel", 8.0);
  publish_clock_ = this->declare_parameter<bool>("publish_clock", false);
  // Completion-aware execution: one timer
  // activation is exactly one 1 ms plant step and at most one /sim/state, so
  // the analyzer proves the /sim/state burst bound 1 and the four state links
  // are admitted. The modes that broke this (a "fast" clock, steps_per_tick
  // > 1, and a grid with a wall-clock catch-up loop) are refused here.
  if (this->declare_parameter<int64_t>("steps_per_tick", 1) != 1) {
    throw std::invalid_argument("steps_per_tick must be 1 (one plant step per activation)");
  }
  // Hold at step 0 until this many nodes subscribe to /sim/state and a brake
  // publisher is connected, so a stock run does not integrate while discovery
  // is still running. The count is the number of /sim/state
  // readers in the topology, not a tuning value: 3 here (radar, camera, wheel
  // speed), 4 in the t2 and r2 launches, which add radar2. A smaller count
  // lets the plant move before one sensor sees it, so the first samples of
  // that sensor, and with them the first brake decision, come late.
  expected_state_subscribers_ = this->declare_parameter<int64_t>("expected_state_subscribers", 3);
  // Fixed horizon: stop integrating after this many 1 ms steps
  // (20 s). The basis is the stopping objective: in the default scenario
  // (X = 0.45 m) the ideal-timing oracle stops at step 3743 (3773 at the earlier
  // threshold 1.095 s, matched by every one-process rcllf run), and the
  // latest stop of all 64 runs of the stop-margin study is step 4936, a
  // stock two-host run whose full stage never fired. 20000 is
  // four times that and still ends inside a 30 s run. A run that
  // reaches it without a vehicle stop logs "horizon", which a summary
  // reports as its own field, never as a stop.
  max_steps_ = this->declare_parameter<int64_t>("max_steps", 20000);
  // Also hold until the node clock reaches this time (0: no wait). A launch
  // can pass the shared start time so plant step 0 lies at the stamp origin.
  start_at_ns_ = this->declare_parameter<int64_t>("start_at_ns", 0);
  if (this->declare_parameter<int64_t>("grid_origin_ns", 0) != 0) {
    throw std::invalid_argument("grid_origin_ns is not supported (no wall-clock catch-up)");
  }
  // Pacing (stock simulation time): one tick every 1 ms / factor of
  // system time, so factor 4 runs four times faster than real time and 0 runs
  // the ticks back to back. 1.0 keeps the 1 ms timer exactly as before.
  real_time_factor_ = this->declare_parameter<double>("real_time_factor", 1.0);
  // Brake actuation latency d_b in ms: a command stamped t
  // acts in the first plant step strictly after t + d_b (BrakeActuator). It models the
  // physical delay from the brake command to the braking force, and it is the
  // logical delay the tag-anchored maxwait design gives the /brake_cmd
  // feedback edge. Whole ms: the plant step is 1 ms, so a finer value would
  // only round to the step. 0 (default) keeps the receipt latch, today's
  // behaviour; candidates 25 and 100 ms (from the Orin stage
  // budgets measured on the Orins). The controller takes the same value for its full-stage
  // threshold (stop_margin.hpp), so every launch passes one argument to both.
  const int64_t delay_ms = this->declare_parameter<int64_t>("brake_actuation_delay_ms", 0);
  if (delay_ms < 0) {
    throw std::invalid_argument("brake_actuation_delay_ms must be >= 0");
  }
  actuator_ = BrakeActuator(delay_ms * kStepNs);
  const int64_t tick_ns = tick_period_ns(real_time_factor_);
  // Plant model: MuJoCo only. The kinematic update was
  // deleted: every run uses MuJoCo, so any other value of
  // `plant` or $BF_PLANT is refused at startup instead of silently running a
  // different plant.
  const char * plant_env = std::getenv("BF_PLANT");
  plant_ = this->declare_parameter<std::string>(
    "plant", plant_env != nullptr && plant_env[0] != '\0' ? plant_env : "mujoco");
  if (plant_ != "mujoco") {
    throw std::invalid_argument(
            "plant must be mujoco (the kinematic plant was removed), got " +
            plant_);
  }
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  // The plant sits behind the rcllf_plant Transactor: one
  // dedicated plant thread, one request per 1 ms step, the backend's step
  // counter and clock checked against the step id, a supervised timeout.
  // plant_step_timeout_ms, MuJoCo basis: one mj_step of this one-joint model
  // takes microseconds (plant_step wall_ns in the log), and the worst
  // scheduling stall measured on the Orin pair is 36.8 ms (start lag), so
  // 1000 ms only fires on a plant that is stuck, never on a slow step.
  const int64_t timeout_ms = this->declare_parameter<int64_t>("plant_step_timeout_ms", 1000);
  if (timeout_ms <= 0) {
    throw std::invalid_argument("plant_step_timeout_ms must be > 0");
  }
  // MuJoCo draws no random numbers in this model; the seed is required by
  // the Transactor and logged, nothing reads it.
  const int64_t seed = this->declare_parameter<int64_t>("plant_seed", 0);
  transactor_ = std::make_unique<rcllf_plant::Transactor>(
    std::make_unique<rcllf_plant::MujocoBackend>(max_decel_, kStepNs),
    std::chrono::milliseconds(timeout_ms));
  rcllf_plant::ResetRequest reset;
  reset.episode = kEpisode;
  reset.seed = static_cast<uint64_t>(seed);
  reset.step_ns = kStepNs;
  reset.initial_state = {position_, velocity_};
  const rcllf_plant::CompletionRecord r0 = transactor_->reset(reset);
  const rcllf_plant::Descriptor & plant = transactor_->descriptor();
  log_.line(
    "\"ev\":\"plant\",\"plant\":\"%s\",\"mujoco_version\":\"%s\",\"timestep\":%.17g,"
    "\"episode\":%" PRIu64 ",\"seed\":%" PRId64 ",\"plant_step_timeout_ms\":%" PRId64
    ",\"fault\":\"%s\",\"state_hash\":\"%016" PRIx64 "\",\"status\":\"%s\"",
    plant.backend.c_str(), plant.version.c_str(), kStepSeconds, kEpisode, seed, timeout_ms,
    transactor_->fault_spec(), r0.state_hash, rcllf_plant::to_string(r0.status));
  if (r0.status != rcllf_plant::Status::OK) {
    plant_failed(r0, 0);
  }
  RCLCPP_INFO(this->get_logger(), "BF_PLANT %s %s", plant.backend.c_str(), plant.version.c_str());

  // /sim/state and /clock writer depth 10: GUESSED. Both are
  // published once per 1 ms step (step_callback), so ten samples cover 10 ms,
  // short of the 90 ms the rule (loss_events.hpp:82) asks for; a lost
  // /clock sample is superseded by the next step's. The rule leaves
  // /sim/state out; its readers keep
  // kSimStateDepth = 100 (sim_state.hpp). A writer depth of 100 did not stop
  // the 9 or 10 consecutive /sim/state losses that some MuJoCo runs report
  // in the first 100 ms after the plant starts (depth 10: 2 of 6 runs,
  // depth 100: 1 of 18),
  // so its value is not what decides them. What would settle it: log each
  // reader's subscription-matched time next to its first loss, to tell a
  // late-matching reader (discovery) from a history bound.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sim/state", qos);
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
  // QoS deadline and deadline / message_lost handlers on /brake_cmd.
  // Best effort, as offered by the controller: a design choice with explicit
  // QoS. Depth 10, DERIVED (rule in loss_events.hpp:82): ten 25 ms commands cover 250 ms, more than
  // kBrakeDeadline 37.5 ms and the largest deadline, 90 ms; it also holds
  // this edge's rcllf burst B = 1. Measured: at most 1 command in
  // flight in stock runs.
  auto brake_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kBrakeDeadline));
  rclcpp::SubscriptionOptions brake_options = loss_options(log_, this->get_clock(), "/brake_cmd");
  brake_sub_ = this->create_subscription<custom_interfaces::msg::BrakeCmd>(
    "/brake_cmd", brake_qos, std::bind(&SimulatorNode::brake_callback, this, std::placeholders::_1),
    brake_options);
  // Node clock, like every other node. The launch files never give the
  // simulator use_sim_time: it is the /clock source, so it runs on system time.
  // 1 ms step: at v0 = 15 m/s the plant moves at most 0.015 m per
  // step, far below every candidate stopping margin X (stop_margin.hpp),
  // and the 1 ms grid divides every sensor and controller period.
  timer_ = this->create_timer(1ms, std::bind(&SimulatorNode::step_callback, this));
  if (real_time_factor_ != 1.0) {
    int64_t old_period = 0;
    if (rcl_timer_exchange_period(timer_->get_timer_handle().get(), tick_ns, &old_period) !=
      RCL_RET_OK)
    {
      throw std::runtime_error("rcl_timer_exchange_period failed");
    }
    timer_->reset();
    // Log the period the timer holds, read back from rcl, not the
    // requested tick_ns, so the pace line proves the factor was installed.
    // Readers of tick_period_ns: test_simulator_pace.cpp and the pace analysis
    // of the development tree.
    int64_t installed_ns = -1;
    if (rcl_timer_get_period(timer_->get_timer_handle().get(), &installed_ns) != RCL_RET_OK) {
      throw std::runtime_error("rcl_timer_get_period failed");
    }
    if (installed_ns != tick_ns) {
      throw std::runtime_error(
              "simulator timer period is " + std::to_string(installed_ns) + " ns, requested " +
              std::to_string(tick_ns) + " ns for real_time_factor " +
              std::to_string(real_time_factor_));
    }
    log_.line(
      "\"ev\":\"pace\",\"real_time_factor\":%.17g,\"tick_period_ns\":%" PRId64,
      real_time_factor_, installed_ns);
  }
}

void SimulatorNode::brake_callback(const custom_interfaces::msg::BrakeCmd & msg)
{
  const double level = std::clamp(msg.level, 0.0, 1.0);
  ++brake_msgs_;
  if (actuator_.delay_ns() == 0) {
    // Receipt latch: the command acts from the next step.
    brake_ = level;
    brake_seq_ = msg.seq;
    brake_tick_ = msg.tick;
    acting_msgs_ = brake_msgs_;
  } else {
    const bool queued = actuator_.push(
      {level, msg.seq, msg.tick, msg.stamp_ns, brake_msgs_, last_state_stamp_ns_,
        last_step_ns_});
    if (!queued) {
      // Not newer than the acting command, or a duplicate stamp
      // (BrakeActuator::push).
      log_.line(
        "\"ev\":\"brake_stale\",\"n\":%" PRId64 ",\"seq\":%" PRIu64
        ",\"cmd_stamp_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
        brake_msgs_, msg.seq, msg.stamp_ns, this->now().nanoseconds());
    }
  }
  // Logged in every mode (was grid
  // only), the receipt endpoint of the end-to-end latency analysis and of the
  // /brake/command arrival gaps; n counts the commands received, seq and tick
  // are the command's identity (BrakeCmd), cmd_stamp_ns its decision time.
  log_.line(
    "\"ev\":\"brake_rx\",\"n\":%" PRId64 ",\"seq\":%" PRIu64 ",\"tick\":%" PRId64
    ",\"level\":%.17g,\"cmd_stamp_ns\":%" PRId64 ",\"next_step\":%" PRId64
    ",\"t_ns\":%" PRId64,
    brake_msgs_, msg.seq, msg.tick, level, msg.stamp_ns, step_ + 1,
    this->now().nanoseconds());
}

rosgraph_msgs::msg::Clock SimulatorNode::clock_msg() const
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = rclcpp::Time(step_ * kStepNs, RCL_ROS_TIME);
  return clock;
}

nav_msgs::msg::Odometry SimulatorNode::state_msg(int64_t stamp_ns)
{
  nav_msgs::msg::Odometry state;
  // stamp_ns < 0: as the /clock source, stamp the step time it just
  // published; otherwise now().
  if (stamp_ns < 0 && publish_clock_) {
    stamp_ns = step_ * kStepNs;
  }
  state.header.stamp = stamp_ns < 0 ? this->now() : rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  last_state_stamp_ns_ = rclcpp::Time(state.header.stamp).nanoseconds();
  state.header.frame_id = "world";
  state.child_frame_id = "vehicle";
  state.pose.pose.position.x = position_;
  state.twist.twist.linear.x = velocity_;
  return state;
}

void SimulatorNode::step_callback()
{
  if (!started_) {
    if (static_cast<int64_t>(state_pub_->get_subscription_count()) <
      expected_state_subscribers_ || brake_sub_->get_publisher_count() == 0 ||
      this->now().nanoseconds() < start_at_ns_)
    {
      ++held_ticks_;
      return;
    }
    started_ = true;
    log_.line(
      "\"ev\":\"start\",\"t_ns\":%" PRId64 ",\"held_ticks\":%" PRId64
      ",\"position\":%.17g,\"velocity\":%.17g",
      this->now().nanoseconds(), held_ticks_, position_, velocity_);
  }
  if (step_ >= max_steps_) {
    return;
  }
  // Every tick after start, before the horizon, must run exactly
  // one plant step, so this count equals step_ (test_simulator_pace.cpp
  // asserts it). A tick that returned here without a step would leave the
  // installed period right and the pace about f / 2 without it.
  ++step_ticks_;
  // One plant step, one state publication, no loop and no catch-up: a tick
  // the executor misses is skipped, never replayed. The latched brake level
  // applies from this step on (integrate_step). Published here: the rcllf
  // analyzer takes a timer callback's outputs from the publish calls in its
  // own body.
  integrate_step();
  if (publish_clock_) {
    clock_pub_->publish(clock_msg());
  }
  state_pub_->publish(state_msg(-1));
  after_step();
}

// Release the queued commands this step acts on. The step's plant time
// is the stamp of the state it publishes: step_ + 1 times 1 ms as the /clock
// source (clock_msg after ++step_), else now() (the tag in rcllf, system or
// simulation time in stock). A command stamped t acts in the first step whose
// plant time is strictly after t + d_b (BrakeActuator), the same rule in both
// arms.
void SimulatorNode::release_delayed_brake()
{
  const int64_t step_ns = publish_clock_ ? (step_ + 1) * kStepNs : this->now().nanoseconds();
  const int64_t prev_step_ns = last_step_ns_;
  last_step_ns_ = step_ns;
  int64_t released = 0;
  const std::optional<QueuedBrake> cmd = actuator_.release(step_ns, &released);
  if (!cmd) {
    return;
  }
  brake_ = cmd->level;
  brake_seq_ = cmd->seq;
  brake_tick_ = cmd->tick;
  acting_msgs_ = cmd->n;
  // act_ns - cmd_stamp_ns - delay_ns is in (0, 1 ms] for an on-time command
  // (exactly 1 ms under rcllf), and prev_step_ns <= cmd_stamp_ns + delay_ns:
  // no earlier step was strictly after t + d_b. late: a step strictly after
  // t + d_b ran before the command arrived, so it acts at the first step
  // after receipt instead.
  const int64_t due = cmd->stamp_ns + actuator_.delay_ns();
  log_.line(
    "\"ev\":\"brake_act\",\"n\":%" PRId64 ",\"seq\":%" PRIu64 ",\"tick\":%" PRId64
    ",\"level\":%.17g,\"cmd_stamp_ns\":%" PRId64 ",\"delay_ns\":%" PRId64 ",\"act_ns\":%" PRId64
    ",\"prev_step_ns\":%" PRId64 ",\"rx_plant_ns\":%" PRId64 ",\"late\":%s,\"released\":%" PRId64
    ",\"step\":%" PRId64 ",\"t_ns\":%" PRId64,
    cmd->n, cmd->seq, cmd->tick, cmd->level, cmd->stamp_ns, actuator_.delay_ns(), step_ns,
    prev_step_ns, cmd->rx_plant_ns, cmd->rx_step_ns > due ? "true" : "false", released, step_ + 1,
    this->now().nanoseconds());
}

// One 1 ms plant step. The caller publishes the new state, then calls
// after_step().
void SimulatorNode::integrate_step()
{
  if (actuator_.delay_ns() > 0) {
    release_delayed_brake();
  }
  // The first step that integrates with a newly received command, the
  // "brake applied" endpoint (brake_applied below logs level changes only,
  // and the held brake repeats levels). A command overwritten before any
  // step used it has a brake_rx record and no brake_take record.
  if (acting_msgs_ != taken_msgs_) {
    taken_msgs_ = acting_msgs_;
    log_.line(
      "\"ev\":\"brake_take\",\"n\":%" PRId64 ",\"seq\":%" PRIu64 ",\"tick\":%" PRId64
      ",\"level\":%.17g,\"step\":%" PRId64 ",\"t_ns\":%" PRId64,
      acting_msgs_, brake_seq_, brake_tick_, brake_, step_ + 1, this->now().nanoseconds());
  }
  if (brake_ != applied_brake_) {
    applied_brake_ = brake_;
    log_.line(
      "\"ev\":\"brake_applied\",\"step\":%" PRId64 ",\"level\":%.17g,\"brake_msgs\":%" PRId64
      ",\"position\":%.17g,\"velocity\":%.17g,\"t_ns\":%" PRId64,
      step_ + 1, applied_brake_, brake_msgs_, position_, velocity_, this->now().nanoseconds());
  }
  rcllf_plant::StepRequest request;
  request.episode = kEpisode;
  request.step_id = static_cast<uint64_t>(step_ + 1);
  request.logical_ns = (step_ + 1) * kStepNs;
  request.step_ns = kStepNs;
  request.inputs = {{"brake", {applied_brake_}}};
  const rcllf_plant::StepResult result = transactor_->step(request);
  const rcllf_plant::CompletionRecord & done = result.done;
  if (done.status != rcllf_plant::Status::OK) {
    plant_failed(done, step_ + 1);
  }
  log_.line(
    "\"ev\":\"plant_step\",\"step\":%" PRId64 ",\"state_hash\":\"%016" PRIx64
    "\",\"output_hash\":\"%016" PRIx64 "\",\"backend_steps\":%" PRIu64
    ",\"clock_drift_ns\":%" PRId64 ",\"plant_wall_ns\":%" PRId64,
    step_ + 1, done.state_hash, done.output_hash, done.backend_steps, done.clock_drift_ns,
    done.wall_ns);
  const double prev_position = position_;
  position_ = result.state.values.at(0);
  velocity_ = result.state.values.at(1);
  ++step_;
  // A collision is logged in the step the vehicle first reaches
  // the obstacle, whether or not it ever stops. Before, the only record was
  // the stop event's collision flag, so a run that never braked (a silent
  // /wheel/speed, BF_WHEEL_SILENT_FROM_MS=0) drove through the obstacle and
  // its log said nothing. The position never decreases (the velocity locks at
  // 0 below kLockSpeed, rcllf_plant mujoco_backend.cpp), so the crossing
  // happens once and the event is logged once.
  if (prev_position < obstacle_distance_ && position_ >= obstacle_distance_) {
    log_.line(
      "\"ev\":\"collision\",\"step\":%" PRId64 ",\"position\":%.17g,\"range\":%.17g,"
      "\"velocity\":%.17g,\"t_ns\":%" PRId64,
      step_, position_, obstacle_distance_ - position_, velocity_, this->now().nanoseconds());
  }
}

// Fail closed: no state of this step or any later one is
// published. quick_exit, not exit: a destructor would join the plant thread,
// which may be stuck inside the backend. The log is line buffered, so every
// line is already written.
void SimulatorNode::plant_failed(const rcllf_plant::CompletionRecord & done, int64_t step)
{
  const bool timeout = done.status == rcllf_plant::Status::TIMEOUT;
  log_.line(
    "\"ev\":\"%s\",\"step\":%" PRId64 ",\"status\":\"%s\",\"detail\":\"%s\","
    "\"timeout_ms\":%" PRId64 ",\"t_ns\":%" PRId64,
    timeout ? "plant_timeout" : "plant_fault", step, rcllf_plant::to_string(done.status),
    done.detail.c_str(), static_cast<int64_t>(transactor_->step_timeout().count()),
    this->now().nanoseconds());
  RCLCPP_FATAL(
    this->get_logger(), "BF_PLANT %s at step %" PRId64 ": %s", rcllf_plant::to_string(done.status),
    step, done.detail.c_str());
  std::fflush(nullptr);
  std::quick_exit(3);
}

// Stop and horizon events of the step just published.
void SimulatorNode::after_step()
{
  if (!stopped_ && velocity_ == 0.0) {
    stopped_ = true;
    log_.line(
      "\"ev\":\"stop\",\"step\":%" PRId64 ",\"position\":%.17g,\"range\":%.17g,"
      "\"collision\":%s,\"t_ns\":%" PRId64,
      step_, position_, obstacle_distance_ - position_,
      position_ >= obstacle_distance_ ? "true" : "false", this->now().nanoseconds());
    RCLCPP_INFO(
      this->get_logger(), "BF_STOP position=%.17g step=%" PRId64, position_, step_);
  }
  if (step_ >= max_steps_) {
    log_.line(
      "\"ev\":\"horizon\",\"step\":%" PRId64 ",\"position\":%.17g,\"velocity\":%.17g,"
      "\"t_ns\":%" PRId64,
      step_, position_, velocity_, this->now().nanoseconds());
  }
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::SimulatorNode)

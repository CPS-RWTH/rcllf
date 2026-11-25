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

#include "brake_fusion/controller_node.hpp"

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

#include "brake_fusion/closing_speed.hpp"
#include "brake_fusion/loss_events.hpp"
#include "brake_fusion/stop_margin.hpp"

namespace brake_fusion
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  // Thresholds, for the scenario of simulator_node.cpp:
  // v0 15 m/s, 40 m to the obstacle, full brake 8 m/s^2.
  // |estimate - wheel| above the plausibility tolerance falls back to the
  // wheel speed. Nominal deviation (oracle, t2approx): 0.08 m/s at constant
  // speed, 0.18 m/s under the partial brake (the full decision is still open
  // there), 0.44 m/s under the full brake (the estimate lags a 40 ms fused
  // age). 0.5 is above all of them, so a nominal run never falls back after
  // startup; the startup estimate (-4.9 m/s, the tracker's first pairs) and
  // one lost radar sample are caught. 0.44 or less: spurious fallbacks while
  // braking. 4.9 or more: the startup estimate is used. It also bounds the
  // speed under-read e_v of the full-stage threshold below.
  plausibility_tolerance_ = this->declare_parameter<double>("plausibility_tolerance", 0.5);
  // Stopping margin X in metres: the car stops at least X before the
  // obstacle, also after one lost message.
  // The value lives once, kStopMargin in stop_margin.hpp (0.45 m). A negative
  // value, the launch files' default, selects it, so a launch never carries
  // its own copy of X; stop_margin_m:=0.6 overrides it.
  stop_margin_m_ = this->declare_parameter<double>("stop_margin_m", kStopMargin);
  if (stop_margin_m_ < 0.0) {
    stop_margin_m_ = kStopMargin;
  }
  // The full stage from X: full_stage_ttc in stop_margin.hpp, 1.141 s at
  // X = 0.45 m. Below 1.110 s (X = 0) the full stage alone cannot keep the
  // worst case with one lost message short of the obstacle; a larger value
  // brakes fully for TTCs that need no full brake.
  // Brake actuation delay d_b in ms: the simulator's
  // parameter of the same name, passed by the launch files to both nodes. The
  // full stage acts d_b later, so its threshold uses L + d_b (stop_margin.hpp):
  // 1.166 s at 25 ms, 1.244 s at 100 ms. 0 (default) is today's 1.141 s.
  brake_actuation_delay_ms_ = this->declare_parameter<int64_t>("brake_actuation_delay_ms", 0);
  if (brake_actuation_delay_ms_ < 0) {
    throw std::invalid_argument("brake_actuation_delay_ms must be >= 0");
  }
  ttc_full_ = full_stage_ttc(
    stop_margin_m_, plausibility_tolerance_, static_cast<double>(brake_actuation_delay_ms_) * 1e-3);
  // The partial stage starts once the closing estimate is settled: valid
  // from tick 5 (125 ms, two tracked pairs), crossing two pair intervals
  // (80 ms) later gives TTC <= (40 - 15 * 0.205) / 15 = 2.46 s. At 2.6 s it
  // fires on the first valid estimate (a startup artefact) and the full
  // stage never fires.
  ttc_partial_ = this->declare_parameter<double>("ttc_partial", 2.45);
  // The partial stage alone, held from the ttc_partial crossing at the
  // no-loss latency L = 77 ms (two hosts, kLatencyBound without the 36 ms a
  // lost camera sample adds at the worst tick phase), stops at the obstacle with p a = v0 /
  // (2 (ttc_partial - L)) = 3.161 m/s^2, p = 0.395, rounded down to 0.39
  // (0.393 at the one program's 67 ms, 0.401 at the 113 ms of one loss, same
  // floor). It then ends within 0.2 m of the obstacle (oracle: 39.815 m), so
  // the TTC falls below ttc_full on the way and the full stage still fires
  // (2902 ms at X = 0.45 m, one program, ideal-timing oracle). At 0.42 the partial
  // stage alone stops 2.76 m short and the full stage never fires. This holds
  // when the partial stage starts on a checked estimate, which
  // select_closing_speed (closing_speed.hpp) enforces: before it, a stock run
  // that decided before its first wheel sample started the partial stage
  // about 120 ms early on the raw stamp estimate and stopped on it alone,
  // 2.03 m short.
  partial_brake_ = this->declare_parameter<double>("partial_brake", 0.39);
  // false: the full stage alone, the worst case the full-stage threshold is
  // derived for, run for real by t2_approx_fullonly.launch.py (a review
  // finding). Default true, the system as designed.
  partial_stage_ = this->declare_parameter<bool>("partial_stage", true);
  // Log-only: one line per fused and wheel input callback (order studies).
  log_inputs_ = this->declare_parameter<bool>("log_inputs", false);
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  log_.line(
    "\"ev\":\"config\",\"stop_margin_m\":%.17g,\"ttc_full\":%.17g,\"ttc_partial\":%.17g"
    ",\"partial_brake\":%.17g,\"plausibility_tolerance\":%.17g,\"partial_stage\":%d",
    stop_margin_m_, ttc_full_, ttc_partial_, partial_brake_, plausibility_tolerance_,
    partial_stage_ ? 1 : 0);
  if (brake_actuation_delay_ms_ != 0) {  // a separate record: the d_b = 0 log stays as before
    log_.line(
      "\"ev\":\"actuation\",\"brake_actuation_delay_ms\":%" PRId64 ",\"ttc_full\":%.17g",
      brake_actuation_delay_ms_, ttc_full_);
  }
  drop_ = PublishDrop(-1, this->declare_parameter<double>("drop_first_brake_ge", -1.0));

  // Every KeepLast(10) in this node: depth 10 DERIVED (rule in
  // loss_events.hpp:82): ten samples cover 100 ms of /wheel/speed (10 ms),
  // at least 200 ms of /fused/range (20 to 40 ms) and 250 ms of /brake_cmd
  // (25 ms), each more than the largest deadline, 90 ms. The /fused/range
  // burst B = 9 does not rest on it: the binding deepens an admitted reader
  // to max(8, B) when no reader_depth is declared, as in every brake_fusion
  // config (lf_generator.py _reader_depth). Reliable inputs
  // are a design choice.
  // QoS deadline and deadline / message_lost handlers on /fused/range.
  // The requested deadline must not be shorter than the fusion's offered one
  // (incompatible QoS, no data), so it follows the fusion policy; the default
  // is the longest offer, compatible with every fusion node.
  const std::string fusion = this->declare_parameter<std::string>("fusion", "exact");
  auto fused_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(fusion == "approximate" ? kFusedApproxDeadline : kFusedPairDeadline));
  rclcpp::SubscriptionOptions fused_options = loss_options(log_, this->get_clock(), "/fused/range");
  fused_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/fused/range", fused_qos,
    std::bind(&ControllerNode::fused_callback, this, std::placeholders::_1), fused_options);
  // /wheel/speed deadline: a missing wheel speed
  // is reported, like a missing fused range. Stock raises it per stream
  // silence of 2.5 periods (1.5 P fired in nominal wheel jitter),
  // stall, rcllf per lost message it can name and per logical window D of
  // silence from the start tag or the last record; a
  // stream that never starts is reported under stock by
  // wheel_startup_check(). The wheel speed node offers the same
  // kWheelDeadline (an offer longer than the request is incompatible QoS
  // and delivers nothing).
  auto wheel_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kWheelDeadline));
  rclcpp::SubscriptionOptions wheel_options = loss_options(log_, this->get_clock(), "/wheel/speed");
  wheel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/wheel/speed", wheel_qos,
    std::bind(&ControllerNode::wheel_callback, this, std::placeholders::_1), wheel_options);
  // /brake_cmd is best effort (republished every tick), so a lost
  // datagram is a loss DDS reports, not a retransmission. Best effort:
  // a design choice. rcllf burst B = 1, DERIVED: one publish()
  // site per tick, measured 1.
  auto brake_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kBrakeDeadline));  // offered
  brake_pub_ = this->create_publisher<custom_interfaces::msg::BrakeCmd>("/brake_cmd", brake_qos);
  // Node clock: the timer follows /clock under use_sim_time.
  // 25 ms tick. Not every fused pair is acted on: pairs come at
  // intervals of 20 to 40 ms (ApproximateTime, one per camera sample), so a
  // pair that a 20 ms interval follows can fall between two ticks and be
  // replaced before either reads it (in one recorded run pair n=2 lies
  // between ticks 4 and 5). What the threshold needs is
  // weaker and holds: every tick reads the newest pair, at most A old
  // (kLatencyBound), so a skipped pair only means a newer one is read. The
  // tick length is a term of kLatencyBound (the decision waits up to one
  // tick) and the decision delay of a lost brake command (one tick, then
  // resent). 25 ms is below the 30 ms mean pair period, so at most one pair
  // in five is skipped; a longer tick grows L by its length and skips more
  // pairs, a shorter one costs CPU for no shorter age (the pair age, not the
  // tick, dominates A).
  timer_ = this->create_timer(25ms, std::bind(&ControllerNode::control_callback, this));
}

void ControllerNode::fused_callback(const sensor_msgs::msg::Range & msg)
{
  fused_range_[0] = fused_range_[1];
  fused_stamp_ns_[0] = fused_stamp_ns_[1];
  fused_range_[1] = msg.range;
  fused_stamp_ns_[1] = rclcpp::Time(msg.header.stamp).nanoseconds();
  ++fused_count_;
  if (log_inputs_) {
    log_.line(
      "\"ev\":\"fused_in\",\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
      fused_count_, fused_stamp_ns_[1], this->now().nanoseconds());
  }
}

void ControllerNode::wheel_callback(const geometry_msgs::msg::TwistStamped & msg)
{
  wheel_speed_ = msg.twist.linear.x;
  wheel_stamp_ns_ = rclcpp::Time(msg.header.stamp).nanoseconds();
  ++wheel_count_;
  if (log_inputs_) {
    log_.line(
      "\"ev\":\"wheel_in\",\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
      wheel_count_, wheel_stamp_ns_, this->now().nanoseconds());
  }
}

void ControllerNode::decide(Step & s)
{
  // Closing speed from the stamps of the last two fused ranges.
  if (s.have_prev && s.stamp_ns != s.prev_stamp_ns) {
    const double dt = static_cast<double>(s.stamp_ns - s.prev_stamp_ns) * 1e-9;
    s.estimate = (s.prev_range - s.range) / dt;
    s.have_estimate = true;
  }
  const ClosingSpeed c = select_closing_speed(
    s.have_estimate, s.estimate, s.have_wheel, s.wheel_speed, plausibility_tolerance_);
  s.closing_speed = c.speed;
  s.source = c.source;

  double brake = 0.0;
  if (s.have_fused && s.closing_speed > 0.0) {
    s.ttc = s.range / s.closing_speed;
    if (s.ttc < ttc_full_) {
      brake = 1.0;
    } else if (partial_stage_ && s.ttc < ttc_partial_) {
      brake = partial_brake_;
    }
  }
  s.raw_brake = brake;
  if (brake < held_brake_) {
    brake = held_brake_;
  }
  held_brake_ = brake;
  s.brake = brake;
}

// The injected loss of one brake command (`drop_first_brake_ge`), logged.
bool ControllerNode::dropped(const custom_interfaces::msg::BrakeCmd & cmd)
{
  if (!drop_.hit(-1, cmd.level)) {
    return false;
  }
  log_.line(
    "\"ev\":\"drop_pub\",\"seq\":%" PRIu64 ",\"tick\":%" PRId64 ",\"brake\":%.17g"
    ",\"t_ns\":%" PRId64,
    cmd.seq, cmd.tick, cmd.level, this->now().nanoseconds());
  return true;
}

// A /wheel/speed stream that never starts. A DDS requested
// deadline counts per instance, and an instance exists only after its first
// sample (loss_events.hpp), so stock raises nothing for it. This timeout
// logs the stock deadline record ("ev":"deadline", same fields) for every
// kWheelDeadline window that ends with no wheel sample since the first tick,
// as DDS would once the instance existed: total = windows so far, change =
// windows since the last record (rmw_requested_deadline_missed_status_t).
// It ends at the first sample, where the DDS deadline takes over. Under rcllf
// the binding raises the logical deadline at t0 + k D itself,
// so the timeout runs only without the binding: the same weak-symbol test as
// log_loss_event's context. Detection only: the controller's rule of no
// decision without a wheel speed is unchanged.
void ControllerNode::wheel_startup_check()
{
  if (ros2_lf_message_lost_context != nullptr || wheel_count_ > 0) {
    return;
  }
  const int64_t now_ns = this->now().nanoseconds();
  if (wheel_wait_start_ns_ < 0) {
    wheel_wait_start_ns_ = now_ns;
    return;
  }
  const int64_t windows = startup_windows_elapsed(
    wheel_wait_start_ns_, now_ns,
    std::chrono::duration_cast<std::chrono::nanoseconds>(kWheelDeadline).count());
  if (windows > wheel_startup_windows_) {
    log_loss_event(
      log_, this->get_clock(), "deadline", "/wheel/speed", windows,
      windows - wheel_startup_windows_);
    wheel_startup_windows_ = windows;
  }
}

void ControllerNode::control_callback()
{
  wheel_startup_check();
  Step s;
  s.have_fused = fused_count_ >= 1;
  s.range = fused_range_[1];
  s.stamp_ns = fused_stamp_ns_[1];
  s.have_prev = fused_count_ >= 2;
  s.prev_range = fused_range_[0];
  s.prev_stamp_ns = fused_stamp_ns_[0];
  s.have_wheel = wheel_count_ > 0;
  s.wheel_speed = wheel_speed_;
  s.wheel_stamp_ns = wheel_stamp_ns_;
  decide(s);

  log_.line(
    "\"ev\":\"tick\",\"tick\":%" PRId64 ",\"t_ns\":%" PRId64 ",\"fused_n\":%" PRId64
    ",\"fused_stamp_ns\":%" PRId64 ",\"prev_fused_stamp_ns\":%" PRId64 ",\"fused_range\":%.9g"
    ",\"prev_fused_range\":%.9g,\"estimate\":%.17g,\"wheel_n\":%" PRId64
    ",\"wheel_stamp_ns\":%" PRId64 ",\"wheel_speed\":%.17g,\"closing_speed\":%.17g"
    ",\"source\":\"%s\",\"ttc\":%.17g,\"brake\":%.17g",
    tick_, this->now().nanoseconds(), fused_count_, fused_stamp_ns_[1], fused_stamp_ns_[0],
    fused_range_[1], fused_range_[0], s.have_estimate ? s.estimate : 0.0, wheel_count_,
    wheel_stamp_ns_, wheel_speed_, s.closing_speed, s.source, s.ttc, s.brake);
  ++tick_;

  send(tick_ - 1, s);
}

// One brake command with its identity. seq counts every decided
// command, the injected drop included, so the simulator sees a gap in seq for
// a command that never arrived (a level alone cannot show that).
custom_interfaces::msg::BrakeCmd ControllerNode::make_cmd(int64_t tick, const Step & s)
{
  custom_interfaces::msg::BrakeCmd cmd;
  cmd.level = s.brake;
  cmd.seq = ++cmd_seq_;
  cmd.tick = tick;
  cmd.stamp_ns = this->now().nanoseconds();
  // A level the tick decided itself starts a new run of that level; a level
  // above the tick's own decision is held and comes from level_seq_.
  if (s.raw_brake == s.brake && s.brake != level_) {
    level_seq_ = cmd.seq;
    level_ = s.brake;
  }
  return cmd;
}

void ControllerNode::send(int64_t tick, const Step & s)
{
  const custom_interfaces::msg::BrakeCmd cmd = make_cmd(tick, s);
  if (!dropped(cmd)) {
    log_brake_pub(cmd, s);  // before publish: see log_brake_pub
    brake_pub_->publish(cmd);
  }
}

// The brake-command publication endpoint and the command's lineage. n
// counts published commands (injected drops excluded), seq/tick/stamp_ns are
// the message's identity fields. The inputs the decision read are named by
// their keys: the current and previous fused sample (stamp, float32 range;
// fusion's `pair` record of the same key names its track and camera reading)
// and the wheel sample (stamp). held_from_seq is the command whose own
// decision set the published level; it differs from seq when the level is
// held from an earlier tick (raw below brake). The record is written just
// BEFORE publish(): its wall_ns must not be later than the subscriber's
// brake_rx. Stamped after publish() returned, a receiver in another process
// logged its brake_rx up to 88 us earlier (a stock simulation-time pilot run on
// the Orin pair).
void ControllerNode::log_brake_pub(const custom_interfaces::msg::BrakeCmd & cmd, const Step & s)
{
  ++brake_pubs_;
  log_.line(
    "\"ev\":\"brake_pub\",\"n\":%" PRId64 ",\"seq\":%" PRIu64 ",\"tick\":%" PRId64
    ",\"brake\":%.17g,\"raw\":%.17g,\"held_from_seq\":%" PRIu64 ",\"cur\":%s"
    ",\"fused_stamp_ns\":%" PRId64 ",\"fused_range\":%.9g,\"prev\":%s"
    ",\"prev_fused_stamp_ns\":%" PRId64 ",\"prev_fused_range\":%.9g,\"wheel\":%s"
    ",\"wheel_stamp_ns\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    brake_pubs_, cmd.seq, cmd.tick, cmd.level, s.raw_brake,
    s.brake > s.raw_brake ? level_seq_ : cmd.seq, s.have_fused ? "true" : "false",
    s.stamp_ns, s.range, s.have_prev ? "true" : "false", s.prev_stamp_ns, s.prev_range,
    s.have_wheel ? "true" : "false", s.wheel_stamp_ns, cmd.stamp_ns,
    this->now().nanoseconds());
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::ControllerNode)

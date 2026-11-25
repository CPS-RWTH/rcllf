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

#include "brake_fusion/wheel_speed_node.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

#include "brake_fusion/loss_events.hpp"
#include "brake_fusion/sim_state.hpp"

namespace brake_fusion
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

WheelSpeedNode::WheelSpeedNode(const rclcpp::NodeOptions & options)
: Node("wheel_speed", options)
{
  const std::string mode = this->declare_parameter<std::string>("stamp_mode", "now");
  const int64_t origin = this->declare_parameter<int64_t>("stamp_origin_ns", 0);
  stamper_ = std::make_unique<Stamper>(mode, origin, 10'000'000, this->now());
  period_ns_ = 10'000'000;
  // Grid sampling: "grid" fires the timer on origin + k * period and publishes,
  // for stamp k, the plant state stamped k (waiting for it if needed)
  // instead of the latest state. Needs stamp_mode assigned_clock.
  grid_ = this->declare_parameter<std::string>("sample_mode", "latest") == "grid";
  if (grid_ && mode != "assigned_clock") {
    throw std::invalid_argument("sample_mode grid needs stamp_mode assigned_clock");
  }
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  // Fault injection: BF_WHEEL_SILENT_FROM_MS makes
  // this sensor publish nothing from the tick at that time on (a dead wheel
  // speed sensor), so the controller's /wheel/speed deadline is checked in a
  // real run. An environment variable like the drop shim's: stock
  // and rcllf runs get it without a rebuild or another analyzed system. A
  // take-side drop cannot stand in for it, the writer would still meet its
  // deadline. Unset: never silent. The first silent tick is the first one at
  // or after the given time (rounded up), so no earlier sample is suppressed.
  if (const char * silent_env = std::getenv("BF_WHEEL_SILENT_FROM_MS")) {
    silent_from_tick_ =
      (std::strtoll(silent_env, nullptr, 10) * 1'000'000 + period_ns_ - 1) / period_ns_;
    log_.line(
      "\"ev\":\"fault_config\",\"kind\":\"silent\",\"from_tick\":%" PRId64,
      silent_from_tick_);
  }

  // /sim/state: one state per 1 ms step, legacy edge; the depth
  // 10 reader replaced states unseen when this executor stalled. Depth
  // kSimStateDepth (the literal is what the analyzer reads), and the lost
  // handler logs what the binding reports.
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
  // A local options variable with only a message_lost handler: the analyzer
  // follows the variable and sees that no deadline handler is set. A factory
  // call (loss_options) is opaque to it, the body being in a header, and would
  // turn this sensor's output into an unbounded burst.
  rclcpp::SubscriptionOptions state_options;
  state_options.event_callbacks.message_lost_callback =
    lost_logger(log_, this->get_clock(), "/sim/state");
  static_assert(kSimStateDepth == 100, "keep the KeepLast literal and kSimStateDepth equal");
  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/sim/state", state_qos,
    std::bind(&WheelSpeedNode::state_callback, this, std::placeholders::_1),
    state_options);
  check_sim_state_depth(*state_sub_, this->get_logger());
  // Offered deadline: the controller requests
  // kWheelDeadline, 2.5 times the 10 ms period (silence, not a single loss:
  // loss_events.hpp).
  // depth 10, DERIVED (rule in loss_events.hpp:82): ten 10 ms
  // samples cover 100 ms of traffic, more than this reader's deadline
  // kWheelDeadline 25 ms and the largest one, 90 ms (loss_events.hpp:59-80).
  // rcllf does not rest on this literal: the binding deepens an admitted
  // reader to reader_depth = max(8, B) when no reader_depth is declared, as
  // in every brake_fusion config (lf_generator.py _reader_depth).
  // Reliable: a design choice.
  // rcllf burst B = 2 on this edge, DERIVED: the analyzer counts the two
  // publish() sites of timer_callback (grid branch wheel_speed_node.cpp:152, latest
  // branch wheel_speed_node.cpp:169; the publish at wheel_speed_node.cpp:123 is in state_callback,
  // a separate reaction), a
  // sound upper bound; one tick takes one branch, measured maximum 1 per tick.
  auto speed_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kWheelDeadline));  // offered
  speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/wheel/speed", speed_qos);
  // Node clock: the timer follows /clock under use_sim_time.
  // 10 ms period: the closing speed comes from this sample, at
  // most 10 ms plus transport old. While braking at 8 m/s^2 that age makes
  // the speed over-read by at most 8 * 0.011 = 0.088 m/s, which is inside the
  // 0.5 m/s plausibility tolerance, so the tolerance (not the period) is the
  // speed error e_v in full_stage_ttc.
  timer_ = this->create_timer(10ms, std::bind(&WheelSpeedNode::timer_callback, this));
  if (grid_) {
    arm_on_grid(*timer_, this->now().nanoseconds(), origin, period_ns_);
  }
}

void WheelSpeedNode::state_callback(const nav_msgs::msg::Odometry & msg)
{
  velocity_ = msg.twist.twist.linear.x;
  state_stamp_ns_ = rclcpp::Time(msg.header.stamp).nanoseconds();
  have_state_ = true;
  if (grid_) {
    sampler_.add(msg, this->now().nanoseconds());
    geometry_msgs::msg::TwistStamped out;
    if (serve_pending(out)) {
      speed_pub_->publish(out);
    }
  }
}

void WheelSpeedNode::timer_callback()
{
  const rclcpp::Time now = this->now();
  if (grid_) {
    arm_on_grid(*timer_, now.nanoseconds(), stamper_->origin_ns(), period_ns_);
  }
  const rclcpp::Time stamp = stamper_->stamp(now);
  const int64_t tick = stamper_->tick();
  stamper_->advance();
  if (grid_) {
    if (stamper_->skip() || silent(tick)) {
      return;
    }
    stamper_->published();
    if (pending_tick_ >= 0) {
      log_.line(
        "\"ev\":\"fault\",\"kind\":\"unserved\",\"tick\":%" PRId64 ",\"t_ns\":%" PRId64,
        pending_tick_, now.nanoseconds());
    }
    pending_tick_ = tick;
    pending_stamp_ns_ = stamp.nanoseconds();
    release_ns_ = now.nanoseconds();
    geometry_msgs::msg::TwistStamped msg;
    if (serve_pending(msg)) {
      speed_pub_->publish(msg);
    }
    return;
  }
  if (!have_state_ || stamper_->skip() || silent(tick)) {
    return;
  }
  stamper_->published();
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "vehicle";
  msg.twist.linear.x = velocity_;
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"speed\":%.17g,\"state_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    seq_, tick, stamp.nanoseconds(), velocity_, state_stamp_ns_, now.nanoseconds());
  ++seq_;
  speed_pub_->publish(msg);
}

bool WheelSpeedNode::silent(int64_t tick)
{
  if (silent_from_tick_ < 0 || tick < silent_from_tick_) {
    return false;
  }
  if (tick == silent_from_tick_) {
    log_.line(
      "\"ev\":\"fault\",\"kind\":\"silent\",\"tick\":%" PRId64 ",\"t_ns\":%" PRId64,
      tick, this->now().nanoseconds());
  }
  return true;
}

// Grid sampling: publish the pending grid tick once the plant state with the
// same stamp is here. The wait is bounded by the state stream itself: the
// simulator publishes in stamp order over a reliable topic, so a newer state
// without this stamp means it is lost (logged as a fault, nothing published).
bool WheelSpeedNode::serve_pending(geometry_msgs::msg::TwistStamped & msg)
{
  if (pending_tick_ < 0) {
    return false;
  }
  PlantSample s;
  const GridSampler::Result found = sampler_.find(pending_stamp_ns_, &s);
  if (found == GridSampler::Result::kWaiting) {
    return false;
  }
  const int64_t tick = pending_tick_;
  pending_tick_ = -1;
  const int64_t acq_ns = this->now().nanoseconds();
  if (found == GridSampler::Result::kLost) {
    log_.line(
      "\"ev\":\"fault\",\"kind\":\"lost_state\",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
      ",\"t_ns\":%" PRId64,
      tick, pending_stamp_ns_, acq_ns);
    return false;
  }
  const int64_t state_step = (s.stamp_ns - stamper_->origin_ns()) / 1'000'000;
  msg.header.stamp = rclcpp::Time(pending_stamp_ns_, RCL_ROS_TIME);
  msg.header.frame_id = "vehicle";
  msg.twist.linear.x = s.velocity;
  const int64_t pub_ns = this->now().nanoseconds();
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"speed\":%.17g,\"state_ns\":%" PRId64 ",\"state_step\":%" PRId64
    ",\"state_rx_ns\":%" PRId64 ",\"release_ns\":%" PRId64 ",\"acq_ns\":%" PRId64
    ",\"pub_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    seq_, tick, pending_stamp_ns_, s.velocity, s.stamp_ns, state_step, s.rx_ns, release_ns_,
    acq_ns, pub_ns, release_ns_);
  ++seq_;
  return true;
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::WheelSpeedNode)

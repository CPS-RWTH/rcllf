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

#include "brake_fusion/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
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

namespace
{
// Detection stand-in: spin on the steady clock for `us` microseconds.
void busy_work(int64_t us)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::microseconds(us);
  while (std::chrono::steady_clock::now() < end) {
  }
}
}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera", options)
{
  obstacle_distance_ = this->declare_parameter<double>("obstacle_distance", 40.0);
  // Detection stand-in: busy_work spins on the steady clock, so
  // it models elapsed detection latency W, not CPU demand (a preempted spin
  // still ends W after it started, having used less CPU). Target: the
  // largest latency, 2 W in data mode at full range, stays within half the
  // 10 ms gap from a camera sample to the next radar sample (camera at 30 k,
  // radar at 20 k ms), so with 5 ms left for transport stock ROS still sees
  // the camera message before that radar sample, and an rcllf record stays
  // well inside the 15 ms join timeout. 2 W = 5 ms gives W = 2.5 ms. At
  // 2 W >= 10 ms the stock pairing input order changes with the work itself.
  // The 5 ms transport budget against evidence: from
  // the camera stamp to the fusion callback, minus the logged work, at most
  // 0.74 ms on one Orin and 0.53 ms across the Orin pair (18 stock wall-clock
  // runs, 2519 camera samples). Three runs had a start-up backlog of up to 7 samples (up to 191 ms
  // late) that drains 30 ms per sample, queued after discovery, not
  // transport. Under sim time (1 ms clock steps, 7 runs) it was at most
  // 0.5 ms after start-up, once 2.5 ms. 5 ms is twice the largest of these.
  detection_work_us_ = this->declare_parameter<int64_t>("detection_work_us", 2500);
  data_dependent_work_ =
    this->declare_parameter<std::string>("detection_work_mode", "fixed") == "data";
  const std::string mode = this->declare_parameter<std::string>("stamp_mode", "now");
  const int64_t origin = this->declare_parameter<int64_t>("stamp_origin_ns", 0);
  stamper_ = std::make_unique<Stamper>(mode, origin, 30'000'000, this->now());
  period_ns_ = 30'000'000;
  // Grid sampling: "grid" fires the timer on origin + k * period and publishes,
  // for stamp k, the plant state stamped k (waiting for it if needed)
  // instead of the latest state. Needs stamp_mode assigned_clock.
  grid_ = this->declare_parameter<std::string>("sample_mode", "latest") == "grid";
  if (grid_ && mode != "assigned_clock") {
    throw std::invalid_argument("sample_mode grid needs stamp_mode assigned_clock");
  }
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  drop_ = PublishDrop(this->declare_parameter<int64_t>("drop_stamp_ms", -1), -1.0);

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
    "/sim/state", state_qos, std::bind(&CameraNode::state_callback, this, std::placeholders::_1),
    state_options);
  check_sim_state_depth(*state_sub_, this->get_logger());
  // depth 10, DERIVED (rule in loss_events.hpp:82): ten 30 ms
  // samples cover 300 ms of traffic, more than this reader's deadline
  // kCameraDeadline 45 ms and the largest one, 90 ms (loss_events.hpp:59-80).
  // rcllf does not rest on this literal: the binding deepens an admitted
  // reader to reader_depth = max(8, B) when no reader_depth is declared, as
  // in every brake_fusion config (lf_generator.py _reader_depth).
  // Reliable: a design choice.
  // rcllf burst B = 2 on this edge, DERIVED: the analyzer counts the two
  // publish() sites of timer_callback (grid branch camera_node.cpp:170, latest
  // branch camera_node.cpp:220; the publish at camera_node.cpp:138 is in state_callback,
  // a separate reaction), a
  // sound upper bound; one tick takes one branch, measured maximum 1 per tick.
  auto range_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kCameraDeadline));  // offered
  range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/camera/range", range_qos);
  // Node clock: the timer follows /clock under use_sim_time.
  // 30 ms period: the slowest sensor and the one every fused
  // pair waits for, so one lost camera sample leaves the newest fused pair
  // one pair interval older, the 36 ms that kLatencyBound adds for one loss
  // (stop_margin.hpp). A longer period grows that term and the pair age.
  timer_ = this->create_timer(30ms, std::bind(&CameraNode::timer_callback, this));
  if (grid_) {
    arm_on_grid(*timer_, this->now().nanoseconds(), origin, period_ns_);
  }
}

void CameraNode::state_callback(const nav_msgs::msg::Odometry & msg)
{
  position_ = msg.pose.pose.position.x;
  state_stamp_ns_ = rclcpp::Time(msg.header.stamp).nanoseconds();
  have_state_ = true;
  if (grid_) {
    sampler_.add(msg, this->now().nanoseconds());
    sensor_msgs::msg::Range out;
    if (serve_pending(out)) {
      if (!dropped(out)) {
        range_pub_->publish(out);
      }
    }
  }
}

void CameraNode::timer_callback()
{
  // Capture: stamp and sample at the start of the tick.
  const rclcpp::Time now = this->now();
  if (grid_) {
    arm_on_grid(*timer_, now.nanoseconds(), stamper_->origin_ns(), period_ns_);
  }
  const rclcpp::Time stamp = stamper_->stamp(now);
  const int64_t tick = stamper_->tick();
  stamper_->advance();
  if (grid_) {
    if (stamper_->skip()) {
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
    sensor_msgs::msg::Range msg;
    if (serve_pending(msg)) {
      if (!dropped(msg)) {
        range_pub_->publish(msg);
      }
    }
    return;
  }
  if (!have_state_ || stamper_->skip()) {
    return;
  }
  stamper_->published();
  const double position = position_;
  const int64_t state_ns = state_stamp_ns_;
  const double range = obstacle_distance_ - position;
  int64_t work_us = detection_work_us_;
  if (data_dependent_work_) {
    // 2 W r / obstacle: the same peak as the fixed
    // mode's budget bound, 2 W = 5 ms at full range, and unequal means. The
    // braking trajectory does not sweep the range uniformly: from the start
    // to the stop the mean is 0.896 W at ttc_full 1.141 s (ideal-timing oracle
    // t2approx; the mean follows the stop position: 0.889 W at the earlier
    // 1.095 s, and at the earlier 1.15 s it was 0.896 W too, matched by
    // the logged ranges of the recorded Orin rcllf runs, 124 frames). Stock
    // starts its camera later on the trajectory: 0.869 W and 0.878 W, peak
    // 4.81 ms, in the recorded Orin data-mode runs (at 1.15 s), the measured
    // work 1 us (median) above the logged work, which matches 2 W r /
    // obstacle in every frame. After the
    // stop the range stays at the stop gap (4.72 m in those runs, 0.236 W
    // per frame), so a whole-run mean depends on how long a run continues
    // past the stop (0.82 W one process, 0.48 W two hosts with the 10 s
    // timeout); only the braking-phase mean is a scenario property.
    // Equal braking-phase means would need 2.25 W at full range, 5.62 ms,
    // past the half-gap camera budget above, which stays a design choice.
    work_us = static_cast<int64_t>(
      2.0 * static_cast<double>(detection_work_us_) *
      std::clamp(range / obstacle_distance_, 0.0, 1.0));
  }
  busy_work(work_us);

  sensor_msgs::msg::Range msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "camera";
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.min_range = 0.0F;
  msg.max_range = 200.0F;
  msg.range = static_cast<float>(range);
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"position\":%.17g,\"work_us\":%" PRId64 ",\"state_ns\":%" PRId64
    ",\"t_ns\":%" PRId64,
    seq_, tick, stamp.nanoseconds(), msg.range, position, work_us, state_ns, now.nanoseconds());
  ++seq_;
  if (!dropped(msg)) {
    range_pub_->publish(msg);
  }
}

// Grid sampling: publish the pending grid tick once the plant state with the
// same stamp is here. The wait is bounded by the state stream itself: the
// simulator publishes in stamp order over a reliable topic, so a newer state
// without this stamp means it is lost (logged as a fault, nothing published).
// The injected loss of one publication (`drop_stamp_ms`), logged.
bool CameraNode::dropped(const sensor_msgs::msg::Range & msg)
{
  const int64_t stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  if (!drop_.hit(stamp_ns, 0.0)) {
    return false;
  }
  log_.line(
    "\"ev\":\"drop_pub\",\"stamp_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    stamp_ns, this->now().nanoseconds());
  return true;
}

bool CameraNode::serve_pending(sensor_msgs::msg::Range & msg)
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
  const double range = obstacle_distance_ - s.position;
  int64_t work_us = detection_work_us_;
  if (data_dependent_work_) {
    work_us = static_cast<int64_t>(
      2.0 * static_cast<double>(detection_work_us_) *
      std::clamp(range / obstacle_distance_, 0.0, 1.0));
  }
  busy_work(work_us);
  msg.header.stamp = rclcpp::Time(pending_stamp_ns_, RCL_ROS_TIME);
  msg.header.frame_id = "camera";
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.min_range = 0.0F;
  msg.max_range = 200.0F;
  msg.range = static_cast<float>(range);
  const int64_t pub_ns = this->now().nanoseconds();
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"position\":%.17g,\"work_us\":%" PRId64 ",\"state_ns\":%" PRId64
    ",\"state_step\":%" PRId64
    ",\"state_rx_ns\":%" PRId64 ",\"release_ns\":%" PRId64 ",\"acq_ns\":%" PRId64
    ",\"pub_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    seq_, tick, pending_stamp_ns_, msg.range, s.position, work_us, s.stamp_ns, state_step, s.rx_ns,
    release_ns_, acq_ns, pub_ns, release_ns_);
  ++seq_;
  return true;
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::CameraNode)

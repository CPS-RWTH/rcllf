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

#include "brake_fusion/radar_node.hpp"

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

RadarNode::RadarNode(const rclcpp::NodeOptions & options)
: Node("radar", options)
{
  obstacle_distance_ = this->declare_parameter<double>("obstacle_distance", 40.0);
  // A second radar instance (node name radar2) publishes on the same topic
  // with a known mount bias, so the fused value tells which radar a pair used.
  range_bias_ = this->declare_parameter<double>("range_bias", 0.0);
  const std::string mode = this->declare_parameter<std::string>("stamp_mode", "now");
  const int64_t origin = this->declare_parameter<int64_t>("stamp_origin_ns", 0);
  stamper_ = std::make_unique<Stamper>(mode, origin, 20'000'000, this->now());
  period_ns_ = 20'000'000;
  // Grid sampling: "grid" fires the timer on origin + k * period and publishes,
  // for stamp k, the plant state stamped k (waiting for it if needed)
  // instead of the latest state. Needs stamp_mode assigned_clock.
  grid_ = this->declare_parameter<std::string>("sample_mode", "latest") == "grid";
  if (grid_ && mode != "assigned_clock") {
    throw std::invalid_argument("sample_mode grid needs stamp_mode assigned_clock");
  }
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  drop_ = PublishDrop(this->declare_parameter<int64_t>("drop_stamp_ms", -1), -1.0);
  log_.line(
    "\"ev\":\"config\",\"name\":\"%s\",\"range_bias\":%.9g,\"stamp_mode\":\"%s\"",
    this->get_name(), range_bias_, mode.c_str());

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
    "/sim/state", state_qos, std::bind(&RadarNode::state_callback, this, std::placeholders::_1),
    state_options);
  check_sim_state_depth(*state_sub_, this->get_logger());
  // /radar/range is best effort (sensor data), so a lost datagram
  // is a loss DDS reports (message_lost), not a retransmission. Best effort:
  // a design choice.
  // depth 10, DERIVED (rule in loss_events.hpp:82): /radar/range
  // has two 20 ms writers (radar, radar2), so ten samples cover 5 x 20 =
  // 100 ms of traffic, more than this reader's deadline kRadarDeadline 30 ms
  // and the largest one, 90 ms (loss_events.hpp:59-80).
  // rcllf does not rest on this literal: the binding deepens an admitted
  // reader to reader_depth = max(8, B) when no reader_depth is declared, as
  // in every brake_fusion config (lf_generator.py _reader_depth).
  // rcllf burst B = 2 on this edge, DERIVED: the analyzer counts the two
  // publish() sites of timer_callback (grid branch radar_node.cpp:146, latest
  // branch radar_node.cpp:168; the publish at radar_node.cpp:115 is in state_callback,
  // a separate reaction), a
  // sound upper bound; one tick takes one branch, measured maximum 1 per tick.
  auto range_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kRadarDeadline));  // offered
  range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/radar/range", range_qos);
  // Node clock: the timer follows /clock under use_sim_time.
  // 20 ms period: the fusion pairs each camera sample with a
  // radar sample, so the radar sets the fused rate. 20 ms is not a multiple
  // of the 30 ms camera, which makes the pairing a real choice (the case the
  // ApproximateTime parameters are about), and its age in the latency bound
  // L of stop_margin.hpp is part of the 77 ms that sets ttc_full.
  timer_ = this->create_timer(20ms, std::bind(&RadarNode::timer_callback, this));
  if (grid_) {
    arm_on_grid(*timer_, this->now().nanoseconds(), origin, period_ns_);
  }
}

void RadarNode::state_callback(const nav_msgs::msg::Odometry & msg)
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

void RadarNode::timer_callback()
{
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
  sensor_msgs::msg::Range msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = this->get_name();
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.min_range = 0.0F;
  msg.max_range = 200.0F;
  msg.range = static_cast<float>(obstacle_distance_ - position_ + range_bias_);
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"position\":%.17g,\"state_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    seq_, tick, stamp.nanoseconds(), msg.range, position_, state_stamp_ns_, now.nanoseconds());
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
bool RadarNode::dropped(const sensor_msgs::msg::Range & msg)
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

bool RadarNode::serve_pending(sensor_msgs::msg::Range & msg)
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
  const double range = obstacle_distance_ - s.position + range_bias_;
  msg.header.stamp = rclcpp::Time(pending_stamp_ns_, RCL_ROS_TIME);
  msg.header.frame_id = this->get_name();
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.min_range = 0.0F;
  msg.max_range = 200.0F;
  msg.range = static_cast<float>(range);
  const int64_t pub_ns = this->now().nanoseconds();
  log_.line(
    "\"ev\":\"pub\",\"seq\":%" PRId64 ",\"tick\":%" PRId64 ",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"position\":%.17g,\"state_ns\":%" PRId64 ",\"state_step\":%" PRId64
    ",\"state_rx_ns\":%" PRId64 ",\"release_ns\":%" PRId64 ",\"acq_ns\":%" PRId64
    ",\"pub_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
    seq_, tick, pending_stamp_ns_, msg.range, s.position, s.stamp_ns, state_step, s.rx_ns,
    release_ns_, acq_ns, pub_ns, release_ns_);
  ++seq_;
  return true;
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::RadarNode)

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

#include "brake_fusion/radar_tracker.hpp"

#include <cinttypes>
#include <functional>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

namespace brake_fusion
{

RadarTrackerNode::RadarTrackerNode(const rclcpp::NodeOptions & options)
: Node("radar_tracker", options)
{
  const double gain = this->declare_parameter<double>("gain", 0.5);
  filter_ = TrackFilter(gain);
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());

  // The subscriber QoS of the fusion's radar input (best effort,
  // as offered by the radar; deadline and loss handlers).
  // Best effort, a design choice; depth 10 DERIVED
  // (rule in loss_events.hpp:82): two 20 ms writers (radar, radar2), so ten
  // samples cover 5 x 20 = 100 ms, more than kRadarDeadline 30 ms and the
  // largest deadline, 90 ms.
  auto radar_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kRadarDeadline));
  // Reliable, KeepLast(10). The deadline is offered because the fusion's
  // radar input (unchanged, remapped to /radar/track) requests kRadarDeadline;
  // a publisher without it would be QoS-incompatible and deliver nothing.
  // Reliable CONVENTION (the writer side of the tracker edge; the fusion
  // reader is best effort, so the edge is best effort). Depth 10 DERIVED as
  // above (one track per radar sample: 100 ms). rcllf burst B = 1, DERIVED:
  // the one publish() in radar_callback, measured 1.
  auto track_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kRadarDeadline));
  track_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/radar/track", track_qos);
  rclcpp::SubscriptionOptions radar_options = loss_options(log_, this->get_clock(), "/radar/range");
  radar_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/radar/range", radar_qos,
    std::bind(&RadarTrackerNode::radar_callback, this, std::placeholders::_1),
    radar_options);
  log_.line("\"ev\":\"config\",\"gain\":%.9g", gain);
}

void RadarTrackerNode::radar_callback(const sensor_msgs::msg::Range & msg)
{
  const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
  const int64_t now_ns = this->now().nanoseconds();
  log_.line(
    "\"ev\":\"radar_in\",\"n\":%" PRId64 ",\"src\":\"%s\",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"t_ns\":%" PRId64,
    radar_in_, msg.header.frame_id.c_str(), stamp, msg.range, now_ns);
  ++radar_in_;
  const double est = filter_.update(msg.range);
  sensor_msgs::msg::Range track = msg;
  track.header.frame_id = "radar_tracker";
  track.range = static_cast<float>(est);
  // "range" is the published float32 value. %.9g round-trips a float32
  // exactly, so the fusion's radar_range of this message equals it bit for
  // bit; the end-to-end analysis joins track to pair on (stamp_ns, range).
  log_.line(
    "\"ev\":\"track\",\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"est\":%.9g,\"src\":\"%s\""
    ",\"range\":%.9g,\"t_ns\":%" PRId64,
    filter_.count() - 1, stamp, est, msg.header.frame_id.c_str(), track.range, now_ns);
  track_pub_->publish(track);
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::RadarTrackerNode)

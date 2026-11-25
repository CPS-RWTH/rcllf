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

#include "brake_fusion/exact_fusion_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <rclcpp_components/register_node_macro.hpp>

namespace brake_fusion
{

namespace
{
int64_t stamp_ns(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  return msg ? rclcpp::Time(msg->header.stamp).nanoseconds() : -1;
}
}  // namespace

ExactFusionNode::ExactFusionNode(const rclcpp::NodeOptions & options)
: Node("fusion", options)
{
  // Queue size: derivation in approximate_fusion_node.cpp.
  const int64_t queue_size = this->declare_parameter<int64_t>("queue_size", 9);
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());

  auto fused_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kFusedPairDeadline));
  fused_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/fused/range", fused_qos);
  // A QoS deadline and deadline / message_lost handlers on both
  // synchronizer inputs, the same code for stock and rcllf.
  // /radar/range best effort, as offered by the radar.
  auto radar_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kRadarDeadline));
  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kCameraDeadline));
  rclcpp::SubscriptionOptions radar_options = loss_options(log_, this->get_clock(), "/radar/range");
  rclcpp::SubscriptionOptions camera_options =
    loss_options(log_, this->get_clock(), "/camera/range");
  radar_sub_.subscribe(this, "/radar/range", radar_qos.get_rmw_qos_profile(), radar_options);
  camera_sub_.subscribe(this, "/camera/range", camera_qos.get_rmw_qos_profile(), camera_options);
  radar_sub_.registerCallback(
    std::bind(&ExactFusionNode::radar_arrival, this, std::placeholders::_1));
  camera_sub_.registerCallback(
    std::bind(&ExactFusionNode::camera_arrival, this, std::placeholders::_1));
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(queue_size)), radar_sub_, camera_sub_);
  sync_->registerCallback(
    std::bind(
      &ExactFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
  sync_->registerDropCallback(
    std::bind(
      &ExactFusionNode::drop_callback, this, std::placeholders::_1, std::placeholders::_2));
  log_.line(
    "\"ev\":\"config\",\"sync_policy\":\"exact\",\"queue_size\":%" PRId64, queue_size);
}

ExactFusionNode::~ExactFusionNode()
{
  // Samples still open when the node stops, then the totals per input
  // (0 = radar, 1 = camera). open_stamps_max is the most distinct open stamps
  // (the policy's tuples) after an arrival, its queue demand.
  for (int i = 0; i < 2; ++i) {
    for (const auto & [stamp, n] : open_[i]) {
      log_.line(
        "\"ev\":\"sample_pending\",\"input\":%d,\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64,
        i, n, stamp);
    }
  }
  const int64_t arrivals[2] = {radar_in_, camera_in_};
  for (int i = 0; i < 2; ++i) {
    log_.line(
      "\"ev\":\"ledger\",\"input\":%d,\"arrivals\":%" PRId64 ",\"paired\":%" PRId64
      ",\"dropped\":%" PRId64 ",\"overwritten_same_stamp\":%" PRId64 ",\"pending\":%zu"
      ",\"open_stamps_max\":%" PRId64,
      i, arrivals[i], paired_[i], dropped_[i], overwritten_[i], open_[i].size(), open_stamps_max_);
  }
}

void ExactFusionNode::open_sample(int input, int64_t n, int64_t stamp)
{
  auto it = open_[input].find(stamp);
  if (it != open_[input].end()) {
    // add<i> assigns tuples_[stamp] slot i: the earlier sample is gone.
    log_.line(
      "\"ev\":\"sample_drop\",\"input\":%d,\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64
      ",\"reason\":\"overwritten_same_stamp\",\"by_n\":%" PRId64 ",\"t_ns\":%" PRId64,
      input, it->second, stamp, n, this->now().nanoseconds());
    ++overwritten_[input];
    it->second = n;
  } else {
    open_[input].emplace(stamp, n);
  }
  std::set<int64_t> stamps;
  for (const auto & m : open_) {
    for (const auto & kv : m) {
      stamps.insert(kv.first);
    }
  }
  open_stamps_max_ = std::max<int64_t>(open_stamps_max_, static_cast<int64_t>(stamps.size()));
}

int64_t ExactFusionNode::close_sample(int input, int64_t stamp)
{
  auto it = open_[input].find(stamp);
  if (it == open_[input].end()) {
    return -1;
  }
  const int64_t n = it->second;
  open_[input].erase(it);
  return n;
}

void ExactFusionNode::radar_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  // Same fields as FirstRadarFusionNode's radar_in, without its own pairing
  // verdict (taken / why): the synchronizer decides after this line.
  log_.line(
    "\"ev\":\"radar_in\",\"n\":%" PRId64 ",\"src\":\"%s\",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"t_ns\":%" PRId64,
    radar_in_, msg->header.frame_id.c_str(), stamp_ns(msg), msg->range,
    this->now().nanoseconds());
  open_sample(0, radar_in_, stamp_ns(msg));
  ++radar_in_;
}

void ExactFusionNode::camera_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  log_.line(
    "\"ev\":\"camera_in\",\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"range\":%.9g"
    ",\"t_ns\":%" PRId64,
    camera_in_, stamp_ns(msg), msg->range, this->now().nanoseconds());
  open_sample(1, camera_in_, stamp_ns(msg));
  ++camera_in_;
}

void ExactFusionNode::sync_callback(
  const sensor_msgs::msg::Range::ConstSharedPtr & radar,
  const sensor_msgs::msg::Range::ConstSharedPtr & camera)
{
  // The fused range is the mean of the two ranges, so its stamp is the mean
  // of the two stamps (equal under ExactTime).
  const int64_t radar_ns = stamp_ns(radar);
  const int64_t camera_ns = stamp_ns(camera);
  const int64_t fused_ns = radar_ns + (camera_ns - radar_ns) / 2;
  sensor_msgs::msg::Range fused;
  fused.header.stamp = rclcpp::Time(fused_ns, this->get_clock()->get_clock_type());
  fused.header.frame_id = "fused";
  fused.radiation_type = radar->radiation_type;
  fused.min_range = radar->min_range;
  fused.max_range = radar->max_range;
  fused.range = 0.5F * (radar->range + camera->range);
  const int64_t radar_n = close_sample(0, radar_ns);
  const int64_t camera_n = close_sample(1, camera_ns);
  ++paired_[0];
  ++paired_[1];
  last_pair_stamp_ = radar_ns;
  log_.line(
    "\"ev\":\"pair\",\"n\":%" PRId64 ",\"radar_stamp_ns\":%" PRId64 ",\"camera_stamp_ns\":%" PRId64
    ",\"fused_stamp_ns\":%" PRId64 ",\"radar_range\":%.9g,\"camera_range\":%.9g"
    ",\"fused_range\":%.9g,\"radar_src\":\"%s\",\"radar_n\":%" PRId64 ",\"camera_n\":%" PRId64
    ",\"t_ns\":%" PRId64,
    pairs_, radar_ns, camera_ns, fused_ns, radar->range, camera->range, fused.range,
    radar->header.frame_id.c_str(), radar_n, camera_n, this->now().nanoseconds());
  ++pairs_;
  fused_pub_->publish(fused);
}

void ExactFusionNode::drop_callback(
  const sensor_msgs::msg::Range::ConstSharedPtr & radar,
  const sensor_msgs::msg::Range::ConstSharedPtr & camera)
{
  // A partial set the ExactTime policy discards; the missing member is null.
  // clearOldTuples drops stamps at or before the last match, the queue bound
  // drops the oldest open stamp past queue_size (exact_time.h checkTuple).
  const int64_t r_ns = stamp_ns(radar);
  const int64_t c_ns = stamp_ns(camera);
  const int64_t key = radar ? r_ns : c_ns;
  const int64_t radar_n = radar ? close_sample(0, r_ns) : -1;
  const int64_t camera_n = camera ? close_sample(1, c_ns) : -1;
  dropped_[0] += radar ? 1 : 0;
  dropped_[1] += camera ? 1 : 0;
  log_.line(
    "\"ev\":\"drop\",\"n\":%" PRId64 ",\"radar_stamp_ns\":%" PRId64 ",\"camera_stamp_ns\":%" PRId64
    ",\"radar_src\":\"%s\",\"radar_n\":%" PRId64 ",\"camera_n\":%" PRId64
    ",\"reason\":\"%s\",\"t_ns\":%" PRId64,
    drops_, r_ns, c_ns, radar ? radar->header.frame_id.c_str() : "", radar_n, camera_n,
    key <= last_pair_stamp_ ? "older_than_pair" : "queue_overflow", this->now().nanoseconds());
  ++drops_;
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::ExactFusionNode)

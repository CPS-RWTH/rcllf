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

#ifndef BRAKE_FUSION__RADAR_TRACKER_HPP_
#define BRAKE_FUSION__RADAR_TRACKER_HPP_

#include <cstdint>

#include "brake_fusion/jsonl_log.hpp"
#include "brake_fusion/loss_events.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace brake_fusion
{

// The tracker's estimate: the first measurement sets
// est = z, every later one moves it by est += gain * (z - est). The update is
// order dependent on purpose: radar then radar2 ends elsewhere than radar2
// then radar (radar2 carries a +0.5 m bias).
class TrackFilter
{
public:
  explicit TrackFilter(double gain)
  : gain_(gain) {}

  double update(double z)
  {
    est_ = n_ == 0 ? z : est_ + gain_ * (z - est_);
    ++n_;
    return est_;
  }

  double estimate() const {return est_;}
  int64_t count() const {return n_;}

private:
  double gain_;
  double est_{0.0};
  int64_t n_{0};
};

// radar_tracker: one plain subscription on /radar/range (same QoS as the
// fusion's radar input), so it sees both radars. Every message updates the
// TrackFilter and publishes /radar/track with the measurement's stamp, range =
// estimate and frame_id "radar_tracker". Parameter `gain` (default 0.5).
class RadarTrackerNode : public rclcpp::Node
{
public:
  explicit RadarTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void radar_callback(const sensor_msgs::msg::Range & msg);

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr radar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr track_pub_;
  JsonlLog log_;
  TrackFilter filter_{0.5};
  int64_t radar_in_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__RADAR_TRACKER_HPP_

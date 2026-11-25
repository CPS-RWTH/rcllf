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

#ifndef BRAKE_FUSION__EXACT_FUSION_NODE_HPP_
#define BRAKE_FUSION__EXACT_FUSION_NODE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <memory>

#include "brake_fusion/jsonl_log.hpp"
#include "brake_fusion/loss_events.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace brake_fusion
{

// Radar/camera fusion with an unmodified message_filters ExactTime
// synchronizer, for stamps on a common grid. The drop callback logs every
// partial set the policy discards, with the reason: older_than_pair
// (clearOldTuples after a match) or queue_overflow (more than queue_size open
// stamps). ExactTime keys its tuples by stamp, so a second sample with the
// same (input, stamp) overwrites the first without any callback
// (exact_time.h add<i>); the node reports that as a `sample_drop` with reason
// overwritten_same_stamp, naming the overwritten sample.
//
// The node is named `fusion`; the launch argument `fusion` (exact or
// approximate) selects this class or its sibling, and `queue_size` is the
// policy's queue (default 9). Every matched pair publishes /fused/range with
// the mean of the two ranges and the mean of the two stamps.
class ExactFusionNode : public rclcpp::Node
{
public:
  explicit ExactFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExactFusionNode() override;

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Range, sensor_msgs::msg::Range>;

  // Arrival records: a second callback on each Subscriber,
  // registered before the Synchronizer connects, so the arrival line of a
  // message precedes the pair it completes.
  void radar_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg);
  void camera_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg);
  // Before the policy's add: report an overwrite and record the sample.
  void open_sample(int input, int64_t n, int64_t stamp);
  int64_t close_sample(int input, int64_t stamp);
  void sync_callback(
    const sensor_msgs::msg::Range::ConstSharedPtr & radar,
    const sensor_msgs::msg::Range::ConstSharedPtr & camera);
  void drop_callback(
    const sensor_msgs::msg::Range::ConstSharedPtr & radar,
    const sensor_msgs::msg::Range::ConstSharedPtr & camera);

  message_filters::Subscriber<sensor_msgs::msg::Range> radar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Range> camera_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr fused_pub_;
  JsonlLog log_;
  int64_t pairs_{0};
  int64_t radar_in_{0};
  int64_t camera_in_{0};
  int64_t drops_{0};
  int64_t last_pair_stamp_{INT64_MIN};
  // Open samples per input (0 radar, 1 camera): stamp -> arrival ordinal.
  std::array<std::map<int64_t, int64_t>, 2> open_;
  std::array<int64_t, 2> overwritten_{0, 0};
  std::array<int64_t, 2> paired_{0, 0};
  std::array<int64_t, 2> dropped_{0, 0};
  int64_t open_stamps_max_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__EXACT_FUSION_NODE_HPP_

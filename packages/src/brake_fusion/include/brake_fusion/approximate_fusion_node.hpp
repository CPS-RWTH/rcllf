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

#ifndef BRAKE_FUSION__APPROXIMATE_FUSION_NODE_HPP_
#define BRAKE_FUSION__APPROXIMATE_FUSION_NODE_HPP_

#include <cstdint>
#include <memory>

#include "brake_fusion/jsonl_log.hpp"
#include "brake_fusion/sample_ledger.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace brake_fusion
{

// Radar/camera fusion with an unmodified message_filters ApproximateTime
// synchronizer, for physical stamps (default). This policy has no drop
// callback, so the node keeps a SampleLedger: every input sample
// ends in one `pair`, one `sample_drop` (with the reason) or one
// `sample_pending` line, and the `ledger` line at shutdown sums them up.
//
// The node is named `fusion`; the launch argument `fusion` (exact or
// approximate) selects this class or its sibling, and `queue_size` is the
// policy's queue (default 9). Every matched pair publishes /fused/range with
// the mean of the two ranges and the mean of the two stamps.
class ApproximateFusionNode : public rclcpp::Node
{
public:
  explicit ApproximateFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ApproximateFusionNode() override;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Range, sensor_msgs::msg::Range>;

  // Arrival records: a second callback on each Subscriber,
  // registered before the Synchronizer connects, so the arrival line of a
  // message precedes the pair it completes.
  void radar_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg);
  void camera_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg);
  void note_duplicate(int input, int64_t n, int64_t stamp, int64_t earlier);
  // Ledger settle, at each arrival and at shutdown (one observer per Subscriber).
  void settle();
  void sync_callback(
    const sensor_msgs::msg::Range::ConstSharedPtr & radar,
    const sensor_msgs::msg::Range::ConstSharedPtr & camera);

  message_filters::Subscriber<sensor_msgs::msg::Range> radar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Range> camera_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr fused_pub_;
  JsonlLog log_;
  std::unique_ptr<SampleLedger> ledger_;
  int64_t pairs_{0};
  int64_t radar_in_{0};
  int64_t camera_in_{0};
  // Fault injection: stall once at camera arrival stall_at_, -1 never.
  int64_t stall_at_{-1};
  int64_t stall_ms_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__APPROXIMATE_FUSION_NODE_HPP_

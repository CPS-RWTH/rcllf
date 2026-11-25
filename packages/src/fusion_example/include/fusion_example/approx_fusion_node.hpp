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

#ifndef FUSION_EXAMPLE__APPROX_FUSION_NODE_HPP_
#define FUSION_EXAMPLE__APPROX_FUSION_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"

namespace fusion_example
{

// Unmodified message_filters ApproximateTime synchronizer over fusion/a and fusion/b.
// Each matched pair is logged as one PAIR line and published on
// fusion/approx_pairs (stamp = a's stamp, temperature = a's sequence number,
// humidity = b's sequence number).
class ApproxFusionNode : public rclcpp::Node
{
public:
  explicit ApproxFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    custom_interfaces::msg::SensorData, custom_interfaces::msg::SensorData>;

  void sync_callback(
    const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
    const custom_interfaces::msg::SensorData::ConstSharedPtr & b);

  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_a_;
  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr pairs_pub_;
};

}  // namespace fusion_example

#endif  // FUSION_EXAMPLE__APPROX_FUSION_NODE_HPP_

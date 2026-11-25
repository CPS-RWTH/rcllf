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

#ifndef BNA_EXAMPLE__BNA_FUSION_NODE_HPP_
#define BNA_EXAMPLE__BNA_FUSION_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "rcllf_message_filters/bna_policy.hpp"

namespace bna_example
{

// BoundedNearestToAnchor over bna/anchor (input 0) and bna/candidate.
// Each match is logged as one PAIR line and published on bna/fused. Both
// inputs register a deadline and a message_lost handler, logged as MISSING
// and LOST lines (these handlers still run under BNA).
class BnaFusionNode : public rclcpp::Node
{
public:
  explicit BnaFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Policy = rcllf_message_filters::sync_policies::BoundedNearestToAnchor<
    custom_interfaces::msg::SensorData, custom_interfaces::msg::SensorData>;

  void sync_callback(
    const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
    const custom_interfaces::msg::SensorData::ConstSharedPtr & c);
  rclcpp::SubscriptionOptions input_options(const char * input);

  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_anchor_;
  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_candidate_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<custom_interfaces::msg::SensorData>::SharedPtr fused_pub_;
};

}  // namespace bna_example

#endif  // BNA_EXAMPLE__BNA_FUSION_NODE_HPP_

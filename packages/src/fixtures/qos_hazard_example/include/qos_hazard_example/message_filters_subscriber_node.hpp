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

#ifndef QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_SUBSCRIBER_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_SUBSCRIBER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// Admitted message_filters form: a two-input ApproximateTime
// synchronizer over two Subscriber members. The analyzer turns each input
// into a subscription (node_analysis/message_filters_scan.py), so RCLLF-H003
// does not fire. The refused counterpart is message_filters_cache_node.
class MessageFiltersSubscriberNode : public rclcpp::Node
{
public:
  explicit MessageFiltersSubscriberNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void sync_callback(
    const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
    const custom_interfaces::msg::SensorData::ConstSharedPtr & b);

  using ApproxTimePolicy = message_filters::sync_policies::ApproximateTime<
    custom_interfaces::msg::SensorData, custom_interfaces::msg::SensorData>;

  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_a_;
  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<ApproxTimePolicy>> sync_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_SUBSCRIBER_NODE_HPP_

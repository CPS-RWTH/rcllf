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

#ifndef QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_CACHE_NODE_HPP_
#define QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_CACHE_NODE_HPP_

#include "custom_interfaces/msg/sensor_data.hpp"
#include "message_filters/cache.h"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"

namespace qos_hazard_example
{

// RCLLF-H003 fixture: a message_filters::Subscriber feeding a
// message_filters::Cache. The analyzer admits only two-input ExactTime and
// ApproximateTime synchronizers, so this chained form stays
// refused and the subscription is never given an LF reaction.
class MessageFiltersCacheNode : public rclcpp::Node
{
public:
  explicit MessageFiltersCacheNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cache_callback(const custom_interfaces::msg::SensorData::ConstSharedPtr & msg);

  message_filters::Subscriber<custom_interfaces::msg::SensorData> sub_;
  message_filters::Cache<custom_interfaces::msg::SensorData> cache_;
};

}  // namespace qos_hazard_example

#endif  // QOS_HAZARD_EXAMPLE__MESSAGE_FILTERS_CACHE_NODE_HPP_

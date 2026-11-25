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

#ifndef PUBSUB_EXAMPLE__SUBSCRIBER_NODE_HPP_
#define PUBSUB_EXAMPLE__SUBSCRIBER_NODE_HPP_

#include <memory>

#include "custom_interfaces/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pubsub_example
{

class SubscriberNode : public rclcpp::Node
{
public:
  explicit SubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void topic_callback(const custom_interfaces::msg::SensorData::SharedPtr msg);
  rclcpp::Subscription<custom_interfaces::msg::SensorData>::SharedPtr subscription_;
};

}  // namespace pubsub_example

#endif  // PUBSUB_EXAMPLE__SUBSCRIBER_NODE_HPP_

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

#ifndef DYNAMIC_TOPIC_EXAMPLE__DYNAMIC_PUBLISHER_NODE_HPP_
#define DYNAMIC_TOPIC_EXAMPLE__DYNAMIC_PUBLISHER_NODE_HPP_

#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dynamic_topic_example
{

/// Publisher whose primary topic name is computed at runtime.
///
/// Fixture for RCLLF-H004 (`topics.dynamic_name`): the `chatter_` publisher's
/// topic argument is a concatenation, which the static analyzer cannot resolve.
/// The `status` publisher uses a plain string literal so the package still
/// contributes one fully resolvable edge to the extracted graph.
class DynamicPublisherNode : public rclcpp::Node
{
public:
  explicit DynamicPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  int64_t instance_id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dynamic_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dynamic_topic_example

#endif  // DYNAMIC_TOPIC_EXAMPLE__DYNAMIC_PUBLISHER_NODE_HPP_

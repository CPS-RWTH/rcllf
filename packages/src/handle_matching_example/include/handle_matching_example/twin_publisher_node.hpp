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

#ifndef HANDLE_MATCHING_EXAMPLE__TWIN_PUBLISHER_NODE_HPP_
#define HANDLE_MATCHING_EXAMPLE__TWIN_PUBLISHER_NODE_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace handle_matching_example
{

/// Publishes 0, 1, 2, ... on "twin" every 100 ms.
class TwinPublisherNode : public rclcpp::Node
{
public:
  explicit TwinPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_;
};

}  // namespace handle_matching_example

#endif  // HANDLE_MATCHING_EXAMPLE__TWIN_PUBLISHER_NODE_HPP_

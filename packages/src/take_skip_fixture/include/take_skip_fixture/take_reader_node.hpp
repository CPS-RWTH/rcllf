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

#ifndef TAKE_SKIP_FIXTURE__TAKE_READER_NODE_HPP_
#define TAKE_SKIP_FIXTURE__TAKE_READER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace take_skip_fixture
{

/// Logs every sample taken on "take" with its tag, and every message_lost
/// report the binding hands it.
class TakeReaderNode : public rclcpp::Node
{
public:
  explicit TakeReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_sample(const std_msgs::msg::Int64::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

}  // namespace take_skip_fixture

#endif  // TAKE_SKIP_FIXTURE__TAKE_READER_NODE_HPP_

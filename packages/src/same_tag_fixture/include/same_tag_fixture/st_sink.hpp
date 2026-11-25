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

#ifndef SAME_TAG_FIXTURE__ST_SINK_HPP_
#define SAME_TAG_FIXTURE__ST_SINK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace same_tag_fixture
{

// Sink Z of the same-tag chain test. /c is a fan-in group of the
// stage (instance "relay", rank 0) and the source (instance "tick", rank 1);
// /d comes from the source alone. Logs every delivery with its value.
class StSink : public rclcpp::Node
{
public:
  explicit StSink(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_c(const std_msgs::msg::Int64 & msg);
  void on_d(const std_msgs::msg::Int64 & msg);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_c_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_d_;
};

}  // namespace same_tag_fixture

#endif  // SAME_TAG_FIXTURE__ST_SINK_HPP_

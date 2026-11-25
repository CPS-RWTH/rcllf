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

#ifndef SAME_TAG_FIXTURE__ST_SOURCE_HPP_
#define SAME_TAG_FIXTURE__ST_SOURCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace same_tag_fixture
{

// Source of the same-tag chain test. One 200 ms timer, ticks 1 to
// 10, then silent. Per tick it publishes, in this order:
//   /a  tick          (to the stage, or the first stage of the chain)
//   /c  2000 + tick   (directly into the sink's /c, a fan-in with the stage)
//   /d  1000 + tick   (a second subscription of the sink)
//   /m  3000 + 10 * tick + k, k = 0, 1, 2  (three records of one tag)
// Every publish is one call site outside any loop, so the analyzer proves the
// burst bound of every edge without an execution config.
class StSource : public rclcpp::Node
{
public:
  explicit StSource(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_tick();

  int tick_{1};

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_a_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_c_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_d_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_m_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace same_tag_fixture

#endif  // SAME_TAG_FIXTURE__ST_SOURCE_HPP_

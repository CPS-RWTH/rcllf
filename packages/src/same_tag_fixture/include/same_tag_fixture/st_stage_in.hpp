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

#ifndef SAME_TAG_FIXTURE__ST_STAGE_IN_HPP_
#define SAME_TAG_FIXTURE__ST_STAGE_IN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace same_tag_fixture
{

// CHAIN variant, first half: /a -> value * 3 -> /b.
class StStageIn : public rclcpp::Node
{
public:
  explicit StStageIn(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_in(const std_msgs::msg::Int64 & msg);

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_b_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_a_;
};

}  // namespace same_tag_fixture

#endif  // SAME_TAG_FIXTURE__ST_STAGE_IN_HPP_

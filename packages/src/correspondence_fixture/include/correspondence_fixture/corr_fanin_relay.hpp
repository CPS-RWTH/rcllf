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

#ifndef CORRESPONDENCE_FIXTURE__CORR_FANIN_RELAY_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_FANIN_RELAY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Shared join fixture, corr_shared_join.launch.py:
// corr_publisher (CORR_SCENARIO=two_publishers) -> /corr/a, /corr/b ->
// corr_fanin_relay -> /corr/j -> corr_relay_sink, and /corr/a -> corr_sub_2.
//
// corr_fanin_relay subscribes /corr/a first and /corr/b second, so in its
// reactor the /corr/b drain comes after the /corr/a drain and sits one LF
// level above the drain of corr_sub_2, at the same tag. It republishes every
// /corr/b sample on /corr/j from that second drain.
class CorrFaninRelay : public rclcpp::Node
{
public:
  explicit CorrFaninRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_a(const std_msgs::msg::Header & msg);
  void on_b(const std_msgs::msg::Header & msg);

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_j_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_a_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_b_;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_FANIN_RELAY_HPP_

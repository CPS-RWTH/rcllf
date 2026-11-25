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

#ifndef CORRESPONDENCE_FIXTURE__CORR_PING_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_PING_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Feedback loop fixture: corr_ping's timer publishes
// /corr/ping, corr_pong answers every ping on /corr/pong, corr_ping logs it.
class CorrPing : public rclcpp::Node
{
public:
  explicit CorrPing(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_tick();
  void on_pong(const std_msgs::msg::Header & msg);

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_ping_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_pong_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t tick_ = 0;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_PING_HPP_

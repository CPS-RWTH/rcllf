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

#ifndef CORRESPONDENCE_FIXTURE__CORR_REENTRANT_PUBLISHER_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_REENTRANT_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Shared-port fixture: two timer callbacks publish /corr/a,
// both in one Reentrant callback group.
// on_a (200 ms) publishes A1..A6 and on_b (310 ms) publishes B1..B4. Both
// skip their first invocation, so under LF (timers start at 0) no two
// publications share a tag: A at 200k ms, B at 310k ms.
class CorrReentrantPublisher : public rclcpp::Node
{
public:
  explicit CorrReentrantPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_a();
  void on_b();

  int tick_a_{0};
  int tick_b_{0};
  rclcpp::CallbackGroup::SharedPtr group_a_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_a_;
  rclcpp::TimerBase::SharedPtr timer_a_;
  rclcpp::TimerBase::SharedPtr timer_b_;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_REENTRANT_PUBLISHER_HPP_

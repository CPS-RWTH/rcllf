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

#ifndef CORRESPONDENCE_FIXTURE__CORR_PUBLISHER_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_PUBLISHER_HPP_

#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Publishes on /corr/a and /corr/b according to CORR_SCENARIO. Exercises the
// publication-correspondence question: which topics a reaction
// (on_tick) actually produces, under fan-out, fan-in, and out-of-band
// publication (the "stale" helper-thread scenario).
class CorrPublisher : public rclcpp::Node
{
public:
  explicit CorrPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CorrPublisher() override;

private:
  void on_tick();

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_a_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_b_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string scenario_;
  int tick_;
  std::thread helper_thread_;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_PUBLISHER_HPP_

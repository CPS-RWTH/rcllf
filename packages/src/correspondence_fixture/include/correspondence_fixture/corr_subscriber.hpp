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

#ifndef CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_HPP_

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Subscribes to both /corr/a and /corr/b. Node name corr_sub_1.
class CorrSubscriber : public rclcpp::Node
{
public:
  explicit CorrSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_a(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info);
  void on_b(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info);
  void on_lost_probe(const std_msgs::msg::Header & msg);
  void log_lost_probe_status();

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_a_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_b_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr lost_probe_;
  size_t lost_probe_total_ = 0;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_HPP_

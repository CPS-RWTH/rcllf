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

#ifndef CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_A_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_A_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Subscribes only to /corr/a. Node name corr_sub_2.
class CorrSubscriberA : public rclcpp::Node
{
public:
  explicit CorrSubscriberA(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_a(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info);

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_a_;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_SUBSCRIBER_A_HPP_

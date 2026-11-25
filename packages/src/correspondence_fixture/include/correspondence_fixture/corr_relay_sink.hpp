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

#ifndef CORRESPONDENCE_FIXTURE__CORR_RELAY_SINK_HPP_
#define CORRESPONDENCE_FIXTURE__CORR_RELAY_SINK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace correspondence_fixture
{

// Relay fixture (W3 slice S3), corr_relay.launch.py:
// corr_publisher_one -> /corr/a -> corr_relay -> /corr/r -> corr_relay_sink.
// Every publishing callback of the chain publishes at most once per
// invocation outside any loop, so the analyzer proves k = 1 and the chain
// needs no execution config.
//
// corr_relay_sink logs every /corr/r sample.
class CorrRelaySink : public rclcpp::Node
{
public:
  explicit CorrRelaySink(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_r(const std_msgs::msg::Header & msg);

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_r_;
};

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__CORR_RELAY_SINK_HPP_

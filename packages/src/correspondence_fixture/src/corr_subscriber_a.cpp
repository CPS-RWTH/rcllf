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

#include "correspondence_fixture/corr_subscriber_a.hpp"

#include <cinttypes>
#include <cstdlib>
#include <string>

namespace correspondence_fixture
{

namespace
{

/// W3: with CORR_DEADLINE_HOOK=1 the subscription registers a
/// requested-deadline-missed callback, through which the binding delivers a
/// missing message at its tag. No deadline QoS is set, so no physical
/// deadline event exists.
rclcpp::SubscriptionOptions missing_hook_options(rclcpp::Node * node)
{
  rclcpp::SubscriptionOptions options;
  const char * hook = std::getenv("CORR_DEADLINE_HOOK");
  if (hook != nullptr && std::string(hook) == "1") {
    options.event_callbacks.deadline_callback =
      [node](rclcpp::QOSDeadlineRequestedInfo & info) {
        RCLCPP_INFO(
          node->get_logger(), "MISSING sub=%s total=%d change=%d", node->get_name(),
          info.total_count, info.total_count_change);
      };
  }
  return options;
}

}  // namespace

CorrSubscriberA::CorrSubscriberA(const rclcpp::NodeOptions & options)
: Node("corr_sub_2", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  sub_a_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/a", qos, std::bind(&CorrSubscriberA::on_a, this, std::placeholders::_1,
    std::placeholders::_2), missing_hook_options(this));
}

void CorrSubscriberA::on_a(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info)
{
  RCLCPP_INFO(
    this->get_logger(), "corr_sub: node=%s topic=/corr/a payload=%s stamp=%d seq=%" PRId64,
    this->get_name(), msg.frame_id.c_str(), msg.stamp.sec,
    info.get_rmw_message_info().publication_sequence_number);
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrSubscriberA>());
  rclcpp::shutdown();
  return 0;
}

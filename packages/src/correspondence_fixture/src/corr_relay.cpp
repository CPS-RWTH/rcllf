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

#include <cstdlib>
#include <string>
#include <thread>

#include "correspondence_fixture/corr_relay.hpp"

namespace correspondence_fixture
{

CorrRelay::CorrRelay(const rclcpp::NodeOptions & options)
: Node("corr_relay", options)
{
  if (const char * w = std::getenv("CORR_RELAY_WORK_US")) {
    work_ = std::chrono::microseconds(std::strtoll(w, nullptr, 10));
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_r_ = this->create_publisher<std_msgs::msg::Header>("/corr/r", qos);
  // No deadline QoS is set, so the handler runs only when the binding
  // delivers a missing /corr/a message, inside the drain reaction.
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo & info) {
      work();
      std_msgs::msg::Header msg;
      msg.frame_id = "missing" + std::to_string(info.total_count);
      pub_r_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "corr_relay: missing payload=%s", msg.frame_id.c_str());
    };
  sub_a_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/a", qos, std::bind(&CorrRelay::on_a, this, std::placeholders::_1), sub_options);
}

void CorrRelay::on_a(const std_msgs::msg::Header & msg)
{
  work();
  std_msgs::msg::Header out = msg;
  out.frame_id = "r-" + msg.frame_id;
  pub_r_->publish(out);
  RCLCPP_INFO(this->get_logger(), "corr_relay: relayed payload=%s", out.frame_id.c_str());
}

void CorrRelay::work() const
{
  if (work_.count() > 0) {
    std::this_thread::sleep_for(work_);
  }
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrRelay>());
  rclcpp::shutdown();
  return 0;
}

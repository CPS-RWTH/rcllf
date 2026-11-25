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

#include "correspondence_fixture/corr_ping.hpp"

#include <chrono>
#include <string>

namespace correspondence_fixture
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

CorrPing::CorrPing(const rclcpp::NodeOptions & options)
: Node("corr_ping", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_ping_ = this->create_publisher<std_msgs::msg::Header>("/corr/ping", qos);
  sub_pong_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/pong", qos, std::bind(&CorrPing::on_pong, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    200ms, std::bind(&CorrPing::on_tick, this));
}

void CorrPing::on_tick()
{
  ++tick_;
  std_msgs::msg::Header msg;
  msg.frame_id = "ping" + std::to_string(tick_);
  msg.stamp.sec = tick_;
  pub_ping_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(), "corr_loop: ping sent payload=%s t=%ld", msg.frame_id.c_str(),
    static_cast<int64_t>(this->now().nanoseconds()));
}

void CorrPing::on_pong(const std_msgs::msg::Header & msg)
{
  // This callback only logs (it publishes nothing), and it is
  // declared after the publishing timer (reaction_order timers_first, the
  // default), so the ping/pong loop has no causality loop: both d = 0 links
  // are admitted and deliver at the publication tag.
  RCLCPP_INFO(
    this->get_logger(), "corr_loop: pong heard payload=%s t=%ld", msg.frame_id.c_str(),
    static_cast<int64_t>(this->now().nanoseconds()));
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrPing>());
  rclcpp::shutdown();
  return 0;
}

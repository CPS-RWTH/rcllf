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

#include "correspondence_fixture/corr_ping_echo.hpp"

#include <chrono>
#include <string>

namespace correspondence_fixture
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

CorrPingEcho::CorrPingEcho(const rclcpp::NodeOptions & options)
: Node("corr_ping_echo", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_ping_ = this->create_publisher<std_msgs::msg::Header>("/corr/ping", qos);
  sub_pong_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/pong", qos, std::bind(&CorrPingEcho::on_pong, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    200ms, std::bind(&CorrPingEcho::on_tick, this));
}

void CorrPingEcho::on_tick()
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

void CorrPingEcho::on_pong(const std_msgs::msg::Header & msg)
{
  RCLCPP_INFO(
    this->get_logger(), "corr_loop: pong heard payload=%s t=%ld", msg.frame_id.c_str(),
    static_cast<int64_t>(this->now().nanoseconds()));
  // The pong to a timer ping is answered with one more ping, so
  // this subscription publishes onward into the loop at the tag it receives.
  // With both links at d = 0 the ping/pong loop is then a causality loop and
  // colcon lf-generate refuses it (launch/corr_loop_echo.launch.py); with
  // one link at d > 0 it runs (launch/corr_loop_echo_delayed.launch.py). The
  // second ping ends in '+' and corr_pong does not answer it, so one pong per
  // tick arrives here (the execution configs declare that 200 ms period).
  if (msg.frame_id.rfind("pong-ping", 0) == 0 && msg.frame_id.back() != '+') {
    std_msgs::msg::Header again = msg;
    again.frame_id = msg.frame_id.substr(5) + "+";
    pub_ping_->publish(again);
    RCLCPP_INFO(
      this->get_logger(), "corr_loop: ping sent payload=%s t=%ld", again.frame_id.c_str(),
      static_cast<int64_t>(this->now().nanoseconds()));
  }
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrPingEcho>());
  rclcpp::shutdown();
  return 0;
}

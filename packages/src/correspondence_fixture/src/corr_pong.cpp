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

#include "correspondence_fixture/corr_pong.hpp"

#include <string>

namespace correspondence_fixture
{

CorrPong::CorrPong(const rclcpp::NodeOptions & options)
: Node("corr_pong", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_pong_ = this->create_publisher<std_msgs::msg::Header>("/corr/pong", qos);
  sub_ping_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/ping", qos, std::bind(&CorrPong::on_ping, this, std::placeholders::_1));
}

void CorrPong::on_ping(const std_msgs::msg::Header & msg)
{
  RCLCPP_INFO(
    this->get_logger(), "corr_loop: ping heard payload=%s t=%ld", msg.frame_id.c_str(),
    static_cast<int64_t>(this->now().nanoseconds()));
  // A ping ending in '+' is corr_ping's answer to a pong and is
  // not answered again, so the zero-delay loop closes once per tick.
  if (msg.frame_id.back() == '+') {
    return;
  }
  std_msgs::msg::Header reply = msg;
  reply.frame_id = "pong-" + msg.frame_id;
  pub_pong_->publish(reply);
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrPong>());
  rclcpp::shutdown();
  return 0;
}

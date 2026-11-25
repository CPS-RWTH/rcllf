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

#include "correspondence_fixture/corr_fanin_relay.hpp"

namespace correspondence_fixture
{

CorrFaninRelay::CorrFaninRelay(const rclcpp::NodeOptions & options)
: Node("corr_fanin_relay", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_j_ = this->create_publisher<std_msgs::msg::Header>("/corr/j", qos);
  sub_a_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/a", qos, std::bind(&CorrFaninRelay::on_a, this, std::placeholders::_1));
  sub_b_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/b", qos, std::bind(&CorrFaninRelay::on_b, this, std::placeholders::_1));
}

void CorrFaninRelay::on_a(const std_msgs::msg::Header & msg)
{
  RCLCPP_INFO(this->get_logger(), "corr_fanin_relay: heard payload=%s", msg.frame_id.c_str());
}

void CorrFaninRelay::on_b(const std_msgs::msg::Header & msg)
{
  std_msgs::msg::Header out = msg;
  out.frame_id = "j-" + msg.frame_id;
  pub_j_->publish(out);
  RCLCPP_INFO(this->get_logger(), "corr_fanin_relay: relayed payload=%s", out.frame_id.c_str());
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrFaninRelay>());
  rclcpp::shutdown();
  return 0;
}

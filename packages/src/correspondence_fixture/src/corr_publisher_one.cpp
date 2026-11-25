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

#include <chrono>
#include <string>

#include "correspondence_fixture/corr_publisher_one.hpp"

namespace correspondence_fixture
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

CorrPublisherOne::CorrPublisherOne(const rclcpp::NodeOptions & options)
: Node("corr_publisher_one", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_a_ = this->create_publisher<std_msgs::msg::Header>("/corr/a", qos);
  timer_ = this->create_wall_timer(200ms, std::bind(&CorrPublisherOne::on_tick, this));
}

void CorrPublisherOne::on_tick()
{
  const int tick = tick_++;
  if (tick > 10) {
    return;
  }
  std_msgs::msg::Header msg;
  msg.frame_id = "t" + std::to_string(tick) + "k1";
  msg.stamp = rclcpp::Time(tick, 0);
  pub_a_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
    msg.frame_id.c_str());
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrPublisherOne>());
  rclcpp::shutdown();
  return 0;
}

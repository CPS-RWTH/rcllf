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

#include "correspondence_fixture/corr_two_group_publisher.hpp"

namespace correspondence_fixture
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

CorrTwoGroupPublisher::CorrTwoGroupPublisher(const rclcpp::NodeOptions & options)
: Node("corr_two_group_publisher", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_a_ = this->create_publisher<std_msgs::msg::Header>("/corr/a", qos);
  group_a_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  group_b_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_a_ = this->create_wall_timer(200ms, std::bind(&CorrTwoGroupPublisher::on_a, this),
      group_a_);
  timer_b_ = this->create_wall_timer(310ms, std::bind(&CorrTwoGroupPublisher::on_b, this),
      group_b_);
}

void CorrTwoGroupPublisher::on_a()
{
  const int tick = tick_a_++;
  if (tick < 1 || tick > 6) {
    return;
  }
  std_msgs::msg::Header msg;
  msg.frame_id = "A" + std::to_string(tick);
  msg.stamp = rclcpp::Time(tick, 0);
  pub_a_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "corr_pub: topic=/corr/a payload=%s", msg.frame_id.c_str());
}

void CorrTwoGroupPublisher::on_b()
{
  const int tick = tick_b_++;
  if (tick < 1 || tick > 4) {
    return;
  }
  std_msgs::msg::Header msg;
  msg.frame_id = "B" + std::to_string(tick);
  msg.stamp = rclcpp::Time(tick, 0);
  pub_a_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "corr_pub: topic=/corr/a payload=%s", msg.frame_id.c_str());
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrTwoGroupPublisher>());
  rclcpp::shutdown();
  return 0;
}

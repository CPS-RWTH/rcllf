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
#include <functional>
#include <memory>

#include "same_tag_fixture/st_source.hpp"

namespace same_tag_fixture
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

StSource::StSource(const rclcpp::NodeOptions & options)
: Node("tick", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_a_ = this->create_publisher<std_msgs::msg::Int64>("/a", qos);
  pub_c_ = this->create_publisher<std_msgs::msg::Int64>("/c", qos);
  pub_d_ = this->create_publisher<std_msgs::msg::Int64>("/d", qos);
  pub_m_ = this->create_publisher<std_msgs::msg::Int64>("/m", qos);
  timer_ = this->create_wall_timer(200ms, std::bind(&StSource::on_tick, this));
}

void StSource::on_tick()
{
  const int tick = tick_++;
  if (tick > 10) {
    return;
  }
  std_msgs::msg::Int64 msg;
  msg.data = tick;
  pub_a_->publish(msg);
  msg.data = 2000 + tick;
  pub_c_->publish(msg);
  msg.data = 1000 + tick;
  pub_d_->publish(msg);
  msg.data = 3000 + 10 * tick;
  pub_m_->publish(msg);
  msg.data = 3000 + 10 * tick + 1;
  pub_m_->publish(msg);
  msg.data = 3000 + 10 * tick + 2;
  pub_m_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "st_source: tick=%d", tick);
}

}  // namespace same_tag_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<same_tag_fixture::StSource>());
  rclcpp::shutdown();
  return 0;
}

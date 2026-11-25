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

#include <functional>
#include <memory>

#include "same_tag_fixture/st_stage_in.hpp"

namespace same_tag_fixture
{

StStageIn::StStageIn(const rclcpp::NodeOptions & options)
: Node("pre", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  pub_b_ = this->create_publisher<std_msgs::msg::Int64>("/b", qos);
  sub_a_ = this->create_subscription<std_msgs::msg::Int64>(
    "/a", qos, std::bind(&StStageIn::on_in, this, std::placeholders::_1));
}

void StStageIn::on_in(const std_msgs::msg::Int64 & msg)
{
  std_msgs::msg::Int64 out;
  out.data = msg.data * 3;
  pub_b_->publish(out);
  RCLCPP_INFO(
    this->get_logger(), "st_stage: node=%s in=%ld out=%ld", this->get_name(),
    static_cast<long>(msg.data), static_cast<long>(out.data));  // NOLINT(runtime/int)
}

}  // namespace same_tag_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<same_tag_fixture::StStageIn>());
  rclcpp::shutdown();
  return 0;
}

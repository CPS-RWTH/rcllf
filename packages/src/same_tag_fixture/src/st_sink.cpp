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

#include "same_tag_fixture/st_sink.hpp"

namespace same_tag_fixture
{

StSink::StSink(const rclcpp::NodeOptions & options)
: Node("sink", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  sub_c_ = this->create_subscription<std_msgs::msg::Int64>(
    "/c", qos, std::bind(&StSink::on_c, this, std::placeholders::_1));
  sub_d_ = this->create_subscription<std_msgs::msg::Int64>(
    "/d", qos, std::bind(&StSink::on_d, this, std::placeholders::_1));
}

void StSink::on_c(const std_msgs::msg::Int64 & msg)
{
  RCLCPP_INFO(
    this->get_logger(), "st_sink: topic=/c value=%ld",
    static_cast<long>(msg.data));  // NOLINT(runtime/int)
}

void StSink::on_d(const std_msgs::msg::Int64 & msg)
{
  RCLCPP_INFO(
    this->get_logger(), "st_sink: topic=/d value=%ld",
    static_cast<long>(msg.data));  // NOLINT(runtime/int)
}

}  // namespace same_tag_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<same_tag_fixture::StSink>());
  rclcpp::shutdown();
  return 0;
}

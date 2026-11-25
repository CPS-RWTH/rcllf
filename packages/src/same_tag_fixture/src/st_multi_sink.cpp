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

#include "same_tag_fixture/st_multi_sink.hpp"

namespace same_tag_fixture
{

StMultiSink::StMultiSink(const rclcpp::NodeOptions & options)
: Node("msink", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  sub_m_ = this->create_subscription<std_msgs::msg::Int64>(
    "/m", qos, std::bind(&StMultiSink::on_m, this, std::placeholders::_1));
}

void StMultiSink::on_m(const std_msgs::msg::Int64 & msg)
{
  RCLCPP_INFO(
    this->get_logger(), "st_msink: topic=/m value=%ld",
    static_cast<long>(msg.data));  // NOLINT(runtime/int)
}

}  // namespace same_tag_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<same_tag_fixture::StMultiSink>());
  rclcpp::shutdown();
  return 0;
}

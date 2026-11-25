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

#include "correspondence_fixture/corr_relay_sink.hpp"

namespace correspondence_fixture
{

CorrRelaySink::CorrRelaySink(const rclcpp::NodeOptions & options)
: Node("corr_relay_sink", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  sub_r_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/r", qos, std::bind(&CorrRelaySink::on_r, this, std::placeholders::_1));
}

void CorrRelaySink::on_r(const std_msgs::msg::Header & msg)
{
  RCLCPP_INFO(this->get_logger(), "corr_relay_sink: payload=%s", msg.frame_id.c_str());
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrRelaySink>());
  rclcpp::shutdown();
  return 0;
}

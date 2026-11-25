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

#include "take_skip_fixture/take_reader_node.hpp"

#include <cinttypes>
#include <functional>
#include <memory>

namespace take_skip_fixture
{

TakeReaderNode::TakeReaderNode(const rclcpp::NodeOptions & options)
: Node("take_reader", options)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.message_lost_callback =
    [this](rclcpp::QOSMessageLostInfo & info) {
      RCLCPP_INFO(
        this->get_logger(), "TSR lost t=%" PRId64 " change=%zu total=%zu",
        this->now().nanoseconds(), static_cast<size_t>(info.total_count_change),
        static_cast<size_t>(info.total_count));
    };
  subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "take", 10, std::bind(&TakeReaderNode::on_sample, this, std::placeholders::_1),
    sub_options);
}

void TakeReaderNode::on_sample(const std_msgs::msg::Int64::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "TSR t=%" PRId64 " v=%" PRId64, this->now().nanoseconds(), msg->data);
}

}  // namespace take_skip_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<take_skip_fixture::TakeReaderNode>());
  rclcpp::shutdown();
  return 0;
}

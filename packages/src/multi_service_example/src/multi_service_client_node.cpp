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

#include "multi_service_example/multi_service_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace multi_service_example
{

MultiServiceClientNode::MultiServiceClientNode()
: Node("multi_service_client"), count_(0)
{
  process_data_client_ = this->create_client<custom_interfaces::srv::ProcessData>("process_data");
  add_two_ints_client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  timer_ = this->create_wall_timer(1s, std::bind(&MultiServiceClientNode::timer_callback, this));
}

void MultiServiceClientNode::timer_callback()
{
  if (!process_data_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service 'process_data'...");
    return;
  }
  if (!add_two_ints_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service 'add_two_ints'...");
    return;
  }

  if (count_ % 2 == 0) {
    auto request = std::make_shared<custom_interfaces::srv::ProcessData::Request>();
    request->values = {1.0, 2.0, 3.0, static_cast<double>(count_)};
    request->method = "sum";
    RCLCPP_INFO(this->get_logger(), "Calling process_data (sum), count=%ld", count_);
    process_data_client_->async_send_request(
      request,
      [this](rclcpp::Client<custom_interfaces::srv::ProcessData>::SharedFuture future) {
        auto resp = future.get();
        RCLCPP_INFO(
          this->get_logger(), "process_data result: %.2f, success=%s, msg='%s'",
          resp->result, resp->success ? "true" : "false", resp->message.c_str());
      });
  } else {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = count_;
    request->b = count_ * 2;
    RCLCPP_INFO(this->get_logger(), "Calling add_two_ints: %ld + %ld", request->a, request->b);
    add_two_ints_client_->async_send_request(
      request,
      [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
        auto resp = future.get();
        RCLCPP_INFO(this->get_logger(), "add_two_ints result: %ld", resp->sum);
      });
  }

  count_++;
}

}  // namespace multi_service_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_service_example::MultiServiceClientNode>());
  rclcpp::shutdown();
  return 0;
}

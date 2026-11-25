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

#include "service_example/service_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace service_example
{

ServiceClientNode::ServiceClientNode(const rclcpp::NodeOptions & options)
: Node("service_client", options), count_(0), sample_values_({1.0, 2.0, 3.0})
{
  client_ = this->create_client<custom_interfaces::srv::ProcessData>("process_data");
  timer_ = this->create_wall_timer(1s, std::bind(&ServiceClientNode::timer_callback, this));
}

void ServiceClientNode::timer_callback()
{
  if (!client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service 'process_data'...");
    return;
  }

  auto request = std::make_shared<custom_interfaces::srv::ProcessData::Request>();
  request->values = sample_values_;
  request->values.push_back(count_ * 0.1);
  request->method = (count_ % 2 == 0) ? "sum" : "average";

  RCLCPP_INFO(
    this->get_logger(), "Sending request: method='%s', %zu values", request->method.c_str(),
    request->values.size());

  auto result_future = client_->async_send_request(
    request, [this](rclcpp::Client<custom_interfaces::srv::ProcessData>::SharedFuture future) {
      auto response = future.get();
      RCLCPP_INFO(
        this->get_logger(), "Result: %.2f, success=%s, message='%s'", response->result,
        response->success ? "true" : "false", response->message.c_str());
    });

  count_++;
}

}  // namespace service_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<service_example::ServiceClientNode>());
  rclcpp::shutdown();
  return 0;
}

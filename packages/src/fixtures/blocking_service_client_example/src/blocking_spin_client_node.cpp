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

#include "blocking_service_client_example/blocking_spin_client_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace blocking_service_client_example
{

BlockingSpinClientNode::BlockingSpinClientNode()
: Node("blocking_spin_client"), count_(0)
{
  client_ = this->create_client<custom_interfaces::srv::ProcessData>("process_data");
  RCLCPP_INFO(this->get_logger(), "BlockingSpinClient ready (spin_until_future_complete style)");
}

bool BlockingSpinClientNode::send_request()
{
  if (!client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service 'process_data'...");
    return false;
  }

  auto request = std::make_shared<custom_interfaces::srv::ProcessData::Request>();
  request->values = {1.0, 2.0, static_cast<double>(count_)};
  request->method = "sum";

  RCLCPP_INFO(this->get_logger(), "Sending request (count=%ld)", count_++);

  // This is the pattern the LF converter recognises but the binding does not support:
  // async_send_request followed by spin_until_future_complete on the SAME node.
  // It works here because the node is not inside a running executor - main drives it.
  auto future = client_->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(this->shared_from_this(), future, 5s);

  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(
      this->get_logger(), "Response (blocking-spin): result=%.2f, success=%s",
      response->result, response->success ? "true" : "false");
    return true;
  }

  RCLCPP_ERROR(this->get_logger(), "Service call timed out or interrupted");
  return false;
}

}  // namespace blocking_service_client_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blocking_service_client_example::BlockingSpinClientNode>();

  // Loop without a spinning executor so that spin_until_future_complete can safely
  // create its own temporary executor and add this node.
  while (rclcpp::ok()) {
    node->send_request();
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  rclcpp::shutdown();
  return 0;
}

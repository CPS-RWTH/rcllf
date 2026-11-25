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

#include "blocking_service_client_example/blocking_future_client_node.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>

using namespace std::chrono_literals;

namespace blocking_service_client_example
{

BlockingFutureClientNode::BlockingFutureClientNode()
: Node("blocking_future_client"), count_(0)
{
  client_ = this->create_client<custom_interfaces::srv::ProcessData>("process_data");
  timer_ = this->create_wall_timer(2s, std::bind(&BlockingFutureClientNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "BlockingFutureClient started (future.wait_for() style)");
}

void BlockingFutureClientNode::timer_callback()
{
  if (!client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service 'process_data'...");
    return;
  }

  auto request = std::make_shared<custom_interfaces::srv::ProcessData::Request>();
  request->values = {3.0, 4.0, static_cast<double>(count_)};
  request->method = "average";

  RCLCPP_INFO(this->get_logger(), "Sending request (count=%ld)", count_++);

  // Blocks this callback thread by waiting on the std::future directly.
  // Requires a MultiThreadedExecutor (2+ threads) so another thread can process
  // the service response while this thread blocks - the pattern the LF binding
  // does not yet support.
  auto future = client_->async_send_request(request);
  auto status = future.wait_for(5s);

  if (status == std::future_status::ready) {
    auto response = future.get();
    RCLCPP_INFO(
      this->get_logger(), "Response (future.wait_for): result=%.2f, success=%s",
      response->result, response->success ? "true" : "false");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service call timed out");
  }
}

}  // namespace blocking_service_client_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blocking_service_client_example::BlockingFutureClientNode>();

  // MultiThreadedExecutor with 2 threads: one runs the timer callback (which blocks on
  // future.wait_for), the other processes the incoming service response so the future
  // becomes ready. This is the minimum setup required for this blocking pattern to work.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

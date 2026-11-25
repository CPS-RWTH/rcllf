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

#include "multi_service_example/multi_service_server_node.hpp"

#include <functional>
#include <memory>
#include <numeric>
#include <string>

namespace multi_service_example
{

MultiServiceServerNode::MultiServiceServerNode()
: Node("multi_service_server")
{
  process_data_service_ = this->create_service<custom_interfaces::srv::ProcessData>(
    "process_data",
    std::bind(
      &MultiServiceServerNode::handle_process_data, this,
      std::placeholders::_1, std::placeholders::_2));

  add_two_ints_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    std::bind(
      &MultiServiceServerNode::handle_add_two_ints, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(), "Multi-service server ready on 'process_data' and 'add_two_ints'");
}

void MultiServiceServerNode::handle_process_data(
  const std::shared_ptr<custom_interfaces::srv::ProcessData::Request> request,
  std::shared_ptr<custom_interfaces::srv::ProcessData::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(), "process_data request: method='%s', %zu values",
    request->method.c_str(), request->values.size());

  if (request->method == "sum") {
    response->result = std::accumulate(request->values.begin(), request->values.end(), 0.0);
    response->success = true;
    response->message = "Computed sum";
  } else if (request->method == "average") {
    if (request->values.empty()) {
      response->result = 0.0;
      response->success = false;
      response->message = "Cannot compute average of empty values";
    } else {
      response->result = std::accumulate(request->values.begin(), request->values.end(), 0.0) /
        static_cast<double>(request->values.size());
      response->success = true;
      response->message = "Computed average";
    }
  } else {
    response->result = 0.0;
    response->success = false;
    response->message = "Unknown method: " + request->method;
  }
}

void MultiServiceServerNode::handle_add_two_ints(
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "add_two_ints request: %ld + %ld", request->a, request->b);
  response->sum = request->a + request->b;
  RCLCPP_INFO(this->get_logger(), "add_two_ints result: %ld", response->sum);
}

}  // namespace multi_service_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_service_example::MultiServiceServerNode>());
  rclcpp::shutdown();
  return 0;
}

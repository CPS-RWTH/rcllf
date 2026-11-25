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

#include "service_example/service_server_node.hpp"

#include <functional>
#include <memory>
#include <numeric>
#include <string>

namespace service_example
{

ServiceServerNode::ServiceServerNode(const rclcpp::NodeOptions & options)
: Node("service_server", options)
{
  service_ = this->create_service<custom_interfaces::srv::ProcessData>(
    "process_data",
    std::bind(
      &ServiceServerNode::handle_process_data, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service server ready on 'process_data'");
}

void ServiceServerNode::handle_process_data(
  const std::shared_ptr<custom_interfaces::srv::ProcessData::Request> request,
  std::shared_ptr<custom_interfaces::srv::ProcessData::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(), "Request: method='%s', %zu values", request->method.c_str(),
    request->values.size());

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

}  // namespace service_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<service_example::ServiceServerNode>());
  rclcpp::shutdown();
  return 0;
}

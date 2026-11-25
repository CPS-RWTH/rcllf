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

#ifndef SERVICE_EXAMPLE__SERVICE_SERVER_NODE_HPP_
#define SERVICE_EXAMPLE__SERVICE_SERVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/srv/process_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace service_example
{

class ServiceServerNode : public rclcpp::Node
{
public:
  explicit ServiceServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_process_data(
    const std::shared_ptr<custom_interfaces::srv::ProcessData::Request> request,
    std::shared_ptr<custom_interfaces::srv::ProcessData::Response> response);

  rclcpp::Service<custom_interfaces::srv::ProcessData>::SharedPtr service_;
};

}  // namespace service_example

#endif  // SERVICE_EXAMPLE__SERVICE_SERVER_NODE_HPP_

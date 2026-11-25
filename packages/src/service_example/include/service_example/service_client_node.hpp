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

#ifndef SERVICE_EXAMPLE__SERVICE_CLIENT_NODE_HPP_
#define SERVICE_EXAMPLE__SERVICE_CLIENT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/srv/process_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace service_example
{

class ServiceClientNode : public rclcpp::Node
{
public:
  explicit ServiceClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Client<custom_interfaces::srv::ProcessData>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_;
  std::vector<double> sample_values_;
};

}  // namespace service_example

#endif  // SERVICE_EXAMPLE__SERVICE_CLIENT_NODE_HPP_

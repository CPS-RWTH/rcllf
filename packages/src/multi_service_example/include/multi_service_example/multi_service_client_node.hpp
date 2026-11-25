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

#ifndef MULTI_SERVICE_EXAMPLE__MULTI_SERVICE_CLIENT_NODE_HPP_
#define MULTI_SERVICE_EXAMPLE__MULTI_SERVICE_CLIENT_NODE_HPP_

#include <memory>
#include <vector>

#include "custom_interfaces/srv/process_data.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace multi_service_example
{

class MultiServiceClientNode : public rclcpp::Node
{
public:
  MultiServiceClientNode();

private:
  void timer_callback();

  rclcpp::Client<custom_interfaces::srv::ProcessData>::SharedPtr process_data_client_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_;
};

}  // namespace multi_service_example

#endif  // MULTI_SERVICE_EXAMPLE__MULTI_SERVICE_CLIENT_NODE_HPP_

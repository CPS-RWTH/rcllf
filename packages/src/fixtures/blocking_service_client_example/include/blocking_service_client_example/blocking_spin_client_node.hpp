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

#ifndef BLOCKING_SERVICE_CLIENT_EXAMPLE__BLOCKING_SPIN_CLIENT_NODE_HPP_
#define BLOCKING_SERVICE_CLIENT_EXAMPLE__BLOCKING_SPIN_CLIENT_NODE_HPP_

#include <memory>

#include "custom_interfaces/srv/process_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blocking_service_client_example
{

// Demonstrates the spin_until_future_complete blocking pattern.
// Called directly from main - no inner executor or timer needed.
class BlockingSpinClientNode : public rclcpp::Node
{
public:
  BlockingSpinClientNode();

  // Sends one request and blocks until the response arrives.
  bool send_request();

private:
  rclcpp::Client<custom_interfaces::srv::ProcessData>::SharedPtr client_;
  int64_t count_;
};

}  // namespace blocking_service_client_example

#endif  // BLOCKING_SERVICE_CLIENT_EXAMPLE__BLOCKING_SPIN_CLIENT_NODE_HPP_

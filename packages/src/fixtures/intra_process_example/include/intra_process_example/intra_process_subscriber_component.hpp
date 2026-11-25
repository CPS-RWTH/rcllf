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

#ifndef INTRA_PROCESS_EXAMPLE__INTRA_PROCESS_SUBSCRIBER_COMPONENT_HPP_
#define INTRA_PROCESS_EXAMPLE__INTRA_PROCESS_SUBSCRIBER_COMPONENT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace intra_process_example
{

class IntraProcessSubscriberComponent : public rclcpp::Node
{
public:
  explicit IntraProcessSubscriberComponent(const rclcpp::NodeOptions & options);

private:
  void message_callback(std::unique_ptr<std_msgs::msg::Int64> msg);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

}  // namespace intra_process_example

#endif  // INTRA_PROCESS_EXAMPLE__INTRA_PROCESS_SUBSCRIBER_COMPONENT_HPP_

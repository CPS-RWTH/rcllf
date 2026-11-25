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

#ifndef REENTRANT_FANOUT_EXAMPLE__FANOUT_WORKER_NODE_HPP_
#define REENTRANT_FANOUT_EXAMPLE__FANOUT_WORKER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace reentrant_fanout_example
{

class FanoutWorkerNode : public rclcpp::Node
{
public:
  FanoutWorkerNode();

private:
  void work_callback(const std_msgs::msg::Int64::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

}  // namespace reentrant_fanout_example

#endif  // REENTRANT_FANOUT_EXAMPLE__FANOUT_WORKER_NODE_HPP_

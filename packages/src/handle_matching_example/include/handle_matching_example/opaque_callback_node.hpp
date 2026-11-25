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

#ifndef HANDLE_MATCHING_EXAMPLE__OPAQUE_CALLBACK_NODE_HPP_
#define HANDLE_MATCHING_EXAMPLE__OPAQUE_CALLBACK_NODE_HPP_

#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace handle_matching_example
{

/// A timer whose callback is a std::function member, and a timer and a
/// subscription whose callback is a conditional between two std::bind forms.
/// Neither names one body at the creation call, so rcllf refuses them
/// (RCLLF-H007).
class OpaqueCallbackNode : public rclcpp::Node
{
public:
  explicit OpaqueCallbackNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_fast();
  void on_slow();
  void on_msg_fast(std_msgs::msg::Int64::SharedPtr msg);
  void on_msg_slow(std_msgs::msg::Int64::SharedPtr msg);

  std::function<void()> tick_;
  bool fast_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr chosen_timer_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr chosen_subscription_;
  int64_t count_;
};

}  // namespace handle_matching_example

#endif  // HANDLE_MATCHING_EXAMPLE__OPAQUE_CALLBACK_NODE_HPP_

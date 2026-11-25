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

/// @file per_subscription_exit_lib.cpp
/// @brief Test library, loaded with dlopen like a component: the
/// subscription it creates gets its shared_ptr control block from code in
/// this library, as a component's subscription does in a ComponentManager.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

extern "C" __attribute__((visibility("default")))
std::shared_ptr<rclcpp::SubscriptionBase> per_subscription_exit_make_sub(
  rclcpp::Node * node, const char * topic)
{
  return node->create_subscription<rosgraph_msgs::msg::Clock>(
    std::string(topic), rclcpp::QoS(10), [](rosgraph_msgs::msg::Clock::ConstSharedPtr) {});
}

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

#ifndef HELPER_METHOD_EXAMPLE__HELPER_NODE_HPP_
#define HELPER_METHOD_EXAMPLE__HELPER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace helper_method_example
{

class HelperNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HelperNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // `override` matters here beyond style: without it, a signature drift
  // against LifecycleNodeInterface would silently declare a new method that
  // the transition machinery never calls, leaving the node in `unconfigured`
  // with none of its resources created.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

private:
  // Helper methods called from on_configure
  void initPubSub();
  void initServices();
  void initParameters();

  // The timer's own callback. Previously the timer was wired straight to
  // initServices() as a "dummy callback", which was harmless only while
  // nothing ever ran on_configure: initServices() declares a parameter, so the
  // second invocation threw ParameterAlreadyDeclaredException and aborted the
  // process. Driving this node to `active` exposed that, so the
  // timer now has a real callback and the init helpers run exactly once.
  void onTimer();

  // Topic name from parameter
  std::string output_topic_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace helper_method_example

#endif  // HELPER_METHOD_EXAMPLE__HELPER_NODE_HPP_

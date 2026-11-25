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

#ifndef PARAMETER_EXAMPLE__PARAMETER_NODE_HPP_
#define PARAMETER_EXAMPLE__PARAMETER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace parameter_example
{

class ParameterNode : public rclcpp::Node
{
public:
  explicit ParameterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace parameter_example

#endif  // PARAMETER_EXAMPLE__PARAMETER_NODE_HPP_

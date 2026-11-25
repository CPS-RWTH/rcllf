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

#include "parameter_example/parameter_node.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace parameter_example
{

ParameterNode::ParameterNode(const rclcpp::NodeOptions & options)
: Node("parameter_node", options)
{
  this->declare_parameter<std::string>("robot_name", "default_robot");
  this->declare_parameter<double>("max_speed", 1.0);
  this->declare_parameter<int>("update_rate", 10);
  this->declare_parameter<bool>("debug_mode", false);

  std::string robot_name = this->get_parameter("robot_name").as_string();
  double max_speed = this->get_parameter("max_speed").as_double();
  int update_rate = this->get_parameter("update_rate").as_int();
  bool debug_mode = this->get_parameter("debug_mode").as_bool();

  RCLCPP_INFO(this->get_logger(), "robot_name: %s", robot_name.c_str());
  RCLCPP_INFO(this->get_logger(), "max_speed: %f", max_speed);
  RCLCPP_INFO(this->get_logger(), "update_rate: %d", update_rate);
  RCLCPP_INFO(this->get_logger(), "debug_mode: %s", debug_mode ? "true" : "false");

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ParameterNode::on_parameter_change, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(1000ms, std::bind(&ParameterNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Parameter node started.");
}

void ParameterNode::timer_callback()
{
  std::string robot_name = this->get_parameter("robot_name").as_string();
  double max_speed = this->get_parameter("max_speed").as_double();
  RCLCPP_INFO(this->get_logger(), "Robot: %s, Max speed: %f", robot_name.c_str(), max_speed);
}

rcl_interfaces::msg::SetParametersResult ParameterNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    RCLCPP_INFO(
      this->get_logger(), "Parameter changed: %s = %s", param.get_name().c_str(),
      param.value_to_string().c_str());
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace parameter_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<parameter_example::ParameterNode>());
  rclcpp::shutdown();
  return 0;
}

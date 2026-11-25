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

// Headerless parameter fixture: the class lives only in this
// .cpp, so the lfc generator builds it through the appended headerless
// factory instead of node_factories.cpp's header path. It declares three
// parameters, logs them at startup and publishes them on `value`.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HeaderlessParamNode : public rclcpp::Node
{
public:
  explicit HeaderlessParamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("headerless_param_node", options)
  {
    this->declare_parameter<std::string>("label", "default_label");
    this->declare_parameter<double>("gain", 0.5);
    this->declare_parameter<int>("count", 1);
    RCLCPP_INFO(this->get_logger(), "headerless params: %s", text().c_str());
    publisher_ = this->create_publisher<std_msgs::msg::String>("value", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&HeaderlessParamNode::on_timer, this));
  }

private:
  std::string text() const
  {
    return "label=" + this->get_parameter("label").as_string() +
           " gain=" + std::to_string(this->get_parameter("gain").as_double()) +
           " count=" + std::to_string(this->get_parameter("count").as_int());
  }

  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = text();
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeaderlessParamNode>());
  rclcpp::shutdown();
  return 0;
}

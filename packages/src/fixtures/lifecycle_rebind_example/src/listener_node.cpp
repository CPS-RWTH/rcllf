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

// A plain node that reports every value the other nodes publish, so the
// federated run can be checked from another federate.

#include <functional>
#include <memory>

#include "lifecycle_rebind_example/rebind_common.hpp"

namespace lifecycle_rebind_example
{

class ListenerNode : public rclcpp::Node
{
public:
  explicit ListenerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("listener", options)
  {
    side_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/rebind/side", 10,
      std::bind(&ListenerNode::on_side, this, std::placeholders::_1));
    tick_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/rebind/sleeper_tick", 10,
      std::bind(&ListenerNode::on_tick, this, std::placeholders::_1));
    heard_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/rebind/heard", 10,
      std::bind(&ListenerNode::on_heard, this, std::placeholders::_1));
  }

private:
  void on_side(const std_msgs::msg::Int64::SharedPtr msg) {report(this, "rx_side", msg->data);}
  void on_tick(const std_msgs::msg::Int64::SharedPtr msg) {report(this, "rx_tick", msg->data);}
  void on_heard(const std_msgs::msg::Int64::SharedPtr msg) {report(this, "rx_heard", msg->data);}

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr side_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr tick_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr heard_sub_;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lifecycle_rebind_example::ListenerNode>());
  rclcpp::shutdown();
  return 0;
}

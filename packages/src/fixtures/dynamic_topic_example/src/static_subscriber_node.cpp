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

#include "dynamic_topic_example/static_subscriber_node.hpp"

#include <functional>

namespace dynamic_topic_example
{

StaticSubscriberNode::StaticSubscriberNode(const rclcpp::NodeOptions & options)
: Node("static_subscriber", options)
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "status", 10,
    std::bind(&StaticSubscriberNode::topic_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Static subscriber node started.");
}

void StaticSubscriberNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace dynamic_topic_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamic_topic_example::StaticSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

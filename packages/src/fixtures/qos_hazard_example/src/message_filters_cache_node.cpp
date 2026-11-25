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

#include "qos_hazard_example/message_filters_cache_node.hpp"

#include <functional>
#include <memory>

namespace qos_hazard_example
{

MessageFiltersCacheNode::MessageFiltersCacheNode(const rclcpp::NodeOptions & options)
: Node("message_filters_cache", options)
{
  sub_.subscribe(this, "sync_topic_a");
  cache_.setCacheSize(10);
  cache_.connectInput(sub_);
  cache_.registerCallback(
    std::bind(&MessageFiltersCacheNode::cache_callback, this, std::placeholders::_1));
}

void MessageFiltersCacheNode::cache_callback(
  const custom_interfaces::msg::SensorData::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "cached: temp=%.1f", msg->temperature);
}

}  // namespace qos_hazard_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qos_hazard_example::MessageFiltersCacheNode>());
  rclcpp::shutdown();
  return 0;
}

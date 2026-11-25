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

// The reader of reader.launch.py with its twin topic read from the
// declared parameter twin_topic instead of an environment variable.
// reader_param.launch.py sets it to /rebind/feed, so the node holds a second
// subscription on the topic that the analysis can resolve from the launch file.

#include <functional>
#include <memory>
#include <string>

#include "lifecycle_rebind_example/rebind_common.hpp"

namespace lifecycle_rebind_example
{

class ParamReaderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ParamReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("reader", options)
  {
    twin_topic_ = this->declare_parameter<std::string>("twin_topic", "");
    g_reader = this;
  }

  ~ParamReaderNode() override
  {
    g_reader = nullptr;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    feed_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/rebind/feed", 10,
      std::bind(&ParamReaderNode::on_feed, this, std::placeholders::_1));
    if (!twin_topic_.empty()) {
      twin_sub_ = this->create_subscription<std_msgs::msg::Int64>(
        twin_topic_, 10,
        std::bind(&ParamReaderNode::on_twin, this, std::placeholders::_1));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    feed_sub_.reset();
    twin_sub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void on_feed(const std_msgs::msg::Int64::SharedPtr msg) {report(this, "rx", msg->data);}
  void on_twin(const std_msgs::msg::Int64::SharedPtr msg) {report(this, "rx_twin", msg->data);}

  std::string twin_topic_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr feed_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr twin_sub_;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // No transition here: the driver moves the node.
  auto node = std::make_shared<lifecycle_rebind_example::ParamReaderNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

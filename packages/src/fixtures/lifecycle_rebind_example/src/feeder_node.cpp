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

// A lifecycle publisher for reader.launch.py. Tick k publishes k on
// /rebind/feed at (k - 1) * 100 ms, from activation at startup to the end.

#include <chrono>
#include <functional>
#include <memory>

#include "lifecycle_rebind_example/rebind_common.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class FeederNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FeederNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("feeder", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    feed_pub_ = this->create_publisher<std_msgs::msg::Int64>("/rebind/feed", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&FeederNode::tick, this));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

private:
  void tick()
  {
    ++count_;
    std_msgs::msg::Int64 msg;
    msg.data = count_;
    feed_pub_->publish(msg);
    report(this, "feed", count_);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr feed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_ = 0;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lifecycle_rebind_example::FeederNode>();
  node->configure();
  node->activate();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

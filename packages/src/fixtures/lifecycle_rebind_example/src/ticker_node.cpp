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

// A lifecycle node that deactivates itself after five ticks and keeps its
// timer. Each tick counts and reports, a side effect the lifecycle
// does not gate (its publisher does drop while inactive). Expected: 1 to 5.

#include <chrono>
#include <functional>
#include <memory>

#include "lifecycle_rebind_example/rebind_common.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class TickerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TickerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("ticker", options)
  {
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    side_pub_ = this->create_publisher<std_msgs::msg::Int64>("/rebind/side", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TickerNode::tick, this));
    return CallbackReturn::SUCCESS;
  }

  // The timer is kept on purpose. Only the runtime can stop it from running.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

private:
  void tick()
  {
    ++count_;
    std_msgs::msg::Int64 msg;
    msg.data = count_;
    side_pub_->publish(msg);
    report(this, "ticker", count_);
    if (count_ == 5) {
      this->deactivate();
    }
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr side_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_ = 0;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lifecycle_rebind_example::TickerNode>();
  node->configure();
  node->activate();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

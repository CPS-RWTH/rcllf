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

// A plain node that drives the sleeper from its own timer. Tick k
// publishes k on /rebind/cmd. Tick 3 configures and activates the sleeper,
// tick 8 deactivates it, tick 12 activates it again.

#include <chrono>
#include <functional>
#include <memory>

#include "lifecycle_rebind_example/rebind_common.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class SupervisorNode : public rclcpp::Node
{
public:
  explicit SupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("supervisor", options)
  {
    cmd_pub_ = this->create_publisher<std_msgs::msg::Int64>("/rebind/cmd", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&SupervisorNode::tick, this));
  }

private:
  void tick()
  {
    ++count_;
    std_msgs::msg::Int64 msg;
    msg.data = count_;
    cmd_pub_->publish(msg);
    report(this, "cmd", count_);
    if (g_sleeper == nullptr) {
      return;
    }
    if (count_ == 3) {
      g_sleeper->configure();
      g_sleeper->activate();
    } else if (count_ == 8) {
      g_sleeper->deactivate();
    } else if (count_ == 12) {
      g_sleeper->activate();
    } else {
      return;
    }
    report(this, "sleeper_state", g_sleeper->get_current_state().id());
  }

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_ = 0;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lifecycle_rebind_example::SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}

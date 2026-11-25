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
// A plain node that serves the sleeper's requests and goals. It
// answers a request at once and holds every goal open until a cancel request
// comes, which it accepts and completes at its next tick. Ticks 7, 11 and 21
// (600, 1000 and 2000 ms) report how many goals it holds.

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "example_interfaces/action/fibonacci.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "lifecycle_rebind_example/rebind_common.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class ResponderNode : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit ResponderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("responder", options)
  {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "/rebind/add",
      std::bind(&ResponderNode::on_add, this, std::placeholders::_1, std::placeholders::_2));
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this, "/rebind/fib",
      std::bind(&ResponderNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ResponderNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ResponderNode::handle_accepted, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&ResponderNode::tick, this));
  }

private:
  void on_add(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    report(this, "responder_add", response->sum);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal)
  {
    goals_.push_back(goal);
  }

  void tick()
  {
    ++ticks_;
    for (auto it = goals_.begin(); it != goals_.end(); ) {
      if ((*it)->is_canceling()) {
        (*it)->canceled(std::make_shared<Fibonacci::Result>());
        it = goals_.erase(it);
      } else {
        ++it;
      }
    }
    if (ticks_ == 7 || ticks_ == 11 || ticks_ == 21) {
      report(this, "responder_goals", static_cast<int64_t>(goals_.size()));
    }
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::shared_ptr<GoalHandle>> goals_;
  int64_t ticks_ = 0;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lifecycle_rebind_example::ResponderNode>());
  rclcpp::shutdown();
  return 0;
}

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

// A lifecycle node whose activation belongs to the supervisor.
// Its subscription exists from the constructor on, each activation makes
// a new timer, and deactivate keeps both. Timer ticks publish
// generation * 1000 + tick, the subscription republishes the command it heard.
// A second timer, the pair timer, is made on activate and dropped on
// deactivate. At 700 ms its reaction and the supervisor's deactivate share a
// tag in two reactors with no order between them, and it must report its tick
// whichever runs first.
//
// A service client and an action client. The call timer asks
// the responder at its fourth and fifth tick of each activation. The fifth
// answer arrives after the deactivation, so deactivate must prune it, and the
// first tick of the next activation reports how many earlier requests are
// still pending. The goal timer sends one goal per activation, which the
// responder holds open until deactivate cancels it.
//
// The cancel and result responses of that goal arrive while the
// node is inactive. The client must still release the goal handle, so the
// first call tick of the next activation reports whether the earlier goal's
// handle is still held.

#include <chrono>
#include <functional>
#include <memory>

#include <future>
#include <vector>

#include "example_interfaces/action/fibonacci.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "lifecycle_rebind_example/rebind_common.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class SleeperNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SleeperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("sleeper", options)
  {
    tick_pub_ = this->create_publisher<std_msgs::msg::Int64>("/rebind/sleeper_tick", 10);
    heard_pub_ = this->create_publisher<std_msgs::msg::Int64>("/rebind/heard", 10);
    cmd_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/rebind/cmd", 10,
      std::bind(&SleeperNode::on_cmd, this, std::placeholders::_1));
    adder_ = this->create_client<example_interfaces::srv::AddTwoInts>("/rebind/add");
    fib_ = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(
      this, "/rebind/fib");
    g_sleeper = this;
  }

  ~SleeperNode() override
  {
    g_sleeper = nullptr;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    ++generation_;
    ticks_ = 0;
    timer_ = this->create_wall_timer(100ms, std::bind(&SleeperNode::tick, this));
    pair_ticks_ = 0;
    pair_timer_ = this->create_wall_timer(100ms, std::bind(&SleeperNode::pair_tick, this));
    calls_ = 0;
    call_timer_ = this->create_wall_timer(100ms, std::bind(&SleeperNode::call, this));
    goal_sent_ = false;
    goal_timer_ = this->create_wall_timer(100ms, std::bind(&SleeperNode::send_goal, this));
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

  // Timer and subscription are kept on purpose. The pair timer is dropped.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    pair_timer_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

private:
  void tick()
  {
    ++ticks_;
    std_msgs::msg::Int64 msg;
    msg.data = generation_ * 1000 + ticks_;
    tick_pub_->publish(msg);
    report(this, "sleeper_tick", msg.data);
  }

  void pair_tick()
  {
    ++pair_ticks_;
    report(this, "sleeper_pair", generation_ * 1000 + pair_ticks_);
  }

  void call()
  {
    ++calls_;
    if (calls_ == 1) {
      // A request of an earlier activation is answered or pruned by now.
      int64_t stale = 0;
      for (const auto & f : requests_) {
        if (f.wait_for(0s) != std::future_status::ready) {
          ++stale;
        }
      }
      requests_.clear();
      report(this, "sleeper_stale", stale);
      report(this, "sleeper_goal_held", goal_.expired() ? 0 : 1);
      goal_.reset();
    }
    if (calls_ == 4 || calls_ == 5) {
      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = generation_ * 1000 + calls_;
      request->b = 0;
      requests_.push_back(
        adder_->async_send_request(
          request, std::bind(&SleeperNode::on_reply, this, std::placeholders::_1)).future);
    }
  }

  void on_reply(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    report(this, "sleeper_reply", future.get()->sum);
  }

  void send_goal()
  {
    if (goal_sent_) {
      return;
    }
    goal_sent_ = true;
    example_interfaces::action::Fibonacci::Goal goal;
    goal.order = static_cast<int32_t>(generation_);
    auto options = rclcpp_action::Client<example_interfaces::action::Fibonacci>::SendGoalOptions();
    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr
      handle) {goal_ = handle;};
    options.result_callback = std::bind(&SleeperNode::on_result, this, std::placeholders::_1);
    fib_->async_send_goal(goal, options);
  }

  // The responder never finishes a goal, so a result is a cancelled goal
  // that reached user code.
  void on_result(
    const rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult &
    result)
  {
    report(this, "sleeper_result", static_cast<int64_t>(result.code));
  }

  void on_cmd(const std_msgs::msg::Int64::SharedPtr msg)
  {
    heard_pub_->publish(*msg);
    report(this, "heard", msg->data);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr tick_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr heard_pub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pair_timer_;
  rclcpp::TimerBase::SharedPtr call_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr adder_;
  rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr fib_;
  std::vector<rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture> requests_;
  std::weak_ptr<rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>> goal_;
  int64_t generation_ = 0;
  int64_t ticks_ = 0;
  int64_t pair_ticks_ = 0;
  int64_t calls_ = 0;
  bool goal_sent_ = false;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // No transition here: the supervisor drives the node.
  auto node = std::make_shared<lifecycle_rebind_example::SleeperNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

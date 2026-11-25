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

#include "action_example/action_dual_server_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace action_example
{

ActionDualServerNode::ActionDualServerNode(const rclcpp::NodeOptions & options)
: Node("dual_server", options)
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  server_a_ = rclcpp_action::create_server<Analyze>(
    this, "analyze_a", std::bind(&ActionDualServerNode::handle_goal_a, this, _1, _2),
    std::bind(&ActionDualServerNode::handle_cancel_a, this, _1),
    std::bind(&ActionDualServerNode::handle_accepted_a, this, _1));
  server_b_ = rclcpp_action::create_server<Analyze>(
    this, "analyze_b", std::bind(&ActionDualServerNode::handle_goal_b, this, _1, _2),
    std::bind(&ActionDualServerNode::handle_cancel_b, this, _1),
    std::bind(&ActionDualServerNode::handle_accepted_b, this, _1));

  step_timer_ =
    this->create_wall_timer(100ms, std::bind(&ActionDualServerNode::step_goals, this));

  // Same shutdown guard as ActionServerNode.
  pre_shutdown_handle_ = this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [this]() {abort_active_goals();});

  RCLCPP_INFO(this->get_logger(), "Action servers ready on 'analyze_a' and 'analyze_b'");
}

ActionDualServerNode::~ActionDualServerNode()
{
  this->get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
    pre_shutdown_handle_);
  abort_active_goals();
}

void ActionDualServerNode::step_goals()
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  for (auto * goals : {&goals_a_, &goals_b_}) {
    const char server = goals == &goals_a_ ? 'a' : 'b';
    std::vector<ActiveGoal> running;
    for (auto & g : *goals) {
      if (g.handle->is_canceling()) {
        g.result->success = false;
        g.handle->canceled(g.result);
        RCLCPP_INFO(this->get_logger(), "Goal on analyze_%c canceled", server);
        continue;
      }
      if (g.next_sample < g.num_samples) {
        const int i = g.next_sample++;
        auto feedback = std::make_shared<Analyze::Feedback>();
        feedback->samples_processed = i;
        feedback->progress_percentage = 100.0 * i / g.num_samples;
        g.handle->publish_feedback(feedback);
        g.result->results.push_back(i * 1.1);
        running.push_back(g);
        continue;
      }
      g.result->success = true;
      g.handle->succeed(g.result);
      RCLCPP_INFO(this->get_logger(), "Goal on analyze_%c succeeded", server);
    }
    goals->swap(running);
  }
}

void ActionDualServerNode::abort_active_goals()
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  stopping_ = true;
  for (auto * goals : {&goals_a_, &goals_b_}) {
    for (auto & g : *goals) {
      if (g.handle->is_active()) {
        g.result->success = false;
        g.handle->abort(g.result);
      }
    }
    goals->clear();
  }
}

rclcpp_action::GoalResponse ActionDualServerNode::handle_goal_a(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "analyze_a received goal: num_samples=%d", goal->num_samples);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse ActionDualServerNode::handle_goal_b(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "analyze_b received goal: num_samples=%d", goal->num_samples);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionDualServerNode::handle_cancel_a(
  const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse ActionDualServerNode::handle_cancel_b(
  const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionDualServerNode::handle_accepted_a(const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto result = std::make_shared<Analyze::Result>();
  if (stopping_) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }
  goals_a_.push_back({goal_handle, goal_handle->get_goal()->num_samples, 0, result});
}

void ActionDualServerNode::handle_accepted_b(const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto result = std::make_shared<Analyze::Result>();
  if (stopping_) {
    result->success = false;
    goal_handle->abort(result);
    return;
  }
  goals_b_.push_back({goal_handle, goal_handle->get_goal()->num_samples, 0, result});
}

}  // namespace action_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_example::ActionDualServerNode>());
  rclcpp::shutdown();
  return 0;
}

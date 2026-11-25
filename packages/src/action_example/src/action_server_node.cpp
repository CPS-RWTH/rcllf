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

#include "action_example/action_server_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace action_example
{

ActionServerNode::ActionServerNode(const rclcpp::NodeOptions & options)
: Node("action_server", options)
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  action_server_ = rclcpp_action::create_server<Analyze>(
    this, "analyze", std::bind(&ActionServerNode::handle_goal, this, _1, _2),
    std::bind(&ActionServerNode::handle_cancel, this, _1),
    std::bind(&ActionServerNode::handle_accepted, this, _1));

  step_timer_ = this->create_wall_timer(100ms, std::bind(&ActionServerNode::step_goals, this));

  // rclcpp_action throws if a goal handle ends or publishes after
  // the context shut down while the server lives. End the goals before the
  // context goes down: pre-shutdown callbacks run on Ctrl-C and on
  // rclcpp::shutdown(). Under lfc the binding runs this callback in the node's
  // generated reaction(shutdown), so the abort results are sent at the shutdown
  // tag.
  pre_shutdown_handle_ = this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [this]() {abort_active_goals();});

  RCLCPP_INFO(this->get_logger(), "Action server ready on 'analyze'");
}

ActionServerNode::~ActionServerNode()
{
  // lfc may release the node while the context is still valid (LF stop
  // without a signal), so end the goals here too.
  this->get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
    pre_shutdown_handle_);
  abort_active_goals();
}

/// One step of every active goal, in acceptance order: a canceling goal ends
/// canceled, a goal with samples left publishes the next one, and a goal with
/// all samples done succeeds.
void ActionServerNode::step_goals()
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  if (goals_.empty()) {
    return;
  }
  std::vector<ActiveGoal> running;
  for (auto & g : goals_) {
    if (g.handle->is_canceling()) {
      g.result->success = false;
      g.handle->canceled(g.result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      continue;
    }
    if (g.next_sample < g.num_samples) {
      const int i = g.next_sample++;
      auto feedback = std::make_shared<Analyze::Feedback>();
      feedback->samples_processed = i;
      feedback->progress_percentage = 100.0 * i / g.num_samples;
      g.handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Progress: %.1f%%", feedback->progress_percentage);
      g.result->results.push_back(i * 1.1);
      running.push_back(g);
      continue;
    }
    g.result->success = true;
    g.handle->succeed(g.result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
  goals_.swap(running);
}

void ActionServerNode::abort_active_goals()
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  stopping_ = true;
  for (auto & g : goals_) {
    if (g.handle->is_active()) {
      g.result->success = false;
      g.handle->abort(g.result);
      RCLCPP_INFO(this->get_logger(), "Goal aborted: shutting down");
    }
  }
  goals_.clear();
}

rclcpp_action::GoalResponse ActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    this->get_logger(), "Received goal: num_samples=%d, analysis_type='%s'", goal->num_samples,
    goal->analysis_type.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerNode::handle_cancel(
  const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerNode::handle_accepted(const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto result = std::make_shared<Analyze::Result>();
  if (stopping_) {
    // Accepted after shutdown began: end the goal here, never leave it active.
    result->success = false;
    goal_handle->abort(result);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  goals_.push_back({goal_handle, goal_handle->get_goal()->num_samples, 0, result});
}

}  // namespace action_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_example::ActionServerNode>());
  rclcpp::shutdown();
  return 0;
}

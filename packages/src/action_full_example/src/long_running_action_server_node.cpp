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

#include "action_full_example/long_running_action_server_node.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace action_full_example
{

LongRunningActionServerNode::LongRunningActionServerNode()
: Node("long_running_action_server")
{
  this->declare_parameter("reject_goals", false);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<Analyze>(
    this, "analyze", std::bind(&LongRunningActionServerNode::handle_goal, this, _1, _2),
    std::bind(&LongRunningActionServerNode::handle_cancel, this, _1),
    std::bind(&LongRunningActionServerNode::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Long-running action server ready on 'analyze'");
}

rclcpp_action::GoalResponse LongRunningActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal)
{
  (void)uuid;
  bool reject = this->get_parameter("reject_goals").as_bool();
  RCLCPP_INFO(
    this->get_logger(), "Received goal: num_samples=%d, type='%s', reject_goals=%s",
    goal->num_samples, goal->analysis_type.c_str(), reject ? "true" : "false");
  if (reject) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal (reject_goals=true)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LongRunningActionServerNode::handle_cancel(
  const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request -- accepting");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LongRunningActionServerNode::handle_accepted(
  const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  current_feedback_ = std::make_shared<Analyze::Feedback>();
  current_feedback_->samples_processed = 0;
  current_feedback_->progress_percentage = 0.0;

  // Feedback timer fires at 5 Hz, decoupled from the 10 Hz execution loop.
  feedback_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    [this, goal_handle]() {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (goal_handle->is_active()) {
        goal_handle->publish_feedback(current_feedback_);
        RCLCPP_INFO(
          this->get_logger(), "Feedback: progress=%.1f%%",
          current_feedback_->progress_percentage);
      }
    });

  using namespace std::placeholders;
  std::thread{std::bind(&LongRunningActionServerNode::execute, this, _1), goal_handle}.detach();
}

void LongRunningActionServerNode::execute(const std::shared_ptr<GoalHandleAnalyze> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal (50 steps at 10 Hz)");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Analyze::Result>();

  rclcpp::Rate loop_rate(10);
  const int steps = goal->num_samples;

  for (int i = 0; i < steps && rclcpp::ok(); ++i) {
    if (goal_handle->is_canceling()) {
      feedback_timer_->cancel();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled at step %d/%d", i, steps);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      current_feedback_->samples_processed = i;
      current_feedback_->progress_percentage = 100.0 * i / steps;
    }

    result->results.push_back(i * 1.1);
    loop_rate.sleep();
  }

  feedback_timer_->cancel();

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

}  // namespace action_full_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_full_example::LongRunningActionServerNode>());
  rclcpp::shutdown();
  return 0;
}

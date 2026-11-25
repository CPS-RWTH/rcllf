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

#include "action_full_example/action_full_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace action_full_example
{

// Goal sequence:
//   0 -> Goal A: let it complete normally
//   1 -> Goal B: cancel mid-flight after 2 s
//   2 -> Goal C: server has reject_goals=true -> expect rejection
static const char * kGoalTypes[] = {"complete", "cancel_me", "rejected"};

ActionFullClientNode::ActionFullClientNode()
: Node("action_full_client"), goal_index_(0)
{
  action_client_ = rclcpp_action::create_client<Analyze>(this, "analyze");
  start_timer_ = this->create_wall_timer(
    2s, std::bind(&ActionFullClientNode::send_next_goal, this));
}

void ActionFullClientNode::send_next_goal()
{
  start_timer_->cancel();

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after 5 s");
    return;
  }

  if (goal_index_ >= 3) {
    RCLCPP_INFO(this->get_logger(), "All three goals done -- shutting down");
    rclcpp::shutdown();
    return;
  }

  // Before Goal C, set reject_goals=true on the server via parameter service.
  if (goal_index_ == 2) {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      this->shared_from_this(), "long_running_action_server");
    if (param_client->wait_for_service(3s)) {
      param_client->set_parameters({rclcpp::Parameter("reject_goals", true)});
      RCLCPP_INFO(this->get_logger(), "Set reject_goals=true on server for Goal C");
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter service unavailable; Goal C may not be rejected");
    }
  }

  auto goal_msg = Analyze::Goal();
  goal_msg.num_samples = 50;
  goal_msg.analysis_type = kGoalTypes[goal_index_];

  RCLCPP_INFO(
    this->get_logger(), "Sending Goal %c (index=%d, type='%s')",
    'A' + goal_index_, goal_index_, goal_msg.analysis_type.c_str());

  auto send_goal_options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ActionFullClientNode::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &ActionFullClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionFullClientNode::result_callback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionFullClientNode::goal_response_callback(const GoalHandleAnalyze::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Goal %c was REJECTED by server", 'A' + goal_index_);
    goal_index_++;
    start_timer_ = this->create_wall_timer(
      1s, std::bind(&ActionFullClientNode::send_next_goal, this));
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Goal %c accepted", 'A' + goal_index_);
  active_goal_handle_ = goal_handle;

  // For Goal B (index 1): cancel after 2 s.
  if (goal_index_ == 1) {
    cancel_timer_ = this->create_wall_timer(
      2s, std::bind(&ActionFullClientNode::cancel_timer_callback, this));
  }
}

void ActionFullClientNode::cancel_timer_callback()
{
  cancel_timer_->cancel();
  if (active_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Sending cancel for Goal B");
    action_client_->async_cancel_goal(active_goal_handle_);
  }
}

void ActionFullClientNode::feedback_callback(
  GoalHandleAnalyze::SharedPtr, const std::shared_ptr<const Analyze::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Feedback: samples=%d, progress=%.1f%%",
    feedback->samples_processed, feedback->progress_percentage);
}

void ActionFullClientNode::result_callback(const GoalHandleAnalyze::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal %c SUCCEEDED", 'A' + goal_index_);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal %c CANCELED", 'A' + goal_index_);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal %c ABORTED", 'A' + goal_index_);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Goal %c unknown result", 'A' + goal_index_);
      break;
  }

  goal_index_++;
  active_goal_handle_.reset();
  start_timer_ = this->create_wall_timer(
    1s, std::bind(&ActionFullClientNode::send_next_goal, this));
}

}  // namespace action_full_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_full_example::ActionFullClientNode>());
  rclcpp::shutdown();
  return 0;
}

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

#include "action_example/action_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace action_example
{

ActionClientNode::ActionClientNode(const rclcpp::NodeOptions & options)
: Node("action_client", options)
{
  action_client_ = rclcpp_action::create_client<Analyze>(this, "analyze");
  timer_ = this->create_wall_timer(2s, std::bind(&ActionClientNode::send_goal, this));
}

void ActionClientNode::send_goal()
{
  timer_->cancel();
  // One goal per run, also on a runtime that ignores cancel().
  if (goal_sent_) {
    return;
  }

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }

  auto goal_msg = Analyze::Goal();
  goal_msg.num_samples = 10;
  goal_msg.analysis_type = "statistical";

  RCLCPP_INFO(
    this->get_logger(), "Sending goal: num_samples=%d, analysis_type='%s'", goal_msg.num_samples,
    goal_msg.analysis_type.c_str());

  auto send_goal_options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ActionClientNode::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &ActionClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionClientNode::result_callback, this, std::placeholders::_1);

  goal_sent_ = true;
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionClientNode::goal_response_callback(const GoalHandleAnalyze::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, will cancel in 3s");
  goal_handle_ = goal_handle;
  cancel_timer_ =
    this->create_wall_timer(3s, std::bind(&ActionClientNode::cancel_timer_callback, this));
}

void ActionClientNode::cancel_timer_callback()
{
  cancel_timer_->cancel();
  if (goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Sending cancel request");
    action_client_->async_cancel_goal(goal_handle_);
  }
}

void ActionClientNode::feedback_callback(
  GoalHandleAnalyze::SharedPtr, const std::shared_ptr<const Analyze::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Feedback: samples_processed=%d, progress=%.1f%%",
    feedback->samples_processed, feedback->progress_percentage);
}

void ActionClientNode::result_callback(const GoalHandleAnalyze::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  std::stringstream ss;
  ss << "Result: success=" << (result.result->success ? "true" : "false") << ", results=[";
  for (size_t i = 0; i < result.result->results.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << result.result->results[i];
  }
  ss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  rclcpp::shutdown();
}

}  // namespace action_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_example::ActionClientNode>());
  rclcpp::shutdown();
  return 0;
}

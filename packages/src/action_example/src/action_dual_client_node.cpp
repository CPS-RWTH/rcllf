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

#include "action_example/action_dual_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace action_example
{

ActionDualClientNode::ActionDualClientNode(const rclcpp::NodeOptions & options)
: Node("dual_client", options)
{
  client_a_ = rclcpp_action::create_client<Analyze>(this, "analyze_a");
  client_b_ = rclcpp_action::create_client<Analyze>(this, "analyze_b");
  timer_a_ = this->create_wall_timer(2s, std::bind(&ActionDualClientNode::send_goal_a, this));
  timer_b_ = this->create_wall_timer(2s, std::bind(&ActionDualClientNode::send_goal_b, this));
}

void ActionDualClientNode::send_goal_a()
{
  timer_a_->cancel();
  if (sent_a_ || !client_a_->wait_for_action_server(5s)) {
    return;
  }
  auto options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  options.feedback_callback = std::bind(
    &ActionDualClientNode::feedback_a, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
    std::bind(&ActionDualClientNode::result_a, this, std::placeholders::_1);
  auto goal = Analyze::Goal();
  goal.num_samples = kSamplesA;
  goal.analysis_type = "dual_test";
  sent_a_ = true;
  client_a_->async_send_goal(goal, options);
}

void ActionDualClientNode::send_goal_b()
{
  timer_b_->cancel();
  if (sent_b_ || !client_b_->wait_for_action_server(5s)) {
    return;
  }
  auto options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  options.feedback_callback = std::bind(
    &ActionDualClientNode::feedback_b, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
    std::bind(&ActionDualClientNode::result_b, this, std::placeholders::_1);
  auto goal = Analyze::Goal();
  goal.num_samples = kSamplesB;
  goal.analysis_type = "dual_test";
  sent_b_ = true;
  client_b_->async_send_goal(goal, options);
}

void ActionDualClientNode::feedback_a(
  GoalHandleAnalyze::SharedPtr goal_handle,
  const std::shared_ptr<const Analyze::Feedback> feedback)
{
  (void)goal_handle;
  RCLCPP_INFO(
    this->get_logger(), "Feedback A: samples_processed=%d, progress=%.1f%%",
    feedback->samples_processed, feedback->progress_percentage);
}

void ActionDualClientNode::feedback_b(
  GoalHandleAnalyze::SharedPtr goal_handle,
  const std::shared_ptr<const Analyze::Feedback> feedback)
{
  (void)goal_handle;
  RCLCPP_INFO(
    this->get_logger(), "Feedback B: samples_processed=%d, progress=%.1f%%",
    feedback->samples_processed, feedback->progress_percentage);
}

void ActionDualClientNode::result_a(const GoalHandleAnalyze::WrappedResult & result)
{
  log_result('A', result);
}

void ActionDualClientNode::result_b(const GoalHandleAnalyze::WrappedResult & result)
{
  log_result('B', result);
}

void ActionDualClientNode::log_result(char label, const GoalHandleAnalyze::WrappedResult & result)
{
  std::stringstream ss;
  ss << "Result " << label << ": code="
     << (result.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" : "OTHER")
     << ", results=[";
  for (size_t i = 0; i < result.result->results.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << result.result->results[i];
  }
  ss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  if (++results_ == 2) {
    RCLCPP_INFO(this->get_logger(), "Both results received");
    rclcpp::shutdown();
  }
}

}  // namespace action_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_example::ActionDualClientNode>());
  rclcpp::shutdown();
  return 0;
}

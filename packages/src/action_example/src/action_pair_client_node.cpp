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

#include "action_example/action_pair_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace action_example
{

ActionPairClientNode::ActionPairClientNode(const rclcpp::NodeOptions & options)
: Node("pair_client", options)
{
  action_client_ = rclcpp_action::create_client<Analyze>(this, "analyze");
  timer_ = this->create_wall_timer(2s, std::bind(&ActionPairClientNode::send_goals, this));
}

void ActionPairClientNode::send_goals()
{
  timer_->cancel();
  if (goals_sent_) {
    return;
  }
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }

  auto options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  options.goal_response_callback =
    std::bind(&ActionPairClientNode::goal_response_callback, this, std::placeholders::_1);
  options.feedback_callback = std::bind(
    &ActionPairClientNode::feedback_callback, this, std::placeholders::_1,
    std::placeholders::_2);
  options.result_callback =
    std::bind(&ActionPairClientNode::result_callback, this, std::placeholders::_1);

  goals_sent_ = true;
  for (const int samples : {kSamplesA, kSamplesB}) {
    auto goal = Analyze::Goal();
    goal.num_samples = samples;
    goal.analysis_type = "pair_test";
    RCLCPP_INFO(this->get_logger(), "Sending goal: num_samples=%d", samples);
    action_client_->async_send_goal(goal, options);
  }
}

char ActionPairClientNode::label_of(const rclcpp_action::GoalUUID & id) const
{
  const auto it = labels_.find(id);
  return it == labels_.end() ? '?' : it->second;
}

void ActionPairClientNode::goal_response_callback(
  const GoalHandleAnalyze::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return;
  }
  const char label = static_cast<char>('A' + labels_.size());
  labels_[goal_handle->get_goal_id()] = label;
  RCLCPP_INFO(this->get_logger(), "Goal %c accepted", label);
}

void ActionPairClientNode::feedback_callback(
  GoalHandleAnalyze::SharedPtr goal_handle,
  const std::shared_ptr<const Analyze::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(), "Feedback %c: samples_processed=%d, progress=%.1f%%",
    label_of(goal_handle->get_goal_id()), feedback->samples_processed,
    feedback->progress_percentage);
}

void ActionPairClientNode::result_callback(const GoalHandleAnalyze::WrappedResult & result)
{
  std::stringstream ss;
  ss << "Result " << label_of(result.goal_id) << ": code="
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
  rclcpp::spin(std::make_shared<action_example::ActionPairClientNode>());
  rclcpp::shutdown();
  return 0;
}

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

#ifndef ACTION_FULL_EXAMPLE__LONG_RUNNING_ACTION_SERVER_NODE_HPP_
#define ACTION_FULL_EXAMPLE__LONG_RUNNING_ACTION_SERVER_NODE_HPP_

#include <memory>
#include <mutex>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_full_example
{

class LongRunningActionServerNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ServerGoalHandle<Analyze>;

  LongRunningActionServerNode();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  void execute(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  rclcpp_action::Server<Analyze>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  std::shared_ptr<Analyze::Feedback> current_feedback_;
  std::mutex feedback_mutex_;
};

}  // namespace action_full_example

#endif  // ACTION_FULL_EXAMPLE__LONG_RUNNING_ACTION_SERVER_NODE_HPP_

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

#ifndef ACTION_EXAMPLE__ACTION_DUAL_SERVER_NODE_HPP_
#define ACTION_EXAMPLE__ACTION_DUAL_SERVER_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_example
{

/// Two action servers of one type on one node, "analyze_a" and "analyze_b",
/// stepped by one 100 ms timer like ActionServerNode. The binding
/// must give each server's reactions the waitable of that server, not the
/// node's first one.
class ActionDualServerNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ServerGoalHandle<Analyze>;

  explicit ActionDualServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ActionDualServerNode() override;

private:
  struct ActiveGoal
  {
    std::shared_ptr<GoalHandleAnalyze> handle;
    int num_samples = 0;
    int next_sample = 0;
    std::shared_ptr<Analyze::Result> result;
  };

  rclcpp_action::GoalResponse handle_goal_a(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal);
  rclcpp_action::GoalResponse handle_goal_b(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel_a(
    const std::shared_ptr<GoalHandleAnalyze> goal_handle);
  rclcpp_action::CancelResponse handle_cancel_b(
    const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  void handle_accepted_a(const std::shared_ptr<GoalHandleAnalyze> goal_handle);
  void handle_accepted_b(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  /// Steps the goals of "analyze_a", then those of "analyze_b".
  void step_goals();

  void abort_active_goals();

  rclcpp_action::Server<Analyze>::SharedPtr server_a_;
  rclcpp_action::Server<Analyze>::SharedPtr server_b_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  std::mutex goals_mutex_;
  std::vector<ActiveGoal> goals_a_;
  std::vector<ActiveGoal> goals_b_;
  bool stopping_ = false;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;
};

}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_DUAL_SERVER_NODE_HPP_

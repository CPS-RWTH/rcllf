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

#ifndef ACTION_EXAMPLE__ACTION_SERVER_NODE_HPP_
#define ACTION_EXAMPLE__ACTION_SERVER_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_example
{

/// Steps every accepted goal from one 100 ms timer created in the
/// constructor, so feedback and results are sent from a timer callback that
/// LF dispatches at a logical time. No goal runs on a thread of
/// its own.
class ActionServerNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ServerGoalHandle<Analyze>;

  explicit ActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ActionServerNode() override;

private:
  /// One accepted goal and the samples it has produced so far.
  struct ActiveGoal
  {
    std::shared_ptr<GoalHandleAnalyze> handle;
    int num_samples = 0;
    int next_sample = 0;
    std::shared_ptr<Analyze::Result> result;
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Analyze::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleAnalyze> goal_handle);

  void step_goals();

  /// Ends every active goal with abort. Called before the context goes down,
  /// so no goal handle outlives the context or the server.
  void abort_active_goals();

  rclcpp_action::Server<Analyze>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  std::mutex goals_mutex_;
  std::vector<ActiveGoal> goals_;
  bool stopping_ = false;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;
};

}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_SERVER_NODE_HPP_

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

#ifndef ACTION_EXAMPLE__ACTION_DUAL_CLIENT_NODE_HPP_
#define ACTION_EXAMPLE__ACTION_DUAL_CLIENT_NODE_HPP_

#include <memory>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_example
{

/// Sends one goal to "analyze_a" (4 samples) and one to "analyze_b"
/// (2 samples), both served by ActionDualServerNode. Each client
/// has a timer and callbacks of its own, so the analysis ties every callback
/// to its action.
class ActionDualClientNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ClientGoalHandle<Analyze>;

  static constexpr int kSamplesA = 4;
  static constexpr int kSamplesB = 2;

  explicit ActionDualClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void send_goal_a();
  void send_goal_b();

  void feedback_a(
    GoalHandleAnalyze::SharedPtr goal_handle,
    const std::shared_ptr<const Analyze::Feedback> feedback);
  void feedback_b(
    GoalHandleAnalyze::SharedPtr goal_handle,
    const std::shared_ptr<const Analyze::Feedback> feedback);

  void result_a(const GoalHandleAnalyze::WrappedResult & result);
  void result_b(const GoalHandleAnalyze::WrappedResult & result);

  void log_result(char label, const GoalHandleAnalyze::WrappedResult & result);

  rclcpp_action::Client<Analyze>::SharedPtr client_a_;
  rclcpp_action::Client<Analyze>::SharedPtr client_b_;
  rclcpp::TimerBase::SharedPtr timer_a_;
  rclcpp::TimerBase::SharedPtr timer_b_;
  bool sent_a_ = false;
  bool sent_b_ = false;
  int results_ = 0;
};

}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_DUAL_CLIENT_NODE_HPP_

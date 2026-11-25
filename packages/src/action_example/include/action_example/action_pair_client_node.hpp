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

#ifndef ACTION_EXAMPLE__ACTION_PAIR_CLIENT_NODE_HPP_
#define ACTION_EXAMPLE__ACTION_PAIR_CLIENT_NODE_HPP_

#include <map>
#include <memory>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_example
{

// Sends two goals from one timer callback, so the server steps both goals in
// the same tick and publishes two feedback samples per step. The
// goals are labelled A and B in the order the server accepts them, which is
// the order they were sent.
class ActionPairClientNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ClientGoalHandle<Analyze>;

  static constexpr int kSamplesA = 4;
  static constexpr int kSamplesB = 2;

  explicit ActionPairClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void send_goals();

  void goal_response_callback(const GoalHandleAnalyze::SharedPtr & goal_handle);

  void feedback_callback(
    GoalHandleAnalyze::SharedPtr goal_handle,
    const std::shared_ptr<const Analyze::Feedback> feedback);

  void result_callback(const GoalHandleAnalyze::WrappedResult & result);

  char label_of(const rclcpp_action::GoalUUID & id) const;

  rclcpp_action::Client<Analyze>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goals_sent_ = false;
  std::map<rclcpp_action::GoalUUID, char> labels_;
  int results_ = 0;
};

}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_PAIR_CLIENT_NODE_HPP_

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

#ifndef ACTION_EXAMPLE__ACTION_CANCEL_CLIENT_NODE_HPP_
#define ACTION_EXAMPLE__ACTION_CANCEL_CLIENT_NODE_HPP_

#include <memory>

#include "custom_interfaces/action/analyze.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_example
{

// Sends one goal and cancels it from its feedback callback once the server
// reports kCancelAtSample processed samples. The cancel request is
// made from a modelled callback, so it runs under lfc, unlike the timer that
// ActionClientNode creates inside goal_response_callback.
class ActionCancelClientNode : public rclcpp::Node
{
public:
  using Analyze = custom_interfaces::action::Analyze;
  using GoalHandleAnalyze = rclcpp_action::ClientGoalHandle<Analyze>;
  using CancelResponse = rclcpp_action::Client<Analyze>::CancelResponse;

  static constexpr int kNumSamples = 20;
  static constexpr int kCancelAtSample = 3;

  explicit ActionCancelClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void send_goal();

  void goal_response_callback(const GoalHandleAnalyze::SharedPtr & goal_handle);

  void feedback_callback(
    GoalHandleAnalyze::SharedPtr goal_handle,
    const std::shared_ptr<const Analyze::Feedback> feedback);

  void cancel_response_callback(CancelResponse::SharedPtr response);

  void result_callback(const GoalHandleAnalyze::WrappedResult & result);

  rclcpp_action::Client<Analyze>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_ = false;
  bool cancel_sent_ = false;
};

}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_CANCEL_CLIENT_NODE_HPP_

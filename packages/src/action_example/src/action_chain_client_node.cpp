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

#include "action_example/action_chain_client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>

using namespace std::chrono_literals;

namespace action_example
{

ActionChainClientNode::ActionChainClientNode(const rclcpp::NodeOptions & options)
: Node("chain_client", options)
{
  client_a_ = rclcpp_action::create_client<Analyze>(this, "analyze_a");
  client_b_ = rclcpp_action::create_client<Analyze>(this, "analyze_b");
  timer_a_ = this->create_wall_timer(2s, std::bind(&ActionChainClientNode::send_goal_a, this));
}

void ActionChainClientNode::send_goal_a()
{
  timer_a_->cancel();
  if (sent_a_ || !client_a_->wait_for_action_server(5s)) {
    return;
  }
  auto options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  options.result_callback =
    std::bind(&ActionChainClientNode::result_a, this, std::placeholders::_1);
  auto goal = Analyze::Goal();
  goal.num_samples = kSamplesA;
  goal.analysis_type = "chain_test";
  sent_a_ = true;
  client_a_->async_send_goal(goal, options);
}

void ActionChainClientNode::result_a(const GoalHandleAnalyze::WrappedResult & result)
{
  log_result('A', result);
  auto options = rclcpp_action::Client<Analyze>::SendGoalOptions();
  options.result_callback =
    std::bind(&ActionChainClientNode::result_b, this, std::placeholders::_1);
  auto goal = Analyze::Goal();
  goal.num_samples = kSamplesB;
  goal.analysis_type = "chain_test";
  client_b_->async_send_goal(goal, options);
}

void ActionChainClientNode::result_b(const GoalHandleAnalyze::WrappedResult & result)
{
  log_result('B', result);
  RCLCPP_INFO(this->get_logger(), "Both results received");
  rclcpp::shutdown();
}

void ActionChainClientNode::log_result(
  char label, const GoalHandleAnalyze::WrappedResult & result)
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
}

}  // namespace action_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<action_example::ActionChainClientNode>());
  rclcpp::shutdown();
  return 0;
}

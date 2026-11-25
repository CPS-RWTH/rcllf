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

/// @file test_action_server_shutdown.cpp
/// @brief A context shutdown (what rclcpp's SIGINT handler does)
///        while a goal executes must not abort the action server. The server
///        source is compiled in the way the lfc generator does it, with its
///        main() renamed.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#define main action_server_node_unused_main
#include "../src/action_server_node.cpp"  // NOLINT(build/include)
#undef main

namespace
{

using Analyze = custom_interfaces::action::Analyze;
using namespace std::chrono_literals;

/// Spins one server node and one client node until the context shuts down.
struct Fixture
{
  std::shared_ptr<action_example::ActionServerNode> server;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<Analyze>::SharedPtr client;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  std::thread spinner;

  Fixture()
  {
    server = std::make_shared<action_example::ActionServerNode>();
    client_node = std::make_shared<rclcpp::Node>("issue233_client");
    client = rclcpp_action::create_client<Analyze>(client_node, "analyze");
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(server);
    exec->add_node(client_node);
    spinner = std::thread([this] {exec->spin();});
  }
};

template<typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds limit)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

Analyze::Goal make_goal()
{
  Analyze::Goal goal;
  goal.num_samples = 10;
  goal.analysis_type = "statistical";
  return goal;
}

/// The shutdown-during-goal scenario. Runs in a death-test child and exits 0 when the
/// server survives the shutdown.
void shutdown_during_goal()
{
  rclcpp::init(0, nullptr);
  auto f = std::make_unique<Fixture>();
  if (!f->client->wait_for_action_server(10s)) {
    std::fprintf(stderr, "action server not available\n");
    std::_Exit(2);
  }
  auto accepted = f->client->async_send_goal(make_goal());
  if (accepted.wait_for(10s) != std::future_status::ready || !accepted.get()) {
    std::fprintf(stderr, "goal not accepted within 10 s\n");
    std::_Exit(2);
  }
  // Mid-goal, between two 100 ms step ticks, while the goal is active.
  std::this_thread::sleep_for(350ms);
  // Mid-goal context shutdown while the server node stays alive. Under lfc the
  // binding holds the node until ros2_lf_runtime_shutdown(), in stock ROS
  // main() holds it until rclcpp::spin() returns.
  rclcpp::shutdown();
  f->spinner.join();
  // The pre-shutdown callback has aborted the goal. Let any step tick drain.
  std::this_thread::sleep_for(500ms);
  f.reset();
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(0);
}

}  // namespace

TEST(ActionServerShutdown, ContextShutdownDuringGoalDoesNotAbort)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  // Before the fix the child dies with SIGABRT and the message
  // "Asked to publish result for goal that does not exist".
  EXPECT_EXIT(shutdown_during_goal(), ::testing::ExitedWithCode(0), "");
}

TEST(ActionServerShutdown, UninterruptedGoalReturnsResultAndFeedback)
{
  rclcpp::init(0, nullptr);
  Fixture f;
  std::mutex m;
  std::vector<int> samples;
  std::vector<float> progress;
  std::atomic<bool> done{false};
  rclcpp_action::ClientGoalHandle<Analyze>::WrappedResult wrapped;
  bool sent = f.client->wait_for_action_server(10s);
  if (sent) {
    rclcpp_action::Client<Analyze>::SendGoalOptions options;
    options.feedback_callback = [&](auto, const std::shared_ptr<const Analyze::Feedback> fb) {
        std::lock_guard<std::mutex> lock(m);
        samples.push_back(fb->samples_processed);
        progress.push_back(fb->progress_percentage);
      };
    options.result_callback = [&](const auto & r) {
        wrapped = r;
        done.store(true);
      };
    f.client->async_send_goal(make_goal(), options);
  }
  const bool finished = sent && wait_for([&done] {return done.load();}, 10s);
  rclcpp::shutdown();
  f.spinner.join();
  ASSERT_TRUE(finished);
  // Expected from the server code: sample i in 0..9 gives feedback
  // samples_processed i, progress 100 * i / 10, and result value i * 1.1.
  ASSERT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->success);
  ASSERT_EQ(wrapped.result->results.size(), 10u);
  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(wrapped.result->results[i], i * 1.1);
  }
  // rclcpp_action drops feedback that reaches the client before the goal
  // response, so sample 0 may be missing. Every sample that arrives is in
  // order and the sequence ends at 9.
  std::lock_guard<std::mutex> lock(m);
  ASSERT_GE(samples.size(), 9u);
  ASSERT_LE(samples.size(), 10u);
  const int first = 10 - static_cast<int>(samples.size());
  for (size_t k = 0; k < samples.size(); ++k) {
    EXPECT_EQ(samples[k], first + static_cast<int>(k));
    EXPECT_FLOAT_EQ(progress[k], 10.0f * samples[k]);
  }
}

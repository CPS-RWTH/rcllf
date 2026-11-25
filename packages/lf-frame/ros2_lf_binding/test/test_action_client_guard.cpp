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

/// @file test_action_client_guard.cpp
/// @brief The action client guard across a deactivate.
///
/// The test executable carries src/action_gate.cpp like a generated one, so
/// rclcpp_action::ClientBase's send members are the gate's and every response
/// callback is guarded. The server rejects every cancel, so a goal outlives
/// the deactivate's cancel. The client's responses run on the test thread
/// with no InactiveClientScope, as they do once the node is active again.
///
/// Review:
/// - item 1: a goal with a feedback callback and no result request has its
///   callbacks stopped by deactivate, so its feedback stays from user code.
///   To falsify: drop track_accepted from the goal wrapper.
/// - item 2: a goal sent before deactivate and accepted after reactivation
///   reaches no user callback and is cancelled once. To falsify: make
///   kept_from_user ignore the epoch.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "action_client_guard.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcutils/logging.h"
#include "ros2_lf_binding/action_gate_api.h"
#include "ros2_lf_binding/node_registry.h"

namespace
{

using Fibonacci = example_interfaces::action::Fibonacci;
using ServerGoal = rclcpp_action::ServerGoalHandle<Fibonacci>;
using ClientGoal = rclcpp_action::ClientGoalHandle<Fibonacci>;
using Client = rclcpp_action::Client<Fibonacci>;

/// The ops the generator emits for Fibonacci (factory_emitter.py).
void register_fibonacci_ops()
{
  Ros2LfActionClientOps ops;
  ops.goal_id = [](rclcpp::Waitable & client, const void * request, bool result,
    Ros2LfGoalId & id) -> bool {
      if (dynamic_cast<Client *>(&client) == nullptr) {return false;}
      id = result ?
        static_cast<const Fibonacci::Impl::GetResultService::Request *>(request)->goal_id.uuid :
        static_cast<const Fibonacci::Impl::SendGoalService::Request *>(request)->goal_id.uuid;
      return true;
    };
  ops.goal_accepted = [](rclcpp::Waitable & client, const void * response,
    bool & accepted) -> bool {
      if (dynamic_cast<Client *>(&client) == nullptr) {return false;}
      accepted =
        static_cast<const Fibonacci::Impl::SendGoalService::Response *>(response)->accepted;
      return true;
    };
  ops.deactivate = [](rclcpp::Waitable & client,
    const std::vector<Ros2LfGoalId> & ids) -> bool {
      auto * typed = dynamic_cast<Client *>(&client);
      if (typed == nullptr) {return false;}
      for (const auto & id : ids) {
        typed->stop_callbacks(id);
      }
      typed->async_cancel_all_goals();
      return true;
    };
  ros2_lf_register_action_client_ops(std::move(ops));
}

std::mutex g_log_mu;
std::vector<std::string> g_infos;
rcutils_logging_output_handler_t g_prev_handler = nullptr;

void capture_infos(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (severity == RCUTILS_LOG_SEVERITY_INFO) {
    va_list copy;
    va_copy(copy, *args);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lk(g_log_mu);
    g_infos.emplace_back(buf);
  }
  g_prev_handler(location, severity, name, timestamp, format, args);
}

size_t infos_containing(const std::string & needle)
{
  std::lock_guard<std::mutex> lk(g_log_mu);
  size_t n = 0;
  for (const auto & e : g_infos) {
    n += e.find(needle) != std::string::npos;
  }
  return n;
}

class ActionClientGuard : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    register_fibonacci_ops();
    g_prev_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_infos);
  }
  static void TearDownTestSuite()
  {
    rcutils_logging_set_output_handler(g_prev_handler);
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    {
      std::lock_guard<std::mutex> lk(g_log_mu);
      g_infos.clear();
    }
    // One action name per process, so concurrent test runs in one domain never meet.
    const std::string id = std::to_string(getpid()) + "_" + std::to_string(++counter);
    const std::string action = "guard_" + id;
    server_node = std::make_shared<rclcpp::Node>("guard_server_" + id);
    client_node = std::make_shared<rclcpp::Node>("guard_client_" + id);
    server = rclcpp_action::create_server<Fibonacci>(
      server_node, action,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<ServerGoal>) {
        ++cancels;
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<ServerGoal> h) {
        std::lock_guard<std::mutex> lk(mu);
        accepted.push_back(h);
      });
    client = rclcpp_action::create_client<Fibonacci>(client_node, action);
    client_exec.add_node(client_node);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  }

  void TearDown() override
  {
    stop_server();
    std::lock_guard<std::mutex> lk(mu);
    for (const auto & h : accepted) {
      if (h->is_active()) {
        h->abort(std::make_shared<Fibonacci::Result>());
      }
    }
  }

  void start_server()
  {
    server_exec.add_node(server_node);
    server_thread = std::thread([this]() {server_exec.spin();});
  }

  void stop_server()
  {
    if (server_thread.joinable()) {
      server_exec.cancel();
      server_thread.join();
    }
  }

  /// Spin the client until `done` or `timeout`; true when `done`.
  bool spin_client_until(
    const std::function<bool()> & done,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      client_exec.spin_some(std::chrono::milliseconds(10));
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
  }

  std::shared_ptr<ServerGoal> server_goal()
  {
    std::lock_guard<std::mutex> lk(mu);
    return accepted.empty() ? nullptr : accepted.back();
  }

  static int counter;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Server<Fibonacci>::SharedPtr server;
  Client::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor client_exec;
  rclcpp::executors::SingleThreadedExecutor server_exec;
  std::thread server_thread;
  std::atomic<int> cancels{0};
  std::mutex mu;
  std::vector<std::shared_ptr<ServerGoal>> accepted;
};
int ActionClientGuard::counter = 0;

TEST_F(ActionClientGuard, GateWrapsTheClientSends)
{
  EXPECT_NE(rcllf_action_gate_symbol("rclcpp_action::ClientBase::send_goal_request"), nullptr);
}

// Item 1: the goal has a feedback callback and no result callback, so no
// result request is pending when the node is deactivated.
TEST_F(ActionClientGuard, DeactivateStopsFeedbackOfAGoalWithoutResultRequest)
{
  start_server();
  std::atomic<int> feedback{0};
  ClientGoal::SharedPtr handle;
  Client::SendGoalOptions options;
  options.goal_response_callback = [&](ClientGoal::SharedPtr h) {handle = h;};
  options.feedback_callback =
    [&](ClientGoal::SharedPtr, std::shared_ptr<const Fibonacci::Feedback>) {++feedback;};
  Fibonacci::Goal goal;
  goal.order = 3;
  client->async_send_goal(goal, options);
  ASSERT_TRUE(spin_client_until([&]() {return handle != nullptr;})) << "goal not accepted";
  ASSERT_TRUE(spin_client_until([&]() {return server_goal() != nullptr;}));
  auto fb = std::make_shared<Fibonacci::Feedback>();
  server_goal()->publish_feedback(fb);
  ASSERT_TRUE(spin_client_until([&]() {return feedback.load() > 0;})) <<
    "feedback did not reach the active client";

  EXPECT_EQ(ros2_lf_binding::action_client_deactivate(*client), 1);
  ASSERT_TRUE(spin_client_until([&]() {return cancels.load() > 0;})) <<
    "the server never saw the deactivate's cancel";

  // Active again: the server rejected the cancel and keeps publishing.
  feedback = 0;
  for (int i = 0; i < 10; ++i) {
    server_goal()->publish_feedback(fb);
    spin_client_until([]() {return false;}, std::chrono::milliseconds(30));
  }
  EXPECT_EQ(feedback.load(), 0) <<
    "feedback of a goal accepted before deactivate reached its callback after the node was "
    "active again: the goal had no result request, so deactivate did not stop its callbacks";
}

// Item 2: the goal is sent, the node is deactivated before the server saw
// the goal, and the goal response is taken once the node is active again.
TEST_F(ActionClientGuard, GoalAcceptedAfterReactivationIsKeptAndCancelledOnce)
{
  std::atomic<int> responses{0};
  std::atomic<int> feedback{0};
  Client::SendGoalOptions options;
  options.goal_response_callback = [&](ClientGoal::SharedPtr) {++responses;};
  options.feedback_callback =
    [&](ClientGoal::SharedPtr, std::shared_ptr<const Fibonacci::Feedback>) {++feedback;};
  Fibonacci::Goal goal;
  goal.order = 3;
  auto future = client->async_send_goal(goal, options);
  EXPECT_EQ(ros2_lf_binding::action_client_deactivate(*client), 0);
  start_server();
  ASSERT_TRUE(spin_client_until([&]() {return server_goal() != nullptr;}));
  const auto cancelled = "cancelled a goal the server accepted after deactivate";
  spin_client_until(
    [&]() {return responses.load() > 0 || infos_containing(cancelled) > 0;});
  auto fb = std::make_shared<Fibonacci::Feedback>();
  for (int i = 0; i < 5; ++i) {
    server_goal()->publish_feedback(fb);
    spin_client_until([]() {return false;}, std::chrono::milliseconds(30));
  }
  EXPECT_EQ(responses.load(), 0) <<
    "a goal sent before deactivate and accepted after the node was active again reached "
    "the user's goal-response callback";
  EXPECT_EQ(feedback.load(), 0) << "its feedback reached the user's feedback callback";
  EXPECT_EQ(infos_containing(cancelled), 1u) << "the goal was not cancelled exactly once";
  // rclcpp's callback was dropped with its promise unset: the future holds
  // broken_promise, never a goal handle.
  EXPECT_THROW(future.get(), std::future_error) << "rclcpp's goal-response callback ran";
}

}  // namespace

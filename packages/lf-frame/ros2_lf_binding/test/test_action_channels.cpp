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

/// @file test_action_channels.cpp
/// @brief Counted action channels with the action gate linked in.
///
/// The test executable carries src/action_gate.cpp like a generated one, so
/// rclcpp_action's rcl_action calls go through the gate. Item 2: a port event
/// drains every sample its reaction sent. Item 8: a sample that never arrives
/// is traced LOST exactly once and logged as an error exactly once. Item 7:
/// an action send with no LF origin is refused while the runtime has armed it.
/// Item 6: a pre-shutdown callback a server node registers while it is
/// constructed is kept from the context and run once by its reaction(shutdown).

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "action_channels.hpp"
#include "corr_runtime.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "fake_trace_sink.hpp"
#include "handle_table.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcutils/logging.h"
#include "ros2_lf_binding/action_gate_api.h"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "semantic_trace.hpp"
#include "thread_role.hpp"

namespace
{

using Fibonacci = example_interfaces::action::Fibonacci;
using ServerGoal = rclcpp_action::ServerGoalHandle<Fibonacci>;
using ClientGoal = rclcpp_action::ClientGoalHandle<Fibonacci>;
namespace st = ros2_lf_binding::semtrace;

constexpr int kServerRid = 0;
constexpr int kClientRid = 1;
constexpr size_t kServerGoalEntity = 0;
constexpr size_t kClientFeedbackEntity = 3;

std::atomic<int> g_wakes{0};
void * g_wake_action = nullptr;

int count_wake(void * action, int64_t record)
{
  (void)record;
  g_wake_action = action;
  return ++g_wakes;
}

std::mutex g_log_mu;
std::vector<std::string> g_errors;
rcutils_logging_output_handler_t g_prev_handler = nullptr;

void capture_errors(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (severity == RCUTILS_LOG_SEVERITY_ERROR) {
    va_list copy;
    va_copy(copy, *args);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lk(g_log_mu);
    g_errors.emplace_back(buf);
  }
  g_prev_handler(location, severity, name, timestamp, format, args);
}

size_t errors_containing(const std::string & needle)
{
  std::lock_guard<std::mutex> lk(g_log_mu);
  size_t n = 0;
  for (const auto & e : g_errors) {
    n += e.find(needle) != std::string::npos;
  }
  return n;
}

class ActionChannels : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    g_prev_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_errors);
  }
  static void TearDownTestSuite()
  {
    rcutils_logging_set_output_handler(g_prev_handler);
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    ros2_lf_binding::action_sends_begin("SetUp", -1);  // drop an earlier test's counts
    {
      std::lock_guard<std::mutex> lk(g_log_mu);
      g_errors.clear();
    }
    // One action name per process, so concurrent test runs in one domain never meet.
    action = "fib_" + std::to_string(getpid()) + "_" + std::to_string(++counter);
    node = std::make_shared<rclcpp::Node>("action_channels_" + std::to_string(counter));
    server = rclcpp_action::create_server<Fibonacci>(
      node, action,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        ++goals_handled;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<ServerGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<ServerGoal> h) {
        std::lock_guard<std::mutex> lk(mu);
        accepted.push_back(h);
      });
    client = rclcpp_action::create_client<Fibonacci>(node, action);
    ros2_lf_binding::g_handle_table.assign(2, ros2_lf_binding::HandleEntry{});
    auto & s = ros2_lf_binding::g_handle_table[kServerRid];
    s.kind = ros2_lf_binding::HandleKind::ACTION_SERVER;
    s.action_waitable = server;
    s.action_entity_id = kServerGoalEntity;
    s.topic = action;
    s.take_timeout_ms = 2000;
    s.node_class = "Server";
    s.callback_name = "handle_goal";
    auto & c = ros2_lf_binding::g_handle_table[kClientRid];
    c.kind = ros2_lf_binding::HandleKind::ACTION_CLIENT;
    c.action_waitable = client;
    c.action_entity_id = kClientFeedbackEntity;
    c.topic = action;
    c.take_timeout_ms = 2000;
    c.node_class = "Client";
    c.callback_name = "feedback_callback";
    st::reset_for_test();
    st::set_sink_for_test(&sink);
    st::begin(1);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  }

  void TearDown() override
  {
    ros2_lf_binding::action_refusal_arm(false);
    ros2_lf_binding::action_external_shutdown();
    st::set_sink_for_test(nullptr);
    st::reset_for_test();
    ros2_lf_binding::g_handle_table.clear();
  }

  std::vector<rcllf_test::Rec> lost_ends()
  {
    std::vector<rcllf_test::Rec> out;
    for (const auto & r : sink.of("dispatch_end")) {
      if (r.outcome == RCLLF_OUTCOME_LOST) {
        out.push_back(r);
      }
    }
    return out;
  }

  static int counter;
  std::string action;
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<Fibonacci>::SharedPtr server;
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  std::atomic<int> goals_handled{0};
  std::mutex mu;
  std::vector<std::shared_ptr<ServerGoal>> accepted;
  rcllf_test::FakeSink sink;
};
int ActionChannels::counter = 0;

TEST_F(ActionChannels, GateIsLinkedIntoTheExecutable)
{
  EXPECT_NE(rcllf_action_gate_symbol("rcl_action_send_goal_request"), nullptr);
  EXPECT_NE(rcllf_action_gate_symbol("rcl_action_take_feedback"), nullptr);
}

// Item 8: a goal request the port promised but that never arrives is LOST,
// traced once with no missing hook, and logged as an error once.
TEST_F(ActionChannels, MissingSampleIsTracedLostAndLoggedOnce)
{
  ros2_lf_binding::g_handle_table[kServerRid].take_timeout_ms = 50;
  const int executed = ros2_lf_binding::action_counted_take(
    "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, 1);
  EXPECT_EQ(executed, 0);
  EXPECT_EQ(goals_handled.load(), 0);
  const auto lost = lost_ends();
  ASSERT_EQ(lost.size(), 1u) << "one LOST dispatch_end for one missing sample";
  EXPECT_EQ(lost[0].missing_hook, RCLLF_MISSING_HOOK_NONE);
  EXPECT_EQ(errors_containing("goal_req sample 1 of 1 did not arrive"), 1u);
}

// Item 2: one port event of value 2 serves both goal requests the client's
// reaction sent, not just the first.
TEST_F(ActionChannels, OnePortEventServesEveryGoalSent)
{
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ), 0);
  Fibonacci::Goal goal;
  goal.order = 3;
  client->async_send_goal(goal);
  client->async_send_goal(goal);
  const int sent = ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  ASSERT_EQ(sent, 2);
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ), 0) << "claimed once";
  const int executed = ros2_lf_binding::action_counted_take(
    "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, sent);
  EXPECT_EQ(executed, 2);
  EXPECT_EQ(goals_handled.load(), 2);
  EXPECT_TRUE(lost_ends().empty());
  // The two goal responses the server sent are counted for its own reaction.
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_RESP), 2);
}

// Merge with the lifecycle gate: a lifecycle node not active at this tag
// takes the promised samples but runs no callback, and each drop is traced.
TEST_F(ActionChannels, InactiveLifecycleNodeDropsTakenSamples)
{
  Fibonacci::Goal goal;
  goal.order = 3;
  client->async_send_goal(goal);
  client->async_send_goal(goal);
  const int sent = ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  ASSERT_EQ(sent, 2);
  const uint64_t dropped_before = ros2_lf_binding::lifecycle_dropped_count();
  const int executed = ros2_lf_binding::action_counted_take(
    "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, sent, false);
  EXPECT_EQ(executed, 0);
  EXPECT_EQ(goals_handled.load(), 0) << "no user callback on an inactive node";
  EXPECT_EQ(ros2_lf_binding::lifecycle_dropped_count() - dropped_before, 2u);
  EXPECT_TRUE(lost_ends().empty()) << "a dropped sample arrived, it is not LOST";
}

// Item 2: two concurrent goals each publish one feedback in one step, and one
// feedback port event of value 2 delivers both to the client.
TEST_F(ActionChannels, OneFeedbackEventDrainsBothConcurrentGoals)
{
  std::atomic<int> feedbacks{0};
  auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  options.feedback_callback =
    [&feedbacks](ClientGoal::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback>) {
      ++feedbacks;
    };
  Fibonacci::Goal goal;
  goal.order = 3;
  // The client holds its goal handles weakly, the futures keep them alive.
  auto first = client->async_send_goal(goal, options);
  auto second = client->async_send_goal(goal, options);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  ASSERT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, 2), 2);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_RESP);
  // The client knows its goal handles only after the goal responses (entity 0).
  ASSERT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kClientRid, 0, RCLLF_ACTION_GOAL_RESP, 2), 2);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_RESULT_REQ);
  std::vector<std::shared_ptr<ServerGoal>> goals;
  {
    std::lock_guard<std::mutex> lk(mu);
    goals = accepted;
  }
  ASSERT_EQ(goals.size(), 2u);
  for (auto & h : goals) {
    auto fb = std::make_shared<Fibonacci::Feedback>();
    fb->sequence = {0, 1};
    h->publish_feedback(fb);
  }
  const int sent = ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_FEEDBACK);
  ASSERT_EQ(sent, 2);
  const int executed = ros2_lf_binding::action_counted_take(
    "test", nullptr, kClientRid, kClientFeedbackEntity, RCLLF_ACTION_FEEDBACK, sent);
  EXPECT_EQ(executed, 2);
  EXPECT_EQ(feedbacks.load(), 2);
  EXPECT_TRUE(lost_ends().empty());
}

// Item 7: while armed, a feedback publish from a thread with no LF origin
// (a worker the node started itself) is refused and never reaches the client.
// The same publish inside a reaction's origin scope goes through, so the
// refusal, not the setup, is what stops the first one.
TEST_F(ActionChannels, SendWithoutLfOriginIsRefusedWhileArmed)
{
  std::atomic<int> feedbacks{0};
  auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  options.feedback_callback =
    [&feedbacks](ClientGoal::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback>) {
      ++feedbacks;
    };
  Fibonacci::Goal goal;
  goal.order = 3;
  auto handle = client->async_send_goal(goal, options);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  ASSERT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, 1), 1);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_RESP);
  ASSERT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kClientRid, 0, RCLLF_ACTION_GOAL_RESP, 1), 1);
  std::shared_ptr<ServerGoal> server_goal;
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(accepted.size(), 1u);
    server_goal = accepted[0];
  }
  auto fb = std::make_shared<Fibonacci::Feedback>();
  fb->sequence = {0, 1};

  ros2_lf_binding::action_refusal_arm(true);
  const uint64_t refused_before = ros2_lf_binding::action_refused_count();
  std::thread worker([&] {
      server_goal->publish_feedback(fb);
      server_goal->publish_feedback(fb);
    });
  worker.join();
  EXPECT_EQ(ros2_lf_binding::action_refused_count() - refused_before, 2u);
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_FEEDBACK), 0);
  EXPECT_EQ(errors_containing("refused feedback on action"), 1u) << "logged once";
  ros2_lf_binding::g_handle_table[kClientRid].take_timeout_ms = 200;
  EXPECT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kClientRid, kClientFeedbackEntity, RCLLF_ACTION_FEEDBACK, 1), 0);
  EXPECT_EQ(feedbacks.load(), 0) << "a refused feedback reached the client";

  {
    ros2_lf_binding::corr_rt::OriginScope origin(nullptr, kServerRid);
    server_goal->publish_feedback(fb);
  }
  EXPECT_EQ(ros2_lf_binding::action_refused_count() - refused_before, 2u);
  ASSERT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_FEEDBACK), 1);
  ros2_lf_binding::g_handle_table[kClientRid].take_timeout_ms = 2000;
  EXPECT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kClientRid, kClientFeedbackEntity, RCLLF_ACTION_FEEDBACK, 1), 1);
  EXPECT_EQ(feedbacks.load(), 1);
}


// Item 3: a goal from a client outside the LF system (here the test thread,
// as `ros2 action send_goal` would be) wakes the registered physical action
// through the server's on-ready callback and is counted, and the reaction
// that action triggers serves it with the pending count.
TEST_F(ActionChannels, ExternalGoalWakesThePhysicalActionAndIsCounted)
{
  int marker = 0;
  g_wakes = 0;
  ros2_lf_action_external_register(nullptr, kServerRid, count_wake, &marker);
  EXPECT_EQ(ros2_lf_action_external_pending(nullptr, kServerRid, 0), 0);
  Fibonacci::Goal goal;
  goal.order = 3;
  client->async_send_goal(goal);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (g_wakes.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GE(g_wakes.load(), 1) << "the on-ready callback never woke the physical action";
  EXPECT_EQ(g_wake_action, &marker);
  const int pending = ros2_lf_action_external_pending(nullptr, kServerRid, 0);
  EXPECT_EQ(pending, 1);
  EXPECT_EQ(ros2_lf_action_external_pending(nullptr, kServerRid, 0), 0) << "taken once";
  EXPECT_EQ(ros2_lf_action_external_pending(nullptr, kServerRid, 2), 0) << "no cancel";
  EXPECT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, pending), 1);
  EXPECT_EQ(goals_handled.load(), 1);
  EXPECT_TRUE(lost_ends().empty());
}

// Item 6: the RID table names "Server" as a node that serves an action.
void register_server_node_once()
{
  static const bool done = [] {
      ros2_lf_binding::RidEntry e{};
      e.reaction_id = kServerRid;
      e.node_class = "Server";
      e.callback_name = "handle_goal";
      e.kind = "action_goal_server";
      ros2_lf_binding::register_rid_entry(e);
      return true;
    }();
  (void)done;
}

// Item 6: under lfc a pre-shutdown callback that ends goals must send in a
// reaction. The one a server node registers while it is constructed is kept
// from the context, and the node's reaction(shutdown) runs it exactly once
// with the goal server's RID as origin, so its sends are claimed, not refused.
TEST_F(ActionChannels, KeptPreShutdownCallbackRunsOnceInTheShutdownReaction)
{
  register_server_node_once();
  Fibonacci::Goal goal;
  goal.order = 3;
  auto handle = client->async_send_goal(goal);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  ASSERT_EQ(
    ros2_lf_binding::action_counted_take(
      "test", nullptr, kServerRid, kServerGoalEntity, RCLLF_ACTION_GOAL_REQ, 1), 1);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_RESP);
  std::shared_ptr<ServerGoal> server_goal;
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(accepted.size(), 1u);
    server_goal = accepted[0];
  }

  auto context = node->get_node_base_interface()->get_context();
  const size_t in_context = context->get_pre_shutdown_callbacks().size();
  int runs = 0;
  bool had_origin = false;
  rclcpp::PreShutdownCallbackHandle kept_handle;
  {
    Ros2LfNodeConstructionScope scope("Server");
    kept_handle = context->add_pre_shutdown_callback(
      [&]() {
        ++runs;
        had_origin = ros2_lf_binding::corr_rt::origin_active();
        auto fb = std::make_shared<Fibonacci::Feedback>();
        fb->sequence = {0, 1};
        server_goal->publish_feedback(fb);
        server_goal->abort(std::make_shared<Fibonacci::Result>());
      });
  }
  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), in_context)
    << "the context got the callback, so rclcpp would run it off any reaction";
  EXPECT_TRUE(ros2_lf_binding::action_pre_shutdown_kept());

  ros2_lf_binding::action_refusal_arm(true);
  const uint64_t refused_before = ros2_lf_binding::action_refused_count();
  const size_t dispatches_before = sink.of("dispatch_end").size();
  ros2_lf_binding::set_guard_disabled_for_test(true);
  ros2_lf_action_server_shutdown(nullptr, kServerRid);
  // The generated reaction claims right after the call.
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_FEEDBACK), 1)
    << "the reaction cannot claim the feedback the callback sent";
  ros2_lf_action_server_shutdown(nullptr, kServerRid);
  ros2_lf_binding::set_guard_disabled_for_test(false);
  EXPECT_EQ(runs, 1) << "the shutdown reaction did not run the kept callback exactly once";
  EXPECT_TRUE(had_origin) << "the kept callback ran with no LF origin";
  EXPECT_EQ(ros2_lf_binding::action_refused_count() - refused_before, 0u);
  EXPECT_EQ(sink.of("dispatch_end").size() - dispatches_before, 1u);
  EXPECT_FALSE(server_goal->is_active());
  EXPECT_FALSE(ros2_lf_binding::action_pre_shutdown_kept());
}

// Item 6: rclcpp keeps every other pre-shutdown callback, also one registered
// while a node without an action server is constructed.
TEST_F(ActionChannels, PreShutdownCallbackOfAnotherNodeStaysWithTheContext)
{
  register_server_node_once();
  auto context = node->get_node_base_interface()->get_context();
  const size_t in_context = context->get_pre_shutdown_callbacks().size();
  int runs = 0;
  auto outside = context->add_pre_shutdown_callback([&runs]() {++runs;});
  rclcpp::PreShutdownCallbackHandle client_node;
  {
    Ros2LfNodeConstructionScope scope("Client");
    client_node = context->add_pre_shutdown_callback([&runs]() {++runs;});
  }
  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), in_context + 2);
  EXPECT_FALSE(ros2_lf_binding::action_pre_shutdown_kept());
  EXPECT_TRUE(context->remove_pre_shutdown_callback(outside));
  EXPECT_TRUE(context->remove_pre_shutdown_callback(client_node));
  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), in_context);
  EXPECT_EQ(runs, 0);
}

// Item 6: only the default context has the binding's hook that stops LF
// before its pre-shutdown callbacks, so a server node's callback on its own
// context stays with that context.
TEST_F(ActionChannels, PreShutdownCallbackOnAnotherContextStaysThere)
{
  register_server_node_once();
  auto other = std::make_shared<rclcpp::Context>();
  int runs = 0;
  rclcpp::PreShutdownCallbackHandle handle;
  {
    Ros2LfNodeConstructionScope scope("Server");
    handle = other->add_pre_shutdown_callback([&runs]() {++runs;});
  }
  EXPECT_EQ(other->get_pre_shutdown_callbacks().size(), 1u);
  EXPECT_FALSE(ros2_lf_binding::action_pre_shutdown_kept());
  EXPECT_TRUE(other->remove_pre_shutdown_callback(handle));
  EXPECT_EQ(runs, 0);
}

// Item 6: a kept callback the node removes again (its destructor, say) is
// dropped and never run.
TEST_F(ActionChannels, RemovedKeptCallbackNeverRuns)
{
  register_server_node_once();
  auto context = node->get_node_base_interface()->get_context();
  int runs = 0;
  rclcpp::PreShutdownCallbackHandle kept_handle;
  {
    Ros2LfNodeConstructionScope scope("Server");
    kept_handle = context->add_pre_shutdown_callback([&runs]() {++runs;});
  }
  EXPECT_TRUE(ros2_lf_binding::action_pre_shutdown_kept());
  EXPECT_TRUE(context->remove_pre_shutdown_callback(kept_handle));
  EXPECT_FALSE(ros2_lf_binding::action_pre_shutdown_kept());
  ros2_lf_binding::set_guard_disabled_for_test(true);
  ros2_lf_action_server_shutdown(nullptr, kServerRid);
  ros2_lf_binding::set_guard_disabled_for_test(false);
  EXPECT_EQ(runs, 0) << "a removed pre-shutdown callback ran";
}

}  // namespace

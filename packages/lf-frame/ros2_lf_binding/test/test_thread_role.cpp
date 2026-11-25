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

/// @file test_thread_role.cpp
/// @brief The one-scheduler guard: a user callback invoked from a
///        non-LF thread is refused with its exact message and never runs.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "handle_table.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "thread_role.hpp"  // NOLINT(build/include_subdir)

extern "C" {
#include "core/environment.h"
}

// No LF program is linked: lf_time_advance_ros_clock() reads no tag.
extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = nullptr;
  return 0;
}

// reactor-c's `-f` flag as a generated executable exports it:
// lf_fast() accepts it because it lives in the same object as
// _lf_get_environments above.
extern "C" {
bool fast = false;
}

using ros2_lf_binding::ScopedThreadRoleForTest;
using ros2_lf_binding::ThreadRole;

namespace
{
std::vector<std::string> g_logged;

void capture_handler(
  const rcutils_log_location_t *, int severity, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  if (severity < RCUTILS_LOG_SEVERITY_ERROR || std::string(name) != "ros2_lf_binding") {
    return;
  }
  char buf[1024];
  va_list copy;
  va_copy(copy, *args);
  vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_logged.emplace_back(buf);
}
}  // namespace

class ThreadRoleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("thread_role_test_node");
    timer_ = node_->create_wall_timer(std::chrono::hours(1), [this]() {fired_++;});
    ros2_lf_binding::g_handle_table.assign(2, ros2_lf_binding::HandleEntry{});
    auto & e = ros2_lf_binding::g_handle_table[1];
    e.kind = ros2_lf_binding::HandleKind::TIMER;
    e.timer = timer_;
    e.node_class = "ThreadRoleTestNode";
    e.callback_name = "tick";
    g_logged.clear();
    rcutils_logging_set_output_handler(capture_handler);
  }

  void TearDown() override
  {
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
    ros2_lf_binding::set_guard_disabled_for_test(false);
    ros2_lf_binding::g_handle_table.clear();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  int fired_ = 0;
};

TEST_F(ThreadRoleTest, InvokeFromIngressIsRefusedWithExactMessage)
{
  ScopedThreadRoleForTest role(ThreadRole::INGRESS);
  const auto refused_before = ros2_lf_binding::refused_callback_count();
  ros2_lf_invoke_timer(nullptr, 1);
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(fired_, 0) << "user callback ran on the ingress thread";
  EXPECT_EQ(ros2_lf_binding::refused_callback_count() - refused_before, 2u);
  ASSERT_EQ(g_logged.size(), 1u) << "refusal must be logged once per entry, RID and thread";
  EXPECT_EQ(
    g_logged[0],
    "rcllf: refused user callback ros2_lf_invoke_timer (RID 1) on non-LF thread 'ingress': "
    "only LF workers may invoke user callbacks (one-scheduler invariant)");
}

TEST_F(ThreadRoleTest, InvokeFromOtherNonLfThreadIsRefused)
{
  ScopedThreadRoleForTest role(ThreadRole::OTHER);
  const auto refused_before = ros2_lf_binding::refused_callback_count();
  ros2_lf_invoke_sub(nullptr, 1, "/x");
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(fired_, 0);
  EXPECT_EQ(ros2_lf_binding::refused_callback_count() - refused_before, 2u);
  // The sub refusal is new for this thread; the timer RID 1 refusal was
  // already logged once by the ingress test (once per entry, RID, thread).
  ASSERT_GE(g_logged.size(), 1u);
  EXPECT_NE(g_logged[0].find("ros2_lf_invoke_sub (RID 1) on non-LF thread 'other'"),
    std::string::npos) << g_logged[0];
}

TEST_F(ThreadRoleTest, InvokeFromLfWorkerRuns)
{
  ScopedThreadRoleForTest role(ThreadRole::LF_WORKER);
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(fired_, 1);
  EXPECT_TRUE(g_logged.empty());
}

// Falsification built in: with the guard removed the same ingress-thread
// invoke runs the callback, so the refusal test above measures the guard.
TEST_F(ThreadRoleTest, WithGuardRemovedCallbackRunsOnIngress)
{
  ros2_lf_binding::set_guard_disabled_for_test(true);
  ScopedThreadRoleForTest role(ThreadRole::INGRESS);
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(fired_, 1);
}

TEST_F(ThreadRoleTest, ScopedRoleRestoresPrevious)
{
  EXPECT_EQ(ros2_lf_binding::current_thread_role(), ThreadRole::UNSPECIFIED);
  {
    ScopedThreadRoleForTest a(ThreadRole::INGRESS);
    {
      ScopedThreadRoleForTest b(ThreadRole::LF_WORKER);
      EXPECT_EQ(ros2_lf_binding::current_thread_role(), ThreadRole::LF_WORKER);
    }
    EXPECT_EQ(ros2_lf_binding::current_thread_role(), ThreadRole::INGRESS);
  }
  EXPECT_EQ(ros2_lf_binding::current_thread_role(), ThreadRole::UNSPECIFIED);
}

TEST_F(ThreadRoleTest, ExpiredTimerIsTracedNoOp)
{
  ScopedThreadRoleForTest role(ThreadRole::LF_WORKER);
  const auto before = ros2_lf_binding::expired_entity_count();
  timer_.reset();  // the node's owner drops it, as on_deactivate does
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(fired_, 0);
  EXPECT_EQ(ros2_lf_binding::expired_entity_count() - before, 1u);
}

// An unresolved `fast` flag reads as real time and brings the
// fast-mode P17e miss back, so both values must come through dlsym.
TEST(LfFastTest, ExecutablesFastFlagResolvesTrueAndFalse)
{
  ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
  fast = true;
  EXPECT_TRUE(ros2_lf_binding::lf_fast()) <<
    "the executable's fast flag is true but lf_fast() returned false: the fast flag did "
    "not resolve, so a fast program would take as if in real time";
  fast = false;
  EXPECT_FALSE(ros2_lf_binding::lf_fast()) <<
    "the executable's fast flag is false but lf_fast() returned true";
}

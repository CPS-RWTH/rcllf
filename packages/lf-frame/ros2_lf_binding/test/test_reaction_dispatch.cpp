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

/// @file test_reaction_dispatch.cpp
/// @brief Tests for ros2_lf_invoke_timer / ros2_lf_invoke_sub guard paths.
///
/// The happy-path (real timer/subscription dispatch) requires a live
/// rclcpp_components node; that is an integration test.  These tests cover
/// the guard branches that must be safe to call without a full component stack:
///   - out-of-range reaction_id
///   - reaction_id bound to wrong kind (timer vs sub)
///   - reaction_id that is UNBOUND

#include <gtest/gtest.h>

#include "../src/handle_table.hpp"
#include "../src/time_source.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"

// reactor-c headers lack extern "C" guards
extern "C" {
#include "core/environment.h"
}

using ros2_lf_binding::g_handle_table;
using ros2_lf_binding::g_lf_clock;
using ros2_lf_binding::HandleEntry;
using ros2_lf_binding::HandleKind;

// ── Stub for lfc-generated symbol ────────────────────────────────────────────
// Defined in test_time_source.cpp as well; provide it again here so this
// translation unit links independently.  Guard against a multi-TU build
// by using a weak attribute (GCC/Clang only).
static environment_t g_dispatch_env;
extern "C" int _lf_get_environments(environment_t ** envs)
{
  g_dispatch_env.current_tag.time = 1'000'000'000LL;
  *envs = &g_dispatch_env;
  return 1;
}

// ── Test fixture ──────────────────────────────────────────────────────────────

class ReactionDispatchTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    g_handle_table.clear();
    g_lf_clock.reset();
    // Provide a valid clock so lf_time_advance_ros_clock() doesn't early-exit.
    ros2_lf_binding::lf_clock_init();
  }
};

// ── ros2_lf_invoke_timer guard paths ──────────────────────────────────────────

TEST_F(ReactionDispatchTest, TimerOutOfRangeLow)
{
  g_handle_table.resize(3);
  // reaction_id == -1: must not crash.
  EXPECT_NO_THROW(ros2_lf_invoke_timer(nullptr, -1));
}

TEST_F(ReactionDispatchTest, TimerOutOfRangeHigh)
{
  g_handle_table.resize(3);
  EXPECT_NO_THROW(ros2_lf_invoke_timer(nullptr, 3));
}

TEST_F(ReactionDispatchTest, TimerUnboundEntry)
{
  g_handle_table.resize(2);
  // Slot 1 is UNBOUND (default); must not crash.
  EXPECT_NO_THROW(ros2_lf_invoke_timer(nullptr, 1));
}

TEST_F(ReactionDispatchTest, TimerSlotKindMismatch_Sub)
{
  g_handle_table.resize(2);
  // Mark slot 0 as SUBSCRIPTION instead of TIMER; dispatch must be skipped safely.
  g_handle_table[0].kind = HandleKind::SUBSCRIPTION;
  g_handle_table[0].subscription.reset();
  EXPECT_NO_THROW(ros2_lf_invoke_timer(nullptr, 0));
}

// ── ros2_lf_invoke_sub guard paths ───────────────────────────────────────────

TEST_F(ReactionDispatchTest, SubOutOfRangeLow)
{
  g_handle_table.resize(3);
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, -1, "/topic"), 0);
}

TEST_F(ReactionDispatchTest, SubOutOfRangeHigh)
{
  g_handle_table.resize(3);
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 3, "/topic"), 0);
}

TEST_F(ReactionDispatchTest, SubUnboundEntry)
{
  g_handle_table.resize(2);
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "/topic"), 0);
}

TEST_F(ReactionDispatchTest, SubSlotKindMismatch_Timer)
{
  g_handle_table.resize(2);
  // Mark slot 0 as TIMER; a sub invoke must be skipped safely.
  g_handle_table[0].kind = HandleKind::TIMER;
  g_handle_table[0].timer.reset();
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "/topic"), 0);
}

// ── Service dispatch guard paths ─────────────────────────────────────────────

TEST_F(ReactionDispatchTest, SrvServerOutOfRange)
{
  g_handle_table.resize(2);
  EXPECT_EQ(ros2_lf_invoke_srv_server(nullptr, -1, "/svc"), 0);
  EXPECT_EQ(ros2_lf_invoke_srv_server(nullptr, 5, "/svc"), 0);
}

TEST_F(ReactionDispatchTest, SrvServerUnbound)
{
  g_handle_table.resize(2);
  // Slot 0 is UNBOUND; must not crash.
  EXPECT_EQ(ros2_lf_invoke_srv_server(nullptr, 0, "/svc"), 0);
}

TEST_F(ReactionDispatchTest, SrvServerWrongKind)
{
  g_handle_table.resize(2);
  // Mark slot 0 as TIMER; srv_server dispatch must be skipped safely.
  g_handle_table[0].kind = HandleKind::TIMER;
  g_handle_table[0].timer.reset();
  EXPECT_EQ(ros2_lf_invoke_srv_server(nullptr, 0, "/svc"), 0);
}

TEST_F(ReactionDispatchTest, SrvClientCbOutOfRange)
{
  g_handle_table.resize(2);
  EXPECT_EQ(ros2_lf_invoke_srv_client_cb(nullptr, -1, "/svc"), 0);
  EXPECT_EQ(ros2_lf_invoke_srv_client_cb(nullptr, 5, "/svc"), 0);
}

TEST_F(ReactionDispatchTest, SrvClientCbUnbound)
{
  g_handle_table.resize(2);
  EXPECT_EQ(ros2_lf_invoke_srv_client_cb(nullptr, 0, "/svc"), 0);
}

TEST_F(ReactionDispatchTest, SrvClientCbWrongKind)
{
  g_handle_table.resize(2);
  // Mark slot 0 as SUBSCRIPTION; client_cb dispatch must be skipped safely.
  g_handle_table[0].kind = HandleKind::SUBSCRIPTION;
  g_handle_table[0].subscription.reset();
  EXPECT_EQ(ros2_lf_invoke_srv_client_cb(nullptr, 0, "/svc"), 0);
}

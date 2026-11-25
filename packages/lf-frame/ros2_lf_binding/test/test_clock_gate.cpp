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

/// @file test_clock_gate.cpp
/// @brief Clock uses rcllf refuses or reports. The
///        executable gate (src/clock_gate.cpp) is linked into this test, so
///        rclcpp::Clock::sleep_for, create_jump_callback and now() reach the
///        hooks exactly as in a generated executable.

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "clock_guard.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/publication_gate_api.h"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

extern "C" {
#include "core/environment.h"
}

static environment_t g_test_envs[2];
static int g_test_n_envs = 1;

extern "C" int _lf_get_environments(environment_t ** envs)
{
  g_test_envs[0].current_tag.time = 1000;
  *envs = g_test_envs;
  return g_test_n_envs;
}

namespace rb = ros2_lf_binding;
using namespace std::chrono_literals;

namespace
{

constexpr int kHungExit = 3;
// Exit code of a falsification probe that saw the bug happen.
constexpr int kBugExit = 4;

/// An RCL_ROS_TIME node clock the binding attached to LF time, set to a
/// synthetic logical time.
std::shared_ptr<rclcpp::Clock> driven_clock(int64_t now_ns)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ros2_lf_attach_node_clock(clock, "driven");
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), now_ns), RCL_RET_OK);
  return clock;
}

/// An RCL_ROS_TIME clock a node drives itself: it enables the override and
/// sets it, the binding never attached it.
std::shared_ptr<rclcpp::Clock> self_driven_clock(int64_t now_ns)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), now_ns), RCL_RET_OK);
  return clock;
}

void bug(const char * what)
{
  std::fprintf(stderr, "%s\n", what);
  std::fflush(stderr);
  std::_Exit(kBugExit);
}

/// Forward-only jump threshold. A helper because braces do not protect commas
/// inside EXPECT_EXIT.
rcl_jump_threshold_t forward_threshold(int64_t min_forward_ns)
{
  return rcl_jump_threshold_t{false, {min_forward_ns}, {0}};
}

void set_time(const std::shared_ptr<rclcpp::Clock> & clock, int64_t ns)
{
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), ns), RCL_RET_OK);
}


/// A raw rcl jump callback, added without rclcpp::JumpHandler.
void raw_forward_jump_callback(const rcl_time_jump_t *, bool before_jump, void *)
{
  if (!before_jump) {
    bug("raw forward jump callback ran at an LF tag");
  }
}

/// Runs `body` the way a dispatch site runs a user callback.
template<typename Fn>
void in_reaction(const std::string & node, const std::string & callback, Fn body)
{
  rb::ScopedThreadRoleForTest role(rb::ThreadRole::LF_WORKER);
  rb::set_current_callback(node, callback);
  rb::UserCallbackScope scope;
  body();
}

/// Sleeps in a reaction on a clock the node drives itself from another thread.
void sleep_in_reaction_on_self_driven(const std::shared_ptr<rclcpp::Clock> & own)
{
  std::thread driver([&] {
      std::this_thread::sleep_for(20ms);
      set_time(own, 1000000000);
    });
  bool slept = false;
  in_reaction(
    "clock_own", "on_timer", [&] {
      slept = own->sleep_until(rclcpp::Time(int64_t{2000000}, RCL_ROS_TIME));
    });
  driver.join();
  std::fprintf(stderr, "slept=%d\n", slept ? 1 : 0);
}

/// A ROS clock with its override on that the binding never attached, as a
/// component's clock fed through its TimeSource looks to the gate.
std::shared_ptr<rclcpp::Clock> component_like_clock(int64_t now_ns)
{
  return self_driven_clock(now_ns);
}

/// The hang a ROS-time sleep in a reaction causes, made visible: logical time
/// never advances, so after 2 s a watchdog ends the process with kHungExit.
void sleep_with_watchdog(const std::shared_ptr<rclcpp::Clock> & clock)
{
  std::thread([] {
      std::this_thread::sleep_for(2s);
      std::fprintf(stderr, "sleep_for on ROS time never returned: logical time is frozen\n");
      std::fflush(stderr);
      std::_Exit(kHungExit);
    }).detach();
  clock->sleep_for(rclcpp::Duration(50ms));
}

void add_forward_handler_in_reaction()
{
  auto clock = driven_clock(1000);
  rcl_jump_threshold_t threshold{false, {1}, {0}};
  in_reaction(
    "clock_own", "on_timer", [&] {
      auto handler = clock->create_jump_callback(nullptr, [](const rcl_time_jump_t &) {},
      threshold);
    });
}

/// A node built inside the construction scope of `key`, recorded the way the
/// rcl_node_init interposer of a generated executable records it.
std::shared_ptr<rclcpp::Node> scoped_node(
  const std::string & key, const std::string & name, const std::string & ns)
{
  Ros2LfNodeConstructionScope scope(key);
  auto node = std::make_shared<rclcpp::Node>(name, ns);
  rcllf_gate_on_node_init(node->get_node_base_interface()->get_rcl_node_handle(), RCL_RET_OK);
  return node;
}

class ClockGateTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void SetUp() override
  {
    g_test_n_envs = 1;
    rb::clock_guard_reset_for_test();
    rb::lf_clock_reset_for_test();
    rb::set_clock_guard_disabled_for_test(false);
  }
};

// --- ROS-time sleeps ---------------------------------------------------------

TEST_F(ClockGateTest, RosSleepInReactionIsRefusedWithNodeAndCallback)
{
  EXPECT_EXIT(
    {
      auto clock = driven_clock(1000);
      in_reaction("talker", "on_timer", [&] {sleep_with_watchdog(clock);});
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'talker' \\(ROS node unknown\\) callback 'on_timer' sleeps on ROS time until "
    "50001000 ns \\(now 1000 ns\\)"
    ".*never return\\. Use a timer.*rcllf stops the program");
}

// Falsification: without the guard the same reaction hangs, which the
// watchdog reports with the message of the bug.
TEST_F(ClockGateTest, FalsifyRosSleepHangsWithoutGuard)
{
  EXPECT_EXIT(
    {
      rb::set_clock_guard_disabled_for_test(true);
      auto clock = driven_clock(1000);
      in_reaction("talker", "on_timer", [&] {sleep_with_watchdog(clock);});
      std::_Exit(0);
  }, ::testing::ExitedWithCode(kHungExit), "sleep_for on ROS time never returned");
}

TEST_F(ClockGateTest, SteadySleepInReactionIsAllowed)
{
  rclcpp::Clock steady(RCL_STEADY_TIME);
  bool slept = false;
  in_reaction("talker", "on_timer", [&] {slept = steady.sleep_for(rclcpp::Duration(1ms));});
  EXPECT_TRUE(slept);
}

TEST_F(ClockGateTest, OwnRosClockSleepInReactionIsAllowed)
{
  // Not attached to LF time: physical time passes and the sleep returns.
  rclcpp::Clock own(RCL_ROS_TIME);
  bool slept = false;
  in_reaction("talker", "on_timer", [&] {slept = own.sleep_for(rclcpp::Duration(1ms));});
  EXPECT_TRUE(slept);
}

// Review, finding 2: a sleep on a clock the node drives from its
// own thread returns, because that thread moves the clock. On ec35721 it is
// refused as if LF time drove the clock.
TEST_F(ClockGateTest, SelfDrivenClockSleepInReactionIsAllowed)
{
  EXPECT_EXIT(
    {
      auto own = self_driven_clock(1000);
      std::thread driver([&] {
        std::this_thread::sleep_for(20ms);
        set_time(own, 1000000000);
        });
      bool slept = false;
      in_reaction(
        "clock_own", "on_timer", [&] {
          slept = own->sleep_until(rclcpp::Time(int64_t{2000000}, RCL_ROS_TIME));
        });
      driver.join();
      std::fprintf(stderr, "slept=%d\n", slept ? 1 : 0);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "slept=1");
}

TEST_F(ClockGateTest, OwnRosClockSleepInConstructorIsAllowed)
{
  Ros2LfNodeConstructionScope scope("clock_fixture::Talker#talker");
  auto node = std::make_shared<rclcpp::Node>("talker");
  rclcpp::Clock own(RCL_ROS_TIME);
  EXPECT_TRUE(own.sleep_for(rclcpp::Duration(1ms)));
}

TEST_F(ClockGateTest, RosSleepToThePastInReactionIsAllowed)
{
  auto clock = driven_clock(5000);
  bool slept = false;
  in_reaction(
    "talker", "on_timer", [&] {
      slept = clock->sleep_until(rclcpp::Time(int64_t{4000}, RCL_ROS_TIME));
    });
  EXPECT_TRUE(slept);
}

TEST_F(ClockGateTest, RosSleepOnUserThreadWaitsForLogicalTime)
{
  auto clock = driven_clock(1000);
  std::thread tags([&] {
      std::this_thread::sleep_for(50ms);
      EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 3000), RCL_RET_OK);
    });
  rb::ScopedThreadRoleForTest role(rb::ThreadRole::OTHER);
  EXPECT_TRUE(clock->sleep_until(rclcpp::Time(int64_t{2000}, RCL_ROS_TIME)));
  tags.join();
  EXPECT_EQ(clock->now().nanoseconds(), 3000);
}

// --- Jump handlers -----------------------------------------------------------

TEST_F(ClockGateTest, ForwardJumpHandlerInReactionIsRefused)
{
  EXPECT_EXIT(
    add_forward_handler_in_reaction(), ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'clock_own' \\(ROS node unknown\\) \\(on_timer\\) adds a forward clock jump "
    "handler \\(min_forward 1 ns\\)"
    ".*React to time in a timer instead");
}

// Falsification: without the guard the handler runs at the next tag, on the
// thread that advances time, outside any reaction of its node.
TEST_F(ClockGateTest, FalsifyForwardJumpHandlerFiresAtEveryTag)
{
  rb::set_clock_guard_disabled_for_test(true);
  auto clock = driven_clock(1000);
  int fired = 0;
  rcl_jump_threshold_t threshold{false, {1}, {0}};
  auto handler = clock->create_jump_callback(
    nullptr, [&](const rcl_time_jump_t &) {++fired;}, threshold);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 2000), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 3000), RCL_RET_OK);
  EXPECT_EQ(fired, 2);
}

TEST_F(ClockGateTest, BackwardAndClockChangeHandlersAreAllowed)
{
  auto clock = driven_clock(1000);
  rcl_jump_threshold_t threshold{true, {0}, {-1}};  // tf2_ros::Buffer's threshold
  in_reaction(
    "tf_user", "on_timer", [&] {
      auto handler = clock->create_jump_callback(nullptr, [](const rcl_time_jump_t &) {},
      threshold);
      EXPECT_NE(handler, nullptr);
    });
}

// Review, finding 1: a forward callback added straight through
// rcl_clock_add_jump_callback is a forward handler too. On ec35721 it is not
// refused and runs at the next tag.
TEST_F(ClockGateTest, RawForwardJumpCallbackInReactionIsRefused)
{
  EXPECT_EXIT(
    {
      auto clock = driven_clock(1000);
      in_reaction(
        "clock_own", "on_timer", [&] {
          EXPECT_EQ(
            rcl_clock_add_jump_callback(
              clock->get_clock_handle(), forward_threshold(1), raw_forward_jump_callback,
              nullptr), RCL_RET_OK);
        });
      set_time(clock, 2000);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'clock_own' \\(ROS node unknown\\) \\(on_timer\\) adds a forward clock jump "
    "handler \\(min_forward 1 ns\\)");
}

// rcl_timer_init adds its own forward callback to every timer on a ROS clock.
// It runs no user code and must not be refused, neither when the timer is made
// in a reaction nor when a clock with a timer is attached.
TEST_F(ClockGateTest, TimersOnLfClockAreAllowed)
{
  EXPECT_EXIT(
    {
      auto node = std::make_shared<rclcpp::Node>("timer_node");
      auto early = rclcpp::create_timer(node, node->get_clock(), 10ms, [] {});
      ros2_lf_attach_node_clock(node->get_clock(), "timer_node");
      rclcpp::TimerBase::SharedPtr late;
      in_reaction(
        "timer_node", "on_timer", [&] {
          late = rclcpp::create_timer(node, node->get_clock(), 10ms, [] {});
        });
      set_time(node->get_clock(), 2000);
      std::fprintf(stderr, "timers kept\n");
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "timers kept");
}

// Review round 2, finding 1: the registry keyed raw clock
// addresses forever. A self-driven clock that reuses the address of a
// destroyed attached clock is not LF-driven, and its sleep must not be refused.
TEST_F(ClockGateTest, SelfDrivenClockAtAddressOfDestroyedLfClockSleepIsAllowed)
{
  EXPECT_EXIT(
    {
      const rcl_clock_t * old_handle = nullptr;
      {
        auto attached = driven_clock(1000);
        old_handle = attached->get_clock_handle();
      }
      std::vector<std::shared_ptr<rclcpp::Clock>> others;
      std::shared_ptr<rclcpp::Clock> reused;
      for (int i = 0; i < 256 && !reused; ++i) {
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        if (clock->get_clock_handle() == old_handle) {
          reused = clock;
        } else {
          others.push_back(clock);
        }
      }
      if (!reused) {
        bug("no clock reused the address, the test proves nothing");
      }
      EXPECT_EQ(rcl_enable_ros_time_override(reused->get_clock_handle()), RCL_RET_OK);
      set_time(reused, 1000);
      sleep_in_reaction_on_self_driven(reused);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "slept=1");
}

// Review round 2, finding 3: a constructor that sleeps on another, existing
// node's clock sleeps on physical time. Only the node under construction's
// clock will follow LF time.
TEST_F(ClockGateTest, OtherNodeClockSleepInConstructorIsAllowed)
{
  EXPECT_EXIT(
    {
      auto other = std::make_shared<rclcpp::Node>("other");
      Ros2LfNodeConstructionScope scope("clock_fixture::Talker#talker");
      auto node = std::make_shared<rclcpp::Node>("talker");
      const bool slept = other->get_clock()->sleep_for(rclcpp::Duration(1ms));
      std::fprintf(stderr, "slept=%d\n", slept ? 1 : 0);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "slept=1");
}

// Review round 4, fails open: a component's clocks are not attached by the
// binding, so a sleep on one in a reaction is not refused. LF time cannot move
// during the reaction, and the sleep hangs as it does without rcllf's gate.
TEST_F(ClockGateTest, ComponentClockSleepInReactionFailsOpen)
{
  EXPECT_EXIT(
    {
      auto clock = component_like_clock(1000);
      in_reaction("ClockRosSleepComponent", "on_timer", [&] {sleep_with_watchdog(clock);});
      std::_Exit(0);
  }, ::testing::ExitedWithCode(kHungExit), "sleep_for on ROS time never returned");
}

// Review round 4, finding 1: a clock the binding never attached may stand
// still longer than any timeout and still move later. A node drives its own
// ROS clock from a thread after 3 s while a reaction sleeps on it. The sleep
// returns, the program is not stopped.
TEST_F(ClockGateTest, SelfDrivenClockThatMovesAfterThreeSecondsIsAllowed)
{
  EXPECT_EXIT(
    {
      std::thread([] {
        std::this_thread::sleep_for(10s);
        std::fprintf(stderr, "the reaction sleep never returned\n");
        std::fflush(stderr);
        std::_Exit(kHungExit);
      }).detach();
      auto own = self_driven_clock(1000);
      std::thread driver([&] {
        std::this_thread::sleep_for(3s);
        set_time(own, 1000000000);
        });
      bool slept = false;
      in_reaction(
        "clock_own", "on_timer", [&] {
          slept = own->sleep_until(rclcpp::Time(int64_t{2000000}, RCL_ROS_TIME));
        });
      driver.join();
      std::fprintf(stderr, "slept=%d\n", slept ? 1 : 0);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "slept=1");
}

// Review round 3, fails open: a constructor runs outside a reaction, so a
// sleep on the node clock is not refused.
TEST_F(ClockGateTest, NodeClockSleepInConstructorFailsOpen)
{
  Ros2LfNodeConstructionScope scope("clock_fixture::Talker#talker");
  auto node = std::make_shared<rclcpp::Node>("talker");
  EXPECT_TRUE(node->get_clock()->sleep_for(rclcpp::Duration(1ms)));
}

// Review round 3, fails open: a forward handler added before the binding
// attaches the clock is not refused. rcl's handler array has no safe reader.
TEST_F(ClockGateTest, ForwardHandlerAddedBeforeAttachFailsOpen)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto handler = clock->create_jump_callback(
    nullptr, [](const rcl_time_jump_t &) {}, forward_threshold(5));
  ros2_lf_attach_node_clock(clock, "clock_own");
  set_time(clock, 2000);
}

// Review round 2, finding 2: a thread already sleeping on the clock when the
// binding attaches it is not refused. ROS time is seeded and moved past the
// deadline before the join, so the sleeper always returns.
TEST_F(ClockGateTest, ThreadSleepingWhenClockIsAttachedIsAllowed)
{
  EXPECT_EXIT(
    {
      std::thread([] {
        std::this_thread::sleep_for(5s);
        std::fprintf(stderr, "the sleeping thread never returned\n");
        std::fflush(stderr);
        std::_Exit(kHungExit);
      }).detach();
      auto clock = self_driven_clock(1000);
      bool slept = false;
      std::thread sleeper([&] {
        slept = clock->sleep_until(rclcpp::Time(int64_t{2000000}, RCL_ROS_TIME));
        });
      for (;; ) {
        {
          std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
          if (clock->get_clock_handle()->num_jump_callbacks > 0) {
            break;
          }
        }
        std::this_thread::sleep_for(1ms);
      }
      ros2_lf_attach_node_clock(clock, "clock_own");
      set_time(clock, 3000000);
      sleeper.join();
      std::fprintf(stderr, "slept=%d\n", slept ? 1 : 0);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "slept=1");
}

// rclcpp_action's goal expire thread waits on a
// ClockConditionalVariable over the node clock, and the wait adds a forward
// jump handler that only wakes the waiter. Before the fix rcllf stopped every
// action server with an external ingress for "adds a forward clock jump
// handler".
TEST_F(ClockGateTest, ConditionWaitOffReactionOnLfClockIsAllowed)
{
  EXPECT_EXIT(
    {
      auto clock = driven_clock(1000);
      bool returned = false;
      std::thread waiter([&] {
        rb::ScopedThreadRoleForTest role(rb::ThreadRole::OTHER);
        rclcpp::ClockConditionalVariable cv(clock);
        std::unique_lock<std::mutex> lock(cv.mutex());
        cv.wait_until(lock, rclcpp::Time(int64_t{3000}, RCL_ROS_TIME), [] {return false;});
        returned = true;
        });
      for (;; ) {
        {
          std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
          if (clock->get_clock_handle()->num_jump_callbacks > 0) {
            break;
          }
        }
        std::this_thread::sleep_for(1ms);
      }
      set_time(clock, 5000);
      waiter.join();
      std::fprintf(stderr, "returned=%d\n", returned ? 1 : 0);
      std::_Exit(0);
  }, ::testing::ExitedWithCode(0), "returned=1");
}

TEST_F(ClockGateTest, ConditionWaitInReactionIsStillRefused)
{
  EXPECT_EXIT(
    {
      auto clock = driven_clock(1000);
      in_reaction(
        "clock_own", "on_timer", [&] {
          rclcpp::ClockConditionalVariable cv(clock);
          std::unique_lock<std::mutex> lock(cv.mutex());
          cv.wait_until(lock, rclcpp::Time(int64_t{3000}, RCL_ROS_TIME), [] {return false;});
        });
      std::_Exit(0);
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'clock_own' \\(ROS node unknown\\) \\(on_timer\\) adds a forward clock jump "
    "handler \\(min_forward 1 ns\\).*React to time in a timer instead");
}

// --- Several LF environments -------------------------------------------------

TEST_F(ClockGateTest, SeveralEnvironmentsAreRefused)
{
  EXPECT_EXIT(
    {
      g_test_n_envs = 2;
      rb::lf_clock_init();
      rb::lf_time_advance_ros_clock();
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "The LF program has 2 environments \\(LF enclaves\\)"
    ".*one logical timeline.*several environments");
}

TEST_F(ClockGateTest, OneEnvironmentAdvancesTime)
{
  rb::lf_clock_init();
  rb::lf_time_advance_ros_clock();
  EXPECT_TRUE(rb::lf_time_started());
  EXPECT_EQ(rb::g_lf_clock->now().nanoseconds(), 1000);
}

// --- Reads the binding cannot drive ------------------------------------------

TEST_F(ClockGateTest, OwnRosClockReadInReactionIsReportedOncePerCallback)
{
  rclcpp::Clock own(RCL_ROS_TIME);
  auto driven = driven_clock(1000);
  in_reaction(
    "clock_own", "on_timer", [&] {
      driven->now();
      EXPECT_EQ(rb::clock_diagnostic_count(), 0u);
      own.now();
      own.now();
    });
  EXPECT_EQ(rb::clock_diagnostic_count(), 1u);
  in_reaction("clock_own", "on_other", [&] {own.now();});
  EXPECT_EQ(rb::clock_diagnostic_count(), 2u);
  own.now();  // outside a reaction: not reported
  EXPECT_EQ(rb::clock_diagnostic_count(), 2u);
}

TEST_F(ClockGateTest, ReadBeforeFirstTagInConstructorIsReportedOncePerNode)
{
  auto clock = driven_clock(0);
  {
    Ros2LfNodeConstructionScope scope("clock_fixture::ClockTimer#clock_timer");
    EXPECT_EQ(clock->now().nanoseconds(), 0);
    clock->now();
  }
  EXPECT_EQ(rb::clock_diagnostic_count(), 1u);
  rb::lf_clock_init();
  rb::lf_time_advance_ros_clock();
  {
    // A node built after the first tag reads a logical time: not reported.
    Ros2LfNodeConstructionScope scope("late#late");
    clock->now();
  }
  EXPECT_EQ(rb::clock_diagnostic_count(), 1u);
}

// --- Messages name the ROS node ------------------------------------------------

TEST_F(ClockGateTest, RosSleepRefusalNamesTheRosNode)
{
  EXPECT_EXIT(
    {
      auto node = scoped_node("ClockKey", "clock_named", "/ns");
      auto clock = driven_clock(1000);
      in_reaction("ClockKey", "on_timer", [&] {sleep_with_watchdog(clock);});
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'ClockKey' \\(ROS node '/ns/clock_named'\\) callback 'on_timer' sleeps on ROS time");
}

TEST_F(ClockGateTest, JumpHandlerRefusalListsEveryRosNodeOfTheKey)
{
  EXPECT_EXIT(
    {
      auto first = scoped_node("ClockKey", "clock_b", "/");
      auto second = scoped_node("ClockKey", "clock_a", "/");
      auto clock = driven_clock(1000);
      in_reaction(
        "ClockKey", "on_timer", [&] {
          auto handler = clock->create_jump_callback(
            nullptr, [](const rcl_time_jump_t &) {}, forward_threshold(1));
        });
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'ClockKey' \\(ROS nodes '/clock_a', '/clock_b'\\) \\(on_timer\\) adds a forward");
}

TEST_F(ClockGateTest, RosSleepRefusalSaysWhenTheRosNodeIsUnknown)
{
  EXPECT_EXIT(
    {
      auto clock = driven_clock(1000);
      in_reaction("talker", "on_timer", [&] {sleep_with_watchdog(clock);});
  }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "Node 'talker' \\(ROS node unknown\\) callback 'on_timer' sleeps on ROS time");
}

TEST_F(ClockGateTest, PreTagWarnAndOwnClockErrorNameTheRosNode)
{
  auto driven = driven_clock(0);
  rclcpp::Clock own(RCL_ROS_TIME);
  ::testing::internal::CaptureStderr();
  std::shared_ptr<rclcpp::Node> node;
  {
    Ros2LfNodeConstructionScope scope("ClockKey");
    node = std::make_shared<rclcpp::Node>("clock_named", "/ns");
    rcllf_gate_on_node_init(node->get_node_base_interface()->get_rcl_node_handle(), RCL_RET_OK);
    driven->now();
  }
  in_reaction("ClockKey", "on_timer", [&] {own.now();});
  const std::string log = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(
    log.find("Node 'ClockKey' (ROS node '/ns/clock_named') reads ROS time while it is "
    "constructed"), std::string::npos) << log;
  EXPECT_NE(
    log.find("Node 'ClockKey' (ROS node '/ns/clock_named') callback 'on_timer' reads an "
    "RCL_ROS_TIME clock"), std::string::npos) << log;
  EXPECT_EQ(rb::clock_diagnostic_count(), 2u);
}

TEST_F(ClockGateTest, MessageTexts)
{
  EXPECT_EQ(
    rb::own_clock_read_message("n", {"/ns/n"}, "cb"),
    "Node 'n' (ROS node '/ns/n') callback 'cb' reads an RCL_ROS_TIME clock that does not "
    "follow LF logical time (a clock the node built itself, not get_clock()). It returns "
    "physical time, not the logical tag. Use this->now() or this->get_clock(). "
    "Logged once per callback.");
  EXPECT_NE(
    rb::pre_tag_read_message("n", {}, 0).find("It got 0 ns (0, the clock waits for the first tag)"),
    std::string::npos);
  EXPECT_NE(
    rb::pre_tag_read_message("n", {}, 7).find("It got 7 ns (physical time, the clock follows LF "
    "time once the node is attached)"), std::string::npos);
}

}  // namespace

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

// ros2_lf_bind_node bound every timer RID to the first timer with
// a matching period and every subscription RID to the first subscription on
// its topic. Two same-period timers (or two same-topic subscriptions) in one
// node then shared one handle and the second callback never ran. These tests
// drive the real ros2_lf_bind_node on such a node and check that every RID
// gets its own handle, in creation order, and that a RID without a handle or a
// handle without a RID refuses to start.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "../src/component_manager.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "handle_matching.hpp"  // NOLINT(build/include_subdir)
#include "handle_table.hpp"  // NOLINT(build/include_subdir)
#include "node_registry_internal.h"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_test_teardown.hpp"  // NOLINT(build/include_subdir)
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ingress.hpp"  // NOLINT(build/include_subdir)
#include "time_source.hpp"  // NOLINT(build/include_subdir)

namespace ros2_lf_binding
{
extern std::shared_ptr<LFComponentManager> g_manager;
}  // namespace ros2_lf_binding

namespace
{

using namespace std::chrono_literals;
using ros2_lf_binding::g_handle_table;
using ros2_lf_binding::RidEntry;
using Clock = rosgraph_msgs::msg::Clock;

/// Two same-period timers and two same-topic subscriptions, like the W1 clock
/// node (timing_gaps) and handle_matching_example's twin subscriber.
class TwinNode : public rclcpp::Node
{
public:
  explicit TwinNode(
    const std::string & name, bool extra_timer = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(name, options)
  {
    timer_1_ = create_wall_timer(100ms, [] {});
    timer_2_ = create_wall_timer(100ms, [] {});
    first_ = create_subscription<Clock>("twin", 10, [](Clock::ConstSharedPtr) {});
    second_ = create_subscription<Clock>("twin", 10, [](Clock::ConstSharedPtr) {});
    if (extra_timer) {
      extra_ = create_wall_timer(100ms, [] {});
    }
  }

  rclcpp::TimerBase::SharedPtr timer_1_, timer_2_, extra_;
  rclcpp::SubscriptionBase::SharedPtr first_, second_;
};

/// Registers the analyzer's view of TwinNode under `cls`, RIDs base..base+3.
void register_twin_rids(const std::string & cls, int base, bool third_timer = false)
{
  ros2_lf_binding::register_rid_entry({base + 0, cls, "timer_1_callback", "timer", "",
      100'000'000LL, 0});
  ros2_lf_binding::register_rid_entry({base + 1, cls, "timer_2_callback", "timer", "",
      100'000'000LL, 1});
  ros2_lf_binding::register_rid_entry({base + 2, cls, "on_first", "sub", "twin", 0, 0});
  ros2_lf_binding::register_rid_entry({base + 3, cls, "on_second", "sub", "twin", 0, 1});
  int top = base + 4;
  if (third_timer) {
    ros2_lf_binding::register_rid_entry({top++, cls, "timer_3_callback", "timer", "",
        100'000'000LL, 2});
  }
  ros2_lf_binding::resize_handle_table(static_cast<std::size_t>(top));
}

class HandleMatchingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    ros2_lf_binding::g_manager = std::make_shared<ros2_lf_binding::LFComponentManager>(
      std::weak_ptr<rclcpp::Executor>(), "test_handle_matching_manager",
      rclcpp::NodeOptions());
  }
  static void TearDownTestSuite()
  {
    // LifecycleHandlesBindWhenTheNodeCreatesThem leaves its node
    // pinned in the lifecycle gate; unreleased, the node and its DDS
    // participant outlived the suite and crashed the process in exit().
    ros2_lf_binding::g_manager.reset();
    rcllf_test::expect_rclcpp_finalized_or_exit();
  }
};

TEST_F(HandleMatchingTest, SamePeriodTimersAndSameTopicSubscriptionsBindOneToOne)
{
  auto node = std::make_shared<TwinNode>("twin_node_161");
  ros2_lf_register_node("TwinNode161", node);
  register_twin_rids("TwinNode161", 0);

  ros2_lf_bind_node(nullptr, "TwinNode161");

  ASSERT_GE(g_handle_table.size(), 4u);
  auto t0 = g_handle_table[0].timer.lock();
  auto t1 = g_handle_table[1].timer.lock();
  auto s2 = g_handle_table[2].subscription.lock();
  auto s3 = g_handle_table[3].subscription.lock();
  EXPECT_NE(t0, t1) << "timer handle bound twice: RID 0 and RID 1 share one timer";
  EXPECT_NE(s2, s3) << "subscription handle bound twice: RID 2 and RID 3 share one "
    "subscription";
  // Creation order: the first-created entity serves the first-created callback.
  EXPECT_EQ(t0, node->timer_1_);
  EXPECT_EQ(t1, node->timer_2_);
  EXPECT_EQ(s2, node->first_);
  EXPECT_EQ(s3, node->second_);
}

TEST_F(HandleMatchingTest, RidWithoutHandleRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<TwinNode>("twin_node_161_missing");
      ros2_lf_register_node("TwinNode161Missing", node);
      register_twin_rids("TwinNode161Missing", 10, /*third_timer=*/ true);
      ros2_lf_bind_node(nullptr, "TwinNode161Missing");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "RID 14 \\(TwinNode161Missing::timer_3_callback, timer, period 100000000 ns\\) has no "
    "unbound timer handle in node 'TwinNode161Missing' that fits it \\(2 timer handle\\(s\\) "
    "found, 3 expected by the analyzer\\)");
}

/// One timer of 99.5 ms where the analysis saw 100 ms.
class OffPeriodNode : public rclcpp::Node
{
public:
  explicit OffPeriodNode(const std::string & name)
  : rclcpp::Node(name)
  {
    timer_ = create_wall_timer(99500us, [] {});
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

// A timer whose period differs from the analyzed one by less than
// 1 ms used to bind, and the run followed the analyzed schedule, not the
// node's, with no error. It refuses now.
TEST_F(HandleMatchingTest, TimerOfAnotherPeriodRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<OffPeriodNode>("off_period_643");
      ros2_lf_register_node("OffPeriod643", node);
      ros2_lf_binding::register_rid_entry({30, "OffPeriod643", "timer_callback", "timer", "",
        100'000'000LL, 0});
      ros2_lf_binding::resize_handle_table(31);
      ros2_lf_bind_node(nullptr, "OffPeriod643");
      std::fprintf(stderr, "bound a 99500000 ns timer to a 100000000 ns RID\n");
      std::exit(0);
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "RID 30 \\(OffPeriod643::timer_callback, timer, period 100000000 ns\\) has no "
    "unbound timer handle in node 'OffPeriod643' that fits it");
}

TEST(TimerFits, OnlyTheExactPeriodFits)
{
  RidEntry e{0, "N", "cb", "timer", "", 100'000'000LL, 0};
  EXPECT_TRUE(ros2_lf_binding::timer_fits(100'000'000LL, e));
  EXPECT_FALSE(ros2_lf_binding::timer_fits(99'500'000LL, e)) <<
    "a 99.5 ms timer fits a 100 ms RID: the run would follow the analyzed schedule";
  EXPECT_FALSE(ros2_lf_binding::timer_fits(100'000'001LL, e)) <<
    "a timer 1 ns off the analyzed period fits";
  RidEntry unknown{0, "N", "cb", "timer", "", 0, 0};
  EXPECT_FALSE(ros2_lf_binding::timer_fits(0, unknown)) <<
    "an entry without an analyzed period (0) fits a timer";
  EXPECT_FALSE(ros2_lf_binding::timer_fits(500'000LL, unknown)) <<
    "an entry without an analyzed period (0) fits a timer";
}

TEST_F(HandleMatchingTest, HandleWithoutRidRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<TwinNode>("twin_node_161_extra", /*extra_timer=*/ true);
      ros2_lf_register_node("TwinNode161Extra", node);
      register_twin_rids("TwinNode161Extra", 20);
      ros2_lf_bind_node(nullptr, "TwinNode161Extra");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "timer handle #2 \\(period 100000000 ns, the default callback group\\) of node "
    "'TwinNode161Extra' has no reaction ID");
}

// --- Lifecycle nodes -----------------------------------------------------------

/// Creates its timer and subscription in on_activate and drops them in
/// on_deactivate, like lifecycle_example.
class LateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LateNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    timer_ = create_wall_timer(100ms, [] {});
    sub_ = create_subscription<Clock>("late", 10, [](Clock::ConstSharedPtr) {});
    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    timer_.reset();
    sub_.reset();
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

void register_late_rids(const std::string & cls, int base)
{
  ros2_lf_binding::register_rid_entry({base, cls, "on_tick", "timer", "", 100'000'000LL, 0});
  ros2_lf_binding::register_rid_entry({base + 1, cls, "on_late", "sub", "late", 0, 0});
  ros2_lf_binding::resize_handle_table(static_cast<std::size_t>(base + 2));
}

TEST_F(HandleMatchingTest, LifecycleHandlesBindWhenTheNodeCreatesThem)
{
  // auto_transition false: the node is still unconfigured at bind, so it owns
  // no timer and no subscription yet. Binding defers both instead of refusing.
  auto node = std::make_shared<LateNode>("late_node_161");
  ros2_lf_register_node("LateNode161", node, /*auto_transition=*/ false);
  register_late_rids("LateNode161", 120);
  ros2_lf_bind_node(nullptr, "LateNode161");
  EXPECT_TRUE(g_handle_table[120].late);
  EXPECT_TRUE(g_handle_table[121].late);
  EXPECT_EQ(g_handle_table[120].kind, ros2_lf_binding::HandleKind::UNBOUND);
  ros2_lf_binding::bind_late_handle(120);
  EXPECT_EQ(g_handle_table[120].kind, ros2_lf_binding::HandleKind::UNBOUND);

  node->configure();
  node->activate();
  ros2_lf_binding::bind_late_handle(120);
  ros2_lf_binding::bind_late_handle(121);
  ASSERT_EQ(g_handle_table[120].kind, ros2_lf_binding::HandleKind::TIMER);
  EXPECT_EQ(g_handle_table[120].timer.lock(), node->timer_);
  // The binding leaves the rcl timer armed (no executor spins it), so is_canceled()
  // reports only a user cancel().
  EXPECT_FALSE(node->timer_->is_canceled());
  ASSERT_EQ(g_handle_table[121].kind, ros2_lf_binding::HandleKind::SUBSCRIPTION);
  EXPECT_EQ(g_handle_table[121].subscription.lock(), node->sub_);

  // Deactivate drops both handles, and the next activation creates new ones.
  node->deactivate();
  EXPECT_TRUE(g_handle_table[120].timer.expired());
  EXPECT_TRUE(g_handle_table[121].subscription.expired());
  node->activate();
  ros2_lf_binding::bind_late_handle(120);
  ros2_lf_binding::bind_late_handle(121);
  EXPECT_EQ(g_handle_table[120].timer.lock(), node->timer_);
  EXPECT_EQ(g_handle_table[121].subscription.lock(), node->sub_);
}

TEST_F(HandleMatchingTest, ActiveLifecycleNodeWithoutHandleRefusesToStart)
{
  // Bring-up reaches active, so every handle the analyzer expects must exist
  // now: a RID that still has none is a missing callback, as for a plain node.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<LateNode>("late_node_161_active");
      ros2_lf_register_node("LateNode161Active", node);
      register_late_rids("LateNode161Active", 130);
      ros2_lf_binding::register_rid_entry(
        {132, "LateNode161Active", "on_slow", "timer", "", 500'000'000LL, 1});
      ros2_lf_binding::resize_handle_table(133);
      ros2_lf_bind_node(nullptr, "LateNode161Active");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "RID 132 \\(LateNode161Active::on_slow, timer, period 500000000 ns\\) has no unbound "
    "timer handle in node 'LateNode161Active' that fits it");
}

// --- Callback groups ----------------------------------------------------------

/// Three same-period timers created A1, B1, A2, with group B created before
/// group A. Handles are discovered group by group (default, B, A), so the
/// discovery order B1, A1, A2 differs from the creation order A1, B1, A2.
class GroupedNode : public rclcpp::Node
{
public:
  explicit GroupedNode(const std::string & name)
  : rclcpp::Node(name)
  {
    group_b_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_a_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    a1_ = create_wall_timer(100ms, [] {}, group_a_);
    b1_ = create_wall_timer(100ms, [] {}, group_b_);
    a2_ = create_wall_timer(100ms, [] {}, group_a_);
  }

  rclcpp::CallbackGroup::SharedPtr group_a_, group_b_;
  rclcpp::TimerBase::SharedPtr a1_, b1_, a2_;
};

/// The analyzer's view of GroupedNode: group B is group 0, group A group 1.
void register_grouped_rids(const std::string & cls, int base, int group_a, int group_b)
{
  ros2_lf_binding::register_rid_entry({base + 0, cls, "a1", "timer", "", 100'000'000LL, 0,
      group_a});
  ros2_lf_binding::register_rid_entry({base + 1, cls, "b1", "timer", "", 100'000'000LL, 1,
      group_b});
  ros2_lf_binding::register_rid_entry({base + 2, cls, "a2", "timer", "", 100'000'000LL, 2,
      group_a});
  ros2_lf_binding::resize_handle_table(static_cast<std::size_t>(base + 3));
}

TEST_F(HandleMatchingTest, SameKeyAcrossCallbackGroupsBindsByGroup)
{
  auto node = std::make_shared<GroupedNode>("grouped_node_161");
  ros2_lf_register_node("GroupedNode161", node);
  register_grouped_rids("GroupedNode161", 30, 1, 0);

  ros2_lf_bind_node(nullptr, "GroupedNode161");

  EXPECT_EQ(g_handle_table[30].timer.lock(), node->a1_) <<
    "RID 30 (a1) bound crosswise to another group's timer";
  EXPECT_EQ(g_handle_table[31].timer.lock(), node->b1_) <<
    "RID 31 (b1) bound crosswise to another group's timer";
  EXPECT_EQ(g_handle_table[32].timer.lock(), node->a2_) <<
    "RID 32 (a2) bound crosswise to another group's timer";
}

TEST_F(HandleMatchingTest, UnresolvedGroupAcrossCallbackGroupsRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<GroupedNode>("grouped_node_161_unknown");
      ros2_lf_register_node("GroupedNode161Unknown", node);
      register_grouped_rids("GroupedNode161Unknown", 40, -2, -2);
      ros2_lf_bind_node(nullptr, "GroupedNode161Unknown");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "RID 40 \\(GroupedNode161Unknown::a1, timer, period 100000000 ns\\) fits 3 timer handles "
    "of node 'GroupedNode161Unknown' in different callback groups \\(#0 in callback group #0, "
    "#1 in callback group #1, #2 in callback group #1\\) and the analyzer could not resolve "
    "its callback group");
}

/// A user callback group the node spins itself (automatic add disabled).
class ManualGroupNode : public rclcpp::Node
{
public:
  explicit ManualGroupNode(const std::string & name)
  : rclcpp::Node(name)
  {
    manual_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    timer_ = create_wall_timer(100ms, [] {});
    hidden_ = create_wall_timer(200ms, [] {}, manual_);
  }

  rclcpp::CallbackGroup::SharedPtr manual_;
  rclcpp::TimerBase::SharedPtr timer_, hidden_;
};

TEST_F(HandleMatchingTest, ManualGroupHandleWithoutRidRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<ManualGroupNode>("manual_group_161");
      ros2_lf_register_node("ManualGroup161", node);
      ros2_lf_binding::register_rid_entry({50, "ManualGroup161", "timer_callback", "timer", "",
        100'000'000LL, 0, -1});
      ros2_lf_binding::resize_handle_table(51);
      ros2_lf_bind_node(nullptr, "ManualGroup161");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "timer handle #1 \\(period 200000000 ns, callback group #0\\) of node 'ManualGroup161' "
    "has no reaction ID");
}

/// A library that spins its own callback group on its own executor, the way
/// tf2_ros::TransformListener does: the group is associated with that
/// executor before ros2_lf_bind_node runs.
class ForeignExecutorNode : public rclcpp::Node
{
public:
  explicit ForeignExecutorNode(const std::string & name)
  : rclcpp::Node(name)
  {
    library_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = library_group_;
    library_sub_ = create_subscription<Clock>("tf", 10, [](Clock::ConstSharedPtr) {}, options);
    library_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    library_executor_->add_callback_group(library_group_, get_node_base_interface());
    timer_ = create_wall_timer(100ms, [] {});
  }

  rclcpp::CallbackGroup::SharedPtr library_group_;
  rclcpp::SubscriptionBase::SharedPtr library_sub_;
  rclcpp::Executor::SharedPtr library_executor_;
  rclcpp::TimerBase::SharedPtr timer_;
};

TEST_F(HandleMatchingTest, ForeignExecutorHandleWithAcknowledgementIsExemptedAndLogged)
{
  auto node = std::make_shared<ForeignExecutorNode>("foreign_161_ack");
  ros2_lf_register_node("Foreign161Ack", node);
  ros2_lf_binding::register_rid_entry({60, "Foreign161Ack", "timer_callback", "timer", "",
      100'000'000LL, 0, -1});
  ros2_lf_binding::register_foreign_executor_ack("Foreign161Ack", "RCLLF-R002");
  ros2_lf_binding::resize_handle_table(61);

  testing::internal::CaptureStderr();
  ros2_lf_bind_node(nullptr, "Foreign161Ack");
  const std::string log = testing::internal::GetCapturedStderr();

  EXPECT_EQ(g_handle_table[60].timer.lock(), node->timer_);
  EXPECT_NE(
    log.find("ros2_lf_bind_node(Foreign161Ack): subscription handle #0 (topic '/tf') runs "
    "outside LF in a callback group another executor spins, under acknowledged RCLLF-R002"),
    std::string::npos) << log;
}

TEST_F(HandleMatchingTest, ForeignExecutorHandleWithoutAcknowledgementRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<ForeignExecutorNode>("foreign_161_plain");
      ros2_lf_register_node("Foreign161Plain", node);
      ros2_lf_binding::register_rid_entry({70, "Foreign161Plain", "timer_callback", "timer", "",
        100'000'000LL, 0, -1});
      ros2_lf_binding::resize_handle_table(71);
      ros2_lf_bind_node(nullptr, "Foreign161Plain");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "subscription handle #0 \\(topic '/tf'\\) of node 'Foreign161Plain' has no reaction ID "
    "and sits in a callback group another executor spins");
}

// --- rclcpp's own subscriptions -------------------------------------------------

/// use_sim_time=true with rclcpp's default clock thread, so the TimeSource
/// creates its own callback group before any user group, plus an application
/// subscription on /clock and a same-period timer in a user group.
class SimTimeClockNode : public rclcpp::Node
{
public:
  explicit SimTimeClockNode(const std::string & name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(name, options)
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    app_clock_ = create_subscription<Clock>("/clock", 10, [](Clock::ConstSharedPtr) {});
    timer_ = create_wall_timer(100ms, [] {}, group_);
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::SubscriptionBase::SharedPtr app_clock_;
  rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::NodeOptions sim_time_options()
{
  return rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
}

TEST_F(HandleMatchingTest, SimTimeAppClockSubscriptionGetsItsRid)
{
  auto node = std::make_shared<SimTimeClockNode>("sim_clock_161", sim_time_options());
  ros2_lf_register_node("SimClock161", node);
  ros2_lf_binding::register_rid_entry({80, "SimClock161", "on_clock", "sub", "clock", 0, 0,
      -1});
  ros2_lf_binding::register_rid_entry({81, "SimClock161", "timer_callback", "timer", "",
      100'000'000LL, 0, 0});
  ros2_lf_binding::resize_handle_table(82);

  ros2_lf_bind_node(nullptr, "SimClock161");

  EXPECT_EQ(g_handle_table[80].subscription.lock(), node->app_clock_);
  EXPECT_EQ(g_handle_table[81].timer.lock(), node->timer_);
}

TEST_F(HandleMatchingTest, SimTimeAppClockSubscriptionWithoutRidRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      auto node = std::make_shared<SimTimeClockNode>("sim_clock_161_norid", sim_time_options());
      ros2_lf_register_node("SimClock161NoRid", node);
      ros2_lf_binding::register_rid_entry({90, "SimClock161NoRid", "timer_callback", "timer",
        "", 100'000'000LL, 0, 0});
      ros2_lf_binding::resize_handle_table(91);
      ros2_lf_bind_node(nullptr, "SimClock161NoRid");
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "subscription handle #0 \\(topic '/clock', the default callback group\\) of node "
    "'SimClock161NoRid' has no reaction ID");
}

/// A component as the binding builds it (lf_time_node_options):
/// the TimeSource /clock subscription sits in the default group, created
/// before the application's own /clock subscription.
class ComponentClockNode : public rclcpp::Node
{
public:
  explicit ComponentClockNode(const std::string & name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(name, options)
  {
    app_clock_ = create_subscription<Clock>("/clock", 10, [](Clock::ConstSharedPtr) {});
  }

  rclcpp::SubscriptionBase::SharedPtr app_clock_;
};

TEST_F(HandleMatchingTest, ComponentTimeSourceSubscriptionIsNotMatched)
{
  auto node = std::make_shared<ComponentClockNode>(
    "component_clock_161", ros2_lf_binding::lf_time_node_options(rclcpp::NodeOptions()));
  ASSERT_TRUE(
    ros2_lf_binding::lf_attach_time_source(node->get_node_base_interface(), "ComponentClock161"));
  ros2_lf_register_node("ComponentClock161", node);
  ros2_lf_binding::register_rid_entry({100, "ComponentClock161", "on_clock", "sub", "clock", 0,
      0, -1});
  ros2_lf_binding::resize_handle_table(101);

  ros2_lf_bind_node(nullptr, "ComponentClock161");

  EXPECT_EQ(g_handle_table[100].subscription.lock(), node->app_clock_) <<
    "RID 100 bound to the TimeSource /clock subscription instead of the application's";
}

TEST_F(HandleMatchingTest, RemappedParameterEventsSubscriptionIsNotMatched)
{
  auto node = std::make_shared<TwinNode>(
    "twin_node_161_remap", false,
    rclcpp::NodeOptions().arguments({"--ros-args", "-r", "/parameter_events:=/pe_161"}));
  ros2_lf_register_node("TwinNode161Remap", node);
  register_twin_rids("TwinNode161Remap", 110);

  ros2_lf_bind_node(nullptr, "TwinNode161Remap");

  EXPECT_EQ(g_handle_table[112].subscription.lock(), node->first_);
  EXPECT_EQ(g_handle_table[113].subscription.lock(), node->second_);
}

// --- match_handles on its own -------------------------------------------------

bool same_key(const RidEntry & e, std::size_t h, const std::vector<int64_t> & keys)
{
  return e.period_ns == keys[h];
}

TEST(MatchHandles, FollowsCreationOrderNotRidOrder)
{
  // The analyzer numbered the second-created timer first.
  RidEntry a{0, "N", "second_cb", "timer", "", 5, 1};
  RidEntry b{1, "N", "first_cb", "timer", "", 5, 0};
  std::vector<int64_t> keys{5, 5};
  auto m = ros2_lf_binding::match_handles(
    "N", "timer", {&a, &b}, {{"h0", 0, ""}, {"h1", 0, ""}},
    [&keys](const RidEntry & e, std::size_t h) {return same_key(e, h, keys);});
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 1u);  // second_cb -> second-created handle
  EXPECT_EQ(m.handle_of[1], 0u);
}

TEST(MatchHandles, DeferMissingLeavesTheEntryWithoutAHandle)
{
  RidEntry present{0, "N", "present", "timer", "", 1, 0};
  RidEntry later{1, "N", "later", "timer", "", 2, 1};
  std::vector<int64_t> keys{1};
  auto fits = [&keys](const RidEntry & e, std::size_t h) {return same_key(e, h, keys);};
  ros2_lf_binding::MatchOptions defer;
  defer.defer_missing = true;
  auto m = ros2_lf_binding::match_handles("N", "timer", {&present, &later}, {{"h0", 0, ""}},
      fits, defer);
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 0u);
  ASSERT_EQ(m.deferred.size(), 1u);
  EXPECT_EQ(m.deferred[0], 1u);
  // Without the option the same input refuses.
  auto strict = ros2_lf_binding::match_handles(
    "N", "timer", {&present, &later}, {{"h0", 0, ""}}, fits);
  EXPECT_FALSE(strict.ok);
  EXPECT_NE(strict.error.find("RID 1 (N::later, timer, period 2 ns) has no unbound timer handle"),
    std::string::npos) << strict.error;
}

TEST(MatchHandles, AllowUnclaimedKeepsAHandleWithoutRid)
{
  RidEntry e{0, "N", "cb", "timer", "", 1, 0};
  std::vector<int64_t> keys{1, 7};
  auto fits = [&keys](const RidEntry & r, std::size_t h) {return same_key(r, h, keys);};
  ros2_lf_binding::MatchOptions late;
  late.allow_unclaimed = true;
  auto m = ros2_lf_binding::match_handles("N", "timer", {&e}, {{"h0", 0, ""}, {"h1", 0, ""}},
      fits, late);
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 0u);
  EXPECT_FALSE(
    ros2_lf_binding::match_handles("N", "timer", {&e}, {{"h0", 0, ""}, {"h1", 0, ""}},
    fits).ok);
}

TEST(MatchHandles, DifferentKeysBindByKey)
{
  RidEntry fast{0, "N", "fast", "timer", "", 1, 1};
  RidEntry slow{1, "N", "slow", "timer", "", 2, 0};
  std::vector<int64_t> keys{1, 2};
  auto m = ros2_lf_binding::match_handles(
    "N", "timer", {&fast, &slow}, {{"h0", 0, ""}, {"h1", 0, ""}},
    [&keys](const RidEntry & e, std::size_t h) {return same_key(e, h, keys);});
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 0u);
  EXPECT_EQ(m.handle_of[1], 1u);
}

TEST(MatchHandles, OnlyAcknowledgedForeignHandleMayStayUnbound)
{
  using ros2_lf_binding::kGroupDefault;
  using ros2_lf_binding::kGroupForeign;
  RidEntry e{0, "N", "cb", "sub", "/a", 0, 0, kGroupDefault};
  auto fits = [](const RidEntry &, std::size_t h) {return h == 1;};
  auto m = ros2_lf_binding::match_handles(
    "N", "subscription", {&e},
    {{"topic '/tf'", kGroupForeign, "RCLLF-R002"}, {"topic '/a'", kGroupDefault, ""}}, fits);
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 1u);
  ASSERT_EQ(m.exempted.size(), 1u);
  EXPECT_EQ(m.exempted[0], 0u);
  // Without the acknowledgement, and in a group of the node's own, it refuses.
  EXPECT_FALSE(
    ros2_lf_binding::match_handles(
      "N", "subscription", {&e},
      {{"topic '/tf'", kGroupForeign, ""}, {"topic '/a'", kGroupDefault, ""}}, fits).ok);
  EXPECT_FALSE(
    ros2_lf_binding::match_handles(
      "N", "subscription", {&e},
      {{"topic '/b'", 0, "RCLLF-R002"}, {"topic '/a'", kGroupDefault, ""}}, fits).ok);
}

TEST(MatchHandles, UnresolvedGroupNeverBindsForeignHandle)
{
  // A RID whose group the analyzer could not resolve must not take a handle
  // another executor spins: both executors would then run its callback.
  using ros2_lf_binding::kGroupDefault;
  using ros2_lf_binding::kGroupForeign;
  using ros2_lf_binding::kGroupUnknown;
  RidEntry e{0, "N", "cb", "sub", "/tf", 0, 0, kGroupUnknown};
  auto fits = [](const RidEntry &, std::size_t) {return true;};
  for (const char * ack : {"RCLLF-R002", ""}) {
    auto only_foreign = ros2_lf_binding::match_handles(
      "N", "subscription", {&e}, {{"topic '/tf'", kGroupForeign, ack}}, fits);
    EXPECT_FALSE(only_foreign.ok) << "ack '" << ack << "' bound RID 0 to the foreign handle";
    EXPECT_NE(only_foreign.error.find("RID 0 (N::cb, sub, topic '/tf') has no unbound"),
      std::string::npos) << only_foreign.error;
  }
  // Next to a handle of the node's own, the foreign one stays out of the
  // span check and is exempted under the acknowledgement.
  auto m = ros2_lf_binding::match_handles(
    "N", "subscription", {&e},
    {{"topic '/tf'", kGroupForeign, "RCLLF-R002"}, {"topic '/tf'", kGroupDefault, ""}}, fits);
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 1u);
  ASSERT_EQ(m.exempted.size(), 1u);
  EXPECT_EQ(m.exempted[0], 0u);
}

// --- tf2_ros::TransformListener inputs LF owns ----------------------

TEST(MatchHandles, Tf2ListenerHandlesBindOnlyToTf2ListenerEntries)
{
  using ros2_lf_binding::kGroupDefault;
  using ros2_lf_binding::kGroupForeign;
  using ros2_lf_binding::kGroupTf2Listener;
  RidEntry tf{0, "N", "tf_ingest_via_lst", "sub", "/tf", 0, 0, kGroupTf2Listener};
  RidEntry st{1, "N", "tf_static_ingest_via_lst", "sub", "/tf_static", 0, 1, kGroupTf2Listener};
  RidEntry own{2, "N", "on_tf", "sub", "/tf", 0, 2, kGroupDefault};
  const std::vector<std::string> topics{"/tf", "/tf_static", "/tf"};
  auto fits = [&topics](const RidEntry & e, std::size_t h) {return e.topic == topics[h];};
  ros2_lf_binding::HandleCandidate tf_h{"topic '/tf'", kGroupForeign, "", true};
  ros2_lf_binding::HandleCandidate st_h{"topic '/tf_static'", kGroupForeign, "", true};
  ros2_lf_binding::HandleCandidate own_h{"topic '/tf'", kGroupDefault, ""};
  // Listed first, the node's own /tf handle must still go to its own entry.
  auto m = ros2_lf_binding::match_handles(
    "N", "subscription", {&own, &tf, &st}, {tf_h, st_h, own_h}, fits);
  ASSERT_TRUE(m.ok) << m.error;
  EXPECT_EQ(m.handle_of[0], 2u);
  EXPECT_EQ(m.handle_of[1], 0u);
  EXPECT_EQ(m.handle_of[2], 1u);
  EXPECT_TRUE(m.exempted.empty());
}

TEST(MatchHandles, Tf2ListenerEntryRefusesAHandleOfAGroupLfDidNotDetach)
{
  // A plain foreign handle is still run by the tf2_ros thread: binding it
  // would give the Buffer two writers.
  using ros2_lf_binding::kGroupForeign;
  using ros2_lf_binding::kGroupTf2Listener;
  RidEntry tf{0, "N", "tf_ingest_via_lst", "sub", "/tf", 0, 0, kGroupTf2Listener};
  auto fits = [](const RidEntry &, std::size_t) {return true;};
  for (const char * ack : {"RCLLF-R002", ""}) {
    auto m = ros2_lf_binding::match_handles(
      "N", "subscription", {&tf}, {{"topic '/tf'", kGroupForeign, ack}}, fits);
    EXPECT_FALSE(m.ok) << "ack '" << ack << "' bound the tf2 entry to an undetached handle";
    EXPECT_NE(m.error.find("RID 0 (N::tf_ingest_via_lst, sub, topic '/tf') has no unbound"),
      std::string::npos) << m.error;
  }
}

TEST(MatchHandles, UnboundTf2ListenerHandleRefusesEvenWhenAcknowledged)
{
  // tf2_ros no longer runs the detached group, so an unbound handle would
  // drop every transform it carries; RCLLF-R002 cannot exempt that.
  using ros2_lf_binding::kGroupForeign;
  using ros2_lf_binding::kGroupTf2Listener;
  RidEntry tf{0, "N", "tf_ingest_via_lst", "sub", "/tf", 0, 0, kGroupTf2Listener};
  const std::vector<std::string> topics{"/tf", "/tf_static"};
  auto fits = [&topics](const RidEntry & e, std::size_t h) {return e.topic == topics[h];};
  auto m = ros2_lf_binding::match_handles(
    "N", "subscription", {&tf},
    {{"topic '/tf'", kGroupForeign, "RCLLF-R002", true},
      {"topic '/tf_static'", kGroupForeign, "RCLLF-R002", true}}, fits);
  ASSERT_FALSE(m.ok);
  EXPECT_EQ(
    m.error,
    "subscription handle #1 (topic '/tf_static') of node 'N' sits in the "
    "tf2_ros::TransformListener callback group LF owns but no reaction ID fits it. tf2_ros no "
    "longer runs that group, so the transforms it carries would never reach the Buffer");
}

TEST(Tf2Ingress, PicksTheOneGroupHoldingExactlyTfAndTfStatic)
{
  using ros2_lf_binding::Tf2GroupView;
  std::string error;
  EXPECT_EQ(
    ros2_lf_binding::pick_tf2_listener_group(
      {Tf2GroupView{{"/other"}, 0}, Tf2GroupView{{"/tf_static", "/tf"}, 0}}, &error), 1);
  // A timer in the group, a missing topic or a second such group: no pick.
  EXPECT_EQ(
    ros2_lf_binding::pick_tf2_listener_group({Tf2GroupView{{"/tf", "/tf_static"}, 1}}, &error),
    -1);
  EXPECT_EQ(ros2_lf_binding::pick_tf2_listener_group({Tf2GroupView{{"/tf"}, 0}}, &error), -1);
  EXPECT_EQ(
    ros2_lf_binding::pick_tf2_listener_group(
      {Tf2GroupView{{"/tf", "/tf_static"}, 0}, Tf2GroupView{{"/tf", "/tf_static"}, 0}}, &error),
    -1);
  EXPECT_EQ(
    error,
    "2 callback groups another executor spins hold exactly the subscriptions /tf and "
    "/tf_static; the admitted tf2_ros::TransformListener creates one");
}

/// The shape tf2_ros::TransformListener(buffer, node) gives a node: /tf and
/// /tf_static in a group of its own that a private executor thread spins.
class Tf2ShapedListenerNode : public rclcpp::Node
{
public:
  explicit Tf2ShapedListenerNode(const std::string & name)
  : rclcpp::Node(name)
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    auto count = [this](Clock::ConstSharedPtr) {++library_runs_;};
    tf_ = create_subscription<Clock>("/tf", 100, count, options);
    tf_static_ = create_subscription<Clock>("/tf_static", 100, count, options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(group_, get_node_base_interface());
    thread_ = std::thread([this] {executor_->spin();});
  }
  ~Tf2ShapedListenerNode() override {stop();}
  void stop()
  {
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::SubscriptionBase::SharedPtr tf_, tf_static_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread thread_;
  std::atomic<int> library_runs_{0};
};

TEST_F(HandleMatchingTest, DetachedTf2ListenerGroupBindsAndItsExecutorStopsRunningIt)
{
  const std::string key = "Tf2Listener91";
  ros2_lf_binding::register_rid_entry({80, key, "tf_ingest_via_lst", "sub", "/tf", 0, 0,
      ros2_lf_binding::kGroupTf2Listener});
  ros2_lf_binding::register_rid_entry({81, key, "tf_static_ingest_via_lst", "sub",
      "/tf_static", 0, 1, ros2_lf_binding::kGroupTf2Listener});
  ros2_lf_binding::resize_handle_table(82);
  std::shared_ptr<Tf2ShapedListenerNode> node;
  {
    Ros2LfNodeConstructionScope scope(key);
    node = std::make_shared<Tf2ShapedListenerNode>("tf2_listener_91");
    ros2_lf_register_node(key, node);
  }
  EXPECT_FALSE(node->group_->can_be_taken_from().load());
  EXPECT_TRUE(ros2_lf_binding::is_detached_tf2_group(node->group_.get()));
  // The private executor spins from construction on, so a /tf sample from
  // another process on the domain (CI runs packages in parallel) may run the
  // group before the detach ends. Only runs after the detach count.
  const int runs_at_detach = node->library_runs_.load();

  ros2_lf_bind_node(nullptr, key.c_str());
  EXPECT_EQ(g_handle_table[80].subscription.lock(), node->tf_);
  EXPECT_EQ(g_handle_table[81].subscription.lock(), node->tf_static_);

  // Messages on both topics: the private executor must not run the group.
  auto pub_node = std::make_shared<rclcpp::Node>("tf2_listener_91_pub");
  auto tf_pub = pub_node->create_publisher<Clock>("/tf", 100);
  auto st_pub = pub_node->create_publisher<Clock>("/tf_static", 100);
  const auto until = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < until &&
    (tf_pub->get_subscription_count() == 0 || st_pub->get_subscription_count() == 0))
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(tf_pub->get_subscription_count(), 0u);
  for (int i = 0; i < 5; ++i) {
    tf_pub->publish(Clock());
    st_pub->publish(Clock());
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(node->library_runs_.load() - runs_at_detach, 0) <<
    "the tf2_ros-shaped executor ran the detached group";

  // Review finding F2: a callback in flight at detach time sets the flag back
  // when it returns. The check after each LF ingest clears it again and names
  // the node; with nothing re-enabled it reports nothing.
  EXPECT_TRUE(ros2_lf_binding::reassert_tf2_groups_detached().empty());
  node->group_->can_be_taken_from().store(true);
  EXPECT_EQ(ros2_lf_binding::reassert_tf2_groups_detached(), std::vector<std::string>{key});
  EXPECT_FALSE(node->group_->can_be_taken_from().load());
  EXPECT_TRUE(ros2_lf_binding::reassert_tf2_groups_detached().empty());
  node->stop();  // the registry keeps the node past rclcpp shutdown
}

TEST_F(HandleMatchingTest, Tf2ListenerEntriesWithoutAListenerGroupRefuseToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      ros2_lf_binding::register_rid_entry({90, "NoListener91", "tf_ingest_via_lst", "sub",
        "/tf", 0, 0, ros2_lf_binding::kGroupTf2Listener});
      ros2_lf_binding::resize_handle_table(91);
      Ros2LfNodeConstructionScope scope("NoListener91");
      ros2_lf_register_node("NoListener91", std::make_shared<rclcpp::Node>("no_listener_91"));
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "tf2 ingress: node 'NoListener91': 0 callback groups another executor "
    "spins hold exactly the subscriptions /tf and /tf_static");
}

/// Counts its timer callback, like action_cancel_client's one-shot send_goal timer.
class CountingTimerNode : public rclcpp::Node
{
public:
  explicit CountingTimerNode(const std::string & name)
  : rclcpp::Node(name)
  {
    timer_ = create_wall_timer(100ms, [this] {++fired_;});
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int fired_ = 0;
};

TEST_F(HandleMatchingTest, UserTimerCancelAndResetAreHonoured)
{
  // bind_timer cancelled the rcl timer itself and
  // ros2_lf_invoke_timer ran the callback on every LF tick, so a user cancel()
  // changed nothing under lfc and a one-shot timer fired again.
  auto node = std::make_shared<CountingTimerNode>("counting_timer_122");
  ros2_lf_register_node("CountingTimer122", node);
  ros2_lf_binding::register_rid_entry({140, "CountingTimer122", "on_tick", "timer", "",
      100'000'000LL, 0});
  ros2_lf_binding::resize_handle_table(141);
  ros2_lf_bind_node(nullptr, "CountingTimer122");
  ASSERT_EQ(g_handle_table[140].kind, ros2_lf_binding::HandleKind::TIMER);

  ros2_lf_invoke_timer(nullptr, 140);
  EXPECT_EQ(node->fired_, 1);
  node->timer_->cancel();
  ros2_lf_invoke_timer(nullptr, 140);
  ros2_lf_invoke_timer(nullptr, 140);
  EXPECT_EQ(node->fired_, 1) << "a user cancel() is ignored: the timer fired after cancel";
  node->timer_->reset();
  ros2_lf_invoke_timer(nullptr, 140);
  EXPECT_EQ(node->fired_, 2) << "a user reset() does not re-arm the timer";
}

/// Two action servers of one type on one node, like action_example's dual_server.
class TwoServerNode : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  explicit TwoServerNode(const std::string & name)
  : rclcpp::Node(name)
  {
    a_ = make("fib_a_122");
    b_ = make("fib_b_122");
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr make(const std::string & action)
  {
    return rclcpp_action::create_server<Fibonacci>(
      this, action,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {});
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr a_, b_;
};

TEST_F(HandleMatchingTest, TwoActionServersOnOneNodeBindByActionName)
{
  // The binding took the node's first action server for
  // every action RID, so both RIDs below dispatched through fib_a_122.
  auto node = std::make_shared<TwoServerNode>("two_server_122");
  ros2_lf_register_node("TwoServer122", node);
  ros2_lf_binding::register_rid_entry({150, "TwoServer122", "handle_goal_b", "action_goal_server",
      "fib_b_122", 0, 0});
  ros2_lf_binding::register_rid_entry({151, "TwoServer122", "handle_goal_a", "action_goal_server",
      "fib_a_122", 0, 0});
  ros2_lf_binding::resize_handle_table(152);
  ros2_lf_bind_node(nullptr, "TwoServer122");
  ASSERT_EQ(g_handle_table[150].kind, ros2_lf_binding::HandleKind::ACTION_SERVER);
  ASSERT_EQ(g_handle_table[151].kind, ros2_lf_binding::HandleKind::ACTION_SERVER);
  EXPECT_EQ(g_handle_table[150].action_waitable.lock(), node->b_)
    << "RID of action fib_b_122 is bound to another action server";
  EXPECT_EQ(g_handle_table[151].action_waitable.lock(), node->a_)
    << "RID of action fib_a_122 is bound to another action server";
}

TEST(CheckBoundOnce, NamesBothRids)
{
  RidEntry a{3, "N", "a_cb", "timer", "", 5, 0};
  RidEntry b{4, "N", "b_cb", "timer", "", 5, 1};
  const std::string msg = ros2_lf_binding::check_bound_once(
    "N", "timer", {&a, &b}, {{"period 5 ns", 0, ""}, {"period 5 ns", 0, ""}}, {0, 0});
  EXPECT_NE(msg.find("timer handle #0 (period 5 ns) of node 'N' bound twice"),
    std::string::npos) << msg;
  EXPECT_NE(msg.find("RID 3"), std::string::npos) << msg;
  EXPECT_NE(msg.find("RID 4"), std::string::npos) << msg;
  EXPECT_TRUE(
    ros2_lf_binding::check_bound_once(
      "N", "timer", {&a, &b}, {{"x", 0, ""}, {"y", 0, ""}}, {0, 1}).empty());
}

}  // namespace

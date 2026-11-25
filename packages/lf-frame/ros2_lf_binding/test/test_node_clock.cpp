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

/// @file test_node_clock.cpp
/// @brief The clock of every node the binding constructs follows
/// LF logical time, not physical time.
///
/// Two construction paths exist. Non-component nodes reach the binding typed,
/// through ros2_lf_register_node(). Component nodes reach it type-erased,
/// through LFComponentManager, whose create_node_options() is the only hook.
/// Each test advances a synthetic logical time with lf_time_advance_ros_clock()
/// and asserts that the node's now() equals it to the nanosecond.

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rcl/timer.h>

#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <vector>

#include "../src/component_manager.hpp"
#include "../src/node_registry_internal.h"
#include "../src/time_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "rosgraph_msgs/msg/clock.hpp"

extern "C" {
#include "core/environment.h"
}

static environment_t g_test_env;
static int64_t g_test_logical_time = 0;

extern "C" int _lf_get_environments(environment_t ** envs)
{
  g_test_env.current_tag.time = g_test_logical_time;
  *envs = &g_test_env;
  return 1;
}

namespace
{

// Synthetic tags, far from the physical epoch time (about 1.8e18 ns in 2026),
// so a node clock still on physical time can never match by accident.
const std::vector<int64_t> kTags = {
  1'000'000'000LL, 1'100'000'000LL, 1'100'000'000LL, 1'350'000'001LL, 5'000'000'000LL};

std::string physical_vs_logical(const char * who, int64_t now_ns, int64_t tag_ns)
{
  const int64_t wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const bool physical = std::llabs(now_ns - wall_ns) < 60'000'000'000LL;
  return std::string(who) + " now() returned " +
         (physical ? "physical time instead of LF logical time" :
         "neither physical time nor LF logical time") +
         ": now_ns=" + std::to_string(now_ns) +
         " tag_ns=" + std::to_string(tag_ns) +
         " (difference " + std::to_string(now_ns - tag_ns) + " ns)";
}

class ExposedManager : public ros2_lf_binding::LFComponentManager
{
public:
  using LFComponentManager::LFComponentManager;
  using LFComponentManager::create_node_options;
};

// A manager that skips the LF node options, so the component it loads has no
// TimeSource /clock subscription for the binding to feed.
class PlainOptionsManager : public ros2_lf_binding::LFComponentManager
{
public:
  using LFComponentManager::LFComponentManager;

protected:
  rclcpp::NodeOptions create_node_options(
    const std::shared_ptr<composition_interfaces::srv::LoadNode::Request> request) override
  {
    return rclcpp_components::ComponentManager::create_node_options(request);
  }
};

// Publish a stray /clock sample for about one second, long enough for
// discovery, so any node clock still listening to /clock receives it.
void publish_stray_clock(int64_t stray_ns)
{
  auto node = std::make_shared<rclcpp::Node>("clock158_stray_clock");
  auto pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  rosgraph_msgs::msg::Clock msg;
  msg.clock.sec = static_cast<int32_t>(stray_ns / 1000000000LL);
  msg.clock.nanosec = static_cast<uint32_t>(stray_ns % 1000000000LL);
  for (int i = 0; i < 20; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

std::string stray_or_logical(const char * who, int64_t now_ns, int64_t tag_ns, int64_t stray_ns)
{
  if (now_ns == stray_ns) {
    return std::string(who) + " now() was set back by a stray /clock sample instead of "
           "following LF logical time: now_ns=" + std::to_string(now_ns) +
           " tag_ns=" + std::to_string(tag_ns);
  }
  return physical_vs_logical(who, now_ns, tag_ns);
}

}  // namespace

class NodeClockTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    g_test_logical_time = 0;
    ros2_lf_clear_node_registry();
    ros2_lf_binding::lf_clock_reset_for_test();
    ros2_lf_binding::lf_clock_init();
  }

  void TearDown() override
  {
    ros2_lf_clear_node_registry();
    ros2_lf_binding::lf_clock_reset_for_test();
  }
};

TEST_F(NodeClockTest, RegisteredNodeNowEqualsLogicalTag)
{
  auto node = std::make_shared<rclcpp::Node>("clock158_plain");
  ros2_lf_register_node("Clock158Plain", node);
  ASSERT_EQ(node->get_clock()->get_clock_type(), RCL_ROS_TIME);

  for (int64_t tag : kTags) {
    g_test_logical_time = tag;
    ros2_lf_binding::lf_time_advance_ros_clock();
    const int64_t now_ns = node->now().nanoseconds();
    EXPECT_EQ(now_ns, tag) << physical_vs_logical("registered rclcpp::Node", now_ns, tag);
  }
}

TEST_F(NodeClockTest, RegisteredNodeTimerClockEqualsLogicalTag)
{
  // A timer on the node clock shares the clock's rcl handle, so its clock
  // follows LF time too. (A wall timer uses RCL_STEADY_TIME and is out of
  // scope; LF dispatches bound timers anyway.)
  auto node = std::make_shared<rclcpp::Node>("clock158_timer");
  auto timer = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration::from_seconds(0.1), []() {});
  ros2_lf_register_node("Clock158Timer", node);

  rcl_clock_t * timer_clock = nullptr;
  ASSERT_EQ(rcl_timer_clock(timer->get_timer_handle().get(), &timer_clock), RCL_RET_OK);
  for (int64_t tag : kTags) {
    g_test_logical_time = tag;
    ros2_lf_binding::lf_time_advance_ros_clock();
    rcl_time_point_value_t now_ns = 0;
    ASSERT_EQ(rcl_clock_get_now(timer_clock, &now_ns), RCL_RET_OK);
    EXPECT_EQ(now_ns, tag) << physical_vs_logical("timer clock", now_ns, tag);
  }
}

TEST_F(NodeClockTest, RegisteredLifecycleNodeNowEqualsLogicalTag)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("clock158_lifecycle");
  ros2_lf_register_node("Clock158Lifecycle", node, /*auto_transition=*/false);

  for (int64_t tag : kTags) {
    g_test_logical_time = tag;
    ros2_lf_binding::lf_time_advance_ros_clock();
    const int64_t now_ns = node->now().nanoseconds();
    EXPECT_EQ(now_ns, tag) << physical_vs_logical("registered LifecycleNode", now_ns, tag);
  }
}

TEST_F(NodeClockTest, ClockRegisteredAfterAdvanceStartsAtCurrentTag)
{
  g_test_logical_time = 2'000'000'000LL;
  ros2_lf_binding::lf_time_advance_ros_clock();

  auto node = std::make_shared<rclcpp::Node>("clock158_late");
  ros2_lf_register_node("Clock158Late", node);
  const int64_t now_ns = node->now().nanoseconds();
  EXPECT_EQ(now_ns, 2'000'000'000LL) <<
    physical_vs_logical("late-registered node", now_ns, 2'000'000'000LL);
}

TEST_F(NodeClockTest, ComponentNodeOptionsMakeNowEqualLogicalTag)
{
  // The component path: the manager's create_node_options() forces
  // use_sim_time=true (overriding a request that says false) and turns the
  // clock thread off, so the node's TimeSource owns an unspun /clock
  // subscription that the binding feeds synchronously.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<ExposedManager>(executor, "clock158_manager");
  auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  request->package_name = "unused";
  request->plugin_name = "unused::Plugin";
  rcl_interfaces::msg::Parameter p;
  p.name = "use_sim_time";
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  p.value.bool_value = false;
  request->parameters.push_back(p);

  const rclcpp::NodeOptions options = manager->create_node_options(request);
  EXPECT_FALSE(options.use_clock_thread());
  auto node = std::make_shared<rclcpp::Node>("clock158_component", options);
  bool use_sim_time = false;
  ASSERT_TRUE(node->get_parameter("use_sim_time", use_sim_time));
  EXPECT_TRUE(use_sim_time);
  // EXPECT, not ASSERT: without the hook the loop below still reports now().
  EXPECT_TRUE(ros2_lf_binding::lf_attach_time_source(node->get_node_base_interface(), "c158"));

  for (int64_t tag : kTags) {
    g_test_logical_time = tag;
    ros2_lf_binding::lf_time_advance_ros_clock();
    const int64_t now_ns = node->now().nanoseconds();
    EXPECT_EQ(now_ns, tag) << physical_vs_logical("component node", now_ns, tag);
  }
}

TEST_F(NodeClockTest, NodeWithoutTimeSourceSubscriptionIsReported)
{
  // A node built with plain options has no /clock subscription to feed:
  // attaching must say so rather than silently leave it on physical time.
  auto node = std::make_shared<rclcpp::Node>("clock158_no_sim_time");
  EXPECT_FALSE(ros2_lf_binding::lf_attach_time_source(node->get_node_base_interface(), "x"));
}

TEST_F(NodeClockTest, RegisteredSimTimeNodeIgnoresStrayClockAtSameTag)
{
  // A non-component node that sets use_sim_time=true itself has a TimeSource
  // clock thread on /clock. Registration must take it off /clock, or a stray
  // /clock sample at the same tag would set now() back, uncorrected, because
  // the binding only re-sets clocks when the tag changes.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("clock158_sim_time", options);
  ros2_lf_register_node("Clock158SimTime", node);

  const int64_t tag = 3'000'000'000LL;
  const int64_t stray = 1'500'000'001LL;
  g_test_logical_time = tag;
  ros2_lf_binding::lf_time_advance_ros_clock();
  publish_stray_clock(stray);
  ros2_lf_binding::lf_time_advance_ros_clock();  // same tag: clocks are not re-set
  const int64_t now_ns = node->now().nanoseconds();
  EXPECT_EQ(now_ns, tag) << stray_or_logical("registered use_sim_time node", now_ns, tag, stray);
  bool use_sim_time = true;
  ASSERT_TRUE(node->get_parameter("use_sim_time", use_sim_time));
  EXPECT_FALSE(use_sim_time);
}

TEST_F(NodeClockTest, LoadedComponentNowEqualsLogicalTagDespiteStrayClock)
{
  // The real load path: load_component() builds the node with the LF options
  // and attaches its TimeSource. A stray /clock sample is never taken.
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<ros2_lf_binding::LFComponentManager>(exec, "clock158_loader");
  ASSERT_TRUE(
    manager->load_component(
      "tf2_ros", "tf2_ros::StaticTransformBroadcasterNode", "comp158", *exec,
      "clock158_loaded", ""));
  auto components = manager->get_all_components();
  ASSERT_EQ(components.size(), 1u);
  // StaticTransformBroadcasterNode derives from rclcpp::Node alone, so the
  // type-erased instance pointer is the rclcpp::Node pointer.
  auto node = std::static_pointer_cast<rclcpp::Node>(components[0].get_node_instance());
  ASSERT_EQ(node->get_node_base_interface(), components[0].get_node_base_interface());

  for (int64_t tag : kTags) {
    g_test_logical_time = tag;
    ros2_lf_binding::lf_time_advance_ros_clock();
    const int64_t now_ns = node->now().nanoseconds();
    EXPECT_EQ(now_ns, tag) << physical_vs_logical("loaded component", now_ns, tag);
  }
  const int64_t stray = 1'500'000'001LL;
  publish_stray_clock(stray);
  ros2_lf_binding::lf_time_advance_ros_clock();  // same tag as the last one
  const int64_t now_ns = node->now().nanoseconds();
  EXPECT_EQ(now_ns, kTags.back()) <<
    stray_or_logical("loaded component", now_ns, kTags.back(), stray);
}

TEST_F(NodeClockTest, ComponentWhoseClockCannotFollowIsUnloaded)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<PlainOptionsManager>(exec, "clock158_plain_loader");
  EXPECT_FALSE(
    manager->load_component(
      "tf2_ros", "tf2_ros::StaticTransformBroadcasterNode", "plain158", *exec,
      "clock158_unloaded", ""));
  EXPECT_TRUE(manager->get_all_components().empty())
    << "a component whose now() stays on physical time was kept loaded";
}

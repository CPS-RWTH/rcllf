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
//
// A subscription the node owns but no RID entry matched used to
// be dropped silently (ros2_lf_bind_node logged "0 subscription(s) bound" and
// the node never received a message). Binding now refuses to start. The
// rclcpp TimeSource subscriptions are exempt, and so is a group another
// executor spins under an RCLLF-R002 acknowledgement.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "bind_check.hpp"  // NOLINT(build/include_subdir)
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "time_source.hpp"  // NOLINT(build/include_subdir)

namespace
{

class BindCheckTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

rclcpp::SubscriptionBase::SharedPtr make_sub(
  rclcpp::Node & node, const std::string & topic,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  return node.create_subscription<std_msgs::msg::Int32>(
    topic, 10, [](std_msgs::msg::Int32::ConstSharedPtr) {}, options);
}

ros2_lf_binding::SubscriptionCheck check_of(
  rclcpp::Node & node, const std::vector<const rclcpp::SubscriptionBase *> & bound = {},
  const std::string & acknowledged = "")
{
  return ros2_lf_binding::check_node_subscriptions(
    *node.get_node_base_interface(), bound, acknowledged,
    ros2_lf_binding::lf_is_time_source_subscription);
}

TEST_F(BindCheckTest, SubscriptionWithoutRidIsReported)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_node");
  auto a = make_sub(*node, "fusion/a");
  auto b = make_sub(*node, "fusion/b");
  // Only fusion/a matched a RID entry, fusion/b would never be dispatched.
  EXPECT_EQ(check_of(*node, {a.get()}).unbound, std::vector<std::string>({"/fusion/b"}))
    << "a subscription with no RID was not reported (silent loss)";
}

TEST_F(BindCheckTest, FullyBoundNodePasses)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_bound");
  auto a = make_sub(*node, "fusion/a");
  const auto topics = check_of(*node, {a.get()}).unbound;
  EXPECT_TRUE(topics.empty());
  ros2_lf_binding::refuse_unbound_subscriptions("BoundNode", topics);  // returns
}

// Builds component nodes with use_sim_time and no clock thread, so
// their TimeSource /clock and /parameter_events subscriptions are in the
// default group. The binding serves both, so they must not refuse the node,
// while a user subscription next to them still must.
TEST_F(BindCheckTest, TimeSourceSubscriptionsOfLfTimeNodeAreExempt)
{
  auto node = std::make_shared<rclcpp::Node>(
    "bind_check_lf_time", ros2_lf_binding::lf_time_node_options(rclcpp::NodeOptions()));
  ASSERT_TRUE(ros2_lf_binding::lf_attach_time_source(node->get_node_base_interface(), "t"));
  auto user = make_sub(*node, "fusion/b");
  EXPECT_EQ(check_of(*node).unbound, std::vector<std::string>({"/fusion/b"}))
    << "the TimeSource subscriptions of an lf_time node were not exempted";
  ros2_lf_binding::lf_clock_reset_for_test();
}

// Every plain rclcpp::Node owns a TimeSource /parameter_events subscription in
// its default group. A registered node with only bound user subscriptions,
// such as a sensor publisher, must bind.
TEST_F(BindCheckTest, TimeSourceParameterEventsOfPlainNodeIsExempt)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_plain_sensor");
  auto user = make_sub(*node, "fusion/b");
  EXPECT_EQ(check_of(*node).unbound, std::vector<std::string>({"/fusion/b"}))
    << "the TimeSource /parameter_events subscription of a plain node was reported";
}

// With `-r /parameter_events:=/my_events` the TimeSource subscribes to
// /my_events. It is still the TimeSource's and must not refuse the node.
TEST_F(BindCheckTest, RemappedTimeSourceParameterEventsIsExempt)
{
  auto node = std::make_shared<rclcpp::Node>(
    "bind_check_remapped",
    rclcpp::NodeOptions().arguments({"--ros-args", "-r", "/parameter_events:=/my_events"}));
  auto user = make_sub(*node, "fusion/b");
  EXPECT_EQ(
    ros2_lf_binding::resolved_parameter_events_topic(*node->get_node_base_interface()),
    "/my_events");
  EXPECT_EQ(check_of(*node).unbound, std::vector<std::string>({"/fusion/b"}))
    << "the remapped TimeSource parameter events subscription was reported";
}

// A /parameter_events subscription made by user code (for example a
// ParameterEventHandler) is not the TimeSource's and stays a refusal.
TEST_F(BindCheckTest, UserParameterEventsSubscriptionIsReported)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_plain");
  auto events = node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10, [](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr) {});
  EXPECT_EQ(check_of(*node).unbound, std::vector<std::string>({"/parameter_events"}))
    << "only the TimeSource /parameter_events subscription may be exempt";
}

// A group with automatic add disabled that no executor spins is still a
// subscription LF never dispatches.
TEST_F(BindCheckTest, UnboundSubscriptionInNonAutoGroupIsReported)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_manual_group");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = make_sub(*node, "fusion/manual", group);
  EXPECT_EQ(
    check_of(*node, {}, "RCLLF-R002").unbound, std::vector<std::string>({"/fusion/manual"}))
    << "a subscription in a callback group outside automatic add was not checked";
}

// A group another executor already spins (tf2_ros::TransformListener) is
// exempt only under an RCLLF-R002 acknowledgement.
TEST_F(BindCheckTest, ForeignExecutorGroupNeedsAcknowledgement)
{
  auto node = std::make_shared<rclcpp::Node>("bind_check_foreign_group");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = make_sub(*node, "tf", group);
  rclcpp::executors::SingleThreadedExecutor foreign;
  foreign.add_callback_group(group, node->get_node_base_interface());
  ASSERT_TRUE(group->get_associated_with_executor_atomic().load());

  EXPECT_EQ(check_of(*node).unbound, std::vector<std::string>({"/tf"}))
    << "a foreign executor group was exempt without an RCLLF-R002 acknowledgement";
  const auto acked = check_of(*node, {}, "RCLLF-R002");
  EXPECT_TRUE(acked.unbound.empty());
  EXPECT_EQ(acked.foreign_executor, std::vector<const rclcpp::SubscriptionBase *>({sub.get()}));
  foreign.remove_callback_group(group);
}

TEST_F(BindCheckTest, UnboundSubscriptionRefusesBinding)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    ros2_lf_binding::refuse_unbound_subscriptions("ExactFusionNode", {"/fusion/a", "/fusion/b"}),
    ::testing::ExitedWithCode(1),
    "ros2_lf_bind_node\\(ExactFusionNode\\): 2 subscription\\(s\\) have no reaction ID "
    "and would never be dispatched: '/fusion/a', '/fusion/b'");
}

}  // namespace

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

// Review finding F91-3. detach_tf2_listener_groups cleared the
// listener group's can_be_taken_from() flag and accepted three false reads
// 10 ms apart. A /tf callback tf2_ros's private executor was running the whole
// time reads false too, and sets the flag back to true when it returns, after
// detach returned; the executor then kept taking /tf outside LF. Here the
// first /tf callback is held in flight well past those reads, released after
// detach was called, and no /tf may reach the private executor once detach
// has returned.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_test_teardown.hpp"  // NOLINT(build/include_subdir)
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ingress.hpp"  // NOLINT(build/include_subdir)

namespace
{

using namespace std::chrono_literals;
using Clock = rosgraph_msgs::msg::Clock;

/// tf2_ros::TransformListener's shape: /tf and /tf_static in a
/// MutuallyExclusive group of their own that a private executor thread spins.
/// The first /tf callback blocks until `release_` is set.
class SlowTf2Node : public rclcpp::Node
{
public:
  SlowTf2Node()
  : rclcpp::Node("tf2_detach_inflight_91")
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    tf_ = create_subscription<Clock>(
      "/tf", 100, [this](Clock::ConstSharedPtr) {
        if (runs_.fetch_add(1) == 0) {
          entered_.store(true);
          while (!release_.load()) {
            std::this_thread::sleep_for(1ms);
          }
        }
      }, options);
    tf_static_ = create_subscription<Clock>(
      "/tf_static", 100, [this](Clock::ConstSharedPtr) {++runs_;}, options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(group_, get_node_base_interface());
    thread_ = std::thread([this] {executor_->spin();});
  }
  ~SlowTf2Node() override {stop();}
  void stop()
  {
    release_.store(true);
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::SubscriptionBase::SharedPtr tf_, tf_static_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread thread_;
  std::atomic<int> runs_{0};
  std::atomic<bool> entered_{false};
  std::atomic<bool> release_{false};
};

template<typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds limit)
{
  const auto until = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > until) {
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

TEST(Tf2DetachInFlight, ACallbackInFlightAtDetachDoesNotReenableTheGroup)
{
  rclcpp::init(0, nullptr);
  const std::string key = "Tf2InFlight91";
  ros2_lf_binding::register_rid_entry({0, key, "tf_ingest_via_lst", "sub", "/tf", 0, 0,
      ros2_lf_binding::kGroupTf2Listener});
  ros2_lf_binding::register_rid_entry({1, key, "tf_static_ingest_via_lst", "sub",
      "/tf_static", 0, 1, ros2_lf_binding::kGroupTf2Listener});
  auto node = std::make_shared<SlowTf2Node>();
  ros2_lf_register_node(key, node);

  auto pub_node = std::make_shared<rclcpp::Node>("tf2_detach_inflight_91_pub");
  auto tf_pub = pub_node->create_publisher<Clock>("/tf", 100);
  ASSERT_TRUE(wait_for([&] {return tf_pub->get_subscription_count() > 0;}, 5000ms));
  tf_pub->publish(Clock());
  ASSERT_TRUE(wait_for([&] {return node->entered_.load();}, 5000ms));

  // The callback stays in flight 300 ms, ten times the old loop's three
  // 10 ms reads and well inside its 1 s bound. A second /tf arrives while it
  // is held, so a re-enabled group has a ready message the moment it returns.
  std::thread releaser([&] {
      std::this_thread::sleep_for(100ms);
      tf_pub->publish(Clock());
      std::this_thread::sleep_for(200ms);
      node->release_.store(true);
    });
  ros2_lf_binding::detach_tf2_listener_groups(key);
  // What tf2's thread took before detach returned happened before the first
  // LF tag (node construction); only what it takes afterwards escapes LF.
  const int runs_at_detach = node->runs_.load();
  releaser.join();

  // Offer it more /tf, spaced out, for a further second. A detached group
  // never runs again.
  for (int i = 0; i < 20; ++i) {
    tf_pub->publish(Clock());
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(node->runs_.load(), runs_at_detach)
    << "tf2_ros's private executor took /tf after detach_tf2_listener_groups returned: the "
    "callback in flight at detach re-enabled the group";
  EXPECT_FALSE(node->group_->can_be_taken_from().load());
  EXPECT_TRUE(ros2_lf_binding::is_detached_tf2_group(node->group_.get()));
  // Detached means out of every wait set, so the binding's ingress can add it.
  EXPECT_FALSE(node->tf_->exchange_in_use_by_wait_set_state(node->tf_.get(), false));
  EXPECT_FALSE(node->tf_static_->exchange_in_use_by_wait_set_state(node->tf_static_.get(), false));
  node->stop();
  tf_pub.reset();
  pub_node.reset();
  node.reset();
  rcllf_test::expect_rclcpp_finalized_or_exit();
}

// F91b: the detach probe claims a subscription it finds free, which is safe
// only because tf2_ros's executor added /tf and /tf_static to its wait set
// before detach starts (tf2_ingress.cpp, point 1). A group marked as held by
// an executor whose wait set never took its subscriptions breaks that
// precondition. The old loop read "not in a wait set" as detached and
// returned; detach must refuse to start instead, before it touches the flag.
TEST(Tf2DetachInFlight, ListenerGroupNoWaitSetHoldsRefusesToStart)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
    {
      rclcpp::init(0, nullptr);
      const std::string key = "Tf2Unheld91";
      ros2_lf_binding::register_rid_entry({0, key, "tf_ingest_via_lst", "sub", "/tf", 0, 0,
        ros2_lf_binding::kGroupTf2Listener});
      ros2_lf_binding::register_rid_entry({1, key, "tf_static_ingest_via_lst", "sub",
        "/tf_static", 0, 1, ros2_lf_binding::kGroupTf2Listener});
      auto node = std::make_shared<rclcpp::Node>("tf2_detach_unheld_91");
      auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      rclcpp::SubscriptionOptions options;
      options.callback_group = group;
      auto tf = node->create_subscription<Clock>("/tf", 100, [](Clock::ConstSharedPtr) {},
      options);
      auto tf_static = node->create_subscription<Clock>(
        "/tf_static", 100, [](Clock::ConstSharedPtr) {}, options);
      // Associated with an executor whose wait set never took the subscriptions.
      group->get_associated_with_executor_atomic().store(true);
      ros2_lf_register_node(key, node);
      ros2_lf_binding::detach_tf2_listener_groups(key);
      std::fprintf(stderr, "detach accepted a group no wait set holds\n");
      std::fflush(stderr);
      std::_Exit(0);
  },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "subscription /tf of the TransformListener group is in no wait set before detach");
}

}  // namespace

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

// Review finding F2 (falsification test). A /tf callback that
// tf2_ros's private executor is running when the listener group is detached
// sets can_be_taken_from() back to true when it returns, and the executor then
// takes /tf outside LF. The binding checks the group after every LF /tf ingest
// (ros2_lf_tf2_ingested): it detaches the group again and traces
// tf_ingress_reattached, which the tf2_acq comparator fails. Before that check
// the re-enabled group stayed enabled and nothing was traced.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_test_teardown.hpp"  // NOLINT(build/include_subdir)
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "ros2_lf_binding/tf2_runtime_api.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ingress.hpp"  // NOLINT(build/include_subdir)

namespace
{

using namespace std::chrono_literals;
using Clock = rosgraph_msgs::msg::Clock;

/// tf2_ros::TransformListener's shape: /tf and /tf_static in a
/// MutuallyExclusive group of their own that a private executor thread spins.
class Tf2ShapedNode : public rclcpp::Node
{
public:
  Tf2ShapedNode()
  : rclcpp::Node("tf2_reattach_91")
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    tf_ = create_subscription<Clock>("/tf", 100, [](Clock::ConstSharedPtr) {}, options);
    tf_static_ = create_subscription<Clock>(
      "/tf_static", 100, [](Clock::ConstSharedPtr) {}, options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(group_, get_node_base_interface());
    thread_ = std::thread([this] {executor_->spin();});
  }
  ~Tf2ShapedNode() override {stop();}
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
};

TEST(Tf2Reattach, AGroupReenabledOutsideLfIsDetachedAgainAndTracedAtTheNextIngest)
{
  rclcpp::init(0, nullptr);
  const std::string key = "Tf2Reattach91";
  ros2_lf_binding::register_rid_entry({0, key, "tf_ingest_via_lst", "sub", "/tf", 0, 0,
      ros2_lf_binding::kGroupTf2Listener});
  ros2_lf_binding::register_rid_entry({1, key, "tf_static_ingest_via_lst", "sub",
      "/tf_static", 0, 1, ros2_lf_binding::kGroupTf2Listener});
  auto node = std::make_shared<Tf2ShapedNode>();
  ros2_lf_register_node(key, node);
  // The private executor must hold the subscriptions before detach (F91b).
  std::this_thread::sleep_for(200ms);
  ros2_lf_binding::detach_tf2_listener_groups(key);
  ASSERT_FALSE(node->group_->can_be_taken_from().load());

  // An LF /tf ingest with the group still detached reports nothing.
  testing::internal::CaptureStderr();
  ros2_lf_tf2_ingested(key.c_str());
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(err.find("tf_ingress_reattached"), std::string::npos) << err;

  // What a callback in flight at detach does when it returns.
  node->group_->can_be_taken_from().store(true);
  testing::internal::CaptureStderr();
  ros2_lf_tf2_ingested(key.c_str());
  err = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(node->group_->can_be_taken_from().load())
    << "the TransformListener group was re-enabled outside LF and stayed enabled after an LF "
    "/tf ingest: tf2_ros's executor keeps taking /tf outside LF";
  EXPECT_NE(
    err.find("RCLLF_TF2 {\"event\":\"tf_ingress_reattached\",\"node\":\"Tf2Reattach91\"}"),
    std::string::npos)
    << "no tf_ingress_reattached trace for a re-enabled listener group; stderr was:\n" << err;

  node->stop();
  node.reset();
  rcllf_test::expect_rclcpp_finalized_or_exit();
}

}  // namespace

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
// The wrapper must drive a managed lifecycle node to `active`
// before binding, otherwise entities created in on_configure/on_activate do
// not exist yet and the callback-group walk in ros2_lf_bind_node finds
// nothing.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "example_interfaces/action/fibonacci.hpp"
#include "lifecycle_gate.hpp"  // NOLINT(build/include_subdir)
#include "ingress.hpp"  // NOLINT(build/include_subdir)
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "name_match.hpp"  // NOLINT(build/include_subdir)
#include "component_manager.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_test_teardown.hpp"  // NOLINT(build/include_subdir)
#include "rid_table.hpp"  // NOLINT(build/include_subdir)
#include "ros2_lf_binding/action_gate_api.h"
#include "ros2_lf_binding/node_registry.h"
#include "node_registry_internal.h"  // NOLINT(build/include_subdir)
#include "rosgraph_msgs/msg/clock.hpp"
#include "handle_table.hpp"  // NOLINT(build/include_subdir)
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "thread_role.hpp"  // NOLINT(build/include_subdir)
#include "user_callback_gate.hpp"  // NOLINT(build/include_subdir)

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

/// Mirrors lifecycle_example: nothing is created in the constructor, the
/// publisher appears in on_configure and the timer in on_activate.
class LateEntityNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LateEntityNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("late_output", 10);
    configured = true;
    configure_count++;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {tick_count++;});
    activated = true;
    activate_count++;
    return CallbackReturn::SUCCESS;
  }

  /// Shadows (does not override — LifecycleNode::configure is non-virtual)
  /// the transition entry point, to count every *attempt*.
  ///
  /// The thunk holds a `shared_ptr<LateEntityNode>` and calls `n->configure()`,
  /// so static dispatch lands here. This is the only way to tell a transition
  /// the wrapper skipped from one it attempted and rcl_lifecycle rejected:
  /// on_configure runs only for a legal attempt, so `configure_count` alone
  /// reports both cases identically.
  const rclcpp_lifecycle::State & configure()
  {
    configure_attempts++;
    return rclcpp_lifecycle::LifecycleNode::configure();
  }

  const rclcpp_lifecycle::State & activate()
  {
    activate_attempts++;
    return rclcpp_lifecycle::LifecycleNode::activate();
  }

  bool configured{false};
  bool activated{false};
  int tick_count{0};
  // Successful callbacks vs. attempted transitions — see configure() above.
  int configure_count{0};
  int activate_count{0};
  int configure_attempts{0};
  int activate_attempts{0};

private:
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// A node whose on_configure fails, to prove failures are reported, not hidden.
class FailingNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FailingNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::FAILURE;
  }
};

/// Counts entities the way ros2_lf_bind_node discovers them.
int count_timers(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base)
{
  int found = 0;
  base->for_each_callback_group(
    [&found](rclcpp::CallbackGroup::SharedPtr group) {
      group->find_timer_ptrs_if(
        [&found](const rclcpp::TimerBase::SharedPtr &) {
          found++;
          return false;
        });
    });
  return found;
}

}  // namespace

class LifecycleBindingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite()
  {
    // Release the lifecycle pins and finalize the rmw context, so
    // no node and no DDS thread outlives the suite into exit().
    rcllf_test::expect_rclcpp_finalized_or_exit();
  }
  void TearDown() override {ros2_lf_clear_node_registry();}
};

TEST_F(LifecycleBindingTest, TransitionDrivesConfigureAndActivate)
{
  auto node = std::make_shared<LateEntityNode>("lc_transition_node");
  ros2_lf_register_node("LateEntityNode", node);

  ASSERT_FALSE(node->configured);
  ASSERT_FALSE(node->activated);

  EXPECT_TRUE(ros2_lf_transition_node("LateEntityNode"));

  EXPECT_TRUE(node->configured);
  EXPECT_TRUE(node->activated);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LifecycleBindingTest, TimerCreatedInOnActivateIsDiscoverable)
{
  // The actual defect: before the transition the node owns no timer, so the
  // discovery walk ros2_lf_bind_node performs would bind nothing.
  auto node = std::make_shared<LateEntityNode>("lc_discoverable_node");
  ros2_lf_register_node("DiscoverableNode", node);

  auto base = ros2_lf_get_node("DiscoverableNode");
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(count_timers(base), 0) << "timer must not exist before activation";

  ASSERT_TRUE(ros2_lf_transition_node("DiscoverableNode"));

  EXPECT_EQ(count_timers(base), 1)
    << "timer created in on_activate must be discoverable after the transition";
}

TEST_F(LifecycleBindingTest, PlainNodeIsANoOp)
{
  // A plain node has no lifecycle; transitioning must succeed trivially
  // rather than report failure.
  auto node = std::make_shared<rclcpp::Node>("plain_node");
  ros2_lf_register_node("PlainNode", node);
  EXPECT_TRUE(ros2_lf_transition_node("PlainNode"));
}

TEST_F(LifecycleBindingTest, UnregisteredNodeIsANoOp)
{
  // Component nodes live in the component manager, not the registry. Not
  // finding a key here is normal, not a failure.
  EXPECT_TRUE(ros2_lf_transition_node("NeverRegistered"));
}

TEST_F(LifecycleBindingTest, FailedConfigureIsReported)
{
  auto node = std::make_shared<FailingNode>("lc_failing_node");
  ros2_lf_register_node("FailingNode", node);
  EXPECT_FALSE(ros2_lf_transition_node("FailingNode"));
}

// The bring-up is state-directed. rcl_lifecycle answers an illegal
// transition with a WARN and the UNCHANGED current state rather than an
// exception, so a thunk that fires configure() unconditionally cannot tell a
// failed configure from a successful one, and reports a node some supervisor
// already activated as a failure. These pin each starting state.

TEST_F(LifecycleBindingTest, TransitionOnInactiveNodeActivatesWithoutReconfiguring)
{
  // A supervisor configured the node but left it inactive.
  auto node = std::make_shared<LateEntityNode>("lc_supervisor_configured");
  ros2_lf_register_node("SupervisorConfigured", node);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->configure_count, 1);

  EXPECT_TRUE(ros2_lf_transition_node("SupervisorConfigured"));

  EXPECT_EQ(
    node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->configure_count, 1);
  EXPECT_EQ(node->activate_count, 1);
  EXPECT_EQ(node->configure_attempts, 1)
    << "the wrapper must not even attempt configure() on a node already past "
       "unconfigured; rcl_lifecycle would reject it and return the unchanged "
       "state, which is indistinguishable from success";
}

TEST_F(LifecycleBindingTest, TransitionOnActiveNodeSucceedsAndDrivesNothing)
{
  // A supervisor already brought the node all the way up. Nothing is wrong,
  // so the wrapper must not report failure.
  auto node = std::make_shared<LateEntityNode>("lc_supervisor_activated");
  ros2_lf_register_node("SupervisorActivated", node);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  EXPECT_TRUE(ros2_lf_transition_node("SupervisorActivated"));

  EXPECT_EQ(node->configure_count, 1) << "no transition should have been re-driven";
  EXPECT_EQ(node->activate_count, 1);
  EXPECT_EQ(node->configure_attempts, 1) << "the wrapper attempted nothing";
  EXPECT_EQ(node->activate_attempts, 1);

  auto base = ros2_lf_get_node("SupervisorActivated");
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(count_timers(base), 1)
    << "the entities the old failure message claimed were unbound do exist";
}

TEST_F(LifecycleBindingTest, TransitionOnFinalizedNodeIsReportedAsFailure)
{
  // `active` is unreachable from `finalized`, so this is a genuine failure and
  // must stay one — the state-directed path must not turn every state into a
  // success by falling through.
  auto node = std::make_shared<LateEntityNode>("lc_finalized");
  ros2_lf_register_node("FinalizedNode", node);
  ASSERT_EQ(
    node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  EXPECT_FALSE(ros2_lf_transition_node("FinalizedNode"));
  EXPECT_EQ(node->activate_attempts, 0);
}

TEST_F(LifecycleBindingTest, AutoTransitionOptOutDrivesNothing)
{
  // A node whose activation belongs to a supervisor that has not acted yet.
  // Registering with auto_transition=false builds no thunk at all, so the
  // wrapper leaves the node exactly where it found it.
  auto node = std::make_shared<LateEntityNode>("lc_opted_out");
  ros2_lf_register_node("OptedOutNode", node, /*auto_transition=*/false);

  EXPECT_TRUE(ros2_lf_transition_node("OptedOutNode"))
    << "declining to drive a node is not a failure";

  EXPECT_EQ(node->configure_attempts, 0);
  EXPECT_EQ(node->activate_attempts, 0);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto base = ros2_lf_get_node("OptedOutNode");
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(count_timers(base), 0)
    << "binding sees only what exists; a later external activation binds nothing";
}

TEST_F(LifecycleBindingTest, RegistryDoesNotKeepNodeAlive)
{
  // The thunk captures a weak_ptr; a strong capture would make the registry
  // entry own itself and leak the node past clear().
  std::weak_ptr<LateEntityNode> weak;
  {
    auto node = std::make_shared<LateEntityNode>("lc_lifetime_node");
    weak = node;
    ros2_lf_register_node("LifetimeNode", node);
    EXPECT_FALSE(weak.expired());
  }
  // The registry still holds an owner reference here.
  EXPECT_FALSE(weak.expired());
  ros2_lf_clear_node_registry();
  EXPECT_TRUE(weak.expired()) << "node outlived the registry: thunk captured a strong ref";
}

// --- Weakly held lifecycle entities -------------

namespace
{

/// lifecycle_example's shape: the timer lives in on_activate..on_deactivate,
/// the publisher in on_configure..on_cleanup. The timer callback transitions
/// its own node in process, the path a remote caller takes too.
class SelfTransitioningNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  enum class Mode {DeactivateThenCleanup, ReactivateFailsWithError};

  SelfTransitioningNode(const std::string & name, Mode mode)
  : rclcpp_lifecycle::LifecycleNode(name), mode_(mode)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("self_output", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (fail_activate_) {
      return CallbackReturn::ERROR;
    }
    timer_ = this->create_wall_timer(std::chrono::hours(1), [this]() {on_tick();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (mode_ == Mode::DeactivateThenCleanup) {
      timer_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  void on_tick()
  {
    ticks++;
    if (!publisher_) {
      // lifecycle_example dereferences here (publisher_->publish); counted so
      // a regression reports "ticks after release" instead of SIGSEGV.
      ticks_after_release++;
      return;
    }
    if (ticks == 1) {
      if (mode_ == Mode::DeactivateThenCleanup) {
        this->deactivate();
        this->cleanup();
      } else {
        this->deactivate();
        fail_activate_ = true;
        this->activate();  // on_activate returns ERROR -> on_error runs
      }
    }
  }

  int ticks = 0;
  int ticks_after_release = 0;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  Mode mode_;
  bool fail_activate_ = false;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
};

/// Binds the node's only timer to RID 0 exactly as ros2_lf_bind_node does
/// (weak entry, timer left armed: no executor spins the node).
void bind_timer_rid0(const std::shared_ptr<SelfTransitioningNode> & node)
{
  ros2_lf_binding::g_handle_table.assign(1, ros2_lf_binding::HandleEntry{});
  auto & e = ros2_lf_binding::g_handle_table[0];
  e.kind = ros2_lf_binding::HandleKind::TIMER;
  e.timer = node->timer_;
  e.node_class = "SelfTransitioningNode";
  e.callback_name = "on_tick";
}

}  // namespace

TEST_F(LifecycleBindingTest, SelfDeactivateCleanupStopsTimerWithoutCrash)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<SelfTransitioningNode>(
    "self_cleanup_node", SelfTransitioningNode::Mode::DeactivateThenCleanup);
  node->configure();
  node->activate();
  bind_timer_rid0(node);
  const auto expired_before = ros2_lf_binding::expired_entity_count();

  for (int i = 0; i < 5; ++i) {
    ros2_lf_invoke_timer(nullptr, 0);
  }
  EXPECT_EQ(node->ticks, 1) << "a timer the node dropped in on_deactivate kept firing";
  EXPECT_EQ(node->ticks_after_release, 0) << "callback ran after on_cleanup released publisher_";
  EXPECT_EQ(ros2_lf_binding::expired_entity_count() - expired_before, 4u);
  EXPECT_EQ(
    node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ros2_lf_binding::g_handle_table.clear();
}

TEST_F(LifecycleBindingTest, ActivateErrorRunsOnErrorAndStopsTimer)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<SelfTransitioningNode>(
    "self_error_node", SelfTransitioningNode::Mode::ReactivateFailsWithError);
  node->configure();
  node->activate();
  bind_timer_rid0(node);
  const auto expired_before = ros2_lf_binding::expired_entity_count();

  for (int i = 0; i < 5; ++i) {
    ros2_lf_invoke_timer(nullptr, 0);
  }
  EXPECT_EQ(node->ticks, 1) << "a timer the node dropped in on_error kept firing";
  EXPECT_EQ(node->ticks_after_release, 0) << "callback ran after on_error released publisher_";
  EXPECT_EQ(ros2_lf_binding::expired_entity_count() - expired_before, 4u);
  EXPECT_EQ(
    node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ros2_lf_binding::g_handle_table.clear();
}

// --- Review findings on the lifecycle gate ------------

namespace ros2_lf_binding
{
extern std::shared_ptr<LFComponentManager> g_manager;
}  // namespace ros2_lf_binding

namespace
{

using ros2_lf_binding::RidEntry;

constexpr uint8_t kStateActive = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

/// Registers `entries`, sizes the handle table and runs the startup bind.
void bind_with_entries(const std::string & node_class, const std::vector<RidEntry> & entries)
{
  ros2_lf_binding::clear_rid_entries_for_test();
  for (const auto & e : entries) {
    ros2_lf_binding::register_rid_entry(e);
  }
  ros2_lf_binding::g_handle_table.clear();
  ros2_lf_binding::resize_handle_table(entries.size());
  ros2_lf_bind_node(nullptr, node_class.c_str());
}

/// A service that exists only while the node is active.
class ServiceOnActivateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ServiceOnActivateNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    service_ = this->create_service<lifecycle_msgs::srv::GetState>(
      "rebind_svc",
      [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
      std::shared_ptr<lifecycle_msgs::srv::GetState::Response>) {});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    service_.reset();
    return CallbackReturn::SUCCESS;
  }

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr service_;
};

using Fibonacci = example_interfaces::action::Fibonacci;

/// Two action servers that exist only while the node is active.
class TwoActionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TwoActionNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    alpha_ = make("alpha");
    beta_ = make("beta");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    alpha_.reset();
    beta_.reset();
    return CallbackReturn::SUCCESS;
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr alpha_;
  rclcpp_action::Server<Fibonacci>::SharedPtr beta_;

private:
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
};

class LifecycleGateBindingTest : public LifecycleBindingTest
{
protected:
  void SetUp() override
  {
    // ros2_lf_bind_node refuses to run before the runtime made its manager.
    ros2_lf_binding::g_manager = std::make_shared<ros2_lf_binding::LFComponentManager>(
      std::weak_ptr<rclcpp::Executor>(), "test_lifecycle_gate_manager", rclcpp::NodeOptions());
  }

  void TearDown() override
  {
    ros2_lf_binding::g_manager.reset();
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    ros2_lf_binding::lifecycle_gate_reset_for_test();
    ros2_lf_binding::g_handle_table.clear();
    ros2_lf_binding::clear_rid_entries_for_test();
    LifecycleBindingTest::TearDown();
  }
};

}  // namespace

// One callback deactivates and re-activates the node. The state
// ends where it began, but on_activate created a new service.
TEST_F(LifecycleGateBindingTest, SameTagDeactivateActivateRebindsService)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<ServiceOnActivateNode>("same_tag_node");
  ros2_lf_register_node("SameTagNode", node);
  bind_with_entries("SameTagNode", {{0, "SameTagNode", "handle", "srv_server", "rebind_svc", 0}});
  ASSERT_TRUE(node->service_);
  ASSERT_EQ(ros2_lf_binding::g_handle_table[0].service_server.lock(), node->service_);

  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{100}, 0u));
  node->deactivate();
  node->activate();
  ros2_lf_binding::lifecycle_gate_observe_all();

  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{200}, 0u));
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_EQ(ros2_lf_binding::g_handle_table[0].service_server.lock(), node->service_) <<
    "service entry still points at the handle on_deactivate released";
}

// An inactive node whose subscription takes in block mode must not
// hold the LF worker, and drops a sample that is already there.
TEST_F(LifecycleGateBindingTest, InactiveBlockingTakeDoesNotBlock)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<rclcpp::Node>("inactive_block_node");
  int runs = 0;
  auto sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "inactive_block_topic", 10, [&runs](const rosgraph_msgs::msg::Clock &) {runs++;});
  auto pub = node->create_publisher<rosgraph_msgs::msg::Clock>("inactive_block_topic", 10);
  ros2_lf_binding::g_handle_table.assign(1, ros2_lf_binding::HandleEntry{});
  auto & e = ros2_lf_binding::g_handle_table[0];
  e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  e.subscription = sub;
  e.node_class = "InactiveBlock";
  e.callback_name = "on_clock";
  e.topic = "inactive_block_topic";
  e.take_mode = ros2_lf_binding::TakeMode::Block;
  e.take_timeout_ms = 300;
  e.lifecycle = true;
  ros2_lf_binding::lifecycle_gate_register(
    "InactiveBlock", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // The compose file sets ROS2_LF_TAKE_MODE=skip, which overrides the entry.
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "300", 1);
  ros2_lf_binding::take_stats_reset();
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "inactive_block_topic"), 0);
  EXPECT_EQ(ros2_lf_binding::take_stats_snapshot().timeouts, 0u) <<
    "an inactive node's blocking take held the LF worker until its timeout";

  pub->publish(rosgraph_msgs::msg::Clock{});
  rclcpp::WaitSet ws({{sub}});
  ASSERT_EQ(ws.wait(std::chrono::seconds(5)).kind(), rclcpp::WaitResultKind::Ready);
  const auto dropped_before = ros2_lf_binding::lifecycle_dropped_count();
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "inactive_block_topic"), 0);
  EXPECT_EQ(ros2_lf_binding::lifecycle_dropped_count() - dropped_before, 1u);
  EXPECT_EQ(runs, 0);
  unsetenv("ROS2_LF_TAKE_MODE");
  unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
}

// The generated reader counts a stub write only when the invoke
// reports a callback, so a take that finds no message must return 0.
TEST_F(LifecycleGateBindingTest, InvokeSubReturnsCallbacksRun)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<rclcpp::Node>("count_node");
  int runs = 0;
  auto sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "count_topic", 10, [&runs](const rosgraph_msgs::msg::Clock &) {runs++;});
  auto pub = node->create_publisher<rosgraph_msgs::msg::Clock>("count_topic", 10);
  ros2_lf_binding::g_handle_table.assign(1, ros2_lf_binding::HandleEntry{});
  auto & e = ros2_lf_binding::g_handle_table[0];
  e.kind = ros2_lf_binding::HandleKind::SUBSCRIPTION;
  e.subscription = sub;
  e.node_class = "Count";
  e.callback_name = "on_clock";
  e.topic = "count_topic";
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "50", 1);
  ros2_lf_binding::take_stats_reset();

  pub->publish(rosgraph_msgs::msg::Clock{});
  rclcpp::WaitSet ws({{sub}});
  ASSERT_EQ(ws.wait(std::chrono::seconds(5)).kind(), rclcpp::WaitResultKind::Ready);
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "count_topic"), 1);
  EXPECT_EQ(runs, 1);
  EXPECT_EQ(ros2_lf_invoke_sub(nullptr, 0, "count_topic"), 0) <<
    "an empty take reported a callback";
  EXPECT_EQ(runs, 1);
  EXPECT_EQ(ros2_lf_binding::take_stats_snapshot().timeouts, 1u) <<
    "the empty blocking take was not counted as a timeout";
  unsetenv("ROS2_LF_TAKE_MODE");
  unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
}

// A lifecycle node bound while still in a transition state gets a
// gate that starts not active.
TEST_F(LifecycleGateBindingTest, NodeBoundMidTransitionIsGated)
{
  auto node = std::make_shared<rclcpp::Node>("mid_transition_node");
  ros2_lf_register_node_erased(
    "MidTransition", node, node->get_node_base_interface(), Ros2LfTransitionFn{},
    []() -> uint8_t {return lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING;});
  bind_with_entries("MidTransition", {});
  const auto view = ros2_lf_binding::lifecycle_gate_view("MidTransition");
  ASSERT_TRUE(view.has_value()) <<
    "a lifecycle node bound mid-transition got no gate and is never gated";
  EXPECT_NE(view->state, kStateActive);
}

// Two action servers created on activate, each RID keeps its own.
TEST_F(LifecycleGateBindingTest, ActionRebindMatchesActionName)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<TwoActionNode>("two_action_node");
  ros2_lf_register_node("TwoAction", node);
  bind_with_entries(
    "TwoAction", {
    {0, "TwoAction", "goal_alpha", "action_goal_server", "alpha", 0},
    {1, "TwoAction", "goal_beta", "action_goal_server", "beta", 0}});
  auto & table = ros2_lf_binding::g_handle_table;
  EXPECT_EQ(table[0].action_waitable.lock(), node->alpha_);
  EXPECT_EQ(table[1].action_waitable.lock(), node->beta_) <<
    "startup bound the beta RID to another action's server";

  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{100}, 0u));
  node->deactivate();
  ros2_lf_binding::lifecycle_gate_observe_all();
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{200}, 0u));
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(1));
  node->activate();
  ros2_lf_binding::lifecycle_gate_observe_all();
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{300}, 0u));
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(1));
  EXPECT_EQ(table[0].action_waitable.lock(), node->alpha_);
  EXPECT_EQ(table[1].action_waitable.lock(), node->beta_) <<
    "re-bind pointed the beta RID at the first action server of the node";
}

// --- Review round 2 -----------------------------------

namespace
{

/// Each activation makes a new 100 ms timer that counts its own ticks. On
/// deactivate the node keeps the timer, drops it, or hands it to another owner.
class GenerationTimerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  enum class Mode {Keep, Drop, Retire};

  GenerationTimerNode(
    const std::string & name, Mode mode,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), bool com_interface = true)
  : rclcpp_lifecycle::LifecycleNode(name, options, com_interface), mode_(mode) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const int g = ++generation;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this, g]() {ticks[g]++;});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (mode_ == Mode::Retire) {
      retired.push_back(timer_);
    }
    if (mode_ != Mode::Keep) {
      timer_.reset();
    }
    return CallbackReturn::SUCCESS;
  }

  /// Hand the timer to the other owner while the node stays as it is.
  void hand_over()
  {
    retired.push_back(timer_);
    timer_.reset();
  }

  int generation = 0;
  std::map<int, int> ticks;
  std::vector<rclcpp::TimerBase::SharedPtr> retired;  ///< the other owner

private:
  Mode mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RidEntry tick_entry(const std::string & node_class)
{
  return {0, node_class, "tick", "timer", "", 100000000, 0};
}

void at_tag(int64_t t)
{
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(t, 0u));
}

int ok_wake(void *) {return 1;}

/// A service, a client and an action server in namespace /ns, made on activate.
class NamespacedNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NamespacedNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name, "/ns") {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    service_ = this->create_service<lifecycle_msgs::srv::GetState>(
      "ns_svc",
      [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
      std::shared_ptr<lifecycle_msgs::srv::GetState::Response>) {});
    client_ = this->create_client<lifecycle_msgs::srv::GetState>("ns_peer");
    action_ = rclcpp_action::create_server<Fibonacci>(
      this, "alpha",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    service_.reset();
    client_.reset();
    action_.reset();
    return CallbackReturn::SUCCESS;
  }

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr service_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_;
};

/// Two 100 ms timers made on activate in the order first, second, and dropped
/// on deactivate. Only creation order tells them apart.
class TwoTimerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TwoTimerNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    first_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {++first;});
    second_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {++second;});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    first_.reset();
    second_.reset();
    return CallbackReturn::SUCCESS;
  }

  int first = 0;
  int second = 0;

private:
  rclcpp::TimerBase::SharedPtr first_;
  rclcpp::TimerBase::SharedPtr second_;
};

/// A waitable that puts nothing into a wait set, so no name can be read.
class NamelessWaitable : public rclcpp::Waitable
{
public:
  void add_to_wait_set(rcl_wait_set_t &) override {}
  bool is_ready(const rcl_wait_set_t &) override {return false;}
  std::shared_ptr<void> take_data() override {return nullptr;}
  void execute(const std::shared_ptr<void> &) override {}
};

}  // namespace

// Round 2, finding 1: a transition run on a thread LF does not own (the
// lifecycle service thread of a stock node), with no LF user callback after it.
TEST_F(LifecycleGateBindingTest, ExternalThreadTransitionIsSeenAtTheNextDispatch)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "external_thread_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("ExternalThread", node);
  bind_with_entries("ExternalThread", {tick_entry("ExternalThread")});
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  ASSERT_EQ(node->ticks[1], 1);

  std::thread([&node]() {node->deactivate();}).join();
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 1) <<
    "an inactive node's timer ran: a deactivate on a non-LF thread was never observed";
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));

  std::thread([&node]() {node->activate();}).join();
  at_tag(300);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[2], 1) <<
    "after an activate on a non-LF thread the gate stayed inactive or kept the old timer";
}

// Round 2, finding 1: `ros2 lifecycle set` reaches a generated node as a
// change_state request that ingress runs on an LF worker.
TEST_F(LifecycleGateBindingTest, IngressChangeStateGatesFromTheNextTag)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "ingress_change_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("IngressChange", node);
  bind_with_entries("IngressChange", {tick_entry("IngressChange")});
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("IngressChange", &ok_wake, nullptr, nullptr);

  rclcpp::ServiceBase::SharedPtr change_state;
  node->get_node_base_interface()->for_each_callback_group(
    [&](rclcpp::CallbackGroup::SharedPtr g) {
      g->find_service_ptrs_if([&](const rclcpp::ServiceBase::SharedPtr & s) {
        if (std::string(s->get_service_name()) == "/ingress_change_node/change_state") {
          change_state = s;
        }
        return false;
      });
    });
  ASSERT_TRUE(change_state);
  auto request = [&](uint8_t transition) {
      ros2_lf_binding::IngressRecord r;
      r.id = ing.next_record_id();
      r.service = change_state;
      r.service_suffix = "change_state";
      r.header = change_state->create_request_header();
      auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      req->transition.id = transition;
      r.request = req;
      r.t_rx = std::chrono::steady_clock::now();
      ing.on_request("IngressChange", std::move(r));
      ASSERT_TRUE(ing.move("IngressChange"));
      ing.drain_one("IngressChange");
    };

  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  request(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 1) << "a timer ran in the tag after an ingress deactivate";
  request(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  at_tag(300);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[2], 1) << "the timer of an ingress activate did not run a tag later";
  EXPECT_EQ(ing.counts("IngressChange").consumed, 2u);
  ing.close("IngressChange");
  ing.stop_and_join(false);
  ing.reset_for_test();
}

// Round 2, finding 2: one callback deactivates and activates the node, and
// another owner keeps the old timer alive. The production path observes only
// once, when the callback ends.
TEST_F(LifecycleGateBindingTest, SameCallbackReplacementWinsOverARetainedTimer)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "retained_timer_node", GenerationTimerNode::Mode::Retire);
  ros2_lf_register_node("RetainedTimer", node);
  bind_with_entries("RetainedTimer", {tick_entry("RetainedTimer")});
  at_tag(100);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
    node->activate();
  }
  ASSERT_EQ(node->retired.size(), 1u);
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 0) <<
    "dispatch ran the retired timer: the deactivate and activate in one callback went unseen";
  EXPECT_EQ(node->ticks[2], 1);
}

// Round 2, finding 2 follow-up: two timers of one period re-made on activate.
// The first sibling to reconcile must not make its new handle look old to the
// second, or the second takes the first's timer (seen in the rebind example).
TEST_F(LifecycleGateBindingTest, SiblingReconcileKeepsCreationOrder)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<TwoTimerNode>("two_timer_node");
  ros2_lf_register_node("TwoTimer", node);
  bind_with_entries(
    "TwoTimer", {{0, "TwoTimer", "first", "timer", "", 100000000, 0},
      {1, "TwoTimer", "second", "timer", "", 100000000, 0}});
  at_tag(100);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
    node->activate();
  }
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(node->first, 1) << "RID 0 (first) ran the wrong timer after a sibling reconcile";
  EXPECT_EQ(node->second, 1) <<
    "RID 1 (second) ran the first timer: the sibling's fresh handle counted as an old one";
}

// Round 2, finding 3: a deactivation X and a timer reaction Y of the same
// node at one tag, in both orders. A node with two callback groups runs them
// in two LF reactors with no order between them. Y's output must not depend
// on the order.
TEST_F(LifecycleGateBindingTest, SameTagTeardownDoesNotDependOnReactionOrder)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  std::map<bool, int> ticks_at_tag;
  std::map<bool, int> ticks_after;
  for (const bool x_first : {false, true}) {
    const std::string cls = x_first ? "TeardownXFirst" : "TeardownYFirst";
    auto node = std::make_shared<GenerationTimerNode>(
      x_first ? "teardown_x_first" : "teardown_y_first", GenerationTimerNode::Mode::Drop);
    ros2_lf_register_node(cls, node);
    bind_with_entries(cls, {tick_entry(cls)});
    at_tag(100);
    auto x = [&node]() {
        ros2_lf_binding::UserCallbackScope user;
        node->deactivate();
      };
    if (x_first) {
      x();
      ros2_lf_invoke_timer(nullptr, 0);
    } else {
      ros2_lf_invoke_timer(nullptr, 0);
      x();
    }
    ticks_at_tag[x_first] = node->ticks[1];
    at_tag(200);
    ros2_lf_invoke_timer(nullptr, 0);
    ticks_after[x_first] = node->ticks[1];
    ros2_lf_binding::lifecycle_gate_reset_for_test();
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
  }
  EXPECT_EQ(ticks_at_tag[true], ticks_at_tag[false]) <<
    "a timer reaction after a same-tag deactivation found its handle released: "
    "its output depends on the reaction order";
  EXPECT_EQ(ticks_at_tag[false], 1);
  EXPECT_EQ(ticks_after[true], ticks_at_tag[true]) << "the timer ran after the deactivation tag";
}

// Round 2, finding 4: names in a namespace. The analyzer records the name the
// source passes, the handle carries the resolved one.
TEST_F(LifecycleGateBindingTest, NamespacedServiceClientAndActionBindAndRebind)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<NamespacedNode>("namespaced_node");
  ros2_lf_register_node("Namespaced", node);
  bind_with_entries(
    "Namespaced", {
    {0, "Namespaced", "handle", "srv_server", "ns_svc", 0},
    {1, "Namespaced", "on_reply", "srv_client_cb", "ns_peer", 0},
    {2, "Namespaced", "goal", "action_goal_server", "alpha", 0}});
  auto & table = ros2_lf_binding::g_handle_table;
  EXPECT_EQ(table[0].service_server.lock(), node->service_) <<
    "a service 'ns_svc' on a node in namespace /ns stayed unbound at startup";
  EXPECT_EQ(table[1].service_client.lock(), node->client_) <<
    "a client 'ns_peer' on a node in namespace /ns stayed unbound at startup";
  EXPECT_EQ(table[2].action_waitable.lock(), node->action_) <<
    "an action server 'alpha' on a node in namespace /ns stayed unbound at startup";

  at_tag(100);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
    node->activate();
  }
  at_tag(200);
  for (int rid = 0; rid < 3; ++rid) {
    EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(rid));
  }
  EXPECT_EQ(table[0].service_server.lock(), node->service_) << "namespaced service not re-bound";
  EXPECT_EQ(table[1].service_client.lock(), node->client_) << "namespaced client not re-bound";
  EXPECT_EQ(table[2].action_waitable.lock(), node->action_) << "namespaced action not re-bound";
}

// Round 2, finding 5: an action whose name cannot be read is no match.
TEST_F(LifecycleGateBindingTest, UnreadableActionNameIsNoMatch)
{
  auto node = std::make_shared<rclcpp::Node>("nameless_action_node");
  NamelessWaitable waitable;
  EXPECT_FALSE(
    ros2_lf_binding::action_matches(waitable, "alpha", node->get_node_base_interface())) <<
    "a waitable whose action name could not be read was matched to the RID 'alpha'";
}

// --- Review round 3 -----------------------------------

// Round 3, finding 1: a
// transition that a thread LF does not own completes enters LF through the
// node's ingress wake, a physical action. It holds from the tag after the
// drain that applies it, whatever its wall-clock order against the boundary
// steps.
TEST_F(LifecycleGateBindingTest, ExternalTransitionHoldsFromItsPhysicalActionTag)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "physical_action_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("PhysicalAction", node);
  bind_with_entries("PhysicalAction", {tick_entry("PhysicalAction")});
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("PhysicalAction", &ok_wake, nullptr, nullptr);
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  ASSERT_EQ(node->ticks[1], 1);

  std::thread([&node]() {node->deactivate();}).join();
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 2) <<
    "a deactivate on a non-LF thread took effect at the first boundary after it, "
    "not at the tag of its physical action";
  ASSERT_TRUE(ing.move("PhysicalAction")) <<
    "the deactivate on a non-LF thread was not routed into LF as a physical action";
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{250}, 0u));
  ing.drain_one("PhysicalAction");
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 3) << "the deactivate held at the tag of the drain that applied it";
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{250}, 1u));
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 3) <<
    "the deactivate did not hold from the tag after its physical action";
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
}

namespace
{

/// Two 100 ms timers A and B made on activate, B first when `b_first`. A
/// user callback can re-make B alone while A stays.
class ReplaceOneTimerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ReplaceOneTimerNode(const std::string & name, bool b_first)
  : rclcpp_lifecycle::LifecycleNode(name), b_first_(b_first) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (b_first_) {
      make_b();
      make_a();
    } else {
      make_a();
      make_b();
    }
    return CallbackReturn::SUCCESS;
  }

  void replace_b()
  {
    b_.reset();
    make_b();
  }

  std::map<int, int> a_ticks;  ///< per generation of A
  std::map<int, int> b_ticks;  ///< per generation of B

private:
  void make_a()
  {
    const int g = ++a_gen_;
    a_ = this->create_wall_timer(std::chrono::milliseconds(100), [this, g]() {a_ticks[g]++;});
  }

  void make_b()
  {
    const int g = ++b_gen_;
    b_ = this->create_wall_timer(std::chrono::milliseconds(100), [this, g]() {b_ticks[g]++;});
  }

  bool b_first_;
  int a_gen_ = 0;
  int b_gen_ = 0;
  rclcpp::TimerBase::SharedPtr a_;
  rclcpp::TimerBase::SharedPtr b_;
};

/// A service made on activate. A user callback can re-make it while another
/// owner keeps the old one alive, or add an unrelated timer.
class RetainedServiceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RetainedServiceNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    make();
    return CallbackReturn::SUCCESS;
  }

  void replace_retaining_old()
  {
    retired.push_back(service_);
    make();
  }

  void add_unrelated_timer()
  {
    extra_ = this->create_wall_timer(std::chrono::seconds(10), []() {});
  }

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr service_;
  std::vector<rclcpp::ServiceBase::SharedPtr> retired;  ///< the other owner

private:
  void make()
  {
    service_ = this->create_service<lifecycle_msgs::srv::GetState>(
      "retained_svc",
      [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
      std::shared_ptr<lifecycle_msgs::srv::GetState::Response>) {});
  }

  rclcpp::TimerBase::SharedPtr extra_;
};

}  // namespace

// Round 3, finding 2: of two 100 ms timers, a callback re-makes only B. A's
// RID keeps A and B's RID takes the new B, whichever the node made first.
TEST_F(LifecycleGateBindingTest, ReplacingOneOfTwoSamePeriodTimersKeepsTheOther)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  for (const bool b_first : {false, true}) {
    SCOPED_TRACE(b_first ? "B made first" : "A made first");
    const std::string cls = b_first ? "ReplaceBFirst" : "ReplaceAFirst";
    auto node = std::make_shared<ReplaceOneTimerNode>(
      b_first ? "replace_b_first" : "replace_a_first", b_first);
    ros2_lf_register_node(cls, node);
    // RID 0 is A, RID 1 is B.
    bind_with_entries(
      cls, {{0, cls, "a", "timer", "", 100000000, b_first ? 1 : 0, ros2_lf_binding::kGroupDefault},
        {1, cls, "b", "timer", "", 100000000, b_first ? 0 : 1, ros2_lf_binding::kGroupDefault}});
    at_tag(100);
    ros2_lf_invoke_timer(nullptr, 0);
    ros2_lf_invoke_timer(nullptr, 1);
    ASSERT_EQ(node->a_ticks[1], 1);
    ASSERT_EQ(node->b_ticks[1], 1);
    {
      ros2_lf_binding::UserCallbackScope user;
      node->replace_b();
    }
    at_tag(200);
    ros2_lf_invoke_timer(nullptr, 0);
    EXPECT_EQ(node->a_ticks[1], 2) << "RID 0 (A) lost its timer when only B was replaced";
    EXPECT_EQ(node->b_ticks[2], 0) << "RID 0 (A) ran the new B: the RIDs swapped";
    ros2_lf_invoke_timer(nullptr, 1);
    EXPECT_EQ(node->b_ticks[2], 1) << "RID 1 (B) did not take the new B";
    EXPECT_EQ(node->a_ticks[1], 2) << "RID 1 (B) ran A: the RIDs swapped";
    ros2_lf_binding::lifecycle_gate_reset_for_test();
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
  }
}

// Round 3, finding 3: a service re-made while another owner keeps the old one
// alive. The RID takes the new service, and a later unrelated change of the
// node's handles must not move it back to the old one.
TEST_F(LifecycleGateBindingTest, RetainedOldServiceIsNotReboundOnAnUnrelatedEpoch)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<RetainedServiceNode>("retained_service_node");
  ros2_lf_register_node("RetainedService", node);
  bind_with_entries(
    "RetainedService", {{0, "RetainedService", "handle", "srv_server", "retained_svc", 0}});
  auto & table = ros2_lf_binding::g_handle_table;
  ASSERT_EQ(table[0].service_server.lock(), node->service_);
  at_tag(100);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->replace_retaining_old();
  }
  at_tag(200);
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  ASSERT_EQ(table[0].service_server.lock(), node->service_) << "RID 0 kept the retired service";
  {
    ros2_lf_binding::UserCallbackScope user;
    node->add_unrelated_timer();
  }
  at_tag(300);
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_EQ(table[0].gate_epoch, ros2_lf_binding::lifecycle_gate_view("RetainedService")->epoch);
  EXPECT_EQ(table[0].service_server.lock(), node->service_) <<
    "an unrelated epoch bump moved RID 0 back to the retired service";
}

// Round 3, finding 4: an active node hands its timer to another owner. The
// timer stays in the node's callback group, and a stock executor keeps firing
// it (measured on a stock executor), so
// its RID keeps it until the other owner lets it go.
TEST_F(LifecycleGateBindingTest, TimerHandedToAnotherOwnerKeepsFiringAsInStock)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "handed_over_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("HandedOver", node);
  bind_with_entries("HandedOver", {tick_entry("HandedOver")});
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->hand_over();
  }
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 2) << "a timer handed to another owner stopped, stock keeps it";
  node->retired.clear();
  at_tag(300);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 2) << "the timer ran after its last owner released it";
}

// Round 4, findings 1 and 2: the publication gate names the node of a
// transition by its rcl state machine. A remapped transition event topic and
// a second node of the same name must not hide or misroute an external
// transition.
TEST_F(LifecycleGateBindingTest, ExternalTransitionOfARemappedTransitionTopicIsRouted)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "remap_event_node", GenerationTimerNode::Mode::Keep,
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r", "/remap_event_node/transition_event:=/remapped_transition_event"}));
  ASSERT_EQ(node->count_publishers("/remapped_transition_event"), 1u) <<
    "the remap rule did not move the transition event topic";
  ros2_lf_register_node("RemapEvent", node);
  bind_with_entries("RemapEvent", {tick_entry("RemapEvent")});
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("RemapEvent", &ok_wake, nullptr, nullptr);
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  ASSERT_EQ(node->ticks[1], 1);

  std::thread([&node]() {node->deactivate();}).join();
  ASSERT_TRUE(ing.move("RemapEvent")) <<
    "the deactivate of a node with a remapped transition event topic was not routed into LF";
  at_tag(200);
  ing.drain_one("RemapEvent");
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{200}, 1u));
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 1) << "the routed deactivate did not hold from the next tag";

  std::thread([&node]() {node->activate();}).join();
  ASSERT_TRUE(ing.move("RemapEvent")) <<
    "the activate of a node with a remapped transition event topic was not routed into LF";
  at_tag(300);
  ing.drain_one("RemapEvent");
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{300}, 1u));
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[2], 1) << "the gate stayed inactive after the routed activate";
  ing.close("RemapEvent");
}

TEST_F(LifecycleGateBindingTest, ExternalTransitionOfOneOfTwoSameNamedNodesReachesThatNode)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto a = std::make_shared<GenerationTimerNode>("twin_node", GenerationTimerNode::Mode::Keep);
  auto b = std::make_shared<GenerationTimerNode>("twin_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("TwinA", a);
  ros2_lf_register_node("TwinB", b);
  ros2_lf_binding::clear_rid_entries_for_test();
  ros2_lf_binding::register_rid_entry({0, "TwinA", "tick", "timer", "", 100000000, 0});
  ros2_lf_binding::register_rid_entry({1, "TwinB", "tick", "timer", "", 100000000, 0});
  ros2_lf_binding::g_handle_table.clear();
  ros2_lf_binding::resize_handle_table(2);
  ros2_lf_bind_node(nullptr, "TwinA");
  ros2_lf_bind_node(nullptr, "TwinB");
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("TwinA", &ok_wake, nullptr, nullptr);
  ing.register_node("TwinB", &ok_wake, nullptr, nullptr);
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  ros2_lf_invoke_timer(nullptr, 1);
  ASSERT_EQ(a->ticks[1], 1);
  ASSERT_EQ(b->ticks[1], 1);

  std::thread([&b]() {b->deactivate();}).join();
  EXPECT_FALSE(ing.move("TwinA")) << "the deactivate of TwinB was routed to TwinA";
  ASSERT_TRUE(ing.move("TwinB")) << "the deactivate of TwinB was not routed to TwinB";
  at_tag(200);
  ing.drain_one("TwinB");
  ros2_lf_binding::set_lf_tag_for_test(std::make_pair(int64_t{200}, 1u));
  ros2_lf_invoke_timer(nullptr, 0);
  ros2_lf_invoke_timer(nullptr, 1);
  EXPECT_EQ(a->ticks[1], 2) << "TwinA stopped on its twin's deactivate";
  EXPECT_EQ(b->ticks[1], 1) << "TwinB ran after its routed deactivate";
  ing.close("TwinA");
  ing.close("TwinB");
}

// Round 4, finding 3: one transition by label posts once, as one by id does.
// In the Jazzy librcl_lifecycle both trigger calls tail-jump to one local
// helper, so the label call never reaches the gated id call (checked by
// disassembling librcl_lifecycle).
TEST_F(LifecycleGateBindingTest, LabelTransitionPostsExactlyOnce)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "label_node", GenerationTimerNode::Mode::Keep);
  ros2_lf_register_node("LabelNode", node);
  bind_with_entries("LabelNode", {tick_entry("LabelNode")});
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("LabelNode", &ok_wake, nullptr, nullptr);

  std::thread([&node]() {node->deactivate();}).join();
  ASSERT_EQ(ing.counts("LabelNode").stored, 1u) << "one transition by id did not post once";
  auto * sm = static_cast<rcl_lifecycle_state_machine_t *>(
    const_cast<void *>(ros2_lf_binding::lifecycle_gate_state_machine("LabelNode")));
  ASSERT_NE(sm, nullptr);
  std::thread([sm]() {
      ASSERT_EQ(rcl_lifecycle_trigger_transition_by_label(sm, "activate", true), RCL_RET_OK);
      const auto * done = rcl_lifecycle_get_transition_by_id(
        sm->current_state, lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS);
      ASSERT_NE(done, nullptr);
      ASSERT_EQ(rcl_lifecycle_trigger_transition_by_label(sm, done->label, true), RCL_RET_OK);
    }).join();
  ASSERT_EQ(sm->current_state->id, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(ing.counts("LabelNode").stored, 2u) <<
    "one transition by label to a primary state did not post exactly once";
  ing.close("LabelNode");
}

// Round 4, finding 4: a node without the lifecycle communication interface
// has no change_state service, so the gate polls it. Its external deactivate
// posts nothing and holds from the first boundary after it.
TEST_F(LifecycleGateBindingTest, NodeWithoutCommunicationInterfaceFallsBackToPolling)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<GenerationTimerNode>(
    "no_com_node", GenerationTimerNode::Mode::Keep, rclcpp::NodeOptions(), false);
  ros2_lf_register_node("NoCom", node);
  bind_with_entries("NoCom", {tick_entry("NoCom")});
  auto & ing = ros2_lf_binding::IngressRole::instance();
  ing.reset_for_test();
  ing.register_node("NoCom", &ok_wake, nullptr, nullptr);
  at_tag(100);
  ros2_lf_invoke_timer(nullptr, 0);
  ASSERT_EQ(node->ticks[1], 1);

  std::thread([&node]() {node->deactivate();}).join();
  EXPECT_EQ(ing.counts("NoCom").stored, 0u) << "a polled node's transition was posted";
  at_tag(200);
  ros2_lf_invoke_timer(nullptr, 0);
  EXPECT_EQ(node->ticks[1], 1) << "the polled deactivate did not hold at the next boundary";
  ing.close("NoCom");
}

namespace
{

/// A service client and an action client made in the constructor and kept
/// across transitions.
class ClientCleanupNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ClientCleanupNode(const std::string & name, const std::string & action)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    client_ = this->create_client<lifecycle_msgs::srv::GetState>("cleanup_peer");
    action_ = rclcpp_action::create_client<Fibonacci>(this, action);
  }

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
  rclcpp_action::Client<Fibonacci>::SharedPtr action_;
};

/// The typed action client ops the factory emitter writes for Fibonacci.
/// `deactivations` counts deactivate calls on `raw`.
Ros2LfActionClientOps fibonacci_ops(
  std::shared_ptr<int> deactivations, rclcpp_action::Client<Fibonacci> * raw)
{
  using Impl = Fibonacci::Impl;
  Ros2LfActionClientOps ops;
  ops.goal_id = [](rclcpp::Waitable & w, const void * request, bool result, Ros2LfGoalId & id) {
      if (dynamic_cast<rclcpp_action::Client<Fibonacci> *>(&w) == nullptr) {
        return false;
      }
      id = result ?
        static_cast<const Impl::GetResultService::Request *>(request)->goal_id.uuid :
        static_cast<const Impl::SendGoalService::Request *>(request)->goal_id.uuid;
      return true;
    };
  ops.goal_accepted = [](rclcpp::Waitable & w, const void * response, bool & accepted) {
      if (dynamic_cast<rclcpp_action::Client<Fibonacci> *>(&w) == nullptr) {
        return false;
      }
      accepted = static_cast<const Impl::SendGoalService::Response *>(response)->accepted;
      return true;
    };
  ops.deactivate = [deactivations, raw](rclcpp::Waitable & w,
    const std::vector<Ros2LfGoalId> & ids) {
      auto * typed = dynamic_cast<rclcpp_action::Client<Fibonacci> *>(&w);
      if (typed == nullptr) {
        return false;
      }
      if (typed == raw) {
        ++*deactivations;
      }
      for (const auto & id : ids) {
        typed->stop_callbacks(id);
      }
      typed->async_cancel_all_goals();
      return true;
    };
  return ops;
}

/// A Fibonacci server spun on its own thread. Goals are accepted and held
/// until cancelled; a cancel is accepted.
class HeldGoalServer
{
public:
  explicit HeldGoalServer(const std::string & action)
  : node_(std::make_shared<rclcpp::Node>("held_goal_server_" + std::to_string(getpid())))
  {
    using ServerGoal = rclcpp_action::ServerGoalHandle<Fibonacci>;
    server_ = rclcpp_action::create_server<Fibonacci>(
      node_, action,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<ServerGoal>) {
        ++cancels;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ServerGoal> h) {
        std::lock_guard<std::mutex> lock(mu_);
        held_.push_back(h);
      });
    executor_.add_node(node_);
    spinner_ = std::thread([this]() {executor_.spin();});
  }

  ~HeldGoalServer()
  {
    executor_.cancel();
    spinner_.join();
  }

  size_t held()
  {
    std::lock_guard<std::mutex> lock(mu_);
    return held_.size();
  }

  /// Wait until goal `i` is canceling, then finish it as canceled.
  bool finish_canceled(size_t i)
  {
    for (int n = 0; n < 500; ++n) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        if (i < held_.size() && held_[i]->is_canceling()) {
          held_[i]->canceled(std::make_shared<Fibonacci::Result>());
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::atomic<int> cancels{0};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::mutex mu_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>> held_;
};

/// Sets the counted take's bounded wait for one test.
class TakeTimeoutForTest
{
public:
  TakeTimeoutForTest()
  {
    const char * previous = std::getenv("ROS2_LF_TAKE_TIMEOUT_MS");
    had_ = previous != nullptr;
    previous_ = had_ ? previous : "";
    setenv("ROS2_LF_TAKE_TIMEOUT_MS", "5000", 1);
  }
  ~TakeTimeoutForTest()
  {
    if (had_) {
      setenv("ROS2_LF_TAKE_TIMEOUT_MS", previous_.c_str(), 1);
    } else {
      unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    }
  }

private:
  bool had_ = false;
  std::string previous_;
};

/// A LifecyclePublisher and a 100 ms timer, both made on configure and kept.
/// Each tick records whether the publisher would publish. The base hooks
/// activate and deactivate the publisher.
class PublishingTimerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PublishingTimerNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("tag_held_pub", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {published.push_back(publisher_->is_activated());});
    return CallbackReturn::SUCCESS;
  }

  std::vector<bool> published;
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

// Deactivate prunes the pending requests of the node's service
// clients and cancels the goals of its action clients, once, at the first
// boundary after the deactivation tag.
//
// The goal sent while active has its callbacks stopped, and the
// cancel and result responses that arrive while the node is inactive are
// executed with no user callback, so the client releases the goal handle.
TEST_F(LifecycleGateBindingTest, DeactivatePrunesRequestsAndCancelsGoalsOnce)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  TakeTimeoutForTest timeout;
  const std::string action = "cleanup_fib_" + std::to_string(getpid());
  auto node = std::make_shared<ClientCleanupNode>("client_cleanup_node", action);
  ros2_lf_register_node("ClientCleanup", node);
  bind_with_entries(
    "ClientCleanup", {
    {0, "ClientCleanup", "on_reply", "srv_client_cb", "cleanup_peer", 0},
    {1, "ClientCleanup", "on_goal", "action_goal_cb", action, 0},
    {2, "ClientCleanup", "on_result", "action_result_cb", action, 0},
    {3, "ClientCleanup", "on_cancel", "action_cancel_cb", action, 0}});
  ros2_lf_register_client_cleanup(
    [](rclcpp::ClientBase & c) -> int64_t {
      auto * typed = dynamic_cast<rclcpp::Client<lifecycle_msgs::srv::GetState> *>(&c);
      return typed == nullptr ? -1 : static_cast<int64_t>(typed->prune_pending_requests());
    });
  auto cancels = std::make_shared<int>(0);
  ros2_lf_register_action_client_ops(fibonacci_ops(cancels, node->action_.get()));
  HeldGoalServer server(action);
  ASSERT_TRUE(node->action_->wait_for_action_server(std::chrono::seconds(10)));

  at_tag(100);
  ASSERT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  int user_results = 0;
  std::weak_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>> goal;
  {
    rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
    options.result_callback = [&user_results](const auto &) {++user_results;};
    auto accepted = node->action_->async_send_goal(Fibonacci::Goal{}, options);
    ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
    ros2_lf_invoke_action_client_cb(nullptr, 1, action.c_str(), 1);
    ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_RESULT_REQ);
    ASSERT_EQ(accepted.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    goal = accepted.get();
    ASSERT_NE(goal.lock(), nullptr) << "the server did not accept the goal";
  }
  auto pending = node->client_->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
    [](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture) {});
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
  }
  EXPECT_EQ(pending.future.wait_for(std::chrono::seconds(0)), std::future_status::timeout) <<
    "the request was pruned at the deactivation tag, where the node still counts as active";
  EXPECT_EQ(*cancels, 0);

  at_tag(200);
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
  ASSERT_EQ(pending.future.wait_for(std::chrono::seconds(0)), std::future_status::ready) <<
    "deactivate left the request sent while active pending in the client";
  EXPECT_THROW(pending.future.get(), std::future_error);
  EXPECT_EQ(*cancels, 1) << "deactivate did not cancel the goals of the action client";
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_CANCEL_REQ), 1);

  // The cancel response and the canceled goal's result arrive while inactive.
  ASSERT_TRUE(server.finish_canceled(0)) << "the server never saw the cancel";
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_RESULT_RESP);
  ros2_lf_invoke_action_client_cb(nullptr, 3, action.c_str(), 1);
  ros2_lf_invoke_action_client_cb(nullptr, 2, action.c_str(), 1);
  EXPECT_EQ(goal.lock(), nullptr) <<
    "the client still holds the goal handle of a goal cancelled at deactivate";
  EXPECT_EQ(user_results, 0) << "a result callback ran for a goal stopped at deactivate";

  at_tag(300);
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_EQ(*cancels, 1) << "an inactive node was cleaned up again at a later tag";

  {
    ros2_lf_binding::UserCallbackScope user;
    node->activate();
  }
  at_tag(400);
  EXPECT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
  }
  at_tag(500);
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
  EXPECT_EQ(*cancels, 2) << "a second deactivate was not cleaned up";
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_CANCEL_REQ);
}

// A goal sent while active is accepted by the server, but its
// goal response arrives after deactivate. No user callback runs, the client
// erases its pending entry, and the binding cancels the goal once.
TEST_F(LifecycleGateBindingTest, GoalResponseWhileInactiveIsCancelledOnce)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  TakeTimeoutForTest timeout;
  const std::string action = "late_goal_fib_" + std::to_string(getpid());
  auto node = std::make_shared<ClientCleanupNode>("late_goal_node", action);
  ros2_lf_register_node("LateGoal", node);
  bind_with_entries(
    "LateGoal", {
    {0, "LateGoal", "on_goal", "action_goal_cb", action, 0},
    {1, "LateGoal", "on_cancel", "action_cancel_cb", action, 0}});
  ros2_lf_register_action_client_ops(
    fibonacci_ops(std::make_shared<int>(0), node->action_.get()));
  HeldGoalServer server(action);
  ASSERT_TRUE(node->action_->wait_for_action_server(std::chrono::seconds(10)));

  at_tag(100);
  ASSERT_TRUE(ros2_lf_binding::lifecycle_admit(0));
  int user_goal_responses = 0;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.goal_response_callback = [&user_goal_responses](const auto &) {
      ++user_goal_responses;
    };
  options.result_callback = [](const auto &) {};
  auto accepted = node->action_->async_send_goal(Fibonacci::Goal{}, options);
  ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_GOAL_REQ);
  for (int n = 0; n < 500 && server.held() == 0; ++n) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(server.held(), 1u) << "the server did not accept the goal";
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
  }
  at_tag(200);
  EXPECT_FALSE(ros2_lf_binding::lifecycle_admit(0));
  ASSERT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_CANCEL_REQ), 1) <<
    "deactivate did not cancel all goals";

  ros2_lf_invoke_action_client_cb(nullptr, 0, action.c_str(), 1);
  EXPECT_EQ(user_goal_responses, 0) << "a goal-response callback ran on an inactive node";
  ASSERT_EQ(accepted.wait_for(std::chrono::seconds(0)), std::future_status::ready) <<
    "the client still holds the pending entry of a goal response taken while inactive";
  EXPECT_THROW(accepted.get(), std::future_error) << "rclcpp's goal callback ran";
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_CANCEL_REQ), 1) <<
    "the goal accepted after deactivate was not cancelled exactly once";

  // Both cancel responses are the binding's own; nothing more is sent.
  ros2_lf_invoke_action_client_cb(nullptr, 1, action.c_str(), 2);
  EXPECT_EQ(ros2_lf_action_sent(action.c_str(), RCLLF_ACTION_CANCEL_REQ), 0);
  EXPECT_EQ(user_goal_responses, 0);
}

// A sibling reaction deactivates the node at tag T, and the node's
// timer reaction runs at T too (the gate holds the old state for the tag). The
// LifecyclePublisher must publish at T in both physical orders, and not after T.
TEST_F(LifecycleGateBindingTest, SameTagPublisherFlagDoesNotDependOnReactionOrder)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  std::map<bool, std::vector<bool>> published;
  for (const bool x_first : {false, true}) {
    const std::string cls = x_first ? "PubXFirst" : "PubYFirst";
    auto node = std::make_shared<PublishingTimerNode>(x_first ? "pub_x_first" : "pub_y_first");
    ros2_lf_register_node(cls, node);
    bind_with_entries(cls, {tick_entry(cls)});
    ASSERT_TRUE(node->publisher_->is_activated());
    at_tag(100);
    auto x = [&node]() {
        ros2_lf_binding::UserCallbackScope user;
        node->deactivate();
      };
    if (x_first) {
      x();
      ros2_lf_invoke_timer(nullptr, 0);
    } else {
      ros2_lf_invoke_timer(nullptr, 0);
      x();
    }
    at_tag(200);
    ros2_lf_invoke_timer(nullptr, 0);
    {
      ros2_lf_binding::UserCallbackScope user;
      EXPECT_FALSE(node->publisher_->is_activated()) << "the deactivation did not hold after T";
    }
    published[x_first] = node->published;
    ros2_lf_binding::lifecycle_gate_reset_for_test();
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
  }
  EXPECT_EQ(published[true], published[false]) <<
    "a timer reaction after a same-tag deactivation by a sibling found its LifecyclePublisher "
    "deactivated: its published output depends on the reaction order";
  EXPECT_EQ(published[false], std::vector<bool>{true});
}

// The callback that deactivates the node sees its own change at
// once, as in stock rclcpp, and so does a re-activation by another callback.
TEST_F(LifecycleGateBindingTest, TransitioningCallbackSeesItsOwnPublisherFlag)
{
  ros2_lf_binding::ScopedThreadRoleForTest lf(ros2_lf_binding::ThreadRole::LF_WORKER);
  auto node = std::make_shared<PublishingTimerNode>("pub_own_change");
  ros2_lf_register_node("PubOwnChange", node);
  bind_with_entries("PubOwnChange", {tick_entry("PubOwnChange")});
  at_tag(100);
  {
    ros2_lf_binding::UserCallbackScope user;
    node->deactivate();
    EXPECT_FALSE(node->publisher_->is_activated()) <<
      "the callback that deactivated the node still publishes";
  }
  {
    ros2_lf_binding::UserCallbackScope user;
    EXPECT_TRUE(node->publisher_->is_activated()) << "a sibling at T saw the change of T";
    node->activate();
    EXPECT_TRUE(node->publisher_->is_activated());
  }
  at_tag(200);
  ros2_lf_binding::UserCallbackScope user;
  EXPECT_TRUE(node->publisher_->is_activated());
}

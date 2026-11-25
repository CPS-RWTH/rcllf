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

/// @file test_external_input.cpp
/// @brief External topic input: a subscription on a topic with no
///        in-system publisher, declared in `external_topics`, is taken by the
///        ingress thread, scheduled once per message and dispatched once per
///        message.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "external_input.hpp"  // NOLINT(build/include_subdir)
#include "geometry_msgs/msg/point_stamped.hpp"
#include "handle_table.hpp"  // NOLINT(build/include_subdir)
#include "ingress.hpp"  // NOLINT(build/include_subdir)
#include "plant_record.hpp"  // NOLINT(build/include_subdir)
#include "plant_replay.hpp"  // NOLINT(build/include_subdir)
#include "policy_table.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp/serialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "thread_role.hpp"  // NOLINT(build/include_subdir)
#include "time_source.hpp"  // NOLINT(build/include_subdir)

extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = nullptr;
  return 0;
}

// The binding stamps arrivals with reactor-c's physical clock. The stamp
// anchor tests pin it so a message's lateness is decided against a
// known "now" instead of the wall clock.
std::atomic<int64_t> g_fake_now{-1};

extern "C" int64_t lf_time_physical(void)
{
  const int64_t pinned = g_fake_now.load();
  if (pinned >= 0) {
    return pinned;
  }
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

using namespace std::chrono_literals;
using ros2_lf_binding::g_handle_table;
using ros2_lf_binding::HandleKind;
using ros2_lf_binding::IngressRole;
using ros2_lf_binding::ScopedThreadRoleForTest;
using ros2_lf_binding::ThreadRole;
namespace ext = ros2_lf_binding::external_input;

namespace
{

/// Stands in for rcllf_external_wake (lf_schedule_copy): records each payload.
struct FakeAction
{
  std::mutex mu;
  std::vector<int64_t> records;
  std::atomic<int> ret{1};
};

int fake_wake(void * action, int64_t record)
{
  auto * a = static_cast<FakeAction *>(action);
  std::lock_guard<std::mutex> lk(a->mu);
  a->records.push_back(record);
  return a->ret.load();
}

class ExternalInputTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    IngressRole::instance().reset_for_test();
    ext::reset_for_test();
    ros2_lf_binding::lf_clock_init();
    node_ = std::make_shared<rclcpp::Node>("external_input_test_node");
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "ext177_in", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String & m) {received_.push_back(m.data);});
    pub_ = node_->create_publisher<std_msgs::msg::String>("ext177_in", rclcpp::QoS(10).reliable());
    g_handle_table.clear();
    g_handle_table.resize(1);
    auto & e = g_handle_table[0];
    e.kind = HandleKind::SUBSCRIPTION;
    e.subscription = sub_;
    e.topic = "ext177_in";
    e.resolved_topic = sub_->get_topic_name();
    e.node_class = "ExternalInputTestNode";
    e.callback_name = "on_input";
  }

  void TearDown() override
  {
    IngressRole::instance().reset_for_test();
    ext::reset_for_test();
    g_handle_table.clear();
  }

  /// Wait until the publisher is matched, so no message is lost to discovery.
  void wait_matched()
  {
    for (int i = 0; i < 200 && pub_->get_subscription_count() == 0; ++i) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(pub_->get_subscription_count(), 0u);
  }

  void publish(int n)
  {
    for (int i = 0; i < n; ++i) {
      std_msgs::msg::String m;
      m.data = "msg" + std::to_string(i);
      pub_->publish(m);
    }
  }

  /// Wait until `action` was scheduled `n` times by the ingress thread.
  size_t wait_scheduled(FakeAction & action, size_t n)
  {
    for (int i = 0; i < 300; ++i) {
      {
        std::lock_guard<std::mutex> lk(action.mu);
        if (action.records.size() >= n) {
          return action.records.size();
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    std::lock_guard<std::mutex> lk(action.mu);
    return action.records.size();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::vector<std::string> received_;
};

}  // namespace

// 7 messages from outside the system, 7 callbacks.
TEST_F(ExternalInputTest, SevenExternalMessagesRunSevenCallbacks)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  ASSERT_TRUE(IngressRole::instance().thread_running());
  wait_matched();
  publish(7);

  ASSERT_EQ(wait_scheduled(action, 7), 7u) <<
    "the ingress thread scheduled the external action for fewer than 7 messages";
  std::vector<int64_t> records;
  {
    std::lock_guard<std::mutex> lk(action.mu);
    records = action.records;
  }
  ASSERT_EQ(ext::pending(0), 7u);

  // One reaction per schedule call, each with its own payload.
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  for (int64_t r : records) {
    ros2_lf_invoke_external_sub(nullptr, 0, r);
  }
  EXPECT_EQ(received_.size(), 7u) <<
    "external subscription ran " << received_.size() << " of 7 callbacks ("
    "a subscription with no in-system publisher dropped every message)";
  const std::vector<std::string> expected{
    "msg0", "msg1", "msg2", "msg3", "msg4", "msg5", "msg6"};
  EXPECT_EQ(received_, expected);
  EXPECT_EQ(ext::pending(0), 0u);
}

// Bind before registration (the other startup order) admits too.
TEST_F(ExternalInputTest, BindBeforeRegisterAdmits)
{
  FakeAction action;
  ext::on_bound(0);
  EXPECT_FALSE(IngressRole::instance().thread_running()) <<
    "a bound subscription without registration must not be admitted";
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  wait_matched();
  publish(2);
  EXPECT_EQ(wait_scheduled(action, 2), 2u);
}

// Two arrivals that LF folds into one tag reach the reaction once with the
// later payload. Both callbacks still run, in arrival order.
TEST_F(ExternalInputTest, FoldedScheduleRunsEveryRecordUpToPayload)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  wait_matched();
  publish(3);
  ASSERT_EQ(wait_scheduled(action, 3), 3u);
  int64_t last = 0;
  {
    std::lock_guard<std::mutex> lk(action.mu);
    last = action.records.back();
  }
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ros2_lf_invoke_external_sub(nullptr, 0, last);
  const std::vector<std::string> expected{"msg0", "msg1", "msg2"};
  EXPECT_EQ(received_, expected);
  // A stale reaction (payload already drained) runs nothing.
  ros2_lf_invoke_external_sub(nullptr, 0, last);
  EXPECT_EQ(received_.size(), 3u);
}

// LF refusing the schedule (stopping) drops the record instead of leaving it
// to run in a later, unrelated reaction.
TEST_F(ExternalInputTest, RefusedScheduleDropsTheRecord)
{
  FakeAction action;
  action.ret.store(0);
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  wait_matched();
  publish(1);
  ASSERT_EQ(wait_scheduled(action, 1), 1u);
  for (int i = 0; i < 100 && ext::pending(0) != 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(ext::pending(0), 0u);
}

// The readiness wait skips a declared external topic. The generated startup
// passes the resolved name; rclcpp reports it with a leading slash.
TEST_F(ExternalInputTest, DeclaredTopicMatchesWithOrWithoutSlash)
{
  EXPECT_FALSE(ext::is_declared_topic("/chatter"));
  ros2_lf_runtime_declare_external_topic("chatter");
  EXPECT_TRUE(ext::is_declared_topic("/chatter"));
  EXPECT_TRUE(ext::is_declared_topic("chatter"));
  EXPECT_FALSE(ext::is_declared_topic("/other"));
}

// Review finding 3. The record id crossed into the int action payload, so
// after 2^31 arrivals the payload wrapped negative, fifo.front().id > record
// held forever and the subscription stopped running callbacks.
TEST_F(ExternalInputTest, RecordIdsCrossInt32MaxAndKeepDispatching)
{
  constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
  ext::set_last_record_id_for_test(kMax - 2);
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  wait_matched();
  publish(5);
  ASSERT_EQ(wait_scheduled(action, 5), 5u);
  std::vector<int64_t> records;
  {
    std::lock_guard<std::mutex> lk(action.mu);
    records = action.records;
  }
  const std::vector<int64_t> expected_ids{kMax - 1, kMax, kMax + 1, kMax + 2, kMax + 3};
  EXPECT_EQ(records, expected_ids) << "the payload must carry the 64-bit record id";

  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  for (int64_t r : records) {
    ros2_lf_invoke_external_sub(nullptr, 0, r);
  }
  EXPECT_EQ(received_.size(), 5u) <<
    "dispatch stopped at the int32 boundary, ran " << received_.size() << " of 5 callbacks";
  EXPECT_EQ(ext::pending(0), 0u);
}

// Review finding 1. ROS2_LF_TAKE_MODE=block does not apply to an external
// subscription. Its messages still dispatch on arrival, and the override WARN
// leaves it out by name.
TEST_F(ExternalInputTest, EnvBlockLeavesExternalSubscriptionOnArrival)
{
  using ros2_lf_binding::PolicyRow;
  using ros2_lf_binding::TakeMode;
  using ros2_lf_binding::TakePolicy;
  ros2_lf_runtime_declare_external_topic(sub_->get_topic_name());
  PolicyRow external{"ExternalInputTestNode", "ext177_in", TakePolicy{}};
  PolicyRow internal{"OtherNode", "/internal", TakePolicy{}};
  const TakePolicy block_defaults{TakeMode::Block, 100};
  ros2_lf_binding::policy_table_set_for_test(block_defaults, {external, internal});
  setenv("ROS2_LF_TAKE_MODE", "block", 1);

  const std::string msg = ros2_lf_binding::policy_table_take_mode_override_message();
  EXPECT_EQ(
    msg, "ROS2_LF_TAKE_MODE=block overrides take_mode for 1 of 1 configured subscription(s), "
    "not for the defaults, which already match, leaving out 1 external_topics "
    "subscription(s), which dispatch on arrival");

  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  wait_matched();
  publish(3);
  // Scheduled at arrival by the ingress thread, before any reaction ran.
  EXPECT_EQ(wait_scheduled(action, 3), 3u);
  EXPECT_EQ(ext::pending(0), 3u);
  std::vector<int64_t> records;
  {
    std::lock_guard<std::mutex> lk(action.mu);
    records = action.records;
  }
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  for (int64_t r : records) {
    ros2_lf_invoke_external_sub(nullptr, 0, r);
  }
  const std::vector<std::string> expected{"msg0", "msg1", "msg2"};
  EXPECT_EQ(received_, expected) <<
    "ROS2_LF_TAKE_MODE=block changed how the external subscription dispatches";

  unsetenv("ROS2_LF_TAKE_MODE");
  EXPECT_EQ(ros2_lf_binding::policy_table_take_mode_override_message(), "");
  ros2_lf_binding::policy_table_reset();
}

// Round-2 review. The callback_group_example run with ROS2_LF_TAKE_MODE=block
// logged "1 of 0 configured subscriptions (+ defaults)": the defaults entry was
// counted as changed but not in the total. Rows and defaults are now apart.
TEST_F(ExternalInputTest, OverrideWarnCountsRowsAndDefaultsApart)
{
  using ros2_lf_binding::PolicyRow;
  using ros2_lf_binding::TakeMode;
  using ros2_lf_binding::TakePolicy;
  ros2_lf_runtime_declare_external_topic("/ext177_in");
  // The example's shape: no internal row, one external row, skip defaults.
  PolicyRow external{"ExternalInputTestNode", "ext177_in", TakePolicy{TakeMode::Skip, 0}};
  ros2_lf_binding::policy_table_set_for_test(TakePolicy{TakeMode::Skip, 0}, {external});
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  unsetenv("ROS2_LF_MAXWAIT_MS");  // the alias alone
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "250", 1);
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_mode_override_message(),
    "ROS2_LF_TAKE_MODE=block overrides take_mode for 0 of 0 configured subscription(s) and "
    "for the defaults, which every subscription without its own entry uses, leaving out 1 "
    "external_topics subscription(s), which dispatch on arrival");
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_timeout_override_message(),
    "ROS2_LF_TAKE_TIMEOUT_MS=250 (alias of ROS2_LF_MAXWAIT_MS) overrides maxwait_ms for 0 "
    "of 0 configured subscription(s) and for the defaults, which every subscription without "
    "its own entry uses, leaving out 1 external_topics subscription(s), which dispatch on "
    "arrival. An "
    "admitted correspondence edge keeps its generated timeout, which this variable does not "
    "change");

  // Two internal rows, one already block: one of two changes, defaults too.
  PolicyRow a{"A", "/a", TakePolicy{TakeMode::Skip, 0}};
  PolicyRow b{"B", "/b", TakePolicy{TakeMode::Block, 100}};
  ros2_lf_binding::policy_table_set_for_test(TakePolicy{TakeMode::Skip, 0}, {a, b});
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_mode_override_message(),
    "ROS2_LF_TAKE_MODE=block overrides take_mode for 1 of 2 configured subscription(s) and "
    "for the defaults, which every subscription without its own entry uses");

  unsetenv("ROS2_LF_TAKE_MODE");
  unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
  ros2_lf_binding::policy_table_reset();
}

// A lifecycle node drops its subscription on cleanup and creates
// a new one on the next configure. Ingress must not keep the old one alive
// (or the late bind never sees it expire), and the new one is admitted again.
// A record taken from the old one is dropped, as stock destroys its queue with
// it, and never runs the new subscription's callback.
TEST_F(ExternalInputTest, NewSubscriptionIsAdmittedAndOldRecordsAreDropped)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_bound(0);
  wait_matched();
  publish(1);
  ASSERT_EQ(wait_scheduled(action, 1), 1u);

  std::weak_ptr<rclcpp::SubscriptionBase> old = sub_;
  sub_.reset();
  // The wait set holds it for at most one 100 ms wait.
  for (int i = 0; i < 200 && !old.expired(); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(old.expired()) <<
    "ingress kept the dropped subscription alive, so a late bind never sees it expire";
  for (int i = 0; i < 300 && pub_->get_subscription_count() > 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  auto sub2 = node_->create_subscription<std_msgs::msg::String>(
    "ext177_in", rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::String & m) {received_.push_back("new:" + m.data);});
  g_handle_table[0].subscription = sub2;
  ext::on_bound(0);
  wait_matched();
  publish(2);
  EXPECT_EQ(wait_scheduled(action, 3), 3u) << "the new subscription was not admitted";

  std::vector<int64_t> records;
  {
    std::lock_guard<std::mutex> lk(action.mu);
    records = action.records;
  }
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  for (int64_t r : records) {
    ros2_lf_invoke_external_sub(nullptr, 0, r);
  }
  const std::vector<std::string> expected{"new:msg0", "new:msg1"};
  EXPECT_EQ(received_, expected) <<
    "a record taken from the dropped subscription ran the new subscription's callback";
  EXPECT_EQ(ext::pending(0), 0u);
}

// Cleanup and configure back to back: the change_state poke can still see the
// old subscription alive (the ingress wait holds it for up to 100 ms), so the
// ingress thread pokes again when it retires the expired admission.
TEST_F(ExternalInputTest, RetiredAdmissionPokesWithoutAnotherChangeState)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::on_lifecycle_entry(0);
  ext::on_bound(0);
  wait_matched();

  auto held = std::static_pointer_cast<rclcpp::SubscriptionBase>(sub_);  // the wait's reference
  sub_.reset();  // cleanup
  auto sub2 = node_->create_subscription<std_msgs::msg::String>(  // configure
    "ext177_in", rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::String & m) {received_.push_back(m.data);});
  g_handle_table[0].subscription = sub2;
  EXPECT_EQ(ext::poke_lifecycle(), 0u) << "the change_state poke sees the old one alive";

  held.reset();
  EXPECT_GE(wait_scheduled(action, 1), 1u) <<
    "no poke after ingress retired the old admission, so the new subscription waits for "
    "the next change_state";
}

// Ingress takes at most one batch per wake, and nothing from a subscription
// whose node dropped it, even while the ingress thread still holds it.
TEST_F(ExternalInputTest, TakeExternalIsBoundedAndStopsOnceTheNodeDroppedIt)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  auto pub = node_->create_publisher<std_msgs::msg::String>(
    "ext177_take", rclcpp::QoS(50).reliable());
  // `node_ref` is the node's member, `sub` the ingress thread's lock.
  rclcpp::SubscriptionBase::SharedPtr node_ref =
    node_->create_subscription<std_msgs::msg::String>(
    "ext177_take", rclcpp::QoS(50).reliable(), [](const std_msgs::msg::String &) {});
  auto sub = node_ref;
  EXPECT_EQ(IngressRole::external_batch(*sub), 50u);
  for (int i = 0; i < 200 && pub->get_subscription_count() == 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  auto send = [&pub](int n) {
      for (int i = 0; i < n; ++i) {
        std_msgs::msg::String m;
        m.data = "t" + std::to_string(i);
        pub->publish(m);
      }
      std::this_thread::sleep_for(300ms);
    };
  send(8);
  EXPECT_EQ(IngressRole::take_external(0, sub, 3), 3u) << "one take drained past its batch";
  EXPECT_EQ(IngressRole::take_external(0, sub, 3), 3u);
  EXPECT_EQ(IngressRole::take_external(0, sub, 3), 2u);

  send(5);
  node_ref.reset();  // cleanup: only the ingress reference is left
  const size_t after = IngressRole::take_external(0, sub, 50);
  EXPECT_EQ(after, 0u) << "ingress took " << after <<
    " message(s) from a subscription the node had dropped";
  EXPECT_EQ(ext::pending(0), 8u);
}

// After change_state the binding schedules each external lifecycle RID that
// has no live admitted subscription, so its own reaction can bind late. The
// payload is the last record id, so it never hides a folded arrival.
TEST_F(ExternalInputTest, PokeSchedulesLifecycleRidsWithoutALiveSubscription)
{
  FakeAction action;
  ros2_lf_external_register(nullptr, 0, &fake_wake, &action);
  ext::set_last_record_id_for_test(41);
  EXPECT_EQ(ext::poke_lifecycle(), 0u) << "not a lifecycle entry";
  ext::on_lifecycle_entry(0);
  EXPECT_EQ(ext::poke_lifecycle(), 1u) << "lifecycle entry with nothing admitted";
  ext::on_bound(0);
  wait_matched();
  EXPECT_EQ(ext::poke_lifecycle(), 0u) << "its admitted subscription is live";
  std::lock_guard<std::mutex> lk(action.mu);
  EXPECT_EQ(action.records, std::vector<int64_t>{41});
}

// ---------------------------------------------------------------------------
// Plan 5.3 items 1 and 2: stamp-anchored external inputs.
// ---------------------------------------------------------------------------

namespace
{

constexpr int64_t kNs = 1000000000LL;
constexpr int64_t kStart = 1700000000LL * kNs;  // the fake scheduler's start tag
constexpr int64_t kDelay = 200000000LL;         // T = 200 ms
constexpr int64_t kReserve = 50000000LL;        // ingress_reserve_ns = 50 ms
constexpr int64_t kGrid = 10000000LL;           // grid_ns = 10 ms

/// A scheduler double with a real mutex, so the test can hold the "env lock"
/// while a message is being admitted and observe what the binding does at
/// the boundary. Every scheduler call that must run under the lock records a
/// violation if the calling thread does not hold it.
struct FakeScheduler
{
  std::mutex mu;                      // the env mutex
  std::thread::id holder;             // who holds mu (set under mu)
  std::mutex hs_mu;                   // handshake: "a thread is about to enter"
  std::condition_variable hs_cv;
  int entering = 0;
  int64_t current = kStart;           // current tag time
  int64_t stop = std::numeric_limits<int64_t>::max();
  std::set<int64_t> queued;           // tag times with an event in the queue
  std::vector<std::pair<int64_t, int64_t>> inserts;  // (tag time, record)
  std::atomic<int> enter_calls{0};
  std::atomic<int> after_stop_calls{0};
  std::atomic<int> insert_calls{0};
  std::atomic<int> violations{0};
  int dummy_env = 0;
  bool enter_fails = false;       // lf_critical_section_enter returns an error
  bool dispatch_in_insert = false;  // a worker runs the event before ingress resumes
  const char * missing = nullptr;  // a scheduler symbol the executable lacks
  bool schedule_refuses = false;  // _lf_schedule_at_tag refuses a future tag
  bool keepalive = true;          // keepalive_specified
  uint32_t stop_microstep = 0;    // with `stop`, env->stop_tag

  void check_held()
  {
    if (holder != std::this_thread::get_id()) {
      ++violations;
    }
  }
};

FakeScheduler * g_fake = nullptr;

void * fake_env() {return &g_fake->dummy_env;}
int64_t fake_start_ns() {return kStart;}
int fake_enter(void * env)
{
  (void)env;
  ++g_fake->enter_calls;
  if (g_fake->enter_fails) {
    return -1;
  }
  {
    std::lock_guard<std::mutex> lk(g_fake->hs_mu);
    ++g_fake->entering;
  }
  g_fake->hs_cv.notify_all();
  g_fake->mu.lock();
  g_fake->holder = std::this_thread::get_id();
  return 0;
}
int fake_exit(void * env)
{
  (void)env;
  g_fake->holder = std::thread::id();
  g_fake->mu.unlock();
  return 0;
}
bool fake_after_stop(void *, int64_t time)
{
  ++g_fake->after_stop_calls;
  g_fake->check_held();
  return time > g_fake->stop;
}
void fake_current_tag(void *, int64_t * time, uint32_t * microstep)
{
  g_fake->check_held();
  *time = g_fake->current;
  *microstep = 0;
}
int fake_insert(void *, void *, int64_t time, int64_t record)
{
  ++g_fake->insert_calls;
  g_fake->check_held();
  if (time <= g_fake->current) {
    return ext::INSERT_PAST;
  }
  if (g_fake->schedule_refuses) {
    return ext::INSERT_REFUSED;
  }
  if (g_fake->queued.count(time) > 0) {
    g_fake->queued.erase(time);  // the plan's rule: the queued event leaves too
    return ext::INSERT_COLLIDED;
  }
  g_fake->queued.insert(time);
  g_fake->inserts.emplace_back(time, record);
  if (g_fake->dispatch_in_insert) {
    // The event is in the queue; the worker that runs it does not wait for
    // the ingress thread to finish its bookkeeping.
    ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
    ros2_lf_invoke_external_sub(nullptr, 0, record);
  }
  return ext::INSERT_OK;
}
std::string fake_symbol_problem()
{
  return g_fake->missing == nullptr ? std::string() :
         std::string("the executable does not export reactor-c's ") + g_fake->missing;
}

void fake_set_stop_tag(void *, int64_t time, uint32_t microstep)
{
  g_fake->check_held();
  // lf_set_stop_tag only lowers the stop tag (reactor_common.c).
  if (time < g_fake->stop || (time == g_fake->stop && microstep < g_fake->stop_microstep)) {
    g_fake->stop = time;
    g_fake->stop_microstep = microstep;
  }
}
void fake_stop_tag(void *, int64_t * time, uint32_t * microstep)
{
  g_fake->check_held();
  *time = g_fake->stop;
  *microstep = g_fake->stop_microstep;
}
bool fake_keepalive() {return g_fake->keepalive;}

const ext::SchedulerOps kFakeOps{
  &fake_env, &fake_start_ns, &fake_enter, &fake_exit, &fake_after_stop, &fake_current_tag,
  &fake_insert, &fake_symbol_problem, &fake_set_stop_tag, &fake_stop_tag, &fake_keepalive};

/// Holds the fake env mutex on the test thread; what the ingress path would
/// wait for while the runtime advances the tag.
class EnvLock
{
public:
  EnvLock()
  {
    g_fake->mu.lock();
    g_fake->holder = std::this_thread::get_id();
  }
  ~EnvLock()
  {
    g_fake->holder = std::thread::id();
    g_fake->mu.unlock();
  }
};

/// The action the generated code registers: a plain physical action.
struct PlainPhysicalAction
{
  trigger_t trigger{};
  lf_action_base_t base{};
  PlainPhysicalAction()
  {
    trigger.offset = 0;
    trigger.period = -1;
    trigger.is_physical = true;
    base.trigger = &trigger;
  }
};

int never_wake(void *, int64_t)
{
  ADD_FAILURE() << "a stamp-anchored RID must not use the arrival-anchored wake";
  return 1;
}

class StampAnchorTest : public ExternalInputTest
{
protected:
  void SetUp() override
  {
    ExternalInputTest::SetUp();
    fake_ = std::make_unique<FakeScheduler>();
    g_fake = fake_.get();
    g_fake_now.store(-1);
    ros2_lf_binding::set_lf_fast_for_test(false);
    ext::set_scheduler_ops_for_test(&kFakeOps);
    psub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "ext776_in", rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PointStamped & m) {
        points_.push_back(static_cast<int64_t>(m.point.x));
      });
    auto & e = g_handle_table[0];
    e.subscription = psub_;
    e.topic = "ext776_in";
    e.resolved_topic = psub_->get_topic_name();
    ros2_lf_runtime_declare_external_stamp_anchor(
      psub_->get_topic_name(), kDelay, kReserve, kGrid);
  }

  void TearDown() override
  {
    ext::set_scheduler_ops_for_test(nullptr);
    ros2_lf_binding::set_lf_fast_for_test(std::nullopt);
    g_fake_now.store(-1);
    psub_.reset();
    ExternalInputTest::TearDown();
    g_fake = nullptr;
  }

  void attach(PlainPhysicalAction & action)
  {
    ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
    ext::on_bound(0);
    ASSERT_NE(ext::stamp_anchor_of("ext776_in"), nullptr);
  }

  /// Feed one message with the given stamp straight into the binding's
  /// sample path (what the ingress thread calls), at the pinned "now".
  void feed(int64_t stamp_ns, double x)
  {
    auto msg = std::make_shared<geometry_msgs::msg::PointStamped>();
    msg->header.stamp.sec = static_cast<int32_t>(stamp_ns / kNs);
    msg->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % kNs);
    msg->point.x = x;
    rmw_message_info_t mi{};
    mi.publication_sequence_number = static_cast<uint64_t>(x);
    rclcpp::MessageInfo info(mi);
    ext::on_sample(0, psub_, std::static_pointer_cast<void>(msg), info);
  }

  std::unique_ptr<FakeScheduler> fake_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr psub_;
  std::vector<int64_t> points_;
};

}  // namespace

// Item 1: late is decided on arrival against T alone. A message that arrives
// exactly at stamp + T is on time; one nanosecond later it is late and never
// reaches the scheduler.
TEST_F(StampAnchorTest, LateIsDecidedAgainstTOnArrivalAndTheBoundaryIsOnTime)
{
  PlainPhysicalAction action;
  attach(action);
  const int64_t stamp = kStart + 1 * kNs;
  g_fake_now.store(stamp + kDelay);  // arrival == stamp + T
  feed(stamp, 1);
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.admitted, 1u) << "a message arriving exactly at stamp + T was not admitted";
  EXPECT_EQ(c.late, 0u) << "a message arriving exactly at stamp + T was reported late";

  // One nanosecond past T for a stamp one grid cell earlier (a different
  // cell, so the outcome cannot be a collision).
  g_fake_now.store(stamp - kGrid + kDelay + 1);
  feed(stamp - kGrid, 2);
  c = ext::counters(0);
  EXPECT_EQ(c.late, 1u) << "a message arriving after stamp + T was not reported late";
  EXPECT_EQ(c.admitted, 1u);
  EXPECT_EQ(fake_->insert_calls.load(), 1) << "a late message touched the scheduler";
  EXPECT_EQ(fake_->enter_calls.load(), 1) << "a late message entered the critical section";
}

// Item 1: the tag of an admitted message, derived by hand:
//   tag = start + ceil(stamp + T + reserve - start, grid), microstep 0.
TEST_F(StampAnchorTest, TagIsStampPlusTPlusReserveRoundedUpToTheGrid)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);  // every message below arrives on time
  // stamp_rel          -> rel = stamp_rel + 250 ms -> tag_rel (ceil to 10 ms)
  // 1 000 ms           -> 1 250 ms                 -> 1 250 ms (on the grid)
  // 1 001 ms           -> 1 251 ms                 -> 1 260 ms
  // 1 009 999 999 ns   -> 1 259 999 999 ns         -> 1 260 ms (collides with the above)
  // 2 000 ms + 1 ns    -> 2 250 ms + 1 ns          -> 2 260 ms
  const std::vector<std::pair<int64_t, int64_t>> cases{
    {1000 * kGrid / 10, 1250 * kGrid / 10},
    {1001 * kGrid / 10, 1260 * kGrid / 10},
    {2000 * kGrid / 10 + 1, 2260 * kGrid / 10},
  };
  for (size_t i = 0; i < cases.size(); ++i) {
    feed(kStart + cases[i].first, static_cast<double>(i + 1));
  }
  ASSERT_EQ(fake_->inserts.size(), cases.size());
  for (size_t i = 0; i < cases.size(); ++i) {
    EXPECT_EQ(fake_->inserts[i].first - kStart, cases[i].second) <<
      "message " << i << " with stamp_rel " << cases[i].first << " ns landed on tag_rel " <<
      fake_->inserts[i].first - kStart << " ns, expected " << cases[i].second << " ns";
  }
  // The runtime dispatches the admitted record by its id, once, and an
  // unknown id runs nothing.
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, fake_->inserts[1].second), 1);
  EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, fake_->inserts[1].second), 0);
  EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, 424242), 0);
  ASSERT_EQ(points_.size(), 1u);
  EXPECT_EQ(points_[0], 2);
  EXPECT_EQ(ext::pending(0), 2u);
}

// Item 2: admission is decided under the scheduler's lock, against the tag
// the scheduler holds at that moment. The test holds the env lock, lets a
// message block on it, moves the current tag, releases, and checks the
// outcome; no sleep decides anything.
TEST_F(StampAnchorTest, AdmissionIsDecidedAtomicallyAgainstTheSchedulerAtTheBoundary)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  const int64_t stamp = kStart + 1 * kNs;
  const int64_t tag = kStart + 1250 * kGrid / 10;  // by hand, as above

  auto run = [&](int64_t current_while_blocked, double x) {
      std::thread worker;
      {
        EnvLock held;
        const int before = fake_->entering;
        worker = std::thread([&] {feed(stamp, x);});
        std::unique_lock<std::mutex> lk(fake_->hs_mu);
        fake_->hs_cv.wait(lk, [&] {return fake_->entering > before;});
        // The ingress path is blocked on the env lock now. The scheduler
        // advances to the boundary before letting it in.
        fake_->current = current_while_blocked;
      }
      worker.join();
    };

  // Case A: the scheduler reached exactly the message's tag while the
  // ingress path waited. The tag is not in the future any more: overrun.
  run(tag, 1);
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.overrun, 1u) <<
    "a message whose tag equals the current tag at insert time was not refused as overrun";
  EXPECT_EQ(c.admitted, 0u);
  EXPECT_TRUE(fake_->queued.empty());

  // Case B: the scheduler stopped one nanosecond before the tag: admitted.
  run(tag - 1, 2);
  c = ext::counters(0);
  EXPECT_EQ(c.admitted, 1u) <<
    "a message whose tag is one nanosecond after the current tag was refused";
  EXPECT_EQ(c.overrun, 1u);
  ASSERT_EQ(fake_->inserts.size(), 1u);
  EXPECT_EQ(fake_->inserts[0].first, tag);

  EXPECT_EQ(fake_->violations.load(), 0) <<
    "the binding read the current tag or inserted outside the scheduler's critical section "
    "(admission is not atomic against the scheduler)";
  EXPECT_EQ(fake_->enter_calls.load(), 2);
}

// With RCLLF_PLANT_RECORD the topic's file carries one entry per
// record when its disposition is first known, and the earlier record of a
// collided cell gets a correction entry, so the last entry per id gives the
// final disposition the counters report. The bytes deserialize to the message.
TEST_F(StampAnchorTest, RecordingCarriesDispositionsCorrectionAndBytes)
{
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "rcllf_ext_record_789";
  fs::remove_all(dir);
  ros2_lf_binding::plant_record::set_dir_for_test(dir.string());
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  const int64_t stamp = kStart + 1 * kNs;
  feed(stamp + 1 * kGrid / 10, 1);   // admitted, then collided by 2
  feed(stamp + 6 * kGrid / 10, 2);   // collision
  feed(stamp + 5 * kGrid, 3);        // admitted, a cell of its own
  feed(kStart - 1 * kNs, 4);         // late: arrival - stamp = 1 s > T
  const ext::Counters c = ext::counters(0);
  ros2_lf_binding::plant_record::close();
  ros2_lf_binding::plant_record::set_dir_for_test(std::string());

  std::ifstream in(dir / ("ext" + std::string(psub_->get_topic_name()).replace(0, 1, ".") +
    ".jsonl"));
  std::vector<std::string> lines;
  for (std::string l; std::getline(in, l); ) {
    lines.push_back(l);
  }
  ASSERT_GE(lines.size(), 5u) << "header and 4 records";
  EXPECT_NE(lines[0].find("\"type\":\"geometry_msgs/msg/PointStamped\""), std::string::npos)
    << lines[0];
  EXPECT_NE(lines[0].find("\"lf_time_start\":" + std::to_string(kStart)), std::string::npos);
  const std::regex re_id("\"id\":(\\d+)");
  const std::regex re_disp("\"disposition\":\"([a-z_]+)\"");
  const std::regex re_cdr("\"cdr\":\"([A-Za-z0-9+/=]*)\"");
  std::map<int64_t, std::string> last;
  std::map<int64_t, std::string> cdr;
  std::vector<std::string> evs;
  for (size_t i = 1; i < lines.size(); ++i) {
    std::smatch m;
    ASSERT_TRUE(std::regex_search(lines[i], m, re_id)) << lines[i];
    const int64_t id = std::stoll(m[1]);
    ASSERT_TRUE(std::regex_search(lines[i], m, re_disp)) << lines[i];
    last[id] = m[1];
    evs.push_back(lines[i].substr(0, lines[i].find(',')));
    if (std::regex_search(lines[i], m, re_cdr)) {
      cdr[id] = m[1];
    }
  }
  std::map<std::string, uint64_t> finals;
  for (const auto & kv : last) {
    ++finals[kv.second];
  }
  EXPECT_EQ(finals["admitted"], c.admitted) <<
    "a reader taking the last entry per record id counts a collided record as admitted";
  EXPECT_EQ(finals["collision"], c.collision);
  EXPECT_EQ(finals["late"], c.late);
  EXPECT_EQ(c.admitted, 1u);
  EXPECT_EQ(c.collision, 2u);
  EXPECT_EQ(c.late, 1u);
  EXPECT_EQ(lines.size(), 6u) << "header, 4 records, 1 correction";
  // Record 1 is recorded admitted, then corrected when record 2 lands in its
  // cell (before record 2's own entry): exactly one correction, after it.
  ASSERT_EQ(std::count(evs.begin(), evs.end(), "{\"ev\":\"correction\""), 1)
    << "one collision correction";
  EXPECT_EQ(evs[1], "{\"ev\":\"correction\"") << "the correction follows record 1's entry";
  EXPECT_NE(lines[2].find("\"id\":1,"), std::string::npos) << lines[2];

  // Each record's bytes are the rmw serialization of the message fed.
  auto unb64 = [](const std::string & s) {
      std::vector<uint8_t> out;
      uint32_t v = 0;
      int bits = 0;
      for (char ch : s) {
        const char * abc = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        const char * p = std::strchr(abc, ch);
        if (ch == '=' || p == nullptr) {
          break;
        }
        v = (v << 6) | static_cast<uint32_t>(p - abc);
        bits += 6;
        if (bits >= 8) {
          bits -= 8;
          out.push_back(static_cast<uint8_t>((v >> bits) & 0xff));
        }
      }
      return out;
    };
  std::set<int64_t> xs;
  rclcpp::Serialization<geometry_msgs::msg::PointStamped> ser;
  for (const auto & kv : cdr) {
    const std::vector<uint8_t> bytes = unb64(kv.second);
    rclcpp::SerializedMessage sm(bytes.size());
    std::memcpy(sm.get_rcl_serialized_message().buffer, bytes.data(), bytes.size());
    sm.get_rcl_serialized_message().buffer_length = bytes.size();
    geometry_msgs::msg::PointStamped msg;
    ser.deserialize_message(&sm, &msg);
    xs.insert(static_cast<int64_t>(msg.point.x));
  }
  EXPECT_EQ(xs, (std::set<int64_t>{1, 2, 3, 4}));
}

// Item 2: two on-time messages in one grid cell are both refused. The queued
// event of the first leaves the queue, the second is not inserted, and a
// third one in the same cell is refused without touching the queue.
TEST_F(StampAnchorTest, TwoOnTimeMessagesInOneCellAreBothRefused)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  const int64_t stamp = kStart + 1 * kNs;
  feed(stamp + 1 * kGrid / 10, 1);  // 1 ms into the cell
  feed(stamp + 6 * kGrid / 10, 2);  // 6 ms into the same cell
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.admitted, 0u) << "a collided record still counts as admitted";
  EXPECT_EQ(c.collision, 2u) << "both messages of a collided cell must be refused";
  EXPECT_TRUE(fake_->queued.empty()) << "the first message's event stayed in the queue";
  EXPECT_EQ(ext::pending(0), 0u);

  const int inserts = fake_->insert_calls.load();
  feed(stamp + 9 * kGrid / 10, 3);  // 9 ms into the cell, after the collision
  c = ext::counters(0);
  EXPECT_EQ(c.collision, 3u);
  EXPECT_EQ(fake_->insert_calls.load(), inserts) <<
    "a message into an already collided cell was offered to the queue again";

  // Nothing of the cell runs, even if the runtime's event survived somewhere.
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, 1), 0);
  EXPECT_TRUE(points_.empty());

  // Once the scheduler is past the cell the marker is dropped and a new cell
  // further out is admitted as usual.
  fake_->current = stamp + kDelay + kReserve + kGrid;
  feed(stamp + 1 * kNs, 4);
  EXPECT_EQ(ext::counters(0).admitted, 1u);
}

// The record is findable by id before its event
// exists, so a worker that runs the event right after the insert, before the
// ingress thread resumes, dispatches it instead of skipping it.
TEST_F(StampAnchorTest, EventRunBeforeTheIngressThreadResumesFindsItsRecord)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  fake_->dispatch_in_insert = true;
  feed(kStart + 1 * kNs, 7);
  EXPECT_EQ(points_, std::vector<int64_t>{7}) <<
    "the event ran before the ingress thread registered its record: dispatch skipped it";
  EXPECT_EQ(ext::pending(0), 0u) << "the record leaked in by_id";
  EXPECT_EQ(ext::counters(0).admitted, 1u);
}

// A cell marked collided whose tag the scheduler
// has passed is an overrun, not a collision, and the marker is dropped.
TEST_F(StampAnchorTest, StaleCollisionMarkerIsAnOverrunOncePassed)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  const int64_t stamp = kStart + 1 * kNs;
  feed(stamp + 1 * kGrid / 10, 1);
  feed(stamp + 6 * kGrid / 10, 2);  // same cell: both refused, marker set
  ASSERT_EQ(ext::counters(0).collision, 2u);
  const int inserts = fake_->insert_calls.load();
  fake_->current = kStart + 126 * kGrid;  // the cell's tag, by hand: ceil(1.251 s / 10 ms)
  feed(stamp + 9 * kGrid / 10, 3);        // same cell, on time, scheduler already there
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.overrun, 1u) << "a stale collision marker was reported as a collision";
  EXPECT_EQ(c.collision, 2u);
  EXPECT_EQ(fake_->insert_calls.load(), inserts) << "the passed cell was offered to the queue";
  EXPECT_EQ(fake_->violations.load(), 0);
}

// A failed critical-section enter refuses the
// message instead of deciding its admission outside the lock.
TEST_F(StampAnchorTest, FailedEnterRefusesWithoutInsert)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  fake_->enter_fails = true;
  feed(kStart + 1 * kNs, 1);
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.refused, 1u) << "a message whose enter failed was not refused";
  EXPECT_EQ(c.admitted, 0u);
  EXPECT_EQ(fake_->insert_calls.load(), 0) << "inserted without the scheduler's lock";
  EXPECT_EQ(ext::pending(0), 0u) << "the refused record stayed registered";
}

// An executable that lacks one of the scheduler
// symbols is refused at attach, before any message could be admitted.
TEST_F(StampAnchorTest, MissingSchedulerSymbolIsRefusedAtAttach)
{
  PlainPhysicalAction action;
  fake_->missing = "lf_critical_section_enter";
  EXPECT_EXIT(
  {
    ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
    ext::on_bound(0);
    std::exit(0);
    }, ::testing::ExitedWithCode(EXIT_FAILURE),
    "does not export reactor-c's lf_critical_section_enter");
}

// A constant above the bound the validator applies
// is refused at declare, so the tag arithmetic cannot overflow.
TEST_F(StampAnchorTest, AnchorConstantAboveTheBoundIsRefusedAtDeclare)
{
  EXPECT_EXIT(
  {
    ros2_lf_runtime_declare_external_stamp_anchor(
      "ext776_in", ext::kAnchorMaxNs + 1, kReserve, kGrid);
    std::exit(0);
    }, ::testing::ExitedWithCode(EXIT_FAILURE), "invalid stamp anchor");
  EXPECT_EXIT(
  {
    ros2_lf_runtime_declare_external_stamp_anchor("ext776_in", kDelay, kReserve, 0);
    std::exit(0);
    }, ::testing::ExitedWithCode(EXIT_FAILURE), "invalid stamp anchor");
}

// A tag after the stop tag is refused under the lock, without an insert.
TEST_F(StampAnchorTest, TagAfterTheStopTagIsRefusedWithoutInsert)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  fake_->stop = kStart + 5 * kNs;
  feed(kStart + 5 * kNs, 1);  // tag_rel = 5 250 ms > stop
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.after_stop, 1u);
  EXPECT_EQ(fake_->insert_calls.load(), 0);
  EXPECT_EQ(fake_->after_stop_calls.load(), 1);
  EXPECT_EQ(fake_->violations.load(), 0);
}

// Fast mode cannot honour a stamp anchor: the binding refuses at attach with
// exit code 3, the same signal the generator's --fast refusal names.
TEST_F(StampAnchorTest, FastModeIsRefusedAtAttach)
{
  PlainPhysicalAction action;
  ros2_lf_binding::set_lf_fast_for_test(true);
  EXPECT_EXIT(
  {
    ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
    ext::on_bound(0);
    std::exit(0);
    }, ::testing::ExitedWithCode(3), "fast mode cannot honour");
}

// A message type without header.stamp cannot be stamp-anchored.
TEST_F(StampAnchorTest, MessageTypeWithoutHeaderStampIsRefusedAtAttach)
{
  PlainPhysicalAction action;
  auto & e = g_handle_table[0];
  e.subscription = sub_;  // std_msgs/String
  e.resolved_topic = sub_->get_topic_name();
  ros2_lf_runtime_declare_external_stamp_anchor(sub_->get_topic_name(), kDelay, kReserve, kGrid);
  EXPECT_EXIT(
  {
    ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
    ext::on_bound(0);
    std::exit(0);
    }, ::testing::ExitedWithCode(EXIT_FAILURE), "has no 'header' field");
}

// A physical action with a min spacing would be ignored by the absolute
// insert; the binding refuses it instead of ignoring it silently.
TEST_F(StampAnchorTest, PhysicalActionWithMinSpacingIsRefusedAtAttach)
{
  PlainPhysicalAction action;
  action.trigger.period = 1000;
  EXPECT_EXIT(
  {
    ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
    ext::on_bound(0);
    std::exit(0);
    }, ::testing::ExitedWithCode(EXIT_FAILURE), "plain physical action");
}

// ---------------------------------------------------------------------------
// Stamp anchors and replay edge cases
// ---------------------------------------------------------------------------

namespace
{
using ext::SymbolOrigin;
int g_exe_object = 0;    // identity of "the executable" for the fake resolver
int g_other_object = 0;  // identity of any other object (the binding)
const char * g_foreign_symbol = nullptr;     // resolves to the other object
const char * g_unresolved_symbol = nullptr;  // does not resolve
int g_dummy_address = 0;

ext::SymbolOrigin fake_resolve(const char * name)
{
  ext::SymbolOrigin o;
  if (g_unresolved_symbol != nullptr && std::strcmp(name, g_unresolved_symbol) == 0) {
    return o;
  }
  o.address = &g_dummy_address;
  if (g_foreign_symbol != nullptr && std::strcmp(name, g_foreign_symbol) == 0) {
    o.object = &g_other_object;
    o.path = "/ws/install/ros2_lf_binding/lib/libros2_lf_binding.so";
  } else {
    o.object = &g_exe_object;
    o.path = "/ws/install/app/lib/app/app";
  }
  return o;
}
}  // namespace

// A scheduler symbol resolved from another object than the
// executable (the binding's own reactor-c) is refused, naming the symbol and
// the object; one origin line per symbol is reported.
TEST(SchedulerSymbolOrigin, SymbolFromAnotherObjectIsRefused)
{
  std::vector<std::string> origins;
  g_foreign_symbol = nullptr;
  g_unresolved_symbol = nullptr;
  EXPECT_EQ(ext::check_scheduler_symbols(&fake_resolve, &g_exe_object, &origins), "");
  EXPECT_EQ(origins.size(), 11u);

  g_foreign_symbol = "lf_critical_section_enter";
  origins.clear();
  const std::string problem =
    ext::check_scheduler_symbols(&fake_resolve, &g_exe_object, &origins);
  EXPECT_NE(problem.find("reactor-c's lf_critical_section_enter resolves to "
    "'/ws/install/ros2_lf_binding/lib/libros2_lf_binding.so', not to the executable"),
    std::string::npos) << "a scheduler symbol from another object was accepted: '" <<
    problem << "'";
  bool logged = false;
  for (const std::string & line : origins) {
    logged = logged || line.find("lf_critical_section_enter -> '/ws/install/ros2_lf_binding/"
      "lib/libros2_lf_binding.so' (NOT the executable)") != std::string::npos;
  }
  EXPECT_TRUE(logged) << "the foreign symbol's origin line is missing";

  g_foreign_symbol = nullptr;
  g_unresolved_symbol = "_lf_schedule_at_tag";
  EXPECT_EQ(
    ext::check_scheduler_symbols(&fake_resolve, &g_exe_object, nullptr),
    "the executable does not export reactor-c's _lf_schedule_at_tag");
  g_unresolved_symbol = nullptr;
}

// Finding 1, production resolver on real objects: the test binary defines
// _lf_get_environments (the stub, exported with -rdynamic), while
// lf_atomic_add_fetch is reactor-c code inside libros2_lf_binding.so. With
// lf_critical_section_enter mapped onto the binding's copy, the check refuses.
namespace
{
SymbolOrigin real_with_enter_from_binding(const char * name)
{
  return ext::resolve_symbol(
    std::strcmp(name, "lf_critical_section_enter") == 0 ? "lf_atomic_add_fetch" :
    "_lf_get_environments");
}
}  // namespace

TEST(SchedulerSymbolOrigin, ProductionResolverTellsTheExecutableFromTheBindingLibrary)
{
  const void * exe = ext::main_program_object();
  ASSERT_NE(exe, nullptr);
  const ext::SymbolOrigin own = ext::resolve_symbol("_lf_get_environments");
  ASSERT_NE(own.address, nullptr);
  EXPECT_EQ(own.object, exe) << "the test binary's own symbol resolved to " << own.path;
  const ext::SymbolOrigin lib = ext::resolve_symbol("lf_atomic_add_fetch");
  ASSERT_NE(lib.address, nullptr);
  EXPECT_NE(lib.object, exe) << "the binding library's symbol was taken for the executable's";
  EXPECT_NE(lib.path.find("libros2_lf_binding.so"), std::string::npos) << lib.path;

  std::vector<std::string> origins;
  const std::string problem =
    ext::check_scheduler_symbols(&real_with_enter_from_binding, exe, &origins);
  EXPECT_NE(problem.find("reactor-c's lf_critical_section_enter resolves to '"),
    std::string::npos) << "a scheduler symbol from libros2_lf_binding.so was accepted: '" <<
    problem << "'";
  EXPECT_NE(problem.find("libros2_lf_binding.so', not to the executable"), std::string::npos) <<
    problem;
}

// reactor-c refusing a future tag is REFUSED, not OVERRUN; a tag
// at or before the current one stays an OVERRUN.
TEST_F(StampAnchorTest, SchedulerRefusalIsRefusedAndPastTagIsOverrun)
{
  PlainPhysicalAction action;
  attach(action);
  g_fake_now.store(kStart);
  fake_->schedule_refuses = true;
  feed(kStart + 1 * kNs, 1);
  ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.refused, 1u) << "reactor-c's refusal of a future tag was not reported as refused";
  EXPECT_EQ(c.overrun, 0u) << "reactor-c's refusal of a future tag was reported as an overrun";

  fake_->schedule_refuses = false;
  fake_->current = kStart + 100 * kNs;  // past every tag of the stamp below
  feed(kStart + 2 * kNs, 2);
  c = ext::counters(0);
  EXPECT_EQ(c.overrun, 1u) << "a tag before the current tag was not reported as an overrun";
  EXPECT_EQ(c.refused, 1u);
  EXPECT_EQ(c.admitted, 0u);
  EXPECT_EQ(ext::pending(0), 0u);
}

namespace
{
namespace rti = rosidl_typesupport_introspection_cpp;

/// A copy of a real type's introspection with one nested message member
/// replaced, so a look-alike keeps the exact field layout of the original.
struct LookAlike
{
  rti::MessageMembers members{};
  std::vector<rti::MessageMember> fields;
  rosidl_message_type_support_t ts{};

  LookAlike(const rti::MessageMembers & real, const char * ns)
  : members(real), fields(real.members_, real.members_ + real.member_count_)
  {
    members.members_ = fields.data();
    if (ns != nullptr) {
      members.message_namespace_ = ns;
    }
    ts.typesupport_identifier = rti::typesupport_identifier;
    ts.data = &members;
  }
  void replace(const char * field, const rosidl_message_type_support_t * nested)
  {
    for (auto & f : fields) {
      if (std::strcmp(f.name_, field) == 0) {
        f.members_ = nested;
      }
    }
  }
};

const rti::MessageMembers & nested_of(const rti::MessageMembers & m, const char * field)
{
  for (uint32_t i = 0; i < m.member_count_; ++i) {
    if (std::strcmp(m.members_[i].name_, field) == 0) {
      return *static_cast<const rti::MessageMembers *>(m.members_[i].members_->data);
    }
  }
  throw std::runtime_error(field);
}
}  // namespace

// A header that only looks like std_msgs/Header (same fields and
// layout, other package), or whose stamp only looks like
// builtin_interfaces/Time, is refused; the real PointStamped is accepted.
TEST_F(StampAnchorTest, LookAlikeHeaderOrTimeTypeIsRefused)
{
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    &psub_->get_message_type_support_handle(), rti::typesupport_identifier);
  ASSERT_NE(ts, nullptr);
  const auto & point = *static_cast<const rti::MessageMembers *>(ts->data);
  const auto & header = nested_of(point, "header");
  const auto & time = nested_of(header, "stamp");

  size_t sec = 0;
  size_t nsec = 0;
  std::string why;
  ASSERT_TRUE(ext::locate_header_stamp_in(&point, &sec, &nsec, &why)) << why;
  geometry_msgs::msg::PointStamped m;
  const auto * base = reinterpret_cast<const char *>(&m);
  EXPECT_EQ(sec, static_cast<size_t>(reinterpret_cast<const char *>(&m.header.stamp.sec) - base));
  EXPECT_EQ(
    nsec, static_cast<size_t>(reinterpret_cast<const char *>(&m.header.stamp.nanosec) - base));

  // my_pkg::msg::Header with std_msgs/Header's exact fields.
  LookAlike fake_header(header, "my_pkg::msg");
  LookAlike top1(point, nullptr);
  top1.replace("header", &fake_header.ts);
  why.clear();
  EXPECT_FALSE(ext::locate_header_stamp_in(&top1.members, &sec, &nsec, &why)) <<
    "a look-alike my_pkg::msg::Header was accepted as std_msgs/Header";
  EXPECT_NE(why.find("'header' is a my_pkg::msg::Header, not a std_msgs::msg::Header"),
    std::string::npos) << why;

  // std_msgs/Header whose stamp is my_pkg::msg::Time with Time's exact fields.
  LookAlike fake_time(time, "my_pkg::msg");
  LookAlike real_header(header, nullptr);
  real_header.replace("stamp", &fake_time.ts);
  LookAlike top2(point, nullptr);
  top2.replace("header", &real_header.ts);
  why.clear();
  EXPECT_FALSE(ext::locate_header_stamp_in(&top2.members, &sec, &nsec, &why)) <<
    "a look-alike my_pkg::msg::Time was accepted as builtin_interfaces/Time";
  EXPECT_NE(why.find("'header.stamp' is a my_pkg::msg::Time, not a builtin_interfaces::msg::Time"),
    std::string::npos) << why;
}

// A replay of a recording inserts exactly the records whose final
// disposition is admitted, at their live tags, delivers the same messages in
// the same tag order, lists the others without inserting them, and lowers the
// stop tag to the recorded lf_stop tag. The recording is the binding's own
// (plant_record, from the live path above), not a hand-written file.
class StampReplayTest : public StampAnchorTest
{
protected:
  void TearDown() override
  {
    ros2_lf_binding::plant_replay::set_dir_for_test(std::nullopt);
    StampAnchorTest::TearDown();
  }

  /// A live run into `dir`: records 1 (admitted, then collided by 2), 2
  /// (collision), 3 and 5 (admitted, cells of their own), 4 (late). Returns
  /// the live run's delivered x values in tag order and its final (tag, x).
  std::vector<std::pair<int64_t, int64_t>> record_live(const std::filesystem::path & dir)
  {
    namespace fs = std::filesystem;
    fs::remove_all(dir);
    ros2_lf_binding::plant_record::set_dir_for_test(dir.string());
    PlainPhysicalAction action;
    attach(action);
    g_fake_now.store(kStart);
    const int64_t stamp = kStart + 1 * kNs;
    feed(stamp + 1 * kGrid / 10, 1);
    feed(stamp + 6 * kGrid / 10, 2);
    feed(stamp + 5 * kGrid, 3);
    feed(kStart - 1 * kNs, 4);
    feed(stamp + 9 * kGrid, 5);
    ros2_lf_binding::plant_record::close();
    ros2_lf_binding::plant_record::set_dir_for_test(std::string());
    // Live delivery: the events still queued, in tag order.
    std::vector<std::pair<int64_t, int64_t>> live;
    for (int64_t t : fake_->queued) {
      int64_t record = -1;
      for (const auto & ins : fake_->inserts) {
        if (ins.first == t) {
          record = ins.second;
        }
      }
      points_.clear();
      EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, record), 1);
      EXPECT_EQ(points_.size(), 1u);
      live.emplace_back(t, points_.empty() ? -1 : points_[0]);
    }
    // The run's stop, as plant_record::lf_stop writes it at a 10 s timeout.
    std::ofstream run(dir / "run.jsonl");
    run << "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" <<
      ros2_lf_binding::plant_record::kFormatVersion << ",\"kind\":\"run\",\"lf_time_start\":" <<
      kStart << "}\n";
    run <<
      "{\"ev\":\"lf_stop\",\"tag_ns\":10000000000,\"microstep\":0,\"condition\":\"timeout\"}\n";
    return live;
  }

  /// Fresh binding state with the anchor declared again, replaying `dir`.
  void start_replay(const std::filesystem::path & dir)
  {
    ext::reset_for_test();
    fake_ = std::make_unique<FakeScheduler>();
    g_fake = fake_.get();
    ext::set_scheduler_ops_for_test(&kFakeOps);
    ros2_lf_runtime_declare_external_stamp_anchor(
      psub_->get_topic_name(), kDelay, kReserve, kGrid);
    ros2_lf_binding::plant_replay::set_dir_for_test(dir.string());
    points_.clear();
  }
};

TEST_F(StampReplayTest, ReplayInsertsOnlyAdmittedRecordsAtTheirLiveTags)
{
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800";
  const auto live = record_live(dir);
  ASSERT_EQ(live.size(), 2u) << "records 3 and 5 stay admitted live";
  EXPECT_EQ(live[0].second, 3);
  EXPECT_EQ(live[1].second, 5);

  start_replay(dir);
  PlainPhysicalAction action;
  attach(action);  // the startup reaction: register, bind, replay
  EXPECT_EQ(fake_->stop, kStart + 10 * kNs) << "the replay did not take the live stop tag";
  EXPECT_EQ(fake_->stop_microstep, 0u);
  ASSERT_EQ(fake_->inserts.size(), live.size()) <<
    "the replay inserted a record that was not admitted in the live run";
  std::vector<std::pair<int64_t, int64_t>> replayed;
  for (const auto & ins : fake_->inserts) {
    points_.clear();
    EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, ins.second), 1);
    replayed.emplace_back(ins.first, points_.empty() ? -1 : points_[0]);
  }
  EXPECT_EQ(replayed, live) << "replayed (tag, value) differ from the live run's";
  const ext::ReplayCounters rc = ext::replay_counters(0);
  EXPECT_EQ(rc.inserted, 2u);
  EXPECT_EQ(rc.delivered, 2u);
  EXPECT_EQ(rc.not_admitted, 3u) << "records 1, 2 (collision) and 4 (late)";
  EXPECT_EQ(rc.past_stop, 0u);
  EXPECT_EQ(ext::counters(0).admitted, 0u) << "a replay admitted a live message";
  EXPECT_EQ(fake_->violations.load(), 0) <<
    "the replay touched the scheduler outside its critical section";
  ext::log_summary();
  EXPECT_FALSE(ext::replay_failed()) << "a faithful replay was reported FAILED";
}

// A replay that a stop request ends
// before the live stop tag is reported FAILED at shutdown, and the runtime
// exits 3 on replay_failed().
TEST_F(StampReplayTest, ReplayStoppedBeforeTheLiveStopTagFails)
{
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800_early";
  record_live(dir);
  start_replay(dir);
  PlainPhysicalAction action;
  attach(action);
  for (const auto & ins : fake_->inserts) {
    EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, ins.second), 1);
  }
  fake_->stop = kStart + 3 * kNs;  // a request_stop at 3 s, after every input
  ext::log_summary();
  EXPECT_TRUE(ext::replay_failed()) <<
    "a replay that stopped at 3 s instead of the live 10 s was not reported FAILED";
}

// A header-only topic file (the live run
// subscribed, nothing arrived) without a subscription in the replay fails.
TEST_F(StampReplayTest, ReplayWithoutARecordedTopicFails)
{
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800_topic";
  record_live(dir);
  std::ofstream other(dir / ros2_lf_binding::plant_record::topic_file("/other"));
  other << "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" <<
    ros2_lf_binding::plant_record::kFormatVersion << ",\"kind\":\"external\",\"topic\":"
    "\"/other\",\"type\":\"geometry_msgs/msg/PointStamped\",\"lf_time_start\":" << kStart <<
    ",\"delay_ns\":1,\"ingress_reserve_ns\":0,\"grid_ns\":1}\n";
  other.close();
  start_replay(dir);
  PlainPhysicalAction action;
  attach(action);
  for (const auto & ins : fake_->inserts) {
    EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, ins.second), 1);
  }
  ext::log_summary();
  EXPECT_TRUE(ext::replay_failed()) <<
    "a recorded topic with no subscription in the replay was not reported FAILED";
}

// Topic files and run.jsonl of different runs.
TEST_F(StampReplayTest, ReplayOfMixedRunsIsRefused)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800_mixed";
  record_live(dir);
  {
    std::ofstream run(dir / "run.jsonl", std::ios::trunc);
    run << "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" <<
      ros2_lf_binding::plant_record::kFormatVersion << ",\"kind\":\"run\",\"lf_time_start\":" <<
      kStart + 1 << "}\n";
    run <<
      "{\"ev\":\"lf_stop\",\"tag_ns\":10000000000,\"microstep\":0,\"condition\":\"timeout\"}\n";
  }
  start_replay(dir);
  PlainPhysicalAction action;
  EXPECT_EXIT(
    attach(action), ::testing::ExitedWithCode(3),
    "rcllf replay: FAILED: .*was recorded by a run that started at");
}

TEST_F(StampReplayTest, ReplayWithoutKeepaliveIsRefused)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800_ka";
  record_live(dir);
  start_replay(dir);
  fake_->keepalive = false;
  PlainPhysicalAction action;
  EXPECT_EXIT(attach(action), ::testing::ExitedWithCode(3), "rcllf replay: FAILED: .*keepalive");
}

TEST_F(StampReplayTest, ReplayWithAnEarlierStopTagIsRefused)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const auto dir = std::filesystem::temp_directory_path() / "rcllf_ext_replay_800_stop";
  record_live(dir);
  start_replay(dir);
  fake_->stop = kStart + 2 * kNs;  // e.g. a shorter -o timeout on the replay
  PlainPhysicalAction action;
  EXPECT_EXIT(
    attach(action), ::testing::ExitedWithCode(3),
    "rcllf replay: FAILED: .*not the live run's");
}

// An external /tf anchored at stamp. tf2_msgs/TFMessage
// has no top-level header; the anchor is the one header.stamp all its
// transforms carry. A message whose transforms carry two stamps, or none, is
// ended unadmitted as tf_mixed_stamps: never split, never anchored at a
// guessed stamp.
class TfStampAnchorTest : public StampAnchorTest
{
protected:
  void SetUp() override
  {
    StampAnchorTest::SetUp();
    tfsub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "ext91_tf", rclcpp::QoS(10).reliable(),
      [this](const tf2_msgs::msg::TFMessage & m) {tf_sizes_.push_back(m.transforms.size());});
    auto & e = g_handle_table[0];
    e.subscription = tfsub_;
    e.topic = "ext91_tf";
    e.resolved_topic = tfsub_->get_topic_name();
    ros2_lf_runtime_declare_external_stamp_anchor(
      tfsub_->get_topic_name(), kDelay, kReserve, kGrid);
  }

  void TearDown() override
  {
    tfsub_.reset();
    StampAnchorTest::TearDown();
  }

  void feed_tf(const std::vector<int64_t> & stamps)
  {
    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
    for (size_t i = 0; i < stamps.size(); ++i) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp.sec = static_cast<int32_t>(stamps[i] / kNs);
      t.header.stamp.nanosec = static_cast<uint32_t>(stamps[i] % kNs);
      t.header.frame_id = "world";
      t.child_frame_id = "f" + std::to_string(i);
      msg->transforms.push_back(t);
    }
    rmw_message_info_t mi{};
    rclcpp::MessageInfo info(mi);
    ext::on_sample(0, tfsub_, std::static_pointer_cast<void>(msg), info);
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfsub_;
  std::vector<size_t> tf_sizes_;
};

TEST_F(TfStampAnchorTest, CommonStampIsTheAnchorAndMixedOrEmptyIsNotAdmitted)
{
  PlainPhysicalAction action;
  ros2_lf_external_register(nullptr, 0, &never_wake, &action.base);
  ext::on_bound(0);
  ASSERT_NE(ext::stamp_anchor_of("ext91_tf"), nullptr);
  g_fake_now.store(kStart);  // every message below arrives on time
  // Two transforms at stamp_rel 1 000 ms: rel 1 250 ms, on the grid -> tag_rel 1 250 ms.
  feed_tf({kStart + 1000 * kGrid / 10, kStart + 1000 * kGrid / 10});
  // The recorded two-stamp message: 2 000 ms and 2 000 ms + 1 ns.
  feed_tf({kStart + 2000 * kGrid / 10, kStart + 2000 * kGrid / 10 + 1});
  feed_tf({});  // no transform, no stamp
  // One transform at 3 001 ms: rel 3 251 ms -> tag_rel 3 260 ms.
  feed_tf({kStart + 3001 * kGrid / 10});
  const ext::Counters c = ext::counters(0);
  EXPECT_EQ(c.tf_mixed_stamps, 2u) <<
    "a TFMessage with two stamps or none was not ended as tf_mixed_stamps";
  EXPECT_EQ(c.admitted, 2u);
  EXPECT_EQ(c.refused + c.late + c.overrun + c.collision + c.after_stop, 0u);
  ASSERT_EQ(fake_->inserts.size(), 2u) << "a mixed or empty TFMessage reached the scheduler";
  EXPECT_EQ(fake_->inserts[0].first - kStart, 1250 * kGrid / 10);
  EXPECT_EQ(fake_->inserts[1].first - kStart, 3260 * kGrid / 10);
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  EXPECT_EQ(ros2_lf_invoke_external_sub(nullptr, 0, fake_->inserts[0].second), 1);
  ASSERT_EQ(tf_sizes_.size(), 1u);
  EXPECT_EQ(tf_sizes_[0], 2u) << "the admitted TFMessage lost transforms";
}

TEST(TfStampLocator, OnlyTheRealTfMessageIsLocatedAndItsRuleCountsStamps)
{
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  using rosidl_typesupport_introspection_cpp::MessageMember;
  const auto * tf_ts = rosidl_typesupport_cpp::get_message_type_support_handle<
    tf2_msgs::msg::TFMessage>();
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    tf_ts, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  ASSERT_NE(ts, nullptr);
  const MessageMember * seq = nullptr;
  size_t sec = 0;
  size_t nsec = 0;
  std::string why;
  ASSERT_TRUE(
    ext::locate_tf_stamps_in(static_cast<const MessageMembers *>(ts->data), &seq, &sec, &nsec,
    &why)) << why;
  tf2_msgs::msg::TFMessage m;
  int64_t stamp = -7;
  EXPECT_EQ(ext::common_tf_stamp(seq, &m, sec, nsec, &stamp), 0);
  EXPECT_EQ(stamp, -7);
  m.transforms.resize(3);
  for (auto & t : m.transforms) {
    t.header.stamp.sec = 5;
    t.header.stamp.nanosec = 7;
  }
  EXPECT_EQ(ext::common_tf_stamp(seq, &m, sec, nsec, &stamp), 1);
  EXPECT_EQ(stamp, 5 * kNs + 7);
  m.transforms[2].header.stamp.nanosec = 8;
  EXPECT_EQ(ext::common_tf_stamp(seq, &m, sec, nsec, &stamp), -1);

  const auto * pt_ts = rosidl_typesupport_cpp::get_message_type_support_handle<
    geometry_msgs::msg::PointStamped>();
  const rosidl_message_type_support_t * pts = get_message_typesupport_handle(
    pt_ts, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  ASSERT_NE(pts, nullptr);
  EXPECT_FALSE(
    ext::locate_tf_stamps_in(static_cast<const MessageMembers *>(pts->data), &seq, &sec, &nsec,
    &why));
  EXPECT_EQ(why, "not a tf2_msgs::msg::TFMessage");
}

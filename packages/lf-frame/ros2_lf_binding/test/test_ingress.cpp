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

/// @file test_ingress.cpp
/// @brief One-scheduler ingress role (T5): admission, mailbox
///        capacity, wake edges, close/refusal terminalization, the atexit
///        backstop, and exceptions from handle_request.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "component_manager.hpp"  // NOLINT(build/include_subdir)
#include "ingress.hpp"  // NOLINT(build/include_subdir)
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "std_msgs/msg/int32.hpp"
#include "thread_role.hpp"  // NOLINT(build/include_subdir)

extern "C" {
#include "core/environment.h"
}

extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = nullptr;
  return 0;
}

using namespace std::chrono_literals;
using ros2_lf_binding::IngressRecord;
using ros2_lf_binding::IngressRole;
using ros2_lf_binding::ScopedThreadRoleForTest;
using ros2_lf_binding::ThreadRole;
using SetParameters = rcl_interfaces::srv::SetParameters;

namespace
{
// One node (and service) name per process, so concurrent runs never meet.
// Slack over the 500 ms backstop deadline, from stop call to reaped exit.
// Measured 502 to 510 ms in 400 runs under 8 concurrent loops (smoke, busy host).
constexpr int64_t kBackstopSlackMs = 500;

std::string node_name() {return "ingress_test_node_" + std::to_string(getpid());}

struct FakeWake
{
  std::atomic<int> calls{0};
  std::atomic<int> ret{1};
  std::mutex * block_on = nullptr;  ///< (i): a held fake LF env mutex
};

int fake_wake(void * a)
{
  auto * w = static_cast<FakeWake *>(a);
  w->calls++;
  if (w->block_on != nullptr) {
    std::lock_guard<std::mutex> lk(*w->block_on);
  }
  return w->ret.load();
}

rclcpp::ServiceBase::SharedPtr find_service(
  const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  rclcpp::ServiceBase::SharedPtr found;
  node->get_node_base_interface()->for_each_callback_group(
    [&](rclcpp::CallbackGroup::SharedPtr g) {
      g->collect_all_ptrs(
        [](const rclcpp::SubscriptionBase::SharedPtr &) {},
        [&](const rclcpp::ServiceBase::SharedPtr & s) {
          if (s->get_service_name() == name) {found = s;}
        },
        [](const rclcpp::ClientBase::SharedPtr &) {},
        [](const rclcpp::TimerBase::SharedPtr &) {},
        [](const rclcpp::Waitable::SharedPtr &) {});
    });
  return found;
}

int64_t mono_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string backstop_stamp_path(pid_t parent)
{
  return std::string(::testing::TempDir()) + "/backstop_stamp_" + std::to_string(parent);
}

}  // namespace

class IngressTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>(node_name());
    node_->declare_parameter("max_speed", 0.0);
    on_set_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        for (const auto & p : ps) {
          seen_.push_back(p.as_double());
          if (throw_in_on_set_) {
            throw std::runtime_error("on-set handler threw");
          }
        }
        return r;
      });
    set_params_ = find_service(node_, "/" + node_name() + "/set_parameters");
    ASSERT_TRUE(set_params_);
  }

  void TearDown() override {IngressRole::instance().reset_for_test();}

  /// A set_parameters record as the ingress thread would build it.
  IngressRecord set_record(double value)
  {
    IngressRecord r;
    r.id = IngressRole::instance().next_record_id();
    r.service = set_params_;
    r.service_suffix = "set_parameters";
    r.header = set_params_->create_request_header();
    auto req = std::make_shared<SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("max_speed", value).to_parameter_msg());
    r.request = req;
    r.t_rx = std::chrono::steady_clock::now();
    return r;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_;
  rclcpp::ServiceBase::SharedPtr set_params_;
  std::vector<double> seen_;
  bool throw_in_on_set_ = false;
  FakeWake wake_;
};

// (a) admit() is the only way in, and refuses everything but infrastructure.
TEST_F(IngressTest, AdmitRefusesNonInfrastructure)
{
  auto & ing = IngressRole::instance();
  auto timer = node_->create_wall_timer(1h, []() {});
  auto sub = node_->create_subscription<rcl_interfaces::msg::SetParametersResult>(
    "t", 1, [](rcl_interfaces::msg::SetParametersResult::ConstSharedPtr) {});
  auto cli = node_->create_client<SetParameters>("other/set_parameters");
  auto user_srv = node_->create_service<SetParameters>(
    "my_service",
    [](SetParameters::Request::ConstSharedPtr, SetParameters::Response::SharedPtr) {});
  auto wrong_type = node_->create_service<SetParameters>(
    "change_state",
    [](SetParameters::Request::ConstSharedPtr, SetParameters::Response::SharedPtr) {});
  EXPECT_FALSE(ing.admit("k", timer));
  EXPECT_FALSE(ing.admit("k", sub));
  EXPECT_FALSE(ing.admit("k", cli));
  EXPECT_FALSE(ing.admit("k", rclcpp::Waitable::SharedPtr()));
  EXPECT_FALSE(ing.admit("k", "/" + node_name(), user_srv));
  EXPECT_FALSE(ing.admit("k", "/" + node_name(), wrong_type)) << "name alone must not admit";
  EXPECT_FALSE(ing.admit("k", "/another_node", set_params_)) << "other node's name space";
  EXPECT_EQ(ing.admitted_count(), 0u);
  EXPECT_TRUE(ing.admit("k", "/" + node_name(), set_params_));
  EXPECT_EQ(ing.admitted_count(), 1u);
}

TEST_F(IngressTest, RegisterAdmitsAllParameterServicesOfANeverAddedNode)
{
  auto & ing = IngressRole::instance();
  ing.register_node("plain", &fake_wake, &wake_, node_->get_node_base_interface());
  EXPECT_EQ(ing.admitted_count(), 7u);  // 6 parameter services + get_type_description
}

// A component loaded through LFComponentManager is never handed to
// the registration executor, so its services are admissible.
TEST_F(IngressTest, LoadedComponentServicesAreAdmissible)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::weak_ptr<rclcpp::Executor> weak_exec = exec;
  auto manager = std::make_shared<ros2_lf_binding::LFComponentManager>(
    weak_exec, "ingress_test_manager");
  ASSERT_TRUE(
    manager->load_component(
      "tf2_ros", "tf2_ros::StaticTransformBroadcasterNode", "comp_key", *exec,
      "ingress_component", ""));
  auto & ing = IngressRole::instance();
  ing.register_node("comp_key", &fake_wake, &wake_, manager->find_by_class("comp_key"));
  EXPECT_EQ(ing.admitted_count(), 7u)
    << "component services claimed by another wait set (add_node_to_executor not a no-op?)";
}

// (b) 9 requests at CAP 8, through the real ingress thread and a real client:
// 8 stored and answered in FIFO order once drained, the 9th refused at once.
TEST_F(IngressTest, CapacityEightFifoNinthRejected)
{
  auto & ing = IngressRole::instance();
  ing.register_node("cap", &fake_wake, &wake_, node_->get_node_base_interface());
  auto client_node = std::make_shared<rclcpp::Node>("ingress_test_client");
  auto client = client_node->create_client<SetParameters>("/" + node_name() + "/set_parameters");
  ASSERT_TRUE(client->wait_for_service(5s));
  std::vector<rclcpp::Client<SetParameters>::SharedFuture> futures;
  for (int i = 1; i <= 9; ++i) {
    auto req = std::make_shared<SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("max_speed", 1.0 * i).to_parameter_msg());
    futures.push_back(client->async_send_request(req).future.share());
  }
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(client_node);
  ex.spin_until_future_complete(futures[8], 5s);
  ASSERT_EQ(futures[8].wait_for(0s), std::future_status::ready);
  ASSERT_EQ(futures[8].get()->results.size(), 1u);
  EXPECT_FALSE(futures[8].get()->results[0].successful);
  EXPECT_NE(futures[8].get()->results[0].reason.find("overflow"), std::string::npos);
  auto c = ing.counts("cap");
  EXPECT_EQ(c.stored, 8u);
  EXPECT_EQ(c.rejected_overflow, 1u);
  EXPECT_EQ(wake_.calls.load(), 1);
  EXPECT_TRUE(seen_.empty()) << "a user callback ran before any LF drain";

  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("cap"));
  int drains = 0;
  while (ing.drain_one("cap")) {
    drains++;
  }
  drains++;
  EXPECT_EQ(drains, 8);
  for (int i = 0; i < 8; ++i) {
    ex.spin_until_future_complete(futures[i], 5s);
    ASSERT_EQ(futures[i].wait_for(0s), std::future_status::ready) << "request " << i + 1;
    EXPECT_TRUE(futures[i].get()->results[0].successful);
  }
  EXPECT_EQ(seen_, (std::vector<double>{1, 2, 3, 4, 5, 6, 7, 8}));
  EXPECT_EQ(ing.counts("cap").consumed, 8u);
  EXPECT_FALSE(ing.wake_pending("cap"));
}

// execution.yaml `ingress_capacity: 2`, set through the C entry
// point the generated startup reaction calls. Three requests through the real
// ingress thread: two stored and answered in order, the third refused at once.
TEST_F(IngressTest, ConfiguredCapacityTwoThirdRejected)
{
  auto & ing = IngressRole::instance();
  ros2_lf_ingress_set_capacity("cap2", 2);
  ing.register_node("cap2", &fake_wake, &wake_, node_->get_node_base_interface());
  EXPECT_EQ(ing.capacity("cap2"), 2u);
  auto client_node = std::make_shared<rclcpp::Node>("ingress_test_client_cap2");
  auto client = client_node->create_client<SetParameters>("/" + node_name() + "/set_parameters");
  ASSERT_TRUE(client->wait_for_service(5s));
  std::vector<rclcpp::Client<SetParameters>::SharedFuture> futures;
  for (int i = 1; i <= 3; ++i) {
    auto req = std::make_shared<SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("max_speed", 1.0 * i).to_parameter_msg());
    futures.push_back(client->async_send_request(req).future.share());
  }
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(client_node);
  ex.spin_until_future_complete(futures[2], 5s);
  ASSERT_EQ(futures[2].wait_for(0s), std::future_status::ready);
  EXPECT_FALSE(futures[2].get()->results[0].successful);
  EXPECT_NE(futures[2].get()->results[0].reason.find("overflow"), std::string::npos);
  auto c = ing.counts("cap2");
  EXPECT_EQ(c.stored, 2u);
  EXPECT_EQ(c.rejected_overflow, 1u);

  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("cap2"));
  EXPECT_TRUE(ing.drain_one("cap2"));
  EXPECT_FALSE(ing.drain_one("cap2"));
  for (int i = 0; i < 2; ++i) {
    ex.spin_until_future_complete(futures[i], 5s);
    ASSERT_EQ(futures[i].wait_for(0s), std::future_status::ready) << "request " << i + 1;
    EXPECT_TRUE(futures[i].get()->results[0].successful);
  }
  EXPECT_EQ(seen_, (std::vector<double>{1, 2}));
}

// A capacity below 1 is ignored and the default stays.
TEST_F(IngressTest, NonPositiveCapacityIgnored)
{
  ros2_lf_ingress_set_capacity("cap0", 0);
  EXPECT_EQ(IngressRole::instance().capacity("cap0"), ros2_lf_binding::kIngressCap);
}

namespace
{
std::vector<std::string> g_warned;

void warn_capture_handler(
  const rcutils_log_location_t *, int severity, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  if (severity != RCUTILS_LOG_SEVERITY_WARN || std::string(name) != "ros2_lf_binding") {
    return;
  }
  char buf[1024];
  va_list copy;
  va_copy(copy, *args);
  vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_warned.emplace_back(buf);
}
}  // namespace

// The ignored-capacity warning names the capacity the key keeps, not the default.
TEST_F(IngressTest, NonPositiveCapacityWarningNamesTheKeptCapacity)
{
  ros2_lf_ingress_set_capacity("capkeep", 2);
  g_warned.clear();
  rcutils_logging_set_output_handler(warn_capture_handler);
  ros2_lf_ingress_set_capacity("capkeep", 0);
  rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  EXPECT_EQ(IngressRole::instance().capacity("capkeep"), 2u);
  ASSERT_EQ(g_warned.size(), 1u);
  EXPECT_EQ(
    g_warned[0],
    "ros2_lf_ingress_set_capacity: key 'capkeep': capacity 0 < 1 ignored, keeping 2");
}

// (c) one wake per empty-to-nonempty edge.
TEST_F(IngressTest, OneWakePerEdge)
{
  auto & ing = IngressRole::instance();
  ing.register_node("edge", &fake_wake, &wake_, nullptr);
  for (int i = 0; i < 3; ++i) {
    ing.on_request("edge", set_record(i));
  }
  EXPECT_EQ(wake_.calls.load(), 1);
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("edge"));
  EXPECT_TRUE(ing.drain_one("edge"));
  EXPECT_TRUE(ing.drain_one("edge"));
  EXPECT_FALSE(ing.drain_one("edge"));
  EXPECT_FALSE(ing.wake_pending("edge"));
  ing.on_request("edge", set_record(9));
  EXPECT_EQ(wake_.calls.load(), 2);
  EXPECT_EQ(ing.counts("edge").consumed, 3u);
}

// (d) close with 3 queued: 3 terminal, no handle_request. (e) then 0 wakes.
TEST_F(IngressTest, CloseTerminalizesQueuedAndRejectsLater)
{
  auto & ing = IngressRole::instance();
  ing.register_node("close", &fake_wake, &wake_, nullptr);
  for (int i = 0; i < 3; ++i) {
    ing.on_request("close", set_record(i));
  }
  ing.close("close");
  auto c = ing.counts("close");
  EXPECT_EQ(c.terminal, 3u);
  EXPECT_TRUE(seen_.empty());
  ing.on_request("close", set_record(4));
  EXPECT_EQ(wake_.calls.load(), 1);
  EXPECT_EQ(ing.counts("close").rejected_closed, 1u);
  EXPECT_EQ(ing.counts("close").stored, 3u);
}

// (f) the wake is refused (handle 0): closed, wake_pending 0, record terminal.
TEST_F(IngressTest, IngressSideWakeRefusalCloses)
{
  auto & ing = IngressRole::instance();
  wake_.ret = 0;
  ing.register_node("refuse", &fake_wake, &wake_, nullptr);
  ing.on_request("refuse", set_record(1));
  EXPECT_TRUE(ing.is_closed("refuse"));
  EXPECT_FALSE(ing.wake_pending("refuse"));
  EXPECT_EQ(ing.counts("refuse").terminal, 1u);
  ing.on_request("refuse", set_record(2));
  EXPECT_EQ(ing.counts("refuse").rejected_closed, 1u);
  EXPECT_EQ(wake_.calls.load(), 1);
}

// (g) drain-side refusal: closed, carrier empty, wake_pending 0.
TEST_F(IngressTest, DrainSideRefusalCloses)
{
  auto & ing = IngressRole::instance();
  ing.register_node("drainref", &fake_wake, &wake_, nullptr);
  ing.on_request("drainref", set_record(1));
  ing.on_request("drainref", set_record(2));
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("drainref"));
  ASSERT_TRUE(ing.drain_one("drainref"));
  ing.schedule_refused("drainref");
  EXPECT_TRUE(ing.is_closed("drainref"));
  EXPECT_EQ(ing.carrier_size("drainref"), 0u);
  EXPECT_FALSE(ing.wake_pending("drainref"));
  EXPECT_EQ(ing.counts("drainref").consumed, 1u);
  EXPECT_EQ(ing.counts("drainref").terminal, 1u);
}

// (h) close with ids already in the carrier, then drain_one: a no-op, every
// record terminal exactly once.
TEST_F(IngressTest, CloseWithCarrierThenDrainIsNoOp)
{
  auto & ing = IngressRole::instance();
  ing.register_node("carrier", &fake_wake, &wake_, nullptr);
  for (int i = 0; i < 3; ++i) {
    ing.on_request("carrier", set_record(i));
  }
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("carrier"));
  EXPECT_EQ(ing.carrier_size("carrier"), 3u);
  ing.close("carrier");
  EXPECT_EQ(ing.carrier_size("carrier"), 0u);
  EXPECT_FALSE(ing.drain_one("carrier"));
  auto c = ing.counts("carrier");
  EXPECT_EQ(c.terminal, 3u);
  EXPECT_EQ(c.consumed, 0u);
  EXPECT_EQ(c.stale_drains, 1u);
  EXPECT_TRUE(seen_.empty());
}

// (j) a throwing handle_request gives TERMINAL(exception), not a crash.
TEST_F(IngressTest, ThrowingHandleRequestIsTerminal)
{
  auto & ing = IngressRole::instance();
  throw_in_on_set_ = true;
  ing.register_node("throw", &fake_wake, &wake_, nullptr);
  ing.on_request("throw", set_record(1));
  ScopedThreadRoleForTest lf(ThreadRole::LF_WORKER);
  ASSERT_TRUE(ing.move("throw"));
  EXPECT_FALSE(ing.drain_one("throw"));
  auto c = ing.counts("throw");
  EXPECT_EQ(c.terminal, 1u);
  EXPECT_EQ(c.consumed, 0u);
  EXPECT_EQ(seen_.size(), 1u);
}

// Drain refuses to run a request off an LF worker (guard at the drain).
TEST_F(IngressTest, DrainOffLfWorkerIsTerminalNotRun)
{
  auto & ing = IngressRole::instance();
  ing.register_node("offlf", &fake_wake, &wake_, nullptr);
  ing.on_request("offlf", set_record(1));
  ScopedThreadRoleForTest other(ThreadRole::OTHER);
  ASSERT_TRUE(ing.move("offlf"));
  ing.drain_one("offlf");
  EXPECT_TRUE(seen_.empty());
  EXPECT_EQ(ing.counts("offlf").terminal, 1u);
}

// (i) the ingress thread is stuck in wake() on a held fake LF env mutex (as
// when reactor-c exit()s holding env->mutex): the backstop takes no lock, waits
// its 500 ms deadline and ends the process (timed from the stop call, not
// from the fork, so DDS discovery in the child no longer counts).
TEST_F(IngressTest, BackstopExitsWhenIngressIsStuckInWake)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const std::string stamp = backstop_stamp_path(getpid());
  std::remove(stamp.c_str());
  EXPECT_EXIT(
  {
    std::mutex env_mutex;
    env_mutex.lock();
    wake_.block_on = &env_mutex;
    auto & ing = IngressRole::instance();
    ing.register_node("stuck", &fake_wake, &wake_, node_->get_node_base_interface());
    auto client_node = std::make_shared<rclcpp::Node>("ingress_stuck_client");
    auto client = client_node->create_client<SetParameters>(
        "/" + node_name() + "/set_parameters");
    client->wait_for_service(5s);
    auto req = std::make_shared<SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("max_speed", 1.0).to_parameter_msg());
    auto f = client->async_send_request(req);
    while (wake_.calls.load() == 0) {
      std::this_thread::sleep_for(5ms);
    }
    {
      std::ofstream out(backstop_stamp_path(getppid()), std::ios::trunc);
      out << mono_ns() << std::endl;
    }
    // A backstop that never fires would hang in stop_and_join on the stuck
    // thread: end with exit code 0 and say so, which fails EXPECT_EXIT.
    std::thread([] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500 + kBackstopSlackMs));
      std::fprintf(stderr, "backstop did not fire within %lld ms\n",
        static_cast<long long>(500 + kBackstopSlackMs));  // NOLINT(runtime/int)
      std::_Exit(0);
    }).detach();
    ros2_lf_ingress_stop_atexit();
    std::exit(0);    // not reached: the backstop must _Exit(EXIT_FAILURE)
    },
    ::testing::ExitedWithCode(EXIT_FAILURE), "did not stop within 500 ms");
  const int64_t reaped = mono_ns();
  std::ifstream in(stamp);
  int64_t stopped = 0;
  ASSERT_TRUE(in >> stopped) << "the child never reached the stop call";
  std::remove(stamp.c_str());
  const int64_t ms = (reaped - stopped) / 1000000;
  std::printf("backstop stop call to reaped exit: %lld ms\n",
    static_cast<long long>(ms));  // NOLINT(runtime/int)
  EXPECT_GE(ms, 500) << "the backstop must wait its full deadline";
  EXPECT_LT(ms, 500 + kBackstopSlackMs);
}

// Review minor 1: queued mutating requests that end TERMINAL at shutdown get a
// failure reply instead of leaving `ros2 param set` to time out.
TEST_F(IngressTest, ShutdownTerminalRequestsGetFailureReplies)
{
  auto & ing = IngressRole::instance();
  ing.register_node("term", &fake_wake, &wake_, node_->get_node_base_interface());
  auto client_node = std::make_shared<rclcpp::Node>("ingress_term_client");
  auto client = client_node->create_client<SetParameters>("/" + node_name() + "/set_parameters");
  ASSERT_TRUE(client->wait_for_service(5s));
  std::vector<rclcpp::Client<SetParameters>::SharedFuture> futures;
  for (int i = 1; i <= 2; ++i) {
    auto req = std::make_shared<SetParameters::Request>();
    req->parameters.push_back(rclcpp::Parameter("max_speed", 1.0 * i).to_parameter_msg());
    futures.push_back(client->async_send_request(req).future.share());
  }
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (ing.counts("term").stored < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(ing.counts("term").stored, 2u);
  ing.close("term");  // the drain host's shutdown reaction
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(client_node);
  for (auto & f : futures) {
    ex.spin_until_future_complete(f, 5s);
    ASSERT_EQ(f.wait_for(0s), std::future_status::ready) << "no reply for a TERMINAL request";
    ASSERT_EQ(f.get()->results.size(), 1u);
    EXPECT_FALSE(f.get()->results[0].successful);
    EXPECT_NE(f.get()->results[0].reason.find("shutdown"), std::string::npos)
      << f.get()->results[0].reason;
  }
  EXPECT_EQ(ing.counts("term").terminal, 2u);
  EXPECT_TRUE(seen_.empty());
}

// Review minor 2: once the ingress thread is gone, admit() must not claim success.
TEST_F(IngressTest, AdmitAfterThreadStoppedIsRefused)
{
  auto & ing = IngressRole::instance();
  ing.register_node("gone", &fake_wake, &wake_, nullptr);
  ing.close("gone");  // last close stops and joins the thread
  EXPECT_FALSE(ing.thread_running());
  EXPECT_FALSE(ing.admit("gone", "/" + node_name(), set_params_));
  EXPECT_EQ(ing.admitted_count(), 0u);
}

// The ingress thread spun at normal priority inside the LF
// workers' cpuset (35 to 180 takes per ms with nothing to take) and delayed
// woken workers by 1.8 to 8 ms (E4 fed1host t30-brake, two outcomes). The
// subscriptions' QoS event handlers enable their statuses on the DDS reader's
// status condition, rmw_fastrtps attaches that condition for the subscription
// itself and reports the wait as OK whenever it is triggered, and only taking
// the event clears it. The binding never took a physical QoS event
// (deadline and message_lost run logically at the drain), so one requested-
// deadline-missed event kept the wait set ready for the rest of the run.
namespace
{
std::string spin_topic(const char * tag)
{
  return std::string("/ingress_spin_") + tag + "_" + std::to_string(getpid());
}

/// Wait until `pub` sees one subscription (discovery), at most 5 s.
void wait_matched(const rclcpp::PublisherBase & pub)
{
  for (int i = 0; i < 500 && pub.get_subscription_count() == 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(pub.get_subscription_count(), 0u);
}

/// Ready wakeups of the ingress wait set over `window`, with nothing to take.
uint64_t idle_ready_wakeups(std::chrono::milliseconds window)
{
  auto & ing = IngressRole::instance();
  const uint64_t before = ing.ready_wakeups_for_test();
  std::this_thread::sleep_for(window);
  return ing.ready_wakeups_for_test() - before;
}
}  // namespace

// (a) As brake_fusion creates its readers (loss_options): a deadline QoS and
// a requested-deadline-missed handler. One sample, then silence: the reader
// misses its deadline once per period (20 ms) from then on, a real physical
// event each time. The loop may wake once per event (25 in 500 ms) and must
// block in between; the spin woke 35 to 180 times per millisecond.
TEST_F(IngressTest, IdleWaitBlocksAfterADeadlineMissedEvent)
{
  auto & ing = IngressRole::instance();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().deadline(20ms);
  rclcpp::SubscriptionOptions opts;
  opts.event_callbacks.deadline_callback = [](rclcpp::QOSDeadlineRequestedInfo &) {};
  auto sub = node_->create_subscription<std_msgs::msg::Int32>(
    spin_topic("deadline"), qos, [](std_msgs::msg::Int32::SharedPtr) {}, opts);
  auto pub = node_->create_publisher<std_msgs::msg::Int32>(spin_topic("deadline"), qos);
  // rid 7 has no registered action: external_input::on_sample drops the sample.
  ASSERT_TRUE(ing.admit_external_subscription(7, sub));
  wait_matched(*pub);
  pub->publish(std_msgs::msg::Int32());
  std::this_thread::sleep_for(200ms);  // several deadline periods: the event is set
  const uint64_t wakeups = idle_ready_wakeups(500ms);
  const std::string spin_msg =
    "rcllf ingress: the wait set reported ready " + std::to_string(wakeups) +
    " times in 500 ms with nothing to take (at most one physical deadline event per "
    "20 ms is due): the ingress thread spins on a subscription QoS event (requested "
    "deadline missed) that the binding never takes";
  EXPECT_LE(wakeups, 25u + 10u) << spin_msg;
  EXPECT_GE(ing.qos_events_taken_for_test(), 10u) << "the deadline events were not taken";
}

// (b) No user event callback at all: rclcpp registers incompatible-QoS and
// incompatible-type handlers by default, and a best-effort publisher on a
// reliable reader sets the first one at discovery. Same spin, same cure.
TEST_F(IngressTest, IdleWaitBlocksWithAnIncompatibleQosPublisher)
{
  auto & ing = IngressRole::instance();
  auto sub = node_->create_subscription<std_msgs::msg::Int32>(
    spin_topic("qos"), rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [](std_msgs::msg::Int32::SharedPtr) {});
  auto pub = node_->create_publisher<std_msgs::msg::Int32>(
    spin_topic("qos"), rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
  ASSERT_TRUE(ing.admit_external_subscription(8, sub));
  std::this_thread::sleep_for(500ms);  // discovery: requested incompatible QoS is set
  const uint64_t wakeups = idle_ready_wakeups(500ms);
  const std::string spin_msg =
    "rcllf ingress: the wait set reported ready " + std::to_string(wakeups) +
    " times in 500 ms with nothing to take: the ingress thread spins on a subscription "
    "QoS event (requested incompatible QoS) that the binding never takes";
  EXPECT_LE(wakeups, 10u) << spin_msg;
}

// The mechanism at the rclcpp level, independent of the ingress: after a
// missed deadline (1) the subscription's event handlers alone keep a wait set
// ready, repeatedly, nothing clears them; (2) the subscription alone, its one
// sample taken, is reported ready too (rmw_fastrtps attaches the reader's
// status condition for the subscription, so leaving the handlers out of the
// wait set is no cure); (3) taking the deadline event once, without running
// the handler, makes the subscription alone time out. That is what the ingress
// does (take_qos_events).
TEST_F(IngressTest, DeadlineEventKeepsTheSubscriptionReadyUntilTaken)
{
  // 1 s deadline: the first miss is 1 s after the sample, then one a second.
  // The test used to sleep 250 ms and then expect the 200 ms miss
  // posted; under ctest -j2 load it was not yet (and a Ready wait alone does
  // not prove it is). It now waits until the deadline event itself is ready
  // (at most 10 s). Under load the steps after it may also outlast the next
  // miss, so step (3) retries only on a newly counted miss.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().deadline(1s);
  rclcpp::SubscriptionOptions opts;
  opts.event_callbacks.deadline_callback = [](rclcpp::QOSDeadlineRequestedInfo &) {};
  auto sub = node_->create_subscription<std_msgs::msg::Int32>(
    spin_topic("mech"), qos, [](std_msgs::msg::Int32::SharedPtr) {}, opts);
  auto pub = node_->create_publisher<std_msgs::msg::Int32>(spin_topic("mech"), qos);
  wait_matched(*pub);
  pub->publish(std_msgs::msg::Int32());

  rclcpp::WaitSet events;
  events.add_subscription(sub, rclcpp::SubscriptionWaitSetMask{false, true, false});
  const auto & handlers = sub->get_event_handlers();
  auto it = handlers.find(RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
  ASSERT_NE(it, handlers.end());
  bool missed = false;
  const auto miss_by = std::chrono::steady_clock::now() + 10s;
  while (!missed && std::chrono::steady_clock::now() < miss_by) {
    auto r = events.wait(100ms);
    missed = r.kind() == rclcpp::WaitResultKind::Ready &&
      it->second->is_ready(r.get_wait_set().get_rcl_wait_set());
  }
  ASSERT_TRUE(missed) << "no deadline miss posted within 10 s of a 1 s deadline";
  std::vector<rcl_subscription_event_type_t> ready_types;
  for (int i = 0; i < 3; ++i) {
    auto r = events.wait(0ms);
    ASSERT_EQ(r.kind(), rclcpp::WaitResultKind::Ready) << "round " << i;
    for (const auto & kv : sub->get_event_handlers()) {
      if (kv.second->is_ready(r.get_wait_set().get_rcl_wait_set())) {
        ready_types.push_back(kv.first);
      }
    }
  }
  EXPECT_EQ(
    ready_types,
    (std::vector<rcl_subscription_event_type_t>(3, RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)));

  // (2) The one sample is taken; the subscription alone is still reported
  // ready with nothing to take.
  std_msgs::msg::Int32 taken;
  rclcpp::MessageInfo info;
  ASSERT_TRUE(sub->take(taken, info));
  ASSERT_FALSE(sub->take(taken, info));
  rclcpp::WaitSet data;
  data.add_subscription(sub, rclcpp::SubscriptionWaitSetMask{true, false, false});
  EXPECT_EQ(data.wait(50ms).kind(), rclcpp::WaitResultKind::Ready);
  EXPECT_FALSE(sub->take(taken, info));

  // (3) Take the deadline event (not executed): now the subscription blocks.
  // A Ready wait counts only if no new miss was posted since the take.
  // Retry for as long as every Ready is explained by a new miss: a test thread
  // delayed past the next 1 s deadline is not a failure, so the budget is time
  // (30 s, about 30 deadline periods), not a small attempt count.
  bool blocked = false;
  const auto block_by = std::chrono::steady_clock::now() + 30s;
  for (int attempt = 0; !blocked && (attempt == 0 || std::chrono::steady_clock::now() < block_by);
    ++attempt)
  {
    auto ev = it->second->take_data();
    ASSERT_NE(ev, nullptr);
    if (attempt > 0) {
      ASSERT_GT(static_cast<rclcpp::QOSDeadlineRequestedInfo *>(ev.get())->total_count_change, 0)
        << "the subscription was ready again after the deadline event was taken, with no new "
        "deadline miss";
    }
    blocked = data.wait(20ms).kind() == rclcpp::WaitResultKind::Timeout;
  }
  EXPECT_TRUE(blocked) << "the subscription stayed ready over 30 s of deadline-event takes, "
    "each explained by a new miss";
}

// An admission the ingress thread does not answer in time is
// refused, and it must stay refused: the request is withdrawn, so the thread
// never adds an entity its caller was told is not admitted.
TEST_F(IngressTest, TimedOutAdmissionIsWithdrawn)
{
  auto & ing = IngressRole::instance();
  // Static and held by a unique_lock: a failed ASSERT below still releases the
  // stuck thread, and the mutex outlives every wake() still using it.
  static std::mutex env_mutex;
  std::unique_lock<std::mutex> hold(env_mutex);
  wake_.block_on = &env_mutex;  // the thread sticks in wake() on the next request
  ing.register_node("slow", &fake_wake, &wake_, node_->get_node_base_interface());
  auto client_node = std::make_shared<rclcpp::Node>("ingress_slow_client_" +
    std::to_string(getpid()));
  auto client = client_node->create_client<SetParameters>("/" + node_name() + "/set_parameters");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto req = std::make_shared<SetParameters::Request>();
  req->parameters.push_back(rclcpp::Parameter("max_speed", 1.0).to_parameter_msg());
  auto pending_reply = client->async_send_request(req);
  const auto stuck_by = std::chrono::steady_clock::now() + 5s;
  while (wake_.calls.load() == 0 && std::chrono::steady_clock::now() < stuck_by) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(wake_.calls.load(), 0) << "the ingress thread never reached wake()";
  const size_t before = ing.admitted_count();

  auto late = node_->create_subscription<std_msgs::msg::Int32>(
    "/ingress_late_" + std::to_string(getpid()), 10, [](std_msgs::msg::Int32::SharedPtr) {});
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(ing.admit_external_subscription(11, late)) <<
    "admitted while the ingress thread was stuck";
  const auto waited = std::chrono::steady_clock::now() - t0;
  // Lower bound only: an upper bound on wall time fails under CI load when
  // the test thread is descheduled, although the admission behaved.
  EXPECT_GE(waited, 1900ms);

  hold.unlock();  // the thread runs again
  // A second admission returns only once the thread applied every request
  // queued before it, so the count below is final.
  auto next = node_->create_subscription<std_msgs::msg::Int32>(
    "/ingress_next_" + std::to_string(getpid()), 10, [](std_msgs::msg::Int32::SharedPtr) {});
  ASSERT_TRUE(ing.admit_external_subscription(12, next));
  EXPECT_EQ(ing.admitted_count(), before + 1) << "the admission that timed out and returned "
    "false was applied later anyway: " << ing.admitted_count() - before <<
    " entities joined, want 1";
  wake_.block_on = nullptr;
}

namespace
{
std::vector<std::string> g_errors;

void error_capture_handler(
  const rcutils_log_location_t *, int severity, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  if (severity != RCUTILS_LOG_SEVERITY_ERROR || std::string(name) != "ros2_lf_binding") {
    return;
  }
  char buf[1024];
  va_list copy;
  va_copy(copy, *args);
  vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_errors.emplace_back(buf);
}
}  // namespace

// Three wait failures in a row end the ingress thread for the
// rest of the run (later admissions refuse). That must be reported as an
// error, not only as three warnings.
TEST_F(IngressTest, ThreadStopAfterWaitFailuresIsReported)
{
  auto & ing = IngressRole::instance();
  g_errors.clear();
  rcutils_logging_set_output_handler(error_capture_handler);
  ing.fail_waits_for_test(3);
  ing.register_node("fails", &fake_wake, &wake_, node_->get_node_base_interface());
  const auto until = std::chrono::steady_clock::now() + 5s;
  while (ing.thread_running() && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(5ms);
  }
  const bool running = ing.thread_running();
  auto later = node_->create_subscription<std_msgs::msg::Int32>(
    "/ingress_after_stop_" + std::to_string(getpid()), 10,
    [](std_msgs::msg::Int32::SharedPtr) {});
  const bool admitted = ing.admit_external_subscription(13, later);
  rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  ASSERT_FALSE(running) << "three forced wait failures did not stop the ingress thread";
  EXPECT_FALSE(admitted);
  const std::string want =
    "rcllf ingress: thread stops after 3 consecutive wait failures; every later admission "
    "is refused";
  EXPECT_EQ(
    std::count_if(
      g_errors.begin(), g_errors.end(),
      [&want](const std::string & e) {return e.find(want) != std::string::npos;}), 1) <<
    "no error reports that the ingress thread stopped; errors seen: " << g_errors.size();
}

// A SIGINT during startup shuts rclcpp down before the first
// ingress registration (callback_group_example holds startup for the 5 s
// discovery timeout). Registration then ran on a dead context and the
// GuardCondition constructor threw out of the LF startup reaction (abort,
// exit -6). It must leave ingress stopped instead. A plain TEST, declared after
// the IngressTest suite, so it owns the context lifecycle.
TEST(IngressShutdownTest, RegisterAfterRclcppShutdownLeavesIngressStopped)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto & ing = IngressRole::instance();
  ing.reset_for_test();
  auto node = std::make_shared<rclcpp::Node>("ingress_shutdown_node");
  rclcpp::shutdown();
  FakeWake wake;
  try {
    ing.register_node("late", &fake_wake, &wake, node->get_node_base_interface());
  } catch (const std::exception & e) {
    FAIL() << "register_node threw after rclcpp shutdown: " << e.what();
  }
  EXPECT_FALSE(ing.thread_running());
  EXPECT_EQ(ing.admitted_count(), 0u);
  EXPECT_EQ(wake.calls.load(), 0);
  ing.reset_for_test();
}

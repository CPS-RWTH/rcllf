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

/// @file ingress_role_probe.cpp
/// @brief Regression test of the one-scheduler ingress role.
///
/// (a) Which node services can an rclcpp::WaitSet admit: services of a
///     component loaded through LFComponentManager, of a node never added to
///     an executor, and (negative control) of a node added to one.
/// (b) take on thread A, handle_request on thread B answers real ChangeState
///     and SetParameters clients.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../src/component_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace
{

// One node (and service) name per process, so concurrent runs never meet.
std::string lc_name() {return "probe_lc_node_" + std::to_string(getpid());}

std::vector<rclcpp::ServiceBase::SharedPtr> node_services(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base)
{
  std::vector<rclcpp::ServiceBase::SharedPtr> out;
  base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      group->collect_all_ptrs(
        [](const rclcpp::SubscriptionBase::SharedPtr &) {},
        [&out](const rclcpp::ServiceBase::SharedPtr & s) {out.push_back(s);},
        [](const rclcpp::ClientBase::SharedPtr &) {},
        [](const rclcpp::TimerBase::SharedPtr &) {},
        [](const rclcpp::Waitable::SharedPtr &) {});
    });
  return out;
}

/// Returns {admitted, refused} counts and prints each refusal.
std::pair<int, int> try_admit_all(const std::vector<rclcpp::ServiceBase::SharedPtr> & services)
{
  rclcpp::WaitSet ws;
  int admitted = 0;
  int refused = 0;
  for (const auto & s : services) {
    try {
      ws.add_service(s);
      ++admitted;
    } catch (const std::exception & e) {
      ++refused;
      std::printf("PROBE refused %s: %s\n", s->get_service_name(), e.what());
    }
  }
  return {admitted, refused};
}

class ProbeEnv : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

}  // namespace

namespace
{
/// Measured remedy candidate: a manager that never hands a loaded component
/// to the executor (rclcpp_components' on_load_node calls the virtual
/// add_node_to_executor, which add_node()s it and claims its services).
class NoExecutorManager : public ros2_lf_binding::LFComponentManager
{
public:
  using LFComponentManager::LFComponentManager;
  void add_node_to_executor(uint64_t /*node_id*/) override {}
};

template<class Manager>
std::pair<int, int> load_and_admit(const std::string & name, size_t & total)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::weak_ptr<rclcpp::Executor> weak_exec = exec;
  auto manager = std::make_shared<Manager>(weak_exec, name + "_manager");
  EXPECT_TRUE(
    manager->load_component(
      "tf2_ros", "tf2_ros::StaticTransformBroadcasterNode", name, *exec, name, ""));
  auto comps = manager->get_all_components();
  EXPECT_EQ(comps.size(), 1u);
  if (comps.size() != 1u) {
    return {0, 0};
  }
  auto services = node_services(comps[0].get_node_base_interface());
  total = services.size();
  return try_admit_all(services);
}
}  // namespace

// Measured 2026-09-24 before the no-op override: LFComponentManager with a live
// executor weak_ptr claimed every component service (7 of 7 refused). Since
// then it overrides add_node_to_executor as a no-op, so none is refused.
TEST(IngressRoleProbe, LFComponentManagerComponentServicesAdmissible)
{
  size_t total = 0;
  auto [admitted, refused] =
    load_and_admit<ros2_lf_binding::LFComponentManager>("probe_comp_claimed", total);
  std::printf(
    "PROBE LFComponentManager component services: %zu total, %d admitted, %d refused\n", total,
    admitted, refused);
  EXPECT_GT(total, 0u);
  EXPECT_EQ(refused, 0);
}

TEST(IngressRoleProbe, ComponentServicesAdmissibleWithoutAddNodeToExecutor)
{
  size_t total = 0;
  auto [admitted, refused] = load_and_admit<NoExecutorManager>("probe_comp_free", total);
  std::printf(
    "PROBE no-op add_node_to_executor component services: %zu total, %d admitted, %d refused\n",
    total, admitted, refused);
  EXPECT_GT(total, 0u);
  EXPECT_EQ(refused, 0);
  EXPECT_EQ(static_cast<size_t>(admitted), total);
}

TEST(IngressRoleProbe, NeverAddedNodeAdmissibleAddedNodeRefused)
{
  auto free_node = std::make_shared<rclcpp::Node>("probe_free_node");
  auto [a1, r1] = try_admit_all(node_services(free_node->get_node_base_interface()));
  std::printf("PROBE never-added node: %d admitted, %d refused\n", a1, r1);
  EXPECT_EQ(r1, 0);
  EXPECT_GT(a1, 0);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto added_node = std::make_shared<rclcpp::Node>("probe_added_node");
  exec->add_node(added_node);
  auto [a2, r2] = try_admit_all(node_services(added_node->get_node_base_interface()));
  std::printf("PROBE executor-added node: %d admitted, %d refused\n", a2, r2);
  EXPECT_GT(r2, 0);
}

TEST(IngressRoleProbe, TakeOnAHandleOnB)
{
  auto lc = std::make_shared<rclcpp_lifecycle::LifecycleNode>(lc_name());
  std::thread::id configure_thread;
  lc->register_on_configure(
    [&configure_thread](const rclcpp_lifecycle::State &) {
      configure_thread = std::this_thread::get_id();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    });
  lc->declare_parameter("max_speed", 1.0);
  std::thread::id on_set_thread;
  auto handle = lc->add_on_set_parameters_callback(
    [&on_set_thread](const std::vector<rclcpp::Parameter> &) {
      on_set_thread = std::this_thread::get_id();
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    });

  auto services = node_services(lc->get_node_base_interface());
  rclcpp::WaitSet ws;
  for (const auto & s : services) {
    ws.add_service(s);
  }

  struct Rec
  {
    rclcpp::ServiceBase::SharedPtr svc;
    std::shared_ptr<rmw_request_id_t> hdr;
    std::shared_ptr<void> req;
  };
  std::mutex mu;
  std::condition_variable cv;
  std::deque<Rec> mailbox;
  std::atomic<bool> stop{false};
  std::thread::id ingress_id;
  std::thread::id worker_id;

  std::thread ingress([&]() {
      ingress_id = std::this_thread::get_id();
      while (!stop.load()) {
        auto res = ws.wait(100ms);
        if (res.kind() != rclcpp::WaitResultKind::Ready) {
          continue;
        }
        for (const auto & s : services) {
          auto req = s->create_request();
          auto hdr = s->create_request_header();
          if (s->take_type_erased_request(req.get(), *hdr)) {
            std::lock_guard<std::mutex> lk(mu);
            mailbox.push_back({s, hdr, req});
            cv.notify_one();
          }
        }
      }
    });
  std::thread worker([&]() {
      worker_id = std::this_thread::get_id();
      while (!stop.load()) {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait_for(lk, 100ms, [&]() {return !mailbox.empty();});
        if (mailbox.empty()) {
          continue;
        }
        Rec r = mailbox.front();
        mailbox.pop_front();
        lk.unlock();
        r.svc->handle_request(r.hdr, r.req);
      }
    });

  struct Joiner
  {
    std::atomic<bool> & stop;
    std::thread & a;
    std::thread & b;
    ~Joiner()
    {
      stop.store(true);
      if (a.joinable()) {a.join();}
      if (b.joinable()) {b.join();}
    }
  } joiner{stop, ingress, worker};

  auto client_node = std::make_shared<rclcpp::Node>("probe_client_node");
  auto cs = client_node->create_client<lifecycle_msgs::srv::ChangeState>(
    "/" + lc_name() + "/change_state");
  auto sp = client_node->create_client<rcl_interfaces::srv::SetParameters>(
    "/" + lc_name() + "/set_parameters");
  ASSERT_TRUE(cs->wait_for_service(5s));
  ASSERT_TRUE(sp->wait_for_service(5s));

  auto cs_req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  cs_req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  auto cs_fut = cs->async_send_request(cs_req);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, cs_fut, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(cs_fut.get()->success);

  auto sp_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  sp_req->parameters.push_back(rclcpp::Parameter("max_speed", 2.5).to_parameter_msg());
  auto sp_fut = sp->async_send_request(sp_req);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, sp_fut, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto sp_resp = sp_fut.get();
  ASSERT_EQ(sp_resp->results.size(), 1u);
  EXPECT_TRUE(sp_resp->results[0].successful);

  stop.store(true);
  ingress.join();
  worker.join();
  EXPECT_EQ(configure_thread, worker_id);
  EXPECT_EQ(on_set_thread, worker_id);
  EXPECT_NE(configure_thread, ingress_id);
  EXPECT_EQ(lc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(lc->get_parameter("max_speed").as_double(), 2.5);
  std::printf(
    "PROBE take-on-A/handle-on-B: change_state success, set_parameters success, "
    "state=%s, callbacks on worker thread\n", lc->get_current_state().label().c_str());
}

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new ProbeEnv);
  return RUN_ALL_TESTS();
}

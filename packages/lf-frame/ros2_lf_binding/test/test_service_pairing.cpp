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

/// Review round 5, M2: a request of an outside client is told from an
/// in-system one by its header GID, a response is delivered only for a
/// sequence number the client sent (seen by the rcl_send_request gate), and
/// a second server of an in-system service is refused (RCLLF-R004).

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rcl/graph.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "service_pairing.hpp"

namespace sp = ros2_lf_binding::service_pairing;
using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

class ServicePairing : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {sp::reset_for_test();}
  void TearDown() override {sp::reset_for_test();}
};

namespace
{
void noop(const AddTwoInts::Request::SharedPtr, AddTwoInts::Response::SharedPtr) {}

std::shared_ptr<rmw_request_id_t> take_one(const rclcpp::Service<AddTwoInts>::SharedPtr & srv)
{
  for (int i = 0; i < 300; ++i) {
    auto req = srv->create_request();
    auto hdr = srv->create_request_header();
    if (srv->take_type_erased_request(req.get(), *hdr)) {
      return hdr;
    }
    std::this_thread::sleep_for(10ms);
  }
  return nullptr;
}

std::shared_ptr<rmw_request_id_t> take_response(const rclcpp::Client<AddTwoInts>::SharedPtr & cli)
{
  for (int i = 0; i < 300; ++i) {
    auto resp = cli->create_response();
    auto hdr = cli->create_request_header();
    if (cli->take_type_erased_response(resp.get(), *hdr)) {
      return hdr;
    }
    std::this_thread::sleep_for(10ms);
  }
  return nullptr;
}
}  // namespace

TEST_F(ServicePairing, RequestHeaderGidTellsInSystemFromOutside)
{
  auto node = std::make_shared<rclcpp::Node>("pairing_gid");
  auto srv = node->create_service<AddTwoInts>("pair_gid", &noop);
  auto inside = node->create_client<AddTwoInts>("pair_gid");
  auto outside = node->create_client<AddTwoInts>("pair_gid");
  ASSERT_TRUE(inside->wait_for_service(5s));
  ASSERT_TRUE(outside->wait_for_service(5s));
  sp::register_client(0, inside);
  const std::string name = srv->get_service_name();

  auto f1 = inside->async_send_request(std::make_shared<AddTwoInts::Request>());
  auto h1 = take_one(srv);
  ASSERT_NE(h1, nullptr);
  EXPECT_FALSE(sp::is_outside_request(name, *h1)) << "in-system request taken as outside";

  auto f2 = outside->async_send_request(std::make_shared<AddTwoInts::Request>());
  auto h2 = take_one(srv);
  ASSERT_NE(h2, nullptr);
  EXPECT_TRUE(sp::is_outside_request(name, *h2)) << "outside request taken as in-system";

  auto f3 = inside->async_send_request(std::make_shared<AddTwoInts::Request>());
  auto h3 = take_one(srv);
  ASSERT_NE(h3, nullptr);
  EXPECT_FALSE(sp::is_outside_request(name, *h3)) << "second in-system request taken as outside";
  inside->remove_pending_request(f3);
  inside->remove_pending_request(f1);
  outside->remove_pending_request(f2);
}

TEST_F(ServicePairing, GateRecordsSequenceAndThreadCount)
{
  auto node = std::make_shared<rclcpp::Node>("pairing_seq");
  auto srv = node->create_service<AddTwoInts>("pair_seq", &noop);
  auto cli = node->create_client<AddTwoInts>("pair_seq");
  ASSERT_TRUE(cli->wait_for_service(5s));
  const void * handle = cli->get_client_handle().get();

  auto f1 = cli->async_send_request(std::make_shared<AddTwoInts::Request>());
  auto f2 = cli->async_send_request(std::make_shared<AddTwoInts::Request>());
  // O1-2: the sender reaction raises its port once per request really sent.
  EXPECT_EQ(ros2_lf_service_sent(0, "pair_seq"), 2);
  EXPECT_EQ(ros2_lf_service_sent(0, "pair_seq"), 0);
  EXPECT_EQ(ros2_lf_service_sent(0, "other"), 0);

  // The server answers both; each response's sequence number is expected
  // exactly once, a replayed one is foreign.
  for (int i = 0; i < 2; ++i) {
    auto hdr = take_one(srv);
    ASSERT_NE(hdr, nullptr);
    AddTwoInts::Response resp;
    srv->send_response(*hdr, resp);
  }
  auto r1 = take_response(cli);
  ASSERT_NE(r1, nullptr);
  EXPECT_TRUE(sp::take_expected(handle, r1->sequence_number));
  EXPECT_FALSE(sp::take_expected(handle, r1->sequence_number)) << "replayed response delivered";
  auto r2 = take_response(cli);
  ASSERT_NE(r2, nullptr);
  EXPECT_TRUE(sp::take_expected(handle, r2->sequence_number));
  EXPECT_FALSE(sp::take_expected(handle, 999999));
  cli->remove_pending_request(f1);
  cli->remove_pending_request(f2);
}

TEST_F(ServicePairing, SecondServerRuleAndNames)
{
  EXPECT_TRUE(sp::second_server(2, 1));
  EXPECT_FALSE(sp::second_server(1, 1));
  EXPECT_FALSE(sp::second_server(3, 0));  // nothing in-system bound here
  EXPECT_TRUE(sp::names_service("/ns/compute", "compute"));
  EXPECT_TRUE(sp::names_service("/compute", "/compute"));
  EXPECT_FALSE(sp::names_service("/recompute", "compute"));
}

TEST_F(ServicePairing, OutsideSecondServerRefusedR004)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
  {
    auto node = std::make_shared<rclcpp::Node>("pairing_r004");
    auto mine = node->create_service<AddTwoInts>("pair_r004", &noop);
    auto other_node = std::make_shared<rclcpp::Node>("pairing_r004_outside");
    auto other = other_node->create_service<AddTwoInts>("pair_r004", &noop);
    size_t count = 0;
    for (int i = 0; i < 300 && count < 2; ++i) {
      rcl_count_services(
        node->get_node_base_interface()->get_rcl_node_handle(),
        mine->get_service_name(), &count);
      std::this_thread::sleep_for(10ms);
    }
    sp::register_server(0, mine, node->get_node_base_interface()->get_shared_rcl_node_handle());
    std::exit(0);
  }, ::testing::ExitedWithCode(EXIT_FAILURE), "RCLLF-R004");
}

TEST_F(ServicePairing, SingleServerAccepted)
{
  auto node = std::make_shared<rclcpp::Node>("pairing_single");
  auto mine = node->create_service<AddTwoInts>("pair_single", &noop);
  sp::register_server(0, mine, node->get_node_base_interface()->get_shared_rcl_node_handle());
  SUCCEED();  // register_server would have exited on a finding
}

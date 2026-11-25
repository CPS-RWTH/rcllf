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

/// @file publication_gate_probe.cpp
/// @brief Standing probe for the executable-level publication gate (T0).
///
/// The generated executable defines rcl_publish (and its siblings) and
/// forwards each call through dlsym(RTLD_NEXT). This probe does the same in a
/// plain gtest executable and asserts the three facts correspondence rests on:
///   1. the executable's definitions preempt librcl for every gated symbol,
///      so librclcpp's PLT calls land in the gate;
///   2. the gate sees exactly the user's publishes on a publisher, in order,
///      and the reader's publication_sequence_number equals the gate ordinal;
///   3. a publish from a node constructor, before any tracking, is counted.
/// If preemption breaks (link order, LTO, -Bsymbolic) it fails with
/// "gate not on the rcl_publish path".

#include <dlfcn.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rcl/publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{
std::mutex g_mu;
std::map<const rcl_publisher_t *, uint64_t> g_publish;       // ordinals per publisher
std::atomic<uint64_t> g_serialized{0};
std::atomic<uint64_t> g_fini{0};
std::atomic<uint64_t> g_init{0};

template<typename Fn>
Fn next_symbol(const char * name)
{
  return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

uint64_t publishes_on(const rcl_publisher_t * p)
{
  std::lock_guard<std::mutex> lk(g_mu);
  auto it = g_publish.find(p);
  return it == g_publish.end() ? 0 : it->second;
}
}  // namespace

extern "C" {

rcl_ret_t rcl_publisher_init(
  rcl_publisher_t * publisher, const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support, const char * topic_name,
  const rcl_publisher_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *,
    const char *, const rcl_publisher_options_t *);
  static Fn next = next_symbol<Fn>("rcl_publisher_init");
  g_init.fetch_add(1);
  return next(publisher, node, type_support, topic_name, options);
}

rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher, const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish");
  {
    std::lock_guard<std::mutex> lk(g_mu);
    ++g_publish[publisher];
  }
  return next(publisher, ros_message, allocation);
}

rcl_ret_t rcl_publish_serialized_message(
  const rcl_publisher_t * publisher, const rcl_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(
    const rcl_publisher_t *, const rcl_serialized_message_t *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish_serialized_message");
  g_serialized.fetch_add(1);
  {
    std::lock_guard<std::mutex> lk(g_mu);
    ++g_publish[publisher];
  }
  return next(publisher, serialized_message, allocation);
}

rcl_ret_t rcl_publish_loaned_message(
  const rcl_publisher_t * publisher, void * ros_message, rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(const rcl_publisher_t *, void *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish_loaned_message");
  {
    std::lock_guard<std::mutex> lk(g_mu);
    ++g_publish[publisher];
  }
  return next(publisher, ros_message, allocation);
}

rcl_ret_t rcl_publisher_fini(rcl_publisher_t * publisher, rcl_node_t * node)
{
  using Fn = rcl_ret_t (*)(rcl_publisher_t *, rcl_node_t *);
  static Fn next = next_symbol<Fn>("rcl_publisher_fini");
  g_fini.fetch_add(1);
  return next(publisher, node);
}

}  // extern "C"

namespace
{

// One topic per process, so concurrent runs in one domain never meet.
std::string probe_topic() {return "/gate_probe_" + std::to_string(getpid());}

class ConstructorPublisher : public rclcpp::Node
{
public:
  ConstructorPublisher()
  : Node("gate_probe_pub")
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(probe_topic(), rclcpp::QoS(10));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std_msgs::msg::Int32 m;
    m.data = 100;  // the constructor publish, before any tracking decision
    pub_->publish(m);
  }
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

class GateProbe : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(GateProbe, ExecutableDefinitionsPreemptLibrcl)
{
  const std::vector<std::pair<const char *, void *>> gated = {
    {"rcl_publish", reinterpret_cast<void *>(&rcl_publish)},
    {"rcl_publish_serialized_message", reinterpret_cast<void *>(&rcl_publish_serialized_message)},
    {"rcl_publish_loaned_message", reinterpret_cast<void *>(&rcl_publish_loaned_message)},
    {"rcl_publisher_fini", reinterpret_cast<void *>(&rcl_publisher_fini)},
    {"rcl_publisher_init", reinterpret_cast<void *>(&rcl_publisher_init)},
  };
  for (const auto & [name, gate] : gated) {
    EXPECT_EQ(dlsym(RTLD_DEFAULT, name), gate) << "gate not on the rcl_publish path: " << name;
    EXPECT_NE(dlsym(RTLD_NEXT, name), nullptr) << "no next definition for " << name;
  }
}

TEST_F(GateProbe, OrdinalsEqualReaderSequenceIncludingConstructorPublish)
{
  auto sub_node = std::make_shared<rclcpp::Node>("gate_probe_sub");
  std::mutex rx_mu;
  std::vector<std::pair<int, uint64_t>> received;  // (payload, publication_sequence_number)
  auto sub = sub_node->create_subscription<std_msgs::msg::Int32>(
    probe_topic(), rclcpp::QoS(10),
    [&](const std_msgs::msg::Int32 & m, const rclcpp::MessageInfo & info) {
      std::lock_guard<std::mutex> lk(rx_mu);
      received.emplace_back(m.data, info.get_rmw_message_info().publication_sequence_number);
    });
  const uint64_t init_before = g_init.load();
  auto pub_node = std::make_shared<ConstructorPublisher>();
  ASSERT_GT(g_init.load(), init_before) << "gate not on the rcl_publish path: rcl_publisher_init";
  const rcl_publisher_t * handle = pub_node->pub_->get_publisher_handle().get();
  ASSERT_EQ(publishes_on(handle), 1u)
    << "gate not on the rcl_publish path: constructor publish not counted";

  for (int i = 1; i <= 3; ++i) {
    std_msgs::msg::Int32 m;
    m.data = i;
    pub_node->pub_->publish(m);
  }
  // One serialized publish on the same writer: gated by its own symbol.
  rclcpp::Serialization<std_msgs::msg::Int32> ser;
  rclcpp::SerializedMessage raw;
  std_msgs::msg::Int32 m4;
  m4.data = 4;
  ser.serialize_message(&m4, &raw);
  pub_node->pub_->publish(raw);
  EXPECT_EQ(g_serialized.load(), 1u) << "gate not on the rcl_publish path: serialized";
  EXPECT_EQ(publishes_on(handle), 5u) << "gate not on the rcl_publish path: count";

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lk(rx_mu);
    if (received.size() >= 5) {
      break;
    }
  }
  std::lock_guard<std::mutex> lk(rx_mu);
  ASSERT_EQ(received.size(), 5u);
  const std::vector<int> payloads = {100, 1, 2, 3, 4};
  for (size_t k = 0; k < received.size(); ++k) {
    EXPECT_EQ(received[k].first, payloads[k]);
    EXPECT_EQ(received[k].second, k + 1) << "reader sequence != gate ordinal at " << k;
  }

  const uint64_t fini_before = g_fini.load();
  pub_node.reset();
  EXPECT_GT(g_fini.load(), fini_before) << "gate not on the rcl_publish path: rcl_publisher_fini";
}

}  // namespace

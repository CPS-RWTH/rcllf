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

/// @file test_signal_end_count.cpp
/// @brief A federate ended by SIGINT still delivers its
/// end-of-run COMPLETE count and ends without an abort.
///
/// A one-host loss-cell smoke on the desk host ended both federates with
/// the runner's SIGINT. rclcpp's signal thread held the context only until
/// the drains settled; the count exchange that follows ran on a dead
/// context, rclcpp dropped the publish silently, each peer waited 1500 ms
/// and failed complete_end_missing, and the acknowledgement wait threw
/// RCLError ("publisher's context is invalid"): terminate, exit -6.
///
/// The forked child stands in for a federate with one egress edge: it binds
/// the edge, makes the end channel, gets SIGINT while "LF runs" and then runs
/// the teardown that reactor-c's atexit call runs. The parent is the peer: it
/// takes the child's count from the end topic.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "corr_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "ros2_lf_msgs/msg/corr_end.hpp"
#include "std_msgs/msg/int32.hpp"

namespace ros2_lf_binding
{
void lf_mark_running();  // runtime.cpp
}  // namespace ros2_lf_binding

namespace
{
std::atomic<bool> g_stop_requested{false};
constexpr int kCapMs = 20000;
}  // namespace

// The LF program's lf_request_stop, which the binding resolves with dlsym
// (the test links with -rdynamic). LF "stops" at once; the test thread then
// runs the teardown as reactor-c's atexit call would.
extern "C" void lf_request_stop(void)
{
  g_stop_requested.store(true);
}

namespace
{

[[noreturn]] void run_federate_child(int global_edge, const std::string & err_path)
{
  const int fd = open(err_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0 || dup2(fd, STDERR_FILENO) < 0) {
    _exit(2);
  }
  char arg0[] = "test_signal_end_count";
  char * argv[] = {arg0, nullptr};
  const std::string system_json = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_end_count_system.json";
  std::ofstream(system_json) << "{}\n";
  setenv("SYSTEM_JSON", system_json.c_str(), 1);
  ros2_lf_runtime_init(1, argv);

  // One egress edge: the publisher is here, its subscriber in the peer.
  const std::string topic = "/sigint_end_count_" + std::to_string(getpid());
  auto pub_node = std::make_shared<rclcpp::Node>("sigint_pub_" + std::to_string(getpid()));
  auto pub = pub_node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(10));
  ros2_lf_binding::CorrEdgeSpec s{0, "Pub", {0}, topic, "", -1, topic, 0};
  s.role = ros2_lf_binding::CorrEdgeRole::kEgress;
  s.global_id = global_edge;
  s.peer_node = "/sigint_peer";
  ros2_lf_binding::register_corr_edge(s);
  const auto f = ros2_lf_binding::corr_rt::startup(
    [&](const std::string &) {return pub_node->get_node_base_interface();});
  if (f) {
    std::fprintf(stderr, "startup failed: %s: %s\n", f->check.c_str(), f->detail.c_str());
    _exit(4);
  }
  ros2_lf_binding::corr_rt::complete_end_channel_init();

  // Wait until the peer's reader of the end topic is discovered, so the
  // count has a matched reader when it is published.
  auto probe = std::make_shared<rclcpp::Node>("sigint_probe_" + std::to_string(getpid()));
  const auto t0 = std::chrono::steady_clock::now();
  while (probe->count_subscribers("/rcllf/corr_end") == 0) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) {
      std::fprintf(stderr, "the peer's end reader was never discovered\n");
      _exit(5);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  probe.reset();

  // LF runs (rclcpp's signal handlers re-installed); the
  // runner's SIGINT arrives.
  ros2_lf_binding::lf_mark_running();
  std::raise(SIGINT);
  while (!g_stop_requested.load()) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(15)) {
      std::fprintf(stderr, "the signal never requested an LF stop\n");
      _exit(6);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // LF returned: reactor-c's atexit call. The test's entities go first, as
  // the generated program's nodes are in the node registry.
  pub.reset();
  pub_node.reset();
  ros2_lf_runtime_shutdown();
  std::fflush(stderr);
  _exit(7);
}

std::string slurp(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

TEST(SignalEndCount, ASigintEndedFederateDeliversItsCountWithoutAnAbort)
{
  const std::string err_path = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_end_count.err";
  // Unique per run: concurrent test runs share the end topic of the domain.
  const int global_edge = 100000 + static_cast<int>(getpid() % 100000);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    run_federate_child(global_edge, err_path);
  }
  // The peer, after the fork: the child must not inherit DDS threads.
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("sigint_peer_" + std::to_string(getpid()));
  std::atomic<bool> got{false};
  auto sub = node->create_subscription<ros2_lf_msgs::msg::CorrEnd>(
    "/rcllf/corr_end", rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local(),
    [&](const ros2_lf_msgs::msg::CorrEnd & m) {
      if (m.global_edge == global_edge) {
        got.store(true);
      }
    });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  int status = 0;
  bool ended = false;
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(kCapMs)) {
    ex.spin_some(std::chrono::milliseconds(10));
    if (!ended && waitpid(child, &status, WNOHANG) == child) {
      ended = true;
    }
    if (ended && got.load()) {
      break;
    }
    if (ended && std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(kCapMs / 2)) {
      break;  // the child is gone; a count still missing will not come
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!ended) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }
  ex.remove_node(node);
  sub.reset();
  node.reset();
  rclcpp::shutdown();
  const std::string err = slurp(err_path);
  ASSERT_TRUE(ended) << "the SIGINT-ended federate stand-in still ran after " << kCapMs <<
    " ms\n--- child stderr ---\n" << err;
  EXPECT_TRUE(got.load()) <<
    "the peer never got the end-of-run COMPLETE count of global edge " << global_edge <<
    " (complete_end_missing on the receiving federate)\n--- child stderr ---\n" << err;
  ASSERT_FALSE(WIFSIGNALED(status)) << "the federate stand-in died by signal " <<
    WTERMSIG(status) << "\n--- child stderr ---\n" << err;
  EXPECT_EQ(WEXITSTATUS(status), 7) << "--- child stderr ---\n" << err;
  EXPECT_EQ(err.find("context is invalid"), std::string::npos) << err;
}

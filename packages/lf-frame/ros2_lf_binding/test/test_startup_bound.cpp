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

/// @file test_startup_bound.cpp
/// @brief The startup bound: a start stuck in node construction ends.
///
/// Under 16 parallel starts, the graph-update publish of the manager node's
/// construction (rclcpp::Node::Node -> ... -> rmw_fastrtps_shared_cpp::
/// __rmw_publish -> SharedMemTransport::send) once waited on a Fast DDS
/// shared-memory port lock for the whole 60 s of the test loop, before any
/// bound of the binding was armed. This test makes that publish block forever
/// on the main thread of a forked child: the executable defines
/// __rmw_publish itself (it links with -rdynamic, so the rmw library's call
/// binds to this definition), and the first call on the main thread never
/// returns. The child must still end, at the startup bound, with code 124 and
/// the bound's message. A healthy start with the same bound must not be cut
/// off once it reached the discovery wait.
///
/// Round 3: the watchdog and a stop-bound arming decide who ends a stuck start
/// through one atomic exchange. The test defines the watchdog's weak expiry
/// hook (runtime_internal.hpp) to send SIGINT and wait until the stop bound is
/// armed, either before the watchdog commits to its exit (the stop bound must
/// govern) or after it did (the startup bound's exit stands).

#include <gtest/gtest.h>

#include <dlfcn.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include "rmw/types.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "runtime_internal.hpp"

namespace
{

constexpr int kCapMs = 15000;  // a hang, not a slow host: the bound is 1000 ms
constexpr int kBoundMs = 1000;
// The latest end a working bound allows: the bound, then at most
// kStartupExitWriteBoundMs (runtime.cpp, 1000 ms) for the expiry's message
// before its SIGALRM backstop, plus 1000 ms for the fork, the 20 ms poll and
// a loaded CI host's scheduling. An end three times the bound or later is not
// this bound's.
constexpr int kBoundEndSlackMs = 2000;
// A SIGINT this long before the startup bound expires.
constexpr int kSignalBeforeBoundMs = 200;
constexpr int kStopBoundMs = 2000;
constexpr char kStopBoundMessage[] =
  "rcllf: process end bounded to 2000 ms (RCLLF_STOP_TIMEOUT_MS)";
constexpr int kStopTimeoutExitCode = 124;  // runtime.cpp, documented
constexpr char kStallMarker[] = "TEST795_STALL main-thread __rmw_publish blocked";
constexpr char kBoundMessage[] =
  "rcllf: startup did not reach the discovery wait within 1000 ms (RCLLF_STARTUP_TIMEOUT_MS)";

std::atomic<bool> g_stall_main_publish{false};
// The watchdog stage (0 before its exit decision, 1 after it)
// at which the expiry hook arms the stop bound; -1: the hook does nothing.
std::atomic<int> g_arm_stop_at_stage{-1};
constexpr int kStopArmWaitMs = 3000;

void say(const char * line)
{
  [[maybe_unused]] const ssize_t w = write(STDERR_FILENO, line, std::strlen(line));
}

}  // namespace

// Runs on the startup watchdog after its wait expired. At the chosen stage it
// sends SIGINT and waits until rclcpp's signal thread has armed the stop bound
// (the pre-shutdown callback), so the arming lands inside the window for sure.
extern "C" void rcllf_test_startup_expiry_hook(int stage)
{
  if (stage != g_arm_stop_at_stage.load()) {
    return;
  }
  say(stage == 0 ? "TEST795_HOOK before the exit decision\n" :
    "TEST795_HOOK after the exit decision\n");
  kill(getpid(), SIGINT);
  const auto t0 = std::chrono::steady_clock::now();
  while (!ros2_lf_binding::stop_bound_armed()) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(kStopArmWaitMs)) {
      say("TEST795_STOP_NOT_ARMED\n");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  say("TEST795_STOP_ARMED\n");
}

// The rmw_fastrtps graph-update publish, the frame the dump stopped in.
namespace rmw_fastrtps_shared_cpp
{
rmw_ret_t __rmw_publish(
                          // upstream's name
  const char * identifier, const rmw_publisher_t * publisher, const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  using Fn = rmw_ret_t (*)(
    const char *, const rmw_publisher_t *, const void *, rmw_publisher_allocation_t *);
  // The rmw library is dlopen'ed RTLD_LOCAL by rmw_implementation, so
  // RTLD_NEXT does not see it: look the real function up in that library.
  static const Fn real = []() {
      void * lib = dlopen("librmw_fastrtps_shared_cpp.so", RTLD_LAZY | RTLD_NOLOAD);
      return reinterpret_cast<Fn>(dlsym(
               lib, "_ZN23rmw_fastrtps_shared_cpp13__rmw_publishEPKcPK15rmw_publisher_sPKvP26"
               "rmw_publisher_allocation_s"));
    }();
  // The thread first: a background publish must not consume the flag.
  if (syscall(SYS_gettid) == getpid() && g_stall_main_publish.exchange(false)) {
    // As the shared-memory port lock that is never released.
    [[maybe_unused]] const ssize_t w = write(
      STDERR_FILENO, kStallMarker, sizeof(kStallMarker) - 1);
    [[maybe_unused]] const ssize_t nl = write(STDERR_FILENO, "\n", 1);
    for (;; ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  if (real == nullptr) {
    std::fputs("TEST795_NO_REAL __rmw_publish not found\n", stderr);
    std::abort();
  }
  return real(identifier, publisher, ros_message, allocation);
}
}  // namespace rmw_fastrtps_shared_cpp

namespace
{

// Forks a child that runs `body` with stdout and stderr on a pipe. Waits up to
// kCapMs, SIGKILLs it on the cap. Returns the wait status (-1: killed at the
// cap), the elapsed time and the child's output.
template<typename Body>
int run_child(Body body, std::chrono::milliseconds * took, std::string * output)
{
  int fds[2];
  if (pipe(fds) != 0) {
    return -2;
  }
  const pid_t pid = fork();
  if (pid == 0) {
    close(fds[0]);
    dup2(fds[1], STDOUT_FILENO);
    dup2(fds[1], STDERR_FILENO);
    body();
    _exit(0);
  }
  close(fds[1]);
  fcntl(fds[0], F_SETFL, O_NONBLOCK);
  const auto drain = [&]() {
      char buf[4096];
      ssize_t n;
      while ((n = read(fds[0], buf, sizeof(buf))) > 0) {
        output->append(buf, static_cast<size_t>(n));
      }
    };
  const auto t0 = std::chrono::steady_clock::now();
  int status = 0;
  for (;; ) {
    drain();
    if (waitpid(pid, &status, WNOHANG) == pid) {
      break;
    }
    if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(kCapMs)) {
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      status = -1;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  *took = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0);
  drain();
  close(fds[0]);
  return status;
}

void set_startup_env()
{
  const std::string system_json = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_startup_bound_system.json";
  std::ofstream(system_json) << "{}\n";
  setenv("SYSTEM_JSON", system_json.c_str(), 1);
  setenv("RCLLF_STARTUP_TIMEOUT_MS", std::to_string(kBoundMs).c_str(), 1);
  unsetenv("RCLLF_EXIT_WITH_PARENT");
}

// The stall stands in for a Fast DDS publish: pin the RMW when it is unset
// (the build's default may be another one), skip when another one is set.
bool fastrtps_rmw()
{
  const char * rmw = std::getenv("RMW_IMPLEMENTATION");
  if (rmw == nullptr || *rmw == '\0') {
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    return true;
  }
  return std::string(rmw) == "rmw_fastrtps_cpp";
}

}  // namespace

TEST(StartupBound, ANodeConstructionStuckInThePublishEndsAtTheBound)
{
  if (!fastrtps_rmw()) {
    GTEST_SKIP() << "the stall stands in for a Fast DDS publish; RMW is not rmw_fastrtps_cpp";
  }
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      g_stall_main_publish.store(true);
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);  // the manager node's construction stalls
      _exit(0);  // not reached when the stall took
    }, &took, &out);
  ASSERT_NE(out.find(kStallMarker), std::string::npos)
    << "the stall did not take (the publish was not interposed); child output:\n" << out;
  ASSERT_NE(status, -1)
    << "the start stuck in node construction still ran " << kCapMs << " ms: the startup "
    "is not bounded; child output:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode) << out;
  EXPECT_GE(took.count(), kBoundMs - 100) << "ended before the bound";
  EXPECT_LE(took.count(), kBoundMs + kBoundEndSlackMs) << "ended long after the bound";
  EXPECT_NE(out.find(kBoundMessage), std::string::npos) << out;
}

// A SIGINT 200 ms before the startup bound expires starts a
// stop whose own bound (RCLLF_STOP_TIMEOUT_MS) then governs the process end.
// The startup bound must not cut that stop off 200 ms later with its message.
// The start stays stuck, so the stop bound ends it: code 124 at the signal
// plus the stop bound, without the startup bound's message.
TEST(StartupBound, ASignalBeforeTheBoundHandsTheStuckStartToTheStopBound)
{
  if (!fastrtps_rmw()) {
    GTEST_SKIP() << "the stall stands in for a Fast DDS publish; RMW is not rmw_fastrtps_cpp";
  }
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      setenv("RCLLF_STOP_TIMEOUT_MS", std::to_string(kStopBoundMs).c_str(), 1);
      unsetenv("RCLLF_STOP_MAX_MS");
      g_stall_main_publish.store(true);
      // Started before the bound is armed, so the signal comes at most
      // kSignalBeforeBoundMs before its expiry.
      std::thread(
        []() {
          std::this_thread::sleep_for(std::chrono::milliseconds(kBoundMs - kSignalBeforeBoundMs));
          [[maybe_unused]] const ssize_t w = write(STDERR_FILENO, "TEST795_SIGINT\n", 15);
          kill(getpid(), SIGINT);
        }).detach();
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);  // the manager node's construction stalls
      _exit(0);  // not reached when the stall took
    }, &took, &out);
  const size_t stall = out.find(kStallMarker);
  const size_t sigint = out.find("TEST795_SIGINT");
  ASSERT_NE(stall, std::string::npos)
    << "the stall did not take (the publish was not interposed); child output:\n" << out;
  ASSERT_NE(sigint, std::string::npos) << out;
  ASSERT_LT(stall, sigint) << "the signal came before the stall:\n" << out;
  ASSERT_NE(status, -1) << "the stuck start ran " << kCapMs << " ms after a SIGINT:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_EQ(out.find("startup did not reach the discovery wait"), std::string::npos)
    << "the startup bound ended a stop the signal had started; the stop bound must govern "
    "it:\n" << out;
  EXPECT_NE(out.find(kStopBoundMessage), std::string::npos) << out;
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode) << out;
  // The stop bound runs from the signal, not from the start.
  EXPECT_GE(took.count(), kBoundMs - kSignalBeforeBoundMs + kStopBoundMs - 100) << out;
  EXPECT_LE(took.count(), kBoundMs - kSignalBeforeBoundMs + kStopBoundMs + kBoundEndSlackMs)
    << out;
}

// The watchdog's expiry and a stop-bound arming race. The
// hook arms the stop bound after the watchdog's wait expired but before it
// committed to its exit (an earlier revision: after it read the stop bound as not
// armed). The stop bound must govern: code 124 at the signal plus the stop
// bound, without the startup bound's message.
TEST(StartupBound, AStopArmedBeforeTheExitDecisionGovernsTheStuckStart)
{
  if (!fastrtps_rmw()) {
    GTEST_SKIP() << "the stall stands in for a Fast DDS publish; RMW is not rmw_fastrtps_cpp";
  }
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      setenv("RCLLF_STOP_TIMEOUT_MS", std::to_string(kStopBoundMs).c_str(), 1);
      unsetenv("RCLLF_STOP_MAX_MS");
      g_stall_main_publish.store(true);
      g_arm_stop_at_stage.store(0);
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);  // the manager node's construction stalls
      _exit(0);  // not reached when the stall took
    }, &took, &out);
  ASSERT_NE(out.find(kStallMarker), std::string::npos)
    << "the stall did not take (the publish was not interposed); child output:\n" << out;
  ASSERT_NE(out.find("TEST795_HOOK before the exit decision"), std::string::npos)
    << "the watchdog did not call the expiry hook:\n" << out;
  ASSERT_NE(out.find("TEST795_STOP_ARMED"), std::string::npos)
    << "the SIGINT did not arm the stop bound inside the window:\n" << out;
  ASSERT_NE(status, -1) << "the stuck start ran " << kCapMs << " ms:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_EQ(out.find("startup did not reach the discovery wait"), std::string::npos)
    << "the watchdog exited with the startup message although the stop bound was armed "
    "before its exit decision:\n" << out;
  EXPECT_NE(out.find(kStopBoundMessage), std::string::npos) << out;
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode) << out;
  // The signal comes at the startup bound; the stop bound runs from there.
  EXPECT_GE(took.count(), kBoundMs + kStopBoundMs - 100) << out;
  EXPECT_LE(took.count(), kBoundMs + kStopBoundMs + kBoundEndSlackMs) << out;
}

// Once the watchdog committed to its exit, a stop-bound
// arming does not race it. The stop path records its bound but sets no timer
// and logs nothing; the process ends at the startup bound with its message.
TEST(StartupBound, AStopArmedAfterTheExitDecisionLeavesTheStartupExit)
{
  if (!fastrtps_rmw()) {
    GTEST_SKIP() << "the stall stands in for a Fast DDS publish; RMW is not rmw_fastrtps_cpp";
  }
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      setenv("RCLLF_STOP_TIMEOUT_MS", std::to_string(kStopBoundMs).c_str(), 1);
      unsetenv("RCLLF_STOP_MAX_MS");
      g_stall_main_publish.store(true);
      g_arm_stop_at_stage.store(1);
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);  // the manager node's construction stalls
      _exit(0);  // not reached when the stall took
    }, &took, &out);
  ASSERT_NE(out.find(kStallMarker), std::string::npos)
    << "the stall did not take (the publish was not interposed); child output:\n" << out;
  ASSERT_NE(out.find("TEST795_HOOK after the exit decision"), std::string::npos)
    << "the watchdog did not commit to its exit:\n" << out;
  ASSERT_NE(out.find("TEST795_STOP_ARMED"), std::string::npos)
    << "the SIGINT did not arm the stop bound inside the window:\n" << out;
  ASSERT_NE(status, -1) << "the stuck start ran " << kCapMs << " ms:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_NE(out.find(kBoundMessage), std::string::npos) << out;
  EXPECT_EQ(out.find("rcllf: process end bounded to"), std::string::npos)
    << "the stop bound armed after the startup bound had committed to its exit:\n" << out;
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode) << out;
  EXPECT_GE(took.count(), kBoundMs - 100) << out;
  EXPECT_LE(took.count(), kBoundMs + kBoundEndSlackMs) << out;
}

// RCLLF_STOP_TIMEOUT_MS=0 arms no stop bound, so a signal
// during a stuck start hands nothing over; the startup bound still ends it.
TEST(StartupBound, AStopBoundTurnedOffLeavesTheStuckStartToTheStartupBound)
{
  if (!fastrtps_rmw()) {
    GTEST_SKIP() << "the stall stands in for a Fast DDS publish; RMW is not rmw_fastrtps_cpp";
  }
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      setenv("RCLLF_STOP_TIMEOUT_MS", "0", 1);
      unsetenv("RCLLF_STOP_MAX_MS");
      g_stall_main_publish.store(true);
      std::thread(
        []() {
          std::this_thread::sleep_for(std::chrono::milliseconds(kBoundMs - kSignalBeforeBoundMs));
          say("TEST795_SIGINT\n");
          kill(getpid(), SIGINT);
        }).detach();
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);  // the manager node's construction stalls
      _exit(0);  // not reached when the stall took
    }, &took, &out);
  ASSERT_NE(out.find(kStallMarker), std::string::npos)
    << "the stall did not take (the publish was not interposed); child output:\n" << out;
  ASSERT_NE(out.find("TEST795_SIGINT"), std::string::npos) << out;
  ASSERT_NE(status, -1)
    << "the stuck start with the stop bound off ran " << kCapMs << " ms after a SIGINT: "
    "the signal left it unbounded:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_NE(out.find(kBoundMessage), std::string::npos) << out;
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode) << out;
  EXPECT_GE(took.count(), kBoundMs - 100) << out;
  EXPECT_LE(took.count(), kBoundMs + kBoundEndSlackMs) << out;
}

TEST(StartupBound, AHealthyStartIsNotCutOffAfterTheDiscoveryWait)
{
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      setenv("ROS2_LF_DISCOVERY_TIMEOUT_MS", "0", 1);
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);
      std::fputs("TEST795_STEP init\n", stderr);
      ros2_lf_runtime_load_nodes();
      std::fputs("TEST795_STEP load_nodes\n", stderr);
      ros2_lf_runtime_wait_ready();
      std::fputs("TEST795_STEP wait_ready\n", stderr);
      // Past the discovery wait: three bounds later the process still runs.
      std::this_thread::sleep_for(std::chrono::milliseconds(3 * kBoundMs));
      std::fflush(nullptr);
      _exit(0);
    }, &took, &out);
  ASSERT_NE(status, -1) << "the healthy start hung; child output:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_EQ(WEXITSTATUS(status), 0) << "the bound cut off a start past its discovery wait:\n"
                                    << out;
  EXPECT_NE(out.find("rcllf: startup bounded to 1000 ms"), std::string::npos) << out;
  EXPECT_EQ(out.find("startup did not reach the discovery wait"), std::string::npos) << out;
}

TEST(StartupBound, AnOutOfRangeTimeoutFallsBackToTheDefault)
{
  std::chrono::milliseconds took{0};
  std::string out;
  const int status = run_child(
    []() {
      set_startup_env();
      // strtoll saturates this; taken as is, the wait's duration overflowed.
      setenv("RCLLF_STARTUP_TIMEOUT_MS", "99999999999999999999", 1);
      setenv("ROS2_LF_DISCOVERY_TIMEOUT_MS", "0", 1);
      char arg0[] = "test_startup_bound";
      char * argv[] = {arg0, nullptr};
      ros2_lf_runtime_init(1, argv);
      ros2_lf_runtime_load_nodes();
      ros2_lf_runtime_wait_ready();
      std::fflush(nullptr);
      _exit(0);
    }, &took, &out);
  ASSERT_NE(status, -1) << "hung; child output:\n" << out;
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status) << "\n" << out;
  EXPECT_EQ(WEXITSTATUS(status), 0) << out;
  EXPECT_NE(out.find("is not a count of ms; using 30000"), std::string::npos) << out;
  EXPECT_NE(out.find("rcllf: startup bounded to 30000 ms"), std::string::npos) << out;
}

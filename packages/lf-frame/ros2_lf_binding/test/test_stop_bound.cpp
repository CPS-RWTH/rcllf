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

/// @file test_stop_bound.cpp
/// @brief The stop bound: federates end after a SIGTERM to ros2 launch.
///
/// A federate whose LF stop waits for an RTI that is gone never ended, and
/// ros2 launch answers a SIGTERM to itself by exiting without stopping its
/// processes. Each test runs the binding in a forked child, which stands in
/// for a federate whose stop hangs (it blocks forever after the signal), and
/// checks that the child ends anyway: after its own signal (the stop bound),
/// after a signal during a teardown that blocks, and after its parent died
/// (RCLLF_EXIT_WITH_PARENT), also when it died before the child got there.
/// The last two check that the shutdown a
/// federate runs before its RTI resign runs once, and that a second call
/// during the first ends the process.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"

namespace
{

constexpr int kCapMs = 10000;  // a hang, not a slow host: the bound is 500 ms
constexpr int kStopTimeoutExitCode = 124;  // runtime.cpp, documented

// In the child: init the binding as a federate's main does. LF never starts,
// so the stop the signal requests stays pending forever, as when the RTI
// cannot grant it.
void init_binding_in_child()
{
  char arg0[] = "test_stop_bound";
  char * argv[] = {arg0, nullptr};
  const std::string system_json = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_stop_bound_system.json";
  std::ofstream(system_json) << "{}\n";
  setenv("SYSTEM_JSON", system_json.c_str(), 1);
  setenv("RCLLF_STOP_TIMEOUT_MS", "500", 1);
  ros2_lf_runtime_init(1, argv);
}

[[noreturn]] void block_forever()
{
  for (;; ) {
    pause();
  }
}

// Waits up to kCapMs for `pid` to end; SIGKILLs it on the cap. Returns the
// wait status, or -1 when it had to be killed.
int wait_capped(pid_t pid, std::chrono::milliseconds * took)
{
  const auto t0 = std::chrono::steady_clock::now();
  for (;; ) {
    int status = 0;
    if (waitpid(pid, &status, WNOHANG) == pid) {
      *took = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);
      return status;
    }
    if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(kCapMs)) {
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool read_capped(int fd, void * buf, size_t len)
{
  struct pollfd p {fd, POLLIN, 0};
  return poll(&p, 1, kCapMs) == 1 && read(fd, buf, len) == static_cast<ssize_t>(len);
}

}  // namespace

TEST(StopBound, ASignalledProcessWhoseStopHangsStillEnds)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    init_binding_in_child();
    std::raise(SIGTERM);  // as from launch's group, or from timeout
    block_forever();
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1)
    << "the federate stand-in still ran " << kCapMs << " ms after its SIGTERM: "
    "a stop that cannot complete keeps it alive";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode);
  EXPECT_GE(took.count(), 400) << "ended before the 500 ms bound";
}

// The federate reached its stop time on its own, so no signal armed the bound,
// and its teardown then blocks (the RTI is gone). The operator's SIGTERM
// arrives during that teardown, where the late signal counter handles it.
TEST(StopBound, ASignalDuringABlockedTeardownStillEndsTheProcess)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    init_binding_in_child();
    // Runs inside the teardown's own rclcpp::shutdown().
    rclcpp::contexts::get_global_default_context()->add_on_shutdown_callback(
      []() {
        std::raise(SIGTERM);
        block_forever();
      });
    ros2_lf_runtime_shutdown();
    _exit(0);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1)
    << "the federate stand-in still ran " << kCapMs << " ms after a SIGTERM during its "
    "blocked teardown: a late signal only counted";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), kStopTimeoutExitCode);
  EXPECT_GE(took.count(), 400) << "ended before the 500 ms bound";
}

// Without a signal, a teardown slower than the bound is not cut off: the
// binding's own rclcpp::shutdown() arms nothing.
TEST(StopBound, ASlowTeardownWithoutASignalIsNotCutOff)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    init_binding_in_child();
    rclcpp::contexts::get_global_default_context()->add_on_shutdown_callback(
      []() {std::this_thread::sleep_for(std::chrono::milliseconds(1500));});
    ros2_lf_runtime_shutdown();
    _exit(0);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1) << "the teardown never ended";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), 0)
    << "a teardown without a signal was cut off at the 500 ms bound";
  EXPECT_GE(took.count(), 1500);
}

TEST(StopBound, ExitWithParentEndsTheProcessWhenItsParentDies)
{
  // Orphans re-parent to this process, so it can wait for the grandchild.
  ASSERT_EQ(prctl(PR_SET_CHILD_SUBREAPER, 1), 0);
  int pid_pipe[2];
  ASSERT_EQ(pipe(pid_pipe), 0);
  const pid_t parent = fork();
  ASSERT_GE(parent, 0);
  if (parent == 0) {
    // The stand-in for ros2 launch: start the federate, wait until it runs,
    // then die without signalling it (launch's SIGTERM handling).
    int ready[2];
    if (pipe(ready) != 0) {
      _exit(2);
    }
    const std::string launch_pid = std::to_string(getpid());
    const pid_t fed = fork();
    if (fed == 0) {
      setenv("RCLLF_EXIT_WITH_PARENT", launch_pid.c_str(), 1);
      init_binding_in_child();
      const pid_t self = getpid();
      [[maybe_unused]] ssize_t n = write(pid_pipe[1], &self, sizeof(self));
      n = write(ready[1], "r", 1);
      block_forever();
    }
    char c;
    struct pollfd p {ready[0], POLLIN, 0};
    _exit(poll(&p, 1, kCapMs) == 1 && read(ready[0], &c, 1) == 1 ? 0 : 3);
  }
  pid_t fed = -1;
  ASSERT_TRUE(read_capped(pid_pipe[0], &fed, sizeof(fed))) << "the federate stand-in never ran";
  int parent_status = 0;
  ASSERT_EQ(waitpid(parent, &parent_status, 0), parent);
  ASSERT_TRUE(WIFEXITED(parent_status) && WEXITSTATUS(parent_status) == 0);
  std::chrono::milliseconds took{0};
  const int status = wait_capped(fed, &took);
  ASSERT_NE(status, -1)
    << "the federate stand-in still ran " << kCapMs << " ms after its parent died: "
    "it outlives a launch that exited on SIGTERM";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
}

// The race: launch dies before the federate reaches the binding's init. The
// federate's parent is then init or a subreaper, which a getppid() taken at
// init would accept as its parent, and no death signal would ever come.
TEST(StopBound, ExitWithParentEndsAProcessWhoseParentDiedBeforeInit)
{
  ASSERT_EQ(prctl(PR_SET_CHILD_SUBREAPER, 1), 0);
  int pid_pipe[2];
  ASSERT_EQ(pipe(pid_pipe), 0);
  const pid_t parent = fork();
  ASSERT_GE(parent, 0);
  if (parent == 0) {
    const pid_t launch = getpid();
    const pid_t fed = fork();
    if (fed == 0) {
      setenv("RCLLF_EXIT_WITH_PARENT", std::to_string(launch).c_str(), 1);
      while (getppid() == launch) {  // until the stand-in for launch is gone
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      init_binding_in_child();
      block_forever();
    }
    [[maybe_unused]] ssize_t n = write(pid_pipe[1], &fed, sizeof(fed));
    _exit(0);
  }
  pid_t fed = -1;
  ASSERT_TRUE(read_capped(pid_pipe[0], &fed, sizeof(fed))) << "the federate stand-in never ran";
  int parent_status = 0;
  ASSERT_EQ(waitpid(parent, &parent_status, 0), parent);
  std::chrono::milliseconds took{0};
  const int status = wait_capped(fed, &took);
  ASSERT_NE(status, -1)
    << "the federate stand-in still ran " << kCapMs << " ms after its parent died "
    "before init: it took its new parent for its launch";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), EXIT_FAILURE);
}

// The old value 1 named no pid and missed that race; it is refused at init.
TEST(StopBound, ExitWithParentRefusesAValueThatIsNotAPid)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    setenv("RCLLF_EXIT_WITH_PARENT", "1", 1);
    init_binding_in_child();
    _exit(0);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1);
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), EXIT_FAILURE) << "RCLLF_EXIT_WITH_PARENT=1 was accepted";
}

// A federate's generated startup runs the
// shutdown before reactor-c's RTI resign and again from its atexit handler.
// The second call only ends the process the way the first one decided: here
// the teardown passed, so it returns and the process ends with its own code.
// The teardown logs its rmw context line once; the child's stderr goes to a
// file the parent counts it in.
TEST(StopBound, ShutdownBeforeResignRunsTheTeardownOnce)
{
  const std::string err_path = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_shutdown_once.err";
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    const int fd = open(err_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0 || dup2(fd, STDERR_FILENO) < 0) {
      _exit(2);
    }
    init_binding_in_child();
    ros2_lf_runtime_shutdown_before_resign();
    ros2_lf_runtime_shutdown();  // the atexit call after reactor-c's termination()
    std::fflush(stderr);
    _exit(7);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1) << "the atexit call after the early shutdown hung";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), 7)
    << "the atexit call after the early shutdown ended the process itself";
  std::ifstream err(err_path);
  int teardowns = 0;
  for (std::string line; std::getline(err, line); ) {
    teardowns += line.find("rcllf: rmw context") != std::string::npos ? 1 : 0;
  }
  EXPECT_EQ(teardowns, 1)
    << "the atexit call after the early shutdown ran the teardown again";
}

// A plain shutdown that finished cleanly, called again afterwards (not
// concurrently), returns: only a call during a running teardown ends the
// process.
TEST(StopBound, ASequentialSecondShutdownReturns)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    init_binding_in_child();
    ros2_lf_runtime_shutdown();
    ros2_lf_runtime_shutdown();
    std::fflush(stderr);
    _exit(7);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1) << "the second sequential shutdown hung";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), 7)
    << "the second sequential shutdown ended the process as if it were concurrent";
}

// A second call while the first still runs comes from another thread's
// exit(), as reactor-c's RTI listener exits on MSG_TYPE_FAILED: it ends the
// process at once with code 1 instead of tearing down the nodes twice.
TEST(StopBound, AConcurrentSecondShutdownEndsTheProcess)
{
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    init_binding_in_child();
    // Runs inside the first call's own rclcpp::shutdown().
    rclcpp::contexts::get_global_default_context()->add_on_shutdown_callback(
      []() {
        std::thread([]() {ros2_lf_runtime_shutdown();}).join();
        _exit(0);  // the second call returned: the guard did not end the process
      });
    ros2_lf_runtime_shutdown_before_resign();
    _exit(0);
  }
  std::chrono::milliseconds took{0};
  const int status = wait_capped(child, &took);
  ASSERT_NE(status, -1) << "a second shutdown during the first hung the process";
  ASSERT_TRUE(WIFEXITED(status)) << "ended by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), 1)
    << "a second shutdown during the first did not end the process at once";
}

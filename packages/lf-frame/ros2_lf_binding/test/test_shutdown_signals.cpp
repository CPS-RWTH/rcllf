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

/// @file test_shutdown_signals.cpp
/// @brief Once ros2_lf_runtime_shutdown() has begun, a SIGINT or
/// SIGTERM neither runs rclcpp's handler nor kills the process, and the
/// teardown finalizes the rmw context before exit() (so no DDS participant
/// is left for Fast DDS's static destructors). Launch forwards the group
/// SIGINT to the child about 1 ms after the child got it, so the second
/// signal is the normal case.

#include <gtest/gtest.h>

#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <fstream>
#include <string>

#include "rcl/context.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"

namespace ros2_lf_binding
{
void lf_mark_running();  // runtime.cpp
}  // namespace ros2_lf_binding

TEST(ShutdownSignals, TeardownFinalizesContextAndLateSignalsOnlyCount)
{
  char arg0[] = "test_shutdown_signals";
  char * argv[] = {arg0, nullptr};
  // Init refuses a missing system.json, so hand it an empty
  // one. No component is loaded here.
  const std::string system_json = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_shutdown_system.json";
  std::ofstream(system_json) << "{}\n";
  setenv("SYSTEM_JSON", system_json.c_str(), 1);
  ros2_lf_runtime_init(1, argv);
  // The first LF startup reaction re-installs rclcpp's handlers.
  ros2_lf_binding::lf_mark_running();
  ASSERT_TRUE(rclcpp::signal_handlers_installed());
  auto context = rclcpp::contexts::get_global_default_context();
  auto rcl_context = context->get_rcl_context();

  ros2_lf_runtime_shutdown();

  EXPECT_FALSE(rclcpp::signal_handlers_installed())
    << "rclcpp's signal thread still runs after the teardown began";
  EXPECT_EQ(rcl_context->impl, nullptr)
    << "rmw context not finalized: the DDS participant lives into exit()";
  // Both late signals only count; the old SIG_DFL ended the process here.
  std::raise(SIGINT);
  std::raise(SIGTERM);
  SUCCEED();
}

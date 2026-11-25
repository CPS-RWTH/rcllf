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

// The end of a test suite that calls rclcpp::init itself, without
// the runtime. ros2_lf_bind_node on a lifecycle node pins the node's entities
// in the lifecycle gate, whose table is never destroyed; in a generated
// executable the runtime's pre-shutdown callback releases the pins
// (runtime.cpp). A suite that skips the release keeps the
// node, and so the Fast DDS participant, alive into exit(): its receive
// threads then pair readers of other processes on the domain with writers the
// static destructors free, a SIGSEGV after every test passed.

#ifndef RCLCPP_TEST_TEARDOWN_HPP_
#define RCLCPP_TEST_TEARDOWN_HPP_

#include <cstdio>
#include <cstdlib>
#include <string>

#include "lifecycle_gate.hpp"  // NOLINT(build/include_subdir)
#include "node_registry_internal.h"  // NOLINT(build/include_subdir)
#include "gtest/gtest.h"
#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/node_registry.h"

namespace rcllf_test
{

/// Releases what the binding holds of the suite's nodes, shuts rclcpp down and
/// finalizes the rmw context, which destroys the DDS participant and joins its
/// threads. Returns "" on success, otherwise why the context stayed alive: a
/// node still exists. The caller drops its own node references (g_manager)
/// first.
inline std::string release_and_finalize_rclcpp()
{
  ros2_lf_binding::lifecycle_gate_release_pins();
  ros2_lf_clear_node_registry();
  rclcpp::shutdown();
  const rcl_ret_t fini = rcl_context_fini(
    rclcpp::contexts::get_global_default_context()->get_rcl_context().get());
  if (fini == RCL_RET_OK) {
    return "";
  }
  std::string why = "rcl_context_fini returned " + std::to_string(fini) +
    ": a node outlived the suite, so the DDS participant and its threads run into exit() "
    "";
  if (rcl_error_is_set()) {
    why += std::string(": ") + rcl_get_error_string().str;
  }
  rcl_reset_error();
  return why;
}

/// The suite-end check. A context that did not finalize leaves the
/// DDS participant and its receive threads alive into exit(), where they crash
/// at random in the static destructors. So after gtest records the failure,
/// the process ends at the start of exit() without static destructors: gtest
/// still prints its summary and writes its XML (main returns first), and the
/// leak is always a clean failing exit code, never a SIGSEGV. An atexit
/// handler registered now runs before the destructors of every static object
/// that was constructed earlier, the DDS singletons among them.
inline void expect_rclcpp_finalized_or_exit()
{
  const std::string why = release_and_finalize_rclcpp();
  EXPECT_EQ(why, "");
  if (!why.empty()) {
    std::atexit(
      []() {
        std::fprintf(stderr, "rcllf_test: rmw context not finalized, ending without static "
        "destructors\n");
        std::fflush(stdout);
        std::fflush(stderr);
        std::_Exit(EXIT_FAILURE);
      });
  }
}

}  // namespace rcllf_test

#endif  // RCLCPP_TEST_TEARDOWN_HPP_

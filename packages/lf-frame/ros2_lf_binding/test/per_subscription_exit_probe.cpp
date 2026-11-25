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

/// @file per_subscription_exit_probe.cpp
/// @brief A process whose static PerSubscription still holds the
/// entry of a subscription made by an unloaded library must exit cleanly.
/// The brake_fusion T2Approx case: the ComponentManager destroys its nodes
/// and dlcloses their libraries, then the binding's static maps (take debt,
/// message_lost totals) are destroyed at exit. With a weak reference to the
/// subscription itself, dropping it calls the control block's destroy
/// function in the unloaded library and the process dies with SIGSEGV.
/// Run as a child process by test_per_subscription_exit.
///
/// The probe first proves the setup: a library whose subscription the map
/// never saw is unloaded by dlclose (else the crash could not happen and the
/// test would prove nothing). Then it loads the library again, records an
/// entry for its subscription, destroys the subscription, calls dlclose and
/// returns. The fix keeps that library mapped (RTLD_NODELETE); the probe
/// prints whether it stayed loaded.
///
/// Exit codes: 0 clean, 2 setup failed, 3 dlclose did not unload the library
/// in the setup check. A crash shows as the signal.

#include <dlfcn.h>

#include <cstdio>
#include <memory>

#include "per_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
// Static like g_debt (take_mode.cpp) and totals (reaction_dispatch.cpp):
// destroyed by the exit handlers after main returns.
ros2_lf_binding::PerSubscription<int> g_state;
}  // namespace

namespace
{
using MakeSub = std::shared_ptr<rclcpp::SubscriptionBase>(*)(rclcpp::Node *, const char *);

bool loaded(const char * path)
{
  return dlopen(path, RTLD_NOW | RTLD_NOLOAD) != nullptr;
}

/// Load the library, make a subscription with it, record an entry for it in
/// g_state when `record`, destroy it, dlclose. False when a step failed.
bool make_and_unload(const char * path, bool record)
{
  void * lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (lib == nullptr) {
    std::fprintf(stderr, "dlopen failed: %s\n", dlerror());
    return false;
  }
  auto make_sub = reinterpret_cast<MakeSub>(dlsym(lib, "per_subscription_exit_make_sub"));
  if (make_sub == nullptr) {
    std::fprintf(stderr, "dlsym failed: %s\n", dlerror());
    return false;
  }
  {
    auto node = std::make_shared<rclcpp::Node>("per_subscription_exit_probe");
    auto sub = make_sub(node.get(), "/per_subscription_exit_probe/topic");
    if (record) {
      g_state.at(*sub) = 7;  // the entry stays after the subscription dies
    }
  }
  dlclose(lib);
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <path to per_subscription_exit_lib.so>\n", argv[0]);
    return 2;
  }
  const char * const path = argv[1];
  rclcpp::init(1, argv);
  if (!make_and_unload(path, false)) {
    return 2;
  }
  if (loaded(path)) {
    std::fprintf(stderr, "setup: dlclose did not unload the library\n");
    return 3;
  }
  if (!make_and_unload(path, true)) {
    return 2;
  }
  rclcpp::shutdown();
  std::fprintf(
    stderr, "probe: %zu entry left for exit, library %s after dlclose\n", g_state.size(),
    loaded(path) ? "kept loaded" : "unloaded");
  return 0;
}

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

#include "ros2_lf_binding/node_registry.h"

#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "node_registry_internal.h"  // NOLINT(build/include_subdir)
#include "rclcpp/rclcpp.hpp"

namespace
{
// Type-erased registry entry: `owner` keeps the concrete node (whatever its
// base — rclcpp::Node, rclcpp_lifecycle::LifecycleNode, ...) alive; `base`
// is what every consumer actually uses. Holding only `base` would not
// extend the node's lifetime, since NodeBaseInterface does not own the node.
struct RegistryEntry
{
  std::shared_ptr<void> owner;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  // Empty for a plain rclcpp::Node. For a lifecycle node this is the
  // configure+activate thunk captured at registration, the only place the
  // concrete type was still visible.
  Ros2LfTransitionFn transition;
  // Empty for a plain rclcpp::Node. Reads a lifecycle node's current primary
  // state, for deferred binding.
  Ros2LfStateFn state;
  // The system.json class this node instantiates, given at registration. Two
  // or more instances of one class are keyed `<class>__<suffix>`, so the key
  // alone does not name the class.
  std::string node_class;
};

std::mutex g_mutex;
std::unordered_map<std::string, RegistryEntry> g_registry;
}  // namespace

void ros2_lf_register_node_erased(
  const std::string & binding_key,
  const std::string & node_class,
  std::shared_ptr<void> owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition,
  Ros2LfStateFn state)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  // Courtesy tripwire: with per-instance binding keys, generated code never
  // registers the same key twice, so this branch should be unreachable via
  // generated code. Kept as a one-line debug log rather than removed, in case
  // a future/legacy code path still calls this with a colliding key.
  if (g_registry.find(binding_key) != g_registry.end()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_register_node: overwriting existing registration for key '%s' — "
      "unreachable via current generated code; two instances would otherwise share "
      "one wire identity",
      binding_key.c_str());
  }
  g_registry[binding_key] = RegistryEntry{
    std::move(owner), std::move(base), std::move(transition), std::move(state), node_class};
}

void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr<void> owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base)
{
  ros2_lf_register_node_erased(
    class_name, class_name, std::move(owner), std::move(base), Ros2LfTransitionFn{},
    Ros2LfStateFn{});
}

void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr<void> owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition)
{
  ros2_lf_register_node_erased(
    class_name, class_name, std::move(owner), std::move(base), std::move(transition),
    Ros2LfStateFn{});
}

void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr<void> owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition,
  Ros2LfStateFn state)
{
  ros2_lf_register_node_erased(
    class_name, class_name, std::move(owner), std::move(base), std::move(transition),
    std::move(state));
}

int ros2_lf_node_lifecycle_state(const std::string & class_name)
{
  // Copied out and run unlocked, like the transition thunk below.
  Ros2LfStateFn state;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_registry.find(class_name);
    if (it == g_registry.end()) {
      return -1;
    }
    state = it->second.state;
  }
  return state ? static_cast<int>(state()) : -1;
}

bool ros2_lf_transition_node(const std::string & class_name)
{
  // Copy the thunk out under the lock, then run it unlocked: on_configure /
  // on_activate is user code that creates publishers, timers, and services,
  // any of which may re-enter the registry.
  Ros2LfTransitionFn transition;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_registry.find(class_name);
    if (it == g_registry.end()) {
      // Not a registry node (e.g. a loaded component, owned by the component
      // manager). Nothing to drive, and not an error.
      return true;
    }
    transition = it->second.transition;
  }

  if (!transition) {
    return true;  // Plain node: nothing to drive, already ready to bind.
  }

  const bool ok = transition();
  if (ok) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "Lifecycle node '%s': configured and activated before binding",
      class_name.c_str());
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "Lifecycle node '%s': configure/activate failed; entities created in "
      "on_configure/on_activate will not be bound",
      class_name.c_str());
  }
  return ok;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ros2_lf_get_node(
  const std::string & class_name)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  auto it = g_registry.find(class_name);
  if (it != g_registry.end()) {
    return it->second.base;
  }
  return nullptr;
}

bool ros2_lf_class_has_registered_node(const std::string & class_name)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  for (const auto & [key, entry] : g_registry) {
    (void)key;
    if (entry.node_class == class_name && entry.base) {
      return true;
    }
  }
  return false;
}

void ros2_lf_clear_node_registry()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_registry.clear();
}

void ros2_lf_for_each_registered_node(
  const std::function<void(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &)> & fn)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  for (const auto & [class_name, entry] : g_registry) {
    (void)class_name;
    if (entry.base) {
      fn(entry.base);
    }
  }
}

std::vector<std::string> ros2_lf_materialize_param_args(
  const std::vector<std::string> & args)
{
  static const char kFlag[] = "--rcllf-params-yaml";
  std::vector<std::string> out;
  out.reserve(args.size());
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] != kFlag) {
      out.push_back(args[i]);
      continue;
    }
    if (i + 1 >= args.size()) {
      throw std::runtime_error(std::string(kFlag) + " without its YAML text");
    }
    const std::string & text = args[++i];
    // Same naming as launch_ros' NamedTemporaryFile(prefix='launch_params_').
    std::string path =
      (std::filesystem::temp_directory_path() / "rcllf_launch_params_XXXXXX").string();
    int fd = mkstemp(path.data());
    if (fd < 0) {
      throw std::runtime_error(
              "cannot create a launch parameter file " + path + ": " + std::strerror(errno));
    }
    const std::string body = text + "\n";
    size_t written = 0;
    while (written < body.size()) {
      ssize_t n = write(fd, body.data() + written, body.size() - written);
      if (n < 0 && errno == EINTR) {
        continue;
      }
      if (n <= 0) {
        int err = errno;
        close(fd);
        throw std::runtime_error(
                "cannot write the launch parameter file " + path + ": " + std::strerror(err));
      }
      written += static_cast<size_t>(n);
    }
    close(fd);
    out.push_back("--params-file");
    out.push_back(path);
  }
  return out;
}

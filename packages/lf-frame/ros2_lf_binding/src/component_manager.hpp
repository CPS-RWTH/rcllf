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

/// @file component_manager.hpp
/// @brief LFComponentManager — thin subclass of rclcpp_components::ComponentManager
/// that exposes on_load_node and get_all_components.

#ifndef COMPONENT_MANAGER_HPP_
#define COMPONENT_MANAGER_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "time_source.hpp"

namespace ros2_lf_binding
{

/// Result of resolving a `find_by_class` lookup key against the
/// `class_to_id_` map's key set, without touching any live node state.
/// Factored out of `LFComponentManager::find_by_class` so the exact-vs-
/// fallback gating logic is unit-testable without a real rclcpp_components
/// node (`test_component_manager.cpp`).
struct BindingKeyLookup
{
  bool found = false;
  bool via_fallback = false;  ///< True if matched only via the legacy suffix fallback.
  std::string matched_key;    ///< The class_to_id_ key that matched, if found.
};

/// Resolve `class_name` against `keys` (the current `class_to_id_` key set).
///
/// 1. Exact match first — the only path a synthesized per-instance key
///    (containing `__`) can ever take.
/// 2. Legacy short-name suffix fallback (strip a "pkg::" prefix and match the
///    trailing class name) — gated two ways: the lookup key itself must
///    contain no `__` marker, and candidate keys containing `__` are skipped,
///    so a synthesized key can never be produced by, or matched via, the
///    fallback.
inline BindingKeyLookup resolve_binding_key_lookup(
  const std::string & class_name, const std::vector<std::string> & keys)
{
  for (const auto & key : keys) {
    if (key == class_name) {
      return {true, false, key};
    }
  }

  if (class_name.find("__") != std::string::npos) {
    return {false, false, ""};
  }
  for (const auto & key : keys) {
    if (key.find("__") != std::string::npos) {
      // Never fall back onto a synthesized instance key: it is not a plain
      // class name and matching its suffix would silently re-alias two
      // distinct instances.
      continue;
    }
    auto pos = key.rfind("::");
    if (pos != std::string::npos && key.substr(pos + 2) == class_name) {
      return {true, true, key};
    }
  }
  return {false, false, ""};
}

class LFComponentManager : public rclcpp_components::ComponentManager
{
public:
  using ComponentManager::ComponentManager;

  /// Load one component instance, registering it in the registry under
  /// `binding_key` — which is the real pluginlib plugin name for
  /// single-instance (legacy) classes, and a synthesized per-instance key for
  /// multi-instance classes.
  /// `plugin_name` is always the real pluginlib class and is what
  /// actually reaches `on_load_node` — the binding key never substitutes for
  /// it, since that would break the load.
  ///
  /// `node_name` / `node_namespace` are optional per-instance identity
  /// realized via the LoadNode request's own fields (this is exactly how
  /// normal multi-instance composition works); empty strings mean "let the
  /// component use its default name/namespace". `remap_rules` are the launch
  /// remaps of the instance, passed the same way. `param_args` are the launch
  /// parameters as --ros-args rules ("-p", "--params-file"); LoadNode has no
  /// field for them, so create_node_options() appends them.
  bool load_component(
    const std::string & package_name, const std::string & plugin_name,
    const std::string & binding_key, rclcpp::Executor & executor,
    const std::string & node_name = "", const std::string & node_namespace = "",
    const std::vector<std::string> & remap_rules = {},
    const std::vector<std::string> & param_args = {})
  {
    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = package_name;
    request->plugin_name = plugin_name;
    if (!node_name.empty()) {
      request->node_name = node_name;
    }
    if (!node_namespace.empty()) {
      request->node_namespace = node_namespace;
    }
    request->remap_rules = remap_rules;  // launch remaps, "src:=dst"

    auto response = std::make_shared<composition_interfaces::srv::LoadNode::Response>();
    auto header = std::make_shared<rmw_request_id_t>();
    pending_param_args_ = param_args;
    on_load_node(header, request, response);
    pending_param_args_.clear();

    if (response->success) {
      RCLCPP_INFO(
        get_logger(), "Loaded %s as '%s' (id=%lu)", plugin_name.c_str(), binding_key.c_str(),
        response->unique_id);
      // Do NOT add nodes to an executor — lfc is the sole scheduler.
      // Nodes are loaded (instantiated) here; lf reactions drive their callbacks.
      (void)executor;
      // The node was built with lf_time_node_options() (create_node_options
      // below); hand its TimeSource to the LF time source. A node
      // whose clock cannot follow LF logical time is unloaded, not kept.
      auto it = node_wrappers_.find(response->unique_id);
      if (it == node_wrappers_.end() ||
        !lf_attach_time_source(it->second.get_node_base_interface(), binding_key))
      {
        RCLCPP_ERROR(
          get_logger(), "Unloading %s as '%s': its clock cannot follow LF logical time",
            plugin_name.c_str(), binding_key.c_str());
        auto unload = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
        unload->unique_id = response->unique_id;
        auto unload_response =
          std::make_shared<composition_interfaces::srv::UnloadNode::Response>();
        on_unload_node(header, unload, unload_response);
        return false;
      }
      // Record binding_key → node_id for ros2_lf_bind_node lookup.  Unique per
      // instance, so a second instance never clobbers the first.
      class_to_id_[binding_key] = response->unique_id;
      return true;
    }
    RCLCPP_WARN(get_logger(), "Failed to load %s::%s", package_name.c_str(), plugin_name.c_str());
    return false;
  }

  /// Backward-compatible overload: binding_key defaults to plugin_name
  /// (legacy single-instance call shape).
  bool load_component(
    const std::string & package_name, const std::string & plugin_name, rclcpp::Executor & executor)
  {
    return load_component(package_name, plugin_name, plugin_name, executor);
  }

  std::vector<rclcpp_components::NodeInstanceWrapper> get_all_components()
  {
    std::vector<rclcpp_components::NodeInstanceWrapper> result;
    for (auto & [id, wrapper] : node_wrappers_) {
      result.push_back(wrapper);
    }
    return result;
  }

  /// Find a loaded component by its binding key.
  ///
  /// Exact match on the stored key first — this is the only path a
  /// synthesized per-instance key (containing a `__` marker) can ever take.
  /// The legacy short-name suffix fallback (strip a "pkg::" prefix and match
  /// the trailing class name) is gated to keys that look like plain class
  /// names, i.e. contain no `__` marker: `sanitize_lf_identifier` collapses
  /// runs of `_` and the key-synthesis scheme's separator is exactly `__`, so
  /// a synthesized key can never accidentally look like a plain class name.
  /// This keeps old `system.json` files (plugin_name-keyed, no binding_key
  /// field) working unchanged while guaranteeing multi-instance keys are
  /// never re-aliased by the lossy suffix match. Logs when the fallback fires
  /// so a match found only that way is visible.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr find_by_class(
    const std::string & class_name)
  {
    std::vector<std::string> keys;
    keys.reserve(class_to_id_.size());
    for (const auto & [key, id] : class_to_id_) {
      (void)id;
      keys.push_back(key);
    }

    const BindingKeyLookup lookup = resolve_binding_key_lookup(class_name, keys);
    if (!lookup.found) {
      return nullptr;
    }
    if (lookup.via_fallback) {
      RCLCPP_WARN(
        get_logger(),
        "find_by_class('%s'): matched via legacy short-name fallback against '%s' — "
        "this system.json predates per-instance component keys",
        class_name.c_str(), lookup.matched_key.c_str());
    }
    auto it = node_wrappers_.find(class_to_id_.at(lookup.matched_key));
    if (it != node_wrappers_.end()) {
      return it->second.get_node_base_interface();
    }
    return nullptr;
  }

  /// Bookkeeping accessor for tests: the number of distinct binding keys
  /// currently registered (i.e. distinct component instances loaded).
  /// No-op: rclcpp_components' on_load_node() hands
  /// every loaded component to the executor, whose add_node() claims the
  /// component's services, so the ingress wait set could not admit them
  /// (measured: 7 of 7 refused, test/ingress_role_probe.cpp). LF is the only
  /// scheduler; no node is ever added to an executor.
  void add_node_to_executor(uint64_t /*node_id*/) override {}

  /// Every component is built with use_sim_time=true and no clock thread, so
  /// its RCL_ROS_TIME clock follows LF logical time and never a /clock
  /// message. The binding never sees a component's concrete
  /// type, so the node's own TimeSource is the only handle to its clock.
  rclcpp::NodeOptions create_node_options(
    const std::shared_ptr<composition_interfaces::srv::LoadNode::Request> request) override
  {
    rclcpp::NodeOptions options = ComponentManager::create_node_options(request);
    if (!pending_param_args_.empty()) {
      // The instance's launch parameters, as launch_ros puts
      // them on a node's command line. Inline dicts become temp params files.
      std::vector<std::string> args = options.arguments();
      args.push_back("--ros-args");
      const std::vector<std::string> params =
        ros2_lf_materialize_param_args(pending_param_args_);
      args.insert(args.end(), params.begin(), params.end());
      options.arguments(args);
    }
    return lf_time_node_options(options);
  }
  void remove_node_from_executor(uint64_t /*node_id*/) override {}

  size_t class_to_id_size_for_test() const {return class_to_id_.size();}

  /// Test-only accessor: true iff `binding_key` has a registered node id.
  bool has_binding_key_for_test(const std::string & binding_key) const
  {
    return class_to_id_.find(binding_key) != class_to_id_.end();
  }

  /// Test-only accessor: the node id registered under `binding_key`, or 0 if
  /// absent. Used to assert instance-keyed bookkeeping (two distinct
  /// instances of one class get two distinct ids, neither clobbering the
  /// other) without needing a real pluginlib load in the gtest.
  uint64_t binding_key_id_for_test(const std::string & binding_key) const
  {
    auto it = class_to_id_.find(binding_key);
    return it != class_to_id_.end() ? it->second : 0;
  }

  /// Seed `class_to_id_` directly (tests only): exercises the same map a real
  /// `load_component()` call writes, without requiring a loadable pluginlib
  /// plugin in this package's gtest. Mirrors the `policy_table_set_for_test`
  /// seam used elsewhere in this binding.
  void set_binding_key_for_test(const std::string & binding_key, uint64_t node_id)
  {
    class_to_id_[binding_key] = node_id;
  }

private:
  std::map<std::string, uint64_t> class_to_id_;
  /// param_args of the load_component() call in progress, read by
  /// create_node_options() (on_load_node passes it only the request).
  std::vector<std::string> pending_param_args_;
};

}  // namespace ros2_lf_binding

#endif  // COMPONENT_MANAGER_HPP_

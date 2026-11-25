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

/// @file runtime_internal.hpp
/// @brief Internal (not installed) declarations for runtime.cpp's system.json
/// component-list parsing, exposed here so gtest can exercise them directly.
///
/// Per-instance component list:
/// `system.json` carries a top-level `"component_instances"` array, one
/// entry per loaded component *instance* (not per class):
///
/// @code
/// { "component_instances": [
///     {"package": "basic_components", "plugin_class": "TimerPublisherNode",
///      "binding_key": "TimerPublisherNode", "node_name": "timer_node",
///      "namespace": ""},
///     {"package": "basic_components", "plugin_class": "TimerPublisherNode",
///      "binding_key": "TimerPublisherNode__robot_1_timer_node",
///      "node_name": "timer_node", "namespace": "/robot_1"} ] }
/// @endcode
///
/// Backward compatibility (fail-closed rule, design §4 schema note): an entry
/// without `binding_key` is legal only when *no* entry in the file has one —
/// that is the old (pre-per-instance) format, and the runtime falls back to
/// legacy behavior for it (dedup on {package, plugin_class}, binding_key ==
/// plugin_class). A file mixing entries with and without `binding_key` is
/// corrupt and is a loud init failure, not a best-effort partial load.

#ifndef RUNTIME_INTERNAL_HPP_
#define RUNTIME_INTERNAL_HPP_

#include <string>
#include <vector>

namespace ros2_lf_binding
{

/// One instance of a component to load, as read from `system.json`'s
/// `component_instances` array.
struct ComponentInstance
{
  std::string package;         ///< ament package providing the plugin.
  std::string plugin_class;    ///< Real pluginlib class name (never the binding key).
  std::string binding_key;     ///< Registry key; == plugin_class for legacy/N=1 entries.
  std::string node_name;       ///< Per-instance ROS node name (may be empty).
  std::string node_namespace;  ///< Per-instance ROS namespace (may be empty).
  std::vector<std::string> remap_rules;  ///< Launch remaps, "src:=dst".
  /// Launch parameters as --ros-args rules, "-p" and "--params-file".
  std::vector<std::string> param_args;
};

/// Result of parsing the component list out of a `system.json` file.
struct ComponentListParseResult
{
  bool ok = true;
  std::string error;  ///< Human-readable reason when `ok` is false.
  std::vector<ComponentInstance> instances;
};

/// Parse `json_path`'s `component_instances` array (entry-scoped: each object
/// is parsed within its own bounds via a real JSON parser, not a proximity
/// window over raw file bytes).
///
/// Legacy compatibility: if no entry in the file has a `binding_key` field,
/// the file predates per-instance components; entries are deduped on
/// {package, plugin_class} (legacy behavior) and each surviving instance's
/// `binding_key` is set to `plugin_class`.
///
/// Fail-closed: if *some but not all* entries have `binding_key`, the file is
/// treated as corrupt (mixed old/new schema) and `ok` is false with `error`
/// set — the same fail-closed rule, applied to the mixed-schema
/// case (design §4).
///
/// A file with no `component_instances` key returns `ok == true` with an
/// empty `instances` list (a system without component nodes). A missing file
/// is an error (`ok` false, `error` from `missing_file_error`): read as
/// "nothing to load", it once started a system with every reaction unbound
/// (OWED rcllf-relative-system-json).
ComponentListParseResult parse_component_list(const std::string & json_path);

/// "" when `path` is empty or names an existing regular file. Otherwise the
/// error the runtime logs before it exits: `what`, the path as given, the
/// absolute path it resolved to and the working directory, so a relative
/// path read from the wrong directory is visible in the message.
std::string missing_file_error(const std::string & what, const std::string & path);

/// Progress after a signal (a dispatch returned). Re-arms the process
/// end bound from now, rate limited, never past RCLLF_STOP_MAX_MS from
/// the first arming. No-op when no bound is armed.
void stop_progress();

/// For tests: true once a stop bound above 0 is armed.
bool stop_bound_armed();

}  // namespace ros2_lf_binding

/// Test only: a weak hook the startup watchdog calls after its
/// wait expired, with 0 before it commits to its exit and with 1 after it
/// did. Undefined (null) in production; a test executable that links with
/// -rdynamic defines it to pause the watchdog in that window.
extern "C" void rcllf_test_startup_expiry_hook(int stage) __attribute__((weak));

#endif  // RUNTIME_INTERNAL_HPP_

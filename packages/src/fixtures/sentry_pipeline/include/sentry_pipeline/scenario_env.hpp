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

#ifndef SENTRY_PIPELINE__SCENARIO_ENV_HPP_
#define SENTRY_PIPELINE__SCENARIO_ENV_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace sentry_pipeline
{

// Reader for the `SENTRY_SCENARIO` runtime overlay file.
// The overlay is a flat
// `KEY=VALUE` text file, one `<node_name>.<key>=<value>` pair per line,
// `#`-prefixed comments allowed, pointed to by the `SENTRY_SCENARIO`
// environment variable. It carries `qos_depth` / `qos_reliability` /
// `busy_us` / `payload_size` — the values that must reach a running process
// at/near construction time and are not part of the LF static graph model.
// `rate_hz` is deliberately never carried by this file (the period reaches the
// lfc variant through the system.json instead).
class ScenarioOverlay
{
public:
  // Loads the overlay pointed to by the `SENTRY_SCENARIO` environment
  // variable, if set. If the variable is unset, returns an empty overlay
  // (every lookup then falls through to the ROS parameter / literal-default
  // tiers). If it is set but names no regular file, or the file cannot be
  // opened or read, logs an error to stderr and throws std::runtime_error,
  // so the process refuses to start instead of silently running on the
  // defaults.
  static ScenarioOverlay load_from_env();

  // Looks up `<node_name>.<key>` in the overlay. Returns std::nullopt if the
  // key is absent, in which case the caller should fall back to its
  // `declare_parameter` value.
  std::optional<std::string> get(const std::string & node_name, const std::string & key) const;

private:
  std::unordered_map<std::string, std::string> values_;
};

// Builds an `rclcpp::QoS` for `node_name` from the overlay -> declare_parameter
// -> literal-default precedence chain. `node` must have
// already declared `qos_depth` (int64_t) and `qos_reliability` (string)
// parameters before this is called. The returned QoS local variable should
// always be bound to a variable named `qos_profile` (or
// `qos_profile_<topic_suffix>`), per the package's QoS naming rule — never a
// name containing a QoS policy word.
rclcpp::QoS qos_from_overlay(
  rclcpp::Node & node, const ScenarioOverlay & overlay, const std::string & node_name);

// Per-topic variant of qos_from_overlay, for the rare node with more than
// one distinct QoS profile across its pubs/subs (e.g.
// DetectorNode's /sentry/image subscription vs. its /sentry/detections
// publisher). `reliability_param` names the node's own declared string
// parameter (e.g. "qos_reliability_image"), and the overlay lookup uses the
// same string as its key suffix (e.g. "<node_name>.qos_reliability_image").
// `qos_depth` remains the single shared depth parameter/overlay key for the
// node (depth is not split per topic). The returned local
// should be named `qos_profile_<topic_suffix>` per the naming rule — never
// a name containing a QoS policy word.
rclcpp::QoS qos_from_overlay(
  rclcpp::Node & node, const ScenarioOverlay & overlay, const std::string & node_name,
  const std::string & reliability_param);

// Reads an integer-valued overlay key (e.g. `busy_us`, `payload_size`),
// falling back to `param_value` if the key is absent or unparseable.
int64_t int_from_overlay(
  const ScenarioOverlay & overlay, const std::string & node_name, const std::string & key,
  int64_t param_value);

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__SCENARIO_ENV_HPP_

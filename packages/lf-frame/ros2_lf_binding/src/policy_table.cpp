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

/// @file policy_table.cpp
/// @brief exec_config.json parsing and (node_class, topic) policy resolution.

#include "policy_table.hpp"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "external_input.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{
namespace
{

// Process-wide table.  Written only by policy_table_load() /
// policy_table_set_for_test() / policy_table_reset() — the single writer point
// documented in policy_table.hpp — and read-only after ros2_lf_runtime_init().
TakePolicy g_defaults;
std::vector<PolicyRow> g_rows;
std::atomic<uint64_t> g_misses{0};

// First kMaxMissExamples (node_class, topic) pairs seen by resolve_policy()
// misses, guarded by g_miss_examples_mutex.  resolve_policy() is only called
// from the single-threaded ros2_lf_bind_node startup path, but the mutex
// keeps this correct if that ever changes and costs nothing on the hot
// dispatch path (dispatch never calls resolve_policy(), see policy_table.hpp).
std::mutex g_miss_examples_mutex;
std::vector<std::pair<std::string, std::string>> g_miss_examples;

/// The three-way slash-tolerant comparison node_binding.cpp:234-238 uses when
/// matching a RID entry's declared topic against a live subscription's resolved
/// name.  Reused verbatim so config keys, RID entries and live topics agree.
bool topic_matches(const std::string & a, const std::string & b)
{
  return a == b || a == "/" + b || "/" + a == b;
}

/// Parse a `take_mode` string strictly (unlike the env-var parser, an unknown
/// value in a generated file is a fail-closed error, not a warn-and-default).
bool parse_mode_strict(const std::string & s, TakeMode * out)
{
  if (s == "skip") {
    *out = TakeMode::Skip;
    return true;
  }
  if (s == "block") {
    *out = TakeMode::Block;
    return true;
  }
  return false;
}

/// Read the take_mode / take_timeout_ms keys of one JSON object into `policy`,
/// leaving unset keys at their incoming value.  Returns false with `error` set
/// on a bad type or a bad value.
bool read_policy_keys(
  const nlohmann::json & obj, const std::string & where, TakePolicy * policy,
  std::string * error)
{
  if (obj.contains("take_mode")) {
    const nlohmann::json & v = obj.at("take_mode");
    if (!v.is_string()) {
      *error = where + ": 'take_mode' must be a string";
      return false;
    }
    if (!parse_mode_strict(v.get<std::string>(), &policy->take_mode)) {
      *error = where + ": unknown 'take_mode' value '" + v.get<std::string>() +
        "' (expected 'skip' or 'block')";
      return false;
    }
  }
  if (obj.contains("take_timeout_ms")) {
    const nlohmann::json & v = obj.at("take_timeout_ms");
    if (!v.is_number_integer()) {
      *error = where + ": 'take_timeout_ms' must be an integer";
      return false;
    }
    const int64_t parsed = v.get<int64_t>();
    if (parsed < 0) {
      *error = where + ": 'take_timeout_ms' must be >= 0";
      return false;
    }
    policy->take_timeout_ms = parsed;
  }
  return true;
}

}  // namespace

PolicyTableLoadResult policy_table_load(const std::string & path)
{
  PolicyTableLoadResult result;

  if (path.empty()) {
    result.file_present = false;
    return result;
  }

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    // Absent file -> built-in defaults, silently (contract item 4).
    result.file_present = false;
    return result;
  }
  result.file_present = true;

  nlohmann::json doc;
  try {
    doc = nlohmann::json::parse(ifs);
  } catch (const nlohmann::json::exception & e) {
    result.ok = false;
    result.error = path + ": not valid JSON (" + e.what() + ")";
    return result;
  }

  if (!doc.is_object()) {
    result.ok = false;
    result.error = path + ": top level must be a JSON object";
    return result;
  }

  if (!doc.contains("schema_version") || !doc.at("schema_version").is_number_integer()) {
    result.ok = false;
    result.error = path + ": missing integer 'schema_version'";
    return result;
  }
  const int64_t version = doc.at("schema_version").get<int64_t>();
  if (version != kExecConfigSchemaVersion) {
    result.ok = false;
    result.error = path + ": unsupported schema_version " + std::to_string(version) +
      " (this binding understands " + std::to_string(kExecConfigSchemaVersion) + ")";
    return result;
  }

  // Built-in defaults, then the file's `defaults` block on top of them.
  TakePolicy defaults;
  if (doc.contains("defaults")) {
    const nlohmann::json & d = doc.at("defaults");
    if (!d.is_object()) {
      result.ok = false;
      result.error = path + ": 'defaults' must be an object";
      return result;
    }
    if (!read_policy_keys(d, path + ": defaults", &defaults, &result.error)) {
      result.ok = false;
      return result;
    }
  }

  std::vector<PolicyRow> rows;
  if (doc.contains("subscriptions")) {
    const nlohmann::json & subs = doc.at("subscriptions");
    if (!subs.is_array()) {
      result.ok = false;
      result.error = path + ": 'subscriptions' must be an array";
      return result;
    }
    rows.reserve(subs.size());
    size_t index = 0;
    for (const auto & item : subs) {
      const std::string where = path + ": subscriptions[" + std::to_string(index) + "]";
      index++;
      if (!item.is_object()) {
        result.ok = false;
        result.error = where + ": must be an object";
        return result;
      }
      if (!item.contains("node_class") || !item.at("node_class").is_string()) {
        result.ok = false;
        result.error = where + ": missing string 'node_class'";
        return result;
      }
      if (!item.contains("topic") || !item.at("topic").is_string()) {
        result.ok = false;
        result.error = where + ": missing string 'topic'";
        return result;
      }
      PolicyRow row;
      row.node_class = item.at("node_class").get<std::string>();
      row.topic = item.at("topic").get<std::string>();
      // Rows inherit the table defaults for keys they do not set; Python has
      // already done the per-key merge, so this is belt-and-braces.
      row.policy = defaults;
      if (!read_policy_keys(item, where, &row.policy, &result.error)) {
        result.ok = false;
        return result;
      }
      rows.push_back(row);
    }
  }

  // Commit only after full validation: a rejected file never half-installs.
  g_defaults = defaults;
  g_rows = rows;
  g_misses.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_miss_examples_mutex);
    g_miss_examples.clear();
  }
  return result;
}

TakePolicy resolve_policy(
  const std::string & node_class, const std::string & declared_topic,
  const std::string & resolved_topic)
{
  for (const auto & row : g_rows) {
    if (row.node_class != node_class) {
      continue;
    }
    if (topic_matches(row.topic, declared_topic) ||
      (!resolved_topic.empty() && topic_matches(row.topic, resolved_topic)))
    {
      return row.policy;
    }
  }
  g_misses.fetch_add(1, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_miss_examples_mutex);
    if (g_miss_examples.size() < kMaxMissExamples) {
      g_miss_examples.emplace_back(node_class, declared_topic);
    }
  }
  return g_defaults;
}

TakePolicy resolve_policy(const std::string & node_class, const std::string & topic)
{
  return resolve_policy(node_class, topic, std::string());
}

TakePolicy policy_table_defaults()
{
  return g_defaults;
}

size_t policy_table_row_count()
{
  return g_rows.size();
}

uint64_t policy_table_miss_count()
{
  return g_misses.load(std::memory_order_relaxed);
}

const std::vector<std::pair<std::string, std::string>> & policy_table_miss_examples()
{
  const std::lock_guard<std::mutex> lock(g_miss_examples_mutex);
  return g_miss_examples;
}

const std::vector<PolicyRow> & policy_table_rows()
{
  return g_rows;
}

void policy_table_set_for_test(const TakePolicy & defaults, const std::vector<PolicyRow> & rows)
{
  g_defaults = defaults;
  g_rows = rows;
  g_misses.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_miss_examples_mutex);
    g_miss_examples.clear();
  }
}

void policy_table_reset()
{
  g_defaults = TakePolicy();
  g_rows.clear();
  g_misses.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_miss_examples_mutex);
    g_miss_examples.clear();
  }
}

namespace
{

/// The override WARN for one env var. `differs(row)` says whether the env
/// value changes that row, `defaults_differ` whether it changes the defaults
/// entry. Rows and defaults are counted apart so "N of M" counts rows only.
/// Rows on a declared external topic are left out: they dispatch on arrival
/// and never take.
template<typename Differs>
std::string override_message(
  const std::string & var_and_value, const char * key, Differs differs, bool defaults_differ)
{
  size_t total = 0;
  size_t changed = 0;
  size_t external = 0;
  for (const auto & row : g_rows) {
    if (external_input::is_declared_topic(row.topic)) {
      external++;
      continue;
    }
    total++;
    if (differs(row)) {
      changed++;
    }
  }
  std::string msg = var_and_value + " overrides " + key + " for " + std::to_string(changed) +
    " of " + std::to_string(total) + " configured subscription(s)" +
    (defaults_differ ?
    " and for the defaults, which every subscription without its own entry uses" :
    ", not for the defaults, which already match");
  if (external > 0) {
    msg += ", leaving out " + std::to_string(external) +
      " external_topics subscription(s), which dispatch on arrival";
  }
  return msg;
}

}  // namespace

std::string policy_table_take_mode_override_message()
{
  const auto env_mode = take_mode_env();
  if (!env_mode) {
    return std::string();
  }
  return override_message(
    std::string("ROS2_LF_TAKE_MODE=") + (*env_mode == TakeMode::Block ? "block" : "skip"),
    "take_mode", [&](const PolicyRow & row) {return row.policy.take_mode != *env_mode;},
    g_defaults.take_mode != *env_mode);
}

std::string policy_table_take_timeout_override_message()
{
  // ROS2_LF_MAXWAIT_MS, with ROS2_LF_TAKE_TIMEOUT_MS as its alias.
  // Two disagreeing values are refused (take_mode.cpp maxwait_env): the WARN
  // then names both and nothing is overridden.
  const MaxwaitEnv env = maxwait_env();
  if (!env.conflict.empty()) {
    return env.conflict;
  }
  if (!env.value) {
    return std::string();
  }
  const int64_t env_timeout = *env.value;
  // An admitted correspondence edge takes its timeout from the generated
  // binding header and reads the env var only when that value is negative,
  // which the generator never emits (corr_runtime.cpp, add_edge call). The
  // rows counted here are the policy rows, so say which edges keep theirs.
  return override_message(
    env.source, "maxwait_ms",
    [&](const PolicyRow & row) {return row.policy.take_timeout_ms != env_timeout;},
    g_defaults.take_timeout_ms != env_timeout) +
         ". An admitted correspondence edge keeps its generated timeout, which this variable "
         "does not change";
}

void policy_table_log_env_overrides()
{
  // Per-key env overrides (design §2.2): each var overrides only its own key,
  // and the WARN names how many resolved entries the override actually changes
  // so a flattened per-subscription config is auditable.
  for (const std::string & msg :
    {policy_table_take_mode_override_message(), policy_table_take_timeout_override_message()})
  {
    if (!msg.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("ros2_lf_binding"), "%s", msg.c_str());
    }
  }
}

void policy_table_log_miss_summary()
{
  const uint64_t misses = policy_table_miss_count();
  if (misses == 0) {
    return;
  }

  std::string examples;
  for (const auto & [node_class, topic] : policy_table_miss_examples()) {
    if (!examples.empty()) {
      examples += ", ";
    }
    examples += "(" + node_class + ", " + topic + ")";
  }

  RCLCPP_WARN(
    rclcpp::get_logger("ros2_lf_binding"),
    "exec_config policy lookup missed %lu subscription(s); falling back to defaults for "
    "them. Examples: %s. This usually means the (node_class, topic) keys in exec_config.json "
    "no longer match the model — re-run `colcon lf-analyze && colcon lf-configure`.",
    static_cast<unsigned long>(misses), examples.c_str());  // NOLINT(runtime/int)
}

}  // namespace ros2_lf_binding

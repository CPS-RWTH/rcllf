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

#include "sentry_pipeline/scenario_env.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sentry_pipeline
{

namespace
{
std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

// A set variable names the scenario this run claims to use.
// Running on the declared defaults instead would record wrong numbers.
[[noreturn]] void refuse(const char * path, const std::string & why)
{
  const std::string message = std::string("SENTRY_SCENARIO is set but ") + why + ": " + path +
    " (a relative path resolves against the process's working directory)";
  std::cerr << "[ERROR] [sentry_pipeline]: " << message << std::endl;
  throw std::runtime_error(message);
}
}  // namespace

ScenarioOverlay ScenarioOverlay::load_from_env()
{
  ScenarioOverlay overlay;
  const char * path = std::getenv("SENTRY_SCENARIO");
  if (path == nullptr) {
    return overlay;
  }
  // An ifstream opens a directory on Linux and only fails on the read.
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) {
    refuse(path, "is not a readable regular file");
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    refuse(path, "cannot be opened");
  }
  std::string line;
  while (std::getline(file, line)) {
    const std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed.front() == '#') {
      continue;
    }
    const auto eq = trimmed.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = trim(trimmed.substr(0, eq));
    const std::string value = trim(trimmed.substr(eq + 1));
    if (!key.empty()) {
      overlay.values_[key] = value;
    }
  }
  if (file.bad()) {
    refuse(path, "cannot be read");
  }
  return overlay;
}

std::optional<std::string> ScenarioOverlay::get(
  const std::string & node_name, const std::string & key) const
{
  const auto it = values_.find(node_name + "." + key);
  if (it == values_.end()) {
    return std::nullopt;
  }
  return it->second;
}

namespace
{
rclcpp::QoS qos_from_overlay_impl(
  rclcpp::Node & node, const ScenarioOverlay & overlay, const std::string & node_name,
  const std::string & reliability_param)
{
  int64_t depth = node.get_parameter("qos_depth").as_int();
  const auto overlay_depth = overlay.get(node_name, "qos_depth");
  if (overlay_depth.has_value()) {
    try {
      depth = std::stoll(*overlay_depth);
    } catch (const std::exception &) {
      // Keep the parameter/default value on a malformed overlay entry.
    }
  }

  std::string reliability_setting = node.get_parameter(reliability_param).as_string();
  const auto overlay_reliability = overlay.get(node_name, reliability_param);
  if (overlay_reliability.has_value()) {
    reliability_setting = *overlay_reliability;
  }

  rclcpp::QoS qos_profile(static_cast<size_t>(depth));
  if (reliability_setting == "best_effort") {
    qos_profile.best_effort();
  } else {
    qos_profile.reliable();
  }
  return qos_profile;
}
}  // namespace

rclcpp::QoS qos_from_overlay(
  rclcpp::Node & node, const ScenarioOverlay & overlay, const std::string & node_name)
{
  return qos_from_overlay_impl(node, overlay, node_name, "qos_reliability");
}

rclcpp::QoS qos_from_overlay(
  rclcpp::Node & node, const ScenarioOverlay & overlay, const std::string & node_name,
  const std::string & reliability_param)
{
  return qos_from_overlay_impl(node, overlay, node_name, reliability_param);
}

int64_t int_from_overlay(
  const ScenarioOverlay & overlay, const std::string & node_name, const std::string & key,
  int64_t param_value)
{
  const auto overlay_value = overlay.get(node_name, key);
  if (!overlay_value.has_value()) {
    return param_value;
  }
  try {
    return std::stoll(*overlay_value);
  } catch (const std::exception &) {
    return param_value;
  }
}

}  // namespace sentry_pipeline

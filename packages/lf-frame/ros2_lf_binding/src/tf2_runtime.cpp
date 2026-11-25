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


/// @file tf2_runtime.cpp
/// @brief Registry and wake bookkeeping of rcllf_tf2::AcquisitionLookup
///.
///
/// Adapters register inside the construction scope, keyed by binding key;
/// a node may hold several. Every hook runs under the registry mutex, so
/// ros2_lf_tf2_unregister returning means no hook of that handle runs again.
/// The mutex is recursive because a hook calls back into request_wake and
/// trace on the same thread. Same wake protocol as the BNA policy
/// (bna_runtime.cpp): a request is a logical deadline; generated code turns
/// each one into an lf_schedule of the node's tf2_wake action.

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "corr_runtime.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/tf2_runtime_api.h"
#include "semantic_trace.hpp"
#include "tf2_ingress.hpp"
#include "thread_role.hpp"

namespace
{

struct Adapter
{
  std::string key;
  bool registered = false;
  rcllf_tf2_hooks hooks{};
};

struct NodeWakes
{
  std::set<int64_t> requested;  // wake deadlines not yet scheduled
  std::set<int64_t> scheduled;  // wake deadlines with a tf2_wake in flight
};

struct Registry
{
  std::recursive_mutex mu;
  std::vector<Adapter> adapters;           // handle -> adapter
  std::map<std::string, NodeWakes> nodes;  // binding key -> wakes
  // binding key -> execution.yaml nodes.<node>.tf2
  std::map<std::string, std::pair<int64_t, std::string>> configs;
};

Registry & reg()
{
  static Registry * r = new Registry();
  return *r;
}

template<typename Hook>
void for_each_adapter(const char * key, Hook && hook)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  const std::string k = key ? key : "";
  const auto tag = ros2_lf_binding::lf_current_tag();
  // By index: a hook may register nothing, but it may request wakes.
  for (std::size_t i = 0; i < r.adapters.size(); ++i) {
    if (r.adapters[i].registered && r.adapters[i].key == k) {
      hook(r.adapters[i].hooks, tag.first, tag.second);
    }
  }
}

/// The tf_request / tf_outcome JSON line of the
/// adapter, as an rcllf schema 1.8 event (adapter and interval since 1.8). The
/// adapter's ABI stays the JSON string; a line that is not one of the two, or
/// lacks a field, emits nothing.
void emit_semantic(const char * json)
{
  const nlohmann::json j = nlohmann::json::parse(json, nullptr, false);
  if (!j.is_object() || !j.contains("event")) {
    return;
  }
  // A null end of the interval (tf2 cannot connect the frames) is -1, an
  // "unavailable" one (no exact range: a reparented link, a dynamic link
  // stamped only at 0; rcllf_tf2::AcquisitionLookup::available()) is -2.
  auto ns_or_none = [&j](const char * key) -> int64_t {
      const auto & v = j.at(key);
      if (v.is_string() && v.get<std::string>() == "unavailable") {
        return -2;
      }
      return v.is_null() ? -1 : v.get<int64_t>();
    };
  try {
    const std::string ev = j.at("event").get<std::string>();
    const std::string node = j.at("node").get<std::string>();
    if (ev == "tf_request") {
      const std::string target = j.at("target").get<std::string>();
      const std::string source = j.at("source").get<std::string>();
      const std::string on_timeout = j.at("on_timeout").get<std::string>();
      rcllf_tp_tf2_request e{};
      e.node = node.c_str();
      e.id = j.at("id").get<uint64_t>();
      e.target = target.c_str();
      e.source = source.c_str();
      e.stamp_ns = j.at("stamp").get<int64_t>();
      e.tag_time = j.at("time").get<int64_t>();
      e.tag_microstep = j.at("micro").get<uint32_t>();
      e.wait_bound_ns = j.at("wait_bound_ns").get<int64_t>();
      e.on_timeout = on_timeout.c_str();
      e.adapter = j.at("adapter").get<uint32_t>();
      e.oldest_common_ns = ns_or_none("oldest_common");
      e.latest_common_ns = ns_or_none("latest_common");
      ros2_lf_binding::semtrace::tf2_request(e);
    } else if (ev == "tf_outcome") {
      const std::string outcome = j.at("outcome").get<std::string>();
      rcllf_tp_tf2_outcome e{};
      e.node = node.c_str();
      e.id = j.at("id").get<uint64_t>();
      e.outcome = outcome.c_str();
      e.stamp_ns = j.at("stamp").get<int64_t>();
      e.tag_time = j.at("time").get<int64_t>();
      e.tag_microstep = j.at("micro").get<uint32_t>();
      e.wait_logical_ns = j.at("wait_logical_ns").get<int64_t>();
      e.wait_physical_ns = j.at("wait_physical_ns").get<int64_t>();
      e.latest_common_ns = ns_or_none("latest_common");
      e.adapter = j.at("adapter").get<uint32_t>();
      e.oldest_common_ns = ns_or_none("oldest_common");
      ros2_lf_binding::semtrace::tf2_outcome(e);
    }
  } catch (const nlohmann::json::exception &) {
    // Not the adapter's shape: the stderr line above still carries it.
  }
}

}  // namespace

namespace ros2_lf_binding
{
namespace tf2_rt
{
void reset_for_test()
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.adapters.clear();
  r.nodes.clear();
}
}  // namespace tf2_rt
}  // namespace ros2_lf_binding

namespace ros2_lf_binding
{
void register_tf2_config(
  const std::string & node_class, int64_t wait_bound_ns, const std::string & on_timeout)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.configs[node_class] = {wait_bound_ns, on_timeout};
}
}  // namespace ros2_lf_binding

extern "C" {

int ros2_lf_tf2_active(void)
{
  return !ros2_lf_binding::corr_rt::construct_key().empty() ||
         ros2_lf_binding::lf_current_tag().first >= 0;
}

int ros2_lf_tf2_register(const rcllf_tf2_hooks * hooks)
{
  const std::string & key = ros2_lf_binding::corr_rt::construct_key();
  if (key.empty() || hooks == nullptr) {
    return RCLLF_TF2_NOT_CONSTRUCTING;
  }
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.adapters.push_back(Adapter{key, true, *hooks});
  r.nodes[key];
  return static_cast<int>(r.adapters.size() - 1);
}

int ros2_lf_tf2_config(int handle, int64_t * wait_bound_ns, char * on_timeout, uint32_t cap)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (handle < 0 || handle >= static_cast<int>(r.adapters.size())) {
    return 0;
  }
  const auto it = r.configs.find(r.adapters[handle].key);
  if (it == r.configs.end()) {
    return 0;
  }
  if (wait_bound_ns != nullptr) {
    *wait_bound_ns = it->second.first;
  }
  if (on_timeout != nullptr && cap > 0) {
    const std::string & s = it->second.second;
    if (s.size() >= cap) {
      return -1;  // the adapter's buffer cannot hold the name: refuse, do not truncate
    }
    std::memcpy(on_timeout, s.c_str(), s.size() + 1);
  }
  return 1;
}

void ros2_lf_tf2_unregister(int handle)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (handle >= 0 && handle < static_cast<int>(r.adapters.size())) {
    r.adapters[handle].registered = false;
    r.adapters[handle].hooks = rcllf_tf2_hooks{};
  }
}

int ros2_lf_tf2_current(int64_t * time, uint32_t * micro)
{
  // lf_current_tag() is the first environment's tag on every thread; only an
  // LF worker is inside a reaction at that tag (review finding F3).
  const auto tag = ros2_lf_binding::lf_current_tag();
  if (tag.first < 0 || !ros2_lf_binding::is_lf_worker_thread()) {
    return -1;
  }
  if (time != nullptr) {
    *time = tag.first;
  }
  if (micro != nullptr) {
    *micro = tag.second;
  }
  return 0;
}

void ros2_lf_tf2_request_wake(int handle, int64_t deadline)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (handle < 0 || handle >= static_cast<int>(r.adapters.size()) ||
    !r.adapters[handle].registered)
  {
    return;
  }
  NodeWakes & n = r.nodes[r.adapters[handle].key];
  if (n.scheduled.count(deadline) == 0) {
    n.requested.insert(deadline);
  }
}

void ros2_lf_tf2_trace(const char * json)
{
  if (json == nullptr) {
    return;
  }
  std::fprintf(stderr, "RCLLF_TF2 %s\n", json);
  std::fflush(stderr);
  if (ros2_lf_binding::trace_callbacks_enabled()) {
    RCLCPP_INFO(rclcpp::get_logger("ros2_lf_binding.trace"), "RCLLF_TF2 %s", json);
  }
  emit_semantic(json);
}

int64_t ros2_lf_tf2_take_wake(const char * node_key)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto it = r.nodes.find(node_key ? node_key : "");
  if (it == r.nodes.end() || it->second.requested.empty()) {
    return -1;
  }
  NodeWakes & n = it->second;
  const int64_t d = *n.requested.begin();
  n.requested.erase(n.requested.begin());
  n.scheduled.insert(d);
  return std::max<int64_t>(0, d - ros2_lf_binding::lf_current_tag().first);
}

void ros2_lf_tf2_wake(const char * node_key)
{
  {
    Registry & r = reg();
    std::lock_guard<std::recursive_mutex> lk(r.mu);
    auto it = r.nodes.find(node_key ? node_key : "");
    if (it != r.nodes.end()) {
      auto & s = it->second.scheduled;
      s.erase(s.begin(), s.upper_bound(ros2_lf_binding::lf_current_tag().first));
    }
  }
  for_each_adapter(
    node_key, [](const rcllf_tf2_hooks & h, int64_t t, uint32_t m) {
      if (h.wake != nullptr) {h.wake(h.ctx, t, m);}
    });
}

void ros2_lf_tf2_ingested(const char * node_key)
{
  // Review finding F2: the listener group must still be detached.
  for (const auto & key : ros2_lf_binding::reassert_tf2_groups_detached()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "tf2 ingress: node '%s': the TransformListener group was re-enabled outside "
      "LF and is detached again; /tf it took meanwhile reached the Buffer outside LF",
      key.c_str());
    const std::string json = "{\"event\":\"tf_ingress_reattached\",\"node\":\"" + key + "\"}";
    ros2_lf_tf2_trace(json.c_str());
  }
  for_each_adapter(
    node_key, [](const rcllf_tf2_hooks & h, int64_t t, uint32_t m) {
      if (h.ingested != nullptr) {h.ingested(h.ctx, t, m);}
    });
}

void ros2_lf_tf2_close(const char * node_key)
{
  for_each_adapter(
    node_key, [](const rcllf_tf2_hooks & h, int64_t t, uint32_t m) {
      if (h.close != nullptr) {h.close(h.ctx, t, m);}
    });
}

}  // extern "C"

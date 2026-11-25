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


/// @file bna_runtime.cpp
/// @brief Registry, dispatch context and wake bookkeeping of the BNA
/// synchronizer policy.
///
/// One policy per node, registered inside the construction scope and
/// keyed by binding key. Every hook runs under the registry mutex, so
/// ros2_lf_bna_unregister returning means no hook of that handle runs again.
/// The mutex is recursive because a hook calls back into request_wake and
/// trace on the same thread.

#include "bna_runtime.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "corr_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/bna_runtime_api.h"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
namespace bna_rt
{
namespace
{

struct Node
{
  int handle = -1;
  bool registered = false;
  rcllf_bna_hooks hooks{};
  rcllf_bna_missing_stamped_fn missing_stamped = nullptr;  // set after register
  std::set<int64_t> requested;  // wake deadlines not yet scheduled
  std::set<int64_t> scheduled;  // wake deadlines with a bna_wake in flight
};

struct Registry
{
  std::recursive_mutex mu;
  std::map<std::string, Node> nodes;          // binding key -> node
  std::vector<std::string> handles;           // handle -> binding key
  std::map<int, std::pair<std::string, int>> inputs;  // drain RID -> (key, input)
  // Drain RID -> declared max_stamp_lag (ns) of an lf_clock input.
  std::map<int, int64_t> stamp_lag;
};

Registry & reg()
{
  static Registry * r = new Registry();
  return *r;
}

struct Dispatch
{
  bool active = false;
  int rid = -1;
  std::string gid;
  uint64_t ordinal = 0;
};
thread_local Dispatch t_dispatch;
thread_local int t_hook_depth = 0;  // >0 inside a binding-originated hook call

Node * node_of_handle(int handle)
{
  Registry & r = reg();
  if (handle < 0 || handle >= static_cast<int>(r.handles.size())) {
    return nullptr;
  }
  auto it = r.nodes.find(r.handles[handle]);
  // A handle from before an unregister and re-register under the same key is
  // stale and must not reach the replacement node.
  if (it == r.nodes.end() || !it->second.registered || it->second.handle != handle) {
    return nullptr;
  }
  return &it->second;
}

Node * node_of_key(const char * key)
{
  auto it = reg().nodes.find(key ? key : "");
  return it == reg().nodes.end() || !it->second.registered ? nullptr : &it->second;
}

[[noreturn]] void refuse(const char * what)
{
  std::fprintf(stderr, "RCLLF_BNA refused: %s\n", what);
  std::fflush(stderr);
  std::_Exit(3);  // plan risk 2: a defect, never a silent nondeterministic path
}

struct HookDepth
{
  HookDepth() {++t_hook_depth;}
  ~HookDepth() {--t_hook_depth;}
};

}  // namespace

DispatchScope::DispatchScope(int rid, const std::string & gid, uint64_t ordinal)
{
  t_dispatch = Dispatch{true, rid, gid, ordinal};
}

DispatchScope::~DispatchScope() {t_dispatch = Dispatch{};}

bool on_missing(int rid, const std::string & gid, uint64_t ordinal, int64_t pub_tag)
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto in = r.inputs.find(rid);
  if (in == r.inputs.end()) {
    return false;
  }
  Node * n = node_of_key(in->second.first.c_str());
  if (n == nullptr || n->hooks.missing == nullptr) {
    return false;
  }
  const auto tag = lf_current_tag();
  HookDepth depth;
  // Only an input with a declared max_stamp_lag has a stamp bound. The
  // publish tag alone is not one: an lf_clock stamp may be taken in an earlier
  // reaction or forwarded by a relay, so it can lie below the publish tag.
  const auto lag = r.stamp_lag.find(rid);
  if (n->missing_stamped != nullptr && pub_tag >= 0 && lag != r.stamp_lag.end() &&
    pub_tag - lag->second >= 0)
  {
    n->missing_stamped(
      n->hooks.ctx, in->second.second, corr_rt::table().epoch(), gid.c_str(), ordinal,
      pub_tag - lag->second, tag.first, tag.second);
    return true;
  }
  n->hooks.missing(
    n->hooks.ctx, in->second.second, corr_rt::table().epoch(), gid.c_str(), ordinal,
    tag.first, tag.second);
  return true;
}

void reset_for_test()
{
  Registry & r = reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.nodes.clear();
  r.handles.clear();
  r.inputs.clear();
  r.stamp_lag.clear();
}

}  // namespace bna_rt
}  // namespace ros2_lf_binding

namespace bna_rt = ros2_lf_binding::bna_rt;

extern "C" {

int ros2_lf_bna_active(void)
{
  return !ros2_lf_binding::corr_rt::construct_key().empty() ||
         ros2_lf_binding::lf_current_tag().first >= 0;
}

int ros2_lf_bna_register(const rcllf_bna_hooks * hooks, uint64_t * epoch)
{
  const std::string & key = ros2_lf_binding::corr_rt::construct_key();
  if (key.empty() || hooks == nullptr) {
    return RCLLF_BNA_NOT_CONSTRUCTING;
  }
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto & n = r.nodes[key];
  if (n.registered) {
    return RCLLF_BNA_MULTIPLE;
  }
  n.registered = true;
  // Exactly the 4-pointer struct: nothing past it is read.
  n.hooks = *hooks;
  n.missing_stamped = nullptr;
  n.requested.clear();
  n.scheduled.clear();
  n.handle = static_cast<int>(r.handles.size());
  r.handles.push_back(key);
  if (epoch != nullptr) {
    *epoch = ros2_lf_binding::corr_rt::table().epoch();
  }
  return n.handle;
}

void ros2_lf_bna_set_missing_stamped(int handle, rcllf_bna_missing_stamped_fn fn)
{
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (auto * n = bna_rt::node_of_handle(handle)) {
    n->missing_stamped = fn;
  }
}

void ros2_lf_bna_unregister(int handle)
{
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (auto * n = bna_rt::node_of_handle(handle)) {
    n->registered = false;
    n->hooks = rcllf_bna_hooks{};
    n->missing_stamped = nullptr;
    n->requested.clear();
    n->scheduled.clear();
  }
}

int ros2_lf_bna_current(int handle, int input, rcllf_bna_dispatch * out)
{
  const auto & d = bna_rt::t_dispatch;
  if (!d.active) {
    return RCLLF_BNA_NOT_DRAIN;
  }
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto in = r.inputs.find(d.rid);
  if (bna_rt::node_of_handle(handle) == nullptr || in == r.inputs.end() ||
    in->second.first != r.handles[handle] || in->second.second != input)
  {
    return RCLLF_BNA_WRONG_INPUT;
  }
  const auto tag = ros2_lf_binding::lf_current_tag();
  if (out != nullptr) {
    out->time = tag.first;
    out->micro = tag.second;
    out->epoch = ros2_lf_binding::corr_rt::table().epoch();
    out->ordinal = d.ordinal;
    std::memset(out->gid, 0, sizeof(out->gid));
    std::strncpy(out->gid, d.gid.c_str(), sizeof(out->gid) - 1);
  }
  return RCLLF_BNA_OK;
}

void ros2_lf_bna_request_wake(int handle, int64_t deadline)
{
  // Plan risk 2: only from add<i> in a drain or from a binding hook.
  if (!bna_rt::t_dispatch.active && bna_rt::t_hook_depth == 0) {
    bna_rt::refuse("wake requested outside a declared drain or BNA reaction");
  }
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  if (auto * n = bna_rt::node_of_handle(handle)) {
    if (n->scheduled.count(deadline) == 0) {
      n->requested.insert(deadline);
    }
  }
}

void ros2_lf_bna_trace(const char * json)
{
  if (json == nullptr) {
    return;
  }
  std::fprintf(stderr, "RCLLF_BNA %s\n", json);
  if (ros2_lf_binding::trace_callbacks_enabled()) {
    RCLCPP_INFO(rclcpp::get_logger("ros2_lf_binding.trace"), "RCLLF_BNA %s", json);
  }
}

void ros2_lf_bna_declare_input(const char * node_key, int reaction_id, int input)
{
  if (node_key == nullptr || (input != 0 && input != 1)) {
    bna_rt::refuse("ros2_lf_bna_declare_input with a null key or an input other than 0 or 1");
  }
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.inputs[reaction_id] = {node_key, input};
}

void ros2_lf_bna_input_stamp_lag(int reaction_id, int64_t max_stamp_lag_ns)
{
  if (max_stamp_lag_ns < 0) {
    return;
  }
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  r.stamp_lag[reaction_id] = max_stamp_lag_ns;
}

int64_t ros2_lf_bna_take_wake(const char * node_key)
{
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto * n = bna_rt::node_of_key(node_key);
  if (n == nullptr || n->requested.empty()) {
    return -1;
  }
  const int64_t d = *n->requested.begin();
  n->requested.erase(n->requested.begin());
  n->scheduled.insert(d);
  return std::max<int64_t>(0, d - ros2_lf_binding::lf_current_tag().first);
}

void ros2_lf_bna_wake(const char * node_key)
{
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto * n = bna_rt::node_of_key(node_key);
  if (n == nullptr) {
    return;
  }
  const auto tag = ros2_lf_binding::lf_current_tag();
  n->scheduled.erase(n->scheduled.begin(), n->scheduled.upper_bound(tag.first));
  if (n->hooks.wake != nullptr) {
    bna_rt::HookDepth depth;
    n->hooks.wake(n->hooks.ctx, tag.first, tag.second);
  }
}

void ros2_lf_bna_close(const char * node_key)
{
  auto & r = bna_rt::reg();
  std::lock_guard<std::recursive_mutex> lk(r.mu);
  auto * n = bna_rt::node_of_key(node_key);
  if (n == nullptr || n->hooks.close == nullptr) {
    return;
  }
  const auto tag = ros2_lf_binding::lf_current_tag();
  bna_rt::HookDepth depth;
  n->hooks.close(n->hooks.ctx, "shutdown", tag.first, tag.second);
}

}  // extern "C"

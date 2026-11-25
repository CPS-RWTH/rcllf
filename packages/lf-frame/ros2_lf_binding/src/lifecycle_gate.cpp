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

#include "lifecycle_gate.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ingress.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/publication_gate_api.h"
#include "thread_role.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{

bool is_primary_lifecycle_state(int state)
{
  return state >= 1 && state <= 4;
}

void LifecycleGate::promote(LfTag tag)
{
  if (pending_ && pending_->tag < tag) {
    effective_ = pending_->state;
    ++epoch_;
    pending_.reset();
  }
}

void LifecycleGate::observe(uint8_t state, LfTag tag)
{
  if (!is_primary_lifecycle_state(state)) {
    return;
  }
  promote(tag);
  const uint8_t latest = pending_ ? pending_->state : effective_;
  if (state == latest) {
    return;
  }
  // A return to the effective state within this tag is still a change: the
  // hooks that ran on the way may have replaced the node's handles.
  pending_ = Pending{state, tag};
}

void LifecycleGate::touch(LfTag tag)
{
  promote(tag);
  if (!pending_) {
    pending_ = Pending{effective_, tag};
  }
}

LifecycleGate::View LifecycleGate::at(LfTag tag)
{
  promote(tag);
  return View{effective_, epoch_};
}

namespace
{

struct GateSlot
{
  LifecycleGate gate;
  /// Strong references to the node's entities, taken at the last boundary.
  std::vector<std::shared_ptr<void>> pins;
  /// Per pinned entity, its generation: the boundary index it was first
  /// pinned at.
  std::map<const void *, uint64_t> generation;
  /// The rcl impls of the node's change_state services, one of them that of
  /// its state machine, which names the node in the transitions
  /// the publication gate reports. Empty: the gate reads the node's state at
  /// callback exit and at the boundary instead.
  std::set<const void *> service_impls;
  /// The state machine of the first reported transition.
  const void * state_machine = nullptr;
  /// Transitions made outside LF that found no ingress: the next boundary
  /// step records them.
  std::vector<uint8_t> unrouted;
  /// Active at the last boundary step: a step that finds the node no longer
  /// active runs the deactivate hook.
  bool active = false;
};

std::mutex g_gate_mutex;
// Never destroyed: a pin released by a static destructor would destroy an
// rclcpp entity after rclcpp's own statics are gone. The runtime releases the
// pins before rclcpp shuts down (runtime.cpp).
std::map<std::string, GateSlot> & g_gates = *new std::map<std::string, GateSlot>();
std::atomic<bool> g_any_gate{false};

/// Serialises boundary steps. Taken before g_gate_mutex, never after it.
std::mutex g_boundary_mutex;
std::optional<LfTag> g_last_boundary;
/// Boundary steps run so far, 0 at registration. Written under both mutexes.
uint64_t g_boundary_index = 0;
LifecycleEntityCollector g_collector;
LifecycleDeactivateHook g_deactivate_hook;

constexpr uint8_t kActive = 3;  // lifecycle_msgs/msg/State PRIMARY_STATE_ACTIVE

/// A LifecyclePublisher flag (SimpleManagedEntity) changed at `tag`:
/// the value it had before, which the other reactions of the tag read,
/// and the user callbacks that changed it, which read the live flag.
struct HeldFlag
{
  LfTag tag;
  bool value;
  std::vector<uint64_t> writers;
};
/// Serialises a flag change with its record, and a read with its lookup, so
/// a read never sees a changed flag without its record.
std::mutex g_flag_mutex;
std::map<const void *, HeldFlag> g_held_flags;

/// Forget the flags held for tags before `tag`. Call with g_flag_mutex held.
void prune_held_flags(LfTag tag)
{
  for (auto it = g_held_flags.begin(); it != g_held_flags.end(); ) {
    it = it->second.tag < tag ? g_held_flags.erase(it) : std::next(it);
  }
}

/// Pin every entity in `found` that the slot lacks, with generation
/// g_boundary_index. Returns true when one was added. Call with g_gate_mutex
/// held.
bool pin_new(GateSlot & slot, std::vector<std::shared_ptr<void>> & found)
{
  bool added = false;
  for (auto & f : found) {
    if (slot.generation.emplace(f.get(), g_boundary_index).second) {
      slot.pins.push_back(std::move(f));
      added = true;
    }
  }
  return added;
}

/// True when the executable's publication gate is on the path of both rcl
/// lifecycle transition calls, as corr_runtime.cpp checks the publish path.
bool transition_hook_installed()
{
  using SymFn = void * (*)(const char *);
  auto gate_symbol = reinterpret_cast<SymFn>(dlsym(RTLD_DEFAULT, "rcllf_gate_symbol"));
  if (gate_symbol == nullptr) {
    return false;
  }
  for (const char * sym :
    {"rcl_lifecycle_trigger_transition_by_id", "rcl_lifecycle_trigger_transition_by_label"})
  {
    void * own = gate_symbol(sym);
    if (own == nullptr || dlsym(RTLD_DEFAULT, sym) != own) {
      return false;
    }
  }
  return true;
}

}  // namespace

void lifecycle_gate_set_collector(LifecycleEntityCollector collector)
{
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  g_collector = std::move(collector);
}

void lifecycle_gate_set_deactivate_hook(LifecycleDeactivateHook hook)
{
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  g_deactivate_hook = std::move(hook);
}

void lifecycle_gate_register(
  const std::string & node_class, uint8_t state, const std::vector<const void *> & service_impls)
{
  static const bool hook = transition_hook_installed();
  LifecycleEntityCollector collector;
  {
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    collector = g_collector;
  }
  std::vector<std::shared_ptr<void>> found;
  if (collector) {
    collector(node_class, found);
  }
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  GateSlot slot{LifecycleGate(state), {}, {}, {}, nullptr, {}, state == kActive};
  if (hook) {
    slot.service_impls.insert(service_impls.begin(), service_impls.end());
    slot.service_impls.erase(nullptr);
  }
  if (hook && slot.service_impls.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "lifecycle gate %s: state machine not found, transitions read from the node's state "
      "at boundaries, without a physical action tag", node_class.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"), "lifecycle gate %s: transitions %s",
      node_class.c_str(), slot.service_impls.empty() ? "read from the node's state" :
      "reported by the publication gate");
  }
  pin_new(slot, found);
  g_gates.insert_or_assign(node_class, std::move(slot));
  g_any_gate.store(true);
}

void lifecycle_gate_on_transition(
  const void * state_machine, const void * change_state_impl, uint8_t state)
{
  if (!g_any_gate.load() || change_state_impl == nullptr || !is_primary_lifecycle_state(state)) {
    return;
  }
  std::string node;
  {
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    for (auto & kv : g_gates) {
      if (kv.second.service_impls.count(change_state_impl) == 0) {
        continue;
      }
      if (kv.second.state_machine == nullptr) {
        kv.second.state_machine = state_machine;
      }
      if (t_user_callback_depth > 0) {
        kv.second.gate.observe(state, lf_current_tag());
        return;
      }
      node = kv.first;
      break;
    }
  }
  if (node.empty()) {
    return;
  }
  // A thread LF does not own made the transition: external input, so it
  // enters LF as a physical action, the node's ingress wake.
  auto & ingress = IngressRole::instance();
  IngressRecord record;
  record.id = ingress.next_record_id();
  record.service_suffix = "lifecycle_transition";
  record.t_rx = std::chrono::steady_clock::now();
  record.local = [node, state]() {lifecycle_gate_apply_external(node, state);};
  if (ingress.on_request(node, std::move(record))) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  auto it = g_gates.find(node);
  if (it != g_gates.end()) {
    it->second.unrouted.push_back(state);
  }
}

void lifecycle_gate_apply_external(const std::string & node_class, uint8_t state)
{
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  auto it = g_gates.find(node_class);
  if (it != g_gates.end()) {
    it->second.gate.observe(state, lf_current_tag());
  }
}

void lifecycle_gate_observe_all()
{
  if (!g_any_gate.load(std::memory_order_relaxed)) {
    return;
  }
  std::vector<std::string> nodes;
  {
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    nodes.reserve(g_gates.size());
    for (const auto & kv : g_gates) {
      if (kv.second.service_impls.empty()) {
        nodes.push_back(kv.first);
      }
    }
  }
  const LfTag tag = lf_current_tag();
  for (const auto & node : nodes) {
    // Read unlocked: the state probe may wait for a transition another thread
    // is running.
    const int state = ros2_lf_node_lifecycle_state(node);
    if (!is_primary_lifecycle_state(state)) {
      continue;
    }
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    auto it = g_gates.find(node);
    if (it != g_gates.end()) {
      it->second.gate.observe(static_cast<uint8_t>(state), tag);
    }
  }
}

void lifecycle_gate_boundary()
{
  if (!g_any_gate.load(std::memory_order_relaxed)) {
    return;
  }
  const LfTag tag = lf_current_tag();
  std::lock_guard<std::mutex> step(g_boundary_mutex);
  if (g_last_boundary && !(*g_last_boundary < tag)) {
    return;
  }
  // What the step reads happened before this tag, so it counts as observed at
  // the previous boundary and holds from this tag on.
  const LfTag before = g_last_boundary.value_or(
    LfTag{std::numeric_limits<int64_t>::min(), 0});
  std::vector<std::pair<std::string, bool>> nodes;  // name, polled
  LifecycleEntityCollector collector;
  LifecycleDeactivateHook deactivate_hook;
  std::vector<std::string> deactivated;
  {
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    ++g_boundary_index;
    for (const auto & kv : g_gates) {
      nodes.emplace_back(kv.first, kv.second.service_impls.empty());
    }
    collector = g_collector;
    deactivate_hook = g_deactivate_hook;
  }
  for (const auto & [node, polled] : nodes) {
    const int state = polled ? ros2_lf_node_lifecycle_state(node) : -1;
    // A pin only the gate still holds is a handle the node dropped. Release
    // it before collecting, so a dropped entity is destroyed and not found.
    std::vector<std::shared_ptr<void>> dropped;
    {
      std::lock_guard<std::mutex> lock(g_gate_mutex);
      auto it = g_gates.find(node);
      if (it == g_gates.end()) {
        continue;
      }
      auto & pins = it->second.pins;
      for (auto p = pins.begin(); p != pins.end(); ) {
        if (p->use_count() == 1) {
          it->second.generation.erase(p->get());
          dropped.push_back(std::move(*p));
          p = pins.erase(p);
        } else {
          ++p;
        }
      }
    }
    const bool released = !dropped.empty();
    dropped.clear();  // destroyed here, outside the gate mutex
    std::vector<std::shared_ptr<void>> found;
    const bool alive = collector && collector(node, found);
    std::vector<std::shared_ptr<void>> orphaned;
    std::lock_guard<std::mutex> lock(g_gate_mutex);
    auto it = g_gates.find(node);
    if (it == g_gates.end()) {
      continue;
    }
    auto & slot = it->second;
    bool changed = released;
    if (alive) {
      changed = pin_new(slot, found) || changed;
    } else if (collector) {
      orphaned.swap(slot.pins);  // the node is gone, keep nothing of it
      slot.generation.clear();
    }
    if (is_primary_lifecycle_state(state)) {
      slot.gate.observe(static_cast<uint8_t>(state), before);
    }
    for (const uint8_t s : slot.unrouted) {
      slot.gate.observe(s, before);
    }
    slot.unrouted.clear();
    if (changed) {
      slot.gate.touch(before);
    }
    const bool active = slot.gate.at(tag).state == kActive;
    if (slot.active && !active) {
      deactivated.push_back(node);
    }
    slot.active = active;
  }
  g_last_boundary = tag;
  // Outside the gate mutex, before any user callback of this tag: the hook
  // reaches into rclcpp's client state.
  if (deactivate_hook) {
    for (const auto & node : deactivated) {
      deactivate_hook(node);
    }
  }
}

void lifecycle_gate_set_entity_flag(
  void * entity, const std::function<void()> & set, const std::function<bool()> & get)
{
  if (t_user_callback_depth == 0) {
    set();
    return;
  }
  const LfTag tag = lf_current_tag();
  std::lock_guard<std::mutex> lock(g_flag_mutex);
  prune_held_flags(tag);
  auto & held = g_held_flags.try_emplace(entity, HeldFlag{tag, get(), {}}).first->second;
  if (std::find(held.writers.begin(), held.writers.end(), t_user_callback_token) ==
    held.writers.end())
  {
    held.writers.push_back(t_user_callback_token);
  }
  set();
}

bool lifecycle_gate_entity_flag(const void * entity, const std::function<bool()> & get)
{
  if (t_user_callback_depth == 0) {
    return get();
  }
  const LfTag tag = lf_current_tag();
  std::lock_guard<std::mutex> lock(g_flag_mutex);
  prune_held_flags(tag);
  const auto it = g_held_flags.find(entity);
  if (it == g_held_flags.end()) {
    return get();
  }
  const auto & w = it->second.writers;
  if (std::find(w.begin(), w.end(), t_user_callback_token) != w.end()) {
    return get();
  }
  return it->second.value;
}

void lifecycle_gate_release_pins()
{
  std::vector<std::shared_ptr<void>> dropped;
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  for (auto & kv : g_gates) {
    for (auto & p : kv.second.pins) {
      dropped.push_back(std::move(p));
    }
    kv.second.pins.clear();
    kv.second.generation.clear();
  }
  // `dropped` dies after the lock: declared before it.
}

std::optional<LifecycleGate::View> lifecycle_gate_view(const std::string & node_class)
{
  if (!g_any_gate.load(std::memory_order_relaxed)) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  auto it = g_gates.find(node_class);
  if (it == g_gates.end()) {
    return std::nullopt;
  }
  return it->second.gate.at(lf_current_tag());
}

uint64_t lifecycle_gate_generation(const std::string & node_class, const void * entity)
{
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  auto it = g_gates.find(node_class);
  if (it != g_gates.end()) {
    const auto g = it->second.generation.find(entity);
    if (g != it->second.generation.end()) {
      return g->second;
    }
  }
  return g_boundary_index + 1;  // not pinned yet: the next boundary pins it
}

const void * lifecycle_gate_state_machine(const std::string & node_class)
{
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  auto it = g_gates.find(node_class);
  return it == g_gates.end() ? nullptr : it->second.state_machine;
}

void lifecycle_gate_reset_for_test()
{
  std::lock_guard<std::mutex> step(g_boundary_mutex);
  std::map<std::string, GateSlot> old;
  std::lock_guard<std::mutex> lock(g_gate_mutex);
  old.swap(g_gates);
  {
    std::lock_guard<std::mutex> flags(g_flag_mutex);
    g_held_flags.clear();
  }
  g_last_boundary.reset();
  g_boundary_index = 0;
  g_any_gate.store(false);
}

}  // namespace ros2_lf_binding

extern "C" void rcllf_gate_on_lifecycle_transition(const void * state_machine)
{
  const auto * sm = static_cast<const rcl_lifecycle_state_machine_t *>(state_machine);
  if (sm == nullptr || sm->current_state == nullptr) {
    return;
  }
  // The state machine's own change_state service names the node: a remapped
  // transition event topic or a second node of the same name cannot mislead
  // it. rclcpp_lifecycle's wrapper of that service shares its impl.
  ros2_lf_binding::lifecycle_gate_on_transition(
    state_machine, sm->com_interface.srv_change_state.impl, sm->current_state->id);
}

extern "C" void rcllf_gate_managed_entity_set(
  void * entity, void (* set)(void *), bool (* get)(const void *))
{
  ros2_lf_binding::lifecycle_gate_set_entity_flag(
    entity, [&]() {set(entity);}, [&]() {return get(entity);});
}

extern "C" bool rcllf_gate_managed_entity_get(const void * entity, bool (* get)(const void *))
{
  return ros2_lf_binding::lifecycle_gate_entity_flag(entity, [&]() {return get(entity);});
}

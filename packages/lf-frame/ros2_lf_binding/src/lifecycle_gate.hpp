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

/// @file lifecycle_gate.hpp
/// @brief Lifecycle state per tag for every dispatch of a lifecycle node.
///
/// A user callback the LF runtime dispatches may move a lifecycle node to
/// another state, its own or a sibling's, and so may a thread outside LF. A
/// change made at tag T takes effect for dispatches at tags after T, so every
/// reaction at one tag sees the same state whatever the physical order of the
/// reactions. Each effective change bumps the node's epoch, and a reaction
/// whose handle entry was reconciled at an older epoch reconciles it again
/// before it reads it.
///
/// The executable's publication gate reports every transition on the thread
/// that made it (lifecycle_gate_on_transition). Inside a user callback the
/// change is recorded at the current tag. A transition made outside LF (a
/// thread LF does not own) is external input, so it enters LF as a physical
/// action: the node's ingress wake, whose drain applies it at
/// a tag the runtime assigns once. Without the publication gate the gate falls
/// back to reading the node's state at callback exit and at the boundary, as
/// it does for a node without the lifecycle communication interface.
///
/// The boundary step (lifecycle_gate_boundary) runs once per tag, before any
/// user callback of that tag. It holds a strong reference to every entity of
/// the node (a pin) until the next boundary. A handle a hook drops at tag T
/// therefore stays valid for the other reactions at T, and a handle a hook
/// creates or drops bumps the epoch, even when the state ends where it began.
///
/// A LifecyclePublisher's own activation flag follows the same rule.
/// rclcpp sets it in the transition hooks, at physical time. A reaction
/// at tag T that did not make the change reads the flag the publisher had
/// before it, so a sibling's same-tag publish does not depend on which
/// reaction ran first. The callback that made the change reads it at once.

#ifndef LIFECYCLE_GATE_HPP_
#define LIFECYCLE_GATE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ros2_lf_binding
{

using LfTag = std::pair<int64_t, uint32_t>;

/// The effective lifecycle state of one node at a tag. Pure, no locking.
class LifecycleGate
{
public:
  struct View
  {
    uint8_t state;
    uint64_t epoch;
  };

  /// `initial` may be a transition state (10 to 15) for a node bound
  /// mid-transition, which counts as not active until a primary state is
  /// observed.
  explicit LifecycleGate(uint8_t initial)
  : effective_(initial) {}

  /// Record the node's current state, read at tag `tag`. A transition state
  /// (10 to 15) is not a decision and is ignored. Every observed change takes
  /// effect at the next tag with a new epoch, even when later observations at
  /// the same tag return to the effective state, because the node's hooks may
  /// have replaced its handles on the way.
  void observe(uint8_t state, LfTag tag);

  /// Record that the node's handles changed at `tag`: the epoch moves at the
  /// next tag whatever the state.
  void touch(LfTag tag);

  /// The state that holds for a dispatch at `tag`.
  View at(LfTag tag);

private:
  struct Pending
  {
    uint8_t state;
    LfTag tag;
  };
  void promote(LfTag tag);

  uint8_t effective_;
  uint64_t epoch_ = 0;
  std::optional<Pending> pending_;
};

/// True for the primary states 1 to 4 of lifecycle_msgs/msg/State.
bool is_primary_lifecycle_state(int state);

/// Collects strong references to the entities of a node that LF dispatches.
/// Returns false when the node is gone.
using LifecycleEntityCollector =
  std::function<bool(const std::string & node_class, std::vector<std::shared_ptr<void>> & out)>;

/// Set the collector the boundary step pins entities with (node_binding.cpp).
void lifecycle_gate_set_collector(LifecycleEntityCollector collector);

/// Cleans up the in-flight work of a node that was active and is not:
/// prunes the pending requests of its service clients and
/// cancels the goals of its action clients.
using LifecycleDeactivateHook = std::function<void(const std::string & node_class)>;

/// Set the hook the boundary step runs for a node that left the active state
/// (node_binding.cpp).
void lifecycle_gate_set_deactivate_hook(LifecycleDeactivateHook hook);

/// Start gating a lifecycle node, with the state its startup bind left it in,
/// primary or not, and pin its entities. `service_impls` are the rcl impls of
/// the node's change_state services. rcl_lifecycle embeds the change_state service in the
/// node's state machine and rclcpp_lifecycle's wrapper shares its impl, so a
/// reported transition names its node by that impl, not by a topic. Empty, or
/// no publication gate on the transition path: the gate reads the node's
/// state at callback exit and at the boundary instead.
void lifecycle_gate_register(
  const std::string & node_class, uint8_t state,
  const std::vector<const void *> & service_impls = {});

/// A transition of the state machine `state_machine`, whose change_state
/// service has the impl `change_state_impl`, reached `state`, on this thread.
/// In a user callback it is recorded at the current tag. Otherwise it is
/// posted to the node's ingress (a physical action), or, for a node without
/// one, left for the next boundary step.
void lifecycle_gate_on_transition(
  const void * state_machine, const void * change_state_impl, uint8_t state);

/// The ingress drain of a posted transition: record `state` at the current tag.
void lifecycle_gate_apply_external(const std::string & node_class, uint8_t state);

/// For a node the publication gate cannot report (see lifecycle_gate_register):
/// read its state and record a change at the current tag. Called when the outermost user callback
/// on a thread returns.
void lifecycle_gate_observe_all();

/// The boundary step. The first call at a tag releases the pins the node
/// dropped, pins the entities it created and bumps the epoch of a node whose
/// handles changed, as of the previous tag. It also records, as of the
/// previous tag, a transition left for it, and the state of a node the
/// publication gate cannot report. A node that was active at the previous
/// step and is not at this tag gets the deactivate hook, once, before the step
/// returns. Later calls at the same tag return at once.
void lifecycle_gate_boundary();

/// A LifecyclePublisher's flag (rclcpp_lifecycle::SimpleManagedEntity)
/// changes: `set` changes it, `get` reads it. Inside a user
/// callback at tag T the value before the change holds for the other
/// reactions at T. The record and the change are one step against
/// lifecycle_gate_entity_flag.
void lifecycle_gate_set_entity_flag(
  void * entity, const std::function<void()> & set, const std::function<bool()> & get);

/// The flag of `entity` for the running user callback: the value held for the
/// current tag when another reaction of the tag changed it, otherwise the live
/// one `get` reads (no change at this tag, a change this callback made, or no
/// user callback).
bool lifecycle_gate_entity_flag(const void * entity, const std::function<bool()> & get);

/// Drop every pin. Called before rclcpp shuts down.
void lifecycle_gate_release_pins();

/// The view of a gated node at the current tag, or nothing for a node without
/// a gate (a plain node).
std::optional<LifecycleGate::View> lifecycle_gate_view(const std::string & node_class);

/// The generation of an entity of the node: the index of the boundary step
/// that first pinned it, so a later handle has a higher one. An entity not
/// pinned yet gets the index of the next boundary step.
uint64_t lifecycle_gate_generation(const std::string & node_class, const void * entity);

/// Test support: the state machine of the node's first reported transition.
const void * lifecycle_gate_state_machine(const std::string & node_class);

/// Test support: forget every gate.
void lifecycle_gate_reset_for_test();

}  // namespace ros2_lf_binding

#endif  // LIFECYCLE_GATE_HPP_

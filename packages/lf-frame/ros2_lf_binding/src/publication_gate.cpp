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

/// @file publication_gate.cpp
/// @brief Executable-side publication gate (D2, T0 probe).
///
/// Compiled into every generated executable (installed to
/// share/ros2_lf_binding/src). Defining these rcl symbols in the executable
/// preempts librcl for librclcpp's PLT calls; each forwards to the next
/// definition via dlsym(RTLD_NEXT). A thread-local flag turns a re-entrant
/// call (rosout from logging inside the real publish) into a lock-free
/// counted pass-through. Nothing here logs.
///
/// rcl_subscription_init is gated too: the subscription of an
/// admitted edge gets a DDS reader deep enough for ingress to take every
/// sample, independent of the user's KEEP_LAST depth. rcl_node_init is gated
/// so that a subscription its node creates on another thread finds the node's
/// binding key too, and rcl_node_fini so that a recreated node of the same key
/// starts its creation ranks at 0 again.
///
/// rcl_lifecycle_trigger_transition_by_id and _by_label are gated so that the
/// lifecycle gate learns of every transition on the thread that made it.
/// librclcpp_lifecycle reaches them through its PLT as
/// it reaches rcl_publish.
///
/// rclcpp_lifecycle::SimpleManagedEntity's on_activate, on_deactivate and
/// is_activated are gated too, so that a LifecyclePublisher's
/// flag holds for the whole tag, as the node's state does. They are C++
/// members of librclcpp_lifecycle, defined here under their mangled names
/// (`this` is the first argument). An executable without rclcpp_lifecycle
/// never calls them.

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <mutex>

#include "rcl/node.h"
#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "ros2_lf_binding/publication_gate_api.h"

// rcl_lifecycle_state_machine_t, opaque here: the binding reads it.
struct rcl_lifecycle_state_machine_s;

namespace
{
thread_local bool t_in_gate = false;

template<typename Fn>
Fn next_symbol(const char * name)
{
  return reinterpret_cast<Fn>(dlsym(RTLD_NEXT, name));
}

struct InGate
{
  InGate() {t_in_gate = true;}
  ~InGate() {t_in_gate = false;}
};

/// One subscription init at a time, from its rank selection through
/// the rank commit, so two threads on one node and topic cannot both take the
/// same creation rank. Recursive in case the real init re-enters.
std::recursive_mutex & sub_init_mu()
{
  static std::recursive_mutex mu;
  return mu;
}
}  // namespace

extern "C" {

rcl_ret_t rcl_publisher_init(
  rcl_publisher_t * publisher, const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support, const char * topic_name,
  const rcl_publisher_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *,
    const char *, const rcl_publisher_options_t *);
  static Fn next = next_symbol<Fn>("rcl_publisher_init");
  const rcl_ret_t ret = next(publisher, node, type_support, topic_name, options);
  if (!t_in_gate) {
    InGate g;
    rcllf_gate_on_init(publisher, node, type_support, ret);
  }
  return ret;
}

rcl_ret_t rcl_node_init(
  rcl_node_t * node, const char * name, const char * namespace_, rcl_context_t * context,
  const rcl_node_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_node_t *, const char *, const char *, rcl_context_t *, const rcl_node_options_t *);
  static Fn next = next_symbol<Fn>("rcl_node_init");
  const rcl_ret_t ret = next(node, name, namespace_, context, options);
  if (!t_in_gate) {
    InGate g;
    rcllf_gate_on_node_init(node, ret);
  }
  return ret;
}

rcl_ret_t rcl_node_fini(rcl_node_t * node)
{
  using Fn = rcl_ret_t (*)(rcl_node_t *);
  static Fn next = next_symbol<Fn>("rcl_node_fini");
  if (!t_in_gate && node != nullptr) {
    InGate g;
    rcllf_gate_on_node_fini(node);
  }
  return next(node);
}

rcl_ret_t rcl_subscription_init(
  rcl_subscription_t * subscription, const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support, const char * topic_name,
  const rcl_subscription_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_subscription_t *, const rcl_node_t *, const rosidl_message_type_support_t *,
    const char *, const rcl_subscription_options_t *);
  static Fn next = next_symbol<Fn>("rcl_subscription_init");
  if (t_in_gate || options == nullptr || node == nullptr || topic_name == nullptr) {
    return next(subscription, node, type_support, topic_name, options);
  }
  std::lock_guard<std::recursive_mutex> serial(sub_init_mu());
  rcl_subscription_options_t deepened = *options;
  {
    InGate g;
    rcllf_gate_sub_qos(node, topic_name, &deepened.qos);
  }
  const rcl_ret_t ret = next(subscription, node, type_support, topic_name, &deepened);
  {
    InGate g;
    rcllf_gate_on_sub_init(ret, subscription);
  }
  return ret;
}

rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher, const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish");
  if (t_in_gate) {
    rcllf_gate_nested(publisher);
    return next(publisher, ros_message, allocation);
  }
  InGate g;
  rcllf_gate_payload(publisher, ros_message, RCLLF_GATE_PUBLISH);
  void * ctx = rcllf_gate_pre(publisher, RCLLF_GATE_PUBLISH);
  const rcl_ret_t ret = next(publisher, ros_message, allocation);
  rcllf_gate_post(ctx, ret);
  return ret;
}

rcl_ret_t rcl_publish_serialized_message(
  const rcl_publisher_t * publisher, const rcl_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(
    const rcl_publisher_t *, const rcl_serialized_message_t *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish_serialized_message");
  if (t_in_gate) {
    rcllf_gate_nested(publisher);
    return next(publisher, serialized_message, allocation);
  }
  InGate g;
  rcllf_gate_payload(publisher, serialized_message, RCLLF_GATE_SERIALIZED);
  void * ctx = rcllf_gate_pre(publisher, RCLLF_GATE_SERIALIZED);
  const rcl_ret_t ret = next(publisher, serialized_message, allocation);
  rcllf_gate_post(ctx, ret);
  return ret;
}

rcl_ret_t rcl_publish_loaned_message(
  const rcl_publisher_t * publisher, void * ros_message, rmw_publisher_allocation_t * allocation)
{
  using Fn = rcl_ret_t (*)(const rcl_publisher_t *, void *, rmw_publisher_allocation_t *);
  static Fn next = next_symbol<Fn>("rcl_publish_loaned_message");
  if (t_in_gate) {
    rcllf_gate_nested(publisher);
    return next(publisher, ros_message, allocation);
  }
  InGate g;
  rcllf_gate_payload(publisher, ros_message, RCLLF_GATE_LOANED);
  void * ctx = rcllf_gate_pre(publisher, RCLLF_GATE_LOANED);
  const rcl_ret_t ret = next(publisher, ros_message, allocation);
  rcllf_gate_post(ctx, ret);
  return ret;
}

rcl_ret_t rcl_publisher_fini(rcl_publisher_t * publisher, rcl_node_t * node)
{
  using Fn = rcl_ret_t (*)(rcl_publisher_t *, rcl_node_t *);
  static Fn next = next_symbol<Fn>("rcl_publisher_fini");
  if (!t_in_gate) {
    InGate g;
    rcllf_gate_on_fini(publisher);
  }
  return next(publisher, node);
}

rcl_ret_t rcl_lifecycle_trigger_transition_by_id(
  rcl_lifecycle_state_machine_s * state_machine, uint8_t id, bool publish_notification)
{
  using Fn = rcl_ret_t (*)(rcl_lifecycle_state_machine_s *, uint8_t, bool);
  static Fn next = next_symbol<Fn>("rcl_lifecycle_trigger_transition_by_id");
  if (next == nullptr) {
    return RCL_RET_ERROR;
  }
  const rcl_ret_t ret = next(state_machine, id, publish_notification);
  if (ret == RCL_RET_OK && !t_in_gate) {
    InGate g;
    rcllf_gate_on_lifecycle_transition(state_machine);
  }
  return ret;
}

rcl_ret_t rcl_lifecycle_trigger_transition_by_label(
  rcl_lifecycle_state_machine_s * state_machine, const char * label, bool publish_notification)
{
  using Fn = rcl_ret_t (*)(rcl_lifecycle_state_machine_s *, const char *, bool);
  static Fn next = next_symbol<Fn>("rcl_lifecycle_trigger_transition_by_label");
  if (next == nullptr) {
    return RCL_RET_ERROR;
  }
  const rcl_ret_t ret = next(state_machine, label, publish_notification);
  if (ret == RCL_RET_OK && !t_in_gate) {
    InGate g;
    rcllf_gate_on_lifecycle_transition(state_machine);
  }
  return ret;
}

using EntityFn = void (*)(void *);
using EntityFlagFn = bool (*)(const void *);

void rcllf_entity_on_activate(void * self)
__asm__ ("_ZN16rclcpp_lifecycle19SimpleManagedEntity11on_activateEv");
void rcllf_entity_on_deactivate(void * self)
__asm__ ("_ZN16rclcpp_lifecycle19SimpleManagedEntity13on_deactivateEv");
bool rcllf_entity_is_activated(const void * self)
__asm__ ("_ZNK16rclcpp_lifecycle19SimpleManagedEntity12is_activatedEv");

static EntityFlagFn rcllf_entity_flag()
{
  static EntityFlagFn next =
    next_symbol<EntityFlagFn>("_ZNK16rclcpp_lifecycle19SimpleManagedEntity12is_activatedEv");
  return next;
}

static void rcllf_entity_set(void * self, EntityFn next)
{
  EntityFlagFn flag = rcllf_entity_flag();
  if (t_in_gate || flag == nullptr) {
    next(self);
    return;
  }
  InGate g;
  rcllf_gate_managed_entity_set(self, next, flag);
}

void rcllf_entity_on_activate(void * self)
{
  static EntityFn next =
    next_symbol<EntityFn>("_ZN16rclcpp_lifecycle19SimpleManagedEntity11on_activateEv");
  rcllf_entity_set(self, next);
}

void rcllf_entity_on_deactivate(void * self)
{
  static EntityFn next =
    next_symbol<EntityFn>("_ZN16rclcpp_lifecycle19SimpleManagedEntity13on_deactivateEv");
  rcllf_entity_set(self, next);
}

bool rcllf_entity_is_activated(const void * self)
{
  EntityFlagFn next = rcllf_entity_flag();
  if (t_in_gate) {
    return next(self);
  }
  InGate g;
  return rcllf_gate_managed_entity_get(self, next);
}

/// Self-check support (D1.5): the gate's own address for each gated symbol.
void * rcllf_gate_symbol(const char * name)
{
  if (std::strcmp(name, "rcl_publish") == 0) {
    return reinterpret_cast<void *>(&rcl_publish);
  }
  if (std::strcmp(name, "rcl_publish_serialized_message") == 0) {
    return reinterpret_cast<void *>(&rcl_publish_serialized_message);
  }
  if (std::strcmp(name, "rcl_publish_loaned_message") == 0) {
    return reinterpret_cast<void *>(&rcl_publish_loaned_message);
  }
  if (std::strcmp(name, "rcl_publisher_fini") == 0) {
    return reinterpret_cast<void *>(&rcl_publisher_fini);
  }
  if (std::strcmp(name, "rcl_publisher_init") == 0) {
    return reinterpret_cast<void *>(&rcl_publisher_init);
  }
  if (std::strcmp(name, "rcl_subscription_init") == 0) {
    return reinterpret_cast<void *>(&rcl_subscription_init);
  }
  if (std::strcmp(name, "rcl_node_init") == 0) {
    return reinterpret_cast<void *>(&rcl_node_init);
  }
  if (std::strcmp(name, "rcl_node_fini") == 0) {
    return reinterpret_cast<void *>(&rcl_node_fini);
  }
  if (std::strcmp(name, "rcl_lifecycle_trigger_transition_by_id") == 0) {
    return reinterpret_cast<void *>(&rcl_lifecycle_trigger_transition_by_id);
  }
  if (std::strcmp(name, "rcl_lifecycle_trigger_transition_by_label") == 0) {
    return reinterpret_cast<void *>(&rcl_lifecycle_trigger_transition_by_label);
  }
  return nullptr;
}

}  // extern "C"

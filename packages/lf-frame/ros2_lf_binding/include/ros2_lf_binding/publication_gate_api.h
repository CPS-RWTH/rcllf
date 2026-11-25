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

/// @file publication_gate_api.h
/// @brief Hooks the executable-side publication gate calls (D2).
///
/// publication_gate.cpp (installed to share/ros2_lf_binding/src and compiled
/// into every generated executable) defines rcl_publish and its siblings,
/// forwards each to the next definition and brackets the call with these
/// hooks. The hooks never log and never publish.

#ifndef ROS2_LF_BINDING__PUBLICATION_GATE_API_H_
#define ROS2_LF_BINDING__PUBLICATION_GATE_API_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum rcllf_gate_api
{
  RCLLF_GATE_PUBLISH = 0,
  RCLLF_GATE_SERIALIZED = 1,
  RCLLF_GATE_LOANED = 2,
};

/** After rcl_publisher_init returned `ret` for `publisher` on `node` with `type_support`. */
void rcllf_gate_on_init(
  const void * publisher, const void * node, const void * type_support, int ret);
/** Before the real publish. Returns an opaque context for rcllf_gate_post, or NULL. */
void * rcllf_gate_pre(const void * publisher, int api);
/**
 * Before rcllf_gate_pre, so the output-deadline check in
 * rcllf_gate_pre stays next to the real publish: `payload` is the
 * ROS message (an rcl_serialized_message_t for RCLLF_GATE_SERIALIZED). Records
 * a bounded publisher's payload hash under RCLLF_PLANT_RECORD; returns at
 * once otherwise. Never logs, never publishes.
 */
void rcllf_gate_payload(const void * publisher, const void * payload, int api);
/** After the real publish returned `ret`. */
void rcllf_gate_post(void * ctx, int ret);
/** A publish re-entered the gate on this thread: counted, lock-free, passes through. */
void rcllf_gate_nested(const void * publisher);
/** Before rcl_publisher_fini. */
void rcllf_gate_on_fini(const void * publisher);
/**
 * Before rcl_subscription_init: adjusts `qos` (an
 * rmw_qos_profile_t holding the user's request) for the reader of `topic_name`
 * on `node`. Only the subscription of an admitted edge changes: its KEEP_LAST
 * depth rises to the edge's reader depth and, when the edge's publisher runs
 * in another federate (an ingress edge), its reliability becomes BEST_EFFORT,
 * since a reliable reader withholds every sample behind a sequence gap until
 * the writer's history drops the lost one. A same-federate edge keeps the
 * user's reliability. The user's values are kept for the startup hazard
 * checks.
 */
void rcllf_gate_sub_qos(const void * node, const char * topic_name, void * qos);
/**
 * After that rcl_subscription_init of `subscription` returned `ret`: a
 * successful init takes the creation rank rcllf_gate_sub_qos saw, a failed
 * one none, and records the LF tag it was created at.
 */
void rcllf_gate_on_sub_init(int ret, const void * subscription);
/**
 * After rcl_node_init: remembers the binding key of a node
 * created inside a construction scope.
 */
void rcllf_gate_on_node_init(const void * node, int ret);
/**
 * Before rcl_node_fini: forgets the node's binding key and, once
 * no other node of that key is alive, the key's subscription creation ranks.
 */
void rcllf_gate_on_node_fini(const void * node);
/**
 * After rcl_lifecycle_trigger_transition_by_id or _by_label succeeded on
 * `state_machine` (an rcl_lifecycle_state_machine_t), on the thread that ran
 * the transition.
 */
void rcllf_gate_on_lifecycle_transition(const void * state_machine);
/**
 * rclcpp_lifecycle::SimpleManagedEntity::on_activate or on_deactivate of
 * `entity`: records the flag `get` reads and runs `set`, as one
 * step against rcllf_gate_managed_entity_get.
 */
void rcllf_gate_managed_entity_set(
  void * entity, void (* set)(void *), bool (* get)(const void *));
/**
 * SimpleManagedEntity::is_activated of `entity`: the flag held
 * for the current tag, or the live one `get` reads.
 */
bool rcllf_gate_managed_entity_get(const void * entity, bool (* get)(const void *));

#ifdef __cplusplus
}
#endif

#endif  // ROS2_LF_BINDING__PUBLICATION_GATE_API_H_

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

/// @file rid_table_api.h
/// @brief Public C++ API for RID table registration.
///
/// Included by generated ros2_lf_reaction_ids_init.cpp.
/// The generated code calls register_rid_entry() and resize_handle_table()
/// before ros2_lf_runtime_init() so that ros2_lf_bind_node() can match
/// ROS2 handles to reaction IDs.

#ifndef ROS2_LF_BINDING__RID_TABLE_API_H_
#define ROS2_LF_BINDING__RID_TABLE_API_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace ros2_lf_binding
{

  struct RidEntry
  {
    int reaction_id;
    std::string node_class;
    std::string callback_name;
    // "timer" | "sub" | "srv_send" | "srv_server" | "srv_client"
    // | "action_goal_send" | "action_goal_server" | "action_goal_cb"
    // | "action_feedback" | "action_result_cb"
    std::string kind;
    std::string topic;  // subscription topic; empty for timers
    int64_t period_ns;  // timer period; 0 for subscriptions
    // Position of this reaction's timer (timer-driven kinds) or subscription
    // ("sub") among the node's timers or subscriptions, in the order the node
    // creates them; -1 when unknown. ros2_lf_bind_node matches handles one to
    // one in this order.
    int creation_index = -1;
    // Position of that timer's or subscription's callback group among the
    // groups the node creates itself, in creation order; kGroupDefault for the
    // node's default group, kGroupUnknown when the analyzer could not resolve
    // it. Handles are matched within their group.
    int group_index = -2;
    // Reliability the analysis saw on this subscription ("reliable",
    // "best_effort"); empty without a value and for other kinds. The binding
    // checks RCLLF-H000 and H001 per subscription at bind time.
    std::string sub_reliability {};
  };

  constexpr int kGroupDefault = -1;
  constexpr int kGroupUnknown = -2;
  // An input (/tf or /tf_static) of the node's admitted
  // tf2_ros::TransformListener. tf2_ros puts both in a group its own executor
  // spins; the binding detaches that group right after the node is
  // constructed and binds the two handles like any other subscription.
  constexpr int kGroupTf2Listener = -4;

  void register_rid_entry(RidEntry entry);
  // The node class acknowledged `hazard_code`, which covers handles spun by an
  // executor of their own (RCLLF-R002, tf2_ros). ros2_lf_bind_node then lets
  // such handles stay without a reaction ID and logs each one.
  void register_foreign_executor_ack(
    const std::string & node_class,
    const std::string & hazard_code);
  void resize_handle_table(std::size_t n);
  // execution.yaml `nodes.<node>.tf2`: the wait bound (0 = keep the
  // constructor literal) and on_timeout ("" = keep) of every
  // rcllf_tf2::AcquisitionLookup the node `node_class` constructs.
  void register_tf2_config(
    const std::string & node_class, int64_t wait_bound_ns,
    const std::string & on_timeout);
  // execution.yaml declares that the callback of `node_class`'s subscription
  // to `topic` runs at most once per `period_ns` (min_input_period_ms). The
  // binding counts and reports every run closer than that.
  void register_min_input_period(
    const std::string & node_class,
    const std::string & topic,
    int64_t period_ns);

}  // namespace ros2_lf_binding

#endif  // ROS2_LF_BINDING__RID_TABLE_API_H_

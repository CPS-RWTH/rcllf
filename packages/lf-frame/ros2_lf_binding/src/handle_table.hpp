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

/// @file handle_table.hpp
/// @brief Global flat table mapping reaction IDs to ROS2 handles.
///
/// Indexed by the RID_* constants from ros2_lf_reaction_ids.h.
/// Populated during ros2_lf_bind_node (startup reaction).
/// Read-only after all startup reactions have fired, except an entry of a
/// lifecycle node, which only its own reaction rewrites, through
/// bind_late_handle and lifecycle_admit.

#ifndef HANDLE_TABLE_HPP_
#define HANDLE_TABLE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rmw/types.h"
#include "take_mode.hpp"

namespace ros2_lf_binding
{

enum class HandleKind
{
  TIMER,
  SUBSCRIPTION,
  SERVICE_SERVER,
  SERVICE_CLIENT,
  ACTION_SERVER,
  ACTION_CLIENT,
  UNBOUND,
};

struct HandleEntry
{
  HandleKind kind = HandleKind::UNBOUND;
  // Entities are held WEAKLY, the ownership a stock executor has:
  // the node owns them. A lifecycle node that drops its timer
  // in on_deactivate or its publisher in on_cleanup ends the entity, and the
  // invoke path turns an expired entry into a traced no-op instead of firing a
  // dropped timer or dereferencing a released member.
  rclcpp::TimerBase::WeakPtr timer;
  rclcpp::SubscriptionBase::WeakPtr subscription;
  rclcpp::ServiceBase::WeakPtr service_server;
  rclcpp::ClientBase::WeakPtr service_client;
  rclcpp::Waitable::WeakPtr action_waitable;    // For action server or client
  size_t action_entity_id = 0;    // Entity ID for take_data_by_entity_id (server=0, client varies)
  std::string topic;              // For subscriptions; service/client/action name otherwise
  std::string resolved_topic;     // sub->get_topic_name() at bind time (subscriptions only)
  std::string node_class;         // For diagnostics
  std::string callback_name;      // For diagnostics
  // Execution policy resolved once at registration from the policy table
  // (policy_table.hpp).  Subscriptions only; services/actions skip-only.
  // Cached here rather than looked up per dispatch — see the reconfiguration
  // note in policy_table.hpp for why.
  TakeMode take_mode = TakeMode::Skip;
  int64_t take_timeout_ms = 100;
  // A timer or subscription entry of a lifecycle node. The node
  // creates and drops these handles on its transitions, so the entry is bound
  // to the handle that fits it when one exists, at the entry's own dispatch.
  bool late = false;
  // Any entry of a lifecycle node. Its dispatch asks the
  // node's lifecycle gate first, and `gate_epoch` is the gate epoch the entry
  // was last reconciled at.
  bool lifecycle = false;
  uint64_t gate_epoch = 0;
  // The gate generation (lifecycle_gate_generation) of the newest handle a
  // lifecycle reconcile saw this entry hold, -1 before the first. Only a
  // handle of a higher generation replaces it.
  int64_t generation = -1;
};

/// Global handle table.  Resized to RID_COUNT in ros2_lf_runtime_init().
/// Access is single-threaded during startup; reactions read it concurrently
/// but only after all startup reactions have completed. The table stays
/// read-only after startup (R15): entries are never written at runtime, an
/// entity's liveness lives in its weak_ptr referent (lock() per invoke).
extern std::vector<HandleEntry> g_handle_table;

/// Bind a `late` entry whose handle is missing or expired to the handle its
/// lifecycle node created since, if there is one. Call only from
/// the entry's own reaction, before reading the entry. No-op for other entries.
void bind_late_handle(int reaction_id);

/// Gate one dispatch of `reaction_id` on its lifecycle node.
/// When the node's gate epoch moved since the entry was last
/// reconciled, the entry is matched again against the node's current handles.
/// Returns true when the user callback may run: the entry is not a lifecycle
/// node's, or the node is active at this tag. Call only from the entry's own
/// reaction, before reading the entry.
bool lifecycle_admit(int reaction_id);

}  // namespace ros2_lf_binding

#endif  // HANDLE_TABLE_HPP_

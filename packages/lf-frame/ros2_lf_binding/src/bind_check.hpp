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

/// @file bind_check.hpp
/// @brief Bind-time refusal of subscriptions that have no RID.
///
/// The binding dispatches a subscription only through its RID entry, and the
/// registration executor in runtime.cpp is never spun. A subscription that an
/// rclcpp executor would run but that no RID entry matched would therefore
/// never deliver a message. Binding refuses to start
/// instead of losing it silently.
/// ros2_lf_bind_node now refuses to start in that case.

#ifndef BIND_CHECK_HPP_
#define BIND_CHECK_HPP_

#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{

/// The resolved name of /parameter_events for the node behind *node_base*,
/// with the node's remap rules applied (`-r /parameter_events:=/x` gives /x).
/// The rclcpp TimeSource subscribes under this name.
std::string resolved_parameter_events_topic(
  rclcpp::node_interfaces::NodeBaseInterface & node_base);

/// The subscriptions in *subs* (all of one node, in callback-group order)
/// that the node's rclcpp TimeSource owns, so they need no RID.
/// Every rclcpp::Node has a TimeSource subscription of type ParameterEvent on
/// *parameter_events_topic* in its default group. It is the first one of that
/// type and name, because the TimeSource creates it in the rclcpp::Node
/// constructor before any user code. The binding hands it its events
/// synchronously when a node is detached from sim time and otherwise never
/// dispatches it, as before the bind check. A node built with
/// lf_time_node_options() also has its TimeSource /clock subscription
/// (`is_time_source_clock`), which the binding feeds itself. A user
/// /parameter_events subscription is not exempt.
std::vector<const rclcpp::SubscriptionBase *> time_source_subscriptions(
  const std::vector<rclcpp::SubscriptionBase::SharedPtr> & subs,
  const std::function<bool(const rclcpp::SubscriptionBase *)> & is_time_source_clock,
  const std::string & parameter_events_topic);

/// Rule (shared with the handle matching): a
/// subscription without a RID may run outside LF only when its callback
/// group is already associated with another executor at bind time
/// (`get_associated_with_executor_atomic()`, true for the dedicated executor
/// of tf2_ros::TransformListener) and the generator registered an RCLLF-R002
/// acknowledgement for the node (*acknowledged_hazard* not empty). A group
/// with automatic add disabled but no executor is not exempt.
bool foreign_executor_exempt(
  bool group_associated_with_executor, const std::string & acknowledged_hazard);

/// Result of check_node_subscriptions.
struct SubscriptionCheck
{
  /// Topics of subscriptions with no RID and no exemption, in group order.
  std::vector<std::string> unbound;
  /// Exempt as rclcpp TimeSource subscriptions.
  std::vector<const rclcpp::SubscriptionBase *> time_source;
  /// Exempt by foreign_executor_exempt.
  std::vector<const rclcpp::SubscriptionBase *> foreign_executor;
};

/// Check every subscription in every callback group of the node behind
/// *node_base*, whether or not the group is added to an executor with the
/// node. A subscription in *bound* has a RID. Any other subscription must be
/// a TimeSource one or pass foreign_executor_exempt, or it is listed in
/// `unbound`.
SubscriptionCheck check_node_subscriptions(
  rclcpp::node_interfaces::NodeBaseInterface & node_base,
  const std::vector<const rclcpp::SubscriptionBase *> & bound,
  const std::string & acknowledged_hazard,
  const std::function<bool(const rclcpp::SubscriptionBase *)> & is_time_source_clock);

/// The error text for *topics* of *node_class*.
std::string unbound_subscription_error(
  const std::string & node_class, const std::vector<std::string> & topics);

/// Log unbound_subscription_error at FATAL and exit the process with status 1
/// when *topics* is non-empty. Returns normally otherwise.
void refuse_unbound_subscriptions(
  const std::string & node_class, const std::vector<std::string> & topics);

}  // namespace ros2_lf_binding

#endif  // BIND_CHECK_HPP_

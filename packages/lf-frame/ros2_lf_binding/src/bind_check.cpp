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

#include "bind_check.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl_interfaces/msg/parameter_event.hpp"

namespace ros2_lf_binding
{

std::string resolved_parameter_events_topic(
  rclcpp::node_interfaces::NodeBaseInterface & node_base)
{
  char * resolved = nullptr;
  const rcl_ret_t ret = rcl_node_resolve_name(
    node_base.get_rcl_node_handle(), "/parameter_events", rcl_get_default_allocator(),
    false, false, &resolved);
  if (ret != RCL_RET_OK || resolved == nullptr) {
    rcl_reset_error();
    return "/parameter_events";
  }
  std::string topic(resolved);
  rcl_get_default_allocator().deallocate(resolved, rcl_get_default_allocator().state);
  return topic;
}

std::vector<const rclcpp::SubscriptionBase *> time_source_subscriptions(
  const std::vector<rclcpp::SubscriptionBase::SharedPtr> & subs,
  const std::function<bool(const rclcpp::SubscriptionBase *)> & is_time_source_clock,
  const std::string & parameter_events_topic)
{
  std::vector<const rclcpp::SubscriptionBase *> owned;
  for (const auto & sub : subs) {
    if (is_time_source_clock(sub.get())) {
      owned.push_back(sub.get());
    }
  }
  for (const auto & sub : subs) {
    if (sub->get_topic_name() == parameter_events_topic &&
      std::dynamic_pointer_cast<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>>(sub))
    {
      owned.push_back(sub.get());
      break;
    }
  }
  return owned;
}

bool foreign_executor_exempt(
  bool group_associated_with_executor, const std::string & acknowledged_hazard)
{
  return group_associated_with_executor && !acknowledged_hazard.empty();
}

SubscriptionCheck check_node_subscriptions(
  rclcpp::node_interfaces::NodeBaseInterface & node_base,
  const std::vector<const rclcpp::SubscriptionBase *> & bound,
  const std::string & acknowledged_hazard,
  const std::function<bool(const rclcpp::SubscriptionBase *)> & is_time_source_clock)
{
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::vector<bool> associated;
  node_base.for_each_callback_group(
    [&subs, &associated](rclcpp::CallbackGroup::SharedPtr group) {
      const bool group_associated = group->get_associated_with_executor_atomic().load();
      group->find_subscription_ptrs_if(
        [&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
          if (std::find(subs.begin(), subs.end(), sp) == subs.end()) {
            subs.push_back(sp);
            associated.push_back(group_associated);
          }
          return false;
        });
    });
  SubscriptionCheck check;
  check.time_source = time_source_subscriptions(
    subs, is_time_source_clock, resolved_parameter_events_topic(node_base));
  for (std::size_t i = 0; i < subs.size(); ++i) {
    const auto * sub = subs[i].get();
    if (std::find(bound.begin(), bound.end(), sub) != bound.end() ||
      std::find(check.time_source.begin(), check.time_source.end(), sub) !=
      check.time_source.end())
    {
      continue;
    }
    if (foreign_executor_exempt(associated[i], acknowledged_hazard)) {
      check.foreign_executor.push_back(sub);
    } else {
      check.unbound.emplace_back(sub->get_topic_name());
    }
  }
  return check;
}

std::string unbound_subscription_error(
  const std::string & node_class, const std::vector<std::string> & topics)
{
  std::string list;
  for (const auto & t : topics) {
    list += (list.empty() ? "'" : ", '") + t + "'";
  }
  return "ros2_lf_bind_node(" + node_class + "): " + std::to_string(topics.size()) +
         " subscription(s) have no reaction ID and would never be dispatched: " + list +
         ". The analyzer did not see them (system.json lists no subscriber on that topic "
         "for this node). Refusing to start instead of dropping their messages silently.";
}

void refuse_unbound_subscriptions(
  const std::string & node_class, const std::vector<std::string> & topics)
{
  if (topics.empty()) {
    return;
  }
  RCLCPP_FATAL(
    rclcpp::get_logger("ros2_lf_binding"), "%s",
    unbound_subscription_error(node_class, topics).c_str());
  std::fflush(stdout);
  std::fflush(stderr);
  // _Exit, not exit: this runs inside an LF startup reaction on a worker
  // thread, and static destructors must not race the other workers (same
  // reasoning as corr_runtime.cpp startup_or_exit).
  std::_Exit(1);
}

}  // namespace ros2_lf_binding

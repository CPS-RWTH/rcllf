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

/// @file node_binding.cpp
/// @brief Implements ros2_lf_bind_node.
///
/// Called from every lfc-generated reaction(startup) body.  Walks the
/// component manager's loaded nodes, finds the one matching node_class,
/// iterates its callback groups to collect timers and subscriptions, matches
/// them against the RID table loaded from ros2_lf_reaction_ids.h, cancels
/// matched timers, and populates g_handle_table.

#include <rcl/client.h>
#include <rcl/error_handling.h>
#include <rcl/service.h>
#include <rcl/timer.h>
#include <rcl/wait.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <vector>

#include "action_channels.hpp"
#include "action_client_guard.hpp"
#include "bind_check.hpp"
#include "component_manager.hpp"
#include "corr_runtime.hpp"
#include "external_input.hpp"
#include "handle_matching.hpp"
#include "service_pairing.hpp"
#include "handle_table.hpp"
#include "input_period.hpp"
#include "lifecycle_gate.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "name_match.hpp"
#include "output_deadline.hpp"
#include "policy_table.hpp"
#include "tf2_ingress.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rid_table.hpp"
#include "tag_log.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "thread_role.hpp"
#include "time_source.hpp"

namespace ros2_lf_binding
{
extern std::shared_ptr<LFComponentManager> g_manager;
void lf_mark_running();  // runtime.cpp

namespace
{

// Group of each callback group as the analyzer counts it: kGroupDefault, the
// position among the node's own groups in creation order (for_each_callback_group
// order), or kGroupForeign for a group another executor already spins. rclcpp's
// own groups (the TimeSource clock thread) are left out. No executor spins the
// node's groups under LF, so an associated group belongs to a library thread
// (tf2_ros::TransformListener).
std::map<const rclcpp::CallbackGroup *, int> analyzer_group_index(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  std::map<const rclcpp::CallbackGroup *, int> group_index;
  const auto default_group = node_base->get_default_callback_group();
  int next_group = 0;
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      if (group == default_group) {
        group_index[group.get()] = kGroupDefault;
      } else if (lf_is_rclcpp_owned_group(group.get())) {
        return;
      } else if (group->get_associated_with_executor_atomic().load()) {
        group_index[group.get()] = kGroupForeign;
      } else {
        group_index[group.get()] = next_group++;
      }
    });
  return group_index;
}

// The TimeSource /parameter_events subscription, which the Node constructor
// creates in the default group before any user code runs.
const rclcpp::SubscriptionBase * parameter_events_subscription(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  const auto default_group = node_base->get_default_callback_group();
  if (!default_group) {
    return nullptr;
  }
  return default_group->find_subscription_ptrs_if(
    [](const rclcpp::SubscriptionBase::SharedPtr & sp) {
      return static_cast<bool>(
        std::dynamic_pointer_cast<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>>(sp));
    }).get();
}

bool is_timer_family(const std::string & kind)
{
  return kind == "timer" || kind == "srv_send" || kind == "action_goal_send";
}

/// The RID's node-local topic resolved by the node's own namespace and remap
/// rules, or "" when rcl cannot resolve it.
std::string resolved_rid_topic(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node, const RidEntry & e)
{
  char * out = nullptr;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  if (!node || rcl_node_resolve_name(
      node->get_rcl_node_handle(), e.topic.c_str(), alloc, false, false, &out) != RCL_RET_OK)
  {
    rcl_reset_error();
    return "";
  }
  std::string r(out);
  alloc.deallocate(out, alloc.state);
  return r;
}

/// The analyzer records the topic name the node source passes. A launch remap
///  or a namespace changes the handle's name, so the RID fits the
/// name the node itself resolves it to. Only when that fails it falls back to
/// comparing the names as written.
bool topic_fits(
  const std::string & sub_topic, const RidEntry & e,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node)
{
  const std::string resolved = resolved_rid_topic(node, e);
  if (!resolved.empty()) {
    return sub_topic == resolved;
  }
  return sub_topic == e.topic || sub_topic == "/" + e.topic || "/" + sub_topic == e.topic;
}

// srv_send and action_goal_send: the user's callback that sends the request or
// goal is driven by a timer, so it is bound as a TIMER handle and re-dispatched
// via ros2_lf_invoke_timer.
void bind_timer(
  const RidEntry & entry, const rclcpp::TimerBase::SharedPtr & timer, const char * how)
{
  int64_t period_ns = 0;  // stays 0 if the timer handle is invalid
  const rcl_ret_t period_ret = rcl_timer_get_period(timer->get_timer_handle().get(), &period_ns);
  (void)period_ret;
  auto & h = g_handle_table[entry.reaction_id];
  h.kind = HandleKind::TIMER;
  h.timer = timer;
  h.node_class = entry.node_class;
  h.callback_name = entry.callback_name;
  // No executor spins a bound node, so the rcl timer stays armed and its
  // canceled flag belongs to the user: ros2_lf_invoke_timer skips the callback
  // while the user has it cancelled.
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "Bound %s%s RID %d (%s::%s, period %ld ns)", how, entry.kind.c_str(),
    entry.reaction_id, entry.node_class.c_str(), entry.callback_name.c_str(), period_ns);
}

void bind_subscription(
  const RidEntry & entry, const rclcpp::SubscriptionBase::SharedPtr & sub, const char * how)
{
  const std::string sub_topic = sub->get_topic_name();
  auto & h = g_handle_table[entry.reaction_id];
  h.kind = HandleKind::SUBSCRIPTION;
  h.subscription = sub;
  h.topic = entry.topic;
  h.resolved_topic = sub_topic;
  h.node_class = entry.node_class;
  h.callback_name = entry.callback_name;
  // Resolve the execution policy once, here: this is the only site
  // that has node_class, the declared topic and the runtime-resolved
  // topic all in scope (design §2.5).
  const TakePolicy policy = resolve_policy(entry.node_class, entry.topic, sub_topic);
  h.take_mode = policy.take_mode;
  h.take_timeout_ms = policy.take_timeout_ms;
  if (external_input::is_declared_topic(sub_topic)) {
    // A declared external topic never takes, so neither take_mode nor its env override applies.
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "Bound %ssubscription RID %d (%s::%s, topic '%s', resolved '%s', "
      "external_topics, dispatch on arrival)",
      how, entry.reaction_id, entry.node_class.c_str(), entry.callback_name.c_str(),
      entry.topic.c_str(), sub_topic.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "Bound %ssubscription RID %d (%s::%s, topic '%s', resolved '%s', "
      "take_mode=%s timeout_ms=%ld)",
      how, entry.reaction_id, entry.node_class.c_str(), entry.callback_name.c_str(),
      entry.topic.c_str(), sub_topic.c_str(),
      policy.take_mode == TakeMode::Block ? "block" : "skip",
      static_cast<long>(policy.take_timeout_ms));  // NOLINT(runtime/int)
  }
  // An external_topics subscription goes to ingress once both this
  // bind and its startup registration happened.
  external_input::on_bound(entry.reaction_id);
  // RCLLF-H000 and H001 on this subscription's effective QoS.
  corr_rt::check_subscription_qos(entry, *sub);
  // The next reader of a lifecycle subscription made or reset in a hook.
  corr_rt::on_subscription_bound(entry.reaction_id, sub);
  // Check the declared minimum input period of this subscription.
  input_period::attach(entry.reaction_id, entry.node_class, entry.topic);
}

bool holds_live_handle(const HandleEntry & h)
{
  return (h.kind == HandleKind::TIMER && !h.timer.expired()) ||
         (h.kind == HandleKind::SUBSCRIPTION && !h.subscription.expired());
}

// Serialises late binds and lifecycle reconciles. Each writes only its own
// entry and reads only entries of its own node.
std::mutex g_late_mutex;

/// The generation an entry's handle must exceed to replace it: the newest of
/// the one recorded and that of the handle it holds now, which is recorded.
int64_t generation_floor(HandleEntry & h, const std::string & node_class, const void * current)
{
  if (current != nullptr) {
    h.generation = std::max(
      h.generation, static_cast<int64_t>(lifecycle_gate_generation(node_class, current)));
  }
  return h.generation;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_of(const std::string & node_class)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  if (g_manager) {
    node_base = g_manager->find_by_class(node_class);
  }
  if (!node_base) {
    node_base = ros2_lf_get_node(node_class);
  }
  return node_base;
}

/// The lifecycle gate's collector: every entity LF dispatches for the node,
/// in the groups LF owns. A subscription on an external topic is left out:
/// the ingress thread holds it as well, so its pin would never be released
/// and the node could not drop it.
bool lifecycle_node_entities(
  const std::string & node_class, std::vector<std::shared_ptr<void>> & out)
{
  const auto node_base = node_base_of(node_class);
  if (!node_base) {
    return false;
  }
  const auto group_index = analyzer_group_index(node_base);
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      const auto g = group_index.find(group.get());
      if (g == group_index.end() || g->second == kGroupForeign) {
        return;
      }
      group->find_timer_ptrs_if([&](const rclcpp::TimerBase::SharedPtr & t) {
        out.push_back(t);
        return false;
      });
      group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
        if (!external_input::is_declared_topic(sp->get_topic_name())) {
          out.push_back(sp);
        }
        return false;
      });
      group->find_service_ptrs_if([&](const rclcpp::ServiceBase::SharedPtr & sp) {
        out.push_back(sp);
        return false;
      });
      group->find_client_ptrs_if([&](const rclcpp::ClientBase::SharedPtr & cp) {
        out.push_back(cp);
        return false;
      });
      group->find_waitable_ptrs_if([&](const rclcpp::Waitable::SharedPtr & wp) {
        out.push_back(wp);
        return false;
      });
    });
  return true;
}

/// Match the `late` entry `reaction_id` to a handle of its node. Call with
/// g_late_mutex held.
///
/// Without `fresh`, only the entries of the node and family that
/// still wait for a handle are matched, against the handles no entry holds.
/// With `fresh` (a lifecycle gate epoch change), the new
/// handles, those no late entry of the node and family holds, are matched by
/// generation (lifecycle_gate_generation). An entry keeps a live handle unless
/// a new handle of a higher generation fits it. The entries whose handle is
/// gone take the new handles first, the entries with a live handle the rest,
/// each in creation order, and an entry only ever takes a handle newer than
/// the newest it held. An entry whose handle is gone and finds none is
/// cleared. Either way only the entry of `reaction_id` is written, and only
/// when its handle changes.
void match_late_locked(int reaction_id, bool fresh)
{
  const auto & rid_entries = get_rid_entries();
  const RidEntry * self = nullptr;
  for (const auto & e : rid_entries) {
    if (e.reaction_id == reaction_id && (is_timer_family(e.kind) || e.kind == "sub")) {
      self = &e;
      break;
    }
  }
  if (self == nullptr) {
    return;
  }
  const std::string & node_class = self->node_class;
  const bool timers = is_timer_family(self->kind);
  const auto node_base = node_base_of(node_class);
  if (!node_base) {
    return;
  }

  // Matching all waiting entries, rather than this entry alone, keeps the
  // analyzer's creation order when two entries fit the same handles.
  std::vector<const RidEntry *> entries;
  std::set<const void *> taken;
  for (const auto & e : rid_entries) {
    if (e.node_class != node_class || e.reaction_id < 0 ||
      e.reaction_id >= static_cast<int>(g_handle_table.size()))
    {
      continue;
    }
    const auto & h = g_handle_table[e.reaction_id];
    if (!fresh || !h.late) {
      if (auto t = h.timer.lock()) {
        taken.insert(t.get());
      }
      if (auto sp = h.subscription.lock()) {
        taken.insert(sp.get());
      }
    }
    if (h.late && (fresh || !holds_live_handle(h)) &&
      (timers ? is_timer_family(e.kind) : e.kind == "sub"))
    {
      entries.push_back(&e);
    }
  }
  const auto group_index = analyzer_group_index(node_base);
  const auto * parameter_events = parameter_events_subscription(node_base);
  std::vector<rclcpp::TimerBase::SharedPtr> timer_handles;
  std::vector<int64_t> periods;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sub_handles;
  std::vector<HandleCandidate> candidates;
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      const auto g = group_index.find(group.get());
      // A handle in a group another executor spins is never bound.
      if (g == group_index.end() || g->second == kGroupForeign) {
        return;
      }
      if (timers) {
        group->find_timer_ptrs_if([&](const rclcpp::TimerBase::SharedPtr & t) {
          if (taken.count(t.get()) == 0) {
            int64_t period_ns = -1;  // stays -1 if the timer handle is invalid
            const rcl_ret_t period_ret =
            rcl_timer_get_period(t->get_timer_handle().get(), &period_ns);
            (void)period_ret;
            timer_handles.push_back(t);
            periods.push_back(period_ns);
            candidates.push_back({"period " + std::to_string(period_ns) + " ns", g->second, ""});
          }
          return false;  // visit every timer
        });
      } else {
        group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
          if (taken.count(sp.get()) == 0 && sp.get() != parameter_events &&
          !lf_is_time_source_subscription(sp.get()))
          {
            sub_handles.push_back(sp);
            candidates.push_back(
              {"topic '" + std::string(sp->get_topic_name()) + "'", g->second, ""});
          }
          return false;  // visit every subscription
        });
      }
    });

  MatchOptions options;
  options.defer_missing = true;
  options.allow_unclaimed = true;
  const std::string family = timers ? "timer" : "subscription";
  auto fits = [&](const RidEntry & e, std::size_t h) {
      return timers ? timer_fits(periods[h], e) :
             topic_fits(sub_handles[h]->get_topic_name(), e, node_base);
    };
  auto report = [&](const HandleMatch & m) {
      static std::set<int> reported;
      if (!m.ok && reported.insert(reaction_id).second) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ros2_lf_binding"),
          "bind_late_handle: %s. RID %d stays unbound.", m.error.c_str(),
          reaction_id);
      }
      return m.ok;
    };
  // handle_of[i]: the candidate entries[i] is bound to, SIZE_MAX for none.
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> handle_of(entries.size(), kNone);
  std::vector<bool> keeps(entries.size(), false);  // a live handle, unchanged
  if (!fresh) {
    const auto match = match_handles(node_class, family, entries, candidates, fits, options);
    if (!report(match)) {
      return;
    }
    handle_of = match.handle_of;
  } else {
    // The candidates hold every live handle, the late entries' own as well.
    std::vector<int64_t> floor(entries.size());
    std::vector<uint64_t> gen(candidates.size());
    std::vector<bool> fresh_handle(candidates.size(), true);
    std::map<const void *, std::size_t> index_of;
    for (std::size_t c = 0; c < candidates.size(); ++c) {
      const void * ptr = timers ? static_cast<const void *>(timer_handles[c].get()) :
        static_cast<const void *>(sub_handles[c].get());
      index_of[ptr] = c;
      gen[c] = lifecycle_gate_generation(node_class, ptr);
    }
    for (std::size_t i = 0; i < entries.size(); ++i) {
      auto & h = g_handle_table[entries[i]->reaction_id];
      const auto t = h.timer.lock();
      const auto sp = h.subscription.lock();
      const void * cur = timers ? static_cast<const void *>(t.get()) :
        static_cast<const void *>(sp.get());
      floor[i] = generation_floor(h, node_class, cur);
      const auto c = index_of.find(cur);
      if (cur != nullptr && c != index_of.end()) {
        handle_of[i] = c->second;
        keeps[i] = true;
        fresh_handle[c->second] = false;
      }
    }
    // First the entries whose handle is gone, then those with a live one.
    for (const bool live : {false, true}) {
      std::vector<const RidEntry *> group;
      std::vector<std::size_t> at;
      for (std::size_t i = 0; i < entries.size(); ++i) {
        if (keeps[i] == live) {
          group.push_back(entries[i]);
          at.push_back(i);
        }
      }
      std::vector<HandleCandidate> open;
      std::vector<std::size_t> open_at;
      for (std::size_t c = 0; c < candidates.size(); ++c) {
        if (fresh_handle[c]) {
          open.push_back(candidates[c]);
          open_at.push_back(c);
        }
      }
      if (group.empty() || open.empty()) {
        continue;
      }
      std::map<const RidEntry *, std::size_t> position;
      for (std::size_t k = 0; k < group.size(); ++k) {
        position[group[k]] = at[k];
      }
      const auto match = match_handles(
        node_class, family, group, open,
        [&](const RidEntry & e, std::size_t o) {
          return static_cast<int64_t>(gen[open_at[o]]) > floor[position.at(&e)] &&
                 fits(e, open_at[o]);
        }, options);
      if (!report(match)) {
        return;
      }
      for (std::size_t k = 0; k < group.size(); ++k) {
        if (match.handle_of[k] != kNone) {
          handle_of[at[k]] = open_at[match.handle_of[k]];
          fresh_handle[open_at[match.handle_of[k]]] = false;
        }
      }
    }
  }
  auto & own = g_handle_table[reaction_id];
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (entries[i] != self) {
      continue;
    }
    const std::size_t h = handle_of[i];
    if (h == kNone) {
      if (fresh && own.kind != HandleKind::UNBOUND) {
        // Unbind. A timer stays cancelled: LF owns its firing until the node
        // drops it, and un-cancelling would let a stock executor fire it.
        own.kind = HandleKind::UNBOUND;
        own.timer.reset();
        own.subscription.reset();
        RCLCPP_INFO(
          rclcpp::get_logger("ros2_lf_binding"),
          "Unbound %s RID %d (%s::%s): its lifecycle node holds no fitting handle",
          self->kind.c_str(), reaction_id, node_class.c_str(), self->callback_name.c_str());
      }
    } else if (timers) {
      if (own.kind != HandleKind::TIMER || own.timer.lock() != timer_handles[h]) {
        bind_timer(*self, timer_handles[h], fresh ? "lifecycle " : "late ");
      }
      generation_floor(own, node_class, timer_handles[h].get());
    } else {
      if (own.kind != HandleKind::SUBSCRIPTION || own.subscription.lock() != sub_handles[h]) {
        bind_subscription(*self, sub_handles[h], fresh ? "lifecycle " : "late ");
      }
      generation_floor(own, node_class, sub_handles[h].get());
    }
  }
}

}  // namespace

bool names_match(const std::string & handle_name, const std::string & rid_name)
{
  return handle_name == rid_name || handle_name == "/" + rid_name || "/" + handle_name == rid_name;
}

bool name_fits(
  const std::string & handle_name, const std::string & rid_name,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  char * out = nullptr;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  if (node_base && rcl_node_resolve_name(
      node_base->get_rcl_node_handle(), rid_name.c_str(), alloc, true, false, &out) == RCL_RET_OK)
  {
    const bool fits = handle_name == out;
    alloc.deallocate(out, alloc.state);
    return fits;
  }
  rcl_reset_error();
  return names_match(handle_name, rid_name);
}

/// The action name of an action server or client waitable, or "" when it
/// cannot be read. rclcpp_action exposes no name getter, so the waitable adds
/// itself to a scratch wait set and the name is cut from its first service
/// or client, "<action>/_action/send_goal".
std::string action_name_of(
  rclcpp::Waitable & waitable,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  rcl_wait_set_t ws = rcl_get_zero_initialized_wait_set();
  if (rcl_wait_set_init(
      &ws, waitable.get_number_of_ready_subscriptions(),
      waitable.get_number_of_ready_guard_conditions(), waitable.get_number_of_ready_timers(),
      waitable.get_number_of_ready_clients(), waitable.get_number_of_ready_services(),
      waitable.get_number_of_ready_events(),
      node_base->get_context()->get_rcl_context().get(),
      rcl_get_default_allocator()) != RCL_RET_OK)
  {
    rcl_reset_error();
    return "";
  }
  std::string name;
  try {
    waitable.add_to_wait_set(ws);
    const char * raw = nullptr;
    if (ws.size_of_services > 0 && ws.services[0] != nullptr) {
      raw = rcl_service_get_service_name(ws.services[0]);
    } else if (ws.size_of_clients > 0 && ws.clients[0] != nullptr) {
      raw = rcl_client_get_service_name(ws.clients[0]);
    }
    if (raw != nullptr) {
      name = raw;
      const auto cut = name.rfind("/_action/");
      name = cut == std::string::npos ? "" : name.substr(0, cut);
    }
  } catch (const std::exception &) {
    name.clear();
  }
  if (rcl_wait_set_fini(&ws) != RCL_RET_OK) {
    rcl_reset_error();
  }
  return name;
}

std::mutex g_cleanup_mutex;
std::vector<Ros2LfClientCleanup> g_client_cleanups;
/// Client names already warned about for want of a typed cleanup.
std::set<std::string> g_cleanup_warned;

/// Apply the first matching cleanup to `client`. Returns its result, or -1
/// when none matches.
template<typename CleanupT, typename ClientT>
int64_t apply_cleanup(const std::vector<CleanupT> & cleanups, ClientT & client)
{
  for (const auto & cleanup : cleanups) {
    const int64_t done = cleanup(client);
    if (done >= 0) {
      return done;
    }
  }
  return -1;
}

/// The lifecycle gate's deactivate hook: the node left the
/// active state, so the requests and goals it sent while active are dropped
/// at the client and cancelled at the server. A service response that arrives
/// later finds no pending entry, and a reactivated node starts with none. An
/// action client's goals have their callbacks stopped before the cancel, and
/// the responses that arrive while the node is inactive are executed under
/// the guard of action_client_guard.hpp, so rclcpp erases its own entries
/// and releases the goal handles.
void lifecycle_node_deactivated(const std::string & node_class)
{
  const auto node_base = node_base_of(node_class);
  if (!node_base) {
    return;
  }
  std::vector<rclcpp::ClientBase::SharedPtr> clients;
  std::vector<std::shared_ptr<rclcpp_action::ClientBase>> action_clients;
  const auto group_index = analyzer_group_index(node_base);
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      const auto g = group_index.find(group.get());
      if (g == group_index.end() || g->second == kGroupForeign) {
        return;
      }
      group->find_client_ptrs_if([&](const rclcpp::ClientBase::SharedPtr & cp) {
        clients.push_back(cp);
        return false;
      });
      group->find_waitable_ptrs_if([&](const rclcpp::Waitable::SharedPtr & wp) {
        if (auto ac = std::dynamic_pointer_cast<rclcpp_action::ClientBase>(wp)) {
          action_clients.push_back(ac);
        }
        return false;
      });
    });
  std::lock_guard<std::mutex> lock(g_cleanup_mutex);
  const auto unmatched = [&](const std::string & kind, const std::string & name) {
      if (g_cleanup_warned.insert(node_class + "|" + name).second) {
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "Lifecycle gate: %s deactivated, %s '%s' has no typed cleanup, its in-flight work "
          "stays", node_class.c_str(), kind.c_str(), name.c_str());
      }
    };
  for (const auto & client : clients) {
    try {
      const int64_t pruned = apply_cleanup(g_client_cleanups, *client);
      if (pruned < 0) {
        unmatched("service client", client->get_service_name());
        continue;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding"),
        "Lifecycle gate: %s deactivated, pruned %lld pending request(s) of client '%s'",
        node_class.c_str(), static_cast<long long>(pruned),  // NOLINT(runtime/int)
        client->get_service_name());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_lf_binding"), "Lifecycle gate: %s deactivated, prune of "
        "client '%s' failed: %s", node_class.c_str(), client->get_service_name(), e.what());
    }
  }
  for (const auto & client : action_clients) {
    const std::string name = action_name_of(*client, node_base);
    try {
      const int64_t stopped = action_client_deactivate(*client);
      if (stopped < 0) {
        unmatched("action client", name);
        continue;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_lf_binding"),
        "Lifecycle gate: %s deactivated, stopped the callbacks of %lld goal(s) and cancelled "
        "the goals of action client '%s'", node_class.c_str(),
        static_cast<long long>(stopped), name.c_str());  // NOLINT(runtime/int)
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_lf_binding"), "Lifecycle gate: %s deactivated, cancel on "
        "action client '%s' failed: %s", node_class.c_str(), name.c_str(), e.what());
    }
  }
}

/// True when an action waitable serves the RID's action. A waitable whose
/// name cannot be read serves none, and says so once per RID name.
bool action_matches(
  rclcpp::Waitable & waitable, const std::string & rid_name,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  const std::string name = action_name_of(waitable, node_base);
  if (name.empty()) {
    static std::mutex reported_mutex;
    static std::set<std::string> reported;
    std::lock_guard<std::mutex> lock(reported_mutex);
    if (reported.insert(rid_name).second) {
      RCLCPP_WARN(
        rclcpp::get_logger("ros2_lf_binding"),
        "An action waitable's name cannot be read, so it is not bound to the action '%s'",
        rid_name.c_str());
    }
    return false;
  }
  return name_fits(name, rid_name, node_base);
}

namespace
{

/// Re-bind a service, client or action entry of a lifecycle node to the
/// handle the node holds now, by the startup bind's name rules.
/// Call with g_late_mutex held. Writes only this entry, and only
/// when a handle is found that differs from the bound one.
void rebind_named_locked(int reaction_id)
{
  const RidEntry * self = nullptr;
  for (const auto & e : get_rid_entries()) {
    if (e.reaction_id == reaction_id) {
      self = &e;
      break;
    }
  }
  if (self == nullptr) {
    return;
  }
  const std::string & kind = self->kind;
  const bool srv = kind == "srv_server";
  const bool cli = kind == "srv_client" || kind == "srv_client_cb";
  const bool action_srv = kind == "action_goal_server" || kind == "action_cancel_server" ||
    kind == "action_result_poll";
  const bool action_cli = kind == "action_goal_cb" || kind == "action_feedback" ||
    kind == "action_result_cb" || kind == "action_cancel_cb";
  if (!srv && !cli && !action_srv && !action_cli) {
    return;
  }
  const auto node_base = node_base_of(self->node_class);
  if (!node_base) {
    return;
  }
  auto & h = g_handle_table[reaction_id];
  // Of the fitting handles, only one of a higher generation than the newest
  // the entry held replaces it, the newest and, among equals, the first
  // found: a hook that replaced the handle while the node keeps the old one
  // alive meant the new one, and the old one never comes back.
  const HandleKind want = srv ? HandleKind::SERVICE_SERVER : cli ? HandleKind::SERVICE_CLIENT :
    action_srv ? HandleKind::ACTION_SERVER : HandleKind::ACTION_CLIENT;
  const void * current = nullptr;
  if (h.kind == want) {
    current = srv ? static_cast<const void *>(h.service_server.lock().get()) :
      cli ? static_cast<const void *>(h.service_client.lock().get()) :
      static_cast<const void *>(h.action_waitable.lock().get());
  }
  int64_t best = generation_floor(h, self->node_class, current);
  auto newer = [&](const void * candidate) {
      const auto g = static_cast<int64_t>(lifecycle_gate_generation(self->node_class, candidate));
      if (g <= best) {
        return false;
      }
      best = g;
      return true;
    };
  rclcpp::ServiceBase::SharedPtr best_srv;
  rclcpp::ClientBase::SharedPtr best_cli;
  rclcpp::Waitable::SharedPtr best_action;
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      if (srv) {
        group->find_service_ptrs_if([&](const rclcpp::ServiceBase::SharedPtr & sp) {
          if (name_fits(sp->get_service_name(), self->topic, node_base) && newer(sp.get())) {
            best_srv = sp;
          }
          return false;
        });
      } else if (cli) {
        group->find_client_ptrs_if([&](const rclcpp::ClientBase::SharedPtr & cp) {
          if (name_fits(cp->get_service_name(), self->topic, node_base) && newer(cp.get())) {
            best_cli = cp;
          }
          return false;
        });
      } else {
        // The action server or client of the RID's action name, as at startup.
        group->find_waitable_ptrs_if([&](const rclcpp::Waitable::SharedPtr & wp) {
          const bool fits = action_srv ?
          dynamic_cast<rclcpp_action::ServerBase *>(wp.get()) != nullptr :
          dynamic_cast<rclcpp_action::ClientBase *>(wp.get()) != nullptr;
          if (fits && action_matches(*wp, self->topic, node_base) && newer(wp.get())) {
            best_action = wp;
          }
          return false;
        });
      }
    });
  if (best_srv) {
    h.kind = want;
    h.service_server = best_srv;
  } else if (best_cli) {
    h.kind = want;
    h.service_client = best_cli;
  } else if (best_action) {
    h.kind = want;
    h.action_waitable = best_action;
  }
  h.generation = best;
  // Otherwise the bound handle is kept, or, gone and unreplaced, it stays
  // expired and the dispatch skips it.
}

/// True when a service, client or action entry holds a handle that has
/// expired since it was bound.
bool named_handle_expired(const HandleEntry & h)
{
  switch (h.kind) {
    case HandleKind::SERVICE_SERVER: return h.service_server.expired();
    case HandleKind::SERVICE_CLIENT: return h.service_client.expired();
    case HandleKind::ACTION_SERVER:
    case HandleKind::ACTION_CLIENT: return h.action_waitable.expired();
    default: return false;
  }
}

}  // namespace

void bind_late_handle(int reaction_id)
{
  if (reaction_id < 0 || reaction_id >= static_cast<int>(g_handle_table.size()) ||
    !g_handle_table[reaction_id].late || holds_live_handle(g_handle_table[reaction_id]))
  {
    return;
  }
  std::lock_guard<std::mutex> lock(g_late_mutex);
  match_late_locked(reaction_id, false);
}

bool lifecycle_admit(int reaction_id)
{
  // Before any user callback of this tag, whichever reaction comes first.
  lifecycle_gate_boundary();
  if (reaction_id < 0 || reaction_id >= static_cast<int>(g_handle_table.size())) {
    return true;
  }
  auto & h = g_handle_table[reaction_id];
  if (!h.lifecycle) {
    return true;
  }
  const auto view = lifecycle_gate_view(h.node_class);
  if (!view) {
    return true;
  }
  if (view->epoch != h.gate_epoch) {
    std::lock_guard<std::mutex> lock(g_late_mutex);
    if (h.late) {
      match_late_locked(reaction_id, true);
    } else {
      rebind_named_locked(reaction_id);
    }
    h.gate_epoch = view->epoch;
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "Lifecycle gate: RID %d (%s::%s) reconciled at epoch %llu, node state %u",
      reaction_id, h.node_class.c_str(), h.callback_name.c_str(),
      static_cast<unsigned long long>(view->epoch),  // NOLINT(runtime/int)
      static_cast<unsigned>(view->state));
  }
  if (h.kind == HandleKind::SUBSCRIPTION && corr_rt::deferred_reader_pending()) {
    // A reader bound while the one it replaces still lived.
    std::lock_guard<std::mutex> lock(g_late_mutex);
    corr_rt::retry_deferred_reader(reaction_id);
  }
  constexpr uint8_t kActive = 3;  // lifecycle_msgs::msg::State
  if (view->state == kActive && !h.late && named_handle_expired(h)) {
    // A deactivate and activate inside one callback ends in the state it
    // began with, so no epoch moved, but the hooks replaced the handle.
    std::lock_guard<std::mutex> lock(g_late_mutex);
    rebind_named_locked(reaction_id);
  }
  return view->state == kActive;
}

}  // namespace ros2_lf_binding

extern "C" {

void ros2_lf_bind_node(void * /*self*/, const char * node_class)
{
  // The generated LF startup reaction binds every node, so the first bind
  // marks LF as running.
  ros2_lf_binding::lf_mark_running();
  // Binding runs in the startup reaction: give node clocks the start tag
  // before lifecycle callbacks run below.
  ros2_lf_binding::lf_time_advance_ros_clock();
  // the start tag exists now; trace it once (lf_start).
  ros2_lf_binding::corr_rt::trace_lf_start();
  // Test-only: widen the bind window so a missing
  // bind-before-dispatch edge shows up as an UNBOUND tag-0 dispatch.
  if (const char * d = std::getenv("RCLLF_TEST_BIND_DELAY_MS")) {
    const int ms = std::atoi(d);
    if (ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
  }
  if (!ros2_lf_binding::g_manager) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"), "ros2_lf_bind_node(%s): runtime not initialised",
      node_class);
    return;
  }

  // Find the loaded component by C++ class name using the class→id registry.
  auto node_base = ros2_lf_binding::g_manager->find_by_class(node_class);

  // Fallback: non-component nodes are not in the component manager.
  // They are registered in the node registry by generated node_factories.cpp.
  // ros2_lf_get_node already returns the base interface (the
  // registry is type-erased, since rclcpp_lifecycle::LifecycleNode is a
  // sibling of rclcpp::Node, not a subclass).
  if (!node_base) {
    node_base = ros2_lf_get_node(node_class);
  }

  if (!node_base) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node: no loaded node matches class '%s'",
      node_class);
    return;
  }

  // Drive the managed lifecycle before discovering entities.
  //
  // A lifecycle node idiomatically creates its publishers in on_configure and
  // its timers in on_activate, so a node still in `unconfigured` owns none of
  // them and the callback-group walk below would find nothing. Transitioning
  // here, rather than re-binding after the fact, keeps binding a single pass:
  // g_handle_table is documented read-only once startup reactions complete, and
  // a second pass would re-cancel live timers and could repoint a RID's entry
  // mid-run.
  //
  // No-op for plain rclcpp::Node and for component nodes (which the component
  // manager owns, and which are not in the registry).
  if (!ros2_lf_transition_node(node_class)) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): lifecycle bring-up did not reach the active "
      "state; binding will only see entities that already exist",
      node_class);
  }
  // A lifecycle node creates and drops its timers and subscriptions on its
  // transitions. Each of its timer and subscription entries is bound late to
  // the handle that fits it once one exists. Only a node that is
  // not active yet, because its activation was declined (auto_transition:
  // false) or failed, may lack handles now. Any other missing handle is a
  // missing callback and refuses to start.
  constexpr int kLifecycleActive = 3;  // lifecycle_msgs::msg::State
  const int lifecycle_state = ros2_lf_node_lifecycle_state(node_class);
  const bool lifecycle = lifecycle_state >= 0;
  ros2_lf_binding::MatchOptions match_options;
  match_options.defer_missing = lifecycle && lifecycle_state != kLifecycleActive;

  // Collect all timers, subscriptions, services, clients and waitables from all callback groups.
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::vector<rclcpp::ServiceBase::SharedPtr> services;
  std::vector<rclcpp::ClientBase::SharedPtr> clients;
  std::vector<rclcpp::Waitable::SharedPtr> waitables;
  // The callback group of each timer and subscription, for matching within
  // groups.
  std::vector<rclcpp::CallbackGroup::SharedPtr> timer_group;
  std::vector<rclcpp::CallbackGroup::SharedPtr> sub_group;

  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
    // Timers
      auto t = group->find_timer_ptrs_if([&timers](const rclcpp::TimerBase::SharedPtr & tp) {
        for (auto & existing : timers) {
          if (existing.get() == tp.get()) {
            return false;
          }
        }
        return true;
    });
      while (t) {
        timers.push_back(t);
        timer_group.push_back(group);
        t = group->find_timer_ptrs_if([&timers](const rclcpp::TimerBase::SharedPtr & tp) {
          for (auto & e : timers) {
            if (e.get() == tp.get()) {
              return false;
            }
          }
          return true;
      });
      }
    // Subscriptions
      auto s = group->find_subscription_ptrs_if(
        [&subscriptions](const rclcpp::SubscriptionBase::SharedPtr & sp) {
          for (auto & existing : subscriptions) {
            if (existing.get() == sp.get()) {
              return false;
            }
          }
          return true;
      });
      while (s) {
        subscriptions.push_back(s);
        sub_group.push_back(group);
        s = group->find_subscription_ptrs_if(
          [&subscriptions](const rclcpp::SubscriptionBase::SharedPtr & sp) {
            for (auto & e : subscriptions) {
              if (e.get() == sp.get()) {
                return false;
              }
            }
            return true;
        });
      }
    // Services
      auto svc = group->find_service_ptrs_if(
        [&services](const rclcpp::ServiceBase::SharedPtr & sp) {
          for (auto & existing : services) {
            if (existing.get() == sp.get()) {
              return false;
            }
          }
          return true;
      });
      while (svc) {
        services.push_back(svc);
        svc = group->find_service_ptrs_if(
          [&services](const rclcpp::ServiceBase::SharedPtr & sp) {
            for (auto & e : services) {
              if (e.get() == sp.get()) {
                return false;
              }
            }
            return true;
        });
      }
    // Clients
      auto cli = group->find_client_ptrs_if(
        [&clients](const rclcpp::ClientBase::SharedPtr & cp) {
          for (auto & existing : clients) {
            if (existing.get() == cp.get()) {
              return false;
            }
          }
          return true;
      });
      while (cli) {
        clients.push_back(cli);
        cli = group->find_client_ptrs_if(
          [&clients](const rclcpp::ClientBase::SharedPtr & cp) {
            for (auto & e : clients) {
              if (e.get() == cp.get()) {
                return false;
              }
            }
            return true;
        });
      }
    // Waitables (action servers and clients)
      auto w = group->find_waitable_ptrs_if(
        [&waitables](const rclcpp::Waitable::SharedPtr & wp) {
          for (auto & existing : waitables) {
            if (existing.get() == wp.get()) {
              return false;
            }
          }
          return true;
      });
      while (w) {
        waitables.push_back(w);
        w = group->find_waitable_ptrs_if(
          [&waitables](const rclcpp::Waitable::SharedPtr & wp) {
            for (auto & e : waitables) {
              if (e.get() == wp.get()) {
                return false;
              }
            }
            return true;
        });
      }
  });

  // Match against the static RID table and populate g_handle_table.
  const auto & rid_entries = ros2_lf_binding::get_rid_entries();

  int bound_timers = 0;
  int bound_subs = 0;
  int bound_services = 0;
  int bound_clients = 0;
  std::vector<const rclcpp::SubscriptionBase *> bound_sub_ptrs;

  // Timer-driven and subscription RIDs are matched to handles one to one
  // before binding: each handle serves at most one RID, and a RID
  // without a handle or a handle without a RID refuses to start.
  std::vector<const ros2_lf_binding::RidEntry *> timer_entries;
  std::vector<const ros2_lf_binding::RidEntry *> sub_entries;
  for (const auto & entry : rid_entries) {
    if (entry.node_class != node_class || entry.reaction_id < 0 ||
      entry.reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size()))
    {
      continue;
    }
    if (entry.kind == "timer" || entry.kind == "srv_send" || entry.kind == "action_goal_send") {
      timer_entries.push_back(&entry);
    } else if (entry.kind == "sub") {
      sub_entries.push_back(&entry);
    }
  }
  // rclcpp's own handles take no part in matching. They are recognised by
  // where they come from, not by topic: the TimeSource /clock subscription
  // the binding feeds, a group the TimeSource created for its clock thread,
  // and the TimeSource /parameter_events subscription.
  const auto * parameter_events_sub = ros2_lf_binding::parameter_events_subscription(node_base);
  const auto group_index = ros2_lf_binding::analyzer_group_index(node_base);
  const std::string acknowledged = ros2_lf_binding::foreign_executor_ack(node_class);
  auto make_candidate = [&](const std::string & description, const rclcpp::CallbackGroup * g) {
      ros2_lf_binding::HandleCandidate c{description, group_index.at(g), ""};
      if (c.group == ros2_lf_binding::kGroupForeign) {
        c.acknowledged_hazard = acknowledged;
        c.tf2_listener = ros2_lf_binding::is_detached_tf2_group(g);
      }
      return c;
    };
  {
    std::vector<rclcpp::TimerBase::SharedPtr> kept;
    std::vector<rclcpp::CallbackGroup::SharedPtr> kept_group;
    for (std::size_t i = 0; i < timers.size(); ++i) {
      if (group_index.count(timer_group[i].get())) {
        kept.push_back(timers[i]);
        kept_group.push_back(timer_group[i]);
      }
    }
    timers.swap(kept);
    timer_group.swap(kept_group);
  }
  {
    std::vector<rclcpp::SubscriptionBase::SharedPtr> kept;
    std::vector<rclcpp::CallbackGroup::SharedPtr> kept_group;
    for (std::size_t i = 0; i < subscriptions.size(); ++i) {
      if (group_index.count(sub_group[i].get()) && subscriptions[i].get() != parameter_events_sub &&
        !ros2_lf_binding::lf_is_time_source_subscription(subscriptions[i].get()))
      {
        kept.push_back(subscriptions[i]);
        kept_group.push_back(sub_group[i]);
      }
    }
    subscriptions.swap(kept);
    sub_group.swap(kept_group);
  }
  std::vector<int64_t> timer_periods(timers.size(), -1);
  std::vector<ros2_lf_binding::HandleCandidate> timer_candidates;
  for (std::size_t i = 0; i < timers.size(); ++i) {
    const rcl_ret_t period_ret =
      rcl_timer_get_period(timers[i]->get_timer_handle().get(), &timer_periods[i]);
    (void)period_ret;
    timer_candidates.push_back(
      make_candidate("period " + std::to_string(timer_periods[i]) + " ns", timer_group[i].get()));
  }
  std::vector<ros2_lf_binding::HandleCandidate> sub_candidates;
  for (std::size_t i = 0; i < subscriptions.size(); ++i) {
    sub_candidates.push_back(
      make_candidate(
        "topic '" + std::string(subscriptions[i]->get_topic_name()) + "'",
        sub_group[i].get()));
  }
  const auto timer_match = ros2_lf_binding::match_handles(
    node_class, "timer", timer_entries, timer_candidates,
    [&timer_periods](const ros2_lf_binding::RidEntry & e, std::size_t h) {
      return ros2_lf_binding::timer_fits(timer_periods[h], e);
    }, match_options);
  if (!timer_match.ok) {
    ros2_lf_binding::fail_bind(timer_match.error);
  }
  for (const std::size_t h : timer_match.exempted) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): timer handle #%zu (%s) runs outside LF in a callback group "
      "another executor spins, under acknowledged %s", node_class, h,
      timer_candidates[h].description.c_str(), acknowledged.c_str());
  }
  const auto sub_match = ros2_lf_binding::match_handles(
    node_class, "subscription", sub_entries, sub_candidates,
    [&subscriptions, &node_base](const ros2_lf_binding::RidEntry & e, std::size_t h) {
      return ros2_lf_binding::topic_fits(subscriptions[h]->get_topic_name(), e, node_base);
    }, match_options);
  if (!sub_match.ok) {
    ros2_lf_binding::fail_bind(sub_match.error);
  }
  for (const std::size_t h : sub_match.exempted) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): subscription handle #%zu (%s) runs outside LF in a callback "
      "group another executor spins, under acknowledged %s", node_class, h,
      sub_candidates[h].description.c_str(), acknowledged.c_str());
  }
  for (const std::size_t i : timer_match.deferred) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): %s RID %d (%s) has no timer handle in lifecycle state %d; "
      "it binds when the node creates one", node_class,
      timer_entries[i]->kind.c_str(), timer_entries[i]->reaction_id,
      timer_entries[i]->callback_name.c_str(), lifecycle_state);
  }
  for (const std::size_t i : sub_match.deferred) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): subscription RID %d (%s, topic '%s') has no handle in lifecycle "
      "state %d; it binds when the node creates one", node_class,
      sub_entries[i]->reaction_id, sub_entries[i]->callback_name.c_str(),
      sub_entries[i]->topic.c_str(), lifecycle_state);
  }
  constexpr std::size_t kNoHandle = std::numeric_limits<std::size_t>::max();
  std::map<int, rclcpp::TimerBase::SharedPtr> timer_of_rid;
  for (std::size_t i = 0; i < timer_entries.size(); ++i) {
    if (timer_match.handle_of[i] != kNoHandle) {
      timer_of_rid[timer_entries[i]->reaction_id] = timers[timer_match.handle_of[i]];
    }
  }
  std::map<int, rclcpp::SubscriptionBase::SharedPtr> sub_of_rid;
  for (std::size_t i = 0; i < sub_entries.size(); ++i) {
    if (sub_match.handle_of[i] != kNoHandle) {
      sub_of_rid[sub_entries[i]->reaction_id] = subscriptions[sub_match.handle_of[i]];
    }
  }

  for (const auto & entry : rid_entries) {
    if (entry.node_class != node_class) {
      continue;
    }
    int rid = entry.reaction_id;
    if (rid < 0 || rid >= static_cast<int>(ros2_lf_binding::g_handle_table.size())) {
      continue;
    }

    if (ros2_lf_binding::is_timer_family(entry.kind) || entry.kind == "sub") {
      // Matched one to one above. An entry of a lifecycle node is bound late
      // when its handle is missing now or expires on a later transition.
      auto & handle = ros2_lf_binding::g_handle_table[rid];
      handle.late = lifecycle;
      handle.node_class = node_class;
      handle.callback_name = entry.callback_name;
      handle.topic = entry.topic;
      if (lifecycle && entry.kind == "sub") {
        ros2_lf_binding::external_input::on_lifecycle_entry(rid);
      }
      const auto timer = timer_of_rid.find(rid);
      const auto sub = sub_of_rid.find(rid);
      if (timer != timer_of_rid.end()) {
        ros2_lf_binding::bind_timer(entry, timer->second, "");
        bound_timers++;
      } else if (sub != sub_of_rid.end()) {
        ros2_lf_binding::bind_subscription(entry, sub->second, "");
        bound_subs++;
        bound_sub_ptrs.push_back(sub->second.get());
      }
    } else if (entry.kind == "srv_server") {
      // Match by service name
      for (auto & svc : services) {
        std::string svc_name = svc->get_service_name();
        if (ros2_lf_binding::name_fits(svc_name, entry.topic, node_base)) {
          ros2_lf_binding::g_handle_table[rid].kind = ros2_lf_binding::HandleKind::SERVICE_SERVER;
          ros2_lf_binding::g_handle_table[rid].service_server = svc;
          ros2_lf_binding::service_pairing::register_server(
            rid, svc, node_base->get_shared_rcl_node_handle());
          ros2_lf_binding::g_handle_table[rid].topic = entry.topic;
          ros2_lf_binding::g_handle_table[rid].node_class = node_class;
          ros2_lf_binding::g_handle_table[rid].callback_name = entry.callback_name;
          bound_services++;
          RCLCPP_INFO(
            rclcpp::get_logger("ros2_lf_binding"),
            "Bound service server RID %d (%s::%s, service '%s')", rid, node_class,
            entry.callback_name.c_str(), entry.topic.c_str());
          break;
        }
      }
    } else if (entry.kind == "srv_client" || entry.kind == "srv_client_cb") {
      // Match by service name
      for (auto & cli : clients) {
        std::string svc_name = cli->get_service_name();
        if (ros2_lf_binding::name_fits(svc_name, entry.topic, node_base)) {
          ros2_lf_binding::g_handle_table[rid].kind = ros2_lf_binding::HandleKind::SERVICE_CLIENT;
          ros2_lf_binding::g_handle_table[rid].service_client = cli;
          ros2_lf_binding::service_pairing::register_client(rid, cli);
          ros2_lf_binding::g_handle_table[rid].topic = entry.topic;
          ros2_lf_binding::g_handle_table[rid].node_class = node_class;
          ros2_lf_binding::g_handle_table[rid].callback_name = entry.callback_name;
          bound_clients++;
          RCLCPP_INFO(
            rclcpp::get_logger("ros2_lf_binding"),
            "Bound service client RID %d (%s::%s, service '%s')", rid, node_class,
            entry.callback_name.c_str(), entry.topic.c_str());
          break;
        }
      }
    } else if (
      entry.kind == "action_goal_server" ||
      entry.kind == "action_cancel_server" ||
      entry.kind == "action_result_server")
    {
      // Action server: match by action name using dynamic_cast to ServerBase.
      // action_goal_send (client timer) is bound as a TIMER handle above.
      // action_cancel_server / action_result_server share the same ACTION_SERVER
      // handle as action_goal_server — all three dispatch through the same
      // waitable, just a different entity id (see
      // ros2_lf_invoke_action_cancel_server / ros2_lf_invoke_action_result_server).
      for (auto & w : waitables) {
        auto * srv_base = dynamic_cast<rclcpp_action::ServerBase *>(w.get());
        // ServerBase exposes no get_action_name(), action_name_of reads it
        // from the server's goal service.
        if (!srv_base || !ros2_lf_binding::action_matches(*w, entry.topic, node_base)) {
          continue;
        }
        ros2_lf_binding::g_handle_table[rid].kind = ros2_lf_binding::HandleKind::ACTION_SERVER;
        ros2_lf_binding::g_handle_table[rid].action_waitable = w;
        ros2_lf_binding::g_handle_table[rid].topic = entry.topic;
        ros2_lf_binding::g_handle_table[rid].node_class = node_class;
        ros2_lf_binding::g_handle_table[rid].callback_name = entry.callback_name;
        RCLCPP_INFO(
          rclcpp::get_logger("ros2_lf_binding"),
          "Bound action server RID %d (%s::%s, action '%s')", rid, node_class,
          entry.callback_name.c_str(), entry.topic.c_str());
        if (entry.kind == "action_goal_server") {
          ros2_lf_binding::action_external_on_bound(rid);
        }
        break;
      }
    } else if (entry.kind == "action_goal_cb" || entry.kind == "action_feedback" ||
      entry.kind == "action_result_cb" || entry.kind == "action_cancel_cb")
    {
      // Action client: match by action name using dynamic_cast to ClientBase.
      for (auto & w : waitables) {
        auto * cli_base = dynamic_cast<rclcpp_action::ClientBase *>(w.get());
        if (!cli_base || !ros2_lf_binding::action_matches(*w, entry.topic, node_base)) {
          continue;
        }
        ros2_lf_binding::g_handle_table[rid].kind = ros2_lf_binding::HandleKind::ACTION_CLIENT;
        ros2_lf_binding::g_handle_table[rid].action_waitable = w;
        // Set entity_id for take_data_by_entity_id:
        // GoalClient=0, ResultClient=1, CancelClient=2, FeedbackSub=3, StatusSub=4
        if (entry.kind == "action_goal_cb") {
          ros2_lf_binding::g_handle_table[rid].action_entity_id = 0;
        } else if (entry.kind == "action_result_cb") {
          ros2_lf_binding::g_handle_table[rid].action_entity_id = 1;
        } else if (entry.kind == "action_cancel_cb") {
          ros2_lf_binding::g_handle_table[rid].action_entity_id = 2;  // CancelClient
        } else {
          ros2_lf_binding::g_handle_table[rid].action_entity_id = 3;  // FeedbackSubscription
        }
        ros2_lf_binding::g_handle_table[rid].topic = entry.topic;
        ros2_lf_binding::g_handle_table[rid].node_class = node_class;
        ros2_lf_binding::g_handle_table[rid].callback_name = entry.callback_name;
        RCLCPP_INFO(
          rclcpp::get_logger("ros2_lf_binding"),
          "Bound action client RID %d (%s::%s, action '%s', entity %zu)", rid, node_class,
          entry.callback_name.c_str(), entry.topic.c_str(),
          ros2_lf_binding::g_handle_table[rid].action_entity_id);
        break;
      }
    }
  }

  // Every entry of a lifecycle node dispatches through the node's lifecycle
  // gate, which starts from the state the bring-up above left, primary or not.
  if (lifecycle) {
    for (const auto & e : rid_entries) {
      if (e.node_class != node_class || e.reaction_id < 0 ||
        e.reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size()))
      {
        continue;
      }
      auto & h = ros2_lf_binding::g_handle_table[e.reaction_id];
      h.lifecycle = true;
      h.node_class = node_class;
      if (h.callback_name.empty()) {
        h.callback_name = e.callback_name;
      }
      if (h.topic.empty()) {
        h.topic = e.topic;
      }
    }
    // Registered whatever the state: a node caught mid-transition starts
    // gated as not active until a primary state is observed.
    ros2_lf_binding::lifecycle_gate_set_collector(&ros2_lf_binding::lifecycle_node_entities);
    ros2_lf_binding::lifecycle_gate_set_deactivate_hook(
      &ros2_lf_binding::lifecycle_node_deactivated);
    // A node with the lifecycle communication interface is named to the gate
    // by its state machine's change_state service, whose rcl impl the
    // rclcpp_lifecycle wrapper shares (see lifecycle_gate_register). Found by
    // its type, not its name, which a remap can change.
    std::vector<const void *> service_impls;
    node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr g) {
        g->find_service_ptrs_if([&](const rclcpp::ServiceBase::SharedPtr & s) {
          if (std::dynamic_pointer_cast<rclcpp::Service<lifecycle_msgs::srv::ChangeState>>(s)) {
            service_impls.push_back(s->get_service_handle()->impl);
          }
          return false;
        });
      });
    ros2_lf_binding::lifecycle_gate_register(
      node_class, static_cast<uint8_t>(lifecycle_state),
      service_impls);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "ros2_lf_bind_node(%s): %d timer(s), %d subscription(s), %d service(s), %d client(s) bound",
    node_class, bound_timers, bound_subs, bound_services, bound_clients);
  // Every subscription of the node, in any callback group, must have a RID
  // or an exemption, see bind_check.hpp.
  const auto check = ros2_lf_binding::check_node_subscriptions(
    *node_base, bound_sub_ptrs, acknowledged, ros2_lf_binding::lf_is_time_source_subscription);
  for (const auto * sub : check.time_source) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): subscription '%s' belongs to the rclcpp TimeSource, "
      "no RID needed", node_class, sub->get_topic_name());
  }
  for (const auto * sub : check.foreign_executor) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_bind_node(%s): subscription '%s' runs outside LF in a callback group "
      "another executor spins, under acknowledged %s", node_class,
      sub->get_topic_name(), acknowledged.c_str());
  }
  ros2_lf_binding::refuse_unbound_subscriptions(node_class, check.unbound);
  ros2_lf_binding::trace_bind(node_class);
}

int ros2_lf_topic_sent(int reaction_id, const char * topic)
{
  auto & counts = ros2_lf_binding::corr_rt::topic_sent_counts();
  if (topic == nullptr || counts.empty()) {
    return 0;
  }
  // The topic as the node source names it, resolved by the node's namespace and
  // remap rules (topic rules: is_service false), as resolved_rid_topic does.
  std::string resolved;
  if (reaction_id >= 0 &&
    reaction_id < static_cast<int>(ros2_lf_binding::g_handle_table.size()))
  {
    const auto node_base = ros2_lf_binding::node_base_of(
      ros2_lf_binding::g_handle_table[reaction_id].node_class);
    char * out = nullptr;
    rcl_allocator_t alloc = rcl_get_default_allocator();
    if (node_base && rcl_node_resolve_name(
        node_base->get_rcl_node_handle(), topic, alloc, false, false, &out) == RCL_RET_OK)
    {
      resolved = out;
      alloc.deallocate(out, alloc.state);
    } else {
      rcl_reset_error();
    }
  }
  int n = 0;
  for (auto it = counts.begin(); it != counts.end(); ) {
    const bool fits = resolved.empty() ?
      ros2_lf_binding::names_match(it->first, topic) : it->first == resolved;
    if (fits) {
      n += it->second;
      it = counts.erase(it);
    } else {
      ++it;
    }
  }
  return n;
}

void ros2_lf_output_deadline_arm(int reaction_id, const char * topic, int64_t bound_ns)
{
  if (topic == nullptr || reaction_id < 0 ||
    reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size()))
  {
    return;
  }
  // The generated line passes a string literal, so (rid, pointer, bound)
  // names one arm line: resolve it once per thread, not at every tag.
  thread_local std::set<std::tuple<int, const char *, int64_t>> t_armed;
  if (!t_armed.emplace(reaction_id, topic, bound_ns).second) {
    return;
  }
  // Resolved like ros2_lf_topic_sent, so it equals the publisher's own topic.
  std::string resolved = topic;
  const auto node_base = ros2_lf_binding::node_base_of(
    ros2_lf_binding::g_handle_table[reaction_id].node_class);
  char * out = nullptr;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  if (node_base && rcl_node_resolve_name(
      node_base->get_rcl_node_handle(), topic, alloc, false, false, &out) == RCL_RET_OK)
  {
    resolved = out;
    alloc.deallocate(out, alloc.state);
  } else {
    rcl_reset_error();
    // Console only, like the late lines: RCLCPP_WARN can reach /rosout.
    ros2_lf_binding::output_deadline::warn_unresolved_topic(reaction_id, topic);
  }
  ros2_lf_binding::output_deadline::arm(reaction_id, resolved, bound_ns);
}

void ros2_lf_claim_publish(const char * resolved_topic)
{
  if (resolved_topic != nullptr) {
    ros2_lf_binding::corr_rt::topic_sent_note(resolved_topic);
    if (ros2_lf_binding::corr_rt::origin_active()) {
      ros2_lf_binding::tag_log::on_claim(resolved_topic);
    }
  }
}

}  // extern "C"

void ros2_lf_register_client_cleanup(Ros2LfClientCleanup cleanup)
{
  std::lock_guard<std::mutex> lock(ros2_lf_binding::g_cleanup_mutex);
  ros2_lf_binding::g_client_cleanups.push_back(std::move(cleanup));
}

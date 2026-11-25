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

#include "tf2_ingress.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "component_manager.hpp"
#include "handle_matching.hpp"
#include "rclcpp/event_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rid_table.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "time_source.hpp"

namespace ros2_lf_binding
{
extern std::shared_ptr<LFComponentManager> g_manager;

namespace
{

// Detach rounds. Clearing can_be_taken_from() alone does not
// prove that tf2_ros's private executor is done with the group: rclcpp
// 28.1.22 checks the flag, then stores false and runs a callback of a
// MutuallyExclusive group, and stores true when the callback returns. A /tf
// callback in flight while the binding clears the flag therefore reads as
// "detached" for as long as it runs and re-enables the group when it ends
// (the old loop accepted three false reads 10 ms apart).
//
// Quiescence is established from the executor's wait set instead. Each round
// clears the flag and wakes the executor; the executor rebuilds its entity
// collection on its own thread, between callbacks, and drops a group whose
// flag is false, which removes the group's subscriptions from its
// rclcpp::WaitSet (wait_set_template.hpp: remove_subscription sets the
// subscription's in-use-by-wait-set state to false). The loop ends only when
// the flag is false and neither /tf nor /tf_static is in any wait set. Then
// no callback of the group can be in flight: one in flight came from the
// executor's last wait, so its subscription was still in the wait set, and
// the removal runs on the executor thread after that callback returned. Nor
// can one start: the executor re-adds the group only on a rebuild that reads
// the flag true, and only a returning callback of the group sets it true.
// After bind the binding's own ingress wait set holds both subscriptions, and
// rclcpp throws on a second wait set's add, so tf2's executor cannot take
// them back silently. reassert_tf2_groups_detached stays as a check.
//
// kRoundPeriod is a design choice (an order above the S0 wake latency of the
// Plan); kMaxRounds bounds startup at 1 s before the binding refuses to
// start, so a tf2 callback that runs longer than that fails closed (design
// choice, not a measured limit).
constexpr int kMaxRounds = 100;
constexpr auto kRoundPeriod = std::chrono::milliseconds(10);

// Why the probe below cannot make tf2's executor fail an add (F91b; sources
// of the pinned rclcpp 28.1.22 and tf2_ros 0.36.23 the image ships):
//
// 1. tf2_ros adds the listener group to its executor before it starts the spin
//    thread (transform_listener.hpp:267-269). add_callback_group calls
//    trigger_entity_recollect (executor.cpp:176-179), which, with the executor
//    not spinning yet, runs collect_entities on the constructing thread
//    (executor.cpp:136-140). The group's flag is still true then, so /tf and
//    /tf_static enter the executor's wait set (executor.cpp:707-711) before the
//    node's constructor returns, which is before detach runs.
// 2. A rebuild adds a subscription only when it is not yet in the executor's
//    collection (update_entities, executor_entities_collection.hpp:85-93) and
//    removes it only when a rebuild reads the group's flag false
//    (executor_entities_collection.cpp:79). Before detach clears the flag it is
//    false only between taking a callback (executor.cpp:905) and returning
//    from it (executor.cpp:502), and no rebuild runs in between on the single
//    spin thread (single_threaded_executor.cpp:40-43). So until detach's first
//    store the subscriptions stay in the executor's wait set.
// 3. After a rebuild removed them, no callback of the group can run again:
//    collect_entities drops the last wait result first (executor.cpp:677) and
//    the next waits do not hold the subscriptions. Nothing sets the flag true
//    again (only executor.cpp:502 does, and the binding only stores false), so
//    no later rebuild adds them back.
//
// Hence each subscription is added to tf2's wait set exactly once, before
// detach starts, and a probe that claims a subscription it finds free cannot
// collide with an add. Point 1 is checked at run time: detach refuses to start
// unless both subscriptions are in a wait set before it touches the flag
// (held_without_claiming), so a tf2_ros or rclcpp that collects the group
// later fails closed instead of racing.

/// True while `sub` is in some rclcpp::WaitSet. rclcpp exposes the state
/// only as an exchange: claim it, and give it back at once when it was free.
/// Safe only after held_without_claiming saw `sub` held (see above).
bool in_a_wait_set(const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  if (sub->exchange_in_use_by_wait_set_state(sub.get(), true)) {
    return true;  // it was in one; the exchange changed nothing
  }
  sub->exchange_in_use_by_wait_set_state(sub.get(), false);
  return false;
}

/// True while `sub` is in some rclcpp::WaitSet, read without ever marking a
/// free subscription as taken: a concurrent add finds it free. A held one is
/// cleared and restored; before detach's first store the executor neither
/// removes nor re-adds it (point 2 above), so the restore is exact.
bool held_without_claiming(const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  if (!sub->exchange_in_use_by_wait_set_state(sub.get(), false)) {
    return false;
  }
  sub->exchange_in_use_by_wait_set_state(sub.get(), true);
  return true;
}

std::mutex g_tf2_mutex;
std::set<const rclcpp::CallbackGroup *> g_tf2_groups;
// The same groups with their binding key, for the re-detach check.
std::vector<std::pair<std::string, std::weak_ptr<rclcpp::CallbackGroup>>> g_tf2_watch;

}  // namespace

int pick_tf2_listener_group(const std::vector<Tf2GroupView> & groups, std::string * error)
{
  int found = -1;
  int count = 0;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    const std::set<std::string> topics(groups[i].sub_topics.begin(), groups[i].sub_topics.end());
    if (groups[i].other_entities == 0 && groups[i].sub_topics.size() == 2 &&
      topics == std::set<std::string>{"/tf", "/tf_static"})
    {
      found = static_cast<int>(i);
      ++count;
    }
  }
  if (count == 1) {
    return found;
  }
  if (error != nullptr) {
    *error = std::to_string(count) + " callback groups another executor spins hold exactly the "
      "subscriptions /tf and /tf_static; the admitted tf2_ros::TransformListener creates one";
  }
  return -1;
}

void detach_tf2_listener_groups(const std::string & binding_key)
{
  bool wanted = false;
  for (const auto & e : get_rid_entries()) {
    if (e.node_class == binding_key && e.group_index == kGroupTf2Listener) {
      wanted = true;
      break;
    }
  }
  if (!wanted) {
    return;
  }
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  if (g_manager) {
    node_base = g_manager->find_by_class(binding_key);
  }
  if (!node_base) {
    node_base = ros2_lf_get_node(binding_key);
  }
  if (!node_base) {
    fail_bind(
      "tf2 ingress: node '" + binding_key + "' has TransformListener reaction IDs "
      "but was not registered when its construction ended");
  }
  std::vector<rclcpp::CallbackGroup::SharedPtr> candidates;
  std::vector<Tf2GroupView> views;
  node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      if (group == node_base->get_default_callback_group() ||
      lf_is_rclcpp_owned_group(group.get()) ||
      !group->get_associated_with_executor_atomic().load())
      {
        return;
      }
      Tf2GroupView view;
      group->collect_all_ptrs(
        [&](const rclcpp::SubscriptionBase::SharedPtr & s) {
          view.sub_topics.emplace_back(s->get_topic_name());
        },
        [&](const rclcpp::ServiceBase::SharedPtr &) {++view.other_entities;},
        [&](const rclcpp::ClientBase::SharedPtr &) {++view.other_entities;},
        [&](const rclcpp::TimerBase::SharedPtr &) {++view.other_entities;},
        [&](const rclcpp::Waitable::SharedPtr & w) {
          // rclcpp adds a QoS event handler per subscription to its group;
          // it belongs to the subscription, not to another entity.
          if (!std::dynamic_pointer_cast<rclcpp::EventHandlerBase>(w)) {
            ++view.other_entities;
          }
        });
      candidates.push_back(group);
      views.push_back(std::move(view));
    });
  std::string error;
  const int pick = pick_tf2_listener_group(views, &error);
  if (pick < 0) {
    fail_bind("tf2 ingress: node '" + binding_key + "': " + error);
  }
  const auto & group = candidates[static_cast<std::size_t>(pick)];
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  group->collect_all_ptrs(
    [&](const rclcpp::SubscriptionBase::SharedPtr & s) {subs.push_back(s);},
    [](const rclcpp::ServiceBase::SharedPtr &) {},
    [](const rclcpp::ClientBase::SharedPtr &) {},
    [](const rclcpp::TimerBase::SharedPtr &) {},
    [](const rclcpp::Waitable::SharedPtr &) {});
  for (const auto & s : subs) {
    if (!held_without_claiming(s)) {
      fail_bind(
        "tf2 ingress: node '" + binding_key + "': subscription " +
        std::string(s->get_topic_name()) + " of the TransformListener group is in no wait set "
        "before detach; tf2_ros's executor did not collect the group when it was added, which "
        "the detach probe relies on (tf2_ingress.cpp, point 1)");
    }
  }
  int rounds = 0;
  bool quiet = false;
  while (!quiet && rounds < kMaxRounds) {
    ++rounds;
    group->can_be_taken_from().store(false);
    group->trigger_notify_guard_condition();
    std::this_thread::sleep_for(kRoundPeriod);
    // A returning callback sets the flag before the executor's rebuild can
    // remove its subscription, so the flag is read on both sides of the probe.
    if (group->can_be_taken_from().load()) {
      continue;
    }
    quiet = true;
    for (const auto & s : subs) {
      if (in_a_wait_set(s)) {
        quiet = false;
        break;
      }
    }
    quiet = quiet && !group->can_be_taken_from().load();
  }
  if (!quiet) {
    fail_bind(
      "tf2 ingress: node '" + binding_key + "': the TransformListener group did "
      "not leave the tf2_ros executor in " + std::to_string(kMaxRounds) + " rounds (a /tf "
      "callback of tf2_ros still running, or the executor not waking)");
  }
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "tf2 ingress: node '%s': TransformListener group detached after %d round(s); "
    "LF takes /tf and /tf_static", binding_key.c_str(), rounds);
  std::lock_guard<std::mutex> lock(g_tf2_mutex);
  g_tf2_groups.insert(group.get());
  g_tf2_watch.emplace_back(binding_key, group);
}

std::vector<std::string> reassert_tf2_groups_detached()
{
  std::vector<std::string> reattached;
  std::lock_guard<std::mutex> lock(g_tf2_mutex);
  for (const auto & watched : g_tf2_watch) {
    auto group = watched.second.lock();
    if (group && group->can_be_taken_from().exchange(false)) {
      group->trigger_notify_guard_condition();
      reattached.push_back(watched.first);
    }
  }
  return reattached;
}

bool is_detached_tf2_group(const rclcpp::CallbackGroup * group)
{
  {
    std::lock_guard<std::mutex> lock(g_tf2_mutex);
    if (g_tf2_groups.count(group) == 0) {
      return false;
    }
  }
  // The const accessor is not offered; reading the flag does not change it.
  return !const_cast<rclcpp::CallbackGroup *>(group)->can_be_taken_from().load();
}

}  // namespace ros2_lf_binding

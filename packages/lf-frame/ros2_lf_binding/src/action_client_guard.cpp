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

/// @file action_client_guard.cpp
/// @brief Action client responses of an inactive lifecycle node.

#include "action_client_guard.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/srv/cancel_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/action_gate_api.h"
#include "ros2_lf_binding/node_registry.h"

namespace ros2_lf_binding
{
namespace
{

thread_local const std::string * t_inactive_node = nullptr;
thread_local bool t_binding_cancel = false;

/// A cancel request sent while alive is the binding's own.
class BindingCancelScope
{
public:
  BindingCancelScope()
  : previous_(t_binding_cancel) {t_binding_cancel = true;}
  ~BindingCancelScope() {t_binding_cancel = previous_;}

private:
  bool previous_;
};

/// One pending result request. The wrapped callback owns it, so it expires
/// when rclcpp erases the pending entry.
struct ResultWatch
{
  Ros2LfGoalId id{};
  bool known = false;
  std::atomic<bool> stopped{false};
};

/// What the guard knows of one action client.
struct ClientState
{
  /// Deactivations so far. A request sent at an earlier epoch belongs to an
  /// activation that has ended, so its response is treated as inactive.
  uint64_t epoch = 0;
  /// Goals the server accepted in this activation whose callbacks run, with
  /// or without a result request.
  std::vector<Ros2LfGoalId> accepted;
  std::vector<std::weak_ptr<ResultWatch>> watches;
};

struct Registry
{
  std::mutex mu;
  std::vector<Ros2LfActionClientOps> ops;
  std::map<const void *, ClientState> clients;
};

/// Leaked: a response callback may run during static destruction.
Registry & registry()
{
  static auto * r = new Registry();
  return *r;
}

std::vector<Ros2LfActionClientOps> ops_snapshot()
{
  auto & r = registry();
  std::lock_guard<std::mutex> lock(r.mu);
  return r.ops;
}

rclcpp::Waitable & waitable_of(void * client)
{
  return *static_cast<rclcpp_action::ClientBase *>(client);
}

rclcpp::Logger logger()
{
  return rclcpp::get_logger("ros2_lf_binding");
}

bool read_goal_id(void * client, const void * request, bool result, Ros2LfGoalId & id)
{
  for (const auto & ops : ops_snapshot()) {
    if (ops.goal_id && ops.goal_id(waitable_of(client), request, result, id)) {
      return true;
    }
  }
  return false;
}

bool read_accepted(void * client, const void * response, bool & accepted)
{
  for (const auto & ops : ops_snapshot()) {
    if (ops.goal_accepted && ops.goal_accepted(waitable_of(client), response, accepted)) {
      return true;
    }
  }
  return false;
}

uint64_t epoch_of(void * client)
{
  auto & r = registry();
  std::lock_guard<std::mutex> lock(r.mu);
  return r.clients[client].epoch;
}

/// A response to a request sent at `epoch` is kept from user code: the node
/// is inactive at this tag, or it was deactivated after the request was sent.
bool kept_from_user(void * client, uint64_t epoch)
{
  return t_inactive_node != nullptr || epoch_of(client) != epoch;
}

/// Why a response is kept from user code, for the log.
std::string kept_reason()
{
  return t_inactive_node != nullptr ?
         *t_inactive_node + " is inactive" :
         std::string("the request was sent before the node was last deactivated");
}

void watch_result(void * client, const std::shared_ptr<ResultWatch> & watch)
{
  auto & r = registry();
  std::lock_guard<std::mutex> lock(r.mu);
  auto & list = r.clients[client].watches;
  list.erase(
    std::remove_if(
      list.begin(), list.end(),
      [](const std::weak_ptr<ResultWatch> & w) {return w.expired();}),
    list.end());
  list.push_back(watch);
}

/// Record a goal the server accepted in the activation at `epoch`, so the
/// next deactivate stops its callbacks.
void track_accepted(
  void * client, uint64_t epoch, const std::shared_ptr<void> & request,
  const std::shared_ptr<void> & response)
{
  bool accepted = false;
  Ros2LfGoalId id{};
  if (!read_accepted(client, response.get(), accepted) || !accepted ||
    !read_goal_id(client, request.get(), false, id))
  {
    return;
  }
  auto & r = registry();
  std::lock_guard<std::mutex> lock(r.mu);
  auto & state = r.clients[client];
  if (state.epoch == epoch) {
    state.accepted.push_back(id);
  }
}

/// A goal's result response ran its callback: rclcpp forgets the goal.
void forget_accepted(void * client, const Ros2LfGoalId & id)
{
  auto & r = registry();
  std::lock_guard<std::mutex> lock(r.mu);
  auto & list = r.clients[client].accepted;
  list.erase(std::remove(list.begin(), list.end(), id), list.end());
}

/// A goal response kept from user code: rclcpp's callback does not run, so
/// no goal handle, user callback or result request comes of it. A goal the
/// server accepted is cancelled once.
void goal_response_kept(
  void * client, const std::shared_ptr<void> & request, const std::shared_ptr<void> & response,
  ActionSendFn send_cancel)
{
  const std::string why = kept_reason();
  bool accepted = false;
  if (!read_accepted(client, response.get(), accepted)) {
    RCLCPP_ERROR(
      logger(), "Lifecycle gate: %s, a goal response of an action client of an untyped "
      "action arrived and was dropped, its goal is not cancelled", why.c_str());
    return;
  }
  if (!accepted) {
    return;
  }
  Ros2LfGoalId id{};
  if (!read_goal_id(client, request.get(), false, id)) {
    RCLCPP_ERROR(
      logger(), "Lifecycle gate: %s, the goal id of an accepted goal cannot be read, its "
      "goal is not cancelled", why.c_str());
    return;
  }
  auto cancel = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  std::copy(id.begin(), id.end(), cancel->goal_info.goal_id.uuid.begin());
  try {
    BindingCancelScope own;
    send_cancel(client, cancel, [](std::shared_ptr<void>) {});
    RCLCPP_INFO(
      logger(), "Lifecycle gate: %s, cancelled a goal the server accepted after deactivate",
          why.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "Lifecycle gate: %s, the cancel of a goal accepted after deactivate "
      "failed: %s", why.c_str(), e.what());
  }
}

}  // namespace

InactiveClientScope::InactiveClientScope(const std::string & node_class)
: previous_(t_inactive_node)
{
  t_inactive_node = &node_class;
}

InactiveClientScope::~InactiveClientScope()
{
  t_inactive_node = previous_;
}

ActionResponseCallback action_client_guard(
  void * client, int channel, const std::shared_ptr<void> & request,
  ActionResponseCallback callback, ActionSendFn send_cancel)
{
  if (!callback) {
    return callback;
  }
  // rclcpp runs the goal and cancel wrappers in handle_*_response with that
  // channel's pending-request mutex held and erases the entry after they
  // return (a result entry is extracted first), so a wrapper must not throw
  // and must not send on its own channel.
  const uint64_t epoch = epoch_of(client);
  switch (channel) {
    case RCLLF_ACTION_GOAL_REQ:
      return [client, epoch, request, callback = std::move(callback), send_cancel](
        std::shared_ptr<void> response) {
               if (kept_from_user(client, epoch)) {
                 goal_response_kept(client, request, response, send_cancel);
                 return;
               }
               track_accepted(client, epoch, request, response);
               callback(std::move(response));
             };
    case RCLLF_ACTION_RESULT_REQ: {
        auto watch = std::make_shared<ResultWatch>();
        watch->known = read_goal_id(client, request.get(), true, watch->id);
        watch_result(client, watch);
        return [client, epoch, watch, callback = std::move(callback)](
          std::shared_ptr<void> response) {
                 if (watch->stopped.load() || !kept_from_user(client, epoch)) {
                   if (watch->known) {
                     forget_accepted(client, watch->id);
                   }
                   callback(std::move(response));
                 }
               };
      }
    case RCLLF_ACTION_CANCEL_REQ: {
        const bool own = t_binding_cancel;
        return [client, epoch, own, callback = std::move(callback)](
          std::shared_ptr<void> response) {
                 if (own || !kept_from_user(client, epoch)) {
                   callback(std::move(response));
                 }
               };
      }
    default:
      return callback;
  }
}

int64_t action_client_deactivate(rclcpp_action::ClientBase & client)
{
  void * key = static_cast<void *>(&client);
  std::vector<std::shared_ptr<ResultWatch>> live;
  std::vector<Ros2LfGoalId> ids;
  {
    auto & r = registry();
    std::lock_guard<std::mutex> lock(r.mu);
    auto & state = r.clients[key];
    ids = state.accepted;
    for (const auto & w : state.watches) {
      auto watch = w.lock();
      if (watch && watch->known && !watch->stopped.load()) {
        live.push_back(watch);
        if (std::find(ids.begin(), ids.end(), watch->id) == ids.end()) {
          ids.push_back(watch->id);
        }
      }
    }
  }
  // The lifecycle gate runs this hook with its boundary mutex held, before
  // any reaction of the tag takes (lifecycle_gate_boundary), so no response
  // of this client executes between stop_callbacks and `stopped` below.
  BindingCancelScope own;
  for (const auto & ops : ops_snapshot()) {
    if (ops.deactivate && ops.deactivate(client, ids)) {
      for (const auto & watch : live) {
        watch->stopped.store(true);
      }
      auto & r = registry();
      std::lock_guard<std::mutex> lock(r.mu);
      auto & state = r.clients[key];
      ++state.epoch;
      state.accepted.clear();
      return static_cast<int64_t>(ids.size());
    }
  }
  return -1;
}

}  // namespace ros2_lf_binding

void ros2_lf_register_action_client_ops(Ros2LfActionClientOps ops)
{
  auto & r = ros2_lf_binding::registry();
  std::lock_guard<std::mutex> lock(r.mu);
  r.ops.push_back(std::move(ops));
}

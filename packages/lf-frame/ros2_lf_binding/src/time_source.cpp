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

/// @file time_source.cpp
/// @brief Implements lf_time_advance_ros_clock() — called before each reaction
///        to set every LF-driven ROS clock to the current LF logical time.
///
/// Design (ADR 0001 + Amendment 2026-04-25):
///   - LF is the sole time authority for in-process nodes. Their clocks are
///     set directly through the ROS time override, never through /clock.
///   - A shared rclcpp::Clock(RCL_ROS_TIME) is created at runtime_init and
///     stored as g_lf_clock.  ROS time override is enabled on it once.
///   - Each node's own RCL_ROS_TIME clock is attached at construction:
///       * non-component nodes: ros2_lf_register_node() sees the concrete
///         type and hands over node->get_clock(); the override is enabled on
///         it and it is set together with g_lf_clock.
///       * component nodes: the binding only sees them type-erased, so the
///         component manager builds them with lf_time_node_options()
///         (use_sim_time=true, no clock thread). Their TimeSource then owns
///         a /clock subscription that nothing spins; the binding hands it a
///         rosgraph_msgs/Clock directly, which sets every clock attached to
///         that TimeSource. No DDS message is ever taken from it.
///   - Before every reaction fires, ros2_lf_invoke_timer/sub call
///     lf_time_advance_ros_clock(), which sets all of the above to the
///     current LF logical time, then calls clock_publisher_tick() to
///     optionally publish /clock and /lf_clock for external observers
///     (rosbag, rqt, plotjuggler).
///   - A non-component node that sets use_sim_time=true itself is taken off
///     /clock at registration (ros2_lf_detach_sim_time), or refused.
///   - Every clock is set under its own get_clock_mutex(), the mutex the
///     TimeSource holds when it sets the same clock.
///   - g_lf_clock and every attached node clock are registered with
///     clock_guard (register_lf_clock); the clock gate treats only these as
///     attached. A component's clocks are not registered: the binding feeds
///     its TimeSource and never holds the clocks.
///   - Not covered: rclcpp::Clock objects a node creates itself (they are not
///     attached anywhere, even when the node enables their override), and
///     steady and system clocks.

#include "time_source.hpp"

#include <rcl/time.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "clock_guard.hpp"
#include "clock_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// reactor-c public API — must be extern "C" because reactor-c is a C library
// with no C++ guard in its headers.
extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

namespace ros2_lf_binding
{

// Shared clock instance — initialized in ros2_lf_runtime_init.
std::shared_ptr<rclcpp::Clock> g_lf_clock;

namespace
{

// Node clocks and TimeSource subscriptions that follow LF logical time.
// Guarded by g_nodes_mutex: reactions of one tag may run on several workers.
// Held weakly: the subscription callback points into its node's TimeSource,
// so it must not outlive the node.
std::mutex g_nodes_mutex;
std::vector<std::weak_ptr<rclcpp::Clock>> g_node_clocks;
std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> g_time_source_subs;
// Callback groups a TimeSource created for its clock thread.
std::vector<std::weak_ptr<rclcpp::CallbackGroup>> g_rclcpp_groups;
// Logical time last pushed into the attached clocks; -1 before the first.
int64_t g_nodes_time_ns = -1;
// Set at the first tag pushed into the clocks; read lock-free by the clock
// gate on every ROS time read during construction.
std::atomic<bool> g_time_started{false};

// Set a clock's ROS time override under the clock's own mutex, the one
// rclcpp's TimeSource holds when it sets the same clock.
bool set_ros_time_override(const std::shared_ptr<rclcpp::Clock> & clock, int64_t ns)
{
  std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
  if (rcl_set_ros_time_override(clock->get_clock_handle(), ns) != RCL_RET_OK) {
    rcl_reset_error();
    return false;
  }
  return true;
}

// The first subscription of type MsgT on `topic` in any callback group of the
// node. A TimeSource creates its /clock and /parameter_events subscriptions
// in the rclcpp::Node constructor, before any user code, so they come first.
template<typename MsgT>
rclcpp::SubscriptionBase::SharedPtr find_subscription(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & topic)
{
  rclcpp::SubscriptionBase::SharedPtr found;
  node_base->for_each_callback_group(
    [&found, &topic](rclcpp::CallbackGroup::SharedPtr group) {
      if (found) {
        return;
      }
      found = group->find_subscription_ptrs_if(
        [&topic](const rclcpp::SubscriptionBase::SharedPtr & sub) {
          return topic == sub->get_topic_name() &&
                 std::dynamic_pointer_cast<rclcpp::Subscription<MsgT>>(sub);
        });
    });
  return found;
}

// Every subscription of type MsgT in any callback group of the node, with its
// group, held weakly so a subscription its owner drops shows up as expired.
template<typename MsgT>
std::vector<std::pair<std::weak_ptr<rclcpp::SubscriptionBase>, rclcpp::CallbackGroup::SharedPtr>>
typed_subscriptions(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  std::vector<std::pair<std::weak_ptr<rclcpp::SubscriptionBase>,
    rclcpp::CallbackGroup::SharedPtr>> found;
  node_base->for_each_callback_group(
    [&found](rclcpp::CallbackGroup::SharedPtr group) {
      group->find_subscription_ptrs_if(
        [&found, &group](const rclcpp::SubscriptionBase::SharedPtr & sub) {
          if (std::dynamic_pointer_cast<rclcpp::Subscription<MsgT>>(sub)) {
            found.emplace_back(sub, group);
          }
          return false;
        });
    });
  return found;
}

void feed_time_source_locked(const rclcpp::SubscriptionBase::SharedPtr & sub, int64_t ns)
{
  // The TimeSource callback takes the message as shared_ptr<const Clock>; the
  // type-erased handle_message() is the documented executor entry point.
  auto msg = std::make_shared<rosgraph_msgs::msg::Clock>();
  msg->clock.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg->clock.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  std::shared_ptr<void> erased = msg;
  sub->handle_message(erased, rclcpp::MessageInfo());
}

}  // namespace

void lf_clock_init()
{
  g_lf_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  // Enable ROS time override mode: subsequent calls to
  // rcl_clock_set_ros_time_override() will drive this clock's value.
  rcl_ret_t ret = rcl_enable_ros_time_override(g_lf_clock->get_clock_handle());
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "lf_clock_init: rcl_enable_ros_time_override failed: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  register_lf_clock(g_lf_clock->get_clock_handle());
}

void lf_time_advance_ros_clock()
{
  if (!g_lf_clock) {
    return;
  }

  // Obtain the current LF logical time from the first environment.
  // _lf_get_environments() is generated by lfc into the linked .lf binary.
  environment_t * envs = nullptr;
  int n_envs = _lf_get_environments(&envs);
  if (n_envs <= 0 || envs == nullptr) {
    return;
  }
  // Node clocks follow envs[0] only: several environments would need several
  // ROS times.
  check_single_environment(n_envs);

  instant_t logical_ns = envs[0].current_tag.time;
  if (logical_ns <= 0) {
    return;
  }

  if (!set_ros_time_override(g_lf_clock, logical_ns)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(g_nodes_mutex);
    if (logical_ns != g_nodes_time_ns) {
      for (const auto & weak : g_node_clocks) {
        if (auto clock = weak.lock()) {
          set_ros_time_override(clock, logical_ns);
        }
      }
      for (const auto & weak : g_time_source_subs) {
        if (auto sub = weak.lock()) {
          feed_time_source_locked(sub, logical_ns);
        }
      }
      g_nodes_time_ns = logical_ns;
      g_time_started.store(true);
    }
  }
  ros2_lf_binding::clock_publisher_tick();
}

rclcpp::NodeOptions lf_time_node_options(rclcpp::NodeOptions options)
{
  auto overrides = options.parameter_overrides();
  overrides.erase(
    std::remove_if(
      overrides.begin(), overrides.end(),
      [](const rclcpp::Parameter & p) {return p.get_name() == "use_sim_time";}),
    overrides.end());
  overrides.emplace_back("use_sim_time", true);
  options.parameter_overrides(overrides);
  options.use_clock_thread(false);
  return options;
}

bool lf_attach_time_source(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const std::string & key)
{
  if (!node_base) {
    return false;
  }
  auto found = find_subscription<rosgraph_msgs::msg::Clock>(node_base, "/clock");
  if (!found) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "Node '%s' has no TimeSource /clock subscription; its now() stays on physical time "
      "instead of LF logical time", key.c_str());
    return false;
  }
  std::lock_guard<std::mutex> lock(g_nodes_mutex);
  g_time_source_subs.push_back(found);
  if (g_nodes_time_ns > 0) {
    feed_time_source_locked(found, g_nodes_time_ns);
  }
  return true;
}

bool lf_is_time_source_subscription(const rclcpp::SubscriptionBase * sub)
{
  std::lock_guard<std::mutex> lock(g_nodes_mutex);
  for (const auto & weak : g_time_source_subs) {
    if (weak.lock().get() == sub) {
      return true;
    }
  }
  return false;
}

bool lf_is_rclcpp_owned_group(const rclcpp::CallbackGroup * group)
{
  std::lock_guard<std::mutex> lock(g_nodes_mutex);
  for (const auto & weak : g_rclcpp_groups) {
    if (weak.lock().get() == group) {
      return true;
    }
  }
  return false;
}

void lf_mark_rclcpp_owned_group(const rclcpp::CallbackGroup::SharedPtr & group)
{
  std::lock_guard<std::mutex> lock(g_nodes_mutex);
  g_rclcpp_groups.push_back(group);
}

bool lf_time_started()
{
  return g_time_started.load();
}

void lf_clock_reset_for_test()
{
  std::lock_guard<std::mutex> lock(g_nodes_mutex);
  g_node_clocks.clear();
  g_time_source_subs.clear();
  g_rclcpp_groups.clear();
  g_nodes_time_ns = -1;
  g_time_started.store(false);
  g_lf_clock.reset();
}

}  // namespace ros2_lf_binding

void ros2_lf_attach_node_clock(std::shared_ptr<rclcpp::Clock> clock, const std::string & key)
{
  if (!clock || clock->get_clock_type() != RCL_ROS_TIME) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "Node '%s' has no RCL_ROS_TIME clock; its now() does not follow LF logical time",
      key.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
    if (rcl_enable_ros_time_override(clock->get_clock_handle()) != RCL_RET_OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_lf_binding"),
        "Node '%s': rcl_enable_ros_time_override failed: %s", key.c_str(),
        rcl_get_error_string().str);
      rcl_reset_error();
      return;
    }
    ros2_lf_binding::register_lf_clock(clock->get_clock_handle());
  }
  std::lock_guard<std::mutex> lock(ros2_lf_binding::g_nodes_mutex);
  ros2_lf_binding::g_node_clocks.push_back(clock);
  if (ros2_lf_binding::g_nodes_time_ns > 0) {
    ros2_lf_binding::set_ros_time_override(clock, ros2_lf_binding::g_nodes_time_ns);
  }
}

bool ros2_lf_detach_sim_time(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  const std::string & key)
{
  auto logger = rclcpp::get_logger("ros2_lf_binding");
  if (!base || !parameters) {
    return false;
  }
  const auto result =
    parameters->set_parameters_atomically({rclcpp::Parameter("use_sim_time", false)});
  if (!result.successful) {
    RCLCPP_ERROR(
      logger, "Node '%s': setting use_sim_time=false was rejected: %s",
      key.c_str(), result.reason.c_str());
    return false;
  }
  // The TimeSource acts on use_sim_time only through its /parameter_events
  // subscription, which no executor spins under LF. Hand it the event that
  // set_parameters_atomically() just published, synchronously.
  auto events = ros2_lf_binding::find_subscription<rcl_interfaces::msg::ParameterEvent>(
    base, "/parameter_events");
  if (!events) {
    RCLCPP_ERROR(
      logger, "Node '%s' has no TimeSource /parameter_events subscription; it cannot be "
      "taken off /clock", key.c_str());
    return false;
  }
  // The node's own /clock subscriptions, if any, stay. The TimeSource's is
  // the one that goes away below.
  const auto clock_subs_before =
    ros2_lf_binding::typed_subscriptions<rosgraph_msgs::msg::Clock>(base);
  auto event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
  event->node = base->get_fully_qualified_name();
  event->changed_parameters.push_back(rclcpp::Parameter("use_sim_time", false).to_parameter_msg());
  std::shared_ptr<void> erased = event;
  events->handle_message(erased, rclcpp::MessageInfo());
  // The TimeSource has now joined its clock thread and dropped its /clock
  // subscription. Check it, rather than trust it. Its clock thread's callback
  // group stays behind, empty, and is rclcpp's, not the node's.
  std::size_t dropped = 0;
  for (const auto & [weak, group] : clock_subs_before) {
    if (weak.expired()) {
      ++dropped;
      if (group != base->get_default_callback_group()) {
        ros2_lf_binding::lf_mark_rclcpp_owned_group(group);
      }
    }
  }
  if (dropped != 1) {
    RCLCPP_ERROR(
      logger, "Node '%s' still has its TimeSource /clock subscription after "
      "use_sim_time=false; its now() could be set back by a late /clock sample", key.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "Node '%s' set use_sim_time=true; it was taken off /clock and follows LF "
    "logical time", key.c_str());
  return true;
}

extern "C" int64_t ros2_lf_logical_time_ns(void)
{
  environment_t * envs = nullptr;
  int n_envs = _lf_get_environments(&envs);
  if (n_envs <= 0 || envs == nullptr) {
    return -1;
  }
  return envs[0].current_tag.time;
}

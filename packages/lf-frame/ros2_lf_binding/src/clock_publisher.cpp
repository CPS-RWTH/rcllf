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

/// @file clock_publisher.cpp
/// @brief Publishes /clock (rosgraph_msgs/Clock) and /lf_clock (ros2_lf_msgs/LFTag)
///        from the LF runtime. One publisher node, one funnel through tick().
///
/// Design (ADR 0001 amendment):
///   - In-process nodes never read this publisher's /clock. Registered nodes
///     get time via rcl_set_ros_time_override, and component nodes via a
///     TimeSource /clock subscription that nothing spins and that
///     time_source.cpp feeds in-memory.
///   - These topics exist purely for external observers: rosbag, rqt,
///     plotjuggler, out-of-process nodes.
///   - Controlled by ROS2_LF_CLOCK_PUBLISH env var (off|on_change|every_reaction).

#include "clock_publisher.hpp"

#include <climits>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <ros2_lf_msgs/msg/lf_tag.hpp>

extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

namespace ros2_lf_binding
{

namespace
{

std::mutex g_mutex;

ClockPublishMode g_mode = ClockPublishMode::Off;

rclcpp::Node::SharedPtr g_clock_pub_node;
rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr g_clock_pub;
rclcpp::Publisher<ros2_lf_msgs::msg::LFTag>::SharedPtr g_lf_clock_pub;

instant_t g_last_time = -1;
microstep_t g_last_microstep = UINT32_MAX;

}  // namespace

void clock_publisher_init(ClockPublishMode mode)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_mode = mode;
  if (mode == ClockPublishMode::Off) {
    return;
  }

  g_clock_pub_node = rclcpp::Node::make_shared("lf_clock_publisher");

  auto qos = rclcpp::QoS(1).reliable().transient_local();
  g_clock_pub = g_clock_pub_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
  g_lf_clock_pub = g_clock_pub_node->create_publisher<ros2_lf_msgs::msg::LFTag>("/lf_clock", qos);
}

// Called once from ros2_lf_runtime_init before reactions start; no lock needed.
rclcpp::Node::SharedPtr clock_publisher_node()
{
  return g_clock_pub_node;
}

void clock_publisher_tick()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_mode == ClockPublishMode::Off || !g_clock_pub_node) {
    return;
  }

  environment_t * envs = nullptr;
  int n = _lf_get_environments(&envs);
  if (n <= 0 || envs == nullptr) {
    return;
  }

  instant_t t = envs[0].current_tag.time;
  microstep_t m = envs[0].current_tag.microstep;
  if (t <= 0) {
    return;
  }

  bool time_changed = (t != g_last_time);
  bool tag_changed = time_changed || (m != g_last_microstep);

  bool publish_clock = false;
  bool publish_lf_clock = false;

  switch (g_mode) {
    case ClockPublishMode::OnTimeChange:
      publish_clock = time_changed;
      publish_lf_clock = tag_changed;
      break;
    case ClockPublishMode::OnEveryReaction:
      publish_clock = tag_changed;
      publish_lf_clock = tag_changed;
      break;
    default:
      return;
  }

  if (!publish_clock && !publish_lf_clock) {
    return;
  }

  g_last_time = t;
  g_last_microstep = m;

  if (publish_clock) {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec = static_cast<int32_t>(t / 1'000'000'000LL);
    clock_msg.clock.nanosec = static_cast<uint32_t>(t % 1'000'000'000LL);
    g_clock_pub->publish(clock_msg);
  }

  if (publish_lf_clock) {
    ros2_lf_msgs::msg::LFTag tag_msg;
    tag_msg.logical.sec = static_cast<int32_t>(t / 1'000'000'000LL);
    tag_msg.logical.nanosec = static_cast<uint32_t>(t % 1'000'000'000LL);
    tag_msg.microstep = m;
    // Wall time: non-overridden system clock for lateness/jitter analysis.
    auto wall_ns = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
    tag_msg.wall.sec = static_cast<int32_t>(wall_ns / 1'000'000'000LL);
    tag_msg.wall.nanosec = static_cast<uint32_t>(wall_ns % 1'000'000'000LL);
    g_lf_clock_pub->publish(tag_msg);
  }
}

void clock_publisher_shutdown()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_clock_pub.reset();
  g_lf_clock_pub.reset();
  g_clock_pub_node.reset();
  g_last_time = -1;
  g_last_microstep = UINT32_MAX;
  g_mode = ClockPublishMode::Off;
}

}  // namespace ros2_lf_binding

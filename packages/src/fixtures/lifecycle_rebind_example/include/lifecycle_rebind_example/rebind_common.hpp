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

// Shared by the nodes of lifecycle_rebind_example.
// Under rcllf every node of the launch is compiled into one binary, so the
// supervisor reaches the sleeper through g_sleeper and drives its transitions
// from a callback the LF runtime dispatches. In the stock run each node is its
// own process and g_sleeper stays null in the supervisor.

#ifndef LIFECYCLE_REBIND_EXAMPLE__REBIND_COMMON_HPP_
#define LIFECYCLE_REBIND_EXAMPLE__REBIND_COMMON_HPP_

#include <cinttypes>

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int64.hpp"

namespace lifecycle_rebind_example
{

inline rclcpp_lifecycle::LifecycleNode * g_sleeper = nullptr;
// The reader of reader.launch.py, driven by the driver node.
inline rclcpp_lifecycle::LifecycleNode * g_reader = nullptr;

// One parseable line per output: stream name, logical time, value.
template<typename NodeT>
void report(NodeT * node, const char * stream, int64_t value)
{
  RCLCPP_INFO(
    node->get_logger(), "REBIND %s t=%" PRId64 " v=%" PRId64, stream,
    static_cast<int64_t>(node->now().nanoseconds()), value);
}

}  // namespace lifecycle_rebind_example

#endif  // LIFECYCLE_REBIND_EXAMPLE__REBIND_COMMON_HPP_

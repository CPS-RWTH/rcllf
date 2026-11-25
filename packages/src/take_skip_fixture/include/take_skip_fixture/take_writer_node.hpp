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

#ifndef TAKE_SKIP_FIXTURE__TAKE_WRITER_NODE_HPP_
#define TAKE_SKIP_FIXTURE__TAKE_WRITER_NODE_HPP_

#include <cstdint>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace take_skip_fixture
{

/// Publishes id_base + tick on "take" every 100 ms from tick 5 on.
/// Fault hooks, read from the environment and applied only by the node named
/// TAKE_SKIP_INJECT_NODE: the IDs in TAKE_SKIP_LATE_IDS are published
/// TAKE_SKIP_DEFER_MS (default 50) later from another thread, those in
/// TAKE_SKIP_LOST_IDS are never published. Both still count as a write at
/// their tick (ros2_lf_claim_publish), as a sample late or lost in transit does.
/// An ID in TAKE_SKIP_BURST_IDS is published TAKE_SKIP_BURST_N times at its
/// tick (default 15): a burst above the reader's KeepLast depth.
class TakeWriterNode : public rclcpp::Node
{
public:
  explicit TakeWriterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t id_base_;
  int64_t ticks_ = 0;
  int64_t burst_ = 1;
  int64_t burst_n_ = 15;
  int64_t defer_ms_ = 50;
  std::set<int64_t> burst_ids_;
  std::set<int64_t> late_ids_;
  std::set<int64_t> lost_ids_;
};

}  // namespace take_skip_fixture

#endif  // TAKE_SKIP_FIXTURE__TAKE_WRITER_NODE_HPP_

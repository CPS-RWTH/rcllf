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

#ifndef BRAKE_FUSION__CAMERA_NODE_HPP_
#define BRAKE_FUSION__CAMERA_NODE_HPP_

#include <cstdint>
#include <memory>

#include "brake_fusion/grid.hpp"
#include "brake_fusion/jsonl_log.hpp"
#include "brake_fusion/publish_drop.hpp"
#include "brake_fusion/stamper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace brake_fusion
{

// A sensor node: keeps the latest plant state from /sim/state and publishes
// on its own wall timer, stamped by its Stamper (`stamp_mode`,
// `stamp_origin_ns`). A sensor publishes nothing before the first state has
// arrived; its tick count k still advances.

// Camera: 30 ms, the same range after `detection_work_us` of busy work.
// The stamp is taken at the start of the tick (capture time), the message is
// published after the work. `detection_work_mode`: fixed (default) or data,
// where the work scales with the range (2x at 40 m, 0 at the obstacle).
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void state_callback(const nav_msgs::msg::Odometry & msg);
  void timer_callback();
  // The callbacks publish themselves: the analyzer attributes a publish to
  // the timer or subscription callback whose body holds the call, so a
  // helper that published would leave the topic's edge legacy.
  bool serve_pending(sensor_msgs::msg::Range & msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Stamper> stamper_;
  JsonlLog log_;
  double obstacle_distance_;
  int64_t detection_work_us_;
  bool data_dependent_work_;
  bool have_state_{false};
  int64_t state_stamp_ns_{0};
  double position_{0.0};
  int64_t seq_{0};
  // `drop_stamp_ms` (>= 0): the one publication not sent.
  PublishDrop drop_{-1, -1.0};
  bool dropped(const sensor_msgs::msg::Range & msg);
  // Grid sampling (sample_mode grid): the grid tick waiting for its plant state.
  bool grid_{false};
  int64_t period_ns_{0};
  GridSampler sampler_;
  int64_t pending_tick_{-1};
  int64_t pending_stamp_ns_{0};
  int64_t release_ns_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__CAMERA_NODE_HPP_

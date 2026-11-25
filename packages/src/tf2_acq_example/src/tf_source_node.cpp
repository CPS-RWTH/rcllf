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


// /tf source of the tf2 acquisition showcase. One 20 ms timer; with tick j at 20j ms after start:
//   - j = 0..15: world -> sensor_link with x = 0.020 j m, i.e. x = t at
//     1 m/s, stamped now() (the LF tag under rcllf); none after 300 ms (T4)
//   - j = 0 and j = 10: world -> held_link (y = 1 m), the only transforms
//     of that frame (T5: the one at 200 ms comes after the request's deadline)
//   - j = 0: world -> static_link (y = 2 m) on /tf_static, sent from this
//     reaction (T6)

#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TfSourceNode : public rclcpp::Node
{
public:
  TfSourceNode()
  : Node("tf_source")
  {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = create_wall_timer(20ms, std::bind(&TfSourceNode::on_tick, this));
  }

private:
  geometry_msgs::msg::TransformStamped at(
    const rclcpp::Time & stamp, const char * child, double x, double y)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "world";
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.w = 1.0;
    return t;
  }

  void on_tick()
  {
    const rclcpp::Time now = this->now();
    if (tick_ == 0) {
      start_ = now;
      static_broadcaster_->sendTransform(at(now, "static_link", 0.0, 2.0));
    }
    // x = t at 1 m/s, t in seconds since the first tick (plan constant).
    const double x = static_cast<double>((now - start_).nanoseconds()) / 1e9;
    std::vector<geometry_msgs::msg::TransformStamped> out;
    if (tick_ <= 15) {
      out.push_back(at(now, "sensor_link", x, 0.0));
    }
    if (tick_ == 0 || tick_ == 10) {
      out.push_back(at(now, "held_link", x, 1.0));
    }
    if (!out.empty()) {
      broadcaster_->sendTransform(out);
    }
    ++tick_;
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;
  int tick_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfSourceNode>());
  rclcpp::shutdown();
  return 0;
}

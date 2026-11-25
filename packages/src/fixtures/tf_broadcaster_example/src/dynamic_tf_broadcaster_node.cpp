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

#include "tf_broadcaster_example/dynamic_tf_broadcaster_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

namespace tf_broadcaster_example
{

DynamicTfBroadcasterNode::DynamicTfBroadcasterNode()
: Node("dynamic_tf_broadcaster"), yaw_(0.0)
{
  dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // 30 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&DynamicTfBroadcasterNode::timer_callback, this));
  RCLCPP_INFO(
    this->get_logger(), "Dynamic TF broadcaster started at 30 Hz (base_link -> sensor_link)");
}

void DynamicTfBroadcasterNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "sensor_link";
  t.transform.translation.x = 0.3 * std::cos(yaw_);
  t.transform.translation.y = 0.3 * std::sin(yaw_);
  t.transform.translation.z = 0.1;
  // Quaternion for yaw rotation around Z
  t.transform.rotation.w = std::cos(yaw_ / 2.0);
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(yaw_ / 2.0);

  dynamic_broadcaster_->sendTransform(t);

  yaw_ += 0.05;
  if (yaw_ > 2.0 * M_PI) {
    yaw_ -= 2.0 * M_PI;
  }
}

}  // namespace tf_broadcaster_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_broadcaster_example::DynamicTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}

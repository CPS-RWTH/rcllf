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

#include "tf_listener_example/tf_listener_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "tf2/exceptions.hpp"

using namespace std::chrono_literals;

namespace tf_listener_example
{

TfListenerNode::TfListenerNode(const rclcpp::NodeOptions & options)
: Node("tf_listener", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // 10 Hz lookup
  timer_ = this->create_wall_timer(100ms, std::bind(&TfListenerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "TF listener started (world -> sensor_link at 10 Hz)");
}

void TfListenerNode::timer_callback()
{
  try {
    auto t = tf_buffer_->lookupTransform("world", "sensor_link", tf2::TimePointZero);
    RCLCPP_INFO(
      this->get_logger(),
      "world->sensor_link: x=%.3f y=%.3f z=%.3f",
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
}

}  // namespace tf_listener_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_listener_example::TfListenerNode>());
  rclcpp::shutdown();
  return 0;
}

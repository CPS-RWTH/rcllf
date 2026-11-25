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

#include "tf_broadcaster_example/static_tf_broadcaster_node.hpp"

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tf_broadcaster_example
{

StaticTfBroadcasterNode::StaticTfBroadcasterNode()
: Node("static_tf_broadcaster")
{
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "base_link";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.5;
  t.transform.rotation.w = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;

  static_broadcaster_->sendTransform(t);
  RCLCPP_INFO(this->get_logger(), "Published static transform: world -> base_link");
}

}  // namespace tf_broadcaster_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_broadcaster_example::StaticTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}

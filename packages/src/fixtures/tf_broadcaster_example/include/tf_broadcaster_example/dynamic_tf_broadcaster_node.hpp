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

#ifndef TF_BROADCASTER_EXAMPLE__DYNAMIC_TF_BROADCASTER_NODE_HPP_
#define TF_BROADCASTER_EXAMPLE__DYNAMIC_TF_BROADCASTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace tf_broadcaster_example
{

class DynamicTfBroadcasterNode : public rclcpp::Node
{
public:
  DynamicTfBroadcasterNode();

private:
  void timer_callback();

  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double yaw_;
};

}  // namespace tf_broadcaster_example

#endif  // TF_BROADCASTER_EXAMPLE__DYNAMIC_TF_BROADCASTER_NODE_HPP_

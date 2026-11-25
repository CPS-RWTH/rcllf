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

#include "fusion_example/approx_fusion_node.hpp"

namespace fusion_example
{

ApproxFusionNode::ApproxFusionNode(const rclcpp::NodeOptions & options)
: Node("approx_fusion", options)
{
  pairs_pub_ =
    this->create_publisher<custom_interfaces::msg::SensorData>("fusion/approx_pairs", 10);
  sub_a_.subscribe(this, "fusion/a");
  sub_b_.subscribe(this, "fusion/b");
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), sub_a_, sub_b_);
  sync_->registerCallback(
    std::bind(
      &ApproxFusionNode::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void ApproxFusionNode::sync_callback(
  const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
  const custom_interfaces::msg::SensorData::ConstSharedPtr & b)
{
  const int64_t a_ns = rclcpp::Time(a->header.stamp).nanoseconds();
  const int64_t b_ns = rclcpp::Time(b->header.stamp).nanoseconds();
  RCLCPP_INFO(
    this->get_logger(), "PAIR approx a_seq=%.0f a_ns=%ld b_seq=%.0f b_ns=%ld",
    a->temperature, static_cast<long>(a_ns),  // NOLINT(runtime/int)
    b->temperature, static_cast<long>(b_ns));  // NOLINT(runtime/int)
  auto out = custom_interfaces::msg::SensorData();
  out.header.stamp = a->header.stamp;
  out.header.frame_id = "approx_pair";
  out.temperature = a->temperature;
  out.humidity = b->temperature;
  pairs_pub_->publish(out);
}

}  // namespace fusion_example

// main() is compiled out when this file is built into a component library
// (FUSION_COMPONENT_ONLY); the standalone executable keeps it.
#ifndef FUSION_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fusion_example::ApproxFusionNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

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

#include "bna_example/bna_fusion_node.hpp"

namespace bna_example
{

BnaFusionNode::BnaFusionNode(const rclcpp::NodeOptions & options)
: Node("bna_fusion", options)
{
  fused_pub_ = this->create_publisher<custom_interfaces::msg::SensorData>("bna/fused", 10);
  rclcpp::SubscriptionOptions anchor_options = input_options("anchor");
  rclcpp::SubscriptionOptions candidate_options = input_options("candidate");
  sub_anchor_.subscribe(this, "bna/anchor", rmw_qos_profile_default, anchor_options);
  sub_candidate_.subscribe(this, "bna/candidate", rmw_qos_profile_default, candidate_options);
  // delta 20 ms, lateness 0 and 30 ms, capacity 4 and 4, retention 350 ms,
  // timeout 300 ms.
  sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
    Policy(20000000, 0, 30000000, 4, 4, 350000000, 300000000), sub_anchor_, sub_candidate_);
  sync_->registerCallback(
    std::bind(
      &BnaFusionNode::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

rclcpp::SubscriptionOptions BnaFusionNode::input_options(const char * input)
{
  rclcpp::SubscriptionOptions options;
  const std::string name(input);
  options.event_callbacks.deadline_callback =
    [this, name](rclcpp::QOSDeadlineRequestedInfo & info) {
      RCLCPP_INFO(
        this->get_logger(), "MISSING input=%s total=%d change=%d", name.c_str(),
        info.total_count, info.total_count_change);
    };
  options.event_callbacks.message_lost_callback =
    [this, name](rclcpp::QOSMessageLostInfo & info) {
      RCLCPP_INFO(
        this->get_logger(), "LOST input=%s total=%zu change=%zu", name.c_str(),
        info.total_count, info.total_count_change);
    };
  return options;
}

void BnaFusionNode::sync_callback(
  const custom_interfaces::msg::SensorData::ConstSharedPtr & a,
  const custom_interfaces::msg::SensorData::ConstSharedPtr & c)
{
  const int64_t a_ns = rclcpp::Time(a->header.stamp).nanoseconds();
  const int64_t c_ns = rclcpp::Time(c->header.stamp).nanoseconds();
  RCLCPP_INFO(
    this->get_logger(), "PAIR anchor_tick=%.0f candidate_tick=%.0f offset_ns=%ld",
    a->temperature, c->temperature, static_cast<long>(c_ns - a_ns));  // NOLINT(runtime/int)
  auto out = custom_interfaces::msg::SensorData();
  out.header.stamp = a->header.stamp;
  out.header.frame_id = "bna_pair";
  out.temperature = a->temperature;
  out.humidity = c->temperature;
  fused_pub_->publish(out);
}

}  // namespace bna_example

#ifndef BNA_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bna_example::BnaFusionNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

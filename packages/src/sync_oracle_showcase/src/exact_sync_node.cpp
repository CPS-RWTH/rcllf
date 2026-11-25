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


#include "sync_oracle_showcase/exact_sync_node.hpp"

#include <string>

#include "sync_oracle_showcase/sync_log.hpp"

namespace sync_oracle_showcase
{

ExactSyncNode::ExactSyncNode(const rclcpp::NodeOptions & options)
: Node("exact_sync_node", options)
{
  const int64_t queue_size = this->declare_parameter<int64_t>("queue_size", 3);
  sub_a_.subscribe(this, "/sync/in0");
  sub_b_.subscribe(this, "/sync/in1");
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(queue_size)), sub_a_, sub_b_);
  sync_->registerCallback(
    std::bind(
      &ExactSyncNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
  sync_->registerDropCallback(
    std::bind(
      &ExactSyncNode::drop_callback, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    this->get_logger(), "RCLLF_SYNC %s",
    with_node(config_json("exact", static_cast<uint32_t>(queue_size)) + "}", this->get_name())
    .c_str());
}

void ExactSyncNode::sync_callback(
  const custom_interfaces::msg::SyncSample::ConstSharedPtr & a,
  const custom_interfaces::msg::SyncSample::ConstSharedPtr & b)
{
  RCLCPP_INFO(
    this->get_logger(), "RCLLF_SYNC %s",
    with_node(event_json(to_event(true, a, b), seq_++), this->get_name()).c_str());
}

void ExactSyncNode::drop_callback(
  const custom_interfaces::msg::SyncSample::ConstSharedPtr & a,
  const custom_interfaces::msg::SyncSample::ConstSharedPtr & b)
{
  RCLCPP_INFO(
    this->get_logger(), "RCLLF_SYNC %s",
    with_node(event_json(to_event(false, a, b), seq_++), this->get_name()).c_str());
}

}  // namespace sync_oracle_showcase

#ifndef SYNC_ORACLE_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sync_oracle_showcase::ExactSyncNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

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


#ifndef SYNC_ORACLE_SHOWCASE__EXACT_SYNC_NODE_HPP_
#define SYNC_ORACLE_SHOWCASE__EXACT_SYNC_NODE_HPP_

#include <cstdint>
#include <memory>

#include "custom_interfaces/msg/sync_sample.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"

namespace sync_oracle_showcase
{

// Unmodified message_filters ExactTime over /sync/in0 and /sync/in1,
// queue size from the queue_size parameter. Logs one RCLLF_SYNC config
// record at construction and one record per signal and drop callback.
class ExactSyncNode : public rclcpp::Node
{
public:
  explicit ExactSyncNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
    custom_interfaces::msg::SyncSample, custom_interfaces::msg::SyncSample>;

  void sync_callback(
    const custom_interfaces::msg::SyncSample::ConstSharedPtr & a,
    const custom_interfaces::msg::SyncSample::ConstSharedPtr & b);
  void drop_callback(
    const custom_interfaces::msg::SyncSample::ConstSharedPtr & a,
    const custom_interfaces::msg::SyncSample::ConstSharedPtr & b);

  message_filters::Subscriber<custom_interfaces::msg::SyncSample> sub_a_;
  message_filters::Subscriber<custom_interfaces::msg::SyncSample> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  uint64_t seq_ = 0;
};

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__EXACT_SYNC_NODE_HPP_

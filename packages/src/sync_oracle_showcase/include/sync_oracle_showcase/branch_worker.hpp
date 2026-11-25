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


#ifndef SYNC_ORACLE_SHOWCASE__BRANCH_WORKER_HPP_
#define SYNC_ORACLE_SHOWCASE__BRANCH_WORKER_HPP_

#include <cstdint>

#include "custom_interfaces/msg/sync_sample.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sync_oracle_showcase
{

// One fork/join branch (corpus P3): copies each /fork/src sample
// to /fork/out after work_ms of busy work. SYNC_WORK_SWAP=1 in the
// environment selects work_ms_swapped instead, so one launch runs both work
// assignments (0/8 and 8/0 ms).
class BranchWorker : public rclcpp::Node
{
public:
  explicit BranchWorker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_sample(const custom_interfaces::msg::SyncSample::ConstSharedPtr & msg);

  int64_t work_ms_ = 0;
  rclcpp::Subscription<custom_interfaces::msg::SyncSample>::SharedPtr sub_;
  rclcpp::Publisher<custom_interfaces::msg::SyncSample>::SharedPtr pub_;
};

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__BRANCH_WORKER_HPP_

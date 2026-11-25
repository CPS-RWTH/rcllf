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


#include "sync_oracle_showcase/branch_worker.hpp"

#include <chrono>
#include <cstdlib>
#include <string>

namespace sync_oracle_showcase
{

BranchWorker::BranchWorker(const rclcpp::NodeOptions & options)
: Node("branch_worker", options)
{
  work_ms_ = this->declare_parameter<int64_t>("work_ms", 0);
  const int64_t swapped = this->declare_parameter<int64_t>("work_ms_swapped", 0);
  const char * env = std::getenv("SYNC_WORK_SWAP");
  if (env != nullptr && std::string(env) == "1") {
    work_ms_ = swapped;
  }
  pub_ = this->create_publisher<custom_interfaces::msg::SyncSample>("/fork/out", 10);
  sub_ = this->create_subscription<custom_interfaces::msg::SyncSample>(
    "/fork/src", 10, std::bind(&BranchWorker::on_sample, this, std::placeholders::_1));
  RCLCPP_INFO(
    this->get_logger(), "RCLLF_SYNC {\"node\":\"%s\",\"ev\":\"branch\",\"work_ms\":%ld}",
    this->get_name(), static_cast<long>(work_ms_));  // NOLINT(runtime/int)
}

void BranchWorker::on_sample(const custom_interfaces::msg::SyncSample::ConstSharedPtr & msg)
{
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(work_ms_);
  while (std::chrono::steady_clock::now() < until) {
  }
  pub_->publish(*msg);
  // One record per forwarded row: a stock run measures the per-tick work plus
  // dispatch span from these against the 50 ms tick.
  RCLCPP_INFO(
    this->get_logger(), "RCLLF_SYNC {\"node\":\"%s\",\"ev\":\"forward\",\"row\":%lu}",
    this->get_name(), static_cast<unsigned long>(msg->row));  // NOLINT(runtime/int)
}

}  // namespace sync_oracle_showcase

#ifndef SYNC_ORACLE_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sync_oracle_showcase::BranchWorker>());
  rclcpp::shutdown();
  return 0;
}
#endif

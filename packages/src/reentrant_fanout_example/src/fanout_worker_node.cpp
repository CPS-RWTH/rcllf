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

#include "reentrant_fanout_example/fanout_worker_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace reentrant_fanout_example
{

FanoutWorkerNode::FanoutWorkerNode()
: Node("fanout_worker")
{
  reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = reentrant_group_;

  subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "work", 10,
    std::bind(&FanoutWorkerNode::work_callback, this, std::placeholders::_1),
    opts);

  RCLCPP_INFO(this->get_logger(), "FanoutWorker started with Reentrant callback group");
}

void FanoutWorkerNode::work_callback(const std_msgs::msg::Int64::SharedPtr msg)
{
  std::ostringstream tid;
  tid << std::this_thread::get_id();

  auto t_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "[thread %s] seq=%ld START", tid.str().c_str(), msg->data);

  std::this_thread::sleep_for(200ms);

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_INFO(
    this->get_logger(), "[thread %s] seq=%ld END (%ldms)",
    tid.str().c_str(), msg->data, elapsed);
}

}  // namespace reentrant_fanout_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<reentrant_fanout_example::FanoutWorkerNode>();

  rclcpp::ExecutorOptions exec_opts;
  rclcpp::executors::MultiThreadedExecutor executor(exec_opts, 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

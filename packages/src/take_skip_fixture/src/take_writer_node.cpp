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

#include "take_skip_fixture/take_writer_node.hpp"

#include <dlfcn.h>

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{

constexpr int64_t kWarmupTicks = 5;  // discovery settles before the first write

std::set<int64_t> env_ids(const char * name)
{
  std::set<int64_t> ids;
  const char * v = std::getenv(name);
  std::stringstream in(v != nullptr ? v : "");
  std::string item;
  while (std::getline(in, item, ',')) {
    if (!item.empty()) {
      ids.insert(std::strtoll(item.c_str(), nullptr, 10));
    }
  }
  return ids;
}

/// Count one write at this tick for a sample that is not published now
/// (absent outside LF, so the node also runs under plain ROS).
void claim_publish(const char * topic)
{
  using ClaimFn = void (*)(const char *);
  auto claim = reinterpret_cast<ClaimFn>(dlsym(RTLD_DEFAULT, "ros2_lf_claim_publish"));
  if (claim != nullptr) {
    claim(topic);
  }
}

}  // namespace

namespace take_skip_fixture
{

TakeWriterNode::TakeWriterNode(const rclcpp::NodeOptions & options)
: Node("take_writer", options)
{
  id_base_ = this->declare_parameter<int64_t>("id_base", 0);
  const char * inject = std::getenv("TAKE_SKIP_INJECT_NODE");
  if (inject != nullptr && std::string(this->get_name()) == inject) {
    late_ids_ = env_ids("TAKE_SKIP_LATE_IDS");
    lost_ids_ = env_ids("TAKE_SKIP_LOST_IDS");
    const char * defer = std::getenv("TAKE_SKIP_DEFER_MS");
    if (defer != nullptr && defer[0] != '\0') {
      defer_ms_ = std::strtoll(defer, nullptr, 10);
    }
    burst_ids_ = env_ids("TAKE_SKIP_BURST_IDS");
    const char * burst_n = std::getenv("TAKE_SKIP_BURST_N");
    if (burst_n != nullptr && burst_n[0] != '\0') {
      burst_n_ = std::strtoll(burst_n, nullptr, 10);
    }
  }
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("take", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&TakeWriterNode::timer_callback, this));
}

void TakeWriterNode::timer_callback()
{
  const int64_t tick = ticks_++;
  const int64_t now = this->now().nanoseconds();
  if (tick < kWarmupTicks) {
    RCLCPP_INFO(
      this->get_logger(), "TSW tick=%" PRId64 " t=%" PRId64 " kind=idle", tick, now);
    return;
  }
  // A loop, so the analyzer proves no burst bound and the edge stays on the
  // legacy take path (burst_unbounded), the path the skip debt rule is on.
  const int64_t id = id_base_ + tick;
  const int64_t burst = burst_ids_.count(id) > 0 ? burst_n_ : burst_;
  for (int64_t i = 0; i < burst; ++i) {
    std_msgs::msg::Int64 msg;
    msg.data = id;
    const char * kind = "pub";
    if (late_ids_.count(msg.data) > 0) {
      kind = "late";
      claim_publish(publisher_->get_topic_name());
      auto pub = publisher_;
      const int64_t defer = defer_ms_;
      std::thread([pub, msg, defer]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(defer));
          pub->publish(msg);
        }).detach();
    } else if (lost_ids_.count(msg.data) > 0) {
      kind = "lost";
      claim_publish(publisher_->get_topic_name());
    } else {
      publisher_->publish(msg);
    }
    RCLCPP_INFO(
      this->get_logger(), "TSW tick=%" PRId64 " t=%" PRId64 " kind=%s id=%" PRId64, tick, now,
      kind, msg.data);
  }
}

}  // namespace take_skip_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<take_skip_fixture::TakeWriterNode>());
  rclcpp::shutdown();
  return 0;
}

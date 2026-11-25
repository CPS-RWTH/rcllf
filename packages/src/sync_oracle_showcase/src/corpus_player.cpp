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


#include "sync_oracle_showcase/corpus_player.hpp"

#include <chrono>
#include <cstdlib>
#include <string>

using namespace std::chrono_literals;

namespace sync_oracle_showcase
{

CorpusPlayer::CorpusPlayer(const rclcpp::NodeOptions & options)
: Node("corpus_player", options)
{
  std::string path = this->declare_parameter<std::string>("corpus_file", "");
  wait_matched_ = this->declare_parameter<int64_t>("wait_matched", 0);
  const char * env = std::getenv("SYNC_CORPUS");
  if (env != nullptr && env[0] != '\0') {
    path = env;
  }
  // Stock runs gate tick 0 on matched
  // subscribers; the converted system never needs it.
  const char * wait_env = std::getenv("SYNC_WAIT_MATCHED");
  if (wait_env != nullptr && wait_env[0] != '\0') {
    wait_matched_ = std::stoll(wait_env);
  }
  corpus_ = load_corpus(path);
  for (const auto & r : corpus_.rows) {
    has_input_[r.input] = true;
    if (r.tick > last_tick_) {
      last_tick_ = r.tick;
    }
  }
  pub0_ = this->create_publisher<custom_interfaces::msg::SyncSample>("/sync/in0", 10);
  pub1_ = this->create_publisher<custom_interfaces::msg::SyncSample>("/sync/in1", 10);
  RCLCPP_INFO(
    this->get_logger(),
    "RCLLF_SYNC {\"node\":\"%s\",\"ev\":\"corpus\",\"name\":\"%s\",\"corpus\":\"%s\","
    "\"rows\":%zu,\"ticks\":%ld}",
    this->get_name(), corpus_.name.c_str(), corpus_.digest.c_str(), corpus_.rows.size(),
    static_cast<long>(last_tick_ + 1));  // NOLINT(runtime/int)
  // 50 ms: P3's slow branch works 4 rows x 8 ms = 32 ms per tick, the rest
  // is dispatch margin (plan constants). A converted run does not depend on it.
  timer_ = this->create_wall_timer(50ms, std::bind(&CorpusPlayer::tick, this));
}

void CorpusPlayer::tick()
{
  if (wait_matched_ > 0 && tick_ == 0 &&
    ((has_input_[0] && pub0_->get_subscription_count() < static_cast<size_t>(wait_matched_)) ||
    (has_input_[1] && pub1_->get_subscription_count() < static_cast<size_t>(wait_matched_))))
  {
    return;
  }
  for (const auto & r : corpus_.rows) {
    if (r.tick != tick_) {
      continue;
    }
    custom_interfaces::msg::SyncSample m;
    m.header.stamp = rclcpp::Time(r.stamp_ns, RCL_ROS_TIME);
    m.input = r.input;
    m.row = r.row;
    m.payload = r.payload;
    if (r.input == 0) {
      pub0_->publish(m);
    } else {
      pub1_->publish(m);
    }
  }
  ++tick_;
  if (!done_ && tick_ > last_tick_) {
    done_ = true;
    RCLCPP_INFO(
      this->get_logger(), "RCLLF_SYNC {\"node\":\"%s\",\"ev\":\"done\",\"ticks\":%ld}",
      this->get_name(), static_cast<long>(tick_));  // NOLINT(runtime/int)
  }
}

}  // namespace sync_oracle_showcase

#ifndef SYNC_ORACLE_COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sync_oracle_showcase::CorpusPlayer>());
  rclcpp::shutdown();
  return 0;
}
#endif

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


#ifndef SYNC_ORACLE_SHOWCASE__CORPUS_PLAYER_HPP_
#define SYNC_ORACLE_SHOWCASE__CORPUS_PLAYER_HPP_

#include <cstdint>

#include "custom_interfaces/msg/sync_sample.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sync_oracle_showcase/corpus.hpp"

namespace sync_oracle_showcase
{

// Plays a sync corpus: every 50 ms tick k it publishes the rows
// with tick == k, in file order, on /sync/in0 or /sync/in1. The corpus comes
// from the SYNC_CORPUS environment variable, else the corpus_file parameter.
// wait_matched > 0 (stock runs only) holds tick 0 until every topic that has
// rows has that many subscriptions. Logs RCLLF_SYNC corpus and done records.
class CorpusPlayer : public rclcpp::Node
{
public:
  explicit CorpusPlayer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();

  Corpus corpus_;
  int64_t wait_matched_ = 0;
  int64_t tick_ = 0;
  int64_t last_tick_ = -1;
  bool has_input_[2] = {false, false};
  bool done_ = false;
  rclcpp::Publisher<custom_interfaces::msg::SyncSample>::SharedPtr pub0_;
  rclcpp::Publisher<custom_interfaces::msg::SyncSample>::SharedPtr pub1_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__CORPUS_PLAYER_HPP_

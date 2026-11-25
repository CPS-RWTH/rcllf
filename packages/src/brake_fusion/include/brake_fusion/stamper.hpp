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

#ifndef BRAKE_FUSION__STAMPER_HPP_
#define BRAKE_FUSION__STAMPER_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace brake_fusion
{

// Header stamp source of a periodic sensor, selected by the `stamp_mode`
// parameter (the stamp modes of fusion_example):
//   now              this->now() at the start of the timer tick (default)
//   assigned_shared  stamp_origin_ns + k * period, the origin shared by all
//                    sensors (one host)
//   assigned_own     origin + k * period, the origin read from the node's own
//                    clock when the node is constructed
// k counts the timer ticks of the node from 0, including ticks that publish
// nothing because no plant state has arrived yet.
class Stamper
{
public:
  Stamper(
    const std::string & mode, int64_t shared_origin_ns, int64_t period_ns,
    const rclcpp::Time & construction_time)
  : period_ns_(period_ns)
  {
    if (mode == "assigned_shared") {
      assigned_ = true;
      origin_ns_ = shared_origin_ns;
    } else if (mode == "assigned_own") {
      assigned_ = true;
      origin_ns_ = construction_time.nanoseconds();
    } else if (mode == "assigned_clock") {
      // k from the clock since a shared start, not from a callback count.
      assigned_ = true;
      from_clock_ = true;
      origin_ns_ = shared_origin_ns;
    } else if (mode != "now") {
      throw std::invalid_argument(
              "stamp_mode must be now, assigned_shared, assigned_own or assigned_clock");
    }
  }

  // Stamp of the current tick; `now` is the node clock read at tick start.
  // In assigned_clock mode the tick is the nearest k with origin + k * period
  // to `now`.
  rclcpp::Time stamp(const rclcpp::Time & now)
  {
    if (!assigned_) {
      return now;
    }
    if (from_clock_) {
      const int64_t since = now.nanoseconds() - origin_ns_;
      const int64_t half = period_ns_ / 2;
      tick_ = since >= -half ? (since + half) / period_ns_ : -1;
    }
    return rclcpp::Time(origin_ns_ + tick_ * period_ns_, now.get_clock_type());
  }

  // assigned_clock only: true when the current tick lies before the start or
  // repeats the last published tick, so the caller must not publish it.
  bool skip() const {return from_clock_ && (tick_ < 0 || tick_ <= last_published_);}
  void published() {last_published_ = tick_;}

  void advance()
  {
    if (!from_clock_) {
      ++tick_;
    }
  }
  int64_t tick() const {return tick_;}
  int64_t origin_ns() const {return origin_ns_;}
  bool assigned() const {return assigned_;}

private:
  int64_t period_ns_;
  bool assigned_{false};
  bool from_clock_{false};
  int64_t last_published_{-1};
  int64_t origin_ns_{0};
  int64_t tick_{0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__STAMPER_HPP_

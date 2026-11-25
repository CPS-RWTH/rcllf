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

/// @file bna_policy.hpp
/// @brief message_filters policy running the BNA engine.
///
///   using Policy = rcllf_message_filters::sync_policies::BoundedNearestToAnchor<A, B>;
///   message_filters::Synchronizer<Policy> sync(
///     Policy(delta_ns, lateness0_ns, lateness1_ns, capacity0, capacity1,
///       retention_ns, timeout_ns), sub_a, sub_b);
///
/// Input 0 is the anchor. Every engine call and the user callback run under
/// one mutex. Under rcllf the tag and key come from the corr drain and wakes
/// are LF reactions. Under stock ROS the tag is the steady clock, made
/// nondecreasing, the key is an admission counter with epoch 0, and one timer
/// thread applies due deadlines.

#ifndef RCLLF_MESSAGE_FILTERS__BNA_POLICY_HPP_
#define RCLLF_MESSAGE_FILTERS__BNA_POLICY_HPP_

#include <message_filters/message_traits.h>
#include <message_filters/null_types.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include "rcllf_message_filters/bna_engine.hpp"
#include "rcllf_message_filters/bna_runtime_hooks.h"

namespace rcllf_message_filters
{
namespace sync_policies
{

namespace detail
{

inline bool rcllf_active()
{
  return ros2_lf_bna_active != nullptr && ros2_lf_bna_register != nullptr &&
         ros2_lf_bna_active() == 1;
}

inline bool trace_enabled()
{
  const char * v = std::getenv("RCLLF_BNA_TRACE");
  return v == nullptr || std::string(v) != "0";
}

}  // namespace detail

using message_filters::NullType;

template<class M0, class M1>
class BoundedNearestToAnchor
  : public message_filters::PolicyBase<M0, M1, NullType, NullType, NullType, NullType, NullType,
    NullType, NullType>
{
public:
  using Sync = message_filters::Synchronizer<BoundedNearestToAnchor>;
  using Super = message_filters::PolicyBase<M0, M1, NullType, NullType, NullType, NullType,
      NullType, NullType, NullType>;
  using Messages = typename Super::Messages;
  using Signal = typename Super::Signal;
  using Events = typename Super::Events;
  using RealTypeCount = typename Super::RealTypeCount;
  using M0Event = typename Super::M0Event;
  using M1Event = typename Super::M1Event;

  /// Times in nanoseconds. See docs/features/bna-synchronizer.md.
  BoundedNearestToAnchor(
    int64_t delta, int64_t lateness0, int64_t lateness1, int64_t capacity0,
    int64_t capacity1, int64_t retention, int64_t timeout)
  {
    params_.delta = delta;
    params_.lateness[0] = lateness0;
    params_.lateness[1] = lateness1;
    params_.capacity[0] = capacity0;
    params_.capacity[1] = capacity1;
    params_.retention = retention;
    params_.timeout = timeout;
    if (!params_.valid()) {
      throw std::invalid_argument("BoundedNearestToAnchor: parameters out of range");
    }
  }

  /// The Synchronizer keeps a copy. Only the parameters and test settings are
  /// copied, the engine is created in initParent of the kept copy.
  BoundedNearestToAnchor(const BoundedNearestToAnchor & o)
  : params_(o.params_), clock_(o.clock_), hold_timer_(o.hold_timer_), sink_(o.sink_) {}
  BoundedNearestToAnchor & operator=(const BoundedNearestToAnchor &) = delete;

  ~BoundedNearestToAnchor() {shutdown("cancelled");}

  void initParent(Sync * parent)
  {
    parent_ = parent;
    std::lock_guard<std::mutex> lock(mutex_);
    if (engine_) {
      return;
    }
    if (detail::rcllf_active()) {
      rcllf_ = true;
      hooks_ = rcllf_bna_hooks{this, &on_missing_hook, &on_wake_hook, &on_close_hook};
      uint64_t epoch = 0;
      handle_ = ros2_lf_bna_register(&hooks_, &epoch);
      if (handle_ < 0) {
        throw std::runtime_error(
          handle_ == RCLLF_BNA_MULTIPLE ?
          "BoundedNearestToAnchor: one BNA synchronizer per node (bna_multiple)" :
          "BoundedNearestToAnchor: must be constructed in the node constructor under rcllf");
      }
      // An older binding has no setter and keeps every report plain.
      if (ros2_lf_bna_set_missing_stamped != nullptr) {
        ros2_lf_bna_set_missing_stamped(handle_, &on_missing_stamped_hook);
      }
      params_.epoch = epoch;
    }
    make_engine();
    trace(rcllf_message_filters::bna::params_json(params_, 0));
    if (!rcllf_) {
      timer_ = std::thread([this] {timer_loop();});
    }
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type & evt)
  {
    if constexpr (i < 2) {
      admit<i>(evt);
    }
  }

private:
  template<int i>
  void admit(const typename std::tuple_element<i, Events>::type & evt)
  {
    namespace mt = message_filters::message_traits;
    using M = typename std::tuple_element<i, Messages>::type;
    std::unique_lock<std::mutex> lock(mutex_);
    if (!engine_ || !evt.getMessage()) {
      return;
    }
    const int64_t stamp = mt::TimeStamp<M>::value(*evt.getMessage()).nanoseconds();
    bna::Key key;
    bna::Tag tag;
    if (rcllf_) {
      rcllf_bna_dispatch d{};
      const int rc = ros2_lf_bna_current(handle_, i, &d);
      if (rc != RCLLF_BNA_OK) {
        std::fprintf(
          stderr, "RCLLF_BNA refused: input %d admitted outside its declared corr drain (%d)\n",
          i, rc);
        std::fflush(stderr);
        std::_Exit(3);  // plan risk 2: a defect, never a silent nondeterministic path
      }
      key = bna::Key{d.epoch, d.gid, d.ordinal};
      tag = bna::Tag{d.time, d.micro};
    } else {
      key = bna::Key{0, i == 0 ? "a" : "b", ++admitted_[i]};
      tag = stock_tag();
    }
    Payload p;
    std::get<i>(p) = evt;
    trace(rcllf_message_filters::bna::admit_json(0, i, stamp, key, tag));
    engine_->admit(i, stamp, key, tag, std::move(p));
    cv_.notify_all();
  }

public:
  // -- test settings (stock mode), set before the Synchronizer is built ---

  /// Replace the steady clock (nanoseconds).
  void setClockForTest(std::function<int64_t()> clock) {clock_ = std::move(clock);}
  /// Keep the timer thread from applying deadlines until releaseTimerForTest.
  void holdTimerForTest() {hold_timer_ = true;}
  void releaseTimerForTest()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_timer_ = false;
    cv_.notify_all();
  }
  /// Receive every trace line instead of stderr.
  void setTraceSinkForTest(std::function<void(const std::string &)> sink) {sink_ = std::move(sink);}
  /// Close the engine now (stock mode), as the node going away would.
  void closeForTest(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    close_locked(reason, stock_tag());
  }

private:
  using Payload = std::tuple<M0Event, M1Event>;
  using Engine = rcllf_message_filters::bna::Engine<Payload>;

  void make_engine()
  {
    engine_ = std::make_unique<Engine>(params_);
    engine_->on_record(
      [this](const bna::Record & r) {trace(rcllf_message_filters::bna::to_json(r));});
    engine_->on_match(
      [this](Payload & a, Payload & c) {
        if (in_wake_) {
          // Watermarks move only on admit and missing, so a deadline step never
          // decides a pair. A match here would run user code in the wake
          // reaction, which holds no input RID.
          std::fprintf(stderr, "RCLLF_BNA defect: a wake produced a match\n");
          std::fflush(stderr);
          std::_Exit(3);
        }
        if (parent_ != nullptr) {
          parent_->signal(
            std::get<0>(a), std::get<1>(c), typename Super::M2Event(),
            typename Super::M3Event(), typename Super::M4Event(), typename Super::M5Event(),
            typename Super::M6Event(), typename Super::M7Event(), typename Super::M8Event());
        }
      });
    engine_->on_deadline(
      [this](const bna::Tag & d) {
        if (rcllf_) {
          ros2_lf_bna_request_wake(handle_, d.time);
        }
      });
  }

  void trace(const std::string & json)
  {
    if (sink_) {
      sink_(json);
    } else if (rcllf_ && ros2_lf_bna_trace != nullptr) {
      ros2_lf_bna_trace(json.c_str());
    } else if (detail::trace_enabled()) {
      std::fprintf(stderr, "RCLLF_BNA %s\n", json.c_str());
    }
  }

  bna::Tag stock_tag()
  {
    const int64_t now = clock_ ? clock_() :
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    last_stock_ = std::max(last_stock_, now);
    return bna::Tag{last_stock_, 0};
  }

  void close_locked(const std::string & reason, const bna::Tag & t)
  {
    if (!engine_ || engine_->closed()) {
      return;
    }
    trace(rcllf_message_filters::bna::close_json(0, reason, t));
    engine_->close(reason, t);
    cv_.notify_all();
  }

  /// Stock mode: apply each due deadline at its own tag. The engine's
  /// catch-up does the same before any later admission, so a late timer
  /// never lets a match happen after its deadline (fix 2).
  void timer_loop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stopping_) {
      auto d = engine_->closed() ? std::nullopt : engine_->next_deadline();
      if (hold_timer_ || !d) {
        cv_.wait(lock);
        continue;
      }
      const bna::Tag now = stock_tag();
      if (d->time <= now.time) {
        trace(rcllf_message_filters::bna::wake_json(0, now));
        engine_->wake(now);
        continue;
      }
      // An injected clock is polled, the steady clock is waited on.
      const auto wait = std::chrono::nanoseconds(std::min<int64_t>(d->time - now.time,
          clock_ ? 1000000 : INT64_MAX / 2));
      cv_.wait_for(lock, wait);
    }
  }

  void shutdown(const char * reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (engine_ && !engine_->closed()) {
        close_locked(reason, rcllf_ ? engine_->last_tag() : stock_tag());
      }
      stopping_ = true;
      cv_.notify_all();
    }
    if (timer_.joinable()) {
      timer_.join();
    }
    if (rcllf_ && handle_ >= 0 && ros2_lf_bna_unregister != nullptr) {
      ros2_lf_bna_unregister(handle_);
      handle_ = -1;
    }
  }

  // -- binding hooks (rcllf) -------------------------------------------
  static void on_missing_hook(
    void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal, int64_t time,
    uint32_t micro)
  {
    auto * self = static_cast<BoundedNearestToAnchor *>(ctx);
    std::lock_guard<std::mutex> lock(self->mutex_);
    if (!self->engine_) {
      return;
    }
    const bna::Key k{epoch, gid, ordinal};
    const bna::Tag t{time, micro};
    self->trace(rcllf_message_filters::bna::missing_json(0, input, k, t));
    self->engine_->missing(input, k, t);
  }

  static void on_missing_stamped_hook(
    void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal, int64_t stamp,
    int64_t time, uint32_t micro)
  {
    auto * self = static_cast<BoundedNearestToAnchor *>(ctx);
    std::lock_guard<std::mutex> lock(self->mutex_);
    if (!self->engine_) {
      return;
    }
    const bna::Key k{epoch, gid, ordinal};
    const bna::Tag t{time, micro};
    self->trace(rcllf_message_filters::bna::missing_json(0, input, k, t, stamp));
    self->engine_->missing(input, k, t, stamp);
  }

  static void on_wake_hook(void * ctx, int64_t time, uint32_t micro)
  {
    auto * self = static_cast<BoundedNearestToAnchor *>(ctx);
    std::lock_guard<std::mutex> lock(self->mutex_);
    if (!self->engine_) {
      return;
    }
    const bna::Tag t{time, micro};
    self->trace(rcllf_message_filters::bna::wake_json(0, t));
    self->in_wake_ = true;
    self->engine_->wake(t);
    self->in_wake_ = false;
  }

  static void on_close_hook(void * ctx, const char * reason, int64_t time, uint32_t micro)
  {
    auto * self = static_cast<BoundedNearestToAnchor *>(ctx);
    std::lock_guard<std::mutex> lock(self->mutex_);
    self->close_locked(reason, bna::Tag{time, micro});
  }

  bna::Params params_;
  std::function<int64_t()> clock_;
  bool hold_timer_ = false;
  std::function<void(const std::string &)> sink_;

  Sync * parent_ = nullptr;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::unique_ptr<Engine> engine_;
  std::thread timer_;
  bool stopping_ = false;
  bool rcllf_ = false;
  bool in_wake_ = false;
  int handle_ = -1;
  rcllf_bna_hooks hooks_{};
  uint64_t admitted_[2] = {0, 0};
  int64_t last_stock_ = INT64_MIN;
};

}  // namespace sync_policies
}  // namespace rcllf_message_filters

#endif  // RCLLF_MESSAGE_FILTERS__BNA_POLICY_HPP_

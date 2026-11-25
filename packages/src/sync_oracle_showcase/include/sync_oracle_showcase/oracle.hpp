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


#ifndef SYNC_ORACLE_SHOWCASE__ORACLE_HPP_
#define SYNC_ORACLE_SHOWCASE__ORACLE_HPP_

// Oracle O1 core: feeds a corpus in lf_arrival_order into the
// pinned message_filters policies (impl "header") or into a small
// reimplementation of ExactTime 4.11.17 (impl "ref") that carries the
// single-flag mutants. No ROS graph, no DDS.

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "message_filters/pass_through.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "sync_oracle_showcase/corpus.hpp"
#include "sync_oracle_showcase/sync_log.hpp"

namespace sync_oracle_showcase
{

inline Sample::ConstSharedPtr to_msg(const Row & r)
{
  auto m = std::make_shared<Sample>();
  m->header.stamp = rclcpp::Time(r.stamp_ns, RCL_ROS_TIME);
  m->input = r.input;
  m->row = r.row;
  m->payload = r.payload;
  return m;
}

// Signal9 accepts member-function binds (as the converted nodes register
// them), not lambdas, so the recorder is a small class.
struct Recorder
{
  std::vector<Event> events;
  void on_signal(const Sample::ConstSharedPtr & a, const Sample::ConstSharedPtr & b)
  {
    events.push_back(to_event(true, a, b));
  }
  void on_drop(const Sample::ConstSharedPtr & a, const Sample::ConstSharedPtr & b)
  {
    events.push_back(to_event(false, a, b));
  }
};

template<class Policy, bool kHasDrop>
std::vector<Event> run_policy(const std::vector<Row> & order, uint32_t queue)
{
  Recorder rec;
  message_filters::PassThrough<Sample> in0;
  message_filters::PassThrough<Sample> in1;
  message_filters::Synchronizer<Policy> sync(Policy(queue), in0, in1);
  sync.registerCallback(
    std::bind(&Recorder::on_signal, &rec, std::placeholders::_1, std::placeholders::_2));
  if constexpr (kHasDrop) {
    sync.registerDropCallback(
      std::bind(&Recorder::on_drop, &rec, std::placeholders::_1, std::placeholders::_2));
  }
  for (const auto & r : order) {
    if (r.input == 0) {
      in0.add(to_msg(r));
    } else {
      in1.add(to_msg(r));
    }
  }
  return rec.events;
}

// The library itself, the authoritative side. ApproximateTime has no drop
// signal, so its stream carries signals only.
inline std::vector<Event> run_header(
  const std::vector<Row> & order, const std::string & policy, uint32_t queue)
{
  if (policy == "exact") {
    using P = message_filters::sync_policies::ExactTime<Sample, Sample>;
    return run_policy<P, true>(order, queue);
  }
  if (policy == "approx") {
    using P = message_filters::sync_policies::ApproximateTime<Sample, Sample>;
    return run_policy<P, false>(order, queue);
  }
  throw std::runtime_error("unknown policy " + policy);
}

// ExactTime 4.11.17 (sync_policies/exact_time.h add, checkTuple,
// clearOldTuples) restated, so that one rule at a time can be broken:
//   first_wins            a repeated stamp keeps the first sample (library: last wins, add)
//   evict_oldest_arrival  overflow drops the entry that arrived first
//                         (library: the smallest stamp, tuples_.begin())
//   no_clear_old          a signal does not drop older stamps (library: clearOldTuples)
//   q_plus1               overflow only above queue + 1 (library: size > queue_size_)
inline std::vector<Event> run_ref(
  const std::vector<Row> & order, uint32_t queue, const std::string & mutant)
{
  if (!mutant.empty() && mutant != "first_wins" && mutant != "evict_oldest_arrival" &&
    mutant != "no_clear_old" && mutant != "q_plus1")
  {
    throw std::runtime_error("unknown mutant " + mutant);
  }
  struct Entry
  {
    Slot slots[2];
    size_t first_arrival = 0;
  };
  std::map<int64_t, Entry> tuples;
  int64_t last_signal = 0;  // rclcpp::Time() of the library: 0 ns
  std::vector<Event> out;
  const uint32_t bound = mutant == "q_plus1" ? queue + 1 : queue;
  auto emit = [&out](bool signal, const Entry & e) {
      Event ev;
      ev.signal = signal;
      ev.slots[0] = e.slots[0];
      ev.slots[1] = e.slots[1];
      out.push_back(ev);
    };
  for (size_t n = 0; n < order.size(); ++n) {
    const Row & r = order[n];
    auto it = tuples.find(r.stamp_ns);
    if (it == tuples.end()) {
      it = tuples.emplace(r.stamp_ns, Entry{}).first;
      it->second.first_arrival = n;
    }
    Slot & slot = it->second.slots[r.input];
    if (!(mutant == "first_wins" && slot.present)) {
      slot = Slot{true, r.input, r.row, r.payload, r.stamp_ns};
    }
    if (it->second.slots[0].present && it->second.slots[1].present) {
      emit(true, it->second);
      last_signal = r.stamp_ns;
      tuples.erase(it);
      if (mutant != "no_clear_old") {
        while (!tuples.empty() && tuples.begin()->first <= last_signal) {
          emit(false, tuples.begin()->second);
          tuples.erase(tuples.begin());
        }
      }
    }
    if (queue > 0) {
      while (tuples.size() > bound) {
        auto victim = tuples.begin();
        if (mutant == "evict_oldest_arrival") {
          for (auto v = tuples.begin(); v != tuples.end(); ++v) {
            if (v->second.first_arrival < victim->second.first_arrival) {
              victim = v;
            }
          }
        }
        emit(false, victim->second);
        tuples.erase(victim);
      }
    }
  }
  return out;
}

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__ORACLE_HPP_

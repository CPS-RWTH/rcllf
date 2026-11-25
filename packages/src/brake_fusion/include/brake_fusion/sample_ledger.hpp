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

#ifndef BRAKE_FUSION__SAMPLE_LEDGER_HPP_
#define BRAKE_FUSION__SAMPLE_LEDGER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace brake_fusion
{

// Fate of every sample a two-input message_filters ApproximateTime policy
// receives. The policy has no drop callback and keeps its queues
// private, so the ledger watches the messages instead: it holds a weak
// reference to every sample that is neither paired nor dropped yet, and a
// held sample whose last owner released it was discarded by the policy.
// settle() collects those. The node calls it at the start of the next
// arrival callback and at shutdown, not in a second Subscriber callback:
// rcllf admits one observer per message_filters Subscriber (RCLLF-H003), and
// by the next arrival the previous add and its callback have returned. No
// other owner holds a sample past its own callback in this package (the
// fusion node is the only subscriber of its inputs), so each sample ends
// in exactly one of: paired, dropped (with a reason) or pending (held when the
// node stops). Reasons:
//   superseded      a newer sample on the same input became the candidate
//                   member (makeCandidate clears the older ones), the normal
//                   fate of two of three tracks per camera period
//   queue_overflow  the oldest sample on the adding input, when the policy
//                   held more than queue_size samples of that input at the
//                   push (approximate_time.h add<i>, the has_dropped_messages_
//                   path). arrive() flags that sample at the push, so the
//                   reason does not depend on when its release is seen.
// Queue size at least 2. The flag is set before the add, so it is right only
// if no sample of the adding input is released inside that add's process()
// while the policy's count still exceeds queue_size. An exact oracle (the
// policy's pop_front hooked, a ledger-oracle script, 300000 random
// sequences over queue sizes 1 to 10) found 10783 samples named
// queue_overflow that were superseded instead, all at queue size 1, and none
// at 2 to 10 (0 of 484017 evictions mislabelled, 0 evictions missed). Every
// launch file uses 9 or 10, so the constructor refuses 0 and 1 rather than
// report a wrong reason.
// Duplicate keys: a sample whose stamp equals that of a sample the policy
// still holds, or of the last paired one, on the same input. ApproximateTime
// keeps both (it orders by stamp only), so the ledger reports the duplicate
// and leaves the decision to the policy.
class SampleLedger
{
public:
  struct Fate
  {
    int input;
    int64_t n;
    int64_t stamp_ns;
    const char * reason;
  };

  explicit SampleLedger(std::size_t queue_size)
  : queue_size_(queue_size)
  {
    if (queue_size_ < 2) {
      throw std::invalid_argument(
              "SampleLedger needs queue_size >= 2: at 1 it names superseded samples "
              "queue_overflow (sample_ledger.hpp)");
    }
  }

  // Before Synchronizer::add. Returns the ordinal of an earlier sample with
  // the same stamp on this input, or -1.
  int64_t arrive(int input, const std::shared_ptr<const void> & msg, int64_t n, int64_t stamp_ns)
  {
    auto & q = held_[input];
    int64_t dup = (last_paired_stamp_[input] == stamp_ns) ? last_paired_n_[input] : -1;
    std::size_t live = 0;
    Entry * oldest_live = nullptr;
    for (auto & e : q) {
      if (!e.msg.expired()) {
        ++live;
        if (oldest_live == nullptr) {
          oldest_live = &e;
        }
      }
      if (e.stamp_ns == stamp_ns) {
        dup = e.n;
      }
    }
    if (live + 1 > queue_size_ && oldest_live != nullptr) {
      oldest_live->overflow = true;
    }
    q.push_back(Entry{msg.get(), msg, n, stamp_ns, false});
    peak_[input] = std::max<int64_t>(peak_[input], static_cast<int64_t>(live + 1));
    ++arrivals_[input];
    if (dup >= 0) {
      ++duplicates_[input];
    }
    return dup;
  }

  // In the Synchronizer callback. Returns the ordinals of the two members.
  std::array<int64_t, 2> paired(const void * radar, const void * camera)
  {
    std::array<int64_t, 2> n{-1, -1};
    const std::array<const void *, 2> members{radar, camera};
    for (int i = 0; i < 2; ++i) {
      auto & q = held_[i];
      for (auto it = q.begin(); it != q.end(); ++it) {
        if (it->ptr == members[i]) {
          n[i] = it->n;
          last_paired_stamp_[i] = it->stamp_ns;
          last_paired_n_[i] = it->n;
          ++paired_[i];
          q.erase(it);
          break;
        }
      }
    }
    return n;
  }

  // Every held sample the policy released since the last call.
  std::vector<Fate> settle()
  {
    std::vector<Fate> out;
    for (int i = 0; i < 2; ++i) {
      auto & q = held_[i];
      for (auto it = q.begin(); it != q.end(); ) {
        if (!it->msg.expired()) {
          ++it;
          continue;
        }
        const char * reason = it->overflow ? "queue_overflow" : "superseded";
        (it->overflow ? overflowed_ : superseded_)[i] += 1;
        out.push_back(Fate{i, it->n, it->stamp_ns, reason});
        it = q.erase(it);
      }
      held_max_[i] = std::max<int64_t>(held_max_[i], static_cast<int64_t>(q.size()));
    }
    return out;
  }

  // Samples the policy still holds (at shutdown).
  std::vector<Fate> pending() const
  {
    std::vector<Fate> out;
    for (int i = 0; i < 2; ++i) {
      for (const auto & e : held_[i]) {
        out.push_back(Fate{i, e.n, e.stamp_ns, "pending"});
      }
    }
    return out;
  }

  int64_t arrivals(int i) const {return arrivals_[i];}
  int64_t paired_count(int i) const {return paired_[i];}
  int64_t superseded(int i) const {return superseded_[i];}
  int64_t overflowed(int i) const {return overflowed_[i];}
  int64_t duplicates(int i) const {return duplicates_[i];}
  int64_t peak(int i) const {return peak_[i];}
  int64_t held_max(int i) const {return held_max_[i];}
  int64_t held(int i) const {return static_cast<int64_t>(held_[i].size());}

private:
  struct Entry
  {
    const void * ptr;
    std::weak_ptr<const void> msg;
    int64_t n;
    int64_t stamp_ns;
    bool overflow;  // the policy evicts it at the push that set this
  };

  std::size_t queue_size_;
  std::array<std::deque<Entry>, 2> held_;
  std::array<int64_t, 2> last_paired_stamp_{INT64_MIN, INT64_MIN};
  std::array<int64_t, 2> last_paired_n_{-1, -1};
  std::array<int64_t, 2> arrivals_{0, 0};
  std::array<int64_t, 2> paired_{0, 0};
  std::array<int64_t, 2> superseded_{0, 0};
  std::array<int64_t, 2> overflowed_{0, 0};
  std::array<int64_t, 2> duplicates_{0, 0};
  std::array<int64_t, 2> peak_{0, 0};
  std::array<int64_t, 2> held_max_{0, 0};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__SAMPLE_LEDGER_HPP_

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

/// @file silence_deadline.cpp
/// @brief Logical deadline of a silent periodic edge.

#include "silence_deadline.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <utility>

namespace ros2_lf_binding
{
namespace silence_deadline
{

namespace
{

constexpr int64_t kNever = std::numeric_limits<int64_t>::max();

/// `a + b` for b > 0, kNever when it would overflow.
int64_t add_or_never(int64_t a, int64_t b)
{
  return a > kNever - b ? kNever : a + b;
}

struct Registry
{
  std::mutex mu;
  std::map<uint32_t, std::unique_ptr<Slot>> slots;
};

Registry & registry()
{
  static auto * r = new Registry();  // leaked: drains may run during static destruction
  return *r;
}

}  // namespace

void Window::arm(int64_t period_ns, int64_t ref_ns)
{
  if (period_ns <= 0) {
    period_ns_ = 0;
    next_due_ns_ = -1;
    return;
  }
  period_ns_ = period_ns;
  ref_ns_ = ref_ns;
  next_due_ns_ = add_or_never(ref_ns, period_ns);
}

void Window::on_record(int64_t now_ns)
{
  if (!armed()) {
    return;
  }
  ref_ns_ = now_ns;
  next_due_ns_ = add_or_never(now_ns, period_ns_);
}

uint32_t Window::take_elapsed(int64_t now_ns)
{
  if (!armed()) {
    return 0;
  }
  uint32_t n = 0;
  while (next_due_ns_ != kNever && next_due_ns_ <= now_ns &&
    n < std::numeric_limits<uint32_t>::max())
  {
    ++n;
    ref_ns_ = next_due_ns_;
    next_due_ns_ = add_or_never(next_due_ns_, period_ns_);
  }
  return n;
}

int64_t combine_wake(int64_t record_wake, int64_t silence_wake)
{
  if (record_wake < 0) {
    return silence_wake;
  }
  if (silence_wake < 0) {
    return record_wake;
  }
  return std::min(record_wake, silence_wake);
}

int64_t silence_wake(const Window & w, int64_t now_ns, int64_t stop_ns)
{
  const int64_t due = w.next_due_ns();
  if (due < 0 || due == kNever) {
    return -1;
  }
  // The stop tag (T, m) admits (T, 0); a window that ends after T never runs.
  if (stop_ns >= 0 && due > stop_ns) {
    return -1;
  }
  return due <= now_ns ? 0 : due - now_ns;
}

std::pair<int64_t, uint32_t> wake_tag(std::pair<int64_t, uint32_t> now, int64_t w)
{
  return w == 0 ? std::make_pair(now.first, now.second + 1) : std::make_pair(now.first + w, 0u);
}

int64_t claim_wake(Slot & s, std::pair<int64_t, uint32_t> now, int64_t w)
{
  if (w < 0) {
    return w;
  }
  return s.queued.insert(wake_tag(now, w)).second ? w : -1;
}

void drained(Slot & s, std::pair<int64_t, uint32_t> now)
{
  s.queued.erase(s.queued.begin(), s.queued.upper_bound(now));
}

int64_t deadline_ns(const rmw_time_t & d)
{
  constexpr uint64_t kNsPerSec = 1000000000ULL;
  if (d.sec == 0 && d.nsec == 0) {
    return -1;  // RMW_DURATION_UNSPECIFIED: the subscription declares no deadline
  }
  // RMW_DURATION_INFINITE (9223372036 s, 854775807 ns) lands here too.
  if (d.sec > static_cast<uint64_t>(kNoDeadlineNs) / kNsPerSec) {
    return -1;
  }
  const uint64_t ns = d.sec * kNsPerSec + d.nsec;
  return ns >= static_cast<uint64_t>(kNoDeadlineNs) ? -1 : static_cast<int64_t>(ns);
}

Slot & slot(uint32_t edge)
{
  auto & r = registry();
  std::lock_guard<std::mutex> lk(r.mu);
  auto & p = r.slots[edge];
  if (!p) {
    p = std::make_unique<Slot>();
  }
  return *p;
}

void reset_for_test()
{
  auto & r = registry();
  std::lock_guard<std::mutex> lk(r.mu);
  r.slots.clear();
}

}  // namespace silence_deadline
}  // namespace ros2_lf_binding

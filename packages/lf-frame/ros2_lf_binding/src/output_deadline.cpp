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

/// @file output_deadline.cpp
/// @brief Per-publisher output deadline check, see
/// output_deadline.hpp.

#include "output_deadline.hpp"

#include <atomic>
#include <cstdarg>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/error_handling.h"
#include "rcutils/logging.h"
#include "rcutils/time.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "console_log.hpp"
#include "plant_record.hpp"
#include "semantic_trace.hpp"
#include "thread_role.hpp"

extern "C" int64_t lf_time_physical(void);

namespace ros2_lf_binding
{

/// Console only (see console_log.hpp). The diagnostics must
/// never reach a ROS topic, or a reaction
/// subscribed to /rosout would run extra times when the bound is on. The
/// RCLCPP_* macros go through rcl's output handler, which also publishes on
/// /rosout whenever a node registered the logger name, and a user node named
/// ros2_lf_binding does exactly that. So these lines call rcutils' console
/// handler directly: same line format and severity filter as the macros, but
/// no rosout publisher and no log file, whatever the nodes are called. Checked
/// by test_rosout_never_carries_the_deadline_diagnostics (actuation_late_example),
/// whose system launches a node named ros2_lf_binding.
#if defined(__GNUC__)
__attribute__((format(printf, 4, 5)))
#endif
void console_log(const char * function, int severity, const char * name, const char * fmt, ...)
{
  if (!rcutils_logging_logger_is_enabled_for(name, severity)) {
    return;
  }
  rcutils_time_point_value_t now = 0;
  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
    rcutils_reset_error();
  }
  const rcutils_log_location_t location = {function, __FILE__, __LINE__};
  static std::mutex console_mu;
  std::lock_guard<std::mutex> lock(console_mu);
  va_list args;
  va_start(args, fmt);
  rcutils_logging_console_output_handler(&location, severity, name, now, fmt, &args);
  va_end(args);
}

namespace output_deadline
{
namespace
{

/// WARN lines per (reaction, topic) before the rest go to the trace and the
/// summary only. Basis: a bound every tag misses (a 1 kHz controller with too
/// tight a bound) would otherwise write a log line per period to the console; the
/// first ten show the pattern, the actuation_late trace event and JSON line
/// (RCLLF_TRACE_CALLBACKS=1) still carry every late publish, and the shutdown
/// summary counts them all.
constexpr uint64_t kWarnLimit = 10;

struct Pending
{
  int rid;
  std::string topic;
  int64_t tag_time;
  uint32_t tag_microstep;
  int64_t physical_ns;
  int64_t lateness_ns;
  int64_t bound_ns;
  bool warn;
  bool last_warn;
};

std::mutex g_mu;
std::map<std::pair<int, std::string>, Stats> g_stats;
std::atomic<bool> g_any{false};
thread_local std::vector<Pending> t_pending;

constexpr char kLogger[] = "ros2_lf_binding";
constexpr char kTraceLogger[] = "ros2_lf_binding.trace";

}  // namespace

void arm(int rid, const std::string & resolved_topic, int64_t bound_ns)
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_stats[{rid, resolved_topic}].bound_ns = bound_ns;
  g_any.store(true, std::memory_order_release);
}

bool armed(int rid, const std::string & resolved_topic)
{
  if (rid < 0 || !g_any.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lk(g_mu);
  return g_stats.count({rid, resolved_topic}) > 0;
}

bool check(int rid, const std::string & resolved_topic)
{
  if (rid < 0 || !g_any.load(std::memory_order_acquire)) {
    return false;
  }
  const auto tag = lf_current_tag();
  if (tag.first < 0) {
    return false;  // not inside an LF program
  }
  int64_t physical = 0;
  int64_t lateness = 0;
  int64_t bound = 0;
  bool warn = false;
  bool last_warn = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_stats.find({rid, resolved_topic});
    if (it == g_stats.end()) {
      return false;
    }
    // Physical time after the lock: a wait on g_mu counts as
    // lateness. What follows (a counter update, one UST event, one vector
    // append) is the only work left before the real publish.
    physical = lf_time_physical();
    lateness = physical - tag.first;
    Stats & s = it->second;
    ++s.checked;
    if (lateness > s.max_lateness_ns) {
      s.max_lateness_ns = lateness;
    }
    bound = s.bound_ns;
    if (lateness <= bound) {
      return false;
    }
    ++s.late;
    warn = s.late <= kWarnLimit;
    last_warn = s.late == kWarnLimit;
  }
  // A UST tracepoint never publishes, so the event is safe inside the gate.
  rcllf_tp_actuation_late e{};
  e.rid = rid;
  e.topic = resolved_topic.c_str();
  e.tag_time = tag.first;
  e.tag_microstep = tag.second;
  e.physical_ns = physical;
  e.lateness_ns = lateness;
  e.bound_ns = bound;
  semtrace::actuation_late(e);
  // Queued for the recorder's thread, no I/O here.
  plant_record::actuation_late(
    rid, resolved_topic, tag.first, tag.second, physical, lateness, bound);
  t_pending.push_back(
    Pending{rid, resolved_topic, tag.first, tag.second, physical, lateness, bound, warn,
      last_warn});
  return true;
}

void flush_pending()
{
  if (t_pending.empty()) {
    return;
  }
  std::vector<Pending> pending;
  pending.swap(t_pending);  // re-entry safe should a log handler ever publish
  const int64_t start = lf_start_tag().first;
  for (const Pending & p : pending) {
    const int64_t elapsed = start >= 0 ? p.tag_time - start : p.tag_time;
    if (p.warn) {
      console_log(
        __func__, RCUTILS_LOG_SEVERITY_WARN, kLogger,
            "rcllf actuation_late: RID %d topic '%s' tag (%lld, %u) published %lld ns "
        "after its tag, above output_deadline_ns=%lld; published anyway%s", p.rid,
        p.topic.c_str(), static_cast<long long>(elapsed),  // NOLINT(runtime/int)
        p.tag_microstep, static_cast<long long>(p.lateness_ns),  // NOLINT(runtime/int)
        static_cast<long long>(p.bound_ns),  // NOLINT(runtime/int)
        p.last_warn ? "; further late publishes of this topic only go to the trace and the "
        "shutdown summary" : "");
    }
    if (trace_callbacks_enabled()) {
      console_log(
        __func__, RCUTILS_LOG_SEVERITY_INFO, kTraceLogger,
        "{\"event\":\"actuation_late\",\"rid\":%d,\"topic\":\"%s\",\"tag_ns\":%lld,"
        "\"microstep\":%u,\"tag_time\":%lld,\"physical_ns\":%lld,\"lateness_ns\":%lld,"
        "\"bound_ns\":%lld}", p.rid, p.topic.c_str(),
        static_cast<long long>(elapsed),  // NOLINT(runtime/int)
        p.tag_microstep, static_cast<long long>(p.tag_time),  // NOLINT(runtime/int)
        static_cast<long long>(p.physical_ns),  // NOLINT(runtime/int)
        static_cast<long long>(p.lateness_ns),  // NOLINT(runtime/int)
        static_cast<long long>(p.bound_ns));  // NOLINT(runtime/int)
    }
  }
}

void warn_unresolved_topic(int rid, const char * topic)
{
  console_log(
    __func__, RCUTILS_LOG_SEVERITY_WARN, kLogger,
    "ros2_lf_output_deadline_arm: RID %d could not resolve topic '%s' against its node; "
    "checking it as written", rid, topic);
}

void log_summary()
{
  flush_pending();
  const auto all = stats();
  for (const auto & kv : all) {
    const Stats & s = kv.second;
    const long long max_l = s.checked > 0 ? s.max_lateness_ns : 0;  // NOLINT(runtime/int)
    console_log(
      __func__, RCUTILS_LOG_SEVERITY_INFO, kLogger,
          "rcllf output deadline summary: RID %d topic '%s' output_deadline_ns=%lld "
      "checked=%llu late=%llu max_lateness_ns=%lld", kv.first.first,
      kv.first.second.c_str(), static_cast<long long>(s.bound_ns),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(s.checked),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(s.late),  // NOLINT(runtime/int)
      max_l);
    if (trace_callbacks_enabled()) {
      console_log(
        __func__, RCUTILS_LOG_SEVERITY_INFO, kTraceLogger,
        "{\"event\":\"output_deadline_summary\",\"rid\":%d,\"topic\":\"%s\",\"bound_ns\":%lld,"
        "\"checked\":%llu,\"late\":%llu,\"max_lateness_ns\":%lld}", kv.first.first,
        kv.first.second.c_str(), static_cast<long long>(s.bound_ns),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(s.checked),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(s.late),  // NOLINT(runtime/int)
        max_l);
    }
  }
}

std::map<std::pair<int, std::string>, Stats> stats()
{
  std::lock_guard<std::mutex> lk(g_mu);
  return g_stats;
}

void reset_for_test()
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_stats.clear();
  g_any.store(false);
  t_pending.clear();
}

}  // namespace output_deadline
}  // namespace ros2_lf_binding

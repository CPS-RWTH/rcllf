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

/// @file plant_record.cpp
/// @brief Recording with dispositions, see plant_record.hpp.

#include "plant_record.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <optional>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl/types.h"
#include "rcutils/allocator.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
namespace plant_record
{
namespace
{

/// One queued line. `bytes` (when `has_bytes`) is base64-encoded by the
/// writer and appended as "cdr", so the caller pays only for the copy.
struct Line
{
  std::string file;
  std::string text;  ///< without the closing brace when has_bytes
  bool has_bytes = false;
  std::vector<uint8_t> bytes;
  /// An `external` entry or a `correction` names a record
  /// id. The writer holds a correction back until its record's entry is
  /// written, so a correction never precedes the entry it corrects, however
  /// two ingress threads interleave after their critical sections.
  enum class Kind {OTHER, EXTERNAL, CORRECTION};
  Kind kind = Kind::OTHER;
  int rid = 0;
  int64_t id = 0;
};

std::mutex g_cfg_mu;
std::atomic<bool> g_cfg_read{false};  // written under g_cfg_mu
/// The directory; guarded by g_cfg_mu until g_cfg_read, read-only after.
std::string & g_dir_ref()
{
  static std::string d;
  return d;
}
std::atomic<bool> g_on{false};

std::mutex g_q_mu;
std::condition_variable g_q_cv;
std::deque<Line> g_queue;     // guarded by g_q_mu
bool g_stop = false;          // guarded by g_q_mu
bool g_writer_started = false;  // guarded by g_q_mu
std::thread g_writer;

std::mutex g_state_mu;
std::set<std::string> g_open_topics;  // guarded by g_state_mu
bool g_run_header = false;            // guarded by g_state_mu
std::map<const void *, const void *> g_publishers;  // guarded by g_state_mu
std::atomic<bool> g_stop_recorded{false};
/// The lf_stop line captured by the shutdown reaction; close() enqueues it
/// last, so no publication of a later shutdown reaction can follow it.
std::optional<std::string> g_stop_line;  // guarded by g_state_mu

// Cost on the recording threads (summary).
std::atomic<uint64_t> g_lines{0};
std::atomic<int64_t> g_enqueue_max_ns{0};
std::mutex g_cost_mu;
std::vector<int64_t> g_serialize_ns;  // guarded by g_cost_mu

// Writer-side results (written by the writer, read after the join).
uint64_t g_written = 0;
uint64_t g_write_errors = 0;
uint64_t g_orphan_corrections = 0;
size_t g_files = 0;

void load_config_locked()
{
  if (g_cfg_read.load()) {
    return;
  }
  const char * v = std::getenv("RCLLF_PLANT_RECORD");
  g_dir_ref() = v != nullptr ? v : "";
  g_on.store(!g_dir_ref().empty(), std::memory_order_release);
  g_cfg_read.store(true);
}

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void writer_main(std::string dir)
{
  std::map<std::string, FILE *> files;
  bool dir_ok = true;
  {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      std::fprintf(
        stderr, "[ros2_lf_binding] rcllf record: cannot create '%s': %s\n", dir.c_str(),
        ec.message().c_str());
      dir_ok = false;
    }
  }
  std::deque<Line> batch;
  // (file, rid, id) of written external entries, and corrections held back.
  std::set<std::tuple<std::string, int, int64_t>> seen;
  std::map<std::tuple<std::string, int, int64_t>, std::vector<Line>> held;
  for (;; ) {
    bool stop = false;
    {
      std::unique_lock<std::mutex> lk(g_q_mu);
      g_q_cv.wait(lk, [] {return g_stop || !g_queue.empty();});
      batch.swap(g_queue);
      stop = g_stop;
    }
    std::set<FILE *> touched;
    std::deque<Line> todo;
    for (Line & l : batch) {
      const auto key = std::make_tuple(l.file, l.rid, l.id);
      if (l.kind == Line::Kind::CORRECTION && seen.count(key) == 0) {
        held[key].push_back(std::move(l));
        continue;
      }
      const bool external = l.kind == Line::Kind::EXTERNAL;
      todo.push_back(std::move(l));
      if (external) {
        seen.insert(key);
        auto h = held.find(key);
        if (h != held.end()) {
          for (Line & c : h->second) {
            todo.push_back(std::move(c));
          }
          held.erase(h);
        }
      }
    }
    if (stop) {
      std::lock_guard<std::mutex> lk(g_q_mu);
      if (g_queue.empty()) {
        // The last batch: a correction whose entry never came is written
        // anyway (and counted), not dropped.
        for (auto & kv : held) {
          for (Line & c : kv.second) {
            ++g_orphan_corrections;
            todo.push_back(std::move(c));
          }
        }
        held.clear();
      }
    }
    for (Line & l : todo) {
      if (!dir_ok) {
        ++g_write_errors;
        continue;
      }
      FILE *& f = files[l.file];
      if (f == nullptr) {
        const std::string path = dir + "/" + l.file;
        f = std::fopen(path.c_str(), "w");
        if (f == nullptr) {
          std::fprintf(
            stderr, "[ros2_lf_binding] rcllf record: cannot open '%s': %s\n",
            path.c_str(), std::strerror(errno));
          files.erase(l.file);
          ++g_write_errors;
          continue;
        }
      }
      if (l.has_bytes) {
        l.text += ",\"cdr\":\"";
        l.text += base64(l.bytes.data(), l.bytes.size());
        l.text += "\"}";
      }
      l.text += '\n';
      if (std::fwrite(l.text.data(), 1, l.text.size(), f) != l.text.size()) {
        ++g_write_errors;
      } else {
        ++g_written;
      }
      touched.insert(f);
    }
    batch.clear();
    for (FILE * f : touched) {
      if (std::fflush(f) != 0) {
        ++g_write_errors;  // a deferred write error (disk full) surfaces here
      }
    }
    if (stop) {
      std::lock_guard<std::mutex> lk(g_q_mu);
      if (g_queue.empty()) {
        break;
      }
    }
  }
  g_files = files.size();
  for (auto & kv : files) {
    if (std::fclose(kv.second) != 0) {
      ++g_write_errors;
    }
  }
}

/// Append `l` to the queue; starts the writer on first use (a header line,
/// at attach or at the first publication, never a record). No I/O here.
void enqueue(Line l)
{
  const int64_t t0 = steady_ns();
  bool spawned = false;
  {
    std::lock_guard<std::mutex> lk(g_q_mu);
    if (g_stop) {
      return;  // closed: a line after close() has no file left to go to
    }
    if (!g_writer_started) {
      g_writer_started = true;
      spawned = true;
      g_writer = std::thread(writer_main, dir());
    }
    g_queue.push_back(std::move(l));
  }
  g_q_cv.notify_one();
  g_lines.fetch_add(1, std::memory_order_relaxed);
  if (spawned) {
    return;  // the thread start is not a per-line cost; the summary excludes it
  }
  const int64_t cost = steady_ns() - t0;
  int64_t prev = g_enqueue_max_ns.load(std::memory_order_relaxed);
  while (cost > prev && !g_enqueue_max_ns.compare_exchange_weak(prev, cost)) {
  }
}

std::string json_str(const std::string & s)
{
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
      out += buf;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

std::string num(int64_t v) {return std::to_string(v);}
std::string opt(const std::optional<int64_t> & v) {return v ? std::to_string(*v) : "null";}

int64_t start_ns() {return lf_start_tag().first;}

/// run.jsonl's header, once, before its first other line.
void ensure_run_header()
{
  // Enqueued under g_state_mu (lock order g_state_mu, then g_q_mu): a second
  // thread that sees the flag set cannot queue its line before the header.
  std::lock_guard<std::mutex> lk(g_state_mu);
  if (g_run_header) {
    return;
  }
  g_run_header = true;
  enqueue(
    Line{"run.jsonl", "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
      num(kFormatVersion) + ",\"kind\":\"run\",\"lf_time_start\":" + num(start_ns()) + "}",
      false, {}});
}

/// reactor-c's `duration` (the timeout; -1 without one). A common name, so it
/// only counts from the object that defines _lf_get_environments (as `fast`).
int64_t lf_duration_ns()
{
  void * sym = dlsym(RTLD_DEFAULT, "duration");
  void * envs = dlsym(RTLD_DEFAULT, "_lf_get_environments");
  if (sym == nullptr || envs == nullptr) {
    return -1;
  }
  Dl_info a{};
  Dl_info b{};
  if (dladdr(sym, &a) == 0 || dladdr(envs, &b) == 0 || a.dli_fname == nullptr ||
    b.dli_fname == nullptr || std::strcmp(a.dli_fname, b.dli_fname) != 0)
  {
    return -1;
  }
  return *static_cast<const int64_t *>(sym);
}

std::string hex64(uint64_t v)
{
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));  // NOLINT
  return buf;
}

}  // namespace

bool enabled()
{
  if (!g_cfg_read.load()) {
    std::lock_guard<std::mutex> lk(g_cfg_mu);
    load_config_locked();
  }
  return g_on.load(std::memory_order_acquire);
}

std::string dir()
{
  std::lock_guard<std::mutex> lk(g_cfg_mu);
  load_config_locked();
  return g_dir_ref();
}

std::string topic_file(const std::string & resolved_topic)
{
  std::string t = resolved_topic;
  if (!t.empty() && t[0] == '/') {
    t.erase(0, 1);
  }
  std::replace(t.begin(), t.end(), '/', '.');
  return "ext." + t + ".jsonl";
}

void open_topic(
  const std::string & resolved_topic, const std::string & type_name, const std::string & rmw,
  int64_t start, int64_t delay_ns, int64_t reserve_ns, int64_t grid_ns)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lk(g_state_mu);  // as ensure_run_header
  if (!g_open_topics.insert(resolved_topic).second) {
    return;
  }
  enqueue(
    Line{topic_file(resolved_topic),
      "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
      num(kFormatVersion) + ",\"kind\":\"external\",\"topic\":" + json_str(resolved_topic) +
      ",\"type\":" + json_str(type_name) + ",\"serialization\":\"rmw_serialize\",\"rmw\":" +
      json_str(rmw) + ",\"lf_time_start\":" + num(start) + ",\"delay_ns\":" + num(delay_ns) +
      ",\"ingress_reserve_ns\":" + num(reserve_ns) + ",\"grid_ns\":" + num(grid_ns) + "}",
      false, {}});
}

void external(const std::string & resolved_topic, External e)
{
  if (!enabled()) {
    return;
  }
  if (e.serialize_ns >= 0) {
    std::lock_guard<std::mutex> lk(g_cost_mu);
    g_serialize_ns.push_back(e.serialize_ns);
  }
  enqueue(
    Line{topic_file(resolved_topic),
      "{\"ev\":\"external\",\"rid\":" + std::to_string(e.rid) + ",\"id\":" + num(e.id) +
      ",\"stamp_ns\":" + num(e.stamp_ns) + ",\"arrival_ns\":" + num(e.arrival_ns) +
      ",\"insert_ns\":" + opt(e.insert_ns) + ",\"tag_ns\":" + opt(e.tag_ns) +
      ",\"disposition\":" + json_str(e.disposition) + ",\"serialize_ns\":" +
      num(e.serialize_ns) +
      (e.replay_of ? ",\"replay_of\":" + num(*e.replay_of) : std::string()) +
      ",\"cdr_len\":" + std::to_string(e.bytes.size()),
      true, std::move(e.bytes), Line::Kind::EXTERNAL, e.rid, e.id});
}

void correction(
  const std::string & resolved_topic, int rid, int64_t id, const char * disposition)
{
  if (!enabled()) {
    return;
  }
  enqueue(
    Line{topic_file(resolved_topic),
      "{\"ev\":\"correction\",\"rid\":" + std::to_string(rid) + ",\"id\":" + num(id) +
      ",\"disposition\":" + json_str(disposition) + "}", false, {}, Line::Kind::CORRECTION,
      rid, id});
}

void note_publisher(const void * publisher, const void * type_support)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lk(g_state_mu);
  g_publishers[publisher] = type_support;
}

void forget_publisher(const void * publisher)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lk(g_state_mu);
  g_publishers.erase(publisher);
}

void publication(
  const void * publisher, const void * payload, bool serialized, int rid,
  const std::string & resolved_topic)
{
  if (!enabled() || payload == nullptr) {
    return;
  }
  const auto tag = lf_current_tag();
  if (tag.first < 0) {
    return;  // not inside an LF program
  }
  uint64_t hash = 0;
  size_t len = 0;
  bool ok = false;
  if (serialized) {
    const auto * m = static_cast<const rcl_serialized_message_t *>(payload);
    hash = fnv1a64(m->buffer, m->buffer_length);
    len = m->buffer_length;
    ok = true;
  } else {
    const void * ts = nullptr;
    {
      std::lock_guard<std::mutex> lk(g_state_mu);
      auto it = g_publishers.find(publisher);
      ts = it != g_publishers.end() ? it->second : nullptr;
    }
    if (ts != nullptr) {
      rmw_serialized_message_t buf = rmw_get_zero_initialized_serialized_message();
      rcutils_allocator_t alloc = rcutils_get_default_allocator();
      const auto * type = static_cast<const rosidl_message_type_support_t *>(ts);
      if (rmw_serialized_message_init(&buf, 0, &alloc) == RMW_RET_OK) {
        if (serialize_zero_padded(payload, type, &buf)) {
          hash = fnv1a64(buf.buffer, buf.buffer_length);
          len = buf.buffer_length;
          ok = true;
        }
        (void)rmw_serialized_message_fini(&buf);
      }
      rcutils_reset_error();
    }
  }
  ensure_run_header();
  const int64_t start = start_ns();
  enqueue(
    Line{"run.jsonl",
      "{\"ev\":\"publish\",\"rid\":" + std::to_string(rid) + ",\"topic\":" +
      json_str(resolved_topic) + ",\"tag_ns\":" + num(tag.first - start) + ",\"microstep\":" +
      std::to_string(tag.second) + ",\"cdr_len\":" + std::to_string(len) + ",\"fnv1a64\":" +
      (ok ? "\"" + hex64(hash) + "\"" : std::string("null")) + "}", false, {}});
}

void actuation_late(
  int rid, const std::string & resolved_topic, int64_t tag_time, uint32_t microstep,
  int64_t physical_ns, int64_t lateness_ns, int64_t bound_ns)
{
  if (!enabled()) {
    return;
  }
  ensure_run_header();
  enqueue(
    Line{"run.jsonl",
      "{\"ev\":\"actuation_late\",\"rid\":" + std::to_string(rid) + ",\"topic\":" +
      json_str(resolved_topic) + ",\"tag_ns\":" + num(tag_time - start_ns()) +
      ",\"microstep\":" + std::to_string(microstep) + ",\"physical_ns\":" + num(physical_ns) +
      ",\"lateness_ns\":" + num(lateness_ns) + ",\"bound_ns\":" + num(bound_ns) + "}",
      false, {}});
}

void lf_stop(const char * source)
{
  if (!enabled() || g_stop_recorded.exchange(true)) {
    return;
  }
  // Tag and condition come from reactor-c's stop tag, not
  // from the current tag, so the teardown fallback (which need not run at
  // the stop tag) classifies the same stop as the shutdown reaction would.
  const auto tag = lf_stop_tag();
  const int64_t start = start_ns();
  const int64_t duration = lf_duration_ns();
  ensure_run_header();
  std::string line =
    "{\"ev\":\"lf_stop\",\"tag_ns\":" + num(tag.first - start) + ",\"microstep\":" +
    std::to_string(tag.second) + ",\"condition\":" +
    json_str(stop_condition(start, duration, tag.first, tag.second)) +
    ",\"stop_tag_ns\":" + num(tag.first - start) + ",\"stop_microstep\":" +
    std::to_string(tag.second) + ",\"duration_ns\":" + num(duration) + ",\"source\":" +
    json_str(source) + "}";
  // Held until close(), so a publication from a later shutdown reaction
  // cannot follow the run's last line.
  std::lock_guard<std::mutex> lk(g_state_mu);
  g_stop_line = std::move(line);
}

void close()
{
  if (!enabled()) {
    return;
  }
  lf_stop("teardown");  // no-op when the shutdown reaction recorded it
  {
    std::optional<std::string> stop_line;
    {
      std::lock_guard<std::mutex> lk(g_state_mu);
      stop_line.swap(g_stop_line);
    }
    if (stop_line) {
      enqueue(Line{"run.jsonl", std::move(*stop_line), false, {}});
    }
  }
  {
    std::lock_guard<std::mutex> lk(g_q_mu);
    g_stop = true;
  }
  g_q_cv.notify_all();
  if (g_writer.joinable()) {
    g_writer.join();
  }
  std::vector<int64_t> ser;
  {
    std::lock_guard<std::mutex> lk(g_cost_mu);
    ser = g_serialize_ns;
  }
  std::sort(ser.begin(), ser.end());
  const long long p50 = ser.empty() ? -1 : ser[ser.size() / 2];  // NOLINT(runtime/int)
  const long long mx = ser.empty() ? -1 : ser.back();  // NOLINT(runtime/int)
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "rcllf record: %llu lines in %zu files under '%s' (%llu write errors); ingress serialize "
    "p50=%lld ns max=%lld ns over %zu records; enqueue max=%lld ns excluding the writer start; "
    "%llu corrections without their entry",
    static_cast<unsigned long long>(g_written), g_files,  // NOLINT(runtime/int)
    dir().c_str(), static_cast<unsigned long long>(g_write_errors),  // NOLINT(runtime/int)
    p50, mx, ser.size(),
    static_cast<long long>(g_enqueue_max_ns.load()),  // NOLINT(runtime/int)
    static_cast<unsigned long long>(g_orphan_corrections));  // NOLINT(runtime/int)
}

const char * stop_condition(
  int64_t start, int64_t duration_ns, int64_t stop_time, uint32_t stop_microstep)
{
  if (duration_ns >= 0 && stop_microstep == 0 && stop_time == start + duration_ns) {
    return "timeout";
  }
  return stop_microstep > 0 ? "request_stop" : "other";
}

bool serialize_zero_padded(
  const void * msg, const rosidl_message_type_support_t * type, rmw_serialized_message_t * buf)
{
  // The CDR writer skips alignment padding without writing it, so the
  // padding of a buffer holds whatever the heap left there and one message
  // could record two hashes. Serialize once to size the buffer, zero it, and
  // serialize again into it (no resize, so the zeros stay in the padding).
  if (rmw_serialize(msg, type, buf) != RMW_RET_OK) {
    return false;
  }
  std::memset(buf->buffer, 0, buf->buffer_capacity);
  return rmw_serialize(msg, type, buf) == RMW_RET_OK;
}

uint64_t fnv1a64(const uint8_t * data, size_t n)
{
  uint64_t h = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < n; ++i) {
    h ^= data[i];
    h *= 0x100000001b3ULL;
  }
  return h;
}

std::string base64(const uint8_t * data, size_t n)
{
  static const char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((n + 2) / 3) * 4);
  size_t i = 0;
  for (; i + 2 < n; i += 3) {
    const uint32_t v = (uint32_t{data[i]} << 16) | (uint32_t{data[i + 1]} << 8) | data[i + 2];
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += kAlphabet[(v >> 6) & 63];
    out += kAlphabet[v & 63];
  }
  if (i + 1 == n) {
    const uint32_t v = uint32_t{data[i]} << 16;
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += "==";
  } else if (i + 2 == n) {
    const uint32_t v = (uint32_t{data[i]} << 16) | (uint32_t{data[i + 1]} << 8);
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    out += kAlphabet[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

void set_dir_for_test(const std::optional<std::string> & d)
{
  {
    std::lock_guard<std::mutex> lk(g_q_mu);
    g_stop = false;
    g_writer_started = false;
    g_queue.clear();
  }
  {
    std::lock_guard<std::mutex> lk(g_state_mu);
    g_open_topics.clear();
    g_run_header = false;
    g_publishers.clear();
  }
  g_stop_recorded.store(false);
  {
    std::lock_guard<std::mutex> lk(g_state_mu);
    g_stop_line.reset();
  }
  g_orphan_corrections = 0;
  std::lock_guard<std::mutex> lk(g_cfg_mu);
  if (d) {
    g_dir_ref() = *d;
    g_on.store(!g_dir_ref().empty(), std::memory_order_release);
    g_cfg_read.store(true);
  } else {
    g_cfg_read.store(false);
    g_dir_ref().clear();
    load_config_locked();
  }
}

}  // namespace plant_record
}  // namespace ros2_lf_binding

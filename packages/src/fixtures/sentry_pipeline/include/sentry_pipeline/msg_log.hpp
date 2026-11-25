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

#ifndef SENTRY_PIPELINE__MSG_LOG_HPP_
#define SENTRY_PIPELINE__MSG_LOG_HPP_

// Opt-in per-publisher message log (rcllf.msglog/1).
//
// Off unless RCLLF_MSGLOG_DIR is set: then every publisher wrapped in a
// PublishLog writes JSON lines to <dir>/<node name><topic, / -> _>.jsonl:
//   {"event":"open",  topic, type, node, publisher gid, pid}       at creation
//   {"event":"pub",   seq (1, 2, ...), t_ns, published, cdr (hex)} per publish
//   {"event":"close", logged, published (the node's own counters)} at destruction
// The files are a source-side record of what each node published, an alternative to a
// rosbag for capturing message values.
//
// Use: ``if (log_.record(msg)) { pub_->publish(msg); }`` keeps the publish call
// visible to lf-analyze (a wrapper that publishes hides it and drops the edge).
//
// Test hook: RCLLF_TEST_WITHHOLD="<topic>:<seq>" records publication <seq> of
// <topic> with published=false and does not publish it, so its subscribers
// never receive it (a consumer of the log must classify it lost).

#include <unistd.h>

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sentry_pipeline
{

class PublishLog
{
public:
  PublishLog() = default;
  PublishLog(const PublishLog &) = delete;
  PublishLog & operator=(const PublishLog &) = delete;

  ~PublishLog()
  {
    if (file_ == nullptr) {
      return;
    }
    std::fprintf(
      file_, "{\"event\":\"close\",\"topic\":\"%s\",\"logged\":%" PRIu64 ",\"published\":%" PRIu64
      ",\"t_ns\":%" PRId64 "}\n",
      topic_.c_str(), seq_, sent_, now_ns());
    std::fclose(file_);
  }

  /// Open the log of *pub* (no-op unless RCLLF_MSGLOG_DIR is set). The
  /// destructor writes the close record, so the log lives as long as the node.
  template<typename MsgT>
  void open(const rclcpp::Node & node, const rclcpp::Publisher<MsgT> & pub)
  {
    const char * dir = std::getenv("RCLLF_MSGLOG_DIR");
    if (dir == nullptr || *dir == '\0') {
      return;
    }
    topic_ = pub.get_topic_name();
    std::string flat = topic_;
    for (char & c : flat) {
      if (c == '/') {
        c = '_';
      }
    }
    const std::string path = std::string(dir) + "/" + node.get_name() + flat + ".jsonl";
    file_ = std::fopen(path.c_str(), "w");
    if (file_ == nullptr) {
      RCLCPP_ERROR(node.get_logger(), "msglog: cannot open %s", path.c_str());
      return;
    }
    if (const char * w = std::getenv("RCLLF_TEST_WITHHOLD")) {
      const std::string spec(w);
      const auto colon = spec.rfind(':');
      if (colon != std::string::npos && spec.substr(0, colon) == topic_) {
        withhold_ = std::strtoull(spec.c_str() + colon + 1, nullptr, 10);
        RCLCPP_WARN(
          node.get_logger(),
          "TEST HOOK RCLLF_TEST_WITHHOLD: publication %" PRIu64 " of %s is withheld",
          withhold_, topic_.c_str());
      }
    }
    const rmw_gid_t & gid = pub.get_gid();
    std::string gid_hex;
    char b[3];
    for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
      std::snprintf(b, sizeof(b), "%02x", gid.data[i]);
      gid_hex += b;
    }
    std::fprintf(
      file_,
      "{\"schema\":\"rcllf.msglog/1\",\"event\":\"open\",\"topic\":\"%s\",\"type\":\"%s\","
      "\"node\":\"%s\",\"gid\":\"%s\",\"pid\":%d,\"t_ns\":%" PRId64 "}\n",
      topic_.c_str(), rosidl_generator_traits::name<MsgT>(),
      node.get_fully_qualified_name(), gid_hex.c_str(), static_cast<int>(getpid()),
      now_ns());
    std::fflush(file_);
  }

  /// Log the next publication of *msg*; true: the caller publishes it now.
  /// Withheld (test hook) publications are logged with published=false and
  /// return false. The caller keeps its own ``pub->publish(msg)`` call (inside
  /// ``if (log.record(msg))``) so lf-analyze still sees which callback publishes
  /// which topic; one publisher is driven from one callback thread, so the log
  /// order is the publish order.
  template<typename MsgT>
  bool record(const MsgT & msg)
  {
    if (file_ == nullptr) {
      return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ++seq_;
    const bool send = seq_ != withhold_;
    rclcpp::SerializedMessage ser;
    serializer<MsgT>().serialize_message(&msg, &ser);
    const auto & raw = ser.get_rcl_serialized_message();
    std::string hex;
    hex.reserve(raw.buffer_length * 2);
    static const char * digits = "0123456789abcdef";
    for (size_t i = 0; i < raw.buffer_length; ++i) {
      hex += digits[raw.buffer[i] >> 4];
      hex += digits[raw.buffer[i] & 0xF];
    }
    std::fprintf(
      file_, "{\"event\":\"pub\",\"seq\":%" PRIu64 ",\"t_ns\":%" PRId64
      ",\"published\":%s,\"cdr\":\"%s\"}\n",
      seq_, now_ns(),
      send ? "true" : "false", hex.c_str());
    if (send) {
      ++sent_;
    }
    return send;
  }

private:
  template<typename MsgT>
  static rclcpp::Serialization<MsgT> & serializer()
  {
    static rclcpp::Serialization<MsgT> s;
    return s;
  }

  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  std::FILE * file_ = nullptr;
  std::string topic_;
  uint64_t seq_ = 0;
  uint64_t sent_ = 0;
  uint64_t withhold_ = 0;
  std::mutex mutex_;
};

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__MSG_LOG_HPP_

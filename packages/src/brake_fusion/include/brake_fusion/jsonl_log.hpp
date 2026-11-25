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

#ifndef BRAKE_FUSION__JSONL_LOG_HPP_
#define BRAKE_FUSION__JSONL_LOG_HPP_

#include <time.h>

#include <atomic>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

namespace brake_fusion
{

// One JSON object per line, one file per node: <dir>/<node>.jsonl.
//
// The directory comes from the node's `log_dir` parameter, or from the
// BF_LOG_DIR environment variable when the parameter is empty. Without either
// the log is disabled and every call is a no-op.
class JsonlLog
{
public:
  JsonlLog() = default;
  JsonlLog(const JsonlLog &) = delete;
  JsonlLog & operator=(const JsonlLog &) = delete;

  ~JsonlLog()
  {
    if (file_ != nullptr) {
      std::fclose(file_);
    }
  }

  void open(std::string dir, const std::string & node_name)
  {
    if (dir.empty()) {
      const char * env = std::getenv("BF_LOG_DIR");
      dir = env != nullptr ? env : "";
    }
    if (dir.empty()) {
      return;
    }
    path_ = dir + "/" + node_name + ".jsonl";
    file_ = std::fopen(path_.c_str(), "w");
    if (file_ != nullptr) {
      std::setvbuf(file_, nullptr, _IOLBF, 0);
    }
  }

  bool enabled() const {return file_ != nullptr;}
  const std::string & path() const {return path_;}

  // Write one line. `fmt` is the body of the JSON object without braces.
  // Every line starts with "gs", a counter shared by all logs of the process,
  // so lines from nodes in one container are totally ordered, and "wall_ns",
  // the physical CLOCK_REALTIME at the write: under rcllf t_ns
  // is the logical tag, so wall_ns is the only physical time of an event
  // (sensor-to-brake latency, lag behind the tag). Like gs it varies from run
  // to run; value-for-value comparisons drop both.
  void line(const char * fmt, ...) __attribute__((format(printf, 2, 3)))
  {
    if (file_ == nullptr) {
      return;
    }
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    const int64_t wall_ns = static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
    std::lock_guard<std::mutex> lock(mutex_);
    std::fprintf(
      file_, "{\"gs\":%" PRId64 ",\"wall_ns\":%" PRId64 ",", process_seq().fetch_add(1),
      wall_ns);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(file_, fmt, args);
    va_end(args);
    std::fputs("}\n", file_);
  }

private:
  static std::atomic<int64_t> & process_seq()
  {
    static std::atomic<int64_t> seq{0};
    return seq;
  }

  std::FILE * file_{nullptr};
  std::string path_;
  std::mutex mutex_;
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__JSONL_LOG_HPP_

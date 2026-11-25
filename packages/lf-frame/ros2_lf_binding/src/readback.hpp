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

/// @file readback.hpp
/// @brief Effective settings of the running program, written for diagnostics.
///
/// A measurement run must prove the settings it depends on from the live
/// process, not from its configuration. At the first startup reaction the
/// binding writes `rcllf_readback.<pid>.json` into `$RCLLF_READBACK_DIR`:
/// the RID table as registered, the timers reactor-c fires, the LF worker
/// count, the reactor-c patch markers compiled in, and the scheduling policy
/// of every thread. A reader can compare them with the configured values.

#ifndef READBACK_HPP_
#define READBACK_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_lf_binding
{

constexpr const char * kReadbackSchema = "rcllf.readback/1";
constexpr const char * kReadbackDirEnv = "RCLLF_READBACK_DIR";

/// The NUL-terminated marker strings in [begin, end), skipping padding.
std::vector<std::string> split_patch_markers(const char * begin, const char * end);

/// `comm`, `policy` and `rt_priority` of one `/proc/<pid>/task/<tid>/stat`
/// line, or null when the line does not parse.
nlohmann::json thread_sched_from_stat(const std::string & stat);

/// The read-back document of this process.
nlohmann::json build_readback();

/// Write build_readback() to `<dir>/rcllf_readback.<pid>.json`, atomically.
/// Returns the path, or "" when the file could not be written.
std::string write_readback(const std::string & dir);

/// Once per process: write the read-back when `$RCLLF_READBACK_DIR` is set.
void write_readback_once();

}  // namespace ros2_lf_binding

#endif  // READBACK_HPP_

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

/// @file console_log.hpp
/// @brief A binding diagnostic that goes to the console only, never to /rosout.

#ifndef CONSOLE_LOG_HPP_
#define CONSOLE_LOG_HPP_

namespace ros2_lf_binding
{

/// Logs one line through rcutils' console handler under logger `name` at
/// `severity` (RCUTILS_LOG_SEVERITY_*): same line format and severity filter
/// as the RCLCPP_* macros, but not through rcl's output handler, so no /rosout
/// publisher and no log file, whatever the nodes are called.
///
/// Two reasons. A reaction subscribed to /rosout must not run
/// extra times for a binding line. rcl's /rosout publish
/// is an rcl_publish, which the publication gate preempts and checks against
/// the corr Table under the Table's lock; a line issued under that lock through
/// RCLCPP_* waits for rclcpp's logging mutex while a node logging at the same
/// moment holds it and waits for the Table lock (a deadlock).
/// This function takes only its own leaf lock, so it is safe under any
/// binding lock.
#if defined(__GNUC__)
__attribute__((format(printf, 4, 5)))
#endif
void console_log(const char * function, int severity, const char * name, const char * fmt, ...);

}  // namespace ros2_lf_binding

#endif  // CONSOLE_LOG_HPP_

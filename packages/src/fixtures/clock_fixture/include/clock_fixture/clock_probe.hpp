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

#ifndef CLOCK_FIXTURE__CLOCK_PROBE_HPP_
#define CLOCK_FIXTURE__CLOCK_PROBE_HPP_

#include <dlfcn.h>

#include <cstdint>

namespace clock_fixture
{

/// LF logical time of the current tag, or -1 when not running under rcllf.
/// Looked up at runtime so the fixture builds without depending on the
/// binding and still runs under plain `ros2 run`.
inline int64_t lf_logical_time_ns()
{
  using Fn = int64_t (*)();
  static Fn fn = reinterpret_cast<Fn>(dlsym(RTLD_DEFAULT, "ros2_lf_logical_time_ns"));
  return fn ? fn() : -1;
}

}  // namespace clock_fixture

#endif  // CLOCK_FIXTURE__CLOCK_PROBE_HPP_

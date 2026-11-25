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

// Test-only stand-in for the lfc-generated _lf_get_environments. It reports
// no environments until a test points g_lf_environments_stub at its own
// environment (lf_start); every caller (time_source, clock_publisher,
// thread_role) treats "no environments" as "not inside an LF program".
//
// Linking does not need it: libros2_lf_binding.so carries reactor-c's util.c.o
// in every build type (see CMakeLists.txt).

extern "C" {
#include "core/environment.h"
}

environment_t * g_lf_environments_stub = nullptr;

extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = g_lf_environments_stub;
  return g_lf_environments_stub != nullptr ? 1 : 0;
}

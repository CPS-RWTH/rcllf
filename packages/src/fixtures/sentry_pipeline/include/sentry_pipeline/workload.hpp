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

#ifndef SENTRY_PIPELINE__WORKLOAD_HPP_
#define SENTRY_PIPELINE__WORKLOAD_HPP_

#include <chrono>

namespace sentry_pipeline
{

// Busy-waits for approximately `us` microseconds, calibrated against
// std::chrono::steady_clock (not a fixed iteration count), so the same call
// costs roughly the same wall-clock time regardless of host CPU speed.
// Used by every node to simulate a parameterized workload on its
// publish/callback hot path (see the scenario overlay's `busy_us` key).
void busy_wait(std::chrono::microseconds us);

}  // namespace sentry_pipeline

#endif  // SENTRY_PIPELINE__WORKLOAD_HPP_

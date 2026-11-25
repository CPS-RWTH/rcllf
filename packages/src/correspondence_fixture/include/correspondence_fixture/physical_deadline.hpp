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
#ifndef CORRESPONDENCE_FIXTURE__PHYSICAL_DEADLINE_HPP_
#define CORRESPONDENCE_FIXTURE__PHYSICAL_DEADLINE_HPP_

#include <cstdlib>

namespace correspondence_fixture
{

/// W3: CORR_PHYSICAL_DEADLINE_MS=<n> puts an n ms deadline QoS on the
/// /corr/a publisher and corr_sub_1's /corr/a subscription. With the 200 ms
/// timer DDS then counts physical deadline misses, which rcllf must never
/// deliver to the user callback. 0 (the default) sets no deadline policy.
inline int physical_deadline_ms()
{
  const char * v = std::getenv("CORR_PHYSICAL_DEADLINE_MS");
  return v != nullptr ? std::atoi(v) : 0;
}

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__PHYSICAL_DEADLINE_HPP_

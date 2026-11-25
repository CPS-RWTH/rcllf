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

#ifndef CORRESPONDENCE_FIXTURE__NESTED_ECHO_HPP_
#define CORRESPONDENCE_FIXTURE__NESTED_ECHO_HPP_

#include <functional>

namespace correspondence_fixture
{

/// Scenario "nested": publish one /corr/b echo, but only while
/// CorrPublisher's timer tick is running on this thread. CorrSubscriber calls
/// it from its /corr/a on-new-message callback, which Fast DDS runs inside the
/// tracked /corr/a publish (synchronous intraprocess delivery), so the echo
/// re-enters the publication gate on the tracked /corr/b publisher. The hook
/// is empty unless CorrPublisher runs in the same process (the LF executable).
inline std::function<void()> & nested_echo_hook()
{
  static std::function<void()> hook;
  return hook;
}

}  // namespace correspondence_fixture

#endif  // CORRESPONDENCE_FIXTURE__NESTED_ECHO_HPP_

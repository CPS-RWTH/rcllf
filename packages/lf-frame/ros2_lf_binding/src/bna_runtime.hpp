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


/// @file bna_runtime.hpp
/// @brief Drain-side calls of the BNA registry.

#ifndef BNA_RUNTIME_HPP_
#define BNA_RUNTIME_HPP_

#include <cstdint>
#include <string>

namespace ros2_lf_binding
{
namespace bna_rt
{

/// The corr dispatch of drain reaction `rid` for the duration of its
/// handle_message, read back by ros2_lf_bna_current.
class DispatchScope
{
public:
  DispatchScope(int rid, const std::string & gid, uint64_t ordinal);
  ~DispatchScope();
  DispatchScope(const DispatchScope &) = delete;
  DispatchScope & operator=(const DispatchScope &) = delete;
};

/// Fix 6: the join of drain `rid` ended missing for (gid, ordinal). Calls
/// the engine's `missing` when `rid` is a declared BNA input, else nothing.
/// Returns true when a policy received it.
/// `pub_tag` (ns, -1 when unknown) is the lost message's publish tag.
/// For an input with a declared max_stamp_lag the engine gets
/// `pub_tag - max_stamp_lag` as a stamp bound, else no stamp.
bool on_missing(int rid, const std::string & gid, uint64_t ordinal, int64_t pub_tag = -1);

/// Test only: forget every registration and declaration.
void reset_for_test();

}  // namespace bna_rt
}  // namespace ros2_lf_binding

#endif  // BNA_RUNTIME_HPP_

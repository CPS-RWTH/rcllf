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

/// @file rid_table.hpp
/// @brief Internal header for the RID entry table.
///
/// Re-exports the public RidEntry type and registration functions from
/// rid_table_api.h, and additionally declares get_rid_entries() for
/// internal use by node_binding.cpp.

#ifndef RID_TABLE_HPP_
#define RID_TABLE_HPP_

#include <string>
#include <vector>

// The public API header owns the RidEntry struct and registration functions
#include "ros2_lf_binding/rid_table_api.h"

namespace ros2_lf_binding
{

/// Return the static RID entry table populated by ros2_lf_reaction_ids_init().
const std::vector<RidEntry> & get_rid_entries();

/// The hazard code `node_class` acknowledged for handles spun by an executor
/// of their own (register_foreign_executor_ack), or "" when there is none.
std::string foreign_executor_ack(const std::string & node_class);

/// Tests only: forget every RID entry and acknowledgement.
void clear_rid_entries_for_test();

}  // namespace ros2_lf_binding

#endif  // RID_TABLE_HPP_

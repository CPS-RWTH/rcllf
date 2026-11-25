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

#include "rid_table.hpp"

#include <map>
#include <string>
#include <utility>

#include "handle_table.hpp"

namespace ros2_lf_binding
{

std::vector<HandleEntry> g_handle_table;

static std::vector<RidEntry> g_rid_entries;

const std::vector<RidEntry> & get_rid_entries() {return g_rid_entries;}

void register_rid_entry(RidEntry entry) {g_rid_entries.push_back(std::move(entry));}

static std::map<std::string, std::string> g_foreign_executor_acks;

void register_foreign_executor_ack(const std::string & node_class, const std::string & hazard_code)
{
  g_foreign_executor_acks[node_class] = hazard_code;
}

std::string foreign_executor_ack(const std::string & node_class)
{
  const auto it = g_foreign_executor_acks.find(node_class);
  return it == g_foreign_executor_acks.end() ? std::string() : it->second;
}

void clear_rid_entries_for_test()
{
  g_rid_entries.clear();
  g_foreign_executor_acks.clear();
}

void resize_handle_table(std::size_t n)
{
  if (g_handle_table.size() < n) {
    g_handle_table.resize(n);
  }
}

}  // namespace ros2_lf_binding

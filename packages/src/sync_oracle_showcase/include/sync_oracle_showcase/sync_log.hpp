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


#ifndef SYNC_ORACLE_SHOWCASE__SYNC_LOG_HPP_
#define SYNC_ORACLE_SHOWCASE__SYNC_LOG_HPP_

// RCLLF_SYNC records of the converted (and stock) sync nodes.

#include <cstdint>
#include <queue>
#include <sstream>
#include <string>

#include "custom_interfaces/msg/sync_sample.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sync_oracle_showcase/corpus.hpp"
#include "sync_oracle_showcase/versions.hpp"

namespace sync_oracle_showcase
{

using Sample = custom_interfaces::msg::SyncSample;

// The slot's input is its position in the tuple, not the message field: the
// P3 join's inputs carry the fork source's rows, all addressed to input 0.
inline Slot to_slot(const Sample::ConstSharedPtr & m, uint32_t input)
{
  Slot s;
  if (m) {
    s.present = true;
    s.input = input;
    s.row = m->row;
    s.payload = m->payload;
    s.stamp_ns = rclcpp::Time(m->header.stamp).nanoseconds();
  }
  return s;
}

inline Event to_event(
  bool signal, const Sample::ConstSharedPtr & a,
  const Sample::ConstSharedPtr & b)
{
  Event e;
  e.signal = signal;
  e.slots[0] = to_slot(a, 0);
  e.slots[1] = to_slot(b, 1);
  return e;
}

inline std::string config_json(const std::string & policy, uint32_t queue)
{
  std::ostringstream o;
  o << "{\"ev\":\"config\",\"domain\":\"" << kDomain << "\",\"policy\":\"" << policy <<
    "\",\"queue\":" << queue << ",\"message_filters\":\"" << kMessageFiltersVersion <<
    "\",\"rclcpp\":\"" << kRclcppVersion << "\"";
  return o.str();
}

// Prefix each record with the node name so one log carries every instance.
inline std::string with_node(const std::string & json, const std::string & node)
{
  return "{\"node\":\"" + node + "\"," + json.substr(1);
}

}  // namespace sync_oracle_showcase

#endif  // SYNC_ORACLE_SHOWCASE__SYNC_LOG_HPP_

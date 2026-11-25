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

/// @file pair_match.hpp
/// @brief Is one writer and one reader of this
///        process matched to each other. A matched count says how many
///        counterparts an endpoint has, not which, so other endpoints on the
///        topic (of another type, another participant, not yet discovered)
///        can make up the counts of a pair that never matches.
///
/// Which proof. The preferred proof,
/// the reader's matched-writer list holding the writer's GID, is not
/// available: Fast DDS 2.14 (Jazzy) answers
/// DataReader::get_matched_publications and
/// DataWriter::get_matched_subscriptions with RETCODE_UNSUPPORTED (checked
/// with a probe), and rmw has no per-pair matched view. So the pair is proven from
/// the ROS graph and the participant:
///   1. same context: the writer's rcl context is the reader node's. rmw_fastrtps
///      makes one DDS participant per context, so both endpoints are local
///      endpoints of one participant (same domain, no partitions).
///   2. identity: the reader node's graph view of the topic's publishers
///      (rcl_get_publishers_info_by_topic) holds an entry whose GID is the
///      writer's own (rmw_get_gid_for_publisher), so this writer, not a
///      decoy, is the one the reader's participant knows.
///   3. type: that entry's type is the reader's type (its type support).
///   4. QoS: the writer's and reader's actual QoS are compatible
///      (rmw_qos_profile_check_compatible returns RMW_QOS_COMPATIBILITY_OK).
///      ERROR refuses the pair. WARNING (compatibility could not be
///      established) gives the unknown verdict, so the warm-up fails closed.
/// Why 1-4 make the pair matched: a Fast DDS participant pairs a new local
/// endpoint with each of its own local endpoints of the same topic, same type
/// and compatible QoS when the endpoint is created (EDP local pairing, the
/// local warm-up reads them matched on its first read, match_us=0). Two
/// endpoints in one participant need no discovery, so "not yet discovered"
/// cannot hold for a pair that passed 1 and 2. A pair in two contexts is
/// refused: its match would rest on cross-participant discovery that no
/// per-pair read shows. Every unknown (a null handle, an unreadable GID,
/// QoS, type or graph) is a nullopt, which the warm-up counts as unmatched.

#ifndef PAIR_MATCH_HPP_
#define PAIR_MATCH_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rcl/context.h"
#include "rcl/node.h"
#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

namespace ros2_lf_binding
{
namespace pair_match
{

/// The answer for one pair: `paired` true (proven matched), false (proven
/// not matched yet), nullopt (cannot be told); `why` says which step decided.
struct Verdict
{
  std::optional<bool> paired;
  std::string why;
};

using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

/// One publisher entry of the reader node's graph view of the topic.
struct GraphWriter
{
  Gid gid{};
  std::string type;
};

/// What judge() decides from, read by check() or written by a test.
struct Facts
{
  bool same_context = false;
  Gid writer_gid{};
  std::string reader_type;  ///< "" when the reader's type cannot be read
  std::vector<GraphWriter> graph_writers;  ///< the reader node's view of the topic
  rmw_qos_profile_t writer_qos = rmw_qos_profile_default;
  rmw_qos_profile_t reader_qos = rmw_qos_profile_default;
};

/// Steps 1-4 of the file comment, pure.
Verdict judge(const Facts & f);

/// Reads the facts of `pub` and `sub` (whose node is `reader_node`, in
/// `reader_context`) on `topic` and judges them. nullopt when a handle is
/// null or a fact cannot be read. `reader_type_support` is the reader's
/// rosidl_message_type_support_t.
Verdict check(
  const rcl_publisher_t * pub, const rcl_subscription_t * sub, const rcl_node_t * reader_node,
  const rcl_context_t * reader_context, const void * reader_type_support,
  const std::string & topic);

}  // namespace pair_match
}  // namespace ros2_lf_binding

#endif  // PAIR_MATCH_HPP_

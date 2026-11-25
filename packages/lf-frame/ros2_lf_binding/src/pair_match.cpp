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

#include "pair_match.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/graph.h"
#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace ros2_lf_binding
{
namespace pair_match
{

Verdict judge(const Facts & f)
{
  if (!f.same_context) {
    return {false, "the writer and the reader are in different contexts (DDS participants), "
      "so no read proves this pair matched"};
  }
  const GraphWriter * own = nullptr;
  for (const auto & w : f.graph_writers) {
    if (w.gid == f.writer_gid) {
      own = &w;
      break;
    }
  }
  if (own == nullptr) {
    return {false, "the reader's node does not see the writer (by its GID) among the " +
      std::to_string(f.graph_writers.size()) + " publisher(s) of the topic"};
  }
  if (f.reader_type.empty() || own->type.empty()) {
    return {std::nullopt, "the type of the writer or the reader cannot be read"};
  }
  if (own->type != f.reader_type) {
    return {false, "the writer has type '" + own->type + "', the reader '" + f.reader_type +
      "'"};
  }
  rmw_qos_compatibility_type_t c = RMW_QOS_COMPATIBILITY_OK;
  char reason[512] = {0};
  if (rmw_qos_profile_check_compatible(
      f.writer_qos, f.reader_qos, &c, reason, sizeof(reason)) != RMW_RET_OK)
  {
    rcl_reset_error();
    return {std::nullopt, "the QoS compatibility check failed"};
  }
  if (c == RMW_QOS_COMPATIBILITY_ERROR) {
    return {false, std::string("the QoS of the writer and the reader is incompatible: ") +
      reason};
  }
  if (c != RMW_QOS_COMPATIBILITY_OK) {
    // A warning means rmw could not establish
    // compatibility (a policy is SYSTEM_DEFAULT or UNKNOWN), so it proves
    // nothing. The verdict is unknown and the warm-up fails closed.
    return {std::nullopt, std::string("the QoS compatibility of the writer and the reader "
             "could not be established: ") + reason};
  }
  return {true, ""};
}

namespace
{

std::string type_name(const void * type_support)
{
  const auto * ts = static_cast<const rosidl_message_type_support_t *>(type_support);
  if (ts == nullptr || ts->get_type_description_func == nullptr) {
    return "";
  }
  const auto * d = ts->get_type_description_func(ts);
  if (d == nullptr || d->type_description.type_name.data == nullptr) {
    return "";
  }
  return std::string(d->type_description.type_name.data, d->type_description.type_name.size);
}

}  // namespace

Verdict check(
  const rcl_publisher_t * pub, const rcl_subscription_t * sub, const rcl_node_t * reader_node,
  const rcl_context_t * reader_context, const void * reader_type_support,
  const std::string & topic)
{
  if (pub == nullptr || sub == nullptr || reader_node == nullptr || reader_context == nullptr) {
    return {std::nullopt, "the writer, the reader or the reader's node is gone"};
  }
  Facts f;
  const rcl_context_t * wctx = rcl_publisher_get_context(pub);
  if (wctx == nullptr) {
    rcl_reset_error();
    return {std::nullopt, "the writer's context cannot be read"};
  }
  f.same_context = wctx == reader_context;
  const rmw_publisher_t * rmw = rcl_publisher_get_rmw_handle(pub);
  rmw_gid_t gid;
  if (rmw == nullptr || rmw_get_gid_for_publisher(rmw, &gid) != RMW_RET_OK) {
    rcl_reset_error();
    return {std::nullopt, "the writer's GID cannot be read"};
  }
  std::memcpy(f.writer_gid.data(), gid.data, f.writer_gid.size());
  const rmw_qos_profile_t * pq = rcl_publisher_get_actual_qos(pub);
  const rmw_qos_profile_t * sq = rcl_subscription_get_actual_qos(sub);
  if (pq == nullptr || sq == nullptr) {
    rcl_reset_error();
    return {std::nullopt, "the actual QoS of the writer or the reader cannot be read"};
  }
  f.writer_qos = *pq;
  f.reader_qos = *sq;
  f.reader_type = type_name(reader_type_support);
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_topic_endpoint_info_array_t info = rmw_get_zero_initialized_topic_endpoint_info_array();
  if (rcl_get_publishers_info_by_topic(reader_node, &alloc, topic.c_str(), false, &info) !=
    RCL_RET_OK)
  {
    rcl_reset_error();
    return {std::nullopt, "the reader node's graph of the topic cannot be read"};
  }
  for (size_t i = 0; i < info.size; ++i) {
    const auto & x = info.info_array[i];
    GraphWriter w;
    std::memcpy(w.gid.data(), x.endpoint_gid, std::min(w.gid.size(), sizeof(x.endpoint_gid)));
    w.type = x.topic_type ? x.topic_type : "";
    f.graph_writers.push_back(std::move(w));
  }
  if (rmw_topic_endpoint_info_array_fini(&info, &alloc) != RMW_RET_OK) {
    rcl_reset_error();
  }
  return judge(f);
}

}  // namespace pair_match
}  // namespace ros2_lf_binding

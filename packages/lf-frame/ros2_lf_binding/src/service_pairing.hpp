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

/// @file service_pairing.hpp
/// @brief In-system service requests and responses, told apart from outside ones.
///
/// One counted write of a request port stands for one request an in-system
/// client sent, and one write of a response port for the response to it. A
/// client outside the system (a `ros2 service call`, another process) sends
/// on the same DDS topics, so a plain take could hand its request to the
/// write of an in-system one, and every later in-system request would be
/// served one write late (review round 5, M2).
///
/// Server side: the request header carries a GUID of the client that sent it
/// and the request's sequence number. That GUID is not the one
/// rmw_get_gid_for_client reports (rmw_fastrtps puts the client's response
/// reader there), so it is learned: a request whose GUID shares the
/// participant prefix (first 12 bytes) of an in-system client bound in this
/// process and whose sequence number that client sent and no server took yet
/// is that client's; its GUID is remembered. Any other request is outside:
/// served at once, counted (`external_requests`) and never consumes a write.
/// When no in-system client of the service is bound in
/// this process (its clients live in another federate) or the send gate is
/// missing, requests cannot be told apart and every one counts as in-system,
/// as before.
///
/// Client side: the executable-side gate (action_gate.cpp) reports every
/// rcl_send_request with its sequence number. A response is delivered only
/// when its sequence number is one this client sent; any other is dropped,
/// counted (`foreign_responses`) and warned about once per client.
///
/// The gate also counts the requests each thread sent per service, which the
/// generated sender reaction reads (ros2_lf_service_sent) to raise its request
/// port once per request really sent (O1-2).
///
/// RCLLF-R004: a second server for a service an in-system server serves
/// (outside the system) would answer in-system requests too. The binding
/// refuses to run when it sees one, at bind time and at each served request.

#ifndef SERVICE_PAIRING_HPP_
#define SERVICE_PAIRING_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rcl/node.h"
#include "rmw/types.h"

namespace ros2_lf_binding
{
namespace service_pairing
{

/// An in-system client bound to RID `rid`: its GID marks its requests at the
/// server of its service.
void register_client(int rid, const std::shared_ptr<rclcpp::ClientBase> & client);

/// An in-system server bound to RID `rid`; checks RCLLF-R004 at once.
void register_server(
  int rid, const std::shared_ptr<rclcpp::ServiceBase> & server,
  const std::shared_ptr<rcl_node_t> & node);

/// True when the request `id` taken by a server of `service` was not sent by
/// an in-system client. False when it was, or when no in-system client of
/// `service` is known in this process (cannot tell, counts as in-system).
bool is_outside_request(const std::string & service, const rmw_request_id_t & id);

/// One outside request served by the server RID `rid`: counted, INFO once.
void note_outside_request(int rid, const std::string & service);

/// Records a request `client` (an rcl_client_t) sent with `seq`, on this
/// thread, for `service` (fully qualified).
void note_sent(const void * client, const std::string & service, int64_t seq);

/// True (and forgotten) when `seq` is a request `client` sent.
bool take_expected(const void * client, int64_t seq);

/// One response the client RID `rid` dropped because it answers no request
/// the client sent: counted, WARN once per RID.
void note_foreign_response(int rid, const std::string & service, int64_t seq);

/// A counted write of RID `rid` that found nothing to take: WARN once per
/// RID, counted for the summary.
void note_shortfall(int rid, const char * entry, bool request);

/// Requests sent on this thread for `service` since the last call, consumed.
int sent_on_thread(const char * service);

/// Requests counted on this thread so far, all services (a mark).
int thread_sends_mark();

/// Forgets the requests counted on this thread since `mark`. A send from a
/// service or response callback has no port to raise (RCLLF-H012 refuses it
/// at analyze time), so it is warned about once per RID and dropped instead
/// of being claimed by the next sender reaction of the thread.
void discard_thread_sends(const char * entry, int rid, int mark);

/// True while `client` (an rcl_client_t) has a request without a response.
bool has_pending(const void * client);

/// True while an in-system client of `service` has a request no in-system
/// server took yet.
bool has_unserved(const std::string & service);

/// How long a counted write waits for a sample that is known to be in
/// flight (an in-system request or response) before it is a shortfall.
constexpr int kInFlightWaitMs = 200;

/// RCLLF-R004 check for the server RID `rid`; refuses to run on a finding.
/// `force` skips the rate limit (one graph query per second and server).
void check_single_server(int rid, bool force);

/// Pure R004 rule: more servers on the graph than in-system ones bound here.
bool second_server(int graph_count, int in_system_count);

/// Counts for the summary line and tests.
struct Stats
{
  uint64_t external_requests = 0;
  uint64_t foreign_responses = 0;
  uint64_t request_shortfalls = 0;
  uint64_t response_shortfalls = 0;
};
Stats stats();

/// The `ros2_lf_service_stats` line, when any service handle was bound.
void log_summary(const char * reason);

/// Test support: forget every registration and count.
void reset_for_test();

/// Suffix match of a fully qualified name against a name the analysis wrote
/// (relative or absolute), on a '/' boundary.
bool names_service(const std::string & full, const char * wanted);

}  // namespace service_pairing
}  // namespace ros2_lf_binding

#endif  // SERVICE_PAIRING_HPP_

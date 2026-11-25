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

/// @file service_pairing.cpp
/// @brief See service_pairing.hpp.

#include "service_pairing.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/client.h"
#include "rcl/graph.h"
#include "rclcpp/logging.hpp"
#include "rmw/rmw.h"
#include "ros2_lf_binding/action_gate_api.h"

namespace ros2_lf_binding
{
namespace service_pairing
{
namespace
{

using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

/// A client keeps at most this many unanswered sequence numbers; past it the
/// oldest is forgotten (its response then counts as foreign).
constexpr std::size_t kMaxPending = 4096;

struct ServerReg
{
  std::weak_ptr<rclcpp::ServiceBase> server;
  std::weak_ptr<rcl_node_t> node;  // weak: never outlives the context
  std::string service;
  std::chrono::steady_clock::time_point last_check{};
};

std::mutex g_mu;
constexpr std::size_t kPrefix = 12;  // DDS GUID prefix: the participant

struct ClientReg
{
  std::string service;
  Gid gid{};
  std::set<int64_t> unserved;  // sent, not yet taken by an in-system server
};
/// rcl_client_t of each in-system client bound here.
std::unordered_map<const void *, ClientReg> g_clients;
/// Service name (fully qualified) -> request GUIDs learned for in-system clients.
std::map<std::string, std::set<Gid>> g_learned;
/// rcl_client_t -> sequence numbers it sent and has no response for yet.
std::unordered_map<const void *, std::deque<int64_t>> g_pending;
/// Server RID -> registration (R004 checks).
std::map<int, ServerReg> g_servers;
std::set<int> g_info_once;
std::set<int> g_warn_foreign_once;
std::set<int> g_warn_shortfall_once;
std::set<int> g_warn_discard_once;
std::atomic<bool> g_any_bound{false};
/// The rcl_send_request gate reported a send in this process. A binary built
/// without the gate (a test) has no sequence record, so its responses are
/// delivered as before.
std::atomic<bool> g_gate_seen{false};

std::atomic<uint64_t> g_external{0};
std::atomic<uint64_t> g_foreign{0};
std::atomic<uint64_t> g_req_short{0};
std::atomic<uint64_t> g_resp_short{0};

struct ThreadSends
{
  std::string service;
  int n = 0;
};
thread_local std::vector<ThreadSends> t_sends;

rclcpp::Logger logger() {return rclcpp::get_logger("ros2_lf_binding");}

/// In-system servers bound here for `service`.
int local_servers(const std::string & service)
{
  int n = 0;
  for (const auto & kv : g_servers) {
    if (kv.second.service == service) {
      ++n;
    }
  }
  return n;
}

}  // namespace

bool names_service(const std::string & full, const char * wanted)
{
  if (wanted == nullptr) {
    return false;
  }
  std::string w = wanted;
  std::string f = full;
  if (!w.empty() && w[0] == '/') {
    w.erase(0, 1);
  }
  if (!f.empty() && f[0] == '/') {
    f.erase(0, 1);
  }
  if (f == w) {
    return true;
  }
  return !w.empty() && f.size() > w.size() &&
         f.compare(f.size() - w.size(), w.size(), w) == 0 &&
         f[f.size() - w.size() - 1] == '/';
}

void register_client(int rid, const std::shared_ptr<rclcpp::ClientBase> & client)
{
  if (!client) {
    return;
  }
  const std::string service = client->get_service_name();
  rmw_gid_t gid{};
  const rmw_client_t * rmw_client = rcl_client_get_rmw_handle(client->get_client_handle().get());
  if (rmw_client == nullptr || rmw_get_gid_for_client(rmw_client, &gid) != RMW_RET_OK) {
    rcl_reset_error();
    // Without the GID its requests cannot be told from outside ones. The
    // service then keeps no GID set and every request counts as in-system.
    RCLCPP_WARN(
      logger(), "service pairing: no GID for the client of RID %d (service '%s'); requests "
      "of outside clients of this service cannot be told apart", rid, service.c_str());
    g_any_bound.store(true);
    return;
  }
  std::lock_guard<std::mutex> lk(g_mu);
  ClientReg & reg = g_clients[client->get_client_handle().get()];
  reg.service = service;
  std::memcpy(reg.gid.data(), gid.data, reg.gid.size());
  g_any_bound.store(true);
}

void register_server(
  int rid, const std::shared_ptr<rclcpp::ServiceBase> & server,
  const std::shared_ptr<rcl_node_t> & node)
{
  if (!server) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(g_mu);
    ServerReg & reg = g_servers[rid];
    reg.server = server;
    reg.node = node;
    reg.service = server->get_service_name();
  }
  check_single_server(rid, true);
}

bool is_outside_request(const std::string & service, const rmw_request_id_t & id)
{
  if (!g_gate_seen.load()) {
    return false;  // no send record (no gate): cannot tell
  }
  Gid g{};
  std::memcpy(g.data(), id.writer_guid, g.size());
  std::lock_guard<std::mutex> lk(g_mu);
  bool any = false;
  ClientReg * match = nullptr;
  for (auto & kv : g_clients) {
    ClientReg & c = kv.second;
    if (c.service != service) {
      continue;
    }
    any = true;
    if (match == nullptr && std::memcmp(c.gid.data(), g.data(), kPrefix) == 0 &&
      c.unserved.count(id.sequence_number) != 0)
    {
      match = &c;
    }
  }
  if (!any) {
    return false;  // no in-system client known here: cannot tell
  }
  auto & learned = g_learned[service];
  if (learned.count(g) != 0) {
    for (auto & kv : g_clients) {
      if (kv.second.service == service) {
        kv.second.unserved.erase(id.sequence_number);
      }
    }
    return false;
  }
  if (match != nullptr) {
    match->unserved.erase(id.sequence_number);
    learned.insert(g);
    return false;
  }
  return true;
}

void note_outside_request(int rid, const std::string & service)
{
  g_external.fetch_add(1);
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    first = g_info_once.insert(rid).second;
  }
  if (first) {
    RCLCPP_INFO(
      logger(), "ros2_lf_invoke_srv_server: RID %d: served a request of a client outside the "
      "system on '%s' at once; it takes no in-system request's place (external_requests in "
      "ros2_lf_service_stats)", rid, service.c_str());
  }
}

void note_sent(const void * client, const std::string & service, int64_t seq)
{
  g_gate_seen.store(true);
  {
    std::lock_guard<std::mutex> lk(g_mu);
    auto & q = g_pending[client];
    q.push_back(seq);
    if (q.size() > kMaxPending) {
      q.pop_front();
    }
    const auto c = g_clients.find(client);
    if (c != g_clients.end() && c->second.unserved.size() < kMaxPending) {
      c->second.unserved.insert(seq);
    }
  }
  for (auto & s : t_sends) {
    if (s.service == service) {
      ++s.n;
      return;
    }
  }
  t_sends.push_back(ThreadSends{service, 1});
}

bool take_expected(const void * client, int64_t seq)
{
  if (!g_gate_seen.load()) {
    return true;
  }
  std::lock_guard<std::mutex> lk(g_mu);
  const auto it = g_pending.find(client);
  if (it == g_pending.end()) {
    return false;
  }
  for (auto q = it->second.begin(); q != it->second.end(); ++q) {
    if (*q == seq) {
      it->second.erase(q);
      return true;
    }
  }
  return false;
}

void note_foreign_response(int rid, const std::string & service, int64_t seq)
{
  g_foreign.fetch_add(1);
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    first = g_warn_foreign_once.insert(rid).second;
  }
  if (first) {
    RCLCPP_WARN(
      logger(), "ros2_lf_invoke_srv_client_cb: RID %d: dropped a response on '%s' (sequence %"
      PRId64 ") that answers no request this client sent; further ones are only counted "
      "(foreign_responses in ros2_lf_service_stats)", rid, service.c_str(), seq);
  }
}

void note_shortfall(int rid, const char * entry, bool request)
{
  (request ? g_req_short : g_resp_short).fetch_add(1);
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    first = g_warn_shortfall_once.insert(rid).second;
  }
  if (first) {
    RCLCPP_WARN(
      logger(), "%s: RID %d: no %s available; this write runs no callback (warned once per "
      "RID; %s in ros2_lf_service_stats counts them)", entry, rid,
      request ? "request" : "response", request ? "request_shortfalls" : "response_shortfalls");
  }
}

int sent_on_thread(const char * service)
{
  int n = 0;
  for (auto & s : t_sends) {
    if (names_service(s.service, service)) {
      n += s.n;
      s.n = 0;
    }
  }
  return n;
}

int thread_sends_mark()
{
  int n = 0;
  for (const auto & s : t_sends) {
    n += s.n;
  }
  return n;
}

void discard_thread_sends(const char * entry, int rid, int mark)
{
  const int sent = thread_sends_mark() - mark;
  if (sent <= 0) {
    return;
  }
  bool first = false;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    first = g_warn_discard_once.insert(rid).second;
  }
  if (first) {
    RCLCPP_WARN(
      logger(), "%s: RID %d: the callback sent %d request(s), which no port carries (a request "
      "from a service handler or a response callback, RCLLF-H012); warned once per RID", entry,
      rid, sent);
  }
  // Drop the newest `sent` counts; earlier ones belong to a sender reaction.
  int left = sent;
  for (auto it = t_sends.rbegin(); it != t_sends.rend() && left > 0; ++it) {
    const int d = std::min(left, it->n);
    it->n -= d;
    left -= d;
  }
}

bool has_pending(const void * client)
{
  std::lock_guard<std::mutex> lk(g_mu);
  const auto it = g_pending.find(client);
  return it != g_pending.end() && !it->second.empty();
}

bool has_unserved(const std::string & service)
{
  std::lock_guard<std::mutex> lk(g_mu);
  for (const auto & kv : g_clients) {
    if (kv.second.service == service && !kv.second.unserved.empty()) {
      return true;
    }
  }
  return false;
}

bool second_server(int graph_count, int in_system_count)
{
  return in_system_count > 0 && graph_count > in_system_count;
}

void check_single_server(int rid, bool force)
{
  std::shared_ptr<rcl_node_t> node;
  std::string service;
  int in_system = 0;
  {
    std::lock_guard<std::mutex> lk(g_mu);
    const auto it = g_servers.find(rid);
    if (it == g_servers.end()) {
      return;
    }
    node = it->second.node.lock();
    if (!node) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - it->second.last_check < std::chrono::seconds(1)) {
      return;
    }
    it->second.last_check = now;
    service = it->second.service;
    in_system = local_servers(service);
  }
  size_t count = 0;
  if (rcl_count_services(node.get(), service.c_str(), &count) != RCL_RET_OK) {
    rcl_reset_error();
    return;
  }
  if (second_server(static_cast<int>(count), in_system)) {
    RCLCPP_FATAL(
      logger(), "RCLLF-R004: service '%s' (server RID %d) has %zu servers on the graph but %d "
      "in-system; a server outside the system would answer in-system requests too. Stop the "
      "other server or rename the service. Refusing to run.", service.c_str(), rid, count,
      in_system);
    std::fflush(stdout);
    std::fflush(stderr);
    // _Exit, not exit: this may run on an LF worker while other threads are live.
    std::_Exit(EXIT_FAILURE);
  }
}

Stats stats()
{
  Stats s;
  s.external_requests = g_external.load();
  s.foreign_responses = g_foreign.load();
  s.request_shortfalls = g_req_short.load();
  s.response_shortfalls = g_resp_short.load();
  return s;
}

void log_summary(const char * reason)
{
  if (!g_any_bound.load()) {
    std::lock_guard<std::mutex> lk(g_mu);
    if (g_servers.empty()) {
      return;
    }
  }
  const Stats s = stats();
  RCLCPP_INFO(
    logger(), "ros2_lf_service_stats: reason=%s external_requests=%" PRIu64
    " foreign_responses=%" PRIu64 " request_shortfalls=%" PRIu64 " response_shortfalls=%" PRIu64,
    reason, s.external_requests, s.foreign_responses, s.request_shortfalls,
    s.response_shortfalls);
}

void reset_for_test()
{
  std::lock_guard<std::mutex> lk(g_mu);
  g_clients.clear();
  g_learned.clear();
  g_pending.clear();
  g_servers.clear();
  g_info_once.clear();
  g_warn_foreign_once.clear();
  g_warn_shortfall_once.clear();
  g_warn_discard_once.clear();
  g_any_bound.store(false);
  g_gate_seen.store(false);
  g_external.store(0);
  g_foreign.store(0);
  g_req_short.store(0);
  g_resp_short.store(0);
  t_sends.clear();
}

}  // namespace service_pairing
}  // namespace ros2_lf_binding

extern "C" {

void rcllf_service_gate_sent(const void * client, int64_t seq)
{
  const char * name = rcl_client_get_service_name(static_cast<const rcl_client_t *>(client));
  if (name == nullptr) {
    rcl_reset_error();
    return;
  }
  // rcl_action's goal/result/cancel clients send through rcl_send_request
  // too; action sends are counted by the action gate itself.
  if (std::strstr(name, "/_action/") != nullptr) {
    return;
  }
  ros2_lf_binding::service_pairing::note_sent(client, name, seq);
}

int ros2_lf_service_sent(int reaction_id, const char * service)
{
  (void)reaction_id;
  return ros2_lf_binding::service_pairing::sent_on_thread(service);
}

}  // extern "C"

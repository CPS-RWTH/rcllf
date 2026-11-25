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

/// @file corr_runtime.cpp
/// @brief Correspondence runtime: gate hooks, startup stage,
///        emit/arrive/drain C API, ingress sample hook, epoch close.

#include "corr_runtime.hpp"

#include <dlfcn.h>
#include <link.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bna_runtime.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "input_period.hpp"
#include "match_warmup.hpp"
#include "output_deadline.hpp"
#include "pair_match.hpp"
#include "plant_record.hpp"
#include "per_subscription.hpp"
#include "rcl/error_handling.h"
#include "rcl/graph.h"
#include "rcl/node.h"
#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rid_table.hpp"
#include "ros2_lf_msgs/msg/corr_end.hpp"
#include "silence_deadline.hpp"
#include "tag_log.hpp"
#include "tf2_ingress.hpp"
#include "rmw/qos_profiles.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rmw/rmw.h"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/publication_gate_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "semantic_trace.hpp"
#include "take_mode.hpp"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{
void stop_progress();  // runtime.cpp
}  // namespace ros2_lf_binding

namespace ros2_lf_binding
{

bool lf_is_running();  // runtime.cpp

extern "C" int64_t lf_time_physical(void);

namespace corr_rt
{
namespace
{
std::atomic<bool> g_federated{false};
}  // namespace

void set_federated(bool federated) {g_federated.store(federated);}

bool federated() {return g_federated.load(std::memory_order_relaxed);}

bool fast_federate() {return g_federated.load(std::memory_order_relaxed) && lf_fast();}

int64_t table_clock()
{
  // A sample a fast federate stamps before its start tag reads
  // the tag reactor-c holds then (0, then the launch time), never above
  // the start tag, so it cannot be judged after a cutoff.
  if (fast_federate()) {
    return std::max<int64_t>(0, lf_current_tag().first);
  }
  return lf_time_physical();
}
}  // namespace corr_rt

namespace
{

using corr::Gid;
using corr::Origin;
using corr::Outcome;

rclcpp::Logger corr_logger() {return rclcpp::get_logger("ros2_lf_binding.corr");}

struct Slot
{
  Gid gid{};
  bool gid_ok = false;
  std::string node_fqn;
  std::string topic;
  bool volatile_durability = true;
  bool best_effort = false;  ///< actual reliability at rcl_publisher_init
  bool internal = false;     ///< the node's own rcl or rclcpp publisher
  std::atomic<uint64_t> assigned{0};
  std::mutex m;
  bool tracked = false;
  std::map<int, std::vector<uint32_t>> edges_by_rid;
  std::atomic<bool> warn_pending{false};
  bool warned = false;
  std::atomic<uint64_t> no_context{0};         ///< no-context publishes, tracked or not (m5)
  std::atomic<uint32_t> refusal_pending{0};    ///< bit per Origin to warn once (m2)
  uint32_t refusal_warned = 0;
  // rcllf trace state. `traced` flips under `m` at trace_begin.
  std::atomic<bool> traced{false};
  uint64_t first_traced_ordinal = 0;
  std::atomic<uint64_t> backlog_refused{0};  ///< tracked, refused origin, before begin
  std::atomic<uint64_t> c_accepted{0}, c_failed{0}, c_nested{0}, c_unsupported{0};
  std::atomic<uint64_t> c_no_context{0}, c_undeclared{0}, c_not_admitted{0};
  std::atomic<uint64_t> c_untracked{0};      ///< publishes on an untracked slot
};

struct EdgeRt
{
  CorrEdgeSpec spec;
  rclcpp::SubscriptionBase::SharedPtr sub;
  // The local end's node, for the startup peer check only. Not owned, so
  // the node registry stays its last owner and the rmw context can be
  // finalized at shutdown.
  std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> node;
  std::string resolved;
  bool closed = false;
  // A `sub_rebind` edge: its current reader, held weakly so the node
  // stays its owner, the reader generation and whether a reader was refused.
  std::weak_ptr<rclcpp::SubscriptionBase> reader;
  uint32_t reader_gen = 0;
  bool reader_refused = false;
  // A next reader bound while `reader` still lived, and the tag
  // (time, microstep) it was deferred at. Settled by retry_deferred_reader.
  std::weak_ptr<rclcpp::SubscriptionBase> pending_reader;
  std::optional<std::pair<int64_t, uint32_t>> deferred_at;
};

struct LegacyEdge
{
  std::string topic, pub_key, sub_key, reason;
  std::string sub_topic, sub_reliability;  ///< empty: no value
  int sub_depth = -1;                      ///< -1: no value
  bool ingress = false;                    ///< listed by the publisher's federate
};

/// One analyzed publisher, for the startup RCLLF-H000 line.
struct PublisherQos
{
  std::string key, topic, reliability;  ///< reliability empty: no value
};

/// The publisher end of an edge whose publishing node is a lifecycle node.
struct RebindPub
{
  std::string node_fqn, topic;
  uint32_t edge = 0;
  std::vector<int> rids;
};

/// Leaked singleton: valid before main, during static destruction, forever.
struct State
{
  std::mutex reg_mu;
  std::unordered_map<const void *, std::shared_ptr<Slot>> slots;
  std::vector<EdgeRt> edges;          // index = edge id
  std::vector<LegacyEdge> legacy;
  std::vector<PublisherQos> publisher_qos;
  std::unique_ptr<corr::Table> table;
  std::atomic<bool> closed{false};
  std::atomic<uint32_t> deferred_readers{0};  ///< edges with a deferred reader
  std::atomic<uint64_t> after_close{0};
  std::atomic<uint64_t> nested{0};
  bool started = false;
  bool reconciled = false;
  bool reconcile_ok = true;
  int64_t close_stop_time = -1;  ///< stop tag time an edge close saw, -1 none
  uint32_t close_stop_micro = 0;  ///< microstep of that stop tag
  uint64_t drain_delay_ordinal = 0;  ///< test hook: delay the last drain of this ordinal
  int64_t drain_delay_ms = 0;
  std::mutex missing_mu;             ///< logical missed count per subscription
  /// Per live subscription, so a reader at a reused address starts at 0.
  PerSubscription<int32_t> missing_count;
  /// Logical deadline misses per subscription, a missing record's and
  /// a silent window's. Kept apart from missing_count, the lost messages the
  /// message_lost handler counts, as DDS keeps the two totals apart.
  PerSubscription<int32_t> deadline_count;
  std::atomic<uint64_t> next_dispatch{0};  ///< dispatch_id, per epoch
  bool trace_begun = false;
  std::atomic<bool> lf_start_traced{false};  ///< lf_start emitted this epoch
  /// The KEEP_LAST depth an admitted edge's reader gets at least; 0 off.
  size_t reader_depth = corr::kRetainPerEdge;
  /// User depth of each deepened reader, by (binding key, resolved
  /// topic, creation rank on that topic).
  std::map<std::tuple<std::string, std::string, size_t>, size_t> deepened;
  /// User reliability (an rmw_qos_reliability_policy_e) of each admitted
  /// reader the binding made BEST_EFFORT, keyed like `deepened`.
  std::map<std::tuple<std::string, std::string, size_t>, int> relaxed;
  /// Subscriptions created so far per (binding key, resolved topic).
  std::map<std::pair<std::string, std::string>, size_t> sub_inits;
  /// Binding key of each rcl node created inside a construction scope,
  /// so a subscription the node creates on another thread finds its key.
  std::unordered_map<const void *, std::string> construct_nodes;
  /// Publisher end of each `pub_rebind` edge. A publisher its lifecycle
  /// node creates later on the topic is bound when created (gate on_init).
  std::vector<RebindPub> rebind;
  /// LF tag each subscription was created at, by rcl_subscription_t:
  /// INT64_MIN before LF ran, nullopt off an LF worker after it started (no
  /// tag orders that creation against the publishes).
  std::unordered_map<const void *, std::optional<int64_t>> sub_created;
  std::optional<bool> lf_running_for_test;  ///< test override
};

extern "C" int64_t lf_time_physical(void);

void fill_gid(uint8_t * dst, const Gid & gid) {std::memcpy(dst, gid.data(), RCLLF_TRACE_GID_LEN);}

void trace_corr(const std::string & json);

/// Observer: expectation and terminal events from inside corr::Table.
class TraceObserver final : public corr::Observer
{
public:
  void on_expectation(const corr::Token & t) override
  {
    rcllf_tp_expectation e{};
    fill_gid(e.gid, t.gid);
    e.ordinal = t.ordinal;
    e.edge = t.edge;
    e.instance = t.instance;
    e.generation = t.generation;
    semtrace::expectation(e);
  }
  void on_terminal(uint32_t edge, const Gid & gid, uint64_t ordinal, corr::Outcome o) override
  {
    rcllf_tp_terminal e{};
    fill_gid(e.gid, gid);
    e.ordinal = ordinal;
    e.edge = edge;
    e.outcome = static_cast<uint8_t>(o);
    semtrace::terminal(e);
  }
  void on_pool_exit(
    uint32_t edge, const Gid & gid, uint64_t ordinal, corr::SampleOutcome o,
    uint64_t retained_id, int64_t stamp) override
  {
    // A sample that waited in its fan-in group's early pool is traced
    // once, when it leaves: on its member, or on the lead when none claimed it.
    rcllf_tp_sample ev{};
    ev.edge = edge;
    fill_gid(ev.gid, gid);
    ev.ordinal = ordinal;
    ev.outcome = static_cast<uint8_t>(o);
    ev.retained_id = retained_id;
    ev.arrival_physical_ns = stamp;
    semtrace::sample(ev);
    trace_corr(
      "{\"event\":\"corr\",\"kind\":\"sample\",\"edge\":" + std::to_string(edge) +
      ",\"gid\":\"" + corr::gid_hex(gid) + "\",\"ordinal\":" + std::to_string(ordinal) +
      ",\"outcome\":\"" + corr::sample_outcome_name(o) + "\",\"pooled\":true}");
  }
};

TraceObserver & trace_observer()
{
  static TraceObserver * o = new TraceObserver();
  return *o;
}

uint64_t random_epoch()
{
  std::random_device rd;
  return (static_cast<uint64_t>(rd()) << 32) ^ rd() ^
         static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
}

State & state()
{
  static State * s = [] {
      auto * st = new State();
      st->table = std::make_unique<corr::Table>(random_epoch());
      st->table->set_observer(&trace_observer());
      st->table->set_clock(corr_rt::table_clock);  // the clock of the cutoff and the lag
      st->table->set_fast(corr_rt::fast_federate);
      return st;
    }();
  return *s;
}

thread_local std::string t_construct_key;  // node under construction

/// The subscription init in flight on this thread, between
/// rcllf_gate_sub_qos and rcllf_gate_on_sub_init.
struct PendingSub
{
  bool active = false;
  bool deepened = false;
  bool relaxed = false;  ///< reliability lowered to BEST_EFFORT
  std::string key;
  std::string resolved;
  size_t rank = 0;
  size_t user_depth = 0;
  int user_reliability = 0;
};
thread_local PendingSub t_pending_sub;
/// The `sub_rebind` edges whose node and topic the subscription being
/// initialized on this thread has, noted as their reader creation once it is.
thread_local std::vector<uint32_t> t_pending_rebind;
std::function<void()> g_sub_commit_hook;  // test: runs before a rank commit
std::function<void()> g_pub_meta_hook;  // test: runs under slot->m in gate on_init
thread_local void * t_origin_self = nullptr;
thread_local uint64_t t_dispatch_id = 0;  // the dispatch publishing on this thread
thread_local int64_t t_dispatch_time = 0;
thread_local uint32_t t_dispatch_micro = 0;
thread_local int t_origin_rid = -1;
thread_local bool t_origin_set = false;
/// Activation origin: S(a), the table clock when the current activation's user
/// code began (its inputs settled), -1 outside an OriginScope.
thread_local int64_t t_origin_phys = -1;

bool read_gid(const void * publisher, Gid * out)
{
  auto * p = static_cast<const rcl_publisher_t *>(publisher);
  const rmw_publisher_t * rmw = rcl_publisher_get_rmw_handle(p);
  if (rmw == nullptr) {
    return false;
  }
  rmw_gid_t gid;
  if (rmw_get_gid_for_publisher(rmw, &gid) != RMW_RET_OK) {
    return false;
  }
  out->fill(0);
  std::memcpy(out->data(), gid.data, std::min(sizeof(gid.data), out->size()));
  return true;
}

/// The message type name ("pkg/msg/Type") of a type support handle, else "".
std::string type_name_of(const void * type_support)
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

/// A publisher rcl or rclcpp creates for the node itself, told
/// apart from a user publisher by its topic and message type. An unknown type
/// never matches, so such a publisher is judged as a user one.
/// The topic of the end-of-run COMPLETE counts.
constexpr const char * kCorrEndTopic = "/rcllf/corr_end";

bool internal_publisher(const std::string & topic, const std::string & type)
{
  auto ends_with = [](const std::string & s, const char * tail) {
      const size_t n = std::strlen(tail);
      return s.size() >= n && s.compare(s.size() - n, n, tail) == 0;
    };
  return (topic == "/rosout" && type == "rcl_interfaces/msg/Log") ||
         (topic == "/parameter_events" && type == "rcl_interfaces/msg/ParameterEvent") ||
         (ends_with(topic, "/transition_event") &&
         type == "lifecycle_msgs/msg/TransitionEvent") ||
         (ends_with(topic, "/_action/feedback") && type.find("/action/") != std::string::npos &&
         ends_with(type, "_FeedbackMessage")) ||
         (ends_with(topic, "/_action/status") && type == "action_msgs/msg/GoalStatusArray") ||
         // The binding's own end-of-run COMPLETE count
         (topic == kCorrEndTopic && type == "ros2_lf_msgs/msg/CorrEnd");
}

std::shared_ptr<Slot> slot_for(const void * publisher, bool create)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  auto it = st.slots.find(publisher);
  if (it != st.slots.end()) {
    return it->second;
  }
  if (!create) {
    return nullptr;
  }
  auto slot = std::make_shared<Slot>();
  slot->gid_ok = read_gid(publisher, &slot->gid);
  const char * topic =
    rcl_publisher_get_topic_name(static_cast<const rcl_publisher_t *>(publisher));
  slot->topic = topic ? topic : "";
  st.slots.emplace(publisher, slot);
  if (slot->gid_ok) {
    st.table->register_gid(slot->gid);
  }
  return slot;
}

/// Tracks `slot` for edge `edge` published by `rids`. Idempotent, so startup
/// and a concurrent gate on_init may both bind one publisher.
void bind_slot(Slot & slot, uint32_t edge, const std::vector<int> & rids)
{
  std::lock_guard<std::mutex> slk(slot.m);
  slot.tracked = true;
  if (slot.no_context.load() > 0) {  // published before startup (m5)
    slot.warn_pending.store(true);
  }
  for (int rid : rids) {  // every publishing reaction
    auto & ids = slot.edges_by_rid[rid];
    if (std::find(ids.begin(), ids.end(), edge) == ids.end()) {
      ids.push_back(edge);
    }
  }
}

/// A publisher found by publishers_of, its durability read under `slot->m`.
struct PubView
{
  std::shared_ptr<Slot> slot;
  bool volatile_durability = true;
};

/// Publishers of `fqn` on `topic`. The gate on_init writes the node name and
/// durability of a slot under `slot->m` only, so each is read under it. A
/// nested publish takes reg_mu while its outer publish holds `slot->m`, so the
/// reads run after reg_mu is released, never inside it. A `pub_rebind` edge
/// records its publisher end in the reg_mu section that copies the slots: a
/// slot inserted later, or one whose name is read before on_init wrote it,
/// meets that record when its on_init scans `rebind`.
std::vector<PubView> publishers_of(
  const std::string & fqn, const std::string & topic, const CorrEdgeSpec & spec)
{
  State & st = state();
  std::vector<std::shared_ptr<Slot>> all;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    all.reserve(st.slots.size());
    for (auto & kv : st.slots) {
      all.push_back(kv.second);
    }
    if (spec.pub_rebind) {
      st.rebind.push_back({fqn, topic, static_cast<uint32_t>(spec.edge_id), spec.pub_rids});
    }
  }
  std::vector<PubView> pubs;
  for (auto & slot : all) {
    std::lock_guard<std::mutex> slk(slot->m);
    if (slot->node_fqn == fqn && slot->topic == topic) {
      pubs.push_back({slot, slot->volatile_durability});
    }
  }
  return pubs;
}

struct GateCtx
{
  std::shared_ptr<Slot> slot;
  std::unique_lock<std::mutex> lk;
  uint64_t ordinal = 0;
  // What the publication event needs, captured under the slot lock.
  bool traced = false;
  uint8_t api = 0;
  corr::Origin origin = corr::Origin::NO_CONTEXT;
  int rid = RCLLF_TRACE_NO_RID;
  uint64_t dispatch_id = 0;
  int64_t tag_time = 0;
  uint32_t tag_micro = 0;
  /// The local_now edges of this admitted publication and the
  /// delivery lock on their readers, held from before rcl_publish until the
  /// publish-time take has classified their samples.
  std::vector<uint32_t> local_edges;
  IngressRole::LocalTake local_take;
};

void trace_corr(const std::string & json)
{
  if (trace_callbacks_enabled()) {
    RCLCPP_INFO(rclcpp::get_logger("ros2_lf_binding.trace"), "%s", json.c_str());
  }
}

std::string tag_json()
{
  const auto e = lf_elapsed_tag();
  return "\"elapsed\":" + std::to_string(e.first) + ",\"microstep\":" + std::to_string(e.second);
}

/// The fully resolved (expanded and remapped) name of `local` for `node`, or "".
std::string resolve_for(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node, const std::string & local)
{
  char * out = nullptr;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  if (rcl_node_resolve_name(
      node->get_rcl_node_handle(), local.c_str(), alloc, false, false, &out) != RCL_RET_OK)
  {
    rcl_reset_error();
    return "";
  }
  std::string r(out);
  alloc.deallocate(out, alloc.state);
  return r;
}

/// resolve_for on a raw rcl node, before rclcpp wraps it.
std::string resolve_rcl(const rcl_node_t * node, const char * local)
{
  char * out = nullptr;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  if (rcl_node_resolve_name(node, local, alloc, false, false, &out) != RCL_RET_OK) {
    rcl_reset_error();
    return "";
  }
  std::string r(out);
  alloc.deallocate(out, alloc.state);
  return r;
}

/// The reader depth an admitted edge wants; 0 when deepening is off.
size_t edge_reader_depth(const CorrEdgeSpec & spec, size_t fallback)
{
  if (fallback == 0) {
    return 0;
  }
  return spec.reader_depth > 0 ? spec.reader_depth : fallback;
}

/// Where subscription reaction *sub_rid* stands among its twins, the
/// subscriptions its node creates on the same topic.
/// Twins are the node's subscription RIDs whose topic *node* resolves
/// (namespace expansion and remaps) to the same name, so two subscriptions
/// declared on different names and remapped onto one topic are twins.
/// RidEntry::node_class is the per-instance binding key, so two instances of
/// one class never add up. This is the one creation-order identity: startup
/// selection (twin_rank) and the publication gate's deepening (edge_sub_rank)
/// both read it.
struct TwinOrder
{
  std::size_t rank = 0;  // twins with a smaller creation index
  std::size_t twins = 0;  // *sub_rid* included
  bool indexed = true;  // every twin has a creation index
  bool one_group = true;  // every twin sits in *sub_rid*'s analyzed callback group
};

std::optional<TwinOrder> twin_order(int sub_rid, const rcl_node_t * node)
{
  const auto & entries = get_rid_entries();
  const RidEntry * own = nullptr;
  for (const auto & r : entries) {
    if (r.reaction_id == sub_rid && r.kind == "sub") {
      own = &r;
    }
  }
  if (own == nullptr) {
    return std::nullopt;
  }
  const std::string topic = resolve_rcl(node, own->topic.c_str());
  if (topic.empty()) {
    return std::nullopt;
  }
  TwinOrder o;
  for (const auto & r : entries) {
    if (r.kind != "sub" || r.node_class != own->node_class ||
      (r.topic != own->topic && resolve_rcl(node, r.topic.c_str()) != topic))
    {
      continue;
    }
    ++o.twins;
    if (r.creation_index < 0 || own->creation_index < 0) {
      o.indexed = false;
    } else if (r.creation_index < own->creation_index) {
      ++o.rank;
    }
    if (r.group_index != own->group_index) {
      o.one_group = false;
    }
  }
  return o;
}

/// Number of twins the analysis gives *sub_rid*, itself included; zero when
/// *sub_rid* is not a registered subscription RID.
std::size_t analyzed_twins(int sub_rid, const rcl_node_t * node)
{
  const auto o = twin_order(sub_rid, node);
  return o ? o->twins : 0;
}

/// Rank of subscription reaction *sub_rid* among its twins at startup.
///
/// The analyzer gives every subscription RID a creation index (its position
/// among the node's subscriptions, in creation order) and a callback group
/// index. A callback group lists its subscriptions in creation order, the same
/// rule match_handles() relies on, so within one group the twin with the
/// k-th smallest creation index is the k-th handle the group lists. The rank
/// is that k. It is defined only when every twin RID has a creation index,
/// all twins share one analyzed group, the node's matching handles sit in one
/// callback group (*handle_groups* == 1) and there are as many twin RIDs as
/// handles (*handles*). Otherwise the result is empty and the caller refuses
/// instead of guessing. Twins in different groups are not ordered by group
/// listing, see handle_matching.hpp.
std::optional<std::size_t> twin_rank(
  int sub_rid, const rcl_node_t * node, std::size_t handles, std::size_t handle_groups)
{
  const auto o = twin_order(sub_rid, node);
  if (!o || !o->indexed || !o->one_group || handle_groups != 1 || o->twins != handles) {
    return std::nullopt;
  }
  return o->rank;
}

/// The creation rank of the edge's subscription among its twins, the
/// reader the publication gate deepens. A lone subscription is rank 0. Without
/// a creation index for every twin the rank is unknown: no reader is deepened
/// and startup refuses the edge.
std::optional<size_t> edge_sub_rank(const CorrEdgeSpec & spec, const rcl_node_t * node)
{
  const auto o = twin_order(spec.sub_rid, node);
  if (!o || (!o->indexed && o->twins > 1)) {
    return std::nullopt;
  }
  return o->rank;
}

}  // namespace

void register_corr_edge(CorrEdgeSpec spec)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (spec.edge_id < 0) {
    return;
  }
  if (st.edges.size() <= static_cast<size_t>(spec.edge_id)) {
    st.edges.resize(spec.edge_id + 1);
  }
  st.edges[spec.edge_id].spec = std::move(spec);
}

std::vector<CorrEdgeSpec> registered_corr_edges()
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  std::vector<CorrEdgeSpec> out;
  out.reserve(st.edges.size());
  for (const auto & e : st.edges) {
    out.push_back(e.spec);
  }
  return out;
}

void set_corr_edge_deferred(int edge_id)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge_id >= 0 && static_cast<size_t>(edge_id) < st.edges.size()) {
    st.edges[edge_id].spec.deferred = true;
  }
}

void set_corr_edge_same_tag(int edge_id)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge_id >= 0 && static_cast<size_t>(edge_id) < st.edges.size()) {
    st.edges[edge_id].spec.same_tag = true;
  }
}

void set_corr_edge_tardy_drop(int edge_id)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge_id >= 0 && static_cast<size_t>(edge_id) < st.edges.size()) {
    st.edges[edge_id].spec.tardy_drop = true;
  }
}

void set_corr_edge_control_stop(int edge_id)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge_id >= 0 && static_cast<size_t>(edge_id) < st.edges.size()) {
    st.edges[edge_id].spec.control_stop = true;
  }
}

void set_corr_edge_fan_in(int edge_id, int lead, uint32_t rank)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge_id >= 0 && static_cast<size_t>(edge_id) < st.edges.size()) {
    st.edges[edge_id].spec.group_lead = lead;
    st.edges[edge_id].spec.fan_in_rank = rank;
  }
}

void register_corr_legacy_edge(
  const std::string & topic, const std::string & pub_key, const std::string & sub_key,
  const std::string & reason, const std::string & sub_topic,
  const std::string & sub_reliability, int sub_depth)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.legacy.push_back({topic, pub_key, sub_key, reason, sub_topic, sub_reliability, sub_depth});
}

void register_corr_legacy_ingress(
  const std::string & topic, const std::string & pub_key, const std::string & sub_key,
  const std::string & reason, const std::string & sub_topic,
  const std::string & sub_reliability, int sub_depth)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.legacy.push_back(
    {topic, pub_key, sub_key, reason, sub_topic, sub_reliability, sub_depth, true});
}

void register_publisher_qos(
  const std::string & key, const std::string & topic, const std::string & reliability)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.publisher_qos.push_back({key, topic, reliability});
}

namespace corr_rt
{

const std::string & construct_key() {return t_construct_key;}

std::vector<std::string> ros_node_names(const std::string & key)
{
  std::vector<std::string> names;
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (const auto & [node, node_key] : st.construct_nodes) {
    if (node_key != key) {
      continue;
    }
    const char * fqn = rcl_node_get_fully_qualified_name(static_cast<const rcl_node_t *>(node));
    if (fqn != nullptr) {
      names.emplace_back(fqn);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

corr::Table & table() {return *state().table;}

OriginScope::OriginScope(void * self, int rid)
: prev_self_(t_origin_self), prev_rid_(t_origin_rid), prev_set_(t_origin_set),
  prev_phys_(t_origin_phys)
{
  t_origin_self = self;
  t_origin_rid = rid;
  t_origin_set = true;
  // Activation origin: the scope opens after the take or join settled the inputs and
  // right before the user callback, so this stamp is S(a) of the activation.
  t_origin_phys = table_clock();
}

OriginScope::~OriginScope()
{
  t_origin_self = prev_self_;
  t_origin_rid = prev_rid_;
  t_origin_set = prev_set_;
  t_origin_phys = prev_phys_;
  // The gate only queues late publishes; the callback has
  // returned, so they are logged here (console only, never /rosout).
  output_deadline::flush_pending();
}

bool origin_active() {return t_origin_set;}

int64_t origin_phys() {return t_origin_phys;}

namespace
{
thread_local std::map<std::string, int> t_topic_sent;
}  // namespace

std::map<std::string, int> & topic_sent_counts() {return t_topic_sent;}
void topic_sent_reset() {t_topic_sent.clear();}
void topic_sent_note(const std::string & resolved_topic)
{
  if (t_origin_set) {
    ++t_topic_sent[resolved_topic];
  }
}

size_t admitted_edge_count()
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  return st.edges.size();
}

void announce_test_hooks();  // defined with the hooks below
/// The RCLLF_TEST_XCORR_DROP_COMPLETE hook, false when compiled out.
bool drop_complete_for_test(uint32_t edge, uint64_t seq);

/// The publisher side of a cross-federate edge. The subscriber, its
/// reader depth and its ingress run in the peer federate.
std::optional<StartupFailure> startup_egress(
  EdgeRt & e, const std::string & what, const NodeResolver & resolve)
{
  State & st = state();
  const int id = e.spec.edge_id;
  auto fail = [id](const std::string & c, const std::string & d) {
      return std::optional<StartupFailure>(StartupFailure{id, c, d});
    };
  if (e.spec.sub_rid >= 0) {
    return fail("subscriber_rid", "an egress edge has a subscription RID " + what);
  }
  auto pub_node = resolve(e.spec.pub_key);
  if (!pub_node) {
    return fail("endpoint", "node not loaded " + what);
  }
  e.node = pub_node;
  if (pub_node->get_use_intra_process_default()) {
    return fail("intra_process", "use_intra_process_comms is on " + what);
  }
  e.resolved = resolve_for(pub_node, e.spec.pub_topic);
  if (e.resolved.empty()) {
    return fail("endpoint", "cannot resolve '" + e.spec.pub_topic + "' " + what);
  }
  const std::string pub_fqn = pub_node->get_fully_qualified_name();
  const auto pubs = publishers_of(pub_fqn, e.resolved, e.spec);
  // A lifecycle node may create its publisher after startup.
  if (pubs.size() > 1 || (pubs.empty() && !e.spec.pub_rebind)) {
    return fail(
      "endpoint", std::to_string(pubs.size()) + " publishers of " + pub_fqn + " on '" +
      e.resolved + "' " + what);
  }
  if (!pubs.empty() && !pubs.front().volatile_durability) {
    return fail("durability", "the publisher is not VOLATILE " + what);
  }
  if (!pubs.empty()) {
    bind_slot(*pubs.front().slot, static_cast<uint32_t>(id), e.spec.pub_rids);
  }
  // T_e of the edge sets the default publisher lag bound L_e = 2 T_e (R1-1).
  const int64_t timeout_ns = e.spec.timeout_ns >= 0 ? e.spec.timeout_ns :
    take_timeout_ms_env().value_or(100) * 1000000;
  st.table->add_edge(
    static_cast<uint32_t>(id), 0, e.spec.delay_ns, timeout_ns, e.spec.max_lag_ns,
    e.spec.burst, 0, 0, corr::Role::EGRESS);
  RCLCPP_INFO(
    corr_logger(),
    "rcllf corr: admitted egress edge %d (global %d) %s -> %s on '%s' (d = %lld ns, L = %lld ns)",
    id, e.spec.global_id, e.spec.pub_key.c_str(), e.spec.peer_node.c_str(), e.resolved.c_str(),
    static_cast<long long>(e.spec.delay_ns),  // NOLINT(runtime/int)
    static_cast<long long>(st.table->max_lag_ns(static_cast<uint32_t>(id))));  // NOLINT
  return std::nullopt;
}

namespace
{

// The startup QoS hazards on a reader's effective QoS (after scenario
// overlays and ROS parameters). A line fires only where the effective value
// differs from what the analysis saw, since lf-generate already reported the
// analyzed one. Both are warnings, as in the static check, and never refuse.
bool startup_h001(bool best_effort, const std::string & analyzed)
{
  return best_effort && analyzed != "best_effort";
}

bool startup_h002(bool keep_last, size_t depth, int analyzed)
{
  return keep_last && depth == 1 && analyzed != 1;
}

std::string analyzed_reliability_text(const std::string & r)
{
  return r.empty() ? std::string("no reliability") : "'" + r + "'";
}

/// The name of an rmw reliability policy the user asked for, as
/// the admitted line reports it. Only an explicit RELIABLE counts as reliable.
const char * reliability_policy_text(int policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE: return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT: return "best effort";
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT: return "system default";
    case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE: return "best available";
    default: return "unknown";
  }
}

std::string analyzed_depth_text(int d)
{
  return d < 0 ? std::string("no depth") : "depth " + std::to_string(d);
}

/// RCLLF-H002 on every reader of a legacy edge. A subscribing node
/// that is not in this process (another federate) is checked by its own
/// process, so it is skipped here. The readers of one node on one topic are
/// checked once, against what the analysis saw on any of them: twin
/// subscriptions cannot be told apart by topic name. RCLLF-H001 is checked
/// per subscription at bind time instead (check_subscription_qos).
void check_legacy_qos(
  const LegacyEdge & l, const std::vector<LegacyEdge> & all, const NodeResolver & resolve,
  std::set<std::pair<std::string, std::string>> & checked)
{
  auto node = resolve(l.sub_key);
  if (!node) {
    return;
  }
  auto topic_of = [&](const LegacyEdge & e) {
      return resolve_for(node, e.sub_topic.empty() ? e.topic : e.sub_topic);
    };
  const std::string want = topic_of(l);
  if (want.empty() || !checked.insert({l.sub_key, want}).second) {
    return;
  }
  int analyzed_depth = l.sub_depth;
  for (const auto & m : all) {
    if (m.sub_key == l.sub_key && topic_of(m) == want && m.sub_depth == 1) {
      analyzed_depth = 1;
    }
  }
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  node->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
      group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
        if (want == sp->get_topic_name()) {
          subs.push_back(sp);
        }
        return false;
      });
    });
  for (const auto & sp : subs) {
    const rclcpp::QoS q = sp->get_actual_qos();
    if (startup_h002(
        q.history() == rclcpp::HistoryPolicy::KeepLast, q.depth(), analyzed_depth))
    {
      RCLCPP_WARN(
        corr_logger(),
        "rcllf corr: RCLLF-H002 legacy edge %s -> %s on '%s': the reader of node '%s' keeps "
        "depth 1, the analysis saw %s. A burst of publishes between two reactions keeps only "
        "the most recent sample; deepen the subscription or analyze the effective QoS",
        l.pub_key.c_str(), l.sub_key.c_str(), want.c_str(), node->get_fully_qualified_name(),
        analyzed_depth_text(analyzed_depth).c_str());
    }
  }
}

/// RCLLF-H000 on the effective reliability of every publisher handle
/// whose node and topic carry an analyzed publisher without a reliability.
/// Twin publishers of one node on one topic cannot be told apart, so with a
/// mix of known and unknown rows every handle of the pair is logged and the
/// line counts the unknown ones. A pair with no handle at startup (created
/// later, for example by a lifecycle transition) gets one line saying so.
/// The node's own rcl and rclcpp publishers (/rosout, /parameter_events,
/// lifecycle and action topics) are never taken for an analyzed one.
void check_publisher_qos(const std::vector<PublisherQos> & rows, const NodeResolver & resolve)
{
  State & st = state();
  std::map<std::pair<std::string, std::string>, std::pair<size_t, size_t>> pairs;
  for (const auto & r : rows) {
    auto & n = pairs[{r.key, r.topic}];
    ++n.first;
    n.second += r.reliability.empty() ? 1 : 0;
  }
  for (const auto & [kt, n] : pairs) {
    if (n.second == 0) {
      continue;
    }
    auto node = resolve(kt.first);
    if (!node) {
      continue;
    }
    const std::string fqn = node->get_fully_qualified_name();
    const std::string resolved = resolve_for(node, kt.second);
    std::vector<std::shared_ptr<Slot>> slots;
    {
      std::lock_guard<std::mutex> lk(st.reg_mu);
      for (const auto & kv : st.slots) {
        slots.push_back(kv.second);
      }
    }
    // The gate writes the slot fields under slot->m, never under reg_mu.
    std::vector<bool> best_effort;
    for (const auto & s : slots) {
      std::lock_guard<std::mutex> lk(s->m);
      if (!s->internal && s->node_fqn == fqn && s->topic == resolved) {
        best_effort.push_back(s->best_effort);
      }
    }
    const std::string which = n.first == 1 ? std::string("the analysis saw no reliability") :
      "the analysis saw no reliability on " + std::to_string(n.second) + " of the " +
      std::to_string(n.first) + " publishers of the node on this topic";
    if (best_effort.empty()) {
      RCLCPP_INFO(
        corr_logger(),
        "rcllf corr: RCLLF-H000 publisher of node '%s' on '%s': %s, and no such publisher exists "
        "at startup, so its effective reliability is not checked", fqn.c_str(),
        resolved.c_str(), which.c_str());
    }
    for (bool be : best_effort) {
      RCLCPP_INFO(
        corr_logger(),
        "rcllf corr: RCLLF-H000 publisher of node '%s' on '%s': %s, the publisher runs %s",
        fqn.c_str(), resolved.c_str(), which.c_str(), be ? "best effort" : "reliable");
    }
  }
}

/// RCLLF_ADMITTED_READER_RELIABLE=1 keeps the user's RELIABLE reader on
/// an admitted edge (comparison runs); the default makes it best effort.
bool keep_admitted_reader_reliable()
{
  const char * keep = std::getenv("RCLLF_ADMITTED_READER_RELIABLE");
  return keep != nullptr && std::strcmp(keep, "1") == 0;
}

}  // namespace

void check_subscription_qos(const RidEntry & entry, const rclcpp::SubscriptionBase & sub)
{
  State & st = state();
  const std::string resolved = sub.get_topic_name();
  std::vector<std::string> pubs;
  std::string legacy_pubs;
  bool admitted_reader = false;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (const auto & e : st.edges) {
      admitted_reader = admitted_reader ||
        (!e.spec.pub_key.empty() && e.spec.role != CorrEdgeRole::kEgress &&
        e.spec.sub_rid == entry.reaction_id);
    }
    for (const auto & l : st.legacy) {
      const std::string & t = l.sub_topic.empty() ? l.topic : l.sub_topic;
      // Twins read one publisher through a row each, it is named once.
      if (l.sub_key == entry.node_class && (t == entry.topic || t == resolved) &&
        std::find(pubs.begin(), pubs.end(), l.pub_key) == pubs.end())
      {
        pubs.push_back(l.pub_key);
      }
    }
  }
  for (const auto & p : pubs) {
    legacy_pubs += (legacy_pubs.empty() ? "" : ", ") + p;
  }
  const bool best_effort =
    sub.get_actual_qos().reliability() == rclcpp::ReliabilityPolicy::BestEffort;
  if (entry.sub_reliability.empty()) {
    RCLCPP_INFO(
      corr_logger(),
      "rcllf corr: RCLLF-H000 subscription RID %d (%s::%s) on '%s': the analysis saw no "
      "reliability, the subscription runs %s", entry.reaction_id,
      entry.node_class.c_str(), entry.callback_name.c_str(), resolved.c_str(),
      best_effort ? "best effort" : "reliable");
  }
  // An admitted reader gets its RCLLF-H001 line from startup(), naming the edge.
  if (admitted_reader || !startup_h001(best_effort, entry.sub_reliability)) {
    return;
  }
  const std::string where = legacy_pubs.empty() ?
    std::string("It is no edge's reader (an input from outside the analyzed graph), so DDS may "
    "drop a sample unseen") :
    "It reads the legacy edge from " + legacy_pubs + ", which reports a sample DDS drops only "
    "when a later take of an unfiltered reader sees the gap, never a trailing one";
  RCLCPP_WARN(
    corr_logger(),
    "rcllf corr: RCLLF-H001 subscription RID %d (%s::%s) on '%s' runs best effort, the "
    "analysis saw %s. %s; make the subscription reliable or analyze the effective QoS",
    entry.reaction_id, entry.node_class.c_str(), entry.callback_name.c_str(), resolved.c_str(),
    analyzed_reliability_text(entry.sub_reliability).c_str(), where.c_str());
}

namespace
{
/// The readers this writer has matched, 0 once it is gone.
/// reg_mu is held across the count, as in publisher_match_counts.
size_t writer_match_count(const std::shared_ptr<Slot> & slot)
{
  if (!slot) {
    return 0;
  }
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (const auto & kv : st.slots) {
    if (kv.second != slot) {
      continue;
    }
    size_t n = 0;
    if (rcl_publisher_get_subscription_count(
        static_cast<const rcl_publisher_t *>(kv.first), &n) != RCL_RET_OK)
    {
      rcl_reset_error();
      n = 0;
    }
    return n;
  }
  return 0;
}

/// Whether this writer and `reader` (of
/// `reader_node`) are proven matched to each other (pair_match::check).
/// Unknown (nullopt) once either is gone. reg_mu is held as in
/// writer_match_count.
pair_match::Verdict writer_pair_match(
  const std::shared_ptr<Slot> & slot, const std::shared_ptr<rclcpp::SubscriptionBase> & reader,
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> & reader_node)
{
  if (!slot || !reader || !reader_node) {
    return {std::nullopt, "the writer, the reader or the reader's node is gone"};
  }
  const auto ctx = reader_node->get_context();
  const rcl_context_t * reader_ctx = ctx ? ctx->get_rcl_context().get() : nullptr;
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (const auto & kv : st.slots) {
    if (kv.second == slot) {
      return pair_match::check(
        static_cast<const rcl_publisher_t *>(kv.first), reader->get_subscription_handle().get(),
        reader_node->get_rcl_node_handle(), reader_ctx,
        &reader->get_message_type_support_handle(), reader->get_topic_name());
    }
  }
  return {std::nullopt, "the writer is no longer registered"};
}

/// ROS2_LF_DISCOVERY_TIMEOUT_MS as runtime.cpp's discovery_timeout_ms reads
/// it (default 5000). A value <= 0 makes the local match one check.
int64_t local_discovery_timeout_ms()
{
  int64_t timeout_ms = 5000;
  if (const char * t = std::getenv("ROS2_LF_DISCOVERY_TIMEOUT_MS")) {
    try {
      timeout_ms = std::stoll(t);
    } catch (...) {
    }
  }
  return timeout_ms;
}
}  // namespace

std::optional<StartupFailure> startup(const NodeResolver & resolve)
{
  State & st = state();
  std::vector<EdgeRt> edges;
  std::vector<LegacyEdge> legacy;
  std::vector<PublisherQos> publisher_qos;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    edges = st.edges;
    legacy = st.legacy;
    publisher_qos = st.publisher_qos;
  }
  check_publisher_qos(publisher_qos, resolve);
  std::set<std::pair<std::string, std::string>> qos_checked;
  size_t listed = 0;
  for (const auto & l : legacy) {
    if (!l.ingress) {
      ++listed;
      RCLCPP_INFO(
        corr_logger(), "rcllf corr: legacy edge %s %s -> %s: %s", l.topic.c_str(),
        l.pub_key.c_str(), l.sub_key.c_str(), l.reason.c_str());
    }
    check_legacy_qos(l, legacy, resolve, qos_checked);
  }
  if (edges.empty()) {
    return std::nullopt;
  }
  auto fail = [](int e, const std::string & c, const std::string & d) {
      return std::optional<StartupFailure>(StartupFailure{e, c, d});
    };
  // D1.1: RMW identity. D1.2: publication mode by configuration.
  const char * rmw_id = rmw_get_implementation_identifier();
  if (rmw_id == nullptr || std::strcmp(rmw_id, "rmw_fastrtps_cpp") != 0) {
    return fail(
      0, "rmw_implementation", std::string("'") + (rmw_id ? rmw_id : "?") +
      "' is not rmw_fastrtps_cpp");
  }
  if (const char * m = std::getenv("RMW_FASTRTPS_PUBLICATION_MODE")) {
    if (std::strcmp(m, "SYNCHRONOUS") != 0) {
      return fail(0, "publication_mode", std::string("RMW_FASTRTPS_PUBLICATION_MODE=") + m);
    }
  }
  for (const char * var :
    {"FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
      "RMW_FASTRTPS_USE_QOS_FROM_XML"})
  {
    if (const char * v = std::getenv(var)) {
      return fail(0, "xml_profile", std::string(var) + "=" + v);
    }
  }
  for (const char * f : {"DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml"}) {
    if (access(f, F_OK) == 0) {
      return fail(0, "xml_profile", std::string(f) + " in the working directory");
    }
  }
  // D1.5: the gate preempts every gated symbol.
  using SymFn = void * (*)(const char *);
  auto gate_symbol = reinterpret_cast<SymFn>(dlsym(RTLD_DEFAULT, "rcllf_gate_symbol"));
  if (gate_symbol == nullptr) {
    return fail(0, "gate_self_check", "publication_gate.cpp is not linked into the executable");
  }
  for (const char * sym :
    {"rcl_publish", "rcl_publish_serialized_message", "rcl_publish_loaned_message",
      "rcl_publisher_fini", "rcl_publisher_init", "rcl_subscription_init", "rcl_node_init",
      "rcl_node_fini"})
  {
    if (dlsym(RTLD_DEFAULT, sym) != gate_symbol(sym)) {
      return fail(0, "gate_self_check", std::string("gate not on the ") + sym + " path");
    }
  }
  announce_test_hooks();
#ifdef RCLLF_CORR_TEST_HOOKS
  // These three older hooks inject faults, so like the edge hooks they
  // exist only in a -DRCLLF_CORR_TEST_HOOKS=ON build; an ordinary build logs
  // that a set one is compiled out (announce_test_hooks).
  if (const char * d = std::getenv("RCLLF_TEST_CORR_DROP_TOKEN")) {
    st.table->set_drop_token_for_test(std::strtoull(d, nullptr, 10));
  }
  if (const char * d = std::getenv("RCLLF_TEST_CORR_DROP_SAMPLE")) {
    // "<ordinal>" drops on edge 0, "<edge>:<ordinal>" on that edge. The
    // table applies it on the member it resolves the sample to.
    const char * colon = std::strchr(d, ':');
    st.table->set_drop_sample_for_test(
      colon ? static_cast<uint32_t>(std::strtoul(d, nullptr, 10)) : 0,
      std::strtoull(colon ? colon + 1 : d, nullptr, 10));
  }
  // lag test hook: "<ordinal>:<ms>" sleeps before the completion
  // measurement of the drain that pops `ordinal` and leaves every carrier empty.
  if (const char * d = std::getenv("RCLLF_TEST_CORR_DRAIN_DELAY")) {
    char * rest = nullptr;
    st.drain_delay_ordinal = std::strtoull(d, &rest, 10);
    st.drain_delay_ms = (rest != nullptr && *rest == ':') ? std::strtoll(rest + 1, nullptr, 10) : 0;
  }
#endif  // RCLLF_CORR_TEST_HOOKS
  // The members of a fan-in group fill one reader, so their bursts add
  // up. The group's reader is admitted once, on its lead, after every member
  // joined the group, so ingress routes each sample by its writer from the
  // first take on.
  std::map<int, uint64_t> group_burst;
  for (const auto & e : edges) {
    if (e.spec.group_lead >= 0) {
      group_burst[e.spec.group_lead] += e.spec.burst;
    }
  }
  std::vector<std::pair<int, rclcpp::SubscriptionBase::SharedPtr>> group_readers;
  // The local_now edges whose reader and writer must match
  // before tag 0, and the first such edge, for the failure.
  std::vector<match_warmup::LocalEdge> local_edges;
  int first_local = -1;

  for (auto & e : edges) {
    const int id = e.spec.edge_id;
    const std::string what = "(" + e.spec.pub_key + " -> " + e.spec.sub_key + " on '" +
      e.spec.sub_topic + "')";
    // An egress edge has only its publisher here, an ingress edge only
    // its subscriber. The other end is checked by its own federate.
    const bool egress = e.spec.role == CorrEdgeRole::kEgress;
    const bool ingress = e.spec.role == CorrEdgeRole::kIngress;
    if (ingress) {
      if (!e.spec.pub_rids.empty()) {
        return fail(id, "publisher_rids", "an ingress edge has publishing RIDs " + what);
      }
    } else {
      std::set<int> rids(e.spec.pub_rids.begin(), e.spec.pub_rids.end());
      if (rids.empty() || rids.size() != e.spec.pub_rids.size()) {
        return fail(
          id, "publisher_rids",
          std::to_string(e.spec.pub_rids.size()) + " publishing RIDs, " +
          std::to_string(rids.size()) + " distinct " + what);
      }
    }
    if (egress && e.spec.deferred) {
      // The deferred join belongs to the ingress row of the subscriber's
      // federate, generation never sets it on the publisher's egress row.
      return fail(id, "join_deferred", "'join: deferred' on an egress edge " + what);
    }
    if (egress) {
      auto f = startup_egress(e, what, resolve);
      if (f) {
        return f;
      }
      continue;
    }
    auto sub_node = resolve(e.spec.sub_key);
    auto pub_node = ingress ? sub_node : resolve(e.spec.pub_key);
    if (!sub_node || !pub_node) {
      return fail(id, "endpoint", "node not loaded " + what);
    }
    // D1.3: intra-process off on both nodes.
    if (sub_node->get_use_intra_process_default() || pub_node->get_use_intra_process_default()) {
      return fail(id, "intra_process", "use_intra_process_comms is on " + what);
    }
    // Review m3: resolve the node-local name the way rcl does for this node
    // (namespace expansion and remaps) and compare exactly.
    const std::string want = resolve_for(sub_node, e.spec.sub_topic);
    if (want.empty()) {
      return fail(id, "endpoint", "cannot resolve '" + e.spec.sub_topic + "' " + what);
    }
    // A lifecycle subscription made or reset in a hook is a deferred
    // binding. Its reader is checked and admitted when its RID binds it
    // (on_subscription_bound), each new reader again.
    const bool deferred = e.spec.sub_rebind;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
    std::set<const rclcpp::CallbackGroup *> sub_groups;
    if (!deferred) {
      sub_node->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
          group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
            if (want == sp->get_topic_name()) {
              subs.push_back(sp);
              sub_groups.insert(group.get());
            }
            return false;
        });
      });
    }
    // Twin subscriptions on one topic get one edge
    // each. This runs before ros2_lf_bind_node, so the handle table is still
    // empty, and the edge's subscription handle is picked by twin_rank().
    // The analyzed twin count must equal the run-time match count even when
    // one subscription matches, or every twin edge would take that handle.
    const rcl_node_t * sub_rcl = sub_node->get_rcl_node_handle();
    const std::size_t analyzed = analyzed_twins(e.spec.sub_rid, sub_rcl);
    if (!deferred && (subs.size() > 1 || analyzed > 1)) {
      if (analyzed != subs.size()) {
        return fail(
          id, "endpoint", "the analysis found " + std::to_string(analyzed) +
          " subscriptions of the node on this topic and " + std::to_string(subs.size()) +
          " match at run time " + what);
      }
      const auto rank = twin_rank(e.spec.sub_rid, sub_rcl, subs.size(), sub_groups.size());
      if (!rank) {
        return fail(
          id, "endpoint", std::to_string(subs.size()) +
          " subscriptions match and the twin's creation index does not single one out " + what);
      }
      subs = {subs[*rank]};
    }
    if (!deferred && subs.size() != 1) {
      return fail(
        id, "endpoint", std::to_string(subs.size()) + " subscriptions match " + what);
    }
    e.sub = deferred ? nullptr : subs.front();
    e.resolved = deferred ? want : e.sub->get_topic_name();
    e.node = sub_node;
    // Review m4: an intra-process subscription gets its messages from the
    // intra-process manager, never from the DDS reader ingress takes from.
    if (!deferred && e.sub->get_intra_process_waitable() != nullptr) {
      return fail(id, "intra_process", "the subscription uses intra-process delivery " + what);
    }
    const std::string pub_fqn = pub_node->get_fully_qualified_name();
    std::vector<PubView> pubs;
    if (!ingress) {
      pubs = publishers_of(pub_fqn, e.resolved, e.spec);
    }
    // A lifecycle node may create its publisher after startup.
    if (!ingress && (pubs.size() > 1 || (pubs.empty() && !e.spec.pub_rebind))) {
      return fail(
        id, "endpoint", std::to_string(pubs.size()) + " publishers of " + pub_fqn + " on '" +
        e.resolved + "' " + what);
    }
    // D1.4: VOLATILE on both endpoints (R10).
    if ((!pubs.empty() && !pubs.front().volatile_durability) ||
      (!deferred && e.sub->get_actual_qos().durability() != rclcpp::DurabilityPolicy::Volatile))
    {
      return fail(id, "durability", "an endpoint is not VOLATILE " + what);
    }
    // The reader depth DDS actually gave (rcl_subscription_get_actual_qos),
    // against the user's request. Below kRetainPerEdge a writer that publishes
    // again before ingress takes overwrites a sample in DDS, so startup refuses.
    const rclcpp::QoS actual = deferred ? rclcpp::QoS(1) : e.sub->get_actual_qos();
    const bool keep_last = !deferred && actual.history() == rclcpp::HistoryPolicy::KeepLast;
    size_t user_depth = actual.depth();
    size_t want_depth = 0;
    // The reliability the user asked for; the binding made the reader
    // best effort when `relaxed` holds the user's value. Only an explicit
    // RELIABLE counts as reliable for RCLLF-H001 (review): SYSTEM_DEFAULT or
    // an unknown policy is reported by name, never as "user reliable".
    bool user_best_effort = actual.reliability() == rclcpp::ReliabilityPolicy::BestEffort;
    bool relaxed = false;
    int user_reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    std::optional<size_t> rank;
    {
      std::lock_guard<std::mutex> lk(st.reg_mu);
      rank = edge_sub_rank(e.spec, sub_node->get_rcl_node_handle());
      auto it = rank ? st.deepened.find({e.spec.sub_key, e.resolved, *rank}) : st.deepened.end();
      if (it != st.deepened.end()) {
        user_depth = it->second;
      }
      auto rit = rank ? st.relaxed.find({e.spec.sub_key, e.resolved, *rank}) : st.relaxed.end();
      if (rit != st.relaxed.end()) {
        relaxed = true;
        user_reliability = rit->second;
        user_best_effort = user_reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      }
      want_depth = edge_reader_depth(e.spec, st.reader_depth);
    }
    if (!rank) {
      RCLCPP_FATAL(
        corr_logger(),
        "rcllf corr: edge %d subscription RID %d of node '%s' (%s) on '%s' has no creation "
        "index in the RID table, so rcllf cannot tell which of the node's subscriptions on the "
        "topic the edge names and deepens none; regenerate with colcon lf-generate. Refusing "
        "to start",
        id, e.spec.sub_rid, sub_node->get_fully_qualified_name(), e.spec.sub_key.c_str(),
        e.resolved.c_str());
      return fail(id, "creation_index", "no creation index for subscription RID " +
               std::to_string(e.spec.sub_rid) + " " + what);
    }
    if (keep_last && actual.depth() < want_depth) {
      RCLCPP_FATAL(
        corr_logger(),
        "rcllf corr: edge %d reader of node '%s' (%s) on '%s' keeps %zu sample(s), fewer than "
        "%zu: a writer that publishes again before ingress takes would overwrite the earlier "
        "sample, which then ends LOST. The subscription was created outside the node's "
        "construction scope and its node; refusing to start",
        id, sub_node->get_fully_qualified_name(), e.spec.sub_key.c_str(), e.resolved.c_str(),
        actual.depth(), want_depth);
      return fail(
        id, "reader_depth", "reader depth " + std::to_string(actual.depth()) + " below " +
        std::to_string(want_depth) + " " + what);
    }
    // The reader of a cross-federate (ingress) edge must run best
    // effort. rcllf_gate_sub_qos relaxes a reader a construction scope
    // created; one created elsewhere keeps the user's RELIABLE, under which
    // one lost sample withholds every later sample behind the sequence gap
    // for the writer's history depth. A same-federate edge keeps the user's
    // reliability: its loss never blocks a join across the federation link,
    // so DDS retransmission still buys it something.
    if (ingress && !deferred && !relaxed && !keep_admitted_reader_reliable() &&
      actual.reliability() == rclcpp::ReliabilityPolicy::Reliable)
    {
      RCLCPP_FATAL(
        corr_logger(),
        "rcllf corr: ingress edge %d reader of node '%s' (%s) on '%s' runs RELIABLE: one lost "
        "sample would withhold every later sample behind the gap until the writer's history "
        "evicts it. The subscription was created outside the node's "
        "construction scope and its node; RCLLF_ADMITTED_READER_RELIABLE=1 keeps it; refusing "
        "to start",
        id, sub_node->get_fully_qualified_name(), e.spec.sub_key.c_str(), e.resolved.c_str());
      return fail(id, "reader_reliability", "reader RELIABLE outside a construction scope " + what);
    }
    // One invocation's burst must fit the reader (lf-generate refuses it
    // too, so only a hand-written or stale registration reaches this).
    const bool grouped = e.spec.group_lead >= 0;
    const uint64_t burst = grouped ? group_burst[e.spec.group_lead] : e.spec.burst;
    if (want_depth > 0 && burst > want_depth) {
      return fail(
        id, "reader_depth", std::string(grouped ? "fan-in group burst " : "burst ") +
        std::to_string(burst) + " above reader depth " + std::to_string(want_depth) + " " + what);
    }
    if (!pubs.empty()) {
      bind_slot(*pubs.front().slot, static_cast<uint32_t>(id), e.spec.pub_rids);
    }
    // T_e of the edge. Until S2 fills the spec it is the take
    // timeout the subscription's dispatch already uses.
    int64_t timeout_ns = e.spec.timeout_ns;
    if (timeout_ns < 0) {
      int64_t ms = 100;
      if (e.spec.sub_rid >= 0 && static_cast<size_t>(e.spec.sub_rid) < g_handle_table.size()) {
        ms = g_handle_table[e.spec.sub_rid].take_timeout_ms;
      }
      timeout_ns = take_timeout_ms_env().value_or(ms) * 1000000;
    }
    st.table->add_edge(
      static_cast<uint32_t>(id), static_cast<uint32_t>(e.spec.sub_rid), e.spec.delay_ns,
      timeout_ns, e.spec.max_lag_ns, e.spec.burst, e.spec.retain, e.spec.carrier,
      ingress ? corr::Role::INGRESS : corr::Role::LOCAL);
    if (grouped) {
      // The lead is the group's lowest edge id, so it is registered
      // first. Every member names the lead's subscription.
      const int lead = e.spec.group_lead;
      const CorrEdgeSpec * ls =
        static_cast<size_t>(lead) < edges.size() ? &edges[lead].spec : nullptr;
      std::string why;
      if (deferred) {
        why = "a fan-in group member's reader is made in a lifecycle hook";
      } else if (ls == nullptr || ls->group_lead != lead || ls->sub_key != e.spec.sub_key ||
        ls->sub_rid != e.spec.sub_rid || ls->sub_topic != e.spec.sub_topic)
      {
        why = "fan-in group " + std::to_string(lead) + ": edge " + std::to_string(id) +
          " does not name the lead's subscription";
      } else {
        st.table->set_fan_in(
          static_cast<uint32_t>(id), static_cast<uint32_t>(lead), e.spec.fan_in_rank, &why);
      }
      if (!why.empty()) {
        return fail(id, "fan_in", why + " " + what);
      }
    }
    if (deferred) {
      st.table->set_sub_rebind(static_cast<uint32_t>(id));
    }
    if (!ingress && !deferred) {
      // Writer and reader share this process, and a SYNCHRONOUS
      // publish puts the sample in the reader before rcl_publish returns. The
      // publication takes it there (rcllf_gate_post), so the edge is decided
      // when the publish returns and has no cutoff. A self edge is one.
      st.table->set_local_now(static_cast<uint32_t>(id));
      if (!pubs.empty()) {
        // A lifecycle writer made after startup is not waited for here.
        // pubs.front() is the edge's only writer: more than one publisher of
        // the node on the topic refused startup above ("endpoint"), and a
        // second writer into the same reader is an edge of its own (a fan-in
        // member) with its own entry here.
        std::weak_ptr<rclcpp::SubscriptionBase> reader = e.sub;
        std::weak_ptr<Slot> writer = pubs.front().slot;
        local_edges.push_back(
          {e.resolved, e.sub.get(), pubs.front().slot.get(),
            [reader]() {
              auto r = reader.lock();
              return r ? r->get_publisher_count() : size_t{0};
            },
            [writer]() {return writer_match_count(writer.lock());},
            [writer, reader, reader_node = e.node]() {
              return writer_pair_match(writer.lock(), reader.lock(), reader_node.lock());
            }});
        if (first_local < 0) {
          first_local = id;
        }
      }
      RCLCPP_INFO(
        corr_logger(), "rcllf corr: edge %d is local, decided when its publish returns, no "
        "cutoff", id);
    } else if (!ingress) {
      // A rebind reader's samples wait in DDS until its
      // admission, so the publish cannot take them. Today's cutoff path stays.
      RCLCPP_WARN(
        corr_logger(), "rcllf corr: edge %d is a local sub_rebind edge: it keeps the cutoff "
        "t + d + T and is not LF-faithful", id);
    }
    if (e.spec.deferred && !st.table->set_deferred(static_cast<uint32_t>(id))) {
      // Generation refuses these, a hand-written header must not turn
      // the opt-in silently into a blocking join.
      return fail(
        id, "join_deferred",
        "'join: deferred' on a local edge is not supported, a local edge is decided at "
        "publish " + what);
    }
    if (e.spec.same_tag && !st.table->set_same_tag(static_cast<uint32_t>(id))) {
      // Generation sets it on in-process d = 0 edges of two nodes only;
      // elsewhere the drain is woken a microstep later and the key would lie.
      return fail(
        id, "same_tag",
            "same-tag delivery needs an in-process d = 0 immediate-join edge of two nodes " + what);
    }
    if (ingress) {
      // The receiver judges under the declared bound on the offset of
      // the publisher's clock, which must stay below T_e.
      const XfedClockBound bound = xfed_clock_bound();
      const std::string problem = !bound.problem.empty() ? bound.problem :
        xfed_bound_problem(bound.eps_ns, timeout_ns);
      if (!problem.empty()) {
        return fail(id, "clock_bound", problem + " " + what);
      }
      st.table->set_clock_bound(static_cast<uint32_t>(id), bound.eps_ns);
      // A centralized d = 0 ingress runs over a plain zero-delay
      // federated connection (generation emits no `after`), so its records
      // arrive and release at the publication tag (t, m). A decentralized one
      // (tardy_drop) keeps `after 0` and (t, m + 1).
      if (e.spec.delay_ns == 0 && !e.spec.tardy_drop) {
        st.table->set_zero_link(static_cast<uint32_t>(id));
      }
      if (bound.declared) {
        RCLCPP_INFO(
          corr_logger(), "rcllf corr: ingress edge %d judges under clock bound eps = %lld ns",
          id, static_cast<long long>(bound.eps_ns));  // NOLINT
      }
    }
    if (deferred) {
      RCLCPP_INFO(
        corr_logger(),
        "rcllf corr: admitted %sedge %d %s -> %s on '%s' with a deferred reader: the lifecycle "
        "node makes or resets the subscription in a hook, each reader is checked when it is "
        "bound", ingress ? "ingress " : "", id, e.spec.pub_key.c_str(),
        e.spec.sub_key.c_str(), e.resolved.c_str());
      continue;
    }
    if (grouped) {
      if (e.spec.group_lead == id) {
        group_readers.emplace_back(id, e.sub);
      }
    } else if (!IngressRole::instance().admit_corr_subscription(static_cast<uint32_t>(id), e.sub)) {
      return fail(id, "ingress", "ingress could not admit the subscription " + what);
    }
    // RCLLF-H001 on the effective QoS. A scenario overlay or a ROS
    // parameter can make the reader best effort after the analysis saw
    // another reliability. H001 warns and never refuses (an analyzed
    // best-effort edge was already noted by lf-generate).
    if (startup_h001(user_best_effort, e.spec.sub_reliability)) {
      const std::string seen = analyzed_reliability_text(e.spec.sub_reliability);
      RCLCPP_WARN(
        corr_logger(),
        "rcllf corr: RCLLF-H001 admitted edge %d %s -> %s on '%s': the reader of node '%s' "
        "runs best effort, the analysis saw %s. DDS may drop a sample, which then ends as a "
        "missing outcome; make the subscription reliable or analyze the effective QoS",
        id, e.spec.pub_key.c_str(), e.spec.sub_key.c_str(), e.resolved.c_str(),
        sub_node->get_fully_qualified_name(), seen.c_str());
    }
    // RCLLF-H002 on the depth the user asked for. The binding deepened
    // the reader, so this warns like the static H002 on an admitted edge.
    if (startup_h002(keep_last, user_depth, e.spec.sub_depth)) {
      RCLCPP_WARN(
        corr_logger(),
        "rcllf corr: RCLLF-H002 admitted edge %d %s -> %s on '%s': the reader of node '%s' "
        "asks for depth 1, the analysis saw %s. rcllf keeps %zu sample(s) in the reader, a "
        "sample overwritten beyond that ends as a missing outcome; deepen the subscription or "
        "analyze the effective QoS",
        id, e.spec.pub_key.c_str(), e.spec.sub_key.c_str(), e.resolved.c_str(),
        sub_node->get_fully_qualified_name(), analyzed_depth_text(e.spec.sub_depth).c_str(),
        actual.depth());
    }
    const std::string depth_text = (keep_last ?
      "reader depth " + std::to_string(actual.depth()) + ", user " + std::to_string(user_depth) :
      std::string("reader keep all")) +
      (deferred ? "" : relaxed ?
      ", reader best effort, user " + std::string(reliability_policy_text(user_reliability)) :
      actual.reliability() == rclcpp::ReliabilityPolicy::BestEffort ? ", reader best effort" :
      ", reader reliable");
    RCLCPP_INFO(
      corr_logger(),
      "rcllf corr: admitted %sedge %d %s -> %s on '%s' (d = %lld ns, T = %lld ns, L = %lld ns, "
      "B = %u, retain = %u, carrier = %u, %s, join %s)",
      ingress ? "ingress " : "", id, e.spec.pub_key.c_str(), e.spec.sub_key.c_str(),
      e.resolved.c_str(),
      static_cast<long long>(e.spec.delay_ns), static_cast<long long>(timeout_ns),  // NOLINT
      static_cast<long long>(st.table->max_lag_ns(static_cast<uint32_t>(id))),  // NOLINT
      e.spec.burst, e.spec.retain, e.spec.carrier, depth_text.c_str(),
      st.table->deferred(static_cast<uint32_t>(id)) ? "deferred" : "blocking");
    if (grouped) {
      RCLCPP_INFO(
        corr_logger(), "rcllf corr: edge %d has rank %u in the fan-in group of edge %d: one "
        "reader, one drain, one delivery order", id, e.spec.fan_in_rank,
        e.spec.group_lead);
    }
  }
  for (const auto & [id, sub] : group_readers) {
    std::vector<uint32_t> group;  // the reader serves every member
    for (size_t m = 0; m < edges.size(); ++m) {
      if (edges[m].spec.group_lead == id) {
        group.push_back(static_cast<uint32_t>(m));
      }
    }
    if (!IngressRole::instance().admit_corr_subscription(
        static_cast<uint32_t>(id), sub, std::move(group)))
    {
      return fail(
        id, "ingress", "ingress could not admit the reader of fan-in group " + std::to_string(id));
    }
  }
  if (!local_edges.empty()) {
    // A publish before its local reader matched the writer
    // leaves no sample to take, which ends local_undelivered (exit 3). So the
    // start waits for every local match, under the discovery budget.
    // As the cross-process warm-up (runtime.cpp): MATCH_WARMUP overrides DISCOVERY.
    const int64_t timeout_ms = match_warmup::timeout_ms_from_env(local_discovery_timeout_ms());
    const auto targets = match_warmup::local_edge_targets(local_edges);
    const auto outcomes = match_warmup::wait(targets, std::chrono::milliseconds(
          std::max<int64_t>(timeout_ms, 0)));
    if (!match_warmup::log_outcomes(outcomes, "local_edges", timeout_ms)) {
      return fail(
        first_local, "local_unmatched",
        match_warmup::unready_message(outcomes, "local_edges", timeout_ms));
    }
  }
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    st.edges = edges;
    st.started = true;
  }
  trace_corr(
    "{\"event\":\"corr\",\"kind\":\"startup\",\"run_epoch\":" +
    std::to_string(st.table->epoch()) + ",\"edges\":" + std::to_string(edges.size()) +
    ",\"legacy\":" + std::to_string(listed) + "}");
  RCLCPP_INFO(
    corr_logger(), "rcllf corr: run_epoch %llu, %zu admitted edge(s), %zu legacy edge(s)",
    static_cast<unsigned long long>(st.table->epoch()), edges.size(),  // NOLINT(runtime/int)
    listed);
  print_pending_warnings();
  return std::nullopt;
}

void startup_or_exit(const NodeResolver & resolve)
{
  auto f = startup(resolve);
  if (!f) {
    return;
  }
  State & st = state();
  std::string who = "edge " + std::to_string(f->edge);
  if (f->edge >= 0 && static_cast<size_t>(f->edge) < st.edges.size()) {
    const auto & s = st.edges[f->edge].spec;
    who += " (" + s.pub_key + " -> " + s.sub_key + " on '" + s.sub_topic + "')";
  }
  RCLCPP_ERROR(
    corr_logger(), "rcllf corr: startup refused admitted %s: check %s: %s. An admitted edge "
    "never falls back to the legacy path", who.c_str(), f->check.c_str(),
    f->detail.c_str());
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(1);
}

std::optional<std::array<uint8_t, 16>> read_boot_id(const std::string & path)
{
  // Fail closed: a missing, short, long or non-hex id gives
  // nothing, never the all-zero id two such hosts would share.
  std::ifstream f(path);
  std::string text;
  if (!f || !std::getline(f, text)) {
    return std::nullopt;
  }
  std::array<uint8_t, 16> b{};
  size_t digits = 0;
  for (const char c : text) {
    if (c == '-') {
      continue;
    }
    int v = -1;
    if (c >= '0' && c <= '9') {
      v = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      v = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      v = c - 'A' + 10;
    }
    if (v < 0 || digits >= 2 * b.size()) {
      return std::nullopt;
    }
    b[digits / 2] = static_cast<uint8_t>((b[digits / 2] << 4) | v);
    ++digits;
  }
  if (digits != 2 * b.size()) {
    return std::nullopt;
  }
  return b;
}

std::string xfed_clock_problem(const std::string & boot_id_path, bool synchronized)
{
  if (synchronized || read_boot_id(boot_id_path)) {
    return "";
  }
  return "no 16-byte boot id in '" + boot_id_path + "', so the one-clock check cannot tell "
         "hosts apart. Declare the bound on the clock offset of the hosts with "
         "ROS2_LF_XFED_CLOCK_BOUND_NS";
}

XfedClockBound xfed_clock_bound_from(const char * bound, const char * synchronized)
{
  XfedClockBound b;
  if (bound == nullptr) {
    // A declared synchronization with no number is the zero bound.
    b.declared = synchronized != nullptr && std::strcmp(synchronized, "1") == 0;
    return b;
  }
  const char * end = bound + std::strlen(bound);
  int64_t v = -1;
  const auto r = std::from_chars(bound, end, v);
  if (bound == end || !std::isdigit(static_cast<unsigned char>(*bound)) ||
    r.ec != std::errc() || r.ptr != end)
  {
    b.problem = "ROS2_LF_XFED_CLOCK_BOUND_NS='" + std::string(bound) +
      "' is not a decimal number of nanoseconds";
    return b;
  }
  b.declared = true;
  b.eps_ns = v;
  return b;
}

XfedClockBound xfed_clock_bound()
{
  return xfed_clock_bound_from(
    std::getenv("ROS2_LF_XFED_CLOCK_BOUND_NS"), std::getenv("ROS2_LF_XFED_CLOCK_SYNCHRONIZED"));
}

std::string xfed_host_problem(bool same_boot, const XfedClockBound & bound)
{
  if (same_boot || bound.declared) {
    return "";
  }
  return "publisher runs on another host (boot id differs), the cutoff needs one clock or a "
         "declared bound on the offset of the two clocks. Set ROS2_LF_XFED_CLOCK_BOUND_NS to "
         "that bound in ns, as measured by chrony or PTP";
}

bool xfed_record_beyond_bound(int64_t phys, int64_t now, int64_t eps_ns)
{
  return phys > now && phys - now > eps_ns;
}

bool xfed_same_boot(const uint8_t * peer, const std::optional<std::array<uint8_t, 16>> & host)
{
  return host.has_value() && std::memcmp(peer, host->data(), host->size()) == 0;
}

int64_t xfed_judge_bound(
  corr::Table & t, uint32_t edge, bool same_boot, const XfedClockBound & bound)
{
  const int64_t eps = same_boot ? 0 : bound.eps_ns;
  t.set_clock_bound(edge, eps);
  return eps;
}

std::string xfed_bound_problem(int64_t eps_ns, int64_t timeout_ns)
{
  if (timeout_ns < 0 || eps_ns < timeout_ns) {
    return "";
  }
  return "the declared clock bound " + std::to_string(eps_ns) + " ns is not below T = " +
         std::to_string(timeout_ns) + " ns, so no sample is guaranteed on time";
}

bool release_tag(int64_t now, int64_t delay, int64_t extra, int64_t * wait, int64_t * release)
{
  int64_t w = 0;
  int64_t r = 0;
  if (__builtin_add_overflow(delay, extra, &w) || __builtin_add_overflow(now, w, &r)) {
    return false;
  }
  *wait = w;
  *release = r;
  return true;
}

void release_overflow(int edge, int64_t now)
{
  RCLCPP_ERROR(
    corr_logger(), "rcllf corr: edge %d release tag past %lld ns overflows int64, the arrive "
    "is refused and its records end missing", edge, static_cast<long long>(now));  // NOLINT
}

uint32_t LegacyGuard::note(const std::string & port, int64_t time, uint32_t microstep)
{
  Seen & s = seen_[port];
  if (s.count == 0 || s.time != time) {
    s = Seen{time, microstep, 1};
  } else if (s.last_micro != microstep) {
    s.last_micro = microstep;
    ++s.count;
  }
  return s.count;
}

void pack_xcorr(const std::vector<corr::XRecord> & records, rcllf_xcorr_t * out)
{
  out->count = static_cast<uint32_t>(records.size());
  for (size_t i = 0; i < records.size(); ++i) {
    std::memcpy(out->rec[i].gid, records[i].gid.data(), sizeof(out->rec[i].gid));
    out->rec[i].ordinal = records[i].ordinal;
    out->rec[i].phys = records[i].phys;
    out->rec[i].act_phys = records[i].act;
  }
}

std::vector<corr::XRecord> unpack_xcorr(const rcllf_xcorr_t & x)
{
  std::vector<corr::XRecord> recs(x.count);
  for (uint32_t i = 0; i < x.count; ++i) {
    std::memcpy(recs[i].gid.data(), x.rec[i].gid, recs[i].gid.size());
    recs[i].ordinal = x.rec[i].ordinal;
    recs[i].phys = x.rec[i].phys;
    recs[i].act = x.rec[i].act_phys;
  }
  return recs;
}

namespace
{
/// The endpoints of node `peer_fqn` among `peers`.
std::vector<const PeerEndpoint *> of_peer(
  const std::vector<PeerEndpoint> & peers, const std::string & peer_fqn)
{
  std::vector<const PeerEndpoint *> mine;
  for (const auto & p : peers) {
    if (p.node_fqn == peer_fqn) {
      mine.push_back(&p);
    }
  }
  return mine;
}

/// External endpoints never count: a match count alone is not the peer (R1-3).
std::string peer_count_problem(
  size_t mine, const std::vector<PeerEndpoint> & peers, const std::string & peer_fqn)
{
  return std::to_string(mine) + " endpoint(s) of the peer node '" + peer_fqn + "' among " +
         std::to_string(peers.size()) + " on the topic";
}
}  // namespace

std::string xfed_peer_problem(
  const std::vector<PeerEndpoint> & peers, const PeerEndpoint & local, const std::string & peer_fqn,
  bool local_is_publisher)
{
  const auto mine = of_peer(peers, peer_fqn);
  if (mine.size() != 1) {
    return peer_count_problem(mine.size(), peers, peer_fqn);
  }
  const PeerEndpoint & p = *mine.front();
  if (p.type != local.type) {
    return "the peer endpoint has type '" + p.type + "', not '" + local.type + "'";
  }
  rmw_qos_compatibility_type_t compat = RMW_QOS_COMPATIBILITY_OK;
  char reason[256] = {0};
  const rmw_qos_profile_t & pub = local_is_publisher ? local.qos : p.qos;
  const rmw_qos_profile_t & sub = local_is_publisher ? p.qos : local.qos;
  if (rmw_qos_profile_check_compatible(pub, sub, &compat, reason, sizeof(reason)) != RMW_RET_OK ||
    compat == RMW_QOS_COMPATIBILITY_ERROR)
  {
    return std::string("the peer endpoint's QoS is incompatible: ") + reason;
  }
  return "";
}

namespace
{

std::vector<PeerEndpoint> endpoints_on(
  const rcl_node_t * node, const std::string & topic,
  bool pubs)
{
  std::vector<PeerEndpoint> out;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_topic_endpoint_info_array_t info = rmw_get_zero_initialized_topic_endpoint_info_array();
  const rcl_ret_t ret = pubs ?
    rcl_get_publishers_info_by_topic(node, &alloc, topic.c_str(), false, &info) :
    rcl_get_subscriptions_info_by_topic(node, &alloc, topic.c_str(), false, &info);
  if (ret != RCL_RET_OK) {
    rcl_reset_error();
    return out;
  }
  for (size_t i = 0; i < info.size; ++i) {
    const auto & x = info.info_array[i];
    std::string ns = x.node_namespace ? x.node_namespace : "";
    while (!ns.empty() && ns.back() == '/') {
      ns.pop_back();
    }
    out.push_back({ns + "/" + (x.node_name ? x.node_name : ""), x.topic_type ? x.topic_type : "",
        x.qos_profile});
  }
  if (rmw_topic_endpoint_info_array_fini(&info, &alloc) != RMW_RET_OK) {
    rcl_reset_error();
  }
  return out;
}

}  // namespace

std::string xfed_bind_peer_problem(
  const rcl_node_t * node, const std::string & topic, const std::string & peer_fqn, bool egress)
{
  const auto peers = endpoints_on(node, topic, !egress);
  if (of_peer(peers, peer_fqn).empty()) {
    return "";
  }
  const char * fqn = rcl_node_get_fully_qualified_name(node);
  const std::string self = fqn ? fqn : "";
  std::vector<PeerEndpoint> local;
  for (auto & x : endpoints_on(node, topic, egress)) {
    if (x.node_fqn == self) {
      local.push_back(x);
    }
  }
  if (local.size() != 1) {
    return std::to_string(local.size()) + " local endpoint(s) of '" + self + "'";
  }
  return xfed_peer_problem(peers, local.front(), peer_fqn, egress);
}

std::vector<size_t> publisher_match_counts(const std::string & resolved_topic)
{
  State & st = state();
  std::vector<size_t> out;
  // reg_mu is held across the counts: rcllf_gate_on_fini takes it to drop a
  // slot before the real rcl_publisher_fini runs, so no key here is freed.
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (const auto & kv : st.slots) {
    if (kv.second->topic != resolved_topic) {
      continue;
    }
    size_t n = 0;
    if (rcl_publisher_get_subscription_count(
        static_cast<const rcl_publisher_t *>(kv.first), &n) != RCL_RET_OK)
    {
      rcl_reset_error();
      n = 0;
    }
    out.push_back(n);
  }
  return out;
}

void xfed_peer_check_or_exit(int timeout_ms)
{
  State & st = state();
  std::vector<EdgeRt> edges;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    edges = st.edges;
  }
  // A cross-federate edge needs this host's boot id for the
  // one-clock check, unless the hosts are declared synchronized.
  // Or a declared bound on the clock offset of the hosts.
  const XfedClockBound bound = xfed_clock_bound();
  const std::string clock = !bound.problem.empty() ? bound.problem :
    xfed_clock_problem(kBootIdPath, bound.declared);
  for (const auto & e : edges) {
    if (e.spec.role == CorrEdgeRole::kLocal || clock.empty()) {
      continue;
    }
    RCLCPP_FATAL(
      corr_logger(),
      "rcllf corr: startup refused %s edge %d (global %d) on '%s': check %s: %s",
      e.spec.role == CorrEdgeRole::kEgress ? "egress" : "ingress", e.spec.edge_id,
      e.spec.global_id, e.resolved.c_str(), bound.problem.empty() ? "boot_id" : "clock_bound",
      clock.c_str());
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(3);
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(std::max(timeout_ms, 0));
  for (const auto & e : edges) {
    if (e.spec.role == CorrEdgeRole::kLocal) {
      continue;
    }
    const bool egress = e.spec.role == CorrEdgeRole::kEgress;
    // An edge whose node is gone would start with no checked peer. Refuse it
    // like the boot id check above.
    const auto node_base = e.node.lock();
    if (!node_base) {
      RCLCPP_FATAL(
        corr_logger(),
        "rcllf corr: startup refused %s edge %d (global %d) on '%s': check peer: the node of "
        "'%s' is gone before the peer check",
        egress ? "egress" : "ingress", e.spec.edge_id, e.spec.global_id, e.resolved.c_str(),
        egress ? e.spec.pub_key.c_str() : e.spec.sub_key.c_str());
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(3);
    }
    const rcl_node_t * node = node_base->get_rcl_node_handle();
    const std::string self = node_base->get_fully_qualified_name();
    // A lifecycle end made in a hook (a publisher or a reader) has no
    // endpoint before tag 0: its hook runs at the startup reaction or later.
    // Without it only the peer's count is checked here; type and QoS are
    // compared when the end is bound (xfed_bind_peer_problem). A peer that is
    // a hook-made end too may be absent: a record no reader matched ends a
    // typed missing outcome, as a discovery gap does in-process.
    const bool local_rebind = egress ? e.spec.pub_rebind : e.spec.sub_rebind;
    const bool peer_rebind = egress ? e.spec.sub_rebind : e.spec.pub_rebind;
    std::string problem, deferred;
    while (true) {
      std::vector<PeerEndpoint> local;
      for (auto & x : endpoints_on(node, e.resolved, egress)) {
        if (x.node_fqn == self) {
          local.push_back(x);
        }
      }
      const auto peers = endpoints_on(node, e.resolved, !egress);
      const size_t n_peer = of_peer(peers, e.spec.peer_node).size();
      const bool peer_absent = n_peer == 0;
      if (local.empty() && local_rebind) {
        if (n_peer == 1 || (peer_absent && peer_rebind)) {
          problem.clear();
          deferred = "its local " + std::string(egress ? "publisher" : "reader") +
            " is made in a lifecycle hook";
        } else {
          deferred.clear();
          problem = peer_count_problem(n_peer, peers, e.spec.peer_node);
        }
      } else if (local.size() != 1) {
        problem = std::to_string(local.size()) + " local endpoint(s) of '" + self + "'";
      } else if (peer_absent && peer_rebind) {
        problem.clear();
        deferred = "the peer's " + std::string(egress ? "reader" : "publisher") +
          " is made in a lifecycle hook";
      } else {
        problem = xfed_peer_problem(peers, local.front(), e.spec.peer_node, egress);
      }
      if (problem.empty() || std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!deferred.empty()) {
      RCLCPP_INFO(
        corr_logger(), "rcllf corr: %s edge %d skips its peer check on '%s': %s, so it is "
        "compared with its peer when it is bound and a record it misses ends missing",
        egress ? "egress" : "ingress", e.spec.edge_id, e.resolved.c_str(), deferred.c_str());
      continue;
    }
    if (!problem.empty()) {
      RCLCPP_FATAL(
        corr_logger(),
        "rcllf corr: startup refused %s edge %d (global %d) on '%s': check peer_check: %s "
        "after %d ms", egress ? "egress" : "ingress", e.spec.edge_id,
        e.spec.global_id, e.resolved.c_str(), problem.c_str(), timeout_ms);
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(3);
    }
    RCLCPP_INFO(
      corr_logger(), "rcllf corr: %s edge %d found its peer '%s' on '%s'",
      egress ? "egress" : "ingress", e.spec.edge_id, e.spec.peer_node.c_str(),
      e.resolved.c_str());
  }
}

// Fault hooks for the injection oracle, keyed by (edge, ordinal).
// Compiled only with -DRCLLF_CORR_TEST_HOOKS=ON. Each hook appends its own
// actions to RCLLF_TEST_CORR_HOOK_LOG (JSON lines).
//   RCLLF_TEST_CORR_DROP=<edge>:<ord>[,<ord>...][;...]       lose the sample at ingress
//   RCLLF_TEST_CORR_DELAY=<edge>:<ord>[,...]@<ms>[;...]      hand it to the table <ms> later
//   RCLLF_TEST_CORR_SKIP_PREDICT=<edge>:<ord>[,...][;...]    predict no expectation for it
//   RCLLF_TEST_XCORR_DROP_COMPLETE=<edge>:<seq>[,...][;...]  egress: do not send the COMPLETE
//                                    message number <seq> of <edge>
// The older hooks RCLLF_TEST_CORR_DROP_TOKEN, _DROP_SAMPLE and _DRAIN_DELAY
// (read in startup()) are compiled only with the option too.
#ifndef RCLLF_CORR_TEST_HOOKS
void announce_test_hooks()
{
  for (const char * var :
    {"RCLLF_TEST_CORR_DROP", "RCLLF_TEST_CORR_DELAY", "RCLLF_TEST_CORR_SKIP_PREDICT",
      "RCLLF_TEST_XCORR_DROP_COMPLETE", "RCLLF_TEST_CORR_DROP_TOKEN",
      "RCLLF_TEST_CORR_DROP_SAMPLE", "RCLLF_TEST_CORR_DRAIN_DELAY"})
  {
    const char * v = std::getenv(var);
    if (v != nullptr && *v != '\0') {
      RCLCPP_ERROR(
        corr_logger(), "rcllf corr: %s is set but the test hooks are compiled out "
        "(build ros2_lf_binding with -DRCLLF_CORR_TEST_HOOKS=ON): no fault is injected", var);
    }
  }
}

bool drop_complete_for_test(uint32_t, uint64_t)
{
  return false;
}
#else
namespace
{
using HookKeys = std::map<std::pair<uint32_t, uint64_t>, int64_t>;

HookKeys parse_hook_keys(const char * var)
{
  HookKeys out;
  const char * env = std::getenv(var);
  std::string rest = env != nullptr ? env : "";
  while (!rest.empty()) {
    const size_t semi = rest.find(';');
    std::string rule = rest.substr(0, semi);
    rest = semi == std::string::npos ? "" : rest.substr(semi + 1);
    int64_t ms = 0;
    const size_t at = rule.find('@');
    if (at != std::string::npos) {
      ms = std::strtoll(rule.c_str() + at + 1, nullptr, 10);
      rule = rule.substr(0, at);
    }
    const size_t colon = rule.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto edge = static_cast<uint32_t>(std::strtoul(rule.c_str(), nullptr, 10));
    std::string ords = rule.substr(colon + 1);
    size_t pos = 0;
    while (pos < ords.size()) {
      const size_t comma = ords.find(',', pos);
      const std::string one = ords.substr(pos, comma == std::string::npos ? comma : comma - pos);
      if (!one.empty()) {
        out[{edge, std::strtoull(one.c_str(), nullptr, 10)}] = ms;
      }
      pos = comma == std::string::npos ? ords.size() : comma + 1;
    }
  }
  return out;
}

struct TestHooks
{
  HookKeys drop, delay, skip;
  std::string log;
  bool any = false;
};

const TestHooks & test_hooks()
{
  static const TestHooks h = [] {
      TestHooks t;
      t.drop = parse_hook_keys("RCLLF_TEST_CORR_DROP");
      t.delay = parse_hook_keys("RCLLF_TEST_CORR_DELAY");
      t.skip = parse_hook_keys("RCLLF_TEST_CORR_SKIP_PREDICT");
      const char * log = std::getenv("RCLLF_TEST_CORR_HOOK_LOG");
      t.log = log != nullptr ? log : "";
      t.any = !t.drop.empty() || !t.delay.empty() || !t.skip.empty();
      return t;
    }();
  return h;
}

void hook_log(const char * what, uint32_t edge, const Gid & gid, uint64_t ordinal, int64_t ms)
{
  const auto & h = test_hooks();
  if (h.log.empty()) {
    return;
  }
  static std::mutex mu;
  std::lock_guard<std::mutex> lk(mu);
  std::ofstream f(h.log, std::ios::app);
  f << "{\"hook\":\"" << what << "\",\"edge\":" << edge << ",\"gid\":\"" << corr::gid_hex(gid) <<
    "\",\"ordinal\":" << ordinal << ",\"delay_ms\":" << ms << ",\"physical_ns\":" <<
    lf_time_physical() << "}\n";
}

thread_local bool t_redelivery = false;
}  // namespace

/// True when a test hook took the sample (dropped, or queued for a delayed delivery).
bool test_hook_intercept(
  uint32_t edge, const Gid & gid, uint64_t ordinal, const std::shared_ptr<void> & message,
  const rclcpp::MessageInfo & info)
{
  const auto & h = test_hooks();
  if (!h.any || t_redelivery) {
    return false;
  }
  if (h.drop.count({edge, ordinal}) != 0) {
    hook_log("drop", edge, gid, ordinal, 0);
    return true;
  }
  auto it = h.delay.find({edge, ordinal});
  if (it == h.delay.end()) {
    return false;
  }
  const int64_t ms = it->second;
  hook_log("delay", edge, gid, ordinal, ms);
  std::thread(
    [edge, message, info, ms]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      t_redelivery = true;
      on_sample(edge, message, info);
    }).detach();
  return true;
}

/// Removes every edge whose (edge, ordinal) key is listed in RCLLF_TEST_CORR_SKIP_PREDICT.
void test_skip_predict(const Gid & gid, uint64_t ordinal, std::vector<uint32_t> * edges)
{
  const auto & h = test_hooks();
  if (h.skip.empty()) {
    return;
  }
  for (auto it = edges->begin(); it != edges->end(); ) {
    if (h.skip.count({*it, ordinal}) != 0) {
      hook_log("skip_predict", *it, gid, ordinal, 0);
      it = edges->erase(it);
    } else {
      ++it;
    }
  }
}

void announce_test_hooks()
{
  const auto & h = test_hooks();
  std::string older;  // the older hooks set in this run
  for (const char * var :
    {"RCLLF_TEST_CORR_DROP_TOKEN", "RCLLF_TEST_CORR_DROP_SAMPLE", "RCLLF_TEST_CORR_DRAIN_DELAY"})
  {
    const char * v = std::getenv(var);
    if (v != nullptr && *v != '\0') {
      older += std::string(older.empty() ? "" : ", ") + var;
    }
  }
  if (!h.any && older.empty()) {
    return;
  }
  RCLCPP_WARN(
    corr_logger(), "rcllf corr: TEST FAULT HOOKS ACTIVE: %zu drop, %zu delay, %zu skip_predict "
    "key(s)%s%s. This run injects faults on purpose and is not a measurement run.",
    h.drop.size(), h.delay.size(), h.skip.size(), older.empty() ? "" : "; set: ",
    older.c_str());
}

bool drop_complete_for_test(uint32_t edge, uint64_t seq)
{
  static const HookKeys keys = parse_hook_keys("RCLLF_TEST_XCORR_DROP_COMPLETE");
  if (keys.count({edge, seq}) == 0) {
    return false;
  }
  RCLCPP_WARN(
    corr_logger(), "rcllf corr: TEST FAULT HOOK: COMPLETE %llu of egress edge %u not sent",
    static_cast<unsigned long long>(seq), edge);  // NOLINT(runtime/int)
  return true;
}
#endif  // RCLLF_CORR_TEST_HOOKS

namespace
{
thread_local bool t_defer_warnings = false;
/// Set by the publish-time take: the edges whose sample a test hook took.
thread_local std::set<uint32_t> * t_planted = nullptr;
std::atomic<bool> g_local_delivery_lock{true};
std::mutex g_local_skip_mu;
std::set<uint32_t> g_local_skip;
std::atomic<bool> g_local_skip_any{false};
std::mutex g_after_publish_mu;
std::function<void()> g_after_publish_hook;
std::atomic<bool> g_after_publish_any{false};

void flush_warnings()
{
  if (!t_defer_warnings) {
    print_pending_warnings();
  }
}

bool local_skip(uint32_t edge)
{
  if (!g_local_skip_any.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lk(g_local_skip_mu);
  return g_local_skip.count(edge) != 0;
}
}  // namespace

WarningsDeferred::WarningsDeferred()
: prev_(t_defer_warnings)
{
  t_defer_warnings = true;
}

WarningsDeferred::~WarningsDeferred()
{
  t_defer_warnings = prev_;
}

void set_local_delivery_lock_for_test(bool on)
{
  g_local_delivery_lock.store(on, std::memory_order_release);
}

void set_local_take_skip_for_test(std::set<uint32_t> edges)
{
  std::lock_guard<std::mutex> lk(g_local_skip_mu);
  g_local_skip = std::move(edges);
  g_local_skip_any.store(!g_local_skip.empty(), std::memory_order_release);
}

void set_local_after_publish_hook_for_test(std::function<void()> hook)
{
  std::lock_guard<std::mutex> lk(g_after_publish_mu);
  g_after_publish_hook = std::move(hook);
  g_after_publish_any.store(static_cast<bool>(g_after_publish_hook), std::memory_order_release);
}

/// Runs and clears the one-shot hook of set_local_after_publish_hook_for_test.
void run_after_publish_hook()
{
  if (!g_after_publish_any.load(std::memory_order_acquire)) {
    return;
  }
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lk(g_after_publish_mu);
    hook.swap(g_after_publish_hook);
    g_after_publish_any.store(false, std::memory_order_release);
  }
  if (hook) {
    hook();
  }
}

bool local_delivery_lock()
{
  return g_local_delivery_lock.load(std::memory_order_acquire);
}

/// The publish-time take of `edges` with `held` locked, then
/// Table::settle_local. Returns the expectations it settled as LOST.
size_t take_and_settle_local(
  const Gid & gid, uint64_t ordinal, IngressRole::LocalTake & held)
{
  std::set<uint32_t> planted;
  t_planted = &planted;
  IngressRole::instance().take_local_held(
    held, [&gid, ordinal](uint32_t reader_edge) {
      return !state().table->local_open(gid, ordinal, reader_edge);
    });
  t_planted = nullptr;
  return state().table->settle_local(gid, ordinal, planted);
}

void on_sample(
  uint32_t edge, std::shared_ptr<void> message, const rclcpp::MessageInfo & info,
  std::optional<int64_t> reader_created)
{
  State & st = state();
  const rmw_message_info_t & mi = info.get_rmw_message_info();
  Gid gid{};
  std::memcpy(gid.data(), mi.publisher_gid.data, std::min(sizeof(mi.publisher_gid.data),
    gid.size()));
  const uint64_t ordinal = mi.publication_sequence_number;
#ifdef RCLLF_CORR_TEST_HOOKS
  // A fan-in group's one reader is admitted on its lead, `edge`. The S4
  // hooks act before the table takes the sample, so they key on the member
  // the table knows for the writer now; an early sample of an ingress group,
  // whose writer no record named yet, keys on the lead.
  if (test_hook_intercept(st.table->sample_edge(edge, gid), gid, ordinal, message, info)) {
    if (t_planted != nullptr) {
      t_planted->insert(st.table->sample_edge(edge, gid));  // planted, not a defect
    }
    return;  // test hook: dropped or delayed
  }
#endif
  if (t_planted != nullptr && local_skip(st.table->sample_edge(edge, gid))) {
    return;  // test seam: a real undelivered sample, never planted
  }
  auto sample = corr::Sample{std::move(message), std::make_shared<rclcpp::MessageInfo>(info)};
  uint64_t retained_id = 0;
  // The table resolves a group's sample by writer GID to
  // its member, `at`, counts it there and applies the F3 hook to it.
  uint32_t at = edge;
  bool dropped = false;
  const auto o = st.table->on_group_sample(
    edge, gid, ordinal, std::move(sample), &at, &retained_id, nullptr, reader_created, &dropped);
  if (dropped) {
    if (t_planted != nullptr) {
      t_planted->insert(at);  // planted, not a defect
    }
    return;  // test hook F3: the sample is lost in transport
  }
  if (o == corr::SampleOutcome::POOLED) {
    // No member knows the writer yet: the sample is traced when it leaves
    // the group's early pool (TraceObserver::on_pool_exit).
    flush_warnings();
    return;
  }
  {
    rcllf_tp_sample ev{};
    ev.edge = at;
    fill_gid(ev.gid, gid);
    ev.ordinal = ordinal;
    ev.outcome = static_cast<uint8_t>(o);
    ev.retained_id = retained_id;
    ev.arrival_physical_ns = lf_time_physical();
    semtrace::sample(ev);
  }
  if (o == corr::SampleOutcome::UNRESOLVED) {
    static std::mutex mu;
    static std::set<Gid, corr::GidLess> warned;
    bool first = false;
    {
      std::lock_guard<std::mutex> lk(mu);
      first = warned.insert(gid).second;
    }
    if (first) {
      RCLCPP_WARN(
        corr_logger(), "rcllf corr: edge %u took a sample from writer %s, unknown to this "
        "process (external writer, another process or a previous run): released as unresolved, "
        "never dispatched", at, corr::gid_hex(gid).c_str());
    }
  }
  trace_corr(
    "{\"event\":\"corr\",\"kind\":\"sample\",\"edge\":" + std::to_string(at) + ",\"gid\":\"" +
    corr::gid_hex(gid) + "\",\"ordinal\":" + std::to_string(ordinal) + ",\"outcome\":\"" +
    corr::sample_outcome_name(o) + "\"}");
  flush_warnings();
}

void print_pending_warnings()
{
  State & st = state();
  std::vector<std::shared_ptr<Slot>> slots;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (auto & kv : st.slots) {
      if (kv.second->warn_pending.load(std::memory_order_relaxed)) {
        slots.push_back(kv.second);
      }
    }
  }
  {
    std::vector<std::shared_ptr<Slot>> refused;
    {
      std::lock_guard<std::mutex> lk(st.reg_mu);
      for (auto & kv : st.slots) {
        if (kv.second->refusal_pending.load(std::memory_order_relaxed) != 0) {
          refused.push_back(kv.second);
        }
      }
    }
    for (auto & s : refused) {
      const uint32_t bits = s->refusal_pending.exchange(0);
      for (Origin o : {Origin::UNDECLARED_EFFECT, Origin::UNSUPPORTED_API}) {
        const uint32_t bit = 1u << static_cast<unsigned>(o);
        if ((bits & bit) != 0 && (s->refusal_warned & bit) == 0) {
          s->refusal_warned |= bit;
          RCLCPP_WARN(
            corr_logger(), "rcllf corr: %s published on '%s' with origin %s: no expectation, "
            "its samples are released as unsupported", s->node_fqn.c_str(),
            s->topic.c_str(), corr::origin_name(o));
        }
      }
    }
  }
  for (auto & s : slots) {
    if (s->warn_pending.exchange(false) && !s->warned) {
      s->warned = true;
      RCLCPP_WARN(
        corr_logger(), "rcllf corr: %s published on '%s' outside an LF reaction (no_context): "
        "that sample does not reach the local LF subscriber", s->node_fqn.c_str(),
        s->topic.c_str());
    }
  }
}

namespace
{
/// How bind_reader treats a reader whose predecessor still lives.
enum class Settle {kBind, kRetry, kFinal};
void bind_reader(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub, Settle mode);
std::vector<std::pair<int, rclcpp::SubscriptionBase::SharedPtr>> deferred_readers(int rid);
}  // namespace

void note_stop_requested(int64_t bound_ms)
{
  State & st = state();
  if (!st.table) {
    return;
  }
  const auto pending = st.table->note_stop_requested(bound_ms * 1000000);
  std::string listed;
  for (size_t i = 0; i < pending.size() && i < 8; ++i) {
    listed += std::string(i ? "; " : ": ") + "edge " + std::to_string(pending[i].edge) +
      " gid " + pending[i].gid + " ordinal " + std::to_string(pending[i].ordinal) +
      " tag " + std::to_string(pending[i].pub_tag);
  }
  RCLCPP_INFO(
    corr_logger(),
    "rcllf corr: LF stop requested with %zu join(s) pending; each ends within %lld ms, "
    "pending at stop if its sample is still in flight%s",
    pending.size(), static_cast<long long>(bound_ms), listed.c_str());  // NOLINT(runtime/int)
}

namespace
{
/// One pass over the admitted readers on the calling thread: take what
/// each holds, up to `max_per_reader`, into the table. No lock of the ingress
/// thread's is needed: it has stopped before the drain (and would hold the
/// same subscription's take lock otherwise).
size_t take_in_flight_once(size_t max_per_reader)
{
  State & st = state();
  struct Reader
  {
    uint32_t edge;
    rclcpp::SubscriptionBase::SharedPtr sub;
    std::optional<int64_t> created;
  };
  std::vector<Reader> readers;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (const auto & e : st.edges) {
      // A fan-in group's one reader is admitted on its lead and serves
      // every member; a member edge has no reader of its own to take from.
      if (e.spec.role == CorrEdgeRole::kEgress ||
        (e.spec.group_lead >= 0 && e.spec.group_lead != e.spec.edge_id))
      {
        continue;
      }
      const uint32_t edge = static_cast<uint32_t>(e.spec.edge_id);
      if (e.sub) {
        readers.push_back({edge, e.sub, std::nullopt});
        continue;
      }
      // A sub_rebind edge's reader, with the creation tag its take carries.
      if (auto reader = e.reader.lock()) {
        std::optional<int64_t> created;
        auto it = st.sub_created.find(reader->get_subscription_handle().get());
        if (it != st.sub_created.end() && it->second) {
          created = *it->second;
        }
        readers.push_back({edge, std::move(reader), created});
      }
    }
  }
  size_t taken_total = 0;
  for (const auto & r : readers) {
    for (size_t n = 0; n < max_per_reader; ++n) {
      std::shared_ptr<void> msg = r.sub->create_message();
      rclcpp::MessageInfo info;
      bool taken = false;
      try {
        taken = r.sub->take_type_erased(msg.get(), info);
      } catch (const std::exception & e) {
        RCLCPP_WARN(corr_logger(), "rcllf corr: stop drain take failed: %s", e.what());
      }
      if (!taken) {
        break;
      }
      ++taken_total;
      on_sample(r.edge, std::move(msg), info, r.created);
    }
  }
  return taken_total;
}
}  // namespace

size_t drain_in_flight(int64_t bound_ms)
{
  State & st = state();
  if (!st.table) {
    return 0;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + std::chrono::milliseconds(bound_ms);
  // Nobody else takes now. The old wait on the table alone ended at its
  // bound with every sample published just below the stop tag still in the
  // readers (counted never arrived, not late). Poll the readers until nothing
  // is owed or the bound passes; a poll that took nothing waits 1 ms.
  constexpr int64_t kPollNs = 1000000;
  size_t taken = 0;
  size_t owed = st.table->drain_in_flight(0);
  while (owed > 0) {
    const size_t n = take_in_flight_once(IngressRole::kCorrDrainTakeMax);
    taken += n;
    const int64_t left = std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      owed = st.table->drain_in_flight(0);
      break;
    }
    owed = st.table->drain_in_flight(n > 0 ? 0 : std::min(kPollNs, left));
  }
  const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    corr_logger(),
    "rcllf corr: stop drain waited %lld ms (bound %lld ms) for samples in flight; "
    "%zu writer(s) still owe one; %zu sample(s) taken by the drain",
    static_cast<long long>(took), static_cast<long long>(bound_ms),  // NOLINT(runtime/int)
    owed, taken);
  return owed;
}

EgressDrain drain_egress(int64_t bound_ms)
{
  State & st = state();
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + std::chrono::milliseconds(bound_ms);
  EgressDrain r;
  // reg_mu is held across the waits: rcllf_gate_on_fini takes it to drop a
  // slot before the real rcl_publisher_fini runs, so no tracked key here is
  // freed. An untracked key is never dereferenced: rcl's rosout publisher is
  // initialized in a stack temporary and copied into rcl's map, so its key
  // dangles from the start and its fini never reaches the gate.
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (const auto & kv : st.slots) {
    bool eligible = false;
    {
      std::lock_guard<std::mutex> slk(kv.second->m);
      // nothing is acknowledged on a best-effort writer
      eligible = kv.second->tracked && !kv.second->best_effort && !kv.second->internal;
    }
    if (!eligible) {
      continue;
    }
    const auto * pub = static_cast<const rcl_publisher_t *>(kv.first);
    if (!rcl_publisher_is_valid(pub)) {
      rcl_reset_error();
      continue;
    }
    const auto left = std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    const rcl_ret_t ret = rcl_publisher_wait_for_all_acked(pub, std::max<int64_t>(0, left));
    ++r.waited;
    if (ret == RCL_RET_TIMEOUT) {
      ++r.unacked;
    } else if (ret != RCL_RET_OK) {
      rcl_reset_error();
    }
  }
  const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    corr_logger(),
    "rcllf corr: stop drain waited %lld ms (bound %lld ms) for %zu reliable publisher(s) "
    "to be acknowledged; %zu not acknowledged",
    static_cast<long long>(took), static_cast<long long>(bound_ms),  // NOLINT(runtime/int)
    r.waited, r.unacked);
  return r;
}

namespace
{
/// The end-of-run COMPLETE counts of the cross-
/// federate edges. A COMPLETE lost after the receiver's last one leaves no
/// sequence gap; only the sender knows how many it numbered. After LF returned
/// the sender publishes, per egress edge, how many land by the stop tag, and
/// the receiver's own reconcile compares them with what arrived. DDS is the
/// only channel left then (LF has stopped), and it reaches a peer on another
/// host too. Reliable and transient local, so a reader that matches late
/// still gets every count; KEEP_ALL, so no count of a process with several
/// egress edges replaces another.
struct EndChannel
{
  std::mutex mu;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<ros2_lf_msgs::msg::CorrEnd>::SharedPtr pub;
  rclcpp::Subscription<ros2_lf_msgs::msg::CorrEnd>::SharedPtr sub;
  std::map<int, uint32_t> ingress_of_global;  ///< global edge id -> table edge
  std::map<uint32_t, int> global_of_egress;   ///< table edge -> global edge id
};

EndChannel & end_channel()
{
  // Never destroyed, like state(). ros2_lf_runtime_shutdown runs as an exit
  // handler, and this channel is first built at the peer check, after that
  // handler was registered. A function-local static would get its destructor
  // registered later, so exit (LIFO) would destroy the subscription before the
  // shutdown takes from it: SIGSEGV in rcl_take on every federate with an
  // ingress edge (seen as crashes on a workstation). complete_end_release frees
  // the ROS entities while the context still exists.
  static EndChannel * c = new EndChannel();
  return *c;
}

rclcpp::QoS end_qos()
{
  return rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();
}
}  // namespace

void complete_end_channel_init()
{
  State & st = state();
  EndChannel & c = end_channel();
  std::lock_guard<std::mutex> lk(c.mu);
  if (c.node) {
    return;
  }
  {
    std::lock_guard<std::mutex> rlk(st.reg_mu);
    for (const auto & e : st.edges) {
      if (e.spec.role == CorrEdgeRole::kEgress) {
        c.global_of_egress[static_cast<uint32_t>(e.spec.edge_id)] = e.spec.global_id;
      } else if (e.spec.role == CorrEdgeRole::kIngress) {
        c.ingress_of_global[e.spec.global_id] = static_cast<uint32_t>(e.spec.edge_id);
      }
    }
  }
  if (c.global_of_egress.empty() && c.ingress_of_global.empty()) {
    return;
  }
  // One node per process; the epoch keeps two federates' names apart.
  c.node = rclcpp::Node::make_shared("rcllf_corr_end_" + std::to_string(st.table->epoch()));
  if (!c.global_of_egress.empty()) {
    c.pub = c.node->create_publisher<ros2_lf_msgs::msg::CorrEnd>(kCorrEndTopic, end_qos());
  }
  if (!c.ingress_of_global.empty()) {
    // Taken by polling at the close (complete_end_wait), never by an executor.
    c.sub = c.node->create_subscription<ros2_lf_msgs::msg::CorrEnd>(
      kCorrEndTopic, end_qos(), [](ros2_lf_msgs::msg::CorrEnd::ConstSharedPtr) {});
    for (const auto & kv : c.ingress_of_global) {
      st.table->expect_complete_end(kv.second);
    }
  }
}

namespace
{
/// After close_epoch: publish this process's counts.
void complete_end_publish()
{
  State & st = state();
  EndChannel & c = end_channel();
  std::lock_guard<std::mutex> lk(c.mu);
  if (!c.pub) {
    return;
  }
  // rclcpp drops a publish on a shut-down context without an error, so
  // "sent" would be claimed for a count no peer can get. Say so instead; each
  // peer then reports complete_end_missing for these edges.
  const bool live = c.node->get_node_base_interface()->get_context()->is_valid();
  for (const auto & end : st.table->complete_ends()) {
    const auto it = c.global_of_egress.find(end.edge);
    if (it == c.global_of_egress.end()) {
      continue;
    }
    ros2_lf_msgs::msg::CorrEnd m;
    m.sender_epoch = st.table->epoch();
    m.global_edge = it->second;
    m.completes_sent = end.sent;
    m.completes_due = end.due;
    if (!live) {
      RCLCPP_ERROR(
        corr_logger(),
        "rcllf corr: complete_end NOT sent edge %u (global %d): the ROS context was already "
        "shut down, so the peer cannot get this count", end.edge, it->second);
      continue;
    }
    c.pub->publish(m);
    RCLCPP_INFO(
      corr_logger(), "rcllf corr: complete_end sent edge %u (global %d): %llu due of %llu sent",
      end.edge, it->second, static_cast<unsigned long long>(end.due),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(end.sent));  // NOLINT(runtime/int)
  }
}

/// Take the peers' counts until every ingress edge has one or `bound_ms`
/// passed. One that never comes fails the reconcile (complete_end_missing).
void complete_end_wait(int64_t bound_ms)
{
  State & st = state();
  EndChannel & c = end_channel();
  std::lock_guard<std::mutex> lk(c.mu);
  if (!c.sub) {
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + std::chrono::milliseconds(std::max<int64_t>(0, bound_ms));
  // A poll that took nothing sleeps 1 ms: the wait ends within 1 ms of the
  // last count, and an idle wait costs at most bound_ms polls.
  constexpr auto kPoll = std::chrono::milliseconds(1);
  while (true) {
    ros2_lf_msgs::msg::CorrEnd m;
    rclcpp::MessageInfo info;
    bool took = false;
    while (c.sub->take(m, info)) {
      took = true;
      const auto it = c.ingress_of_global.find(m.global_edge);
      if (it == c.ingress_of_global.end()) {
        continue;  // an edge this process does not receive
      }
      st.table->note_complete_end(corr::Table::CompleteEnd{it->second, m.completes_due,
          m.completes_sent});
      RCLCPP_INFO(
        corr_logger(),
        "rcllf corr: complete_end received edge %u (global %d) from epoch %llu: %llu due",
        it->second, m.global_edge,
        static_cast<unsigned long long>(m.sender_epoch),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(m.completes_due));  // NOLINT(runtime/int)
    }
    if (st.table->complete_ends_missing().empty() ||
      std::chrono::steady_clock::now() >= deadline)
    {
      break;
    }
    if (!took) {
      std::this_thread::sleep_for(kPoll);
    }
  }
  const auto took_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    corr_logger(),
    "rcllf corr: complete_end waited %lld ms (bound %lld ms), %zu edge(s) without a count",
    static_cast<long long>(took_ms), static_cast<long long>(bound_ms),  // NOLINT(runtime/int)
    st.table->complete_ends_missing().size());
}
}  // namespace

void complete_end_ack(int64_t bound_ms)
{
  EndChannel & c = end_channel();
  std::lock_guard<std::mutex> lk(c.mu);
  if (!c.pub) {
    return;
  }
  // The peer waits for this count; a process that exits before its readers
  // acknowledged it could take it with it.
  // On a shut-down context rcl refuses the wait and rclcpp throws
  // RCLError ("publisher's context is invalid"); uncaught in this exit
  // handler it aborted the process. The count is then lost, reported here.
  bool acked = false;
  const char * why = "";
  if (!c.node->get_node_base_interface()->get_context()->is_valid()) {
    why = " (the ROS context was already shut down)";
  } else {
    try {
      acked = c.pub->wait_for_all_acked(
        std::chrono::milliseconds(std::max<int64_t>(0, bound_ms)));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(corr_logger(), "rcllf corr: complete_end acknowledgement wait: %s", ex.what());
      why = " (the wait failed)";
    }
  }
  RCLCPP_INFO(
    corr_logger(), "rcllf corr: complete_end %s within %lld ms%s",
    acked ? "acknowledged" : "NOT acknowledged",
    static_cast<long long>(bound_ms), why);  // NOLINT(runtime/int)
}

void complete_end_release()
{
  EndChannel & c = end_channel();
  std::lock_guard<std::mutex> lk(c.mu);
  c.sub.reset();
  c.pub.reset();
  c.node.reset();
}

bool close_and_reconcile(int64_t end_wait_ms)
{
  State & st = state();
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    if (st.reconciled) {
      return st.reconcile_ok;
    }
    st.reconciled = true;
  }
  // A reader still deferred at close whose predecessor still
  // lives is a twin of it, refused. One whose predecessor is gone had no
  // record drained since (each drain retries), so nothing depends on it.
  for (const auto & d : deferred_readers(-1)) {
    bind_reader(d.first, d.second, Settle::kFinal);
  }
  const uint64_t released0 = st.table->released_retained();
  const uint64_t closed0 = st.table->epoch_close_terminals();
  // Egress records of the stop tag's time are retained at stop. Outside
  // an LF program (tests) no stop tag is known and nothing is cut. The
  // reconcile runs after LF returned, so the stop tag an edge close read at
  // the stop tag is used first.
  int64_t stop_time = -1;
  uint32_t stop_micro = 0;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    stop_time = st.close_stop_time;
    stop_micro = st.close_stop_micro;
  }
  if (stop_time < 0) {
    std::tie(stop_time, stop_micro) = lf_stop_tag();
  }
  st.table->close_epoch(
    stop_time >= 0 ? stop_time : std::numeric_limits<int64_t>::max(), stop_micro);
  st.closed.store(true);
  // One trace line per expectation the close ended
  // EPOCH_CLOSE, with its message time (release tag) and the stop tag, so a
  // checker counts it declared only when the run records both. A token in no
  // carrier has no known release tag: its line carries no release fields.
  {
    const int64_t start = lf_start_tag().first;
    for (const auto & ec : st.table->epoch_closed_entries()) {
      std::string rel;
      if (ec.release_known) {
        rel = ",\"release_elapsed\":" + std::to_string(ec.q.release_ns - start) +
          ",\"release_microstep\":" + std::to_string(ec.q.release_micro);
      }
      std::string stop;
      if (ec.stop_time >= 0 && ec.stop_time != std::numeric_limits<int64_t>::max()) {
        stop = ",\"stop_elapsed\":" + std::to_string(ec.stop_time - start) +
          ",\"stop_microstep\":" + std::to_string(ec.stop_micro);
      }
      trace_corr(
        "{\"event\":\"corr\",\"kind\":\"epoch_close\",\"edge\":" + std::to_string(ec.edge) +
        ",\"gid\":\"" + corr::gid_hex(ec.q.token.gid) + "\",\"ordinal\":" +
        std::to_string(ec.q.token.ordinal) + ",\"outcome\":\"EPOCH_CLOSE\"" + rel + stop + "}");
    }
  }
  // Exchange the end-of-run COMPLETE counts before
  // the reconcile, so a trailing CONTROL_FAILURE fails this process.
  complete_end_publish();
  complete_end_wait(end_wait_ms);
  {
    rcllf_tp_cleanup ev{};
    ev.released_retained = st.table->released_retained() - released0;
    ev.carrier_records_closed = st.table->epoch_close_terminals() - closed0;
    ev.reason = RCLLF_CLEANUP_EPOCH_CLOSE;
    semtrace::cleanup(ev);
  }
  // Untracked publishes per topic, read with the reconcile and
  // logged after it, so a run check can compare them with what it observed.
  std::map<std::string, uint64_t> untracked_by_topic;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (auto & kv : st.slots) {
      if (const uint64_t n = kv.second->c_untracked.load()) {
        untracked_by_topic[kv.second->topic] += n;
      }
    }
  }
  auto r = st.table->reconcile();
  st.reconcile_ok = r.ok;
  RCLCPP_INFO(corr_logger(), "rcllf corr: %s", r.summary.c_str());
  std::string by_topic = "{";
  for (const auto & kv : untracked_by_topic) {
    by_topic += (by_topic.size() > 1 ? ",\"" : "\"") + kv.first + "\":" + std::to_string(kv.second);
  }
  RCLCPP_INFO(corr_logger(), "rcllf corr: untracked_by_topic %s}", by_topic.c_str());
  for (const auto & p : r.problems) {
    RCLCPP_ERROR(corr_logger(), "rcllf corr: reconcile: %s", p.c_str());
  }
  if (!r.ok) {
    RCLCPP_ERROR(corr_logger(), "rcllf corr: invariant broken, the process exits nonzero");
  }
  return r.ok;
}

bool reconcile_failed()
{
  State & st = state();
  return st.reconciled && !st.reconcile_ok;
}

namespace
{
void clear_deferred_locked(State & st, EdgeRt & e)
{
  if (e.deferred_at) {
    st.deferred_readers.fetch_sub(1);
  }
  e.pending_reader.reset();
  e.deferred_at.reset();
}

void bind_reader(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub, Settle mode)
{
  if (!sub) {
    return;
  }
  State & st = state();
  struct Verdict
  {
    uint32_t edge = 0;
    uint32_t gen = 0;
    int64_t created = std::numeric_limits<int64_t>::min();
    std::string code, detail, what;
    bool deferred = false;
  };
  std::vector<Verdict> verdicts;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    if (!st.started || st.closed.load()) {
      return;
    }
    const void * handle = sub->get_subscription_handle().get();
    auto created = st.sub_created.find(handle);
    const std::pair<int64_t, uint32_t> now = lf_current_tag();
    for (auto & e : st.edges) {
      if (!e.spec.sub_rebind || e.spec.sub_rid != rid ||
        e.spec.role == CorrEdgeRole::kEgress || e.reader_refused)
      {
        continue;
      }
      if (!e.reader.expired() && e.reader.lock() == sub) {
        continue;  // this generation is admitted already
      }
      if (mode != Settle::kBind && !e.deferred_at) {
        continue;  // settled since
      }
      Verdict v;
      v.edge = static_cast<uint32_t>(e.spec.edge_id);
      v.what = "(" + e.spec.pub_key + " -> " + e.spec.sub_key + " on '" + e.resolved + "')";
      const bool untagged = created != st.sub_created.end() && !created->second;
      v.created = created == st.sub_created.end() || untagged ?
        std::numeric_limits<int64_t>::min() : *created->second;
      auto node = e.node.lock();
      // Bind only a proven identity. No mechanism ties a callback
      // to its handle, so a node with twin subscriptions on the topic is refused.
      // While the reader this one replaces lives, `sub_ = create_subscription`
      // in a hook (the old one goes at the assignment) and a twin created next
      // to it look the same, so the check waits until the old one is released.
      // Every reaction of a tag ends before the next tag, so a hook running on
      // another worker has assigned by then: a later tag that still sees the
      // old reader sees a twin. The tag is compared whole, so the next
      // microstep is already a later tag.
      const bool replaced = e.reader.lock() != nullptr;
      if (replaced) {
        const bool later = e.deferred_at && now > *e.deferred_at;
        // Two candidates pending at once: neither can be proven the RID's.
        auto pending = e.pending_reader.lock();
        const bool second = pending && pending != sub;
        if (mode == Settle::kFinal || later || second) {
          v.code = "rebind_identity";
          v.detail = second ?
            "a second new reader came while the first still waits for the one it replaces, "
            "so rcllf cannot prove which one the RID names " + v.what :
            "the reader it replaces is still alive at a later tag, so the new one is "
            "a twin of it and rcllf cannot prove which one the RID names " + v.what;
          clear_deferred_locked(st, e);
          e.reader_refused = true;
          e.reader.reset();
          verdicts.push_back(std::move(v));
          continue;
        }
        if (!e.deferred_at) {
          st.deferred_readers.fetch_add(1);
          e.deferred_at = now;
          v.deferred = true;
          verdicts.push_back(std::move(v));
        }
        e.pending_reader = sub;
        continue;
      }
      clear_deferred_locked(st, e);
      if (mode == Settle::kFinal) {
        continue;  // the old reader is gone and no drain came since
      }
      size_t live = 0;
      bool is_sub = false;
      if (node) {
        node->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
            group->find_subscription_ptrs_if([&](const rclcpp::SubscriptionBase::SharedPtr & sp) {
              if (e.resolved == sp->get_topic_name()) {
                ++live;
                is_sub = is_sub || sp == sub;
              }
              return false;
            });
          });
      }
      const size_t analyzed = node ? analyzed_twins(rid, node->get_rcl_node_handle()) : 0;
      // An ingress reader skipped the startup peer check, so it
      // meets its peer here (type, QoS) when the peer is present.
      const std::string peer = node && e.spec.role == CorrEdgeRole::kIngress ?
        xfed_bind_peer_problem(node->get_rcl_node_handle(), e.resolved, e.spec.peer_node, false) :
        "";
      const rclcpp::QoS actual = sub->get_actual_qos();
      const size_t want_depth = edge_reader_depth(e.spec, st.reader_depth);
      if (!node || e.resolved != sub->get_topic_name()) {
        v.code = "rebind_identity";
        v.detail = "the reader is on '" + std::string(sub->get_topic_name()) + "' " + v.what;
      } else if (analyzed != 1 || live != 1 || !is_sub) {
        v.code = "rebind_identity";
        v.detail = "the analysis found " + std::to_string(analyzed) + " and the node holds " +
          std::to_string(live) + " subscription(s) on the topic; rcllf cannot prove which one "
          "the RID names " + v.what;
      } else if (!peer.empty()) {
        v.code = "peer_check";
        v.detail = peer + " " + v.what;
      } else if (untagged) {
        v.code = "rebind_untagged";
        v.detail = "the reader was created off an LF worker while LF ran, so no tag orders "
          "it against the publishes " + v.what;
      } else if (sub->get_intra_process_waitable() != nullptr) {
        v.code = "intra_process";
        v.detail = "the subscription uses intra-process delivery " + v.what;
      } else if (actual.durability() != rclcpp::DurabilityPolicy::Volatile) {
        v.code = "durability";
        v.detail = "the reader is not VOLATILE " + v.what;
      } else if (actual.history() == rclcpp::HistoryPolicy::KeepLast &&
        actual.depth() < want_depth)
      {
        v.code = "reader_depth";
        v.detail = "reader depth " + std::to_string(actual.depth()) + " below " +
          std::to_string(want_depth) + " " + v.what;
      } else if (want_depth > 0 && e.spec.burst > want_depth) {
        v.code = "reader_depth";
        v.detail = "burst " + std::to_string(e.spec.burst) + " above reader depth " +
          std::to_string(want_depth) + " " + v.what;
      }
      if (v.code.empty()) {
        e.reader = sub;
        v.gen = ++e.reader_gen;
      } else {
        e.reader_refused = true;
        e.reader.reset();
      }
      verdicts.push_back(std::move(v));
    }
  }
  // Ingress and the table outside reg_mu: on_sample takes it on the ingress thread.
  for (const auto & v : verdicts) {
    if (v.deferred) {
      RCLCPP_INFO(
        corr_logger(), "rcllf corr: edge %u defers its new reader %s: the reader it replaces "
        "is still alive, so it is checked once that one is released", v.edge,
        v.what.c_str());
      continue;
    }
    if (!v.code.empty()) {
      RCLCPP_ERROR(
        corr_logger(), "rcllf corr: edge %u refused its new reader, %s: %s. The edge stops, "
        "its records end UNBOUND and reconcile fails", v.edge, v.code.c_str(),
        v.detail.c_str());
      st.table->note_reader_refused(v.edge, v.code, v.detail);
      IngressRole::instance().retire_corr_reader(v.edge);
      trace_corr(
        "{\"event\":\"corr\",\"kind\":\"reader_refused\",\"edge\":" +
        std::to_string(v.edge) + ",\"code\":\"" + v.code + "\"}");
      continue;
    }
    // The creation tag first, so no sample of the new reader meets the old one.
    st.table->note_reader_generation(v.edge, v.gen, v.created);
    const bool admitted = IngressRole::instance().replace_corr_reader(v.edge, sub, v.created);
    if (!admitted) {
      RCLCPP_ERROR(
        corr_logger(), "rcllf corr: edge %u: ingress could not admit reader generation %u %s; "
        "the edge stops", v.edge, v.gen, v.what.c_str());
      st.table->note_reader_refused(v.edge, "ingress", "ingress could not admit the reader");
      {
        std::lock_guard<std::mutex> lk(st.reg_mu);
        if (v.edge < st.edges.size()) {
          st.edges[v.edge].reader_refused = true;
          st.edges[v.edge].reader.reset();
        }
      }
      // The stopped edge takes no more through the reader it had (review).
      IngressRole::instance().retire_corr_reader(v.edge);
      trace_corr(
        "{\"event\":\"corr\",\"kind\":\"reader_refused\",\"edge\":" +
        std::to_string(v.edge) + ",\"code\":\"ingress\"}");
      continue;
    }
    const bool before_lf = v.created == std::numeric_limits<int64_t>::min();
    const std::string when = before_lf ? std::string("before LF ran") :
      "at tag " + std::to_string(v.created);
    RCLCPP_INFO(
      corr_logger(), "rcllf corr: edge %u admitted reader generation %u %s, created %s",
      v.edge, v.gen, v.what.c_str(), when.c_str());
    trace_corr(
      "{\"event\":\"corr\",\"kind\":\"reader_bound\",\"edge\":" + std::to_string(v.edge) +
      ",\"generation\":" + std::to_string(v.gen) + ",\"created\":" +
      (before_lf ? std::string("null") : std::to_string(v.created)) + "}");
  }
}

/// The deferred readers to settle, (RID, reader) per edge. Those
/// the node dropped too are forgotten.
std::vector<std::pair<int, rclcpp::SubscriptionBase::SharedPtr>> deferred_readers(int rid)
{
  State & st = state();
  std::vector<std::pair<int, rclcpp::SubscriptionBase::SharedPtr>> out;
  if (st.deferred_readers.load() == 0) {
    return out;
  }
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (auto & e : st.edges) {
    if (!e.deferred_at || (rid >= 0 && e.spec.sub_rid != rid)) {
      continue;
    }
    if (auto sub = e.pending_reader.lock()) {
      out.emplace_back(e.spec.sub_rid, std::move(sub));
    } else {
      clear_deferred_locked(st, e);
    }
  }
  return out;
}
}  // namespace

void on_subscription_bound(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  bind_reader(rid, sub, Settle::kBind);
}

bool deferred_reader_pending()
{
  return state().deferred_readers.load() != 0;
}

void retry_deferred_reader(int rid)
{
  for (const auto & d : deferred_readers(rid)) {
    bind_reader(d.first, d.second, Settle::kRetry);
  }
}

void release_subscriptions()
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  for (auto & e : st.edges) {
    e.sub.reset();
  }
}

void reset_for_test()
{
  // The ingress thread calls on_sample on the table replaced below and
  // keeps every subscription earlier tests admitted. Join it and forget them.
  IngressRole::instance().reset_for_test();
  set_federated(false);
  silence_deadline::reset_for_test();  // edge ids restart with the table
  State & st = state();
  // An edge may hold the last reference to a node. Its rosout publisher's fini
  // enters the gate, which takes reg_mu, so the edges die after the lock.
  decltype(st.edges) dropped;
  std::lock_guard<std::mutex> lk(st.reg_mu);
  dropped.swap(st.edges);
  st.legacy.clear();
  st.publisher_qos.clear();
  st.rebind.clear();
  for (auto & kv : st.slots) {
    kv.second->tracked = false;
    kv.second->edges_by_rid.clear();
  }
  st.table = std::make_unique<corr::Table>(random_epoch());
  st.table->set_observer(&trace_observer());
  st.table->set_clock(table_clock);
  st.table->set_fast(fast_federate);
  st.trace_begun = false;
  st.lf_start_traced.store(false);
  st.next_dispatch.store(0);
  for (auto & kv : st.slots) {
    kv.second->traced.store(false);
    kv.second->first_traced_ordinal = 0;
    for (auto * c : {&kv.second->backlog_refused, &kv.second->c_accepted, &kv.second->c_failed,
        &kv.second->c_nested, &kv.second->c_unsupported, &kv.second->c_no_context,
        &kv.second->c_undeclared, &kv.second->c_not_admitted, &kv.second->c_untracked})
    {
      c->store(0);
    }
  }
  for (auto & kv : st.slots) {
    if (kv.second->gid_ok) {
      st.table->register_gid(kv.second->gid);
    }
  }
  st.closed.store(false);
  st.started = false;
  st.reconciled = false;
  st.reconcile_ok = true;
  st.close_stop_time = -1;
  st.close_stop_micro = 0;
  st.deepened.clear();
  st.relaxed.clear();
  st.sub_inits.clear();
  st.construct_nodes.clear();
  st.sub_created.clear();
  st.lf_running_for_test.reset();
}

uint64_t after_close_publishes() {return state().after_close.load();}

void set_reader_depth_for_test(size_t depth)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.reader_depth = depth;
  st.deepened.clear();
  st.relaxed.clear();
  st.sub_inits.clear();
}

void set_drop_sample_for_test(uint32_t edge, uint64_t ordinal)
{
  // The table is replaced only by reset_for_test, so no lock guards the
  // pointer; the table locks its own state.
  state().table->set_drop_sample_for_test(edge, ordinal);
}

void set_lf_running_for_test(std::optional<bool> running)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.lf_running_for_test = running;
}

void set_sub_commit_hook_for_test(std::function<void()> hook)
{
  g_sub_commit_hook = std::move(hook);
}

void set_pub_meta_hook_for_test(std::function<void()> hook)
{
  g_pub_meta_hook = std::move(hook);
}

size_t sub_inits_for_test(const std::string & key, const std::string & resolved)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  auto it = st.sub_inits.find({key, resolved});
  return it == st.sub_inits.end() ? 0 : it->second;
}

const rclcpp::SubscriptionBase * edge_subscription_for_test(size_t id)
{
  State & st = state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  return id < st.edges.size() ? st.edges[id].sub.get() : nullptr;
}

namespace
{

struct ClockTriple
{
  int64_t mono = 0, real = 0, lf = 0, gap = 0;
};

int64_t ts_ns(const timespec & t)
{
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

ClockTriple read_clocks()
{
  timespec a{}, b{}, c{};
  ClockTriple t;
  clock_gettime(CLOCK_MONOTONIC, &a);
  clock_gettime(CLOCK_REALTIME, &b);
  t.lf = lf_time_physical();
  clock_gettime(CLOCK_MONOTONIC, &c);
  t.mono = ts_ns(a);
  t.real = ts_ns(b);
  t.gap = ts_ns(c) - ts_ns(a);
  return t;
}

std::string read_line(const char * path)
{
  std::ifstream f(path);
  std::string s;
  std::getline(f, s);
  return s;
}

/// Parent PID chain of this process, parent first, space separated, up to PID 1
/// or the first PID whose /proc/<pid>/stat cannot be read.
std::string ancestry_chain()
{
  std::string out;
  int64_t p = static_cast<int64_t>(getppid());
  for (int depth = 0; p > 0 && depth < 64; ++depth) {
    out += (out.empty() ? "" : " ") + std::to_string(p);
    if (p == 1) {
      break;
    }
    const std::string stat = read_line(("/proc/" + std::to_string(p) + "/stat").c_str());
    const auto rp = stat.rfind(')');
    if (rp == std::string::npos || rp + 4 >= stat.size()) {
      break;
    }
    // After ") ": the state character, a space, then the parent PID.
    const char * first = stat.data() + rp + 4;
    const char * last = stat.data() + stat.size();
    int64_t parent = 0;
    if (std::from_chars(first, last, parent).ec != std::errc{}) {
      break;
    }
    p = parent;
  }
  return out;
}

/// RCLLF_RUN_TOKEN of this process's environment, read once, or "".
const std::string & run_token()
{
  static const std::string token = [] {
      const char * v = std::getenv("RCLLF_RUN_TOKEN");
      return std::string(v != nullptr ? v : "");
    }();
  return token;
}

/// GNU build-id of the main executable, hex, or "".
std::string exe_build_id()
{
  std::string out;
  dl_iterate_phdr(
    [](struct dl_phdr_info * info, size_t, void * data) -> int {
      auto * o = static_cast<std::string *>(data);
      for (int i = 0; i < info->dlpi_phnum; ++i) {
        const auto & ph = info->dlpi_phdr[i];
        if (ph.p_type != PT_NOTE) {
          continue;
        }
        const char * p = reinterpret_cast<const char *>(info->dlpi_addr + ph.p_vaddr);
        const char * end = p + ph.p_memsz;
        while (p + sizeof(ElfW(Nhdr)) <= end) {
          const auto * n = reinterpret_cast<const ElfW(Nhdr) *>(p);
          const char * name = p + sizeof(ElfW(Nhdr));
          const char * desc = name + ((n->n_namesz + 3) & ~3u);
          if (n->n_type == NT_GNU_BUILD_ID && n->n_namesz == 4 && std::memcmp(name, "GNU",
                4) == 0)
          {
            static const char * kHex = "0123456789abcdef";
            for (uint32_t k = 0; k < n->n_descsz; ++k) {
              const auto byte = static_cast<unsigned char>(desc[k]);
              o->push_back(kHex[byte >> 4]);
              o->push_back(kHex[byte & 15]);
            }
            return 1;
          }
          p = desc + ((n->n_descsz + 3) & ~3u);
        }
      }
      return 1;  // only the first object (the executable)
    }, &out);
  return out;
}

}  // namespace

}  // namespace corr_rt
}  // namespace ros2_lf_binding

/// Build facts the generator emits into the executable. Weak, so a
/// binary without them (hand-written, older generator) links and reports
/// manifest_mismatch instead of failing.
extern "C" __attribute__((weak)) const char * const rcllf_trace_build_info[3];

namespace ros2_lf_binding
{
namespace corr_rt
{

void trace_begin()
{
  State & st = state();
  std::vector<std::shared_ptr<Slot>> slots;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    if (st.trace_begun) {
      return;
    }
    st.trace_begun = true;
    for (auto & kv : st.slots) {
      slots.push_back(kv.second);
    }
  }
  semtrace::begin(st.table->epoch());
  const ClockTriple c = read_clocks();
  timespec rm{}, rr{};
  clock_getres(CLOCK_MONOTONIC, &rm);
  clock_getres(CLOCK_REALTIME, &rr);
  const char * const * bi = rcllf_trace_build_info;
  char exe[4096] = {0};
  const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n > 0) {
    exe[n] = 0;
  }
  char host[256] = {0};
  gethostname(host, sizeof(host) - 1);
  const std::string boot = read_line("/proc/sys/kernel/random/boot_id");
  const std::string build_id = exe_build_id();
  const std::string ancestry = ancestry_chain();
  const auto start = lf_current_tag();
  rcllf_tp_run_begin rb{};
  rb.schema_major = RCLLF_TRACE_SCHEMA_MAJOR;
  rb.schema_minor = RCLLF_TRACE_SCHEMA_MINOR;
  rb.binding_version = RCLLF_BINDING_VERSION;
  rb.static_manifest_hash = bi != nullptr && bi[0] ? bi[0] : "";
  rb.generated_source_hash = bi != nullptr && bi[1] ? bi[1] : "";
  rb.model_hash = bi != nullptr && bi[2] ? bi[2] : "";
  rb.build_id = build_id.c_str();
  rb.executable = exe;
  rb.pid = static_cast<int64_t>(getpid());
  rb.boot_id = boot.c_str();
  rb.host = host;
  rb.clock_monotonic_ns = c.mono;
  rb.clock_realtime_ns = c.real;
  rb.clock_lf_physical_ns = c.lf;
  rb.clock_read_gap_ns = c.gap;
  rb.res_monotonic_ns = ts_ns(rm);
  rb.res_realtime_ns = ts_ns(rr);
  rb.lf_start_time_ns = start.first;
  rb.ancestry = ancestry.c_str();
  rb.run_token = run_token().c_str();
#ifdef _LF_CLOCK_SYNC_ON
  rb.lf_clock_sync = 1;
#else
  rb.lf_clock_sync = 0;
#endif
  semtrace::run_begin(rb);
  // Test hook (T10, case 7): flood the rcllf channel so a small sub-buffer drops.
  if (const char * burst = std::getenv("RCLLF_TEST_TRACE_BURST")) {
    const uint64_t n = std::strtoull(burst, nullptr, 10);
    rcllf_tp_provider p{};
    p.category = RCLLF_CAT_LIFECYCLE;
    p.provider_version = RCLLF_TRACE_PROVIDER_VERSION;
    for (uint64_t i = 0; i < n; ++i) {
      semtrace::provider(p);
    }
  }
  for (uint8_t cat = 0; cat < RCLLF_CAT_COUNT; ++cat) {
    rcllf_tp_provider p{};
    p.category = cat;
    p.provider_version = RCLLF_TRACE_PROVIDER_VERSION;
    p.compiled_in = semtrace::compiled_in() && cat != RCLLF_CAT_POLICY && cat != RCLLF_CAT_TUPLE;
    semtrace::provider(p);
  }
  // Backlog handover (D1): under each tracked slot's mutex, everything
  // assigned so far is backlog, the rest is traced.
  for (auto & s : slots) {
    if (!s->gid_ok) {
      continue;
    }
    std::lock_guard<std::mutex> lk(s->m);
    if (!s->tracked) {
      continue;
    }
    const uint64_t assigned = s->assigned.load();
    rcllf_tp_backlog b{};
    fill_gid(b.gid, s->gid);
    b.first_ordinal = assigned > 0 ? 1 : 0;
    b.last_ordinal = assigned;
    b.refused = s->backlog_refused.load();
    semtrace::backlog(b);
    s->first_traced_ordinal = assigned + 1;
    s->traced.store(true, std::memory_order_release);
  }
}

void trace_close()
{
  State & st = state();
  if (!semtrace::close(nullptr)) {
    return;
  }
  std::vector<std::shared_ptr<Slot>> slots;
  size_t n_edges = 0;
  bool reconciled = false;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (auto & kv : st.slots) {
      if (kv.second->gid_ok) {
        slots.push_back(kv.second);
      }
    }
    n_edges = st.edges.size();
    reconciled = st.reconciled && st.reconcile_ok;
  }
  uint64_t untracked = 0;
  for (auto & s : slots) {
    rcllf_tp_run_end_publisher p{};
    fill_gid(p.gid, s->gid);
    p.tracked = s->tracked;
    p.accepted = s->c_accepted.load();
    p.failed = s->c_failed.load();
    p.refused_no_context = s->c_no_context.load();
    p.refused_undeclared_effect = s->c_undeclared.load();
    p.refused_edge_not_admitted = s->c_not_admitted.load();
    p.nested = s->c_nested.load();
    p.unsupported_api = s->c_unsupported.load();
    p.first_traced_ordinal = s->first_traced_ordinal;
    p.untracked = s->c_untracked.load();
    untracked += p.untracked;
    semtrace::close_publisher(p);
  }
  for (size_t i = 0; i < n_edges; ++i) {
    const corr::EdgeCounts c = st.table->counts(static_cast<uint32_t>(i));
    rcllf_tp_run_end_edge e{};
    e.edge = static_cast<uint32_t>(i);
    e.expectations = c.expectations;
    for (const auto & kv : c.outcomes) {
      (kv.first == corr::Outcome::MATCHED ? e.matched : e.terminal_other) += kv.second;
    }
    e.samples_taken = c.samples_taken;
    auto r = c.samples.find(corr::SampleOutcome::RETAINED);
    e.samples_retained = r == c.samples.end() ? 0 : r->second;
    e.orphan_dispatches = c.orphan_dispatches;
    e.count_mismatch = c.count_mismatch;
    semtrace::close_edge(e);
  }
  const ClockTriple c = read_clocks();
  rcllf_tp_run_end end{};
  end.n_publishers = static_cast<uint32_t>(slots.size());
  end.n_edges = static_cast<uint32_t>(n_edges);
  end.untracked_publishes = untracked;
  end.identity_faults = st.table->identity_faults();
  // Same meaning as the JSON record's "reconciled": accounting only.
  end.reconciled = reconciled;
  end.clock_monotonic_ns = c.mono;
  end.clock_realtime_ns = c.real;
  end.clock_lf_physical_ns = c.lf;
  end.clock_read_gap_ns = c.gap;
  semtrace::close_run_end(end);
}

void trace_lf_start()
{
  State & st = state();
  if (st.lf_start_traced.load(std::memory_order_acquire) || !semtrace::begun()) {
    return;
  }
  const auto start = lf_start_tag();
  if (start.first <= 0 || st.lf_start_traced.exchange(true)) {
    return;
  }
  rcllf_tp_lf_start e{};
  e.lf_start_time_ns = start.first;
  e.lf_start_microstep = start.second;
  semtrace::lf_start(e);
}

DispatchTrace::DispatchTrace(
  int rid, uint8_t entry_kind, uint32_t edge, const corr::Gid * gid, uint64_t ordinal)
: id_(state().next_dispatch.fetch_add(1) + 1), prev_id_(t_dispatch_id),
  prev_time_(t_dispatch_time), prev_micro_(t_dispatch_micro),
  outcome_(edge == RCLLF_TRACE_NO_EDGE ? RCLLF_OUTCOME_NONE : RCLLF_OUTCOME_PENDING)
{
  trace_lf_start();
  const auto tag = lf_current_tag();
  t_dispatch_id = id_;
  t_dispatch_time = tag.first;
  t_dispatch_micro = tag.second;
  stall_progress(tag.first, tag.second);  // stall watch: the tag advanced
  rcllf_tp_dispatch_begin b{};
  b.dispatch_id = id_;
  b.rid = rid;
  b.entry_kind = entry_kind;
  b.edge = edge;
  if (gid != nullptr) {
    fill_gid(b.gid, *gid);
  }
  b.ordinal = ordinal;
  b.tag_time = tag.first;
  b.tag_microstep = tag.second;
  semtrace::dispatch_begin(b);
}

DispatchTrace::~DispatchTrace()
{
  rcllf_tp_dispatch_end e{};
  e.dispatch_id = id_;
  e.outcome = outcome_;
  e.retained_id = retained_;
  e.join_wait_ns = join_wait_ns_;
  e.callback_ran = ran_;
  e.cutoff = cutoff_;
  e.missing_hook = missing_hook_;
  semtrace::dispatch_end(e);
  t_dispatch_id = prev_id_;
  t_dispatch_time = prev_time_;
  t_dispatch_micro = prev_micro_;
}

}  // namespace corr_rt
}  // namespace ros2_lf_binding

using ros2_lf_binding::corr::Origin;
using ros2_lf_binding::corr::Outcome;
namespace rt = ros2_lf_binding::corr_rt;
extern "C" int64_t lf_time_physical(void);

namespace
{
/// JSON array of the missing-message hooks that ran, e.g. ["deadline"].
std::string hooks_json(const std::vector<std::string> & hooks)
{
  std::string out = "[";
  for (size_t i = 0; i < hooks.size(); ++i) {
    out += (i == 0 ? "\"" : ",\"") + hooks[i] + "\"";
  }
  return out + "]";
}

/// A broken cross-federate invariant. Never silent (Q3 style).
[[noreturn]] void xfed_fatal(const std::string & msg)
{
  RCLCPP_FATAL(rclcpp::get_logger("ros2_lf_binding.corr"), "rcllf corr: %s", msg.c_str());
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(3);
}

/// The kernel boot id of this host, the one-clock check.
const std::array<uint8_t, 16> & host_boot_id()
{
  static const std::array<uint8_t, 16> id =
    ros2_lf_binding::corr_rt::read_boot_id(ros2_lf_binding::corr_rt::kBootIdPath).value_or(
    std::array<uint8_t, 16>{});
  return id;
}

int64_t edge_delay_ns(int edge)
{
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge >= 0 && static_cast<size_t>(edge) < st.edges.size()) {
    return std::max<int64_t>(0, st.edges[edge].spec.delay_ns);
  }
  return 0;
}

/// The ingress edge runs over a decentralized link (set_corr_edge_tardy_drop).
bool edge_tardy_drop(int edge)
{
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  return edge >= 0 && static_cast<size_t>(edge) < st.edges.size() &&
         st.edges[edge].spec.tardy_drop;
}

/// The ingress edge stops LF on a CONTROL_FAILURE (set_corr_edge_control_stop).
bool edge_control_stop(int edge)
{
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  return edge >= 0 && static_cast<size_t>(edge) < st.edges.size() &&
         st.edges[edge].spec.control_stop;
}

/// `control_failure_policy: stop`. Resolve the executable's threaded
/// lf_request_stop as runtime.cpp does (this library links its own
/// single-threaded reactor-c copy). Once per process.
void control_failure_stop(int edge)
{
  static std::atomic<bool> issued{false};
  if (issued.exchange(true)) {
    return;
  }
  using StopFn = void (*)(void);
  auto stop = reinterpret_cast<StopFn>(dlsym(RTLD_DEFAULT, "lf_request_stop"));
  if (stop == nullptr) {
    xfed_fatal(
      "edge " + std::to_string(edge) +
      " control_failure_policy stop but lf_request_stop is not resolvable");
  }
  stop();
}

/// Stall watch: the stalled-sender watch of this federate. Leaked so
/// the detached watchdog never reads a destroyed object during exit.
struct StallState
{
  std::mutex m;
  ros2_lf_binding::corr::StallWatch watch;
  std::once_flag armed;
};

StallState & stall_state()
{
  static StallState * s = new StallState();
  return *s;
}

/// The longest period among the timers bound when the first reaction ran.
int64_t max_bound_timer_period()
{
  int64_t p = -1;
  for (const auto & h : ros2_lf_binding::g_handle_table) {
    if (h.kind != ros2_lf_binding::HandleKind::TIMER) {
      continue;
    }
    if (auto t = h.timer.lock()) {
      int64_t v = -1;
      if (rcl_timer_get_period(t->get_timer_handle().get(), &v) == RCL_RET_OK) {
        p = std::max(p, v);
      }
    }
  }
  return p;
}

void stall_watchdog(int64_t bound)
{
  // Poll at an eighth of the bound, so a stall is reported at most W / 8 past
  // W; at least 1 ms, so a small bound does not spin a core.
  constexpr int64_t kStallPollFloorNs = 1000000;
  const auto poll = std::chrono::nanoseconds(std::max<int64_t>(bound / 8, kStallPollFloorNs));
  auto & s = stall_state();
  while (true) {
    std::this_thread::sleep_for(poll);
    if (ros2_lf_binding::g_user_callbacks_closed.load() || rt::table().closed()) {
      return;  // shutdown began: logical time stops by design
    }
    const int64_t now = lf_time_physical();
    int64_t tag = 0;
    int64_t since = 0;
    {
      std::lock_guard<std::mutex> lk(s.m);
      if (!s.watch.check(now, bound)) {
        continue;
      }
      tag = s.watch.tag();
      since = s.watch.since();
    }
    const int64_t stop = ros2_lf_binding::lf_stop_tag().first;
    if (stop >= 0 && tag >= stop) {
      // Logical time stands at the stop tag, LF is done. After a
      // stop request the shutdown can hold a federate there for the end-count
      // wait of its peer (up to 1.5 s in a stop-with-drop run on a workstation), no stall.
      return;
    }
    rt::table().note_stall();
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "CONTROL_FAILURE stall: logical time %lld stood still for %lld ns, above the stall "
      "bound %lld ns: an upstream federate stopped advancing",
      static_cast<long long>(tag), static_cast<long long>(now - since),  // NOLINT(runtime/int)
      static_cast<long long>(bound));  // NOLINT(runtime/int)
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"control_failure\",\"cause\":\"stall\",\"tag\":" +
      std::to_string(tag) + ",\"waited_ns\":" + std::to_string(now - since) +
      ",\"bound_ns\":" + std::to_string(bound) + "}");
    const int edges = static_cast<int>(rt::table().edge_count());
    for (int e = 0; e < edges; ++e) {
      if (edge_control_stop(e)) {
        control_failure_stop(e);
        break;
      }
    }
  }
}

void stall_arm_once();

/// The stop-due backstop. A federate without a timer
/// has no stall bound (bound -1), since it may idle legitimately. It may not
/// idle past its stop tag, though: once physical time passes stop + window,
/// window = sum (d + T + eps) + lag over its ingress edges, every healthy
/// upstream has granted the stop and LF has closed the epoch. Still open
/// means an upstream federate stopped advancing. Reported once, then handled
/// like a stall of the timer watch.
void stall_backstop(int64_t window)
{
  constexpr int64_t kBackstopPollNs = 10000000;  // 10 ms, well under any T
  while (true) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(kBackstopPollNs));
    if (ros2_lf_binding::g_user_callbacks_closed.load() || rt::table().closed()) {
      return;  // LF reached its stop: nothing stalled
    }
    const int64_t stop = ros2_lf_binding::lf_stop_tag().first;
    if (stop < 0 || stop == std::numeric_limits<int64_t>::max()) {
      continue;  // no stop tag yet: no due time to judge against
    }
    if (ros2_lf_binding::lf_current_tag().first >= stop) {
      return;  // LF reached its stop tag: its shutdown may take a while, no stall
    }
    const int64_t now = lf_time_physical();
    if (now <= stop || now - stop <= window) {
      continue;
    }
    if (rt::table().stalls() > 0) {
      return;  // the timer watch already reported this episode
    }
    rt::table().note_stall();
    const int64_t tag = ros2_lf_binding::lf_current_tag().first;
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "CONTROL_FAILURE stall: the stop tag %lld is %lld ns past, above the stall "
      "window %lld ns, logical time stands at %lld: an upstream federate stopped advancing",
      static_cast<long long>(stop), static_cast<long long>(now - stop),  // NOLINT(runtime/int)
      static_cast<long long>(window), static_cast<long long>(tag));  // NOLINT(runtime/int)
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"control_failure\",\"cause\":\"stall_stop_due\","
      "\"tag\":" + std::to_string(tag) + ",\"stop\":" + std::to_string(stop) +
      ",\"waited_ns\":" + std::to_string(now - stop) +
      ",\"bound_ns\":" + std::to_string(window) + "}");
    const int edges = static_cast<int>(rt::table().edge_count());
    for (int e = 0; e < edges; ++e) {
      if (edge_control_stop(e)) {
        control_failure_stop(e);
        break;
      }
    }
    return;
  }
}

void stall_progress_impl(int64_t tag, uint32_t microstep)
{
  if (!rt::federated() || rt::fast_federate()) {
    return;  // one process has no upstream federate; a fast one has no physical clock
  }
  auto & s = stall_state();
  {
    std::lock_guard<std::mutex> lk(s.m);
    s.watch.progress(tag, microstep, lf_time_physical());
  }
  stall_arm_once();
}

/// The start tag (start, 0) is due at physical time `start_ns`,
/// so logical time stands still from then on until a dispatch moves it. The
/// first dispatch at (start, 0) keeps this since; a later tag resets it.
void stall_watch_start_impl(int64_t start_ns)
{
  if (!rt::federated() || rt::fast_federate()) {
    return;
  }
  auto & s = stall_state();
  {
    std::lock_guard<std::mutex> lk(s.m);
    s.watch.progress(start_ns, 0, start_ns);
  }
  // A lifecycle node binds its timers on activation, after this point. With
  // no timer bound yet the bound would read as "no timer" and the once-only
  // arming would keep the watch off for the whole run, so the first dispatch
  // arms it instead (the earlier behaviour for such a federate).
  if (max_bound_timer_period() > 0) {
    stall_arm_once();
  }
  // Every federate with a bounded ingress window
  // gets the stop-due backstop, the timer-free one included.
  const int64_t window = rt::table().stall_window_ns();
  ros2_lf_binding::trace_corr(
    "{\"event\":\"corr\",\"kind\":\"stall_backstop\",\"window_ns\":" +
    std::to_string(window) + "}");
  if (window >= 0) {
    std::thread(stall_backstop, window).detach();
  }
}

void stall_arm_once()
{
  auto & s = stall_state();
  std::call_once(s.armed, [] {
      const int64_t bound = rt::table().stall_bound_ns(max_bound_timer_period());
      ros2_lf_binding::trace_corr(
        "{\"event\":\"corr\",\"kind\":\"stall_watch\",\"bound_ns\":" +
        std::to_string(bound) + "}");
      if (bound >= 0) {
        std::thread(stall_watchdog, bound).detach();
      }
    });
}

/// What the missing hooks running now on this thread were raised for.
thread_local rcllf_lost_ctx_t t_lost_ctx{};
thread_local bool t_lost_ctx_valid = false;

/// Publish a lost context for the hooks run in this scope and restore
/// the previous one on every exit, a throwing hook included, so a later
/// query outside any hook never reads a stale context. Nesting safe.
class LostCtxScope
{
public:
  explicit LostCtxScope(const rcllf_lost_ctx_t & ctx)
  : prev_(t_lost_ctx), prev_valid_(t_lost_ctx_valid)
  {
    t_lost_ctx = ctx;
    t_lost_ctx_valid = true;
  }
  ~LostCtxScope()
  {
    t_lost_ctx = prev_;
    t_lost_ctx_valid = prev_valid_;
  }
  LostCtxScope(const LostCtxScope &) = delete;
  LostCtxScope & operator=(const LostCtxScope &) = delete;

private:
  rcllf_lost_ctx_t prev_;
  bool prev_valid_;
};

/// The release key (tag) an arrive files `count` records of
/// `edge` under, published at microstep `pub_microstep`. Lets a run compare
/// the cross-host key with the one-process key.
void trace_release_key(
  int edge, uint32_t count, uint32_t pub_microstep, int64_t release, bool remote)
{
  if (!ros2_lf_binding::trace_callbacks_enabled()) {
    return;  // the key lookup takes the table lock, skip it when nothing prints
  }
  const int64_t start =
    ros2_lf_binding::lf_current_tag().first - ros2_lf_binding::lf_elapsed_tag().first;
  ros2_lf_binding::trace_corr(
    "{\"event\":\"corr\",\"kind\":\"release_key\",\"edge\":" + std::to_string(edge) +
    ",\"remote\":" + (remote ? "true" : "false") + ",\"count\":" + std::to_string(count) +
    ",\"pub_microstep\":" + std::to_string(pub_microstep) + ",\"release_elapsed\":" +
    std::to_string(release - start) + ",\"release_microstep\":" +
    std::to_string(rt::table().release_microstep(static_cast<uint32_t>(edge), pub_microstep)) +
    "," + ros2_lf_binding::tag_json() + "}");
}
}  // namespace

namespace ros2_lf_binding
{
namespace corr_rt
{
void stall_progress(int64_t tag, uint32_t microstep) {stall_progress_impl(tag, microstep);}
void stall_watch_start(int64_t start_ns) {stall_watch_start_impl(start_ns);}
}  // namespace corr_rt
}  // namespace ros2_lf_binding

extern "C" {

void rcllf_gate_sub_qos(const void * node, const char * topic_name, void * qos_ptr)
{
  // A depth-1 reader keeps only the newest sample, so a writer
  // that publishes twice before ingress takes loses the first one in DDS,
  // before any rcllf code runs. The reader of an admitted edge keeps up to
  // kRetainPerEdge samples instead; each dispatch still hands the callback
  // exactly the one sample its publish produced.
  // A RELIABLE reader hands samples out in sequence order, so
  // after one unrepairable loss it withholds every later sample until the
  // writer's history drops the lost one (KeepLast(n): n publications later).
  // Under centralized coordination each join waiting for a withheld sample
  // delays the publication that would end the wait, and one loss becomes n
  // missing outcomes. The records over the LF connection already detect every
  // loss on an admitted edge, so the reader of a cross-federate (ingress)
  // edge runs BEST_EFFORT (a compatible match with a reliable writer). A
  // same-federate edge keeps the user's reliability: no join across the
  // federation link holds its publisher, so one loss stays one lost call and
  // DDS retransmission still repairs a transient receive-buffer overflow.
  // RCLLF_ADMITTED_READER_RELIABLE=1 keeps the user's
  // reliability on every edge for comparison runs.
  auto * qos = static_cast<rmw_qos_profile_t *>(qos_ptr);
  const int history = static_cast<int>(qos->history);
  const size_t depth = qos->depth;
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  const auto * n = static_cast<const rcl_node_t *>(node);
  // By node handle and resolved topic, so no binding key is needed.
  auto & rebind = ros2_lf_binding::t_pending_rebind;
  rebind.clear();
  for (const auto & e : st.edges) {
    if (!e.spec.sub_rebind || e.spec.role == ros2_lf_binding::CorrEdgeRole::kEgress) {
      continue;
    }
    const auto base = e.node.lock();
    if (base && base->get_rcl_node_handle() == n &&
      ros2_lf_binding::resolve_rcl(n, topic_name) == e.resolved)
    {
      rebind.push_back(static_cast<uint32_t>(e.spec.edge_id));
    }
  }
  // The key of the node under construction on this thread, else of the node
  // itself when a construction scope created it (a worker thread's subscription).
  std::string key = ros2_lf_binding::t_construct_key;
  if (key.empty()) {
    auto it = st.construct_nodes.find(node);
    if (it == st.construct_nodes.end()) {
      return;
    }
    key = it->second;
  }
  const std::string resolved = ros2_lf_binding::resolve_rcl(n, topic_name);
  if (resolved.empty()) {
    return;
  }
  // Every subscription counts toward the rank, so a legacy twin keeps its
  // place and only the admitted rank is deepened. The rank is committed in
  // rcllf_gate_on_sub_init once the init succeeded.
  const size_t rank = st.sub_inits[{key, resolved}];
  auto & pending = ros2_lf_binding::t_pending_sub;
  pending = {true, false, false, key, resolved, rank, depth, static_cast<int>(qos->reliability)};
  bool admitted = false;
  bool cross = false;  // the publisher runs in another federate
  std::optional<size_t> want;
  for (const auto & e : st.edges) {
    // A lone lifecycle subscription made or reset in a hook is the
    // edge's reader at every creation, so it keeps logical rank 0 however
    // many readers the node created on the topic before.
    const bool lone_rebind = e.spec.sub_rebind &&
      ros2_lf_binding::analyzed_twins(e.spec.sub_rid, n) == 1;
    if (e.spec.role != ros2_lf_binding::CorrEdgeRole::kEgress &&
      e.spec.sub_key == key &&
      (lone_rebind || ros2_lf_binding::edge_sub_rank(e.spec, n) == rank) &&
      ros2_lf_binding::resolve_rcl(n, e.spec.sub_topic.c_str()) == resolved)
    {
      // The edge's exec-config reader_depth, else kRetainPerEdge. The
      // members of a fan-in group name one reader: the deepest.
      admitted = true;
      cross = cross || e.spec.role == ros2_lf_binding::CorrEdgeRole::kIngress;
      want = std::max(
        want.value_or(0), ros2_lf_binding::edge_reader_depth(e.spec, st.reader_depth));
    }
  }
  if (!admitted) {
    return;
  }
  if (cross && qos->reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
    !ros2_lf_binding::corr_rt::keep_admitted_reader_reliable())
  {
    pending.relaxed = true;
    qos->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
  if (history == RMW_QOS_POLICY_HISTORY_KEEP_LAST && want && depth < *want) {
    pending.deepened = true;
    qos->depth = *want;
  }
}

void rcllf_gate_on_sub_init(int ret, const void * subscription)
{
  std::vector<uint32_t> rebind;
  rebind.swap(ros2_lf_binding::t_pending_rebind);
  if (ret == RCL_RET_OK && subscription != nullptr) {
    // The reader's creation tag. A hook on an LF worker runs at its
    // reaction's tag. Before LF runs (the constructor, an auto_transition) a
    // creation precedes every publish. Off a worker while LF runs no tag
    // orders it against the publishes: nullopt, and its bind is refused.
    auto & st = ros2_lf_binding::state();
    std::optional<int64_t> tag;
    if (ros2_lf_binding::is_lf_worker_thread()) {
      tag = ros2_lf_binding::lf_current_tag().first;
    } else {
      std::lock_guard<std::mutex> lk(st.reg_mu);
      if (!st.lf_running_for_test.value_or(ros2_lf_binding::lf_is_running())) {
        tag = std::numeric_limits<int64_t>::min();
      }
    }
    {
      std::lock_guard<std::mutex> lk(st.reg_mu);
      st.sub_created[subscription] = tag;
    }
    if (tag && *tag != std::numeric_limits<int64_t>::min()) {
      for (const uint32_t edge : rebind) {
        st.table->note_reader_created(edge, *tag);
      }
    }
  }
  auto & pending = ros2_lf_binding::t_pending_sub;
  if (!pending.active) {
    return;
  }
  pending.active = false;
  if (ret != RCL_RET_OK) {
    return;  // no subscription exists, so it takes no rank
  }
  if (ros2_lf_binding::g_sub_commit_hook) {
    ros2_lf_binding::g_sub_commit_hook();
  }
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  st.sub_inits[{pending.key, pending.resolved}] = pending.rank + 1;
  if (pending.deepened) {
    st.deepened[{pending.key, pending.resolved, pending.rank}] = pending.user_depth;
  }
  if (pending.relaxed) {
    st.relaxed[{pending.key, pending.resolved, pending.rank}] = pending.user_reliability;
  }
}

void rcllf_gate_on_node_init(const void * node, int ret)
{
  if (ret != RCL_RET_OK) {
    return;
  }
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (ros2_lf_binding::t_construct_key.empty()) {
    st.construct_nodes.erase(node);  // a reused address, outside any scope
  } else {
    st.construct_nodes[node] = ros2_lf_binding::t_construct_key;
  }
}

void rcllf_gate_on_node_fini(const void * node)
{
  // A key may own several rcl nodes (a node class constructing helper nodes
  // inside its scope). They share the key's ranks, so the ranks reset only
  // when the last of them is finalized, and a recreated node starts at 0.
  auto & st = ros2_lf_binding::state();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  auto it = st.construct_nodes.find(node);
  if (it == st.construct_nodes.end()) {
    return;
  }
  const std::string key = it->second;
  st.construct_nodes.erase(it);
  for (const auto & kv : st.construct_nodes) {
    if (kv.second == key) {
      return;
    }
  }
  for (auto s = st.sub_inits.begin(); s != st.sub_inits.end(); ) {
    s = s->first.first == key ? st.sub_inits.erase(s) : std::next(s);
  }
  for (auto d = st.deepened.begin(); d != st.deepened.end(); ) {
    d = std::get<0>(d->first) == key ? st.deepened.erase(d) : std::next(d);
  }
}

void rcllf_gate_on_init(
  const void * publisher, const void * node, const void * type_support, int ret)
{
  if (ret != RCL_RET_OK) {
    return;
  }
  ros2_lf_binding::plant_record::note_publisher(publisher, type_support);
  auto slot = ros2_lf_binding::slot_for(publisher, true);
  if (slot->gid_ok) {
    // A cross-federate take link announces this publisher.
    ros2_lf_binding::tag_log::on_writer(slot->topic, slot->gid);
  }
  const char * fqn = rcl_node_get_fully_qualified_name(static_cast<const rcl_node_t *>(node));
  const rmw_qos_profile_t * qos =
    rcl_publisher_get_actual_qos(static_cast<const rcl_publisher_t *>(publisher));
  {
    std::lock_guard<std::mutex> lk(slot->m);
    if (ros2_lf_binding::g_pub_meta_hook) {
      ros2_lf_binding::g_pub_meta_hook();
    }
    slot->node_fqn = fqn ? fqn : "";
    slot->volatile_durability = qos == nullptr ||
      qos->durability != RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    slot->best_effort = qos != nullptr &&
      qos->reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    slot->internal = ros2_lf_binding::internal_publisher(
      slot->topic, ros2_lf_binding::type_name_of(type_support));
  }
  if (slot->gid_ok) {
    // The writer's identity, traced where it is created.
    ros2_lf_binding::trace_publisher(
      slot->node_fqn, slot->topic, ros2_lf_binding::corr::gid_hex(slot->gid));
  }
  // A lifecycle node created this publisher for an admitted edge after
  // startup scanned (on_configure, or a cleanup and configure cycle).
  auto & st = ros2_lf_binding::state();
  std::vector<ros2_lf_binding::RebindPub> mine;
  std::vector<std::string> peer_of;  // per edge in `mine`: its peer node if egress
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    for (const auto & r : st.rebind) {
      if (r.node_fqn == slot->node_fqn && r.topic == slot->topic) {
        mine.push_back(r);
        const bool egress = r.edge < st.edges.size() &&
          st.edges[r.edge].spec.role == ros2_lf_binding::CorrEdgeRole::kEgress;
        peer_of.push_back(egress ? st.edges[r.edge].spec.peer_node : std::string());
      }
    }
  }
  for (size_t i = 0; i < mine.size(); ++i) {
    const auto & r = mine[i];
    // An egress publisher skipped the startup peer check, so it
    // meets its peer here (type, QoS) when the peer is present, refused as D7.
    const std::string problem = peer_of[i].empty() ? std::string() :
      rt::xfed_bind_peer_problem(
      static_cast<const rcl_node_t *>(node), r.topic, peer_of[i], true);
    if (!problem.empty()) {
      RCLCPP_FATAL(
        ros2_lf_binding::corr_logger(),
        "rcllf corr: bind refused egress edge %u on '%s': check peer_check: %s",
        r.edge, r.topic.c_str(), problem.c_str());
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(3);
    }
    if (!slot->volatile_durability) {
      RCLCPP_ERROR(
        ros2_lf_binding::corr_logger(),
        "rcllf corr: admitted edge %u: node '%s' created a publisher on '%s' that is not "
        "VOLATILE. An admitted edge never falls back to the legacy path",
        r.edge, r.node_fqn.c_str(), r.topic.c_str());
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(1);
    }
    ros2_lf_binding::bind_slot(*slot, r.edge, r.rids);
    RCLCPP_INFO(
      ros2_lf_binding::corr_logger(), "rcllf corr: bound %s's publisher on '%s' to edge %u",
      r.node_fqn.c_str(), r.topic.c_str(), r.edge);
  }
}

// The output deadline check. Called as the last step of the
// gate on every path that forwards to the real publish, after the slot lock,
// the delivery lock and the bookkeeping, so a wait on any of them counts as
// lateness. The publish happens whatever it returns.
static void output_deadline_check(const std::string & topic)
{
  ros2_lf_binding::output_deadline::check(
    ros2_lf_binding::t_origin_set ? ros2_lf_binding::t_origin_rid : -1, topic);
}

void * rcllf_gate_pre(const void * publisher, int api)
{
  auto & st = ros2_lf_binding::state();
  if (st.closed.load(std::memory_order_acquire)) {
    st.after_close.fetch_add(1);
    return nullptr;
  }
  auto slot = ros2_lf_binding::slot_for(publisher, true);
  ros2_lf_binding::corr_rt::topic_sent_note(slot->topic);
  if (!slot->gid_ok) {
    ros2_lf_binding::tag_log::on_publish(
      slot->topic, ros2_lf_binding::t_origin_set, false, {}, 0);
    output_deadline_check(slot->topic);
    return nullptr;
  }
  auto * ctx = new ros2_lf_binding::GateCtx();
  ctx->slot = slot;
  ctx->lk = std::unique_lock<std::mutex>(slot->m);
  ctx->ordinal = slot->assigned.fetch_add(1) + 1;
  // The write's identity, for the take path's drop by tag.
  ros2_lf_binding::tag_log::on_publish(
    slot->topic, ros2_lf_binding::t_origin_set, true, slot->gid, ctx->ordinal);
  Origin origin = Origin::EDGE_NOT_ADMITTED;
  std::vector<uint32_t> edges;
  if (!ros2_lf_binding::t_origin_set) {
    slot->no_context.fetch_add(1, std::memory_order_relaxed);
  }
  if (slot->tracked) {
    // A loaned write takes the same Fast DDS sequence number as an
    // ordinary one (publication_identity_probe loaned-sequence), so only the
    // serialized path stays refused.
    if (api == RCLLF_GATE_SERIALIZED) {
      origin = Origin::UNSUPPORTED_API;
    } else if (!ros2_lf_binding::t_origin_set) {
      origin = Origin::NO_CONTEXT;
      slot->warn_pending.store(true, std::memory_order_relaxed);
    } else {
      auto it = slot->edges_by_rid.find(ros2_lf_binding::t_origin_rid);
      if (it == slot->edges_by_rid.end()) {
        origin = Origin::UNDECLARED_EFFECT;
      } else {
        origin = Origin::ADMITTED;
        edges = it->second;
#ifdef RCLLF_CORR_TEST_HOOKS
        rt::test_skip_predict(slot->gid, ctx->ordinal, &edges);
#endif
      }
    }
    if (origin == Origin::UNDECLARED_EFFECT || origin == Origin::UNSUPPORTED_API) {
      slot->refusal_pending.fetch_or(1u << static_cast<unsigned>(origin));
    }
  } else if (!ros2_lf_binding::t_origin_set) {
    origin = Origin::NO_CONTEXT;
  }
  // Tracked slots emit `publication` in post, untracked are counted.
  if (slot->tracked) {
    ctx->traced = slot->traced.load(std::memory_order_acquire);
    ctx->api = static_cast<uint8_t>(api);
    ctx->origin = origin;
    ctx->rid = ros2_lf_binding::t_origin_set ? ros2_lf_binding::t_origin_rid : RCLLF_TRACE_NO_RID;
    ctx->dispatch_id = ros2_lf_binding::t_dispatch_id;
    ctx->tag_time = ros2_lf_binding::t_dispatch_time;
    ctx->tag_micro = ros2_lf_binding::t_dispatch_micro;
    if (!ctx->traced && origin != Origin::ADMITTED) {
      slot->backlog_refused.fetch_add(1, std::memory_order_relaxed);
    }
  } else {
    slot->c_untracked.fetch_add(1, std::memory_order_relaxed);
  }
  // the publishing reaction's tag fixes the cutoff before the
  // real publish, so a sample can never be taken before its cutoff exists.
  const int64_t tag = origin == Origin::ADMITTED ? ros2_lf_binding::lf_current_tag().first : -1;
  const int64_t act =
    origin == Origin::ADMITTED && ros2_lf_binding::t_origin_set ?
    ros2_lf_binding::corr_rt::origin_phys() : -1;
  if (origin == Origin::ADMITTED) {
    // A local_now edge is decided when this publish returns.
    for (uint32_t e : edges) {
      if (st.table->local_now(e)) {
        ctx->local_edges.push_back(e);
      }
    }
  }
  st.table->predict(slot->gid, ctx->ordinal, origin, std::move(edges), slot->tracked, tag, act);
  if (!ctx->local_edges.empty() && ros2_lf_binding::corr_rt::local_delivery_lock()) {
    // The delivery lock. No other take reaches these readers from
    // here until this publication's samples are classified, so a later
    // sample can never overwrite this one in a depth-1 reader first. Order:
    // the slot lock, then the take mutexes by lead edge, then the table.
    ctx->local_take =
      ros2_lf_binding::IngressRole::instance().lock_local_readers(ctx->local_edges);
  }
  output_deadline_check(slot->topic);
  return ctx;
}

void rcllf_gate_payload(const void * publisher, const void * payload, int api)
{
  // Off unless RCLLF_PLANT_RECORD is set (one atomic load).
  if (!ros2_lf_binding::plant_record::enabled()) {
    return;
  }
  const int rid = ros2_lf_binding::t_origin_set ? ros2_lf_binding::t_origin_rid : -1;
  auto slot = ros2_lf_binding::slot_for(publisher, false);
  if (!slot || !ros2_lf_binding::output_deadline::armed(rid, slot->topic)) {
    return;  // only the bounded (actuator) publishers are recorded
  }
  ros2_lf_binding::plant_record::publication(
    publisher, payload, api == RCLLF_GATE_SERIALIZED, rid, slot->topic);
}

void rcllf_gate_post(void * ctx_void, int ret)
{
  if (ctx_void == nullptr) {
    return;
  }
  auto * ctx = static_cast<ros2_lf_binding::GateCtx *>(ctx_void);
  const bool ok = ret == RCL_RET_OK;
  // The event precedes commit, so `publication` precedes its
  // `expectation` events in the delivery stream. Still under the slot mutex.
  if (ctx->traced) {
    auto & s = *ctx->slot;
    (ok ? s.c_accepted : s.c_failed).fetch_add(1, std::memory_order_relaxed);
    switch (ctx->origin) {
      case Origin::NO_CONTEXT: s.c_no_context.fetch_add(1); break;
      case Origin::UNDECLARED_EFFECT: s.c_undeclared.fetch_add(1); break;
      case Origin::EDGE_NOT_ADMITTED: s.c_not_admitted.fetch_add(1); break;
      case Origin::UNSUPPORTED_API: s.c_unsupported.fetch_add(1); break;
      default: break;
    }
    rcllf_tp_publication e{};
    ros2_lf_binding::fill_gid(e.gid, s.gid);
    e.ordinal = ctx->ordinal;
    e.result = ok ? RCLLF_PUB_ACCEPTED : RCLLF_PUB_FAILED;
    e.api = ctx->api;
    e.origin = static_cast<uint8_t>(ctx->origin);
    e.rid = ctx->rid;
    e.dispatch_id = ctx->dispatch_id;
    e.tag_time = ctx->tag_time;
    e.tag_microstep = ctx->tag_micro;
    ros2_lf_binding::semtrace::publication(e);
  }
  ros2_lf_binding::state().table->commit(ctx->slot->gid, ctx->ordinal, ok);
  if (ctx->local_edges.empty()) {
    delete ctx;
    return;
  }
  {
    // The publish-time take. With SYNCHRONOUS publication the
    // sample is in each local reader now (S0 premise probe), so every local
    // expectation is decided here: MATCHED, or LOST local_undelivered.
    ros2_lf_binding::corr_rt::WarningsDeferred defer;
    if (ok) {
      ros2_lf_binding::corr_rt::run_after_publish_hook();
      if (ctx->local_take.readers() == 0 &&
        !ros2_lf_binding::corr_rt::local_delivery_lock())
      {
        ctx->local_take =
          ros2_lf_binding::IngressRole::instance().lock_local_readers(ctx->local_edges);
      }
      ros2_lf_binding::corr_rt::take_and_settle_local(
        ctx->slot->gid, ctx->ordinal, ctx->local_take);
    }
    ctx->local_take.release();
    ctx->lk.unlock();
  }
  delete ctx;
  ros2_lf_binding::corr_rt::print_pending_warnings();
}

void rcllf_gate_nested(const void * publisher)
{
  auto & st = ros2_lf_binding::state();
  st.nested.fetch_add(1);
  if (st.closed.load(std::memory_order_acquire)) {
    st.after_close.fetch_add(1);
    return;
  }
  // No lock beyond the registry lookup: the outer call may hold this slot's
  // mutex. The ordinal still advances, so the writer's sequence stays exact.
  auto slot = ros2_lf_binding::slot_for(publisher, true);
  // A nested write is a publish of this thread's invoke too. Only
  // this path sees it (t_in_gate keeps it out of rcllf_gate_pre), and the
  // count is thread-local, so it needs no slot lock.
  ros2_lf_binding::corr_rt::topic_sent_note(slot->topic);
  if (slot->gid_ok) {
    const uint64_t ordinal = slot->assigned.fetch_add(1) + 1;
    // Lock-free NESTED publication (a UST tracepoint never publishes).
    // An ordinal assigned before the handover is backlog-covered even when
    // `traced` flipped in between (first_traced_ordinal is published by the
    // release store of `traced`).
    if (slot->tracked && slot->traced.load(std::memory_order_acquire) &&
      ordinal >= slot->first_traced_ordinal)
    {
      slot->c_nested.fetch_add(1, std::memory_order_relaxed);
      rcllf_tp_publication e{};
      ros2_lf_binding::fill_gid(e.gid, slot->gid);
      e.ordinal = ordinal;
      e.result = RCLLF_PUB_ACCEPTED;
      e.api = RCLLF_API_PUBLISH;
      e.origin = RCLLF_ORIGIN_NESTED;
      e.rid = ros2_lf_binding::t_origin_set ? ros2_lf_binding::t_origin_rid : RCLLF_TRACE_NO_RID;
      e.dispatch_id = ros2_lf_binding::t_dispatch_id;
      e.tag_time = ros2_lf_binding::t_dispatch_time;
      e.tag_microstep = ros2_lf_binding::t_dispatch_micro;
      ros2_lf_binding::semtrace::publication(e);
    } else if (!slot->tracked) {
      slot->c_untracked.fetch_add(1, std::memory_order_relaxed);
    }
    st.table->note_nested(slot->gid, ordinal, slot->tracked);
  }
  output_deadline_check(slot->topic);
}

void rcllf_gate_on_fini(const void * publisher)
{
  ros2_lf_binding::plant_record::forget_publisher(publisher);
  auto & st = ros2_lf_binding::state();
  std::shared_ptr<ros2_lf_binding::Slot> slot;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    auto it = st.slots.find(publisher);
    if (it == st.slots.end()) {
      return;
    }
    slot = it->second;
    st.slots.erase(it);  // address reuse by a later publisher is a new slot
  }
  if (slot->gid_ok && !st.closed.load()) {
    st.table->finalize(slot->gid);
  }
}

uint32_t ros2_lf_corr_emit(void * /*self*/, int edge)
{
  uint32_t fresh = 0;
  const bool traced = ros2_lf_binding::trace_callbacks_enabled();
  std::vector<ros2_lf_binding::corr::Token> tokens;
  const uint32_t n =
    rt::table().emit(static_cast<uint32_t>(edge), &fresh, traced ? &tokens : nullptr);
  if (fresh > 0) {  // the trace carries this reaction's records
    rcllf_tp_carrier c{};
    c.edge = static_cast<uint32_t>(edge);
    c.op = RCLLF_CARRIER_EMIT;
    c.count = fresh;
    c.release_tag_time = ros2_lf_binding::lf_current_tag().first;
    ros2_lf_binding::semtrace::carrier(c);
    // Each fresh record by (gid, ordinal), so a publication's emit tag
    // joins its sample, dispatch or refusal line.
    std::string ids;
    for (const auto & t : tokens) {
      ids += std::string(ids.empty() ? "" : ",") + "{\"gid\":\"" +
        ros2_lf_binding::corr::gid_hex(t.gid) + "\",\"ordinal\":" + std::to_string(t.ordinal) + "}";
    }
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"emit\",\"edge\":" + std::to_string(edge) + ",\"count\":" +
      std::to_string(fresh) + "," + ros2_lf_binding::tag_json() + ",\"tokens\":[" + ids + "]}");
  }
  rt::print_pending_warnings();
  return n;
}

int64_t ros2_lf_corr_arrive(void * /*self*/, int edge, uint32_t count)
{
  auto & st = ros2_lf_binding::state();
  int64_t d = 0;
  {
    std::lock_guard<std::mutex> lk(st.reg_mu);
    if (edge >= 0 && static_cast<size_t>(edge) < st.edges.size()) {
      d = std::max<int64_t>(0, st.edges[edge].spec.delay_ns);
    }
  }
  // The publication microstep orders a fan-in group's members at d = 0.
  const auto tag = ros2_lf_binding::lf_current_tag();
  const int64_t now = tag.first;
  // A deferred edge releases at the cutoff tag, publish tag + d + T.
  // The release tag is computed once, checked, and used for the trace and
  // the table alike.
  int64_t release = 0;
  if (!rt::release_tag(
      now, d, rt::table().release_extra_ns(static_cast<uint32_t>(edge)), &d, &release))
  {
    rt::release_overflow(edge, now);
    return -1;
  }
  if (count > 0) {
    rcllf_tp_carrier c{};
    c.edge = static_cast<uint32_t>(edge);
    c.op = RCLLF_CARRIER_ARRIVE;
    c.count = count;
    c.release_tag_time = release;
    ros2_lf_binding::semtrace::carrier(c);
    trace_release_key(edge, count, tag.second, release, false);
  }
  return rt::table().arrive(static_cast<uint32_t>(edge), count, release, tag.second) ? d : -1;
}

int ros2_lf_corr_flush_due(int edge)
{
  const int64_t now = ros2_lf_binding::lf_current_tag().first;
  return rt::table().flush_due(static_cast<uint32_t>(edge), now) ? 1 : 0;
}

void ros2_lf_corr_flush_refused(int edge)
{
  rt::table().flush_refused(static_cast<uint32_t>(edge));
  const int64_t now = ros2_lf_binding::lf_current_tag().first;
  if (ros2_lf_binding::lf_stop_tag().first <= now) {
    // The stop tag is at this time, the flush at now + 1 ns is past it.
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding.corr"),
      "rcllf corr: edge %d flush falls after the stop tag, its records are retained at stop",
      edge);
    return;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger("ros2_lf_binding.corr"),
    "rcllf corr: edge %d flush not scheduled (past the stop tag?), its records stay unsent", edge);
}

}  // extern "C"

namespace
{

/// Fill `out` with the edge's records of `tag` (the flush, or the d = 0 send).
/// `flush` names the flush time in the STALE exit.
uint32_t send_xcorr(int edge, std::pair<int64_t, uint32_t> tag, bool flush, rcllf_xcorr_t * out)
{
  using ros2_lf_binding::corr::EmitStatus;
  std::vector<ros2_lf_binding::corr::XRecord> recs;
  uint32_t fresh = 0;
  uint64_t complete_seq = 0;
  // A send at the publishing tag is one message per microstep; only
  // the flush (decentralized d > 0, or RCLLF_XFED_FLUSH=1) sends one per time.
  // Both carry a completion number.
  const EmitStatus status = rt::table().emit_remote(
    static_cast<uint32_t>(edge), tag.first, tag.second, RCLLF_XCORR_MAX, &recs, &fresh, !flush,
    &complete_seq);
  if (status == EmitStatus::COLLAPSE) {
    xfed_fatal(
      "edge " + std::to_string(edge) + " cross-federate records at time " +
      std::to_string(tag.first) + " sent twice (microstep collapse)");
  }
  if (status == EmitStatus::STALE) {
    xfed_fatal(
      "edge " + std::to_string(edge) +
      (flush ? " flush at time " + std::to_string(tag.first + 1) : " send at time " +
      std::to_string(tag.first)) +
      " found a cross-federate record not of time " + std::to_string(tag.first) +
      (flush ? " (lost or late flush)" : ""));
  }
  if (status == EmitStatus::CAP) {
    xfed_fatal(
      "edge " + std::to_string(edge) + " has " + std::to_string(recs.size()) +
      " cross-federate records at time " + std::to_string(tag.first) + ", above " +
      std::to_string(RCLLF_XCORR_MAX) + " (xfed_record_cap)");
  }
  if (recs.empty()) {
    // This activation published nothing on
    // the edge, so there is no message and no completion number.
    rt::print_pending_warnings();
    return 0;
  }
  std::memset(out, 0, sizeof(*out));
  out->magic = RCLLF_XCORR_MAGIC;
  out->epoch = rt::table().epoch();
  out->pub_time = tag.first;
  // The d = 0 send names its microstep; the flush's records of
  // every microstep land on (t + d, 0), where the microstep has no role.
  out->pub_microstep = flush ? 0 : tag.second;
  std::memcpy(out->boot_id, host_boot_id().data(), sizeof(out->boot_id));
  rt::pack_xcorr(recs, out);
  out->send_phys = rt::table_clock();
  // The send at the publishing tag is the activation's
  // COMPLETE record. Changed 2026-11-03: only an activation that
  // published on the edge sends one, so the numbers count real messages and a
  // gap is still a message that never came (CONTROL_FAILURE). The
  // Flush is one COMPLETE record per time, numbered the same way.
  out->flags = RCLLF_XCORR_COMPLETE;
  out->complete_seq = complete_seq;
  if (rt::drop_complete_for_test(static_cast<uint32_t>(edge), out->complete_seq)) {
    rt::print_pending_warnings();
    return 0;
  }
  if (fresh > 0) {
    rcllf_tp_carrier c{};
    c.edge = static_cast<uint32_t>(edge);
    c.op = RCLLF_CARRIER_EMIT;
    c.count = fresh;
    c.release_tag_time = tag.first;
    ros2_lf_binding::semtrace::carrier(c);
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"emit_remote\",\"edge\":" + std::to_string(edge) +
      ",\"count\":" + std::to_string(fresh) + ",\"records\":" + std::to_string(recs.size()) +
      "," + ros2_lf_binding::tag_json() + "}");
  }
  rt::print_pending_warnings();
  return 1;
}

}  // namespace

namespace
{
/// One trace line per record a refused drain ends, keyed by (gid,
/// ordinal), with its release tag, the stop tag and the reason. A carrier
/// record ends schedule_refused, a terminal one (already lost) undelivered.
void trace_refused_entries(uint32_t m)
{
  std::vector<ros2_lf_binding::corr::Queued> terminal;
  const auto carrier = rt::table().carrier_entries(m, &terminal);
  const int64_t start = ros2_lf_binding::lf_start_tag().first;
  const auto stop = ros2_lf_binding::lf_stop_tag();
  auto line = [&](const ros2_lf_binding::corr::Queued & q, const char * outcome) {
      const bool past = q.release_ns > stop.first ||
        (q.release_ns == stop.first && q.release_micro > stop.second);
      const char * reason =
        past ? "release tag after the stop tag" : "lf_schedule refused at or before the stop tag";
      ros2_lf_binding::trace_corr(
        "{\"event\":\"corr\",\"kind\":\"refused\",\"edge\":" + std::to_string(m) +
        ",\"gid\":\"" + ros2_lf_binding::corr::gid_hex(q.token.gid) + "\",\"ordinal\":" +
        std::to_string(q.token.ordinal) + ",\"outcome\":\"" + outcome +
        "\",\"release_elapsed\":" + std::to_string(q.release_ns - start) +
        ",\"release_microstep\":" + std::to_string(q.release_micro) + ",\"stop_elapsed\":" +
        std::to_string(stop.first - start) + ",\"stop_microstep\":" + std::to_string(stop.second) +
        ",\"reason\":\"" + reason + "\"," + ros2_lf_binding::tag_json() + "}");
    };
  for (const auto & q : carrier) {
    line(q, "schedule_refused");
  }
  for (const auto & q : terminal) {
    line(q, "undelivered");
  }
}
}  // namespace

extern "C" {

uint32_t ros2_lf_corr_flush_remote(void * /*self*/, int edge, rcllf_xcorr_t * out)
{
  // The flush runs at (t + 1 ns, 0) and sends the records of time t.
  auto tag = ros2_lf_binding::lf_current_tag();
  tag.first -= 1;
  return send_xcorr(edge, tag, true, out);
}

uint32_t ros2_lf_corr_send_remote(void * /*self*/, int edge, rcllf_xcorr_t * out)
{
  // d = 0, the publishing reaction sends the records of its own tag
  // over `after 0`; they land on (t, m + 1). d > 0: the same over
  // `after d`; they land on (t + d, k) for the k-th message of t.
  // flush_due only takes burst count of this reaction, its result is
  // not needed.
  const auto tag = ros2_lf_binding::lf_current_tag();
  rt::table().flush_due(static_cast<uint32_t>(edge), tag.first);
  return send_xcorr(edge, tag, false, out);
}

void ros2_lf_xfed_legacy_sent(const char * port, uint32_t max_per_time)
{
  static rt::LegacyGuard guard;
  static std::mutex mu;
  const auto tag = ros2_lf_binding::lf_current_tag();
  uint32_t n = 0;
  {
    std::lock_guard<std::mutex> lk(mu);
    n = guard.note(port, tag.first, tag.second);
  }
  if (n > max_per_time) {
    xfed_fatal(
      std::string("legacy cross-federate port '") + port + "' written at " + std::to_string(n) +
      " microsteps of time " + std::to_string(tag.first) + ". Its `after` link maps them to one "
      "tag and " + (max_per_time < rt::LegacyGuard::kMaxPerTime ?
      "under decentralized coordination the receiver judges the second message tardy and "
      "moves or drops it" :
      "the receiver would merge the third message"));
  }
}

}  // extern "C"

namespace
{

/// Account one COMPLETE message on ingress edge `edge` at tag time
/// `now`. Traces the completion and every CONTROL_FAILURE (a completion
/// sequence number that never arrived, so the receiver cannot tell what that
/// activation published), applies control_failure_policy, and returns whether
/// the message's records are new (false: a repeated completion number).
bool note_complete_message(int edge, const rcllf_xcorr_t & x, int64_t now)
{
  const std::string E = std::to_string(edge);
  const auto res = rt::table().note_complete(static_cast<uint32_t>(edge), x.complete_seq);
  // The sender stamped send_phys with its Table clock right before lf_set,
  // so this is the time from send to settle on this clock.
  const int64_t wait = rt::table_clock() - x.send_phys;
  ros2_lf_binding::trace_corr(
    "{\"event\":\"corr\",\"kind\":\"complete\",\"edge\":" + E + ",\"seq\":" +
    std::to_string(x.complete_seq) + ",\"count\":" + std::to_string(x.count) +
    ",\"pub_ns\":" + std::to_string(x.pub_time) +
    ",\"duplicate\":" + (res.duplicate ? "true" : "false") +
    ",\"settle_wait_ns\":" + std::to_string(wait) + "," + ros2_lf_binding::tag_json() + "}");
  for (const uint64_t seq : res.control_failures) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding.corr"),
      "rcllf corr: CONTROL_FAILURE edge %d: sender activation %llu has no COMPLETE record "
      "(next seen %llu at time %lld)", edge,
      static_cast<unsigned long long>(seq),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(x.complete_seq),  // NOLINT(runtime/int)
      static_cast<long long>(now));  // NOLINT(runtime/int)
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"control_failure\",\"edge\":" + E +
      ",\"seq\":" + std::to_string(seq) + ",\"next_seq\":" + std::to_string(x.complete_seq) +
      "," + ros2_lf_binding::tag_json() + "}");
  }
  if (!res.control_failures.empty() && edge_control_stop(edge)) {
    control_failure_stop(edge);
  }
  // A repeated completion already arrived its records once.
  return !res.duplicate;
}

int64_t arrive_remote_impl(int edge, const rcllf_xcorr_t * x)
{
  const int64_t now = ros2_lf_binding::lf_current_tag().first;
  const std::string E = std::to_string(edge);
  // Every message carries one or more records. An
  // activation that published nothing sends no message, COMPLETE or not.
  const bool complete = (x->flags & RCLLF_XCORR_COMPLETE) != 0;
  if (x->magic != RCLLF_XCORR_MAGIC || x->count == 0 || x->count > RCLLF_XCORR_MAX) {
    xfed_fatal(
      "edge " + E + " cross-federate message with magic " + std::to_string(x->magic) +
      " and count " + std::to_string(x->count) + " (layout or byte-order mismatch)");
  }
  const int64_t d = edge_delay_ns(edge);
  // On a decentralized link the RTI gives no
  // guarantee, so a record after its intended tag is dropped and its message
  // reported lost at this tag. Centralized it stays fatal.
  const int64_t due = x->pub_time + d;
  const bool tardy = now > due && edge_tardy_drop(edge);
  if (due != now && !tardy) {
    xfed_fatal(
      "edge " + E + " cross-federate records of time " + std::to_string(x->pub_time) +
      " arrived at " + std::to_string(now) + ", not at publish time + " + std::to_string(d) +
      " ns");
  }
  // Over a zero-delay federated connection the records arrive at their
  // publication tag (t, m) itself, microstep included.
  const uint32_t now_micro = ros2_lf_binding::lf_current_tag().second;
  if (!tardy && rt::table().zero_link(static_cast<uint32_t>(edge)) &&
    now_micro != x->pub_microstep)
  {
    xfed_fatal(
      "edge " + E + " cross-federate records of tag (" + std::to_string(x->pub_time) + ", " +
      std::to_string(x->pub_microstep) + ") arrived at microstep " + std::to_string(now_micro) +
      ", not at their publication tag over a zero-delay link");
  }
  // Another host needs a declared bound eps on the offset of the clocks.
  static const rt::XfedClockBound bound = rt::xfed_clock_bound();
  static const std::optional<std::array<uint8_t, 16>> own = rt::read_boot_id(rt::kBootIdPath);
  const bool same_boot = rt::xfed_same_boot(x->boot_id, own);
  const std::string host = rt::xfed_host_problem(same_boot, bound);
  if (!host.empty()) {
    xfed_fatal("edge " + E + " " + host);
  }
  // Startup set the declared bound, the first record tells whether the
  // publisher shares this clock.
  const int64_t eps =
    rt::xfed_judge_bound(rt::table(), static_cast<uint32_t>(edge), same_boot, bound);
  if (complete && !note_complete_message(edge, *x, now)) {
    return -1;
  }
  const std::vector<ros2_lf_binding::corr::XRecord> recs = rt::unpack_xcorr(*x);
  // The publisher stamped r.phys with its Table clock, the
  // logical tag in a fast federate, so compare with this federate's own.
  const int64_t phys_now = rt::table_clock();
  for (const auto & r : recs) {
    if (!rt::xfed_record_beyond_bound(r.phys, phys_now, eps)) {
      continue;
    }
    if (!bound.declared) {
      xfed_fatal(
        "edge " + E + " record stamped at " + std::to_string(r.phys) +
        " ns, after this host's physical time");
    }
    rt::table().note_clock_skew(static_cast<uint32_t>(edge));
  }
  rcllf_tp_carrier c{};
  c.edge = static_cast<uint32_t>(edge);
  c.op = RCLLF_CARRIER_ARRIVE;
  c.count = x->count;
  // A deferred ingress edge releases at its cutoff tag, now + T,
  // checked once and used for the trace and the table alike.
  int64_t extra = 0;
  int64_t release = 0;
  if (!rt::release_tag(
      now, 0, rt::table().release_extra_ns(static_cast<uint32_t>(edge)), &extra, &release))
  {
    rt::release_overflow(edge, now);
    return -1;
  }
  c.release_tag_time = release;
  ros2_lf_binding::semtrace::carrier(c);
  trace_release_key(edge, x->count, x->pub_microstep, release, true);
  const bool wake = rt::table().arrive_remote(
    static_cast<uint32_t>(edge), recs, x->pub_time, release, x->pub_microstep);
  if (tardy) {
    rt::table().terminate_tardy(static_cast<uint32_t>(edge), recs);
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"tardy\",\"edge\":" + E + ",\"count\":" +
      std::to_string(recs.size()) + ",\"intended_ns\":" + std::to_string(x->pub_time + d) +
      ",\"late_ns\":" + std::to_string(now - (x->pub_time + d)) + "," +
      ros2_lf_binding::tag_json() + "}");
  }
  return wake ? extra : -1;
}

/// The bound subscription of a drain's RID, or none.
rclcpp::SubscriptionBase::SharedPtr silence_sub(int reaction_id)
{
  if (reaction_id < 0 ||
    reaction_id >= static_cast<int>(ros2_lf_binding::g_handle_table.size()))
  {
    return nullptr;
  }
  ros2_lf_binding::bind_late_handle(reaction_id);  // as the dispatch does
  const auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  return entry.kind == ros2_lf_binding::HandleKind::SUBSCRIPTION ?
         entry.subscription.lock() : nullptr;
}

/// Arm the window once the drain's subscription is bound. It is armed
/// when the subscription declares a QoS deadline D and registered a
/// requested-deadline-missed handler (the declaration that the edge is
/// periodic), with its first window starting at the start tag.
void silence_decide(
  ros2_lf_binding::silence_deadline::Slot * slot, int reaction_id, int64_t start_ns)
{
  const auto sub = silence_sub(reaction_id);
  if (!sub) {
    return;  // not bound yet (a lifecycle node): decided at a later drain
  }
  slot->decided = true;
  const auto & handlers = sub->get_event_handlers();
  if (handlers.find(RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED) == handlers.end()) {
    return;
  }
  const int64_t d = ros2_lf_binding::silence_deadline::deadline_ns(
    sub->get_actual_qos().get_rmw_qos_profile().deadline);
  if (d > 0) {
    slot->window.arm(d, start_ns);
  }
}

/// Run the subscription's deadline handler once per window that ended
/// with no record. The first window started at `ref_ns` (the last record's tag
/// time or the start tag), each is `period_ns` long. Like a missing record's
/// call, it runs at this tag with a logical count, under the drain's
/// origin, and never when the lifecycle node is inactive or shutdown began.
void silence_fire(
  void * self, uint32_t edge, int reaction_id, uint32_t windows, int64_t ref_ns,
  int64_t period_ns, int64_t start_ns)
{
  const auto sub = silence_sub(reaction_id);
  if (!sub) {
    return;
  }
  const auto & handlers = sub->get_event_handlers();
  const auto it = handlers.find(RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
  if (it == handlers.end()) {
    return;
  }
  auto & entry = ros2_lf_binding::g_handle_table[reaction_id];
  auto & st = ros2_lf_binding::state();
  const bool active = ros2_lf_binding::lifecycle_admit(reaction_id);
  for (uint32_t k = 0; k < windows; ++k) {
    const int64_t end_ns = ref_ns + static_cast<int64_t>(k + 1) * period_ns;
    // No record, so no edge on the dispatch: outcome NONE, as a timer's.
    rt::DispatchTrace dt(reaction_id, RCLLF_ENTRY_CORR_SUB);
    ros2_lf_binding::UserCallbackScope user;
    const char * hook = "deadline";
    int32_t total = 0;
    if (!active) {
      hook = "inactive";
    } else if (!user.admitted()) {
      hook = "shutdown";
    } else {
      {
        std::lock_guard<std::mutex> lk(st.missing_mu);
        total = ++st.deadline_count.at(*sub);
      }
      dt.set_ran(true);
      ros2_lf_binding::trace_callback_start(
        "ros2_lf_corr_drain", reaction_id, entry.node_class, entry.callback_name);
      rt::OriginScope origin(self, reaction_id);
      rclcpp::QOSDeadlineRequestedInfo info{};
      info.total_count = total;
      info.total_count_change = 1;
      it->second->execute(std::make_shared<rclcpp::QOSDeadlineRequestedInfo>(info));
    }
    // Once per edge at WARN, so a silent producer is visible without traces.
    static std::mutex warned_mu;
    static std::set<uint32_t> warned;
    bool first = false;
    {
      std::lock_guard<std::mutex> lk(warned_mu);
      first = warned.insert(edge).second;
    }
    if (first) {
      RCLCPP_WARN(
        ros2_lf_binding::corr_logger(),
        "rcllf corr: edge %u (%s on '%s') delivered no record in the %lld ns deadline window "
        "ending at elapsed %lld ns; its deadline handler runs at every further window while "
        "the edge stays silent", edge, entry.node_class.c_str(),
        sub->get_topic_name(), static_cast<long long>(period_ns),  // NOLINT(runtime/int)
        static_cast<long long>(end_ns - start_ns));  // NOLINT(runtime/int)
    }
    const int64_t window_start = end_ns - period_ns - start_ns;
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"silence_deadline\",\"edge\":" +
      std::to_string(edge) + ",\"window_start_elapsed\":" + std::to_string(window_start) +
      ",\"window_end_elapsed\":" + std::to_string(end_ns - start_ns) + ",\"period_ns\":" +
      std::to_string(period_ns) + ",\"hook\":\"" + hook + "\",\"total\":" +
      std::to_string(total) + "," + ros2_lf_binding::tag_json() + "}");
  }
}

}  // namespace

extern "C" {

int64_t ros2_lf_corr_arrive_remote(void * /*self*/, int edge, const rcllf_xcorr_t * x)
{
  return arrive_remote_impl(edge, x);
}

int ros2_lf_message_lost_context(rcllf_lost_ctx_t * out)
{
  if (!t_lost_ctx_valid || out == nullptr) {
    return 0;
  }
  *out = t_lost_ctx;
  return 1;
}

int64_t ros2_lf_corr_drain(void * self, int edge, int reaction_id, const char * /*topic*/)
{
  if (!ros2_lf_binding::require_lf_worker("ros2_lf_corr_drain", reaction_id)) {
    return -1;
  }
  // The drain reaction raises its legacy stub ports by the
  // publishes of this drain only (ros2_lf_topic_sent), as an invoke does.
  ros2_lf_binding::corr_rt::topic_sent_reset();
  ros2_lf_binding::lf_time_advance_ros_clock();
  const auto e = static_cast<uint32_t>(edge);
  const auto tag = ros2_lf_binding::lf_current_tag();
  const int64_t now = tag.first;
  // worker lag (physical minus logical time) at drain start, after
  // the join and at the single exit, whether or not a token was drained.
  auto note_lag = [&](const char * point) {
      if (rt::fast_federate()) {
        return;  // physical minus logical time means nothing in fast mode
      }
      rt::table().note_lag(e, now, ros2_lf_binding::lf_time_physical() - now, point);
    };
  // The logical deadline window of this subscription
  // (silence_deadline.hpp). A silence tick (the startup arm at the start tag,
  // or a window end this edge scheduled) that drains no record is no
  // dispatch, so its worker lag is not noted: a drain that only checks the
  // window leaves the lag statistics unchanged.
  namespace sd = ros2_lf_binding::silence_deadline;
  sd::Slot & silence = sd::slot(e);
  // The queued event of this tag, if one triggered this drain, is spent.
  sd::drained(silence, tag);
  const int64_t start_ns = std::max<int64_t>(0, ros2_lf_binding::lf_start_tag().first);
  const bool first_drain = !silence.started;
  silence.started = true;
  const bool tick = silence.ticks.erase(now) > 0 || (first_drain && now == start_ns);
  const int64_t entry_lag =
    tick && !rt::fast_federate() ? ros2_lf_binding::lf_time_physical() - now : 0;
  bool lag_started = !tick;
  if (!tick) {
    note_lag("start");
  }
  if (!silence.decided) {
    silence_decide(&silence, reaction_id, start_ns);
  }
  auto & st = ros2_lf_binding::state();
  // Due when its release tag is at most this tag.
  // Every record due at this tag is delivered
  // at this tag, in one firing, one callback (or missing handler) invocation
  // per record, in (release tag, rank, ordinal) order. The wake below re-arms
  // only for records released later than this tag.
  for (auto tok = rt::table().pop_released(e, now, tag.second); tok;
    tok = rt::table().pop_released(e, now, tag.second))
  {
    // `e` leads a fan-in group, the token's own edge is its member's.
    // The queue, the wake, the reader and the lag stay on the lead.
    const uint32_t te = tok->edge;
    if (!lag_started) {
      lag_started = true;
      if (!rt::fast_federate()) {
        rt::table().note_lag(e, now, entry_lag, "start");
      }
    }
    // Any record, matched or missing, restarts the window; a missing
    // one runs the deadline handler below, so its window never reports again.
    silence.window.on_record(now);
    rcllf_tp_carrier c{};
    c.edge = te;
    c.op = RCLLF_CARRIER_DRAIN;
    c.count = 1;
    c.release_tag_time = now;
    ros2_lf_binding::semtrace::carrier(c);
    rt::DispatchTrace dt(reaction_id, RCLLF_ENTRY_CORR_SUB, te, &tok->gid, tok->ordinal);
    const auto t0 = std::chrono::steady_clock::now();
    Outcome outcome = Outcome::UNBOUND;
    const char * missing_hook = "none";
    uint8_t missing_hook_code = RCLLF_MISSING_HOOK_NONE;  // wire copy
    std::vector<std::string> missing_hooks;  // every hook that ran, in call order
    ros2_lf_binding::HandleEntry * entry = nullptr;
    bool active = true;
    if (reaction_id >= 0 &&
      reaction_id < static_cast<int>(ros2_lf_binding::g_handle_table.size()))
    {
      // A lifecycle node that is not active at this tag joins the
      // sample, so it does not linger, and runs no callback.
      active = ros2_lf_binding::lifecycle_admit(reaction_id);
      ros2_lf_binding::bind_late_handle(reaction_id);
      entry = &ros2_lf_binding::g_handle_table[reaction_id];
    }
    rclcpp::SubscriptionBase::SharedPtr sub;
    if (entry != nullptr && entry->kind == ros2_lf_binding::HandleKind::SUBSCRIPTION) {
      sub = entry->subscription.lock();
    }
    if (const auto settled = rt::table().rebind_settle(*tok, sub != nullptr)) {
      // A lifecycle subscription made or reset in a hook. Its record
      // was published to a reader the node dropped or replaced, or before any
      // reader existed, or the edge refused its reader: settled, never joined.
      outcome = *settled;
    } else if (!sub) {
      // A late entry's lifecycle node has no subscription now, so a stock
      // node would not receive the sample either: no ordering defect.
      if (entry == nullptr || !entry->late) {
        ros2_lf_binding::trace_unbound("ros2_lf_corr_drain", reaction_id);
      }
      rt::table().mark(*tok, Outcome::UNBOUND);
    } else {
      // A local edge was decided when its publish returned (the
      // publish-time take), so its join never waits.
      const bool local = rt::table().local_now(te);
      const int64_t bound_ms = local ? 0 :
        ros2_lf_binding::take_timeout_ms_env().value_or(entry->take_timeout_ms);
      ros2_lf_binding::corr::Sample sample;
      uint64_t retained_id = 0;
      outcome = rt::table().join(
        *tok, std::chrono::milliseconds(bound_ms), &sample, &retained_id);
      note_lag("join");
      ros2_lf_binding::stop_progress();  // a dispatch that returns is progress
      dt.set_join_wait(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - t0).count());
      dt.set_retained(retained_id);
      // A matched sample after shutdown began is not handed to user code.
      ros2_lf_binding::UserCallbackScope user;
      if (!active) {
        ros2_lf_binding::trace_lifecycle_gated(
          "ros2_lf_corr_drain", reaction_id, entry->node_class, entry->callback_name,
          outcome == Outcome::MATCHED);
      } else if (outcome == Outcome::MATCHED && user.admitted()) {
        dt.set_ran(true);
        const auto info = std::static_pointer_cast<rclcpp::MessageInfo>(sample.info);
        ros2_lf_binding::trace_callback_start(
          "ros2_lf_corr_drain", reaction_id, entry->node_class, entry->callback_name,
          info->get_rmw_message_info());
        rt::OriginScope origin(self, reaction_id);
        // A BNA policy reads this dispatch's tag and key in add<i>.
        ros2_lf_binding::bna_rt::DispatchScope bna(
          reaction_id, ros2_lf_binding::corr::gid_hex(tok->gid), tok->ordinal);
        // A run closer than the declared min input period is counted.
        ros2_lf_binding::input_period::observe_message(reaction_id, *info);
        sub->handle_message(sample.message, *info);
      } else if (ros2_lf_binding::corr::is_missing(outcome)) {
        // the missing message is delivered at its tag through the
        // subscription's requested-deadline-missed handler, with a logical
        // count (one per missing message), never a physical deadline.
        // The message_lost handler, if registered, runs in the same
        // reaction with the same count. Neither handler is ever waited on, so
        // no physical DDS event reaches either one.
        const auto & handlers = sub->get_event_handlers();
        auto handler = [&handlers](rcl_subscription_event_type_t t) {
            auto it = handlers.find(t);
            return it == handlers.end() ? nullptr : it->second;
          };
        const auto h_deadline = handler(RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
        const auto h_lost = handler(RCL_SUBSCRIPTION_MESSAGE_LOST);
        if (!h_deadline && !h_lost) {
          missing_hook = "none";
          missing_hook_code = RCLLF_MISSING_HOOK_NONE;
        } else if (!user.admitted()) {
          missing_hook = "shutdown";
          missing_hook_code = RCLLF_MISSING_HOOK_SHUTDOWN;
        } else {
          int32_t total = 0;
          int32_t deadline_total = 0;
          {
            std::lock_guard<std::mutex> lk(st.missing_mu);
            total = ++st.missing_count.at(*sub);
            deadline_total = ++st.deadline_count.at(*sub);
          }
          dt.set_ran(true);
          ros2_lf_binding::trace_callback_start(
            "ros2_lf_corr_drain", reaction_id, entry->node_class, entry->callback_name);
          rt::OriginScope origin(self, reaction_id);
          // The intended tag of the missing message, for the callbacks
          // (ros2_lf_message_lost_context).
          const int64_t pt = rt::table().pub_tag(*tok);
          rcllf_lost_ctx_t ctx{};
          const int64_t t_start = std::max<int64_t>(0, ros2_lf_binding::lf_start_tag().first);
          ctx.intended_elapsed_ns =
            pt < 0 ? -1 : pt + edge_delay_ns(static_cast<int>(te)) - t_start;
          ctx.ordinal = tok->ordinal;
          std::memcpy(ctx.gid, tok->gid.data(), sizeof(ctx.gid));
          ctx.tardy = rt::table().is_tardy(*tok) ? 1 : 0;
          // The physical time the missing-message handlers ran,
          // on the trace line below. The clock is
          // read only when that line is written (RCLLF_TRACE_CALLBACKS=1), so
          // a run without the trace makes no extra clock call around its
          // handlers.
          const bool timed = ros2_lf_binding::trace_callbacks_enabled();
          std::chrono::steady_clock::time_point h0{};
          if (timed) {
            h0 = std::chrono::steady_clock::now();
          }
          {
            LostCtxScope lost_ctx(ctx);
            if (h_deadline) {
              rclcpp::QOSDeadlineRequestedInfo info{};
              info.total_count = deadline_total;
              info.total_count_change = 1;
              h_deadline->execute(std::make_shared<rclcpp::QOSDeadlineRequestedInfo>(info));
              missing_hooks.emplace_back("deadline");
            }
            // The deadline callback may itself begin shutdown, and the gate closes
            // synchronously in rclcpp's pre-shutdown callback. Recheck
            // it so no user callback runs after shutdown has begun.
            if (h_lost && !ros2_lf_binding::g_user_callbacks_closed.load()) {
              rclcpp::QOSMessageLostInfo info{};
              info.total_count = static_cast<size_t>(total);
              info.total_count_change = 1;
              h_lost->execute(std::make_shared<rclcpp::QOSMessageLostInfo>(info));
              missing_hooks.emplace_back("message_lost");
            }
          }
          const int64_t handler_ns = timed ?
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - h0).count() : -1;
          // Backward compatible: the first callback that ran, deadline first.
          missing_hook = h_deadline ? "deadline" : "message_lost";
          missing_hook_code =
            h_deadline ? RCLLF_MISSING_HOOK_DEADLINE : RCLLF_MISSING_HOOK_MESSAGE_LOST;
          // One record per missing message whose hooks completed,
          // for the oracle. `hooks` lists every callback that ran.
          if (timed) {
            ros2_lf_binding::trace_corr(
              "{\"event\":\"corr\",\"kind\":\"missing_hook\",\"edge\":" + std::to_string(te) +
              ",\"gid\":\"" + ros2_lf_binding::corr::gid_hex(tok->gid) + "\",\"ordinal\":" +
              std::to_string(tok->ordinal) + ",\"total\":" + std::to_string(total) +
              ",\"hooks\":" + hooks_json(missing_hooks) + ",\"handler_ns\":" +
              std::to_string(handler_ns) + "}");
          }
        }
        // Fix 6: after the handlers, a declared BNA input drain reports
        // the missing sample to its engine, which ends it as data, not as a
        // second handler call. With the lost message's
        // publish tag, which an input with a declared max_stamp_lag turns into
        // a lower bound of its header stamp.
        ros2_lf_binding::bna_rt::on_missing(
          reaction_id, ros2_lf_binding::corr::gid_hex(tok->gid), tok->ordinal,
          rt::table().pub_tag(*tok));
      }
    }
    dt.set_outcome(static_cast<uint8_t>(outcome));
    const auto wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - t0).count();
    const int64_t cutoff = rt::table().cutoff(*tok);
    const int64_t start = now - ros2_lf_binding::lf_elapsed_tag().first;
    // The JSON's cutoff and missing_hook, on the LTTng dispatch_end too.
    dt.set_cutoff(cutoff == ros2_lf_binding::corr::kNoCutoff ? -1 : cutoff);
    dt.set_missing_hook(
      ros2_lf_binding::corr::is_missing(outcome) ?
      missing_hook_code : static_cast<uint8_t>(RCLLF_MISSING_HOOK_NA));
    ros2_lf_binding::trace_corr(
      "{\"event\":\"corr\",\"kind\":\"dispatch\",\"edge\":" + std::to_string(te) +
      ",\"gid\":\"" + ros2_lf_binding::corr::gid_hex(tok->gid) + "\",\"ordinal\":" +
      std::to_string(tok->ordinal) + ",\"outcome\":\"" +
      ros2_lf_binding::corr::outcome_name(outcome) + "\"," + ros2_lf_binding::tag_json() +
      ",\"cutoff_elapsed\":" +
      std::to_string(cutoff == ros2_lf_binding::corr::kNoCutoff ? -1 : cutoff - start) +
      ",\"missing_hook\":\"" +
      (ros2_lf_binding::corr::is_missing(outcome) ? missing_hook : "n/a") +
      "\",\"missing_hooks\":" + hooks_json(missing_hooks) + ",\"wait_us\":" +
      std::to_string(wait_us) + "}");
    if (st.drain_delay_ms > 0 && tok->ordinal == st.drain_delay_ordinal) {
      bool empty = true;
      const auto n = static_cast<uint32_t>(rt::table().edge_count());
      for (uint32_t i = 0; i < n; ++i) {
        empty = empty && rt::table().carrier_size(i) == 0;
      }
      if (empty) {  // test hook: a slow worker in the last drain of its tag
        std::this_thread::sleep_for(std::chrono::milliseconds(st.drain_delay_ms));
      }
    }
    // B_e is per callback invocation, and this loop runs one per record.
    rt::table().end_invocation();
  }
  // The windows that ended by this tag with no record, one deadline
  // call each, after every record due at this tag was drained.
  const int64_t window_ref = silence.window.ref_ns();
  if (const uint32_t windows = silence.window.take_elapsed(now)) {
    silence_fire(self, e, reaction_id, windows, window_ref, silence.window.period_ns(), start_ns);
  }
  rt::print_pending_warnings();
  if (lag_started) {
    note_lag("complete");
  }
  const int64_t record_wake = rt::table().next_wake(e, now);
  const int64_t silence_wake =
    sd::silence_wake(silence.window, now, ros2_lf_binding::lf_stop_tag().first);
  const int64_t w = sd::combine_wake(record_wake, silence_wake);
  if (w >= 0 && w == silence_wake && w > 0) {
    silence.ticks.insert(now + w);
  }
  return w;
}

int64_t ros2_lf_corr_claim_wake(int edge, int64_t w)
{
  if (w < 0 || edge < 0) {
    return w;
  }
  return ros2_lf_binding::silence_deadline::claim_wake(
    ros2_lf_binding::silence_deadline::slot(static_cast<uint32_t>(edge)),
    ros2_lf_binding::lf_current_tag(), w);
}

void ros2_lf_corr_schedule_refused(int edge)
{
  // One log line per refusal with the current and the stop tag, as
  // diagnostic context. The line does not name the reason: reactor-c 2f63540d
  // lib/schedule.c returns 0 both past the stop tag and for a same-tag
  // conflict, so a stop tag still FOREVER rules the stop out, and a set stop
  // tag at the current tag only makes the stop the likely cause.
  {
    const auto tag = ros2_lf_binding::lf_current_tag();
    const auto stop = ros2_lf_binding::lf_stop_tag();
    RCLCPP_INFO(
      ros2_lf_binding::corr_logger(),
      "rcllf corr: {\"event\":\"schedule_refused\",\"edge\":%d,\"tag_ns\":%lld,\"micro\":%u,"
      "\"stop_ns\":%lld,\"stop_micro\":%u,\"carrier\":%zu}",
      edge, static_cast<long long>(tag.first), tag.second,  // NOLINT(runtime/int)
      static_cast<long long>(stop.first), stop.second,  // NOLINT(runtime/int)
      rt::table().carrier_size(static_cast<uint32_t>(edge)));
  }
  // The drain action of a fan-in group is its lead's; every member's
  // carrier ends refused, each traced on its own edge.
  for (const uint32_t m : rt::table().group_members(static_cast<uint32_t>(edge))) {
    rcllf_tp_carrier c{};
    c.edge = m;
    c.op = RCLLF_CARRIER_REFUSED;
    c.count = static_cast<uint32_t>(rt::table().carrier_size(m));
    c.release_tag_time = ros2_lf_binding::lf_current_tag().first;
    ros2_lf_binding::semtrace::carrier(c);
    if (ros2_lf_binding::trace_callbacks_enabled()) {
      trace_refused_entries(m);
    }
  }
  // The stop tag decides which refused tokens the stop ended; outside
  // LF (-1) no stop tag is known.
  const auto stop = ros2_lf_binding::lf_stop_tag();
  if (stop.first < 0) {
    rt::table().schedule_refused(static_cast<uint32_t>(edge));
  } else {
    rt::table().schedule_refused(static_cast<uint32_t>(edge), stop.first, stop.second);
  }
}

void ros2_lf_corr_close(int edge)
{
  // Only mark the edge. LF orders the reactions of different
  // reactors at one tag only by dependencies, and this reaction has no
  // effects, so another reactor of the process may still publish at the stop
  // tag after it. The one reconcile runs in ros2_lf_runtime_shutdown, after LF
  // returned and every reaction of the stop tag ran.
  auto & st = ros2_lf_binding::state();
  const auto stop = ros2_lf_binding::lf_stop_tag();
  std::lock_guard<std::mutex> lk(st.reg_mu);
  if (edge >= 0 && static_cast<size_t>(edge) < st.edges.size()) {
    st.edges[edge].closed = true;
  }
  if (stop.first >= 0) {
    st.close_stop_time = stop.first;
    st.close_stop_micro = stop.second;
  }
}

}  // extern "C"

// Read-only tag for a drop-injection shim preloaded into a benchmark run.
// Returns 1 with the elapsed tag of the dispatch publishing on this thread, else 0
// with the current elapsed tag, or -1 outside an LF program.
extern "C" int rcllf_publish_tag(int64_t * elapsed_ns, uint32_t * microstep)
{
  const auto now = ros2_lf_binding::lf_elapsed_tag();
  if (now.first < 0) {
    return -1;
  }
  if (ros2_lf_binding::t_dispatch_id == 0) {
    *elapsed_ns = now.first;
    *microstep = now.second;
    return 0;
  }
  const int64_t start = ros2_lf_binding::lf_current_tag().first - now.first;
  *elapsed_ns = ros2_lf_binding::t_dispatch_time - start;
  *microstep = ros2_lf_binding::t_dispatch_micro;
  return 1;
}

Ros2LfNodeConstructionScope::Ros2LfNodeConstructionScope(const std::string & binding_key)
: previous_(ros2_lf_binding::t_construct_key)
{
  ros2_lf_binding::t_construct_key = binding_key;
}

Ros2LfNodeConstructionScope::~Ros2LfNodeConstructionScope()
{
  // A TransformListener's group leaves its executor before any message
  // can reach the Buffer outside LF.
  ros2_lf_binding::detach_tf2_listener_groups(ros2_lf_binding::t_construct_key);
  ros2_lf_binding::t_construct_key = previous_;
}

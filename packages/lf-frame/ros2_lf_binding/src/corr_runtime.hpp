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

/// @file corr_runtime.hpp
/// @brief Correspondence runtime: gate state, origin context,
///        startup admission checks, ingress sample hook, epoch close.

#ifndef CORR_RUNTIME_HPP_
#define CORR_RUNTIME_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "correspondence.hpp"
#include "ros2_lf_binding/rcllf_trace_schema.h"
#include "ros2_lf_binding/rid_table_api.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{
namespace corr_rt
{

corr::Table & table();

/// This process is a federate.
/// Set by ros2_lf_runtime_peer_check, the federate-only entry point, which
/// runs after reactor-c parsed `-f`.
void set_federated(bool federated);
/// This process is a federate (set_federated). A legacy take with an unknown
/// write count waits for its tag deadline only here.
bool federated();
/// A federate in fast mode (`-f true`). Fast mode skips the wait for the
/// start time, so its logical time runs ahead of physical time.
bool fast_federate();
/// The clock the table judges cutoffs on, chosen at each call: the current
/// logical tag (clamped at 0) in a fast federate, lf_time_physical otherwise.
int64_t table_clock();

/// Stall watch: a dispatch at logical time `tag`; arms the stall watchdog
/// at the first one (corr_runtime.cpp, stall_watchdog).
void stall_progress(int64_t tag, uint32_t microstep);
/// Start the stall watch at the negotiated start
/// time `start_ns`, before any dispatch. A federate whose upstream stalls
/// before the start tag runs no reaction at all, so a watch armed by the
/// first dispatch alone never started.
void stall_watch_start(int64_t start_ns);

/// Thread-local publishing origin for the duration of a user callback.
class OriginScope
{
public:
  OriginScope(void * self, int rid);
  ~OriginScope();
  OriginScope(const OriginScope &) = delete;
  OriginScope & operator=(const OriginScope &) = delete;

private:
  void * prev_self_;
  int prev_rid_;
  bool prev_set_;
  int64_t prev_phys_;
};

/// True inside an OriginScope on this thread, that is, while an LF reaction
/// runs a user callback (the action gate refuses a send without).
bool origin_active();

/// Activation origin: S(a) of the activation on this thread, -1 outside a scope.
int64_t origin_phys();

/// Probe P17e: publishes this thread made inside an OriginScope since the last
/// reset, per resolved topic. Fed by the publication gate, read by
/// ros2_lf_topic_sent, reset at the entry of each timer and sub invoke.
std::map<std::string, int> & topic_sent_counts();
void topic_sent_reset();
void topic_sent_note(const std::string & resolved_topic);

using NodeResolver =
  std::function<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr(const std::string &)>;

/// Result of the D1 startup stage for one admitted edge.
struct StartupFailure
{
  int edge = -1;
  std::string check;
  std::string detail;
};

/// D1 startup stage: resolve both endpoints of every admitted edge, run the
/// checks, admit the subscriptions into ingress. Returns the first failure.
std::optional<StartupFailure> startup(const NodeResolver & resolve);
/// startup(), and on failure an ERROR naming edge and check, then _Exit(1).
void startup_or_exit(const NodeResolver & resolve);
/// RCLLF-H000 and RCLLF-H001 on the effective QoS of one bound
/// subscription, told apart by its RID, so twins on one topic and inputs no
/// edge reads are checked too. bind_subscription calls it. The reader of an
/// admitted edge is left to startup(), whose line names the edge. Warns,
/// never refuses.
void check_subscription_qos(const RidEntry & entry, const rclcpp::SubscriptionBase & sub);
/// RID `rid` was bound to `sub`. For each admitted `sub_rebind` edge
/// whose subscription RID it is, `sub` becomes the edge's next reader
/// generation when it provably is the edge's subscription (the analysis saw
/// one subscription of the node on the topic and the node holds exactly this
/// one) and passes the startup checks again (intra-process, VOLATILE, reader
/// depth, burst). The old reader is fenced off ingress first. Otherwise the
/// reader is refused with a typed reason, the edge stops and reconcile fails.
/// While the reader it replaces still lives, the check is deferred: a reader
/// made before the old one is released and a twin made next to it look the
/// same. retry_deferred_reader settles it at a later tag: admitted once the
/// old reader is gone, refused (rebind_identity) if it still lives.
/// bind_subscription calls it, under the late-binding lock.
void on_subscription_bound(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub);

/// Settle a reader on_subscription_bound deferred for RID `rid`.
/// lifecycle_admit calls it at each reaction of the RID, under the
/// late-binding lock. Cheap when nothing is deferred.
void retry_deferred_reader(int rid);
/// Whether any reader is deferred (no lock).
bool deferred_reader_pending();

/// One endpoint seen on a cross-federate edge's topic.
struct PeerEndpoint
{
  std::string node_fqn;
  std::string type;
  rmw_qos_profile_t qos;
};
/// Pure selection of the designated peer among `peers` (the other side's
/// endpoints on the topic): exactly one of node `peer_fqn`, same type as
/// `local`, compatible QoS. Returns "" or the problem.
std::string xfed_peer_problem(
  const std::vector<PeerEndpoint> & peers, const PeerEndpoint & local, const std::string & peer_fqn,
  bool local_is_publisher);
/// The D7 comparison for a lifecycle end made in a hook, run when
/// it is bound: "" when node `peer_fqn` has no endpoint on `topic` yet (a record
/// it misses ends missing) or the one it has matches the node's own end; else
/// the problem. `egress`: the node's end is the publisher.
std::string xfed_bind_peer_problem(
  const rcl_node_t * node, const std::string & topic, const std::string & peer_fqn, bool egress);
/// The kernel boot id, the one-clock check of a cross-federate edge.
inline constexpr const char * kBootIdPath = "/proc/sys/kernel/random/boot_id";
/// The 16 bytes of the boot id in the file at `path` (a UUID text line), or
/// nothing when the file cannot be read or holds anything but 16 bytes.
std::optional<std::array<uint8_t, 16>> read_boot_id(const std::string & path);
/// Startup check of a cross-federate edge: "" when the boot id at
/// `boot_id_path` reads as 16 bytes or a clock bound is declared (`synchronized`), else the
/// reason startup refuses the edge.
std::string xfed_clock_problem(const std::string & boot_id_path, bool synchronized);
/// The declared clock offset bound eps of cross-federate edges.
struct XfedClockBound
{
  bool declared = false;  ///< false: same host only (one boot id)
  int64_t eps_ns = 0;     ///< max |offset| of the publisher's clock from the receiver's
  std::string problem;    ///< non-empty: the declaration is malformed, startup refuses
};
/// From the texts of ROS2_LF_XFED_CLOCK_BOUND_NS (decimal ns) and
/// ROS2_LF_XFED_CLOCK_SYNCHRONIZED ("1" alone declares eps 0), either may be null.
XfedClockBound xfed_clock_bound_from(const char * bound, const char * synchronized);
/// The same, read from the environment.
XfedClockBound xfed_clock_bound();
/// "" when a message whose boot id is `same_boot` may be judged under `bound`,
/// else why the receiver refuses it.
std::string xfed_host_problem(bool same_boot, const XfedClockBound & bound);
/// A record stamped `phys` on the publisher's clock, received at `now`, breaks
/// the bound: it was committed after it arrived on every clock within `eps_ns`.
bool xfed_record_beyond_bound(int64_t phys, int64_t now, int64_t eps_ns);
/// A record's 16-byte `peer` boot id names this host's clock only when this
/// host's own boot id was read: two hosts without one would both send zeros.
bool xfed_same_boot(const uint8_t * peer, const std::optional<std::array<uint8_t, 16>> & host);
/// The bound a message of `edge` is judged under, 0 when its boot id is
/// `same_boot`, else the declared eps. Sets it as the edge's table bound
/// before its records are admitted, so a same-host edge never runs on eps.
int64_t xfed_judge_bound(
  corr::Table & t, uint32_t edge, bool same_boot, const XfedClockBound & bound);
/// "" when an ingress edge with timeout `timeout_ns` can run under `eps_ns`,
/// else why startup refuses it (no sample would be guaranteed on time).
std::string xfed_bound_problem(int64_t eps_ns, int64_t timeout_ns);
/// The release tag of an arrive at tag `now`, now + delay + extra with
/// `*wait` = delay + extra, the delay a drain is scheduled after. False, with
/// both outputs untouched, when a sum leaves int64_t: the arrive is refused.
bool release_tag(int64_t now, int64_t delay, int64_t extra, int64_t * wait, int64_t * release);
/// Logs that `edge`'s release tag past `now` overflows and is refused.
void release_overflow(int edge, int64_t now);
/// Writes of the legacy stub ports of one federate per logical time.
/// `after d` with d > 0 maps every microstep of a time to one tag. Stock
/// reactor-c moved a second message of a tag to its next microstep but merged
/// a third into that one (_lf_schedule_at_tag), losing a trigger. Patch 0002
/// (third_party/patches/reactor-c) now defers each colliding message past the
/// queued ones. The limit stays until delivery above two microsteps is tested.
class LegacyGuard
{
public:
  static constexpr uint32_t kMaxPerTime = 2;
  /// Decentralized, a second message of one tag is tardy at the receiver.
  static constexpr uint32_t kMaxPerTimeDecentralized = 1;
  /// The number of distinct microsteps of `time` at which `port` was written.
  uint32_t note(const std::string & port, int64_t time, uint32_t microstep);

private:
  struct Seen
  {
    int64_t time = 0;
    uint32_t last_micro = 0;
    uint32_t count = 0;
  };
  std::map<std::string, Seen> seen_;
};
/// The records of one cross-federate message, written into and read
/// back from the wire struct (at most RCLLF_XCORR_MAX, checked by the caller).
void pack_xcorr(const std::vector<corr::XRecord> & records, rcllf_xcorr_t * out);
std::vector<corr::XRecord> unpack_xcorr(const rcllf_xcorr_t & x);
/// Every egress and ingress endpoint finds its peer within
/// `timeout_ms`, else a FATAL naming the edge and _Exit(3). A cross-federate
/// edge without a readable boot id (xfed_clock_problem) is refused the same way.
void xfed_peer_check_or_exit(int timeout_ms);
/// The matched reader count of every live publisher on
/// `resolved_topic` that the publication gate saw created. Empty if none.
std::vector<size_t> publisher_match_counts(const std::string & resolved_topic);

size_t admitted_edge_count();

/// Ingress: one taken sample of edge `edge`. `reader_created`: the
/// creation tag of the `sub_rebind` reader that took it.
void on_sample(
  uint32_t edge, std::shared_ptr<void> message, const rclcpp::MessageInfo & info,
  std::optional<int64_t> reader_created = std::nullopt);

/// R13 warnings set by the gate, printed outside it.
void print_pending_warnings();

/// While one is alive on this thread, on_sample leaves the R13
/// warnings pending instead of printing them. A take holds the reader's take
/// mutex, which a publication holds under its slot lock, and printing takes
/// reg_mu, which is taken before a slot lock elsewhere: the caller prints
/// once the take mutex is released.
class WarningsDeferred
{
public:
  WarningsDeferred();
  ~WarningsDeferred();
  WarningsDeferred(const WarningsDeferred &) = delete;
  WarningsDeferred & operator=(const WarningsDeferred &) = delete;

private:
  bool prev_;
};

/// Test seam: false makes a publication take its local readers'
/// take mutexes only after rcl_publish returned (no delivery lock), so the
/// ingress thread can take the sample in between. Default true.
void set_local_delivery_lock_for_test(bool on);
/// Test seam: a sample the publish-time take finds on one of
/// `edges` is discarded unclassified and not planted, a real undelivered
/// sample. Empty (default) discards nothing.
void set_local_take_skip_for_test(std::set<uint32_t> edges);
/// Test seam: `hook` runs once, in the next admitted publication
/// with a local edge, after rcl_publish returned and before the publish-time
/// take. Another writer's publish from the hook shows whether the delivery
/// lock keeps the sample in the reader. Empty (default) runs nothing.
void set_local_after_publish_hook_for_test(std::function<void()> hook);

/// The LF stop was requested. Every join pending for a tag below the
/// stop tag ends within `bound_ms` (Table::note_stop_requested); logs the
/// joins pending now.
void note_stop_requested(int64_t bound_ms);
/// After LF returned, take the samples still in flight at the stop from
/// the admitted readers on the calling thread, up to `bound_ms`, and log what
/// is still owed. The ingress thread is stopped by then (the last
/// mailbox close at the stop tag, or the exit backstop before this atexit
/// handler), so the drain takes itself; Table::drain_in_flight only paces the
/// polls. Returns how many writers still owe a sample.
size_t drain_in_flight(int64_t bound_ms);
/// What the egress drain did: the publishers it waited on and those
/// still unacknowledged at the bound.
struct EgressDrain
{
  size_t waited = 0;
  size_t unacked = 0;
};
/// Before the process ends, wait up to `bound_ms` for the reliable
/// publishers of this program's tracked edges to have their last samples
/// acknowledged by their readers (an overloaded federate flushes after the
/// stop tag and would exit before delivery). Only a tracked slot's key is a
/// live rcl publisher: rcl registers its own rosout publisher from a stack
/// temporary and copies it away, so an untracked key may point anywhere.
EgressDrain drain_egress(int64_t bound_ms);
/// Epoch close + reconcile (D7). Idempotent. Returns reconcile().ok.
/// A federate first publishes its egress edges' end-of-run
/// COMPLETE counts and waits up to `end_wait_ms` for those of its ingress
/// edges (complete_end_channel_init); a count still missing fails the reconcile.
bool close_and_reconcile(int64_t end_wait_ms = 0);
/// Create the end-of-run COMPLETE channel of a federate with
/// cross-federate edges, before LF starts so discovery has the whole run.
void complete_end_channel_init();
/// After the reconcile, wait up to `bound_ms` for the readers of
/// this process's counts to acknowledge them, so the peer has them.
void complete_end_ack(int64_t bound_ms);
/// Release the channel's node while the context still lives.
void complete_end_release();
/// True after a reconcile that failed (Q3: the process exits nonzero).
bool reconcile_failed();

/// Start the rcllf trace of this epoch (run_begin, provider,
/// one backlog per tracked publisher). Idempotent. Call after startup().
void trace_begin();
/// lf_start with reactor-c's start tag, once per epoch, at the
/// first startup dispatch. No-op before trace_begin() or outside LF.
void trace_lf_start();
/// The one trace-close point. Snapshots every category, emits the
/// run_end group, later events take no seq. Idempotent.
void trace_close();
/// Drop the edge subscriptions. They keep their nodes, and with
/// them the DDS participant and its threads, alive into exit() otherwise.
/// Call after close_and_reconcile() and before the nodes are released.
void release_subscriptions();

/// dispatch_begin on construction, dispatch_end on destruction.
/// While alive, publications on this thread name its dispatch_id and tag.
class DispatchTrace
{
public:
  DispatchTrace(
    int rid, uint8_t entry_kind, uint32_t edge = 0xFFFFFFFFu, const corr::Gid * gid = nullptr,
    uint64_t ordinal = 0);
  ~DispatchTrace();
  DispatchTrace(const DispatchTrace &) = delete;
  DispatchTrace & operator=(const DispatchTrace &) = delete;
  void set_outcome(uint8_t outcome) {outcome_ = outcome;}
  void set_retained(uint64_t id) {retained_ = id;}
  void set_join_wait(int64_t ns) {join_wait_ns_ = ns;}
  void set_ran(bool ran) {ran_ = ran;}
  /// The cutoff tag time, -1 for none.
  void set_cutoff(int64_t cutoff) {cutoff_ = cutoff;}
  /// An rcllf_trace_missing_hook code.
  void set_missing_hook(uint8_t code) {missing_hook_ = code;}
  uint64_t id() const {return id_;}

private:
  uint64_t id_;
  uint64_t prev_id_;
  int64_t prev_time_;
  uint32_t prev_micro_;
  uint8_t outcome_;
  uint64_t retained_ = 0;
  int64_t join_wait_ns_ = 0;
  bool ran_ = false;
  int64_t cutoff_ = -1;
  uint8_t missing_hook_ = RCLLF_MISSING_HOOK_NA;
};

/// Tests: forget edges and gate slots, new epoch. Stops and resets ingress
/// first, so no ingress thread is inside the table it replaces.
void reset_for_test();
/// Tests: the KEEP_LAST depth the reader of an admitted edge gets at
/// least; 0 leaves the user's depth (the behaviour before the fix).
void set_reader_depth_for_test(size_t depth);
/// Tests: the sample of `ordinal` on (member) edge `edge` is lost in
/// transport, as RCLLF_TEST_CORR_DROP_SAMPLE makes it in a hooks build; 0
/// disables. Only test code can call it, no environment variable reaches it,
/// so it exists in every build. Call it after startup, before the publish.
void set_drop_sample_for_test(uint32_t edge, uint64_t ordinal);
/// Tests: the subscription startup bound admitted edge `id` to, or null.
const rclcpp::SubscriptionBase * edge_subscription_for_test(size_t id);
/// Test: runs in each subscription init after the real init succeeded
/// and before its creation rank is committed. Empty to clear.
void set_sub_commit_hook_for_test(std::function<void()> hook);
/// Test: runs in each gate on_init under `slot->m`, before the slot's
/// node name and durability are written. Empty to clear.
void set_pub_meta_hook_for_test(std::function<void()> hook);
/// Test: whether LF counts as running for a subscription created off an
/// LF worker; nullopt uses the runtime's own flag again.
void set_lf_running_for_test(std::optional<bool> running);
/// Test: subscriptions created so far for (binding key, resolved topic).
size_t sub_inits_for_test(const std::string & key, const std::string & resolved);
uint64_t after_close_publishes();
/// Binding key of the node under construction on this thread (a
/// Ros2LfNodeConstructionScope), empty otherwise.
const std::string & construct_key();
/// Fully qualified names of the live rcl nodes created inside the
/// construction scope of binding key `key`, sorted (clock gate
/// messages). Empty when none was created there.
std::vector<std::string> ros_node_names(const std::string & key);

}  // namespace corr_rt
}  // namespace ros2_lf_binding

#endif  // CORR_RUNTIME_HPP_

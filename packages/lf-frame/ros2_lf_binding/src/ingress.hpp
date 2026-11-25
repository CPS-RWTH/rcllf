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

/// @file ingress.hpp
/// @brief One-scheduler ingress role: one binding thread waits on
///        an rclcpp::ThreadSafeWaitSet holding ONLY allowlisted node
///        infrastructure services (parameters, lifecycle, type description),
///        takes requests, retains them in a per-node MAILBOX and wakes the
///        node's drain host once. Only LF workers run handle_request.
///
/// Record states, each transition under the mailbox lock, traced once:
///   RECEIVED -> REJECTED(overflow|closed)                   (never stored)
///   RECEIVED -> STORED -> CONSUMED | TERMINAL(shutdown|schedule_refused|
///                                             exception|not_ok|not_lf)

#ifndef INGRESS_HPP_
#define INGRESS_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{

/// Default per-node STORED capacity (R7). execution.yaml `ingress_capacity`
/// overrides it per node.
constexpr size_t kIngressCap = 8;

using IngressWakeFn = int (*)(void * action);

struct IngressRecord
{
  uint64_t id = 0;
  std::string service_suffix;
  rclcpp::ServiceBase::SharedPtr service;
  std::shared_ptr<rmw_request_id_t> header;
  std::shared_ptr<void> request;
  std::chrono::steady_clock::time_point t_rx;
  /// Set for a record the binding posts itself (a lifecycle transition made
  /// outside LF): the drain runs it instead of a service.
  std::function<void()> local;
};

/// Counters per node, for tests and the shutdown accounting (Output 7).
struct IngressCounts
{
  uint64_t stored = 0;
  uint64_t consumed = 0;
  uint64_t rejected_overflow = 0;
  uint64_t rejected_closed = 0;
  uint64_t terminal = 0;
  uint64_t wakes = 0;
  uint64_t stale_drains = 0;  ///< drain_one on a closed node or a missing id
};

class NodeMailbox
{
public:
  std::mutex mu;
  bool registered = false;
  bool closed = false;
  bool wake_pending = false;
  std::string close_reason;
  size_t capacity = kIngressCap;  ///< STORED limit
  IngressWakeFn wake = nullptr;
  void * action = nullptr;
  std::map<uint64_t, IngressRecord> stored;  ///< STORED, ordered by id (FIFO)
  std::deque<uint64_t> unmoved;              ///< STORED ids not yet in the carrier
  std::deque<uint64_t> carrier;              ///< reactor-owned id-only carrier
  IngressCounts counts;
};

/// Service-name suffixes admitted to the ingress wait set (the allowlist).
bool ingress_suffix_allowed(const std::string & suffix);
/// True when `service` has the concrete rclcpp::Service<T> type the suffix implies.
bool ingress_type_matches(const std::string & suffix, const rclcpp::ServiceBase * service);
/// Mutating services get a binding-built failure reply when rejected.
bool ingress_suffix_mutating(const std::string & suffix);

class IngressRole
{
public:
  static IngressRole & instance();

  /// The only way into the wait set. Returns false (and logs) for anything
  /// that is not an allowlisted infrastructure service of node `node_fqn`.
  bool admit(
    const std::string & key, const std::string & node_fqn,
    const rclcpp::ServiceBase::SharedPtr & service);
  bool admit(const std::string & key, const rclcpp::TimerBase::SharedPtr & timer);
  bool admit(const std::string & key, const rclcpp::SubscriptionBase::SharedPtr & sub);
  bool admit(const std::string & key, const rclcpp::ClientBase::SharedPtr & client);
  bool admit(const std::string & key, const rclcpp::Waitable::SharedPtr & waitable);
  /// The subscription of an admitted correspondence edge. Ingress
  /// takes, classifies and retains its samples; it never runs its callback.
  /// A fan-in group's one reader is admitted on its lead `edge`, and a
  /// take for any edge of `group` (its members) uses it.
  bool admit_corr_subscription(
    uint32_t edge, const rclcpp::SubscriptionBase::SharedPtr & sub,
    std::vector<uint32_t> group = {});
  /// `sub` becomes the reader of `edge`, a lifecycle subscription
  /// made or reset in a hook. Ingress holds only its rcl handle (never the
  /// SubscriptionBase, so the node's drop stays the drop the lifecycle gate
  /// sees) and stops taking once the node dropped it. The edge's previous
  /// reader is retired first, and no take of it is in flight on return.
  /// `created` is the reader's creation tag; its samples carry it, so the
  /// table can tell a sample of a reader a later one replaced (STALE_READER).
  bool replace_corr_reader(
    uint32_t edge, const rclcpp::SubscriptionBase::SharedPtr & sub, int64_t created);
  /// Retire `edge`'s reader (a refused one), fenced as above.
  void retire_corr_reader(uint32_t edge);
  /// Tests: whether `edge` has a reader admitted through replace_corr_reader.
  bool has_corr_reader_for_test(uint32_t edge);
  /// Tests: replace_corr_reader fails as without introspection.
  void refuse_corr_admit_for_test(bool refuse) {refuse_corr_admit_.store(refuse);}
  /// A worker draining the local self edge `edge`: take what its
  /// reader holds now, as this thread would. The publish wrote the sample
  /// before the arrive, so the join finds it although this thread has not run.
  /// Serialized with the thread's own take of correspondence samples. Takes
  /// batch after batch, releasing the take lock between batches, until `done`
  /// holds or the reader is empty. Bounded by what the reader can hold at entry
  /// (its KEEP_LAST depth, kCorrDrainTakeMax for KEEP_ALL), and never more,
  /// so a sustained publisher on the same subscription cannot hold the worker
  /// forever. Returns the number of samples taken.
  size_t take_corr_now(uint32_t edge, const std::function<bool()> & done = {});
  /// The delivery lock of one publication's local edges, the take
  /// mutex of each reader that serves one of them (a fan-in group's reader
  /// once), held from before rcl_publish until its samples are classified.
  /// Movable, released on destruction.
  class LocalTake;
  /// Lock the readers serving `edges` in ascending order of the
  /// edge each reader is admitted on (its lead), the one order every
  /// publisher uses, so two publications never wait on each other in a cycle.
  /// An edge no admitted node subscription serves (a rebind reader or
  /// none) is skipped.
  LocalTake lock_local_readers(const std::vector<uint32_t> & edges);
  /// With `held` locked, take every sample its readers hold now
  /// and hand each to corr_rt::on_sample, as the ingress thread would. A
  /// KEEP_LAST reader holds at most its depth. A KEEP_ALL reader may hold
  /// more than one batch of older samples ahead of this publication's, so it
  /// is taken batch after batch until a batch comes
  /// back short (the reader is empty) or `done(edge)` holds for the edge the
  /// reader is admitted on (that reader's samples of the publication are in).
  /// The publication's sample is in the reader at entry, so it comes before
  /// anything written later, and the loop ends once it is taken however fast
  /// other writers fill the reader. Returns the number of samples taken.
  size_t take_local_held(
    LocalTake & held, const std::function<bool(uint32_t)> & done = {});
  /// The subscription of RID `rid` on a topic declared in
  /// `external_topics`. Ingress takes each sample and hands it to
  /// external_input::on_sample, which schedules the reaction's physical action.
  bool admit_external_subscription(int rid, const rclcpp::SubscriptionBase::SharedPtr & sub);

  /// Ingress thread (and tests): take at most `max_batch` messages from
  /// the external subscription `sub` of `rid` and hand each to
  /// external_input::on_sample. Takes nothing once the caller holds the only
  /// reference, that is once the node dropped the subscription. Returns the
  /// number taken.
  static size_t take_external(
    int rid, const rclcpp::SubscriptionBase::SharedPtr & sub, size_t max_batch);
  /// One batch: the subscription's history depth (at least 1).
  static size_t external_batch(const rclcpp::SubscriptionBase & sub);

  /// Drain-host startup: create/arm the mailbox, admit the node's
  /// infrastructure services (when `node_base` is non-null), start the thread.
  void register_node(
    const std::string & key, IngressWakeFn wake, void * action,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base);

  /// Set the node's STORED capacity (creates the mailbox). Called
  /// before register_node, so no request is stored against the default.
  void set_capacity(const std::string & key, size_t capacity);
  size_t capacity(const std::string & key);

  /// Ingress side of one taken request (the thread loop, and tests), or of a
  /// record the binding posts itself. True: STORED, false: REJECTED.
  bool on_request(const std::string & key, IngressRecord record);

  /// ingress_wake reaction: STORED ids -> carrier. True: schedule a drain.
  bool move(const std::string & key);
  /// ingress_drain reaction: handle one record. True: schedule another drain.
  bool drain_one(const std::string & key);
  /// A drain-side lf_schedule returned <= 0.
  void schedule_refused(const std::string & key);
  /// Shutdown reaction of the drain host; the last close stops the thread.
  void close(const std::string & key, const char * reason = "shutdown");

  /// atexit backstop (R16): no mailbox lock, 500 ms deadline, then _Exit.
  void stop_backstop();
  /// Stop and join the thread (idempotent). `log`: false on the exit path.
  void stop_and_join(bool log = true);

  IngressCounts counts(const std::string & key);
  bool is_closed(const std::string & key);
  bool wake_pending(const std::string & key);
  size_t carrier_size(const std::string & key);
  bool thread_running() const {return thread_running_.load();}
  /// Tests: wait-set wakeups that reported Ready since the last reset.
  /// Idle, this stays at zero (the 100 ms wait times out); a spinning thread
  /// counts thousands per second.
  uint64_t ready_wakeups_for_test() const {return ready_wakeups_.load();}
  /// Tests: physical QoS events taken (and dropped) by the thread.
  uint64_t qos_events_taken_for_test() const {return qos_events_taken_.load();}
  size_t admitted_count();
  uint64_t next_record_id() {return next_id_.fetch_add(1) + 1;}

  /// Tests: forget every mailbox and admitted service (thread must be stopped).
  void reset_for_test();
  /// Tests: the next `n` waits of the ingress thread throw, as a failing
  /// rcl_wait would.
  void fail_waits_for_test(int n) {forced_wait_failures_.store(n);}
  /// Tests: the backstop deadline (default 500 ms).
  void set_backstop_deadline_for_test(std::chrono::milliseconds d) {backstop_deadline_ = d;}
  /// Tests: while paused the thread takes no correspondence sample, so a
  /// writer can publish several samples before the next take. Pausing returns
  /// once no take loop is in flight.
  void pause_corr_takes_for_test(bool paused)
  {
    corr_paused_.store(paused);
    while (paused && corr_takes_in_flight_.load() > 0) {
      std::this_thread::yield();
    }
  }
  /// Tests: whether pause_corr_takes_for_test(true) is in effect.
  bool corr_takes_paused_for_test() const {return corr_paused_.load();}
  /// Tests: `hook(n)` runs in the ingress thread after each
  /// take loop on a correspondence reader, with the loop's sample count, after
  /// the samples are counted and before the loop's call is. A hook that blocks
  /// holds that interleaving open. nullptr removes it; both wait out a loop in
  /// flight, so a loop never sees a hook being replaced.
  void set_corr_loop_end_hook_for_test(std::function<void(size_t)> hook)
  {
    corr_loop_end_hooked_.store(false);
    while (corr_takes_in_flight_.load() > 0) {
      std::this_thread::yield();
    }
    corr_loop_end_hook_ = std::move(hook);
    corr_loop_end_hooked_.store(static_cast<bool>(corr_loop_end_hook_));
  }
  /// Tests: hold the lock a take of correspondence samples on
  /// `edge` holds, as a take whose reader refills forever would. Not owning
  /// when no subscription is admitted for `edge`.
  std::unique_lock<std::mutex> hold_corr_take_for_test(uint32_t edge);
  /// How many correspondence samples one take loop takes from a
  /// reader, the ingress thread's loop and each lock-held batch of a worker's
  /// take_corr_now alike. A KEEP_LAST reader holds at most its depth, so a
  /// batch of the depth (at least 1) takes every sample present when the loop
  /// starts: no sample waits for a second wait-set round trip because of the
  /// batch, and one loop holds the reader's take lock for at most depth takes,
  /// which is all a worker's take_corr_now waits for. It is the rule the
  /// external-input path already uses (external_batch). A KEEP_ALL
  /// reader has no depth that bounds its backlog, so its batch is the named
  /// bound kCorrKeepAllTakeBatch.
  static size_t corr_batch(bool keep_last, size_t depth);
  /// The batch of a KEEP_ALL correspondence reader. Not derived
  /// from the system, an explicit bound: one loop holds the reader's take lock
  /// for at most this many takes, and a backlog of k samples needs
  /// ceil(k / 16) loops, so a sample at backlog position k waits at most
  /// ceil(k / 16) - 1 extra wait-set round trips. Each such loop counts as
  /// `full` in the ros2_lf_ingress_batch line, so its cost is visible in every
  /// run. A smaller value adds round trips for any backlog, a larger one lets a
  /// reader that a writer refills hold the lock longer while a worker's drain
  /// waits on it. No admitted reader of the shipped examples is KEEP_ALL.
  static constexpr size_t kCorrKeepAllTakeBatch = 16;
  /// Occupancy and intake delay of one take path, the ingress
  /// thread's loops (`thread`) or the workers' take_corr_now calls (`worker`).
  struct CorrTakeStats
  {
    uint64_t calls = 0;    ///< take loops (thread) or take_corr_now calls (worker)
    uint64_t full = 0;     ///< calls that took their whole batch (thread) or limit (worker)
    uint64_t max = 0;      ///< most samples one call took
    uint64_t samples = 0;  ///< samples taken
    uint64_t timed = 0;    ///< samples with a DDS reception time
    uint64_t delay_sum_ns = 0;  ///< sum of reception-to-take delays of `timed`
    uint64_t delay_max_ns = 0;  ///< largest reception-to-take delay
  };
  CorrTakeStats corr_take_stats(bool worker) const;
  /// The publish-time takes of local edges (take_local_held).
  CorrTakeStats corr_publish_take_stats() const;
  /// One `ros2_lf_ingress_batch:` line per take path (thread,
  /// worker) with its stats.
  void log_corr_batch_summary() const;
  /// Most samples one take_corr_now takes from a KEEP_ALL reader.
  static constexpr size_t kCorrDrainTakeMax = 1024;
  /// Tests: one take loop on `edge`, as the ingress thread runs.
  size_t take_corr_batch_for_test(uint32_t edge);

private:
  IngressRole();
  std::shared_ptr<NodeMailbox> mailbox(const std::string & key, bool create);
  void thread_main();
  void ensure_thread();
  void close_locked(
    NodeMailbox & mb, const std::string & key, const std::string & reason,
    bool clear_carrier);
  void reject(const IngressRecord & record, const std::string & key, const char * why);
  void send_failure_reply(const IngressRecord & record, const char * why);
  void trace(
    const std::string & key, const IngressRecord & r, const char * state,
    const std::string & detail = "");

  std::mutex boxes_mu_;
  std::map<std::string, std::shared_ptr<NodeMailbox>> boxes_;
  struct Admitted
  {
    std::string key;
    std::string suffix;
    rclcpp::ServiceBase::SharedPtr service;
    rclcpp::SubscriptionBase::SharedPtr sub;  ///< correspondence edge (service is null)
    uint32_t edge = 0;
    int ext_rid = -1;  ///< external subscription RID (`ext_sub` set, edge unused)
    /// Held weakly, so a lifecycle node that drops its subscription
    /// destroys it and the RID binds late to the one it creates next.
    std::weak_ptr<rclcpp::SubscriptionBase> ext_sub;
    /// One take of this subscription's correspondence samples at
    /// a time. Per subscription, so a long take on one edge never blocks
    /// another edge's drain. Copies of the entry share it.
    std::shared_ptr<std::mutex> take_mu = std::make_shared<std::mutex>();
    /// A reader of replace_corr_reader. Its rcl handle, the node's
    /// subscription held weakly, the wait-set waitable on the handle, the
    /// message's introspection members, and the reader's history.
    std::shared_ptr<rcl_subscription_t> rcl_sub{};
    std::weak_ptr<rclcpp::SubscriptionBase> reader{};
    rclcpp::Waitable::SharedPtr waitable{};
    const void * members = nullptr;
    size_t depth = 0;
    bool keep_last = true;
    int64_t created = std::numeric_limits<int64_t>::min();  ///< the reader's creation tag
    std::vector<uint32_t> group{};  ///< a fan-in group's member edges, `edge` its lead
    bool corr() const {return ext_rid < 0 && (sub || rcl_sub);}
    /// The reader of correspondence edge `e`: its own, or its fan-in group's (R6).
    bool serves(uint32_t e) const
    {
      return corr() && (edge == e || std::find(group.begin(), group.end(), e) != group.end());
    }
  };
  struct PendingAdmit
  {
    Admitted entry;
    std::promise<bool> done;
    bool add = true;      ///< false retires only
    int64_t retire = -1;  ///< edge whose rebind reader goes first, -1 none
  };
  bool add_to_wait_set(const Admitted & a);
  /// Take (and drop) every admitted subscription's QoS event that the
  /// wait reported ready, so the reader's status condition clears.
  void take_qos_events(const rcl_wait_set_t & ws);
  bool submit_admit(
    const Admitted & a, const std::string & name, int64_t retire = -1, bool add = true);
  /// Caller holds admitted_mu_ on the ingress thread (or none runs).
  void retire_corr_reader_locked(uint32_t edge);
  struct AtomicTakeStats
  {
    std::atomic<uint64_t> calls{0};
    std::atomic<uint64_t> full{0};
    std::atomic<uint64_t> max{0};
    std::atomic<uint64_t> samples{0};
    std::atomic<uint64_t> timed{0};
    std::atomic<uint64_t> delay_sum_ns{0};
    std::atomic<uint64_t> delay_max_ns{0};
    void record_call(uint64_t n, bool full_call);
    void record_sample(const rmw_message_info_t & info);
  };
  size_t take_rcl_loop_held(const Admitted & a, size_t max_count, AtomicTakeStats & stats);
  void take_corr_samples(const Admitted & a);
  /// The reader's history and its corr_batch.
  static size_t corr_batch_of(const Admitted & a, bool * keep_last = nullptr);
  /// Take until empty or `max_count` samples, no pause check. Each
  /// sample's intake delay goes to `stats`.
  size_t take_corr_loop(const Admitted & a, size_t max_count, AtomicTakeStats & stats);
  /// take_corr_loop with the reader's take_mu already held by the caller.
  /// `publish`: the publish-time take, which empties the reader even
  /// once the ingress thread stops, since every expectation it settles needs
  /// its sample classified first.
  size_t take_corr_loop_held(
    const Admitted & a, size_t max_count, AtomicTakeStats & stats, bool publish = false);
  /// Drop expired external admissions, then poke the lifecycle RIDs.
  void retire_expired_external();
  void apply_pending_admits();
  std::mutex admitted_mu_;
  std::vector<Admitted> admitted_;
  std::vector<std::unique_ptr<PendingAdmit>> pending_;
  // Owned by the ingress thread while it runs: admissions made then are
  // handed over through pending_ (rclcpp::WaitSet is not thread-safe, and
  // ThreadSafeWaitSet failed to resize: "services set is full", measured).
  std::unique_ptr<rclcpp::WaitSet> wait_set_;
  rclcpp::GuardCondition::SharedPtr stop_guard_;
  std::thread thread_;
  std::mutex thread_mu_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> corr_paused_{false};  ///< test pause
  std::atomic<bool> refuse_corr_admit_{false};  ///< test seam
  std::atomic<int> corr_takes_in_flight_{0};  ///< take loops past the pause check
  std::function<void(size_t)> corr_loop_end_hook_;  ///< r9 test hook
  std::atomic<bool> corr_loop_end_hooked_{false};  ///< the hook is set (no lock in production)
  std::atomic<bool> thread_running_{false};
  std::atomic<bool> thread_exited_{false};
  std::atomic<uint64_t> next_id_{0};
  std::atomic<uint64_t> ready_wakeups_{0};  ///< wait returns with Ready
  std::atomic<int> forced_wait_failures_{0};  ///< test hook
  AtomicTakeStats thread_stats_;  ///< the ingress thread's corr take loops
  AtomicTakeStats worker_stats_;  ///< the workers' take_corr_now calls
  AtomicTakeStats publish_stats_;  ///< the publish-time takes of local edges
  std::atomic<uint64_t> qos_events_taken_{0};  ///< events taken and dropped
  std::chrono::milliseconds backstop_deadline_{500};

public:
  class LocalTake
  {
public:
    LocalTake() = default;
    LocalTake(LocalTake &&) = default;
    LocalTake & operator=(LocalTake &&) = default;
    LocalTake(const LocalTake &) = delete;
    LocalTake & operator=(const LocalTake &) = delete;
    /// The number of readers held.
    size_t readers() const {return readers_.size();}
    /// Release every take mutex (also done on destruction), in reverse order.
    void release()
    {
      while (!locks_.empty()) {
        locks_.pop_back();
      }
      readers_.clear();
    }
    ~LocalTake() {release();}

private:
    friend class IngressRole;
    std::vector<Admitted> readers_;  ///< copies: they keep the take mutex alive
    std::vector<std::unique_lock<std::mutex>> locks_;
  };
};

}  // namespace ros2_lf_binding

#endif  // INGRESS_HPP_

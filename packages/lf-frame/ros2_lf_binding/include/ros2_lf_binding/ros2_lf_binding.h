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

#ifndef ROS2_LF_BINDING__ROS2_LF_BINDING_H_
#define ROS2_LF_BINDING__ROS2_LF_BINDING_H_

#ifndef __cplusplus
#include <stdbool.h>
#include <stdint.h>
#else
#include <cstdint>
#endif

#include "ros2_lf_binding/action_gate_api.h"  // RCLLF_ACTION_* channels
// The BNA synchronizer calls of generated reactions.
#include "ros2_lf_binding/bna_runtime_api.h"
#include "ros2_lf_binding/tf2_runtime_api.h"  // rcllf_tf2 adapter hooks

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// Initialize rclcpp, load component nodes from SYSTEM_JSON env var, and
/// attach the LF time source. Must be called before lf_reactor_c_main().
void ros2_lf_runtime_init(int argc, char ** argv);

/// Load component and non-component nodes from the system.json parsed during
/// ros2_lf_runtime_init().  Call this AFTER ros2_lf_runtime_init() AND after
/// all non-component nodes have been registered via ros2_lf_register_node().
/// For non-component (lfc) builds this is called by the generated
/// ros2_lf_startup.cpp constructor after node_factories.cpp has registered nodes.
void ros2_lf_runtime_load_nodes(void);

/// Block until DDS discovery has settled: every subscription has at least one
/// matched publisher and every publisher has at least one matched subscription.
/// Call AFTER ros2_lf_runtime_load_nodes() and BEFORE lf_reactor_c_main() sets
/// the LF start time.  This anchors logical time zero to the instant the system
/// is ready to communicate, eliminating the startup-transient latency spike
/// caused by the first logical tags being released in a catch-up burst once
/// discovery completes.  Honors the ROS2_LF_DISCOVERY_TIMEOUT_MS env var
/// (default 5000); returns after the timeout even if not all endpoints matched.
void ros2_lf_runtime_wait_ready(void);

/// Leave the cross-federate peer check out of
/// ros2_lf_runtime_wait_ready(). Called by a generated federate startup, which
/// then calls ros2_lf_runtime_peer_check() once the RTI start time is known.
void ros2_lf_runtime_defer_peer_check(void);

/// Run the cross-federate peer check with the
/// ROS2_LF_DISCOVERY_TIMEOUT_MS budget (default 5000), or exit 3. Called after
/// the RTI sent the start time and before tag 0, when every federate is past
/// its own node loading.
void ros2_lf_runtime_peer_check(void);

/// Declare `topic` (resolved name, leading slash optional) external:
/// it has no publisher in the generated system, so
/// ros2_lf_runtime_wait_ready() does not wait for its subscriptions to match
/// one, and the ROS2_LF_TAKE_MODE override leaves them out. Called by the
/// generated ros2_lf_startup.cpp before ros2_lf_runtime_init(), once per
/// execution.yaml `external_topics` entry.
void ros2_lf_runtime_declare_external_topic(const char * topic);

/// `topic`, declared external, anchors
/// its messages at `header.stamp` instead of at arrival. A message's tag is
/// start + ceil(header.stamp + delay_ns + ingress_reserve_ns - start, grid_ns)
/// at microstep 0; one arriving after header.stamp + delay_ns is late. Called
/// by the generated ros2_lf_startup.cpp right after the topic's declaration,
/// from execution.yaml's `anchor: stamp` entry. The topic's message type must
/// carry a std_msgs/Header named `header`, else startup refuses.
void ros2_lf_runtime_declare_external_stamp_anchor(
  const char * topic, int64_t delay_ns, int64_t ingress_reserve_ns, int64_t grid_ns);

/// Spin down rclcpp after lf_reactor_c_main() returns. Runs once; a second
/// call only ends the process with the exit code the first call deferred.
void ros2_lf_runtime_shutdown(void);

/// The federated startup wraps reactor-c's
/// lf_terminate_execution() and calls this before the resign to the RTI, so
/// the end-of-run counts go out while the RTI can still block this federate's
/// resign behind a peer's. Runs the shutdown; a nonzero exit it decides is
/// deferred to the later atexit call, after reactor-c's termination().
void ros2_lf_runtime_shutdown_before_resign(void);

/// Test only: ROS2_LF_TEST_RESIGN_DELAY="<name>:<ms>" delays the RTI resign of
/// the named federate (the lf_terminate_execution() wrapper calls it).
void ros2_lf_runtime_test_resign_delay(void);

/// Current LF logical time in nanoseconds (the time of the first
/// environment's current tag), or -1 outside a running LF program.
/// Diagnostic helper: inside a reaction, every bound node's `now()` returns
/// exactly this value.
int64_t ros2_lf_logical_time_ns(void);

// ---------------------------------------------------------------------------
// Startup binding (called from lfc-generated reaction(startup) bodies)
// ---------------------------------------------------------------------------

/// Register all ROS2 handles (timers, subscriptions) of the component
/// identified by node_class into the global handle table, indexed by
/// reaction IDs from ros2_lf_reaction_ids.h.  Cancels all matched timers
/// so LF becomes the sole scheduler.
///
/// @param self   Pointer to the lfc-generated reactor self struct (unused;
///               reserved for future per-reactor state).
/// @param node_class  C++ class name, e.g. "PublisherNode".
void ros2_lf_bind_node(void * self, const char * node_class);

// ---------------------------------------------------------------------------
// Reaction dispatch (called from lfc-generated reaction bodies)
// ---------------------------------------------------------------------------

/// Invoke the ROS2 timer callback registered under reaction_id.
/// No-op if the handle was not bound.
void ros2_lf_invoke_timer(void * self, int reaction_id);

/// Take one message from the DDS queue for the subscription registered under
/// reaction_id and topic, then dispatch the ROS2 callback.
/// No-op if no message is available or the handle was not bound.
/// Returns 1 if the callback ran and 0 otherwise, so the reaction counts a
/// write on its stub outputs only for a callback that ran.
int ros2_lf_invoke_sub(void * self, int reaction_id, const char * topic);

/// Probe P17e: ros2_lf_invoke_sub for a trigger that still owes `remaining`
/// writes (this one included). A skip-mode take that missed on an earlier
/// trigger owes a sample (debt), and takes up to remaining + debt samples
/// here, dropping the oldest extras as late (reported, at most debt) so the
/// edge returns to its tag. ros2_lf_invoke_sub is remaining = 0: the write
/// count is unknown (stub port counts callback runs), so a miss owes nothing (P17f).
int ros2_lf_invoke_sub_n(void * self, int reaction_id, const char * topic, int remaining);

/// Probe P17e (fix B): the number of publishes on `topic` (as named in the
/// node's source, resolved against the node of `reaction_id`) this thread made
/// inside the current invoke, and resets it. The generated reaction raises the
/// publisher's port with this count instead of the callback's run count.
int ros2_lf_topic_sent(int reaction_id, const char * topic);

/// Requests sent on this thread on `service` (as the node's source names it)
/// since the last call, consumed. The generated sender reaction writes its
/// request port this many times (one write per request really sent).
int ros2_lf_service_sent(int reaction_id, const char * service);

/// Arms the output deadline of `topic` (as the node's source names
/// it, resolved against the node of `reaction_id`) for the reaction's next
/// publishes. The generated reaction calls it right before its invoke when the
/// execution config sets `publishers.<node>.<topic>.output_deadline_ns`. A
/// publish later than `bound_ns` after the reaction's tag is traced and logged
/// as actuation_late and published anyway.
void ros2_lf_output_deadline_arm(int reaction_id, const char * topic, int64_t bound_ns);

/// Probe P17e: count one publish on `resolved_topic` for the running reaction,
/// for a sample the callback defers to another thread (fault injection only).
void ros2_lf_claim_publish(const char * resolved_topic);

/// Register the physical action of the subscription reaction
/// `reaction_id`, whose topic is declared in execution.yaml `external_topics`.
/// Called from a generated startup reaction. The ingress thread takes each
/// message of that subscription and calls wake(action, record_id), which
/// schedules the action (payload type int64_t) at arrival with the record id.
void ros2_lf_external_register(
  void * self, int reaction_id, int (*wake)(void *, int64_t), void * action);

/// Run the subscription callback of `reaction_id` once for every
/// taken message with a record id up to `record` (the action payload), in
/// arrival order. Returns how many callbacks ran, so the reaction can count
/// that many writes on its stub outputs.
int ros2_lf_invoke_external_sub(void * self, int reaction_id, int64_t record);

// ---------------------------------------------------------------------------
// Service dispatch
// ---------------------------------------------------------------------------

/// Re-invoke the timer/sub-driven user callback that calls async_send_request.
/// The callback is already bound as a TIMER or SUBSCRIPTION handle; this
/// wrapper just uses the same timer-dispatch path.
void ros2_lf_invoke_srv_send(void * self, int reaction_id, const char * service_name);

/// Take one request from the service's DDS queue and dispatch the user's
/// service callback.  Stores the rmw_request_id so the callback can send
/// the response.  No-op if no request is available or handle not bound.
/// Returns 1 if the callback ran and 0 otherwise.
int ros2_lf_invoke_srv_server(void * self, int reaction_id, const char * service_name);

/// Take one response from the client's DDS queue and invoke the registered
/// response callback (lambda captured in async_send_request).
/// No-op if no response is available or handle not bound.
/// Returns 1 if the callback ran and 0 otherwise.
int ros2_lf_invoke_srv_client_cb(void * self, int reaction_id, const char * service_name);

// ---------------------------------------------------------------------------
// Action dispatch
// ---------------------------------------------------------------------------

/// Re-invoke the timer-driven user callback that calls async_send_goal.
/// The callback is bound as a TIMER handle; this wrapper uses the same
/// timer-dispatch path.
void ros2_lf_invoke_action_goal_send(void * self, int reaction_id, const char * action_name);

/// Every action invoke below takes `count` samples, the value of
/// its counted trigger port, each with a bounded wait (the take timeout), and
/// traces LOST for one that does not arrive.

/// The number of `channel` (rcllf_action_channel) samples of
/// `action` this thread sent since the last call for that pair, and resets it.
/// Fed by the executable-side action gate. Called by the generated reaction
/// after its invoke to raise the channel's counted port.
int ros2_lf_action_sent(const char * action, int channel);

/// Execute the action server's waitable entity to dispatch handle_goal +
/// handle_accepted.
/// No-op if no goal request is available or handle not bound.
void ros2_lf_invoke_action_goal_server(
  void * self, int reaction_id, const char * action_name, int count);

/// Execute the action client's waitable entity to dispatch whichever
/// callback is ready (goal_response / feedback / result / cancel response).
/// No-op if no data is available or handle not bound.
void ros2_lf_invoke_action_client_cb(
  void * self, int reaction_id, const char * action_name, int count);

/// Execute the action server's waitable entity to dispatch a pending cancel
/// request (handle_cancel, and — if accepted — the goal_handle transitions
/// the executing goal to canceling).
/// No-op if no cancel request is available or handle not bound.
void ros2_lf_invoke_action_cancel_server(
  void * self, int reaction_id, const char * action_name, int count);

/// Execute the action server's waitable entity to dispatch a pending
/// GetResult service request (ResultService entity). The client's
/// async_send_goal(result_callback=...) automatically sends this request once
/// the goal is accepted; the server only replies once this runs — if the
/// goal has already succeeded/aborted/canceled the reply goes out
/// immediately, otherwise the request is queued until it does.
/// No-op if no GetResult request is available or handle not bound.
void ros2_lf_invoke_action_result_server(
  void * self, int reaction_id, const char * action_name, int count);

/// An action server whose action has no client inside the
/// LF system (a stock client such as `ros2 action send_goal`). Called from a
/// generated startup reaction with the goal server RID. The server's on-ready
/// callback counts each arriving goal, result and cancel request and calls
/// wake(action, 0), which schedules the physical action at arrival.
/// Its tags are physical, so the repeatable-tag claim does not
/// cover this traffic.
void ros2_lf_action_external_register(
  void * self, int goal_rid, int (*wake)(void *, int64_t), void * action);

/// The requests of service `entity` (0 goal, 1 result,
/// 2 cancel) counted since the last call for this goal server RID, and resets
/// the count. The generated reaction passes it as the invoke's count.
int ros2_lf_action_external_pending(void * self, int goal_rid, int entity);

/// Called from the generated reaction(shutdown) of a node
/// that serves an action, with the RID of its first goal server. Runs once,
/// with that RID as the LF origin, each pre-shutdown callback the node's code
/// registered on the default context while it was constructed (the binding
/// kept them from the context), so a goal they abort sends its result in a
/// reaction.
void ros2_lf_action_server_shutdown(void * self, int goal_rid);

/* --- One-scheduler ingress -------------------------------------
 * Generated into each node's drain host (default-group reactor, else the
 * outer node reactor, R11). `wake` is a generated thunk that calls
 * lf_schedule(action, 0) on the physical action `ingress_wake` and returns
 * its handle; a handle <= 0 is a refusal and closes the node's mailbox.
 */

/** The node's ingress STORED capacity (execution.yaml `ingress_capacity`).
 *  Emitted before ros2_lf_ingress_register only when it differs from the default 8.
 *  A capacity < 1 is ignored with a warning. */
void ros2_lf_ingress_set_capacity(const char * key, int capacity);

/** Startup reaction of the drain host: admit the node's infrastructure services. */
void ros2_lf_ingress_register(void * self, const char * key, int (*wake)(void *), void * action);

/** Reaction to ingress_wake: move STORED ids to the carrier. True: schedule ingress_drain. */
bool ros2_lf_ingress_move(const char * key);

/** Reaction to ingress_drain: handle one request on this LF worker. True: schedule again. */
bool ros2_lf_ingress_drain_one(void * self, const char * key);

/** A drain-side lf_schedule(ingress_drain) returned <= 0: close the node's mailbox. */
void ros2_lf_ingress_schedule_refused(const char * key);

/** Shutdown reaction of the drain host. The last close stops and joins ingress. */
void ros2_lf_ingress_close(const char * key);

/** atexit backstop, registered after reactor-c's termination handler (R8, R16). */
void ros2_lf_ingress_stop_atexit(void);

/* --- Publication correspondence --------------------------------
 * Admitted topic edges carry an rcllf_corr_t over a zero-delay LF connection
 * from the publishing reaction to the subscriber, which moves the records into
 * a subscriber-side carrier released at t+d (d = the edge's after-delay) and
 * dispatches every record due at a tag at that tag, each joined with the exact DDS sample
 * the publish produced.
 */

/** LF port type of an admitted edge: `count` records published at this tag. */
typedef struct rcllf_corr_t
{
  uint32_t edge;
  uint32_t count;
} rcllf_corr_t;

/** Publishing reaction, after the invoke: records of `edge` not yet arrived (0: no lf_set).
 *  Cumulative over the tag, so when several reactions of one reactor publish on
 *  `edge` the last lf_set (the value LF keeps) carries every record. */
uint32_t ros2_lf_corr_emit(void * self, int edge);

/** Arrive reaction: move `count` records into the carrier. Returns the wake delay (ns) or -1. */
int64_t ros2_lf_corr_arrive(void * self, int edge, uint32_t count);

/** corr_drain reaction: dispatch every record released at the current tag, one callback
 *  invocation each, in rank then ordinal order. Returns the next wake delay (ns) or -1. */
int64_t ros2_lf_corr_drain(void * self, int edge, int reaction_id, const char * topic);

/** Before every lf_schedule(corr_drain_E, w): -1 when a drain event of edge E is
 *  already queued at the wake's tag (that drain computes every later wake), else `w`,
 *  noted as queued. The record wake and the deadline window's wake schedule one action,
 *  and reactor-c refuses a second event at a queued tag. */
int64_t ros2_lf_corr_claim_wake(int edge, int64_t w);

/** A drain-side lf_schedule(corr_drain_E) returned <= 0: terminalize the carrier. */
void ros2_lf_corr_schedule_refused(int edge);

/** Shutdown reaction of an edge host: mark the edge closed. The epoch close and the
 *  reconcile run in ros2_lf_runtime_shutdown, after every reaction of the stop tag. */
void ros2_lf_corr_close(int edge);

/* --- Cross-federate correspondence ------------------------------
 * A cross-federate admitted edge carries the publication records themselves
 * over the LF federated connection `xcorr_g<G> -> xcorr_g<G> after d`. A plain
 * struct, sent by value. One message per publishing time (D2).
 */
#define RCLLF_XCORR_MAX 8
/* The layout gained pub_microstep, so the magic moved from
 * 0x52434C58 ("RCLX") to the next value; a peer built before the change exits 3
 * at the magic check instead of reading a field it never wrote. */
#define RCLLF_XCORR_MAGIC 0x52434C5Bu
/* The layout gained flags, complete_seq and send_phys, so
 * the magic moved from 0x52434C59 to 0x52434C5A, for the reason above. The
 * record gained act_phys (part 3, the causal window), so it moved again to
 * 0x52434C5B. */
/* flags bit: the message is the COMPLETE record of one publishing activation.
 * The receiver checks complete_seq for gaps. Changed 2026-11-03:
 * an activation that published nothing on the edge sends no message. */
#define RCLLF_XCORR_COMPLETE 0x1u

typedef struct rcllf_xcorr_rec_t
{
  uint8_t gid[16];
  uint64_t ordinal;
  int64_t phys;          /* physical commit stamp of this record (R1-1) */
  /* S(a), the physical start of the publishing
   * activation's window, after its inputs settled and before its user code.
   * The deadline is S(a) + d + T for every record of the activation. -1 when
   * the publication had no activation scope, then phys is the base. */
  int64_t act_phys;
} rcllf_xcorr_rec_t;

typedef struct rcllf_xcorr_t
{
  uint32_t magic;        /* RCLLF_XCORR_MAGIC, a layout mismatch exits 3 at the receiver */
  uint32_t count;        /* records in rec[], 1..RCLLF_XCORR_MAX */
  uint64_t epoch;        /* publisher run epoch */
  int64_t pub_time;      /* absolute LF time of the publishing tag */
  /* Microstep m of the publishing tag of a d = 0 send, so the
   * receiver keys the records at (t, m + 1) as one process does. A d > 0
   * send carries m too, unused: those records release at (t + d, 0). 0 for
   * the flush. */
  uint32_t pub_microstep;
  uint32_t flags;        /* RCLLF_XCORR_COMPLETE (the flush too) */
  uint8_t boot_id[16];   /* publisher host kernel boot id, the one-clock check */
  /* The activation's completion number on this edge, 1, 2,
   * ... per publishing tag; per time for the flush. */
  uint64_t complete_seq;
  /* The sender's table clock when it filled the message: the receiver's
   * settle wait of the completion is measured from it (trace only). */
  int64_t send_phys;
  rcllf_xcorr_rec_t rec[RCLLF_XCORR_MAX];
} rcllf_xcorr_t;

/** Egress publishing reaction, after the invoke: 1 the first time in
 *  the current logical time that records are queued, then the reaction
 *  schedules the flush action 1 ns later. */
int ros2_lf_corr_flush_due(int edge);

/** Egress: lf_schedule refused the flush (past the stop tag). The records stay
 *  unsent and reconcile reports flush_refused. */
void ros2_lf_corr_flush_refused(int edge);

/** Egress flush reaction at t + 1 ns: fill `out` with every record of
 *  time t, pub_time = t. The link is `after d - 1 ns`, so they arrive at t + d.
 *  the message is time t's COMPLETE record with its completion
 *  number, so a lost flush is a CONTROL_FAILURE at the receiver.
 *  Returns 1 when there is a message to lf_set, 0 when no record is queued.
 *  A record of another time exits 3. */
uint32_t ros2_lf_corr_flush_remote(void * self, int edge, rcllf_xcorr_t * out);

/** Egress publishing reaction of an edge that sends at the
 *  publishing tag (every edge but a flush edge), after the invoke. Fills `out`
 *  with the records of the current tag (t, m), cumulative over the reactions
 *  at that tag, pub_time = t, pub_microstep = m. The link is `after d`: with
 *  d = 0 they arrive at (t, m + 1), with d > 0 the k-th message of time t
 *  arrives at (t + d, k). The message is the activation's
 *  COMPLETE record (flags, complete_seq). Returns 1 (lf_set), 0 when the
 *  activation published nothing on the edge (changed 2026-11-03: no message,
 *  no completion number) or when the RCLLF_TEST_XCORR_DROP_COMPLETE test hook
 *  withholds it. */
uint32_t ros2_lf_corr_send_remote(void * self, int edge, rcllf_xcorr_t * out);

/** The federate reactor's guard on a legacy stub output port with
 *  `after d`, d > 0. Exits 3 when the port is written at more than
 *  `max_per_time` microsteps of one logical time. Centralized passes 2
 *  (reactor-c merges a third message at the receiver), decentralized passes 1
 *  (the receiver judges a second message of one tag tardy and
 *  moves or drops it). */
void ros2_lf_xfed_legacy_sent(const char * port, uint32_t max_per_time);

/** Ingress arrive reaction on the xcorr input: expectations from the records.
 *  Returns the wake delay (ns) or -1. */
int64_t ros2_lf_corr_arrive_remote(void * self, int edge, const rcllf_xcorr_t * x);

/** What the deadline or message-lost callback of an
 *  admitted edge that is running now was raised for. `intended_elapsed_ns` is
 *  the intended tag of the missing message (publish tag + edge delay) as
 *  elapsed logical time, -1 when unknown; `tardy` is 1 when its record arrived
 *  after that tag on a decentralized link and was dropped. Valid only inside
 *  the callback, on the thread that runs it. Returns 1 and fills `out` there,
 *  0 anywhere else (a take-mode loss report included). */
typedef struct rcllf_lost_ctx
{
  int64_t intended_elapsed_ns;
  uint64_t ordinal;
  uint8_t gid[16];
  int32_t tardy;
} rcllf_lost_ctx_t;
int ros2_lf_message_lost_context(rcllf_lost_ctx_t * out);

/* --- Cross-federate take link -------------------------------------
 * A legacy stub link across federates carries, with the write count, the
 * events of the publishing process's tag log of the topic since its last
 * trigger (tag_log.hpp: each publish with its writer GID and sequence number,
 * each ros2_lf_claim_publish). The receiving federate applies them to its own
 * tag log before the take, so a late sample is dropped by tag as in one
 * process. A plain struct, sent by value.
 *
 * Every message also announces the GIDs of the writer federate's publishers
 * of the topic (each goes through the publish gate), so the receiver tells a
 * gated writer whose events are still on the way (hold its sample) from a
 * writer outside rcllf (deliver untagged) by GID, not by waiting (second
 * review).
 *
 * The two sizes are a contract of the wire struct, which LF sends
 * by value on every trigger of the link (1 KiB of events and 256 bytes of
 * GIDs here). Provenance: CONVENTION, they are wire sizes, not derived from a
 * system's configuration. RCLLF_XTAG_MAX bounds the tag log events of one
 * topic between two messages of a link, RCLLF_XTAG_WRITERS the publishers of
 * the topic in the writer federate. A run that exceeds either one cannot drop
 * a late sample by its tag any more, so the reader ends the run with exit
 * code 3 and the reason (ros2_lf_xtag_import), where it used to warn once and
 * go on with the count rule, which can deliver a sample one tag late. Raise
 * them here, for every federate at once, for a system that needs more.
 */
#define RCLLF_XTAG_MAX 32
#define RCLLF_XTAG_WRITERS 16
#define RCLLF_XTAG_MAGIC 0x52435433u

#define RCLLF_XTAG_PUBLISH 1u    /* a publish: flags carry in-reaction and GID-known */
#define RCLLF_XTAG_CLAIM 2u      /* a ros2_lf_claim_publish */
#define RCLLF_XTAG_IN_REACTION 1u
#define RCLLF_XTAG_GID_OK 2u

typedef struct rcllf_xtag_ev_t
{
  uint8_t gid[16];
  uint64_t seq;
  uint32_t kind;         /* RCLLF_XTAG_PUBLISH or RCLLF_XTAG_CLAIM */
  uint32_t flags;        /* RCLLF_XTAG_IN_REACTION | RCLLF_XTAG_GID_OK */
} rcllf_xtag_ev_t;

typedef struct rcllf_xtag_t
{
  uint32_t magic;        /* RCLLF_XTAG_MAGIC, a layout mismatch exits 3 at the receiver */
  int32_t count;         /* the trigger's write count, what a plain stub link carries */
  uint64_t session;      /* the writer process's journal session; a change breaks the topic */
  uint64_t first;        /* journal ordinal of ev[0] */
  uint32_t n;            /* events in ev[], 0..RCLLF_XTAG_MAX */
  uint32_t overflow;     /* 1: older events of the link left out (another link may carry them) */
  uint32_t nw;           /* GIDs in writer[], 0..RCLLF_XTAG_WRITERS */
  uint32_t writers_more; /* 1: the federate has more publishers of the topic than writer[] holds */
  uint8_t writer[RCLLF_XTAG_WRITERS][16];  /* the writer federate's publishers of the topic */
  rcllf_xtag_ev_t ev[RCLLF_XTAG_MAX];
} rcllf_xtag_t;

/** Publishing federate's reaction on a legacy stub output `link` that leaves
 *  the federate: `out` gets `count`, the newest RCLLF_XTAG_MAX events of
 *  `topic` since the last pack on `link`, and the GIDs of the federate's
 *  publishers of `topic`. */
void ros2_lf_xtag_pack(const char * topic, const char * link, int32_t count, rcllf_xtag_t * out);

/** Receiving federate's reaction on that input, before the subscriber's take:
 *  apply the events to the tag log of `topic` and return the write count.
 *  Missing events (a gap no link filled) or a changed writer session (a
 *  restarted writer) put the topic on the count rule with one WARN naming `edge`. */
int32_t ros2_lf_xtag_import(const char * topic, const char * edge, const rcllf_xtag_t * in);

/** Receiving federate's startup: one writer federate publishes `topic` to
 *  this one over take links, from `publishers` gated publishers (the
 *  generator counts them from the analysis). A sample of a writer not
 *  announced is held, not delivered as untagged, until an import arrived and
 *  `publishers` GIDs are announced, so a gated publisher created after the
 *  federate's earlier messages (a lifecycle publisher) is never taken for a
 *  plain one. A crossing topic has one writer
 *  federate: the generator refuses publishers in two, and a second writer
 *  session puts the topic on the count rule. */
void ros2_lf_xtag_expect(const char * topic, int32_t publishers);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_LF_BINDING__ROS2_LF_BINDING_H_

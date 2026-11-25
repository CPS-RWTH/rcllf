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

/// @file rcllf_trace_schema.h
/// @brief Semantic trace schema `rcllf.events/1` (D2).
///
/// Single source of truth for the event names, categories, fields and codes
/// of the `rcllf` LTTng provider. `lf_eval/semantic/schema.py` mirrors this
/// file and `test_schema_parity.py` fails on any drift. Enum values are
/// stable wire values: append, never renumber.
#ifndef ROS2_LF_BINDING__RCLLF_TRACE_SCHEMA_H_
#define ROS2_LF_BINDING__RCLLF_TRACE_SCHEMA_H_

#define RCLLF_TRACE_SCHEMA_NAME "rcllf.events"
#define RCLLF_TRACE_SCHEMA_MAJOR 1
#define RCLLF_TRACE_SCHEMA_MINOR 8
#define RCLLF_TRACE_PROVIDER_VERSION 1
#define RCLLF_TRACE_GID_LEN 16
#define RCLLF_TRACE_NO_EDGE 0xFFFFFFFFu
#define RCLLF_TRACE_NO_RID (-1)

/* Categories: one seq counter per (epoch, category). */
enum rcllf_trace_cat
{
  RCLLF_CAT_LIFECYCLE = 0,
  RCLLF_CAT_PUBLICATION = 1,
  RCLLF_CAT_DELIVERY = 2,
  RCLLF_CAT_SAMPLE = 3,
  RCLLF_CAT_DISPATCH = 4,
  RCLLF_CAT_CLEANUP = 5,
  RCLLF_CAT_POLICY = 6,  /* reserved, no producer */
  RCLLF_CAT_TUPLE = 7,   /* reserved, no producer (message_filters) */
  RCLLF_CAT_COUNT = 8,
};

/* Expectation outcomes, wire copy of corr::Outcome (correspondence.hpp). */
enum rcllf_trace_outcome
{
  RCLLF_OUTCOME_PENDING = 0,
  RCLLF_OUTCOME_MATCHED = 1,
  RCLLF_OUTCOME_NOT_READY = 2,
  RCLLF_OUTCOME_TIMED_OUT = 3,
  RCLLF_OUTCOME_LOST = 4,
  RCLLF_OUTCOME_REJECTED = 5,
  RCLLF_OUTCOME_UNBOUND = 6,
  RCLLF_OUTCOME_UNSUPPORTED = 7,
  RCLLF_OUTCOME_DUPLICATE = 8,
  RCLLF_OUTCOME_OVERFLOW = 9,
  RCLLF_OUTCOME_EPOCH_CLOSE = 10,
  RCLLF_OUTCOME_PUSH_OVERFLOW = 11,
  RCLLF_OUTCOME_SCHEDULE_REFUSED = 12,
  RCLLF_OUTCOME_READER_REPLACED = 13,
  RCLLF_OUTCOME_NONE = 255,  /* dispatch_end of a timer or legacy entry */
};

/* Sample outcomes, wire copy of corr::SampleOutcome. */
enum rcllf_trace_sample_outcome
{
  RCLLF_SAMPLE_RETAINED = 0,
  RCLLF_SAMPLE_UNSUPPORTED = 1,
  RCLLF_SAMPLE_UNRESOLVED = 2,
  RCLLF_SAMPLE_LATE = 3,
  RCLLF_SAMPLE_DUPLICATE = 4,
  RCLLF_SAMPLE_REJECTED_IDENTITY = 5,
  RCLLF_SAMPLE_OVERFLOW = 6,
  RCLLF_SAMPLE_EPOCH_CLOSE = 7,
  RCLLF_SAMPLE_STALE_READER = 8,
  RCLLF_SAMPLE_POOLED = 9,
};

/* Publication origin, wire copy of corr::Origin. */
enum rcllf_trace_origin
{
  RCLLF_ORIGIN_ADMITTED = 0,
  RCLLF_ORIGIN_NO_CONTEXT = 1,
  RCLLF_ORIGIN_UNDECLARED_EFFECT = 2,
  RCLLF_ORIGIN_EDGE_NOT_ADMITTED = 3,
  RCLLF_ORIGIN_UNSUPPORTED_API = 4,
  RCLLF_ORIGIN_NESTED = 5,
};

enum rcllf_trace_pub_result
{
  RCLLF_PUB_ACCEPTED = 0,
  RCLLF_PUB_FAILED = 1,
};

/* Publish API, wire copy of rcllf_gate_api (publication_gate_api.h). */
enum rcllf_trace_api
{
  RCLLF_API_PUBLISH = 0,
  RCLLF_API_SERIALIZED = 1,
  RCLLF_API_LOANED = 2,
};

enum rcllf_trace_entry_kind
{
  RCLLF_ENTRY_TIMER = 0,
  RCLLF_ENTRY_LEGACY_SUB = 1,
  RCLLF_ENTRY_CORR_SUB = 2,
  RCLLF_ENTRY_SERVICE = 3,
  RCLLF_ENTRY_ACTION = 4,
  RCLLF_ENTRY_EXTERNAL_SUB = 5,  /* minor 1: subscription on a declared external topic */
};

enum rcllf_trace_carrier_op
{
  RCLLF_CARRIER_EMIT = 0,
  RCLLF_CARRIER_ARRIVE = 1,
  RCLLF_CARRIER_DRAIN = 2,
  RCLLF_CARRIER_REFUSED = 3,
};

enum rcllf_trace_cleanup_reason
{
  RCLLF_CLEANUP_SHUTDOWN = 0,
  RCLLF_CLEANUP_EPOCH_CLOSE = 1,
};

/* dispatch_end.missing_hook, the trace_corr JSON field of
 * the same name. NA: the dispatch delivered no missing message. */
enum rcllf_trace_missing_hook
{
  RCLLF_MISSING_HOOK_NA = 0,
  RCLLF_MISSING_HOOK_NONE = 1,
  RCLLF_MISSING_HOOK_SHUTDOWN = 2,
  RCLLF_MISSING_HOOK_DEADLINE = 3,
  RCLLF_MISSING_HOOK_MESSAGE_LOST = 4,
};

/* What became of one message on an external topic.
 * ARRIVAL: anchored at arrival, scheduled by lf_schedule_copy (the only
 * value before minor 5). The rest are the stamp anchor's dispositions: the
 * tag was inserted (ADMITTED), the message arrived after
 * header.stamp + delay_ns (LATE), it was on time but its tag was at or
 * before the current tag at insert (OVERRUN, the ingress reserve did not
 * hold), two on-time messages of the topic fell on one grid cell
 * (COLLISION, both refused), the tag was after the stop tag (AFTER_STOP),
 * or reactor-c refused the schedule (REFUSED, stopping). A
 * tf2_msgs/TFMessage on a stamp-anchored /tf or /tf_static whose
 * transforms do not all carry one header.stamp, or carry none, is ended
 * unadmitted (TF_MIXED_STAMPS, stamp_ns 0): it has no single stamp to
 * anchor at, and splitting or picking one would guess. */
enum rcllf_trace_ext_disposition
{
  RCLLF_EXT_ARRIVAL = 0,
  RCLLF_EXT_ADMITTED = 1,
  RCLLF_EXT_LATE = 2,
  RCLLF_EXT_OVERRUN = 3,
  RCLLF_EXT_COLLISION = 4,
  RCLLF_EXT_AFTER_STOP = 5,
  RCLLF_EXT_REFUSED = 6,
  RCLLF_EXT_TF_MIXED_STAMPS = 7,
};

/*
 * Event table: X(event, category). Field lists: RCLLF_FIELDS_<event>(F) with
 * F(ctf_kind, c_type, name). ctf_kind is one of int, text (C string), gid (u8[16])
 * or totals (u64[RCLLF_CAT_COUNT], indexed by rcllf_trace_cat). Every event
 * also carries the common fields epoch (u64), cat (u8) and seq (u64), first
 * and in that order. run_end.totals[c] is the number of category-c events
 * that took a seq, run_end.attempts[c] the number the producers offered
 * (including those not taken because the event was disabled).
 *
 * The run_end group is emitted after trace close: run_end_publisher and
 * run_end_edge records carry seq 0 and are counted by run_end itself
 * (n_publishers, n_edges). run_end is the last event of an epoch.
 *
 * run_begin.ancestry (schema 1.3) is the process's parent PID chain
 * read from /proc when run_begin is emitted, space separated, parent first, up
 * to PID 1 (or the first PID whose parent cannot be read).
 *
 * run_begin.run_token (schema 1.4) is the value of the environment
 * variable RCLLF_RUN_TOKEN, read once per process, or "" when it is unset. A
 * driver that exports a fresh random token per launch tells its own processes
 * from any other program in the same trace session by it.
 *
 * run_begin is emitted before reactor-c picks its start time, so its
 * lf_start_time_ns is 0 (not yet known). lf_start (schema 1.1) is emitted once
 * per epoch at the first startup dispatch and carries the start tag (in a
 * federation the RTI-negotiated one).
 */
#define RCLLF_TRACE_EVENTS(X) \
  X(run_begin, RCLLF_CAT_LIFECYCLE) \
  X(provider, RCLLF_CAT_LIFECYCLE) \
  X(backlog, RCLLF_CAT_LIFECYCLE) \
  X(lf_start, RCLLF_CAT_LIFECYCLE) \
  X(publication, RCLLF_CAT_PUBLICATION) \
  X(expectation, RCLLF_CAT_DELIVERY) \
  X(carrier, RCLLF_CAT_DELIVERY) \
  X(terminal, RCLLF_CAT_DELIVERY) \
  X(sample, RCLLF_CAT_SAMPLE) \
  X(dispatch_begin, RCLLF_CAT_DISPATCH) \
  X(dispatch_end, RCLLF_CAT_DISPATCH) \
  X(cleanup, RCLLF_CAT_CLEANUP) \
  X(external_arrival, RCLLF_CAT_SAMPLE) \
  X(ingress_callback_start, RCLLF_CAT_DISPATCH) \
  X(actuation_late, RCLLF_CAT_PUBLICATION) \
  X(tf2_request, RCLLF_CAT_SAMPLE) \
  X(tf2_outcome, RCLLF_CAT_SAMPLE) \
  X(run_end_publisher, RCLLF_CAT_LIFECYCLE) \
  X(run_end_edge, RCLLF_CAT_LIFECYCLE) \
  X(run_end, RCLLF_CAT_LIFECYCLE)

#define RCLLF_FIELDS_run_begin(F) \
  F(int, uint32_t, schema_major) \
  F(int, uint32_t, schema_minor) \
  F(text, const char *, binding_version) \
  F(text, const char *, static_manifest_hash) \
  F(text, const char *, generated_source_hash) \
  F(text, const char *, model_hash) \
  F(text, const char *, build_id) \
  F(text, const char *, executable) \
  F(int, int64_t, pid) \
  F(text, const char *, boot_id) \
  F(text, const char *, host) \
  F(int, int64_t, clock_monotonic_ns) \
  F(int, int64_t, clock_realtime_ns) \
  F(int, int64_t, clock_lf_physical_ns) \
  F(int, int64_t, clock_read_gap_ns) \
  F(int, int64_t, res_monotonic_ns) \
  F(int, int64_t, res_realtime_ns) \
  F(int, int64_t, lf_start_time_ns) \
  F(int, uint8_t, lf_clock_sync) \
  F(text, const char *, ancestry) \
  F(text, const char *, run_token)

#define RCLLF_FIELDS_lf_start(F) \
  F(int, int64_t, lf_start_time_ns) \
  F(int, uint32_t, lf_start_microstep)

#define RCLLF_FIELDS_provider(F) \
  F(int, uint8_t, category) \
  F(int, uint32_t, provider_version) \
  F(int, uint8_t, compiled_in)

#define RCLLF_FIELDS_backlog(F) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, first_ordinal) \
  F(int, uint64_t, last_ordinal) \
  F(int, uint64_t, refused)

#define RCLLF_FIELDS_publication(F) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, uint8_t, result) \
  F(int, uint8_t, api) \
  F(int, uint8_t, origin) \
  F(int, int32_t, rid) \
  F(int, uint64_t, dispatch_id) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep)

#define RCLLF_FIELDS_expectation(F) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, uint32_t, edge) \
  F(int, uint32_t, instance) \
  F(int, uint32_t, generation)

#define RCLLF_FIELDS_carrier(F) \
  F(int, uint32_t, edge) \
  F(int, uint8_t, op) \
  F(int, uint32_t, count) \
  F(int, int64_t, release_tag_time)

#define RCLLF_FIELDS_terminal(F) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, uint32_t, edge) \
  F(int, uint8_t, outcome)

#define RCLLF_FIELDS_sample(F) \
  F(int, uint32_t, edge) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, uint8_t, outcome) \
  F(int, uint64_t, retained_id) \
  F(int, int64_t, arrival_physical_ns)

#define RCLLF_FIELDS_dispatch_begin(F) \
  F(int, uint64_t, dispatch_id) \
  F(int, int32_t, rid) \
  F(int, uint8_t, entry_kind) \
  F(int, uint32_t, edge) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep)

/* cutoff is cutoff tag time in ns of the
 * expectation this dispatch joined (publish tag + d_e + T_e), -1 when it has
 * none: a local_now edge (decided when the publish returns; a local
 * sub_rebind or deferred edge is not one and has a cutoff), an edge without a
 * timeout, a dispatch that is no correspondence drain. -1 is no tag.
 * missing_hook is an rcllf_trace_missing_hook code. */
#define RCLLF_FIELDS_dispatch_end(F) \
  F(int, uint64_t, dispatch_id) \
  F(int, uint8_t, outcome) \
  F(int, uint64_t, retained_id) \
  F(int, int64_t, join_wait_ns) \
  F(int, uint8_t, callback_ran) \
  F(int, int64_t, cutoff) \
  F(int, uint8_t, missing_hook)

#define RCLLF_FIELDS_cleanup(F) \
  F(int, uint64_t, released_retained) \
  F(int, uint64_t, carrier_records_closed) \
  F(int, uint8_t, reason)

/* One message on a declared external topic, taken by
 * the ingress thread. record_id is the dispatch_end.retained_id of the
 * EXTERNAL_SUB dispatch that runs its callback. Minor 5 adds
 * the stamp anchor's view: stamp_ns (header.stamp, 0 at arrival anchor),
 * tag_ns (the tag the event was inserted at, elapsed since start, -1 when
 * not admitted), insert_ns (physical time right after the insert, 0 when
 * not admitted) and disposition (rcllf_trace_ext_disposition). */
#define RCLLF_FIELDS_external_arrival(F) \
  F(int, int32_t, rid) \
  F(gid, uint8_t, gid) \
  F(int, uint64_t, ordinal) \
  F(int, uint64_t, record_id) \
  F(int, int64_t, arrival_physical_ns) \
  F(int, int64_t, stamp_ns) \
  F(int, int64_t, tag_ns) \
  F(int, int64_t, insert_ns) \
  F(int, uint8_t, disposition)

/* One infrastructure service request handed to its
 * rclcpp service by the ingress drain, the LTTng copy of the callback_start
 * JSON line. node is the ingress key, callback the service suffix and
 * ingress_record_id the ingress record of the request. */
#define RCLLF_FIELDS_ingress_callback_start(F) \
  F(text, const char *, entry) \
  F(int, int32_t, rid) \
  F(text, const char *, node) \
  F(text, const char *, callback) \
  F(int, int32_t, lf_thread_id) \
  F(int, int64_t, gettid) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep) \
  F(int, uint64_t, ingress_record_id)

/* A publish of a topic with a configured
 * `publishers.<node>.<topic>.output_deadline_ns` that was late. Emitted
 * just before the real rcl publish, which still happens with the unchanged
 * message. rid is the publishing reaction, topic the resolved topic,
 * tag_time/tag_microstep the reaction's tag (absolute ns), physical_ns
 * lf_time_physical() at the check, lateness_ns = physical_ns - tag_time and
 * bound_ns the configured output_deadline_ns (late: lateness_ns > bound_ns). */
#define RCLLF_FIELDS_actuation_late(F) \
  F(int, int32_t, rid) \
  F(text, const char *, topic) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep) \
  F(int, int64_t, physical_ns) \
  F(int, int64_t, lateness_ns) \
  F(int, int64_t, bound_ns)

/* One rcllf_tf2::AcquisitionLookup request and its
 * outcome, the LTTng copies of the RCLLF_TF2 tf_request / tf_outcome JSON
 * lines. node is the adapter's node name, id the request id (unique per
 * adapter), stamp_ns the requested stamp, tag_time/tag_microstep the tag of
 * the request (tf2_request) or of the decision (tf2_outcome), absolute ns.
 * on_timeout and outcome use the execution.yaml spelling ("reject",
 * "failure", "fallback:<name>"; outcome also "ok"). wait_logical_ns is the
 * decision tag minus the request tag, wait_physical_ns the steady-clock time
 * between them and latest_common_ns the Buffer's latest common time of the
 * frame pair at the decision (-1: none).
 * Both events end with adapter, the adapter's
 * construction ordinal on its node, so (node, adapter, id) pairs them when a
 * node has several adapters, and the Buffer's available interval for the
 * frame pair at that moment: oldest_common_ns (the latest of the per-link
 * oldest stamps along the path, static links excluded) and, on tf2_request,
 * latest_common_ns too (-1: tf2 cannot connect the frames; 0: no dynamic
 * link, any stamp; -2 on both: unavailable, the path holds no single exact
 * range, i.e. a link whose child was reparented within its stored samples or
 * a dynamic link stamped only at 0). */
#define RCLLF_FIELDS_tf2_request(F) \
  F(text, const char *, node) \
  F(int, uint64_t, id) \
  F(text, const char *, target) \
  F(text, const char *, source) \
  F(int, int64_t, stamp_ns) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep) \
  F(int, int64_t, wait_bound_ns) \
  F(text, const char *, on_timeout) \
  F(int, uint32_t, adapter) \
  F(int, int64_t, oldest_common_ns) \
  F(int, int64_t, latest_common_ns)

#define RCLLF_FIELDS_tf2_outcome(F) \
  F(text, const char *, node) \
  F(int, uint64_t, id) \
  F(text, const char *, outcome) \
  F(int, int64_t, stamp_ns) \
  F(int, int64_t, tag_time) \
  F(int, uint32_t, tag_microstep) \
  F(int, int64_t, wait_logical_ns) \
  F(int, int64_t, wait_physical_ns) \
  F(int, int64_t, latest_common_ns) \
  F(int, uint32_t, adapter) \
  F(int, int64_t, oldest_common_ns)

#define RCLLF_FIELDS_run_end_publisher(F) \
  F(gid, uint8_t, gid) \
  F(int, uint8_t, tracked) \
  F(int, uint64_t, accepted) \
  F(int, uint64_t, failed) \
  F(int, uint64_t, refused_no_context) \
  F(int, uint64_t, refused_undeclared_effect) \
  F(int, uint64_t, refused_edge_not_admitted) \
  F(int, uint64_t, nested) \
  F(int, uint64_t, unsupported_api) \
  F(int, uint64_t, first_traced_ordinal) \
  F(int, uint64_t, untracked)

#define RCLLF_FIELDS_run_end_edge(F) \
  F(int, uint32_t, edge) \
  F(int, uint64_t, expectations) \
  F(int, uint64_t, matched) \
  F(int, uint64_t, terminal_other) \
  F(int, uint64_t, samples_taken) \
  F(int, uint64_t, samples_retained) \
  F(int, uint64_t, orphan_dispatches) \
  F(int, uint64_t, count_mismatch)

#define RCLLF_FIELDS_run_end(F) \
  F(totals, uint64_t, totals) \
  F(totals, uint64_t, attempts) \
  F(int, uint32_t, n_publishers) \
  F(int, uint32_t, n_edges) \
  F(int, uint64_t, untracked_publishes) \
  F(int, uint64_t, identity_faults) \
  F(int, uint8_t, reconciled) \
  F(int, int64_t, clock_monotonic_ns) \
  F(int, int64_t, clock_realtime_ns) \
  F(int, int64_t, clock_lf_physical_ns) \
  F(int, int64_t, clock_read_gap_ns)

#endif  // ROS2_LF_BINDING__RCLLF_TRACE_SCHEMA_H_

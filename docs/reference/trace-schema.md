# Semantic trace schema (`rcllf.events/1`)

The `rcllf` LTTng UST provider in `libros2_lf_binding.so` records the identity of
every tracked publication and follows it through expectation, sample and LF
dispatch. The source of truth is
`packages/lf-frame/ros2_lf_binding/include/ros2_lf_binding/rcllf_trace_schema.h`.
`tools/lf_eval/lf_eval/semantic/schema.py` mirrors it, and
`tools/lf_eval/tests/semantic/test_schema_parity.py` fails when the two drift.

## Produce one

Trace a generated LF program (see [Tracing](../features/tracing.md#recording-a-trace)),
then dump the `rcllf` events of the run with `babeltrace2`:

```bash
babeltrace2 output/experiments/trace_<timestamp>/ust | grep rcllf:dispatch_end
```

One line looks like this (a run of the generated `pubsub_example` program):

```text
[10:40:44.418221772] (+0.000004220) enzo rcllf:dispatch_end: { cpu_id = 21 }, { vpid = 2030779, vtid = 2030813, procname = "Bringup" }, { epoch = 15818350373098743036, cat = 4, seq = 2, dispatch_id = 1, outcome = 255, retained_id = 0, join_wait_ns = 0, callback_ran = 1, cutoff = -1, missing_hook = 0 }
```

The first brace holds the CPU, the second the contexts `docker/trace.sh` adds. The
third holds the event fields: the three common fields (`epoch`, `cat`, `seq`) and the
fields of `dispatch_end` from the table below. A stock ROS 2 run records no `rcllf`
events. The provider also has two more groups, `rcllf_stop` and `rcllf_dds`. They are
not recorded by `docker/trace.sh`.

## Changes by version

| Schema | Adds |
|---|---|
| 1.0 | the lifecycle, publication, delivery, sample, dispatch and cleanup events |
| 1.1 | `lf_start`, external arrivals |
| 1.2 | `cutoff` and `missing_hook` on `dispatch_end`, `ingress_callback_start` |
| 1.3 | process ancestry |
| 1.4 | run token |
| 1.5 | stamp-anchored arrivals |
| 1.6 | `actuation_late` |
| 1.7 | `tf2_request`, `tf2_outcome` |
| 1.8 | tf2 adapter and available interval fields |

The sections after the event table explain the fields each version added.

## Common fields

Every event starts with `epoch` (u64, the run epoch), `cat` (u8, category)
and `seq` (u64). `seq` counts per (epoch, category), starting at 1. It is taken
only when the event is enabled in the session and the trace is not closed yet.
The `run_end` group (`run_end_publisher`, `run_end_edge`, `run_end`) is emitted
after trace close with `seq` 0. `run_end.totals[c]` is the number of category `c`
events that took a seq, `run_end.attempts[c]` the number the producers offered.

## Categories

| Code | Category | Producer |
|---|---|---|
| 0 | `lifecycle` | runtime lifecycle |
| 1 | `publication` | publication gate |
| 2 | `delivery` | correspondence table and runtime |
| 3 | `sample` | ingress |
| 4 | `dispatch` | reaction dispatch and correspondence runtime |
| 5 | `cleanup` | correspondence runtime |
| 6 | `policy` | reserved |
| 7 | `tuple` | reserved (message_filters) |

## Events

Field kinds: `int` (CTF integer), `text` (CTF string), `gid` (16 bytes), `totals` (one u64 per category).

| Event | Category | Fields beyond the common three |
|---|---|---|
| `run_begin` | `lifecycle` | `schema_major` (int), `schema_minor` (int), `binding_version` (text), `static_manifest_hash` (text), `generated_source_hash` (text), `model_hash` (text), `build_id` (text), `executable` (text), `pid` (int), `boot_id` (text), `host` (text), `clock_monotonic_ns` (int), `clock_realtime_ns` (int), `clock_lf_physical_ns` (int), `clock_read_gap_ns` (int), `res_monotonic_ns` (int), `res_realtime_ns` (int), `lf_start_time_ns` (int), `lf_clock_sync` (int), `ancestry` (text), `run_token` (text) |
| `provider` | `lifecycle` | `category` (int), `provider_version` (int), `compiled_in` (int) |
| `backlog` | `lifecycle` | `gid` (gid), `first_ordinal` (int), `last_ordinal` (int), `refused` (int) |
| `lf_start` | `lifecycle` | `lf_start_time_ns` (int), `lf_start_microstep` (int) |
| `publication` | `publication` | `gid` (gid), `ordinal` (int), `result` (int), `api` (int), `origin` (int), `rid` (int), `dispatch_id` (int), `tag_time` (int), `tag_microstep` (int) |
| `expectation` | `delivery` | `gid` (gid), `ordinal` (int), `edge` (int), `instance` (int), `generation` (int) |
| `carrier` | `delivery` | `edge` (int), `op` (int), `count` (int), `release_tag_time` (int) |
| `terminal` | `delivery` | `gid` (gid), `ordinal` (int), `edge` (int), `outcome` (int) |
| `sample` | `sample` | `edge` (int), `gid` (gid), `ordinal` (int), `outcome` (int), `retained_id` (int), `arrival_physical_ns` (int) |
| `dispatch_begin` | `dispatch` | `dispatch_id` (int), `rid` (int), `entry_kind` (int), `edge` (int), `gid` (gid), `ordinal` (int), `tag_time` (int), `tag_microstep` (int) |
| `dispatch_end` | `dispatch` | `dispatch_id` (int), `outcome` (int), `retained_id` (int), `join_wait_ns` (int), `callback_ran` (int), `cutoff` (int), `missing_hook` (int) |
| `cleanup` | `cleanup` | `released_retained` (int), `carrier_records_closed` (int), `reason` (int) |
| `external_arrival` | `sample` | `rid` (int), `gid` (gid), `ordinal` (int), `record_id` (int), `arrival_physical_ns` (int), `stamp_ns` (int), `tag_ns` (int), `insert_ns` (int), `disposition` (int) |
| `ingress_callback_start` | `dispatch` | `entry` (text), `rid` (int), `node` (text), `callback` (text), `lf_thread_id` (int), `gettid` (int), `tag_time` (int), `tag_microstep` (int), `ingress_record_id` (int) |
| `actuation_late` | `publication` | `rid` (int), `topic` (text), `tag_time` (int), `tag_microstep` (int), `physical_ns` (int), `lateness_ns` (int), `bound_ns` (int) |
| `tf2_request` | `sample` | `node` (text), `id` (int), `target` (text), `source` (text), `stamp_ns` (int), `tag_time` (int), `tag_microstep` (int), `wait_bound_ns` (int), `on_timeout` (text), `adapter` (int), `oldest_common_ns` (int), `latest_common_ns` (int) |
| `tf2_outcome` | `sample` | `node` (text), `id` (int), `outcome` (text), `stamp_ns` (int), `tag_time` (int), `tag_microstep` (int), `wait_logical_ns` (int), `wait_physical_ns` (int), `latest_common_ns` (int), `adapter` (int), `oldest_common_ns` (int) |
| `run_end_publisher` | `lifecycle` | `gid` (gid), `tracked` (int), `accepted` (int), `failed` (int), `refused_no_context` (int), `refused_undeclared_effect` (int), `refused_edge_not_admitted` (int), `nested` (int), `unsupported_api` (int), `first_traced_ordinal` (int), `untracked` (int) |
| `run_end_edge` | `lifecycle` | `edge` (int), `expectations` (int), `matched` (int), `terminal_other` (int), `samples_taken` (int), `samples_retained` (int), `orphan_dispatches` (int), `count_mismatch` (int) |
| `run_end` | `lifecycle` | `totals` (totals), `attempts` (totals), `n_publishers` (int), `n_edges` (int), `untracked_publishes` (int), `identity_faults` (int), `reconciled` (int), `clock_monotonic_ns` (int), `clock_realtime_ns` (int), `clock_lf_physical_ns` (int), `clock_read_gap_ns` (int) |

`run_begin` is emitted before reactor-c picks its start time, so its
`lf_start_time_ns` is 0 (not yet known). `lf_start` (schema 1.1) is emitted once
per epoch at the first startup dispatch and carries reactor-c's start tag. In a
federation that is the start tag negotiated with the RTI, the same in every
federate. Tags are measured from it. A schema 1.0 trace has no `lf_start`, so a reader falls back
to the earliest observed tag and says so.

## Fields added in 1.1: External arrivals

Schema 1.1 also adds the event `external_arrival` and the entry kind
`EXTERNAL_SUB`. A subscription on a topic declared in `external_topics` of
`execution.yaml` has no in-system publisher. The ingress thread takes each of its
messages and emits one `external_arrival`. `record_id` is a process-wide counter
and is also the payload of the LF physical action the binding schedules at that
moment, so `arrival_physical_ns` is the physical time the message's tag derives
from. `gid` and `ordinal` are the sender's publisher GID and publication sequence
number as DDS reports them.

The callback run that consumes the message emits `dispatch_begin` with
`entry_kind` `EXTERNAL_SUB` and `rid`, `gid`, `ordinal` copied from the arrival,
and a `dispatch_end` whose `retained_id` is the same `record_id`. Joining on
`record_id` pairs every arrival with its callback run. The binding schedules the
action first and emits `external_arrival` right after, so a slow trace sink cannot
delay the tag. Its trace timestamp can therefore come after the `dispatch_begin`
it caused. Order by `arrival_physical_ns`, not by the event timestamp. An arrival without a
dispatch was refused by LF (the program was stopping) or was still queued at the
end of the run.

A lifecycle node can destroy its external subscription (cleanup) after the ingress
thread took a message from it and before the callback ran. Stock ROS 2 destroys that
message with the subscription, so the binding runs no callback for it. Its
`dispatch_end` has `callback_ran` 0 and `outcome` `LOST`, and with
`RCLLF_TRACE_CALLBACKS=1` the JSON log carries an `external_lifecycle_drop` event
with the `rid` and `record_id`.

## Fields added in 1.2: Cutoff and missing hook

Schema 1.2 adds two fields to `dispatch_end`, the LTTng copies of
the `cutoff_elapsed` and `missing_hook` fields of the `trace_corr` JSON dispatch
record. `cutoff` is the absolute cutoff tag time in ns of the expectation the
dispatch joined, its publish tag plus `d_e` plus `T_e`. It is -1 when the
expectation has no cutoff: on a `local_now` edge (writer and reader in one
process, decided when the publish returns, so a missing sample is `LOST` at once
and `T_e` only sizes the lag bound), on an edge without a timeout, and for every
dispatch that is not a correspondence drain. A local edge that rebinds its
subscription or is deferred is not `local_now` and carries a cutoff when it has
a timeout. The trace manifest marks a `local_now` edge with
`"local_now": true, "cutoff": "none"`. Subtract `lf_start_time_ns` of
`lf_start` from a cutoff of 0 or more to get the JSON's elapsed value; -1 stays
-1 there (`lf_eval.semantic.schema.cutoff_elapsed` returns `None` for it).
`missing_hook` says which handler delivered a missing message: `NA` when the
dispatch delivered no missing message, `NONE` when the subscription registered
no deadline or message lost handler, `SHUTDOWN` when shutdown had begun, and
otherwise the first handler that ran, `DEADLINE` before `MESSAGE_LOST`. A schema
1.1 trace has neither field, so a reader treats them as absent.

## Fields added in 1.3: Process ancestry

Schema 1.3 adds the field `ancestry` to `run_begin`: the process's
parent PID chain, read from `/proc` when `run_begin` is emitted, space separated,
parent first, up to PID 1. LTTng user-space tracing has no parent PID context, so
this is how a trace reader links a publishing process to the launcher
of its run, even when an intermediate shell or wrapper is not traced and has
exited. A schema 1.2 trace has no `ancestry`, so a reader treats it as absent.

## Fields added in 1.4: Run token

Schema 1.4 adds the field `run_token` to `run_begin`: the value of
the environment variable `RCLLF_RUN_TOKEN`, read once per process, or the empty
string when it is unset. A benchmark runner can export a fresh random token for every launch
(recorded as `launch.run_token` in `run.json`) and `ros2 launch` passes it on to
every node process, so a process of the run is told apart from another program
in the same trace session by its token, not by its PID, which the kernel reuses.
The ancestry is then only a cross-check for a process without a token. A schema
1.2 or 1.3 trace has no `run_token`; a reader treats it as absent.

## Fields added in 1.5: Stamp-anchored arrivals

Schema 1.5 adds four fields to `external_arrival` and the code set
`rcllf_trace_ext_disposition`. A topic declared with `anchor: stamp` in
`execution.yaml` is inserted at an absolute tag derived from the message's
`header.stamp`; every message of it emits one `external_arrival` whose
`disposition` says what became of it. `stamp_ns` is `header.stamp` in
nanoseconds, `tag_ns` the tag the message was inserted at (or would have been,
for `OVERRUN`, `COLLISION` and `AFTER_STOP`) as nanoseconds after the start tag,
and `insert_ns` the physical time right after the insert for `ADMITTED` (0
otherwise). An arrival-anchored topic emits `ARRIVAL` with `stamp_ns` 0,
`tag_ns` -1 and `insert_ns` 0, and a `LATE` message, decided before any tag was
computed, carries `tag_ns` -1. A collision reports both records of the cell: the
one already queued gets a second `external_arrival` with `COLLISION` at the time
its event is removed, with its own `arrival_physical_ns` and `stamp_ns`. A
schema 1.1 to 1.4 trace has none of these fields; a reader treats every arrival
as `ARRIVAL`.

## Fields added in 1.2: Ingress callback starts

Schema 1.2 also adds the event `ingress_callback_start`. The ingress
drain emits it each time it hands an infrastructure service request, such as
`get_parameters` or `change_state`, to its rclcpp service. Its fields are those
of the `callback_start` JSON line that `RCLLF_TRACE_CALLBACKS=1` prints for the
same call, but the event needs no environment variable. `node` is the ingress key,
`callback` the service suffix and `ingress_record_id` the ingress record of the
request, the `record` of the `ingress` JSON lines. `rid` is -1 because the drain
is not a reaction of the user node.

## Fields added in 1.6: Actuation lateness

Schema 1.6 adds the event `actuation_late`. A topic with
`publishers.<node>.<topic>.output_deadline_ns` in `execution.yaml` is checked in the
publication gate immediately before the real publish; a publish whose
`lateness_ns = physical_ns - tag_time` exceeds `bound_ns` emits one `actuation_late`
and is published anyway. `rid` is the publishing reaction, `topic` the resolved topic,
`tag_time` and `tag_microstep` the reaction's tag (absolute), `physical_ns` the
`lf_time_physical()` reading of the check. A schema 1.1 to 1.5 trace has no such event.

## Fields added in 1.7: tf2 acquisition lookups

Schema 1.7 adds the events `tf2_request` and `tf2_outcome`, the LTTng
copies of the `RCLLF_TF2` `tf_request` and `tf_outcome` JSON lines that an
`rcllf_tf2::AcquisitionLookup` writes (see [tf2 Lookups](../features/tf2.md)).
`node` is the adapter's node name and `id` the request id, unique per adapter; since
schema 1.8, `(node, adapter, id)` pairs a request with its single outcome (below). `stamp_ns` is the requested
stamp; `tag_time` and `tag_microstep` are the absolute tag of the request
(`tf2_request`) or of the decision (`tf2_outcome`). `on_timeout` and `outcome` use the
`execution.yaml` spelling: `reject`, `failure`, `fallback:<name>`, and `ok` for an
outcome. `wait_logical_ns` is the decision tag minus the request tag,
`wait_physical_ns` the steady-clock time between them (traced, never claimed), and
`latest_common_ns` the Buffer's latest common time of the frame pair at the decision,
-1 if it has none. The transform itself is only in the JSON line.

The same minor adds the `external_arrival` disposition `TF_MIXED_STAMPS` = 7: a
stamp-anchored `/tf` or `/tf_static` message whose transforms do not all carry one
`header.stamp` is not admitted (it is never split or anchored at a guessed stamp). A
schema 1.1 to 1.6 trace has none of these.

## Fields added in 1.8: tf2 adapter and available interval

Schema 1.8 appends three fields to `tf2_request` and two to `tf2_outcome`,
and the JSON lines carry the same values (`adapter`, `oldest_common`, `latest_common`):

- `adapter` is the adapter's construction ordinal on its node (0 for the first
  `rcllf_tf2::AcquisitionLookup` the node makes, 1 for the second, ...). Request ids
  count per adapter, so `(node, adapter, id)` pairs a request with its outcome; in a
  1.7 trace two adapters of one node can share a `(node, id)`.
- `oldest_common_ns` and `latest_common_ns` are the Buffer's available interval for
  the frame pair when the request is made (`tf2_request`) or decided (`tf2_outcome`).
  `latest_common_ns` is tf2's latest common time. `oldest_common_ns` is the latest of
  the per-link oldest stamps along the path between the two frames, static links
  excluded. Both are 0 when every link on the path is static (tf2 answers for any
  stamp) and -1 when tf2 cannot connect the two frames. `oldest_common_ns >
  latest_common_ns` means the links hold stamps that do not overlap, so no stamp is
  transformable.
  One edge is not exact: a dynamic link whose oldest stamp is 0 ns reports 1 ns, since
  tf2 reads time 0 as "the latest transform", not as stamp 0.
- Both are -2 (JSON `"unavailable"`) when the path's interval is not one exact range:
  a link whose child had another parent within the samples the Buffer holds for it
  (tf2 allows reparenting; a child under A at 10 ms, under B at 20 ms and under A
  again at 100 ms is answerable from A at 10..20 ms and from 100 ms on), or a dynamic
  link whose only samples are stamped 0 (tf2 answers it at no positive stamp, while
  0 and 0 mean an all-static path). The adapter finds each link's range by bisection
  over tf2's public `_getParent(child, t)`: the child's first stored sample, then the
  oldest time from which its parent is the link's at every probe; the link is exact
  only when the two agree. tf2's public API does not list a link's samples, so a
  reparent episode that starts and ends between two probes is not seen.

tf2 has no public call that returns a link's oldest stamp in nanoseconds
(`BufferCore::getFrame` is private, and `allFramesAsYAML` prints `oldest_transform`
as text in microseconds). The adapter therefore finds each dynamic link's oldest stamp
by bisection over the public `canTransform(parent, child, t)`, which is true exactly
on that link's `[oldest, newest]`. The result is exact to the nanosecond. A 1.7 trace
has none of these fields; a reader treats them as absent.

## Codes

- `rcllf_trace_outcome`: `PENDING` = 0, `MATCHED` = 1, `NOT_READY` = 2, `TIMED_OUT` = 3, `LOST` = 4, `REJECTED` = 5, `UNBOUND` = 6, `UNSUPPORTED` = 7, `DUPLICATE` = 8, `OVERFLOW` = 9, `EPOCH_CLOSE` = 10, `PUSH_OVERFLOW` = 11, `SCHEDULE_REFUSED` = 12, `READER_REPLACED` = 13, `NONE` = 255
- `rcllf_trace_sample_outcome`: `RETAINED` = 0, `UNSUPPORTED` = 1, `UNRESOLVED` = 2, `LATE` = 3, `DUPLICATE` = 4, `REJECTED_IDENTITY` = 5, `OVERFLOW` = 6, `EPOCH_CLOSE` = 7, `STALE_READER` = 8, `POOLED` = 9
- `rcllf_trace_origin`: `ADMITTED` = 0, `NO_CONTEXT` = 1, `UNDECLARED_EFFECT` = 2, `EDGE_NOT_ADMITTED` = 3, `UNSUPPORTED_API` = 4, `NESTED` = 5
- `rcllf_trace_pub_result`: `ACCEPTED` = 0, `FAILED` = 1
- `rcllf_trace_api`: `PUBLISH` = 0, `SERIALIZED` = 1, `LOANED` = 2
- `rcllf_trace_entry_kind`: `TIMER` = 0, `LEGACY_SUB` = 1, `CORR_SUB` = 2, `SERVICE` = 3, `ACTION` = 4, `EXTERNAL_SUB` = 5
- `rcllf_trace_carrier_op`: `EMIT` = 0, `ARRIVE` = 1, `DRAIN` = 2, `REFUSED` = 3
- `rcllf_trace_cleanup_reason`: `SHUTDOWN` = 0, `EPOCH_CLOSE` = 1
- `rcllf_trace_missing_hook`: `NA` = 0, `NONE` = 1, `SHUTDOWN` = 2, `DEADLINE` = 3, `MESSAGE_LOST` = 4
- `rcllf_trace_ext_disposition` (schema 1.5): `ARRIVAL` = 0, `ADMITTED` = 1, `LATE` = 2, `OVERRUN` = 3, `COLLISION` = 4, `AFTER_STOP` = 5, `REFUSED` = 6, `TF_MIXED_STAMPS` = 7 (schema 1.7)

Outcome, sample outcome and origin codes are the ordinals of the correspondence enums in
`src/correspondence.hpp`. `NONE` (255) marks the `dispatch_end` of a timer or
legacy entry, which has no expectation. Wire codes are appended, never renumbered.

## Walking a chain

Keys, never timestamps, connect the events of one chain:
`dispatch_begin(dispatch_id)` produces `publication(gid, ordinal, dispatch_id)`,
which creates one `expectation` per admitted edge. Ingress takes the sample and
emits `sample(edge, gid, ordinal, retained_id)`. The subscriber dispatch emits
`dispatch_begin(edge, gid, ordinal)` and `dispatch_end(retained_id)` consuming it.
Every expectation ends in exactly one `terminal`.

## Status codes

Integrity, per epoch and channel: `no_run_manifest`, `discarded:<channel>`,
`discards_unknown:<channel>`, `crosscheck_discard`, `missing_producer:<executable>`,
`no_begin`, `no_end`, `seq_gap:<category>:<first missing seq>`, `seq_duplicate`,
`schema_version`, `manifest_mismatch`, `read_error`, `missing_close:publisher` and
`missing_close:edge` (a close-record count that differs from `run_end`),
`duplicate_publisher_gid` (two tracked publishers share a GID), and
`analyzer_error:<exception>` (the analysis of the run raised, so it has one claim of
kind `analysis`). Every epoch of a run feeds every claim of that run.

Coverage, per claim: `provider_absent`, `legacy:<reason>`, `clock_domain` and
`multi_epoch` (unsupported, since edges are reconciled inside one process), `provider_disabled:<event>`, `provider_filtered:<event>`,
`provider_silent:<category>`, `uninstrumented_api`, `path_bypass`,
`ros2_provider_missing` (uncovered).

Reconciliation, per admitted edge: `missing_terminal`, `duplicate_terminal`,
`terminal_without_expectation`, `orphan_dispatch`, `dispatch_without_end`,
`sample_key_mismatch`, `missing_publication`, `missing_expectation`,
`summary_mismatch`, `in_process`, `producer_orphan_dispatch` and
`producer_count_mismatch` (the runtime's own per-edge fault counters in `run_end_edge`).

## Correspondence JSON records

Besides the LTTng provider, the correspondence runtime prints one JSON line per
event on the `RCLLF_TRACE_CALLBACKS` channel, each with `"event":"corr"` and a
`kind`. The fields for [missing messages](../features/missing-messages.md):

| `kind` | Field | Meaning |
|---|---|---|
| `dispatch` | `missing_hook` | For a missing outcome, the first event callback that ran: `deadline`, `message_lost`, `none` (neither registered) or `shutdown` (shutdown had begun). `n/a` for every other outcome |
| `dispatch` | `missing_hooks` | List of every event callback that ran, in call order, for example `["deadline","message_lost"]`. Empty when none ran. Added after `missing_hook`, which keeps its meaning |
| `dispatch` | `cutoff_elapsed` | Admission cutoff of the expectation, elapsed ns, or -1 when it has none (a `local_now` edge, an edge without a timeout) |
| `missing_hook` | `edge`, `gid`, `ordinal` | The missing message |
| `missing_hook` | `total` | The logical count passed to every callback as `total_count` |
| `missing_hook` | `hooks` | Same list as `missing_hooks`. One record per missing message whose callbacks ran |
| `silence_deadline` | `edge` | The drain edge of the subscription whose [deadline window](../features/missing-messages.md#a-producer-that-falls-silent) ended with no message |
| `silence_deadline` | `window_start_elapsed`, `window_end_elapsed` | The window, elapsed ns; the record's tag is its end |
| `silence_deadline` | `period_ns`, `total` | The QoS deadline D, and the `total_count` passed to `deadline_callback` |
| `silence_deadline` | `hook` | `deadline` (the callback ran), `inactive` (node not active) or `shutdown` (shutdown had begun) |

## Test hooks

`RCLLF_TEST_TRACE_DROP_SEQ=<category>:<n>` takes seq `n` without emitting it,
`RCLLF_TEST_TRACE_SKIP_RUN_END=1` suppresses `run_end`, `RCLLF_TEST_TRACE_BURST=<n>`
emits `n` extra `provider` events after `run_begin` to overflow a small buffer.
They exist for the output checks only.

The traced output checks (`tools/colcon_lf/tests/runtime/test_runtime_semantic_trace.py`)
run only with `RCLLF_RUN_SEMANTIC_TRACE=1`, since they start their own `lttng-sessiond`.

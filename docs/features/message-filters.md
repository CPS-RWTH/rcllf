# message_filters Synchronizers

A node with a two-input `message_filters::Synchronizer` using the `ExactTime` or
`ApproximateTime` policy converts without changes. The real library runs inside the
node's LF reactions: each synchronizer input is an ordinary subscription with its own
reaction, and the policy and your callback run inside it. rcllf does not reimplement
the policy. It makes the order in which the policy sees its inputs a logical order, so
a converted run gives the same tuples every time.

This page describes what is claimed for these synchronizers (the domain `mf-sync/1`),
what generation refuses and why, and how to check the claim with the oracle.
The rcllf-specific `BoundedNearestToAnchor` policy has its own page,
[BNA Synchronizer](bna-synchronizer.md).

## What is claimed

On the domain below, the converted synchronizer emits the same signal and drop
stream as `message_filters` 4.11.17 fed with the same samples in the **LF arrival
order**: by tick (the tag a sample lands at), then input 0 before input 1, then
publish order. Generation declares input 0's drain before input 1's whatever order
the node creates its subscriptions in. A stock ROS 2 run delivers in wall-clock order, which depends on
executor scheduling and branch work. The claim is therefore "matches message_filters
4.11.17 for the LF arrival order", never "equivalent to a stock ROS run".

| Domain `mf-sync/1` | Requirement |
|---|---|
| Library | `message_filters` `4.11.17-*` and `rclcpp` `28.1.22-*` (Debian build suffix free) |
| Shape | one `Synchronizer`, two plain `message_filters::Subscriber<M>` inputs, one `registerCallback` (the form [RCLLF-H003](execution-config.md#rcllf-h003) admits) |
| Policy | `ExactTime` or `ApproximateTime` with library defaults: age penalty 0.1, no `setMaxIntervalDuration`, no `setInterMessageLowerBound` |
| Queue size | a literal or a parameter that resolves to an integer >= 1 |
| Inputs | each input an admitted correspondence edge (publisher inside the analyzed system, see the Publication correspondence row of the [Support Matrix](support-matrix.md)), both with one common link delay |
| Burst | at most the edge's declared publishes per tag; a sample past it is reported, not compared |
| Timestamps | `header.stamp` is data and may repeat, run backwards or collide across inputs |

Every delivered sample is classified once: signalled in a tuple, dropped by the
policy, overwritten by a newer sample with the same stamp, or pending at the end. The
classification is derived offline from the correspondence dispatch records.

## What generation refuses

The analyzer admits the synchronizer shape. Generation then checks the
inputs and refuses with [RCLLF-H003](execution-config.md#rcllf-h003) and a named
reason when an input would arrive in wall-clock order:

| Diagnostic | Cause | What to do |
|---|---|---|
| `sync_input_legacy:external` | the input's publisher is outside the analyzed system (for example a single node launched on its own) | convert a launch that contains the publishers too |
| `sync_input_legacy:<reason>` | the input is a legacy edge, for example `transient_local` QoS or a publisher the correspondence analysis cannot bound | fix the named reason on the edge (see limit [L6](support-matrix.md#l6)) |
| `sync_input_delay_unequal` | the input links have different `after` delays, between the two inputs or between two publishers into one input, so a pair published at one tag lands at two | give every input link one delay |

A lost sample (a dropped message on a local edge) or a burst above the declared bound
does not refuse generation. The oracle comparison reports it as `out_of_domain`
instead of comparing the stream.

Rate bounds, three or more inputs, `LatestTime`, `Cache`, `TimeSequencer` and chained
filters stay refused by the scan.

## How the claim is checked

`packages/src/sync_oracle_showcase` is the evidence package. Its
[README](https://github.com/CPS-RWTH/rcllf/tree/main/packages/src/sync_oracle_showcase)
lists every corpus and the rule each one separates.

- **Oracle O1** (`sync_oracle`) links the installed `message_filters` and feeds a
  corpus to the policy in LF arrival order, without a ROS graph. Each corpus carries
  an `expect:` block that O1 must equal: derived by hand from the 4.11.17 headers for
  the ExactTime corpora, and from an independent Python port of `approximate_time.h`
  for the ApproximateTime corpora.
- **The converted run** plays the same corpus through five synchronizer instances
  (ExactTime with queue 1, 2 and 3, ApproximateTime with queue 2 and 3). Each node
  logs its config, its signals and drops as `RCLLF_SYNC` lines.
- **The comparator** `python3 -m lf_eval.sync_oracle` checks each instance against O1
  in a fixed order and prints one line per instance: `compatible`, or the first
  difference.


| Report | Meaning |
|---|---|
| `version_unpinned:<package> <found>` | a stream ran against a library version outside the pin |
| `config_mismatch:<field> ...` | policy, queue, corpus digest or domain differ between O1 and the run, or O1 was not produced by the pinned header (`--impl ref` or a mutant) |
| `out_of_domain: ...` | an input has no admitted edge record or a legacy edge on its topic, the input delays differ, a sample was lost or a burst overflowed; nothing is compared. Only the instance's two inputs count, the admitted edges into the node on `/sync/in0` and `/sync/in1`: a legacy edge or another subscription elsewhere does not take it out of the domain. Each input must be the node's only subscription on its topic: a second subscription on `/sync/in0` or `/sync/in1` (two admitted edges from one writer, or another callback draining an input's writer) is `out_of_domain: second subscription on <topic>: ...`, and an input that drained none of its corpus rows is `out_of_domain` because nothing ties its edge to the synchronizer's subscription |
| `arrival_order_mismatch at dispatch <k>` | the run fed the policy in another order than the LF arrival order |
| `tuple_mismatch at seq <k>` | the k-th signal or drop differs |
| `classification_mismatch ...` | a sample ended in another class than under O1 |

Three checks make the comparison able to fail. Because O1 and the converted run
execute the same library code, these checks carry the claim:

- single-flag mutants of a reference ExactTime policy (first stamp wins, evict the
  oldest arrival, keep old entries, queue + 1) each differ from O1 on corpus F1 at the
  event derived by hand;
- generating with `RCLLF_TEST_SYNC_REVERSE_DRAINS=1` (input 1 drained first) gives
  `arrival_order_mismatch at dispatch 0` on corpus B8;
- dropping one sample on an input edge (`RCLLF_TEST_CORR_DROP_SAMPLE`) gives
  `out_of_domain: LOST on edge <id> ordinal 2 (row 2)` on corpus P1.

A fork/join case (P3) sends one source through two branches with 0 ms and 8 ms of work
per row into an ExactTime join. Both work assignments give the same join stream when
converted. Under stock ROS 2 the join's stream changes with the assignment.

### Running it

Inside the dev container, with the workspace built:

```bash
colcon lf --package sync_oracle_showcase
cd tools/colcon_lf && python -m pytest tests/runtime/test_runtime_sync_oracle.py -v
```

Set `RCLLF_SYNC_RUNS=20` for the 20-run determinism check of F1, B8 and P3.

# Publication Correspondence

In stock ROS 2 a subscription callback runs whenever a sample sits in the
reader. How often it runs, and which sample it sees, depend on DDS timing. On a
topic edge with publication correspondence, rcllf ties every publish to exactly
one callback run with exactly that sample, at a fixed
[tag](../design/glossary.md#tag). A topic edge is one publisher node and one
subscriber node on one topic.

Edges that get this treatment are called **admitted**. All other topic edges
are **legacy** edges: they keep the older behavior described under
[Limits](#limits). The concept and its place in rcllf's determinism claim are on
[Time & Coordination](../design/time-and-coordination.md#publication-correspondence-topic-edges).

## What it guarantees

For an admitted edge with delay d (default 0):

- The subscriber's callback runs once per publish. Zero publishes give zero
  runs. Three publishes in one reaction give three runs, at one tag, in
  publication order.
- Each run gets the sample that publish produced, matched by publisher and
  sequence number.
- A publish at tag t is delivered at tag t + d. With d = 0 on an edge inside
  one process, that is the publication tag itself. On an edge between two
  [federates](federation.md), and on an edge from a node to itself, it is the
  next microstep.
- Each run ends with a typed outcome: `MATCHED`, or a missing outcome such as
  `LOST` or `TIMED_OUT` (see [Missing Messages](missing-messages.md)). When the
  program stops, a reconcile summary checks that every expected publish ended
  exactly once.

## When it applies

`colcon lf-generate` decides per edge and writes the result to
`correspondence.json` in the generated package. An edge is admitted when the
generator can bound how many samples the publisher sends per callback, and the
publishing callback is a timer or subscription callback. The
[Correspondence bounds](execution-config.md#correspondence-bounds) section explains
the bounds and how to declare one the analyzer cannot prove.

An edge stays legacy, with its reason listed in `correspondence.json` and in
the startup log, when:

- the publish comes from something other than a timer or subscription
  callback, for example a service handler. A private, non-virtual member
  helper counts as the callbacks that call it. A helper with an unknown caller,
  or called from the constructor, stays legacy;
- the publishing callbacks sit in more than one callback group
  (`publisher_callback_split`);
- the code publishes a serialized message (`unsupported_api`);
- the bounds are unknown (`burst_unbounded`, `period_unknown`,
  `rate_unbounded`);
- the topic uses `transient_local` QoS;
- a node has two subscriptions on the topic with the same callback
  (`ambiguous_endpoint`);
- under decentralized coordination, a cross-federate edge has `after 1 nsec`
  (its sender needs at least 2 ns), or the edge feeds a BoundedNearestToAnchor
  synchronizer over a federation link;
- the edge belongs to a group of several publishers that cannot be admitted
  as a whole (see [Several publishers into one subscription](#several-publishers-into-one-subscription)).

Services and actions do not use correspondence.

Set `require_admitted: true` in `execution.yaml` to turn any legacy topic edge
into a generation error.

At startup the program refuses an admitted edge (exit 1, before the first tag)
when the ROS middleware is not `rmw_fastrtps_cpp`, the publication mode is not
synchronous, a Fast DDS XML profile is configured, intra-process communication is
on, an endpoint is not VOLATILE, or the publication gate is not linked. An
admitted edge never falls back to the legacy path.

## How you see it

The example uses `pubsub_example`. Inside the dev container (see
[First LF Run](../getting-started/first-lf-run.md)):

```bash
colcon lf --package pubsub_example
```

The generated package holds `correspondence.json`, one entry per topic edge:

```bash
cat lf-generated/lf_generated_pubsub_example_bringup/correspondence.json
```

```json
[{"id": 0, "topic": "/sensor_data", "pub": "PublisherNode", "sub": "SubscriberNode", "delay_ns": 0, "status": "admitted", "reason": null}]
```

In the generated LF program (`lf/Bringup.lf`) an admitted edge is a port of type
`rcllf_corr_t`. The publishing reaction sets it only when the callback actually
published, with the number of publishes. The connection in the main reactor
has no `after`:

```lf
reaction(t_timer_callback) rcllf_bound -> corr_e0 {=
    ros2_lf_invoke_timer(self, RID_PublisherNode_timer_callback);
    { uint32_t n = ros2_lf_corr_emit(self, 0); if (n > 0) { rcllf_corr_t c = {0, n}; lf_set(corr_e0, c); } }
=}
...
publisher_node.corr_e0 -> subscriber_node.corr_e0
```

On the subscriber side a drain reaction takes these records and runs the
callback once per record at t + d.

Startup logs each admitted edge with its sizes:

```bash
ros2 lf launch pubsub_example bringup
```

```text
[ros2_lf_binding.corr]: rcllf corr: edge 0 is local, decided when its publish returns, no cutoff
[ros2_lf_binding.corr]: rcllf corr: admitted edge 0 PublisherNode -> SubscriberNode on '/sensor_data' (d = 0 ns, T = 100000000 ns, L = 200000000 ns, B = 1, retain = 2, carrier = 1, reader depth 10, user 10, reader reliable, join blocking)
[ros2_lf_binding.corr]: rcllf corr: run_epoch 910612842231703590, 1 admitted edge(s), 0 legacy edge(s)
```

Here d is the delay, T the timeout, L the worker lag bound and B the burst
bound, all explained under
[Correspondence bounds](execution-config.md#correspondence-bounds).

When you stop the program with Ctrl+C, the reconcile summary reports each
edge (trimmed):

```text
[ros2_lf_binding.corr]: rcllf corr: {"event":"reconcile",...,"edges":[{"edge":0,"delay_ns":0,"timeout_ns":100000000,"expectations":15,"outcomes":{"MATCHED":15},...,"missing_total":0,...}],...,"missing_total":0,"reconciled":true}
```

15 publishes, 15 matched runs, nothing missing. `"reconciled":true` means every
expectation ended exactly once.

With [`RCLLF_TRACE_CALLBACKS=1`](../reference/environment-variables.md#rcllf_trace_callbacks)
each step is also a JSON line (`emit` from the publisher, `sample` when the
sample is taken, `dispatch` when the callback runs):

```bash
RCLLF_TRACE_CALLBACKS=1 ros2 lf launch pubsub_example bringup
```

```text
[ros2_lf_binding.trace]: {"event":"corr","kind":"emit","edge":0,"count":1,"elapsed":0,"microstep":0,"tokens":[{"gid":"010f7fc4330622140000000000003e03","ordinal":1}]}
[ros2_lf_binding.trace]: {"event":"corr","kind":"dispatch","edge":0,"gid":"010f7fc4330622140000000000003e03","ordinal":1,"outcome":"MATCHED","elapsed":0,"microstep":0,...}
```

The fields are listed in
[Trace Schema: Correspondence JSON records](../reference/trace-schema.md#correspondence-json-records).

## Reader QoS

The binding changes two QoS settings of an admitted edge's reader:

- **Depth.** A KEEP_LAST reader of depth 1 holds only the newest sample, so a
  second publish at one tag could replace the first before it is taken. The
  binding raises every admitted reader to the edge's KEEP_LAST depth when the
  user asked for less. That depth is 8, or the edge's burst when it is larger
  (the `fusion_example` ApproximateTime edge gets 9), or the `reader_depth` set
  in `execution.yaml`. Startup logs the depth (`reader depth 8, user 1`). Legacy
  edges, KEEP_ALL readers and readers already at that depth or more keep the
  user's QoS. An admitted reader that still holds fewer samples than that, for example one
  created on a node built outside the generated factories, refuses startup with
  a log line that names the node, topic and depth.
- **Reliability.** The reader of a cross-federate admitted edge becomes
  BEST_EFFORT. A loss there is already detected by the record that travels over
  the federation link, and a RELIABLE reader would hold back every later sample
  behind a lost one, which stalls the federation. Startup logs it
  (`reader best effort, user reliable`). A same-federate edge keeps the
  reliability the user asked for. A cross-federate reader the binding could not
  relax refuses startup.
  [`RCLLF_ADMITTED_READER_RELIABLE=1`](../reference/environment-variables.md#rcllf_admitted_reader_reliable)
  keeps the user's reliability, for comparison runs only.

## Lifecycle nodes

Edges of lifecycle nodes are admitted too. The binding binds every publisher
the node creates on the edge's topic, also one made in `on_configure` or made
again after a cleanup. A later publisher that is not VOLATILE ends the run with
exit 1.

A subscription that a lifecycle hook creates again is bound when it is created
and must pass the startup checks again. Records still open for the old reader
end `READER_REPLACED`. Two rules keep this unambiguous:

- The new reader is admitted only once the old one is released. The usual
  `sub_ = create_subscription(...)` in `on_configure` does this, and the new
  reader takes over at the next tag.
- If rcllf cannot tell which subscription the new reader replaces (the old one
  still lives at a later tag, two new readers wait at once, or the node holds
  more subscriptions on the topic than the analysis found), it refuses the new
  reader with `rebind_identity` and the reconcile fails. A reader created
  outside an LF reaction after start is refused with `rebind_untagged`.

There is a short loss window: DDS matches a new reader to the writer some time
after it is created, and a publish in between reaches only the old reader. Such
records end `LOST` or `TIMED_OUT`; when the old reader took a copy they are
also counted as `lost_discovery_gap`.

## Several publishers into one subscription

Several admitted edges into one subscription callback form a fan-in group. The
publishers can be two instances of one class or different nodes. They share
one reader and one delivery order, sorted by:

1. the release tag: publication tag plus d;
2. on an equal release tag, the publisher's rank, the position of its LF
   instance name in alphabetical order. Generation fixes the ranks for the
   whole system and writes them as `fan_in_rank` in `correspondence.json`, so
   every federate uses the same order;
3. on an equal publisher, publication order.

Two publishers whose timers fire at the same tags are delivered alternately,
`a0 b0 a1 b1 ...`, in every run. Stock ROS 2 delivers them in arrival order. A
missing sample keeps its place in the order, see
[Missing Messages](missing-messages.md#several-publishers-into-one-subscription).
A delivery out of this order is counted as `order_violations` and fails the run
with exit code 3.

A group is admitted whole or not at all. If one member is refused, every member
stays legacy with `fan_in_member_refused`. A group also stays legacy when the
members' delays differ (`fan_in_release_mismatch`), some members cross
federates and others do not (`fan_in_mixed_locality`), a member is the
subscribing node itself (`fan_in_self_loop`), a member rebinds as a lifecycle
node (`fan_in_rebind`), a member node publishes the topic from two publisher
objects (`fan_in_shared_publisher`),
or the subscription feeds a BoundedNearestToAnchor synchronizer
(`fan_in_bna_input`, which stops generation with
[RCLLF-H003](../reference/diagnostic-codes.md#rcllf-h003)). The reader depth must
hold the members' bursts together (`reader_depth_below_burst`). A writer outside
the system, for example `ros2 topic pub`, is `UNRESOLVED`: its samples are
counted and never delivered.

## Limits

- On a legacy edge the LF connection carries only a trigger. The callback
  takes whatever sample is in the reader when it runs, so the value it sees
  depends on wall-clock arrival. See
  [Take policy at the boundary](../design/time-and-coordination.md#take-policy-at-the-boundary).
- Only topics are covered. Services and actions keep their own connections.
- Only `rmw_fastrtps_cpp` is supported for admitted edges.
- A message from outside the system (a bag, `ros2 topic pub`, a sensor driver)
  is not a publish rcllf can count. Declare such topics as
  [external topics](execution-config.md#external_topics).
- The [Support Matrix](support-matrix.md) lists the evaluated scope.

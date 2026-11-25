# Time & Coordination Model

This chapter is the single definition of what rcllf means by *logical time*, *tag*,
*after delay*, *federation*, and *determinism*, and, just as importantly, which of
those words apply to which way of running a converted system.

Lingua Franca supports more than one coordination scheme, and those schemes have
**disjoint vocabularies**. rcllf implements a subset of them. Every page in these
docs that uses one of those words is talking about exactly one regime. This chapter
says which.

Read this before [Execution Model](architecture.md),
[LF Code Generation](lf-generation.md), and [Federation](../features/federation.md),
all three use the vocabulary defined here.

---

## The three coordination regimes

A *coordination regime* is the mechanism by which logical time advances consistently
across all the reactors in a program. LF defines three, and rcllf exposes all three:
the first is the default, the second is what `--federate` generates, and the third
(decentralized) is experimental and opt-in with `--coordination decentralized`.

### Regime 1 (Unfederated)

!!! success "Validated"
    The full pipeline (extraction, generation, `lfc` compilation, execution) is
    exercised on this regime. Inside each OS process `ros2_lf_binding` assumes a
    single LF environment (`envs[0]`); a federated program runs one such process per
    federate. [ADR 0001](adr/0001-lf-owns-time.md) applies here.

One OS process, one LF environment, one reactor-c scheduler. All reactors share an
in-process event queue. A tag is "safe to process" as soon as the scheduler holds no
earlier event. There is no coordinator, no network, and nothing to be late for.

**Vocabulary that belongs to this regime:** logical time, tag, microstep, `after`
delay, reaction declaration order, logical action.

### Regime 2 (Federated, centralized)

!!! note "Supported for `--federate per-node`; other partitions experimental"
    The default coordination of `colcon lf-generate --federate`. One process per node
    (`--federate per-node`) under this regime is a supported deployment of this
    release, and CI builds and runs federated systems on every change. Per-process
    partitions, a custom federation YAML and federation across hosts are experimental:
    smoke runs show that they run and that the expected messages flow, without the
    evaluation evidence Regime 1 has.

The program is split into federates (one OS process each), and a **Run-Time
Infrastructure (RTI)** daemon mediates every cross-federate tag advance. A federate
blocks at a tag boundary until the RTI grants it permission to proceed. Because the
RTI holds back every advance until it is provably safe, **messages are never tardy in
this regime**. There is no such thing as a late message, only a federate that is
waiting.

The price is that the RTI is the single point of coordination and the coordination
bottleneck: it is on the critical path of every cross-federate tag advance, and the
federation is all-or-nothing (if any federate stalls, the rest stall behind it). See
[Federation](../features/federation.md) for the operational consequences.

**Vocabulary that belongs to this regime:** RTI, federate, federation, message
routing through the RTI, "safe to advance", tag advance grant, all-or-nothing
progress.

### Regime 3 (Federated, decentralized)

!!! warning "Experimental"
    `colcon lf-generate --federate ... --coordination decentralized` emits LF's
    `coordination: decentralized` target property and a per-cross-federate-input
    STP (safe-to-process) offset with a logging tardy handler. Real
    (not hand-authored) example: `pubsub_example`, 2 federates, see
    [Federation: Coordination mode](../features/federation.md#coordination-mode).
    "Experimental" means: it builds, runs, and its tardiness actually shows up
    in a real container run. It does **not** mean the fault/availability story
    below (the CAL tradeoff this regime exists for) is implemented. A stalled
    federate still stalls the whole federation, same as centralized.

No RTI. Federates exchange messages peer-to-peer and each one decides locally when a
tag is safe to process, using a **safe-to-process (STP) offset**: a per-federate
allowance for how far physical time must lead a logical tag before that tag may be
processed. Because the decision is local and the network is not omniscient, a message
can arrive after its tag has already been processed (it is **tardy**) and the
program must supply a **tardy handler** to say what to do about it.

That is exactly the point: by accepting the possibility of tardiness, a decentralized
federation can keep making progress when a peer is slow or unreachable. This is the
**consistency-availability-latency (CAL) tradeoff**, and it exists **only** in this
regime. A centralized federation trades availability for consistency by construction.
an unfederated program has no tradeoff to make.

**Vocabulary that belongs to this regime, and nowhere else:** STP offset, tardiness,
tardy message, tardy handler, CAL tradeoff, availability under faults.

### Regime property summary

| Regime | Message routing | Tardiness | Fault / availability story | Status in rcllf |
|---|---|---|---|---|
| Unfederated (single process) | in-process event queue | n/a | n/a | **Validated** |
| Federated, centralized | via RTI | **never tardy** | RTI is coordination bottleneck. All-or-nothing | **Experimental** |
| Federated, decentralized | peer-to-peer | **possible** (STP offsets) | CAL tradeoff space. Can stay available under faults | **Experimental** (STP/tardiness only. Fault story not implemented) |

### Mixing the vocabularies is a documentation bug

The three vocabulary lists above are exclusive. Concretely:

- Text about RTI-mediated coordination must never mention tardiness or STP offsets.
  Under an RTI there are no tardy messages.
- Text about tardiness, STP, tardy handlers, or the CAL tradeoff must be scoped to
  decentralized federation specifically (`--coordination decentralized`), and never presented as a property of rcllf's federation in general, since
  centralized is still the default and the only mode without an explicit flag.
- Text about "LF federation" without a centralized/decentralized qualifier is
  incomplete. It silently asserts that RTI-mediated coordination is the only kind LF
  has, which is false at the language level even though it is true of everything this
  tool generates today.

If you find a page in these docs that breaks one of those rules, that is a bug worth
reporting. Please
[open an issue](https://github.com/CPS-RWTH/rcllf/issues/new). The
distinction is easy to blur and hard to notice, so external eyes are genuinely useful
here.

---

## Logical time, tags, and the LET connection

### Logical time and tags

**Logical time** is a discrete clock shared by every reactor in an LF program. It is
not a measurement of anything: it advances only when the scheduler decides to advance
it, from one event to the next, and it never advances *during* a reaction.

A **tag** is a pair `(logical time, microstep)` that totally orders every event in the
program. Two events at the same logical time but different microsteps are still
strictly ordered. Reactions triggered at the same tag are ordered by their declaration
order within a reactor and by the dependency graph across reactors. This tagged-event
model, and the determinism argument built on it, is the reactor model of Lohstroh
et al. [1].

Reactor semantics are *sparse* rather than dense-tick: tags exist where events exist,
not at every instant of a global clock [8]. That is why an LF program with a 10 ms
timer does no work at the 4999 tags a dense-tick model would have between two firings.

### What LET is

**Logical Execution Time (LET)** originates with Giotto [2]. A LET task has a declared
interval. It **reads its inputs at the release instant** of that interval and its
**outputs become visible at the deadline** of that interval, regardless of when
within the interval the code actually ran, or how long it took. The task is treated as
taking zero time and then jumping to its deadline. Kirsch and Sokolova [3] give the
mature statement of the paradigm: LET buys *time determinism* (when values become
visible is fixed by the schedule, not by execution time) and *data-flow determinism*
(which value a reader observes is fixed by the same schedule).

The reason LET is worth naming rather than reinventing is that it is not a niche idea:
it is the standard answer to scheduling jitter in automotive multicore software [4] and
is standardized in AUTOSAR Classic's Timing Extensions.

### Why `after` delays are a LET-style contract, not a latency measurement

This is the load-bearing paragraph of this chapter.

rcllf's generator emits every inter-node topic connection with an `after`
delay (the examples in this chapter use 5 msec, while the default is 0, one
microstep,
set via `--after-delay` or `execution.yaml`, see
[Execution Configuration](../features/execution-config.md#the-after-delay-default)):

```lf
publisher.sensor_data -> listener.sensor_data after 5 msec
```

That delay is a **declared logical-time contract**, in exactly the LET sense. The
value published by the upstream reaction at tag `T` becomes observable to the
downstream reaction at tag `T + 5 msec`, not "about 5 ms later", not "as soon as DDS
delivers it", but *at that tag*, on every run, regardless of how long the transport
actually took. Write-at-deadline, read-at-release, with the connection playing the
role of the LET interval.

It follows that:

- **The `after` value is not a measurement.** A measured DDS latency distribution may
  *inform your choice* of the value. You want the contract to be satisfiable, so the
  declared delay should exceed the transport's worst case (see
  [Execution Configuration](../features/execution-config.md)). But the value is the contract, and
  the program's semantics are defined by the declared value alone. Describing `after`
  as "modelling DDS transport latency" gets the causality backwards: latency data
  constrains a design choice. It does not define the semantics.
- **If the contract is not satisfiable, that is an observable failure, not a silent
  slowdown.** If the payload has not arrived by the tag at which the reaction fires,
  the ROS→LF binding's take policy decides what happens (skip, or bounded block). See
  [Take policy at the boundary](#take-policy-at-the-boundary) below.

Lee and Lohstroh [5] give the precise account of how LF's logical-time model
*generalizes* LET, and it is the citation to reach for on any LET-adjacent claim about
LF. The generalization matters because LF is not LET:

> In LET, logical time strictly matches physical time at each task invocation and
> termination. In the reactor model, reactions are logically instantaneous and thus
> compose without requiring explicit alignment. [6]

So in LF, a reaction has *no* LET interval of its own. It is logically
instantaneous, and reactions therefore compose without the designer aligning task
periods and offsets by hand. The LET-style interval appears only where you *declare*
it, as an `after` delay on a connection. `after` is an **opt-in, declared logical
delay**, not an implicit task interval imposed on every component.

### Distributed LET versus federated tags

The LET lineage has its own answer to distribution: **SL-LET** (system-level LET)
extends LET across a distributed system with explicit time zones and interconnect
tasks that carry values between them [7]. LF federation answers the same question
differently, one global tag order, enforced by coordination (an RTI, in
[Regime 2](#regime-2-federated-centralized)) rather than by aligning per-zone LET
schedules. They are alternative solutions to the same problem, not layers of one
solution. If you know SL-LET, expect tags where you would expect time zones.

---

## `Clock::now()` after conversion

Post-conversion, **LF is the sole time authority**. This is
[ADR 0001](adr/0001-lf-owns-time.md). Read it for the decision, the rejected `/clock`
alternative, and the observability amendment. The consequences you need in order to
read code:

- The `RCL_ROS_TIME` clock of every node the binding constructs is set to the
  current tag before each reaction, so `Node::now()` returns **current LF
  logical time**. An `rclcpp::Clock` the node creates itself is not covered.
- Logical time advances **only at tag boundaries**. It does not advance while a
  reaction is running.
- `std::chrono::steady_clock` and `RCL_STEADY_TIME` are **not** intercepted and still
  return wall time. Use them for diagnostics (latency measurement, log timestamps),
  never for scheduling decisions.

Practically, inside a converted callback:

| You write | You get |
|---|---|
| `now()` twice in one callback | The **same** value both times. No elapsed time is observable within a reaction. |
| `now()` in two reactions at the same tag | The same value in both. |
| `(now() - t_start)` to measure your own runtime | Always zero. Use `steady_clock` for that. |
| A sleep or wait on ROS time inside a reaction | Conceptually a deadlock: logical time cannot advance until the reaction returns, so the condition you are waiting for can never become true. |
| A timeout computed from `steady_clock` that creates a timer at runtime | The timer is not registered with reactor-c. All timers must be created at node construction time. |

The concrete symptoms and fixes for these live in
[Troubleshooting](../reference/troubleshooting.md#timer-fires-but-nodenow-returns-wall-time)
and the
[`steady_clock` note](../reference/troubleshooting.md#steady_clocknow-returns-wall-time).

---

## Three kinds of determinism

"Deterministic" is used loosely in the ROS 2 world. rcllf promises two of the
following three things, and the third is frequently assumed and never promised.

| | What it means | Promised? |
|---|---|---|
| **Firing-order determinism** | Given a set of reactions triggered at the same tag, the order in which they execute is fixed by the program text (declaration order within a reactor, dependency order across reactors), never by thread scheduling or DDS arrival order. Reactions of different reactors with no dependency path between them are not ordered: they may run at the same time on different worker threads, and no connection can observe their order (`workers: 1` runs them one at a time). | **Yes**, in Regimes 1 and 2. |
| **Data determinism** | The value a reaction observes is a function of the input timeline: given the same inputs arriving at the same *logical* tags, every reaction sees the same values on every run. | **Yes**, in Regimes 1 and 2, for two runs with the same modeled input timeline in which every modeled message arrives within its timeout T (see [the determinism claim](#the-determinism-claim) for what is modeled). Arrival-anchored external topics, topic connections outside the [publication correspondence](#publication-correspondence-topic-edges) criteria and other unmodeled inputs are not covered. |
| **Timing determinism** | The wall-clock instant at which a reaction runs is repeatable. | **No.** Only *logical* timing is deterministic. Wall-clock execution instants depend on OS scheduling, CPU contention, and DDS delivery, exactly as in stock ROS 2. |

Two consequences worth stating plainly:

- LF does not make your system faster or its wall-clock latency repeatable. It makes
  the *program's meaning* independent of wall-clock accidents. Wall-clock behaviour is
  still an engineering problem, and the real-time ROS 2 literature is the right tool
  for it, see [below](#relation-to-real-time-ros-2-work).
- Where a mechanism is only *partly* under LF's control, the docs must say so
  explicitly. The house pattern is
  [action_example](../examples/action-example.md)'s scope. Action feedback and
  result run at repeatable tags only when both ends are inside the LF system and the
  server sends from callbacks that LF reactions run. A server whose only clients are
  outside the system takes their goals through a physical action, so its tags follow
  arrival. Any determinism claim in these docs should be scoped that tightly.
  Unscoped claims are bugs, same as mixed regime vocabulary.

### The determinism claim

**Two runs of the same generated program that receive the same modeled input
timeline, and in which every modeled message arrives within its timeout T, give
identical outputs.** The modeled input timeline consists of:

- the timers, whose tags follow from the program;
- the samples on topic connections that meet the
  [publication correspondence](#publication-correspondence-topic-edges)
  criteria (admitted edges), each tied to the publish that produced it;
- the messages of stamp-anchored external topics (`anchor: stamp` under
  [`external_topics`](../features/execution-config.md#anchoring-a-topic-at-headerstamp)),
  each with its payload and its `header.stamp` relative to the run's start, which
  sets its tag. "Same timeline" means the same messages with the same relative
  stamps; when they arrive does not matter as long as each arrives within its T.

Not modeled, so outside the claim even when nothing is late:

- arrival-anchored external topics (a plain `external_topics` entry): the tag is the
  physical time the message was taken, so two runs with the same messages see
  different tags;
- topic connections outside the correspondence criteria, which take whatever sample
  the DDS reader holds when the callback fires;
- other inputs from outside the LF system: physical actions, clients and servers
  outside it, lifecycle transitions made outside LF;
- clocks the application reads itself (`steady_clock`, a clock the node creates).

The [supported scope](../features/support-matrix.md) is narrower still.

This is the claim rcllf is held to ("every run in which every message arrives within
its timeout T gives identical outputs", with the conditions above written into it), and every guarantee, doc sentence and test
assertion is written against it. A modeled message later than T counts as lost, and
the run must report it, never show a silent difference; a stamp-anchored message
refused as `late`, `overrun`, `collision` or `after_stop` is such a report. The claim
is not deterministic replay: rcllf records no outcome log to rerun. rcllf does not yet
meet the claim everywhere. The numbered list below names every known place where it
does not.

What T is, and how a loss is reported, per edge kind.

- An admitted edge inside one federate: no T and no cutoff. The
  publishing thread takes the sample from the subscriber's in-process reader before
  its publish returns and retains it, so the drain at t + d finds the edge decided. A
  sample the publish did not find ends `LOST`, counted `local_undelivered`, and the
  reconcile fails. Nothing is late on such an edge.
- An admitted cross-federate edge: its generated join timeout, cutoff = t + max(0, d) +
  maxwait_ms + eps, with t the publication's logical tag (the formula of
  [Missing messages](../features/missing-messages.md)), judged by the sample's ingress stamp. A missing sample ends `TIMED_OUT`,
  `NOT_READY`, `LOST` or `OVERFLOW` with its identity and is counted in the reconcile
  ([Missing messages](../features/missing-messages.md)). A join that the stop
  deadline wakes while the table is still open ends `EPOCH_CLOSE` and is counted as
  `pending_at_stop`. A join that the table's closing wakes ends
  `EPOCH_CLOSE` and is counted as missing.
- A legacy trigger with a known write count: `maxwait_ms`, counted from the
  moment the take runs, not from the tag. A miss is warned once per topic and counted
  in `take_stats` (`skipped`), a late sample that is dropped in `late_dropped`, and a
  reader overflow in `reader_lost`, which also goes to the subscription's message_lost
  handler; the takes of the replaced samples end at once, without a poll
  to the take timeout each.

Known places where a run without a loss report can still differ, where a late message
is not reported, or where a run reports a loss although no message was late on the
wire.

1. A legacy trigger whose write count is unknown, in `skip` mode, does not wait. A
   sample still on its way is not observed at that tag and is read at a later trigger.
   It is counted only in `take_stats` `skipped`, with no warning (`take_mode.cpp`, the
   `!known` return in `take_skip`). Within one program Fast DDS delivers in the publish
   call, so only a link between federates is exposed.
2. A legacy take measures its timeout from when it runs. A worker that runs its take
   late accepts a sample that arrived later than tag + T without any report. With a
   known write count the take still gets the counted sample, so the value does not
   change, but the lateness is not reported.
3. Reader overflow before a writer's first take is not seen. The first sample taken
   from a writer only sets the sequence baseline (`note_sequence`), and DDS raises no
   event for a KeepLast replacement, so a sample replaced before that first take is
   lost silently.
4. A subscription on an arrival-anchored external topic dispatches on arrival, so its
   tags follow wall-clock arrival. Such inputs are outside the claim; a stamp-anchored
   external topic is inside it.
5. After a reported miss on a legacy link, a lone late sample may be delivered one tag
   late, and with several writers the dropped sample may be another
   writer's (probe P17f). The run carries the loss report, but the difference can
   outlast the lost message.
6. Application code that waits for discovery before it starts (the brake_fusion
   simulator holds its first step until its subscribers are there) starts at a tag that
   follows wall-clock discovery. Two runs without any lost message can then differ, and
   only the node's own start event shows it (brake_fusion logs a `start` event with the
   number of held ticks).
7. On a cross-federate edge a worker that lags its tag by more than d + T joins after
   the cutoff, so a sample that arrived in time ends missing. This is reported, so not silent, but the run
   reports a loss that the wire did not have. `max_worker_lag_ms` sizes
   for this.
8. Decentralized coordination (STP offsets) and physical actions were not audited
   against this claim.

### Take policy at the boundary

The ROS→LF boundary is where a declared contract meets an actual transport. A
subscription reaction fires at its tag. The DDS payload may not have arrived. The
binding's take policy (`ROS2_LF_TAKE_MODE`) decides:

- An **admitted** edge (publication correspondence) does not take on trigger. Its
  drain joins the record of each publication with its sample. Across federates it
  waits up to the edge's generated timeout T, so a sample missing at the cutoff
  (publication tag + d + T) ends missing with its identity. Inside one federate the
  sample was taken when the publish returned, so the drain never waits.
- A **legacy** trigger with a known write count polls for its samples up to
  `maxwait_ms` (default 100 ms) before it declares the miss, in `skip` and
  `block` mode alike (`take_mode.cpp`).
- A legacy trigger whose write count is unknown (`RCLLF_COUNT_PUBLISHES=0`, a publisher
  outside any reaction) does not wait in `skip` mode (the default). `block` polls up
  to the timeout.
- A subscription on a declared external topic dispatches on arrival and never takes.

This is a property of the **binding boundary**, orthogonal to the coordination regime:
it is the same question in Regime 1 and Regime 2, and it is not tardiness (there is no
tardy handler, no STP offset, and the message is not delivered late at a later tag,
it is simply not observed). A take that gives up is a lost message under
[the determinism claim](#the-determinism-claim) and is reported. A run without such a
report must give the same outputs as every other one. A
system that regularly misses takes has a configuration problem, not a semantics
problem. Check the take timeout and the transport (reader depth, reliability, host
load) first. A larger `after` gives a legacy take more physical-time slack in real-time
execution, but it also moves every callback tag on that link.

!!! info "Take mode is now configurable, not just an env var"
    The description above is the semantics. As of the execution-configuration
    feature, `skip` vs `block` (and the block timeout) can be set **per node or per
    subscription**, not only globally via `ROS2_LF_TAKE_MODE`. The env vars still work
    as quick-experiment overrides. None of this changes the semantics on this page. Policy selects *which* of skip/block applies where and it never touches tag
    semantics. See [Execution Configuration](../features/execution-config.md) for the
    config file, precedence rules, and the block-timing footgun that comes with
    per-subscription blocking.

---

## Cross-tag coupling: why the honest claim is input-timeline determinism

Determinism *at* each tag does not make a system insensitive to input *timing*. This
is the most common misreading of what LF buys you, so it is worth being explicit.

State survives tags. Two mechanisms carry it:

- **Reactor and node state.** A converted ROS 2 node keeps its member variables. A
  reaction at tag `T` writes them. A reaction at tag `T'` reads them. Nothing about
  tag ordering resets that.
- **Buffering at the boundary.** The DDS queue in front of a subscription, and the
  take policy that drains it, mean the mapping from "message arrived" to "message
  observed at tag `T`" depends on physical arrival time.

Put those together: a sensor message that arrives 200 µs later lands on the *next*
tag instead of this one. Every tag is still processed deterministically, and the
firing order at each tag is still fixed, but the *sequence of states* the program
walks through differs, and so may its outputs. A depth-1 subscription that would have
been read at tag `T` may be overwritten before tag `T+1`.

So the honest claim is:

> **Given the same input timeline (the same values arriving at the same logical tags),
> a converted system produces the same outputs, in the same order, on every run.**

Not: "the system produces the same outputs regardless of when inputs arrive." That
would be false, and no coordination regime provides it. What LF removes is the *other*
source of variation: two runs with an identical input timeline no longer diverge
because a thread got scheduled differently. In stock ROS 2 both sources are live at
once and are indistinguishable in a trace. Under LF only the input timeline remains,
which is what makes a failure reproducible.

---

## Publication correspondence (topic edges)

A topic edge, in one process or across federates, that passes generation-stage admission
dispatches the subscriber once per actual publish, with the exact sample that publish
produced:

- The generated executable contains a publication gate: it defines `rcl_publish` and
  its siblings, counts every publish per publisher from process start and records
  which LF reaction (if any) published.
- The publishing reaction sets the edge's output port only when its callback actually
  published, carrying the count of records. The connection has no `after`.
- The subscriber moves the records into a carrier and drains them at t+d, where d is
  the edge's after-delay (default `0`). Every record due at a tag is
  delivered at that tag, in one firing of the drain reaction, with one callback
  invocation per record. Zero publishes give zero dispatches. Three
  publishes give three dispatches, all at (t+d, 0) for d > 0. For d = 0 and a
  publication at (t, m) all three run at (t, m) on an edge inside one process, whose
  drain reaction is triggered by the record port itself, as an LF connection without
  `after` delivers. A self edge and a cross-federate edge, whose records come over
  `after 0`, deliver all three at (t, m+1).
- Each dispatch joins its record with the DDS sample of the same publisher GID and
  sequence number, retained by the ingress thread. The outcome is typed (`MATCHED`,
  `LOST`, `TIMED_OUT`, ...), traced on the `RCLLF_TRACE_CALLBACKS` channel, and a
  reconcile summary at shutdown checks that every expectation ended exactly once
  (`reconciled`); a missing one is also warned when it is decided, rate-limited per
  edge, and counted in the summary's `missing_total` beside it.

The ingress thread takes samples from the DDS reader, and a reader with KEEP_LAST
depth 1 holds only the newest one. When a writer publishes twice at one tag and the
ingress thread is preempted between the two, the second sample replaces the first in
the reader and the first dispatch ends `LOST`. The binding therefore
raises every admitted edge's reader to KEEP_LAST depth 8 at creation when the user
asked for less, matching the carrier capacity. Startup logs the actual depth per
admitted edge (`reader depth 8, user 1`), and `get_actual_qos()` on that subscription
reports 8. Legacy edges, KEEP_ALL readers and readers already at depth 8 or more keep
the user's QoS. A node with two subscriptions on one topic (twins) has each admitted
edge matched by creation order, so a legacy twin keeps its depth. A subscription that a
worker thread creates on a node built in a generated node factory or the component
loader is raised as well. An admitted edge whose reader still holds fewer than 8
samples, for example one created on a node built outside both, refuses startup with a
fatal log line that names the node, the topic and the depth.

The same hook makes the reader of a cross-federate admitted edge BEST_EFFORT:
an ingress edge, whose publisher runs in another federate. A loss on such an
edge is detected by the record that travels over the federation link, so DDS
retransmission adds nothing the correspondence table needs, and a RELIABLE stateful
reader withholds every sample behind a sequence gap until the writer's history evicts
the lost one. The reader of a same-federate admitted edge keeps the reliability the
user asked for: a loss there is one lost call with no link to stall behind, and
the user's RELIABLE is the right choice. Under centralized coordination that is a self-sustaining stall: the
drain reaction waits the full timeout for the withheld sample, the stalled federate
delays the publication that would evict the lost one, and the next drain waits again,
once per slot of the writer's history, so one lost message became 10 lost calls and
could end the other federate with `publisher_late` (seen in early pilot runs; that counter no longer exists). Startup logs
the effective reliability next to the depth (`reader best effort, user reliable`) and
RCLLF-H001 judges the reliability the user declared; only an explicit RELIABLE
request counts as reliable, a SYSTEM_DEFAULT request is logged by name (`user
system default`) and judged best effort. A cross-federate reader the binding could
not relax (created outside the node's construction scope, still RELIABLE) is refused
at startup like a too shallow reader, since its depth alone would admit it with the
cascade intact; the same reader on a same-federate edge is admitted. `RCLLF_ADMITTED_READER_RELIABLE=1` keeps the user's reliability and
lifts that refusal, for comparison runs only.

Startup refuses an admitted edge (exit 1, before tag 0) when the RMW is not
`rmw_fastrtps_cpp`, the publication mode is not synchronous, an XML profile is
configured, intra-process is on, an endpoint is not VOLATILE, or the gate is not
linked. An admitted edge never falls back to the legacy path.

Edges that stay on the legacy path (one stub event per reaction, the subscriber takes
whatever sample is in the reader), each listed with its reason at startup and in the
generated `correspondence.json`: a cross-federate edge with `after 1 nsec` under
decentralized coordination (its sender needs at least 2 ns), a
BoundedNearestToAnchor input over a link under decentralized coordination, `transient_local`
QoS, fan-in groups that cannot be admitted as a whole (see
[Several publishers into one subscription](#several-publishers-into-one-subscription)),
publishing callbacks that are not
timer or subscription reactions (a private, non-virtual member helper counts as the
reactions that call it; a helper with an unknown caller, or called from the
constructor, stays legacy), publishing callbacks spread over more than one
callback group (`publisher_callback_split`), serialized publishes (`unsupported_api`),
edges whose bounds are unknown
(`burst_unbounded`, `period_unknown`, `rate_unbounded`, see
[Correspondence bounds](../features/execution-config.md#correspondence-bounds)), services
and actions.

An edge whose publisher belongs to a lifecycle node is admitted. The
binding binds every publisher the node creates on the edge's topic, also one made in
`on_configure` after startup or made again after a cleanup, so each of its publishes is
joined at its own tag. A publisher made later that is not VOLATILE ends the run with
exit 1. A subscription of a lifecycle node is admitted too, also one that a hook
creates or resets. Each new reader is bound when the node creates it and
passes the startup checks again (intra-process off, VOLATILE, its depth at least the
required KEEP_LAST depth). Ingress stops taking from the old reader. A record already
taken keeps its outcome, and every record still open for the old reader ends
`READER_REPLACED` once, traced and counted in the reconcile, as does a record published
before the new reader's creation tag. When the node holds more subscriptions on the
topic than the analysis found, rcllf cannot tell which one the RID names: it refuses
the new reader with `rebind_identity`, logs it and fails the reconcile instead of
guessing. While the reader the edge is bound to still lives, a new reader made next to
it could be its replacement or an unrelated twin, so the check waits until the old one
is released and runs at the next reaction of the RID. The usual
`sub_ = create_subscription(...)` in `on_configure` makes the new reader before the
assignment releases the old one, and the assignment happens within the hook's tag, so
the new reader is admitted at a later tag. "Later" compares the whole tag, so the next
microstep already counts. If the old reader still lives at a later tag (or at
shutdown), the new one is a twin and is refused with `rebind_identity`. A second new
reader that arrives while the first still waits is refused the same way: with two
candidates pending, rcllf cannot prove which one the RID names. While a new reader
waits, ingress does not take from it, so its samples stay in its DDS queue (KEEP_LAST
at the edge's reader depth, 8 by default) and are taken once it is admitted. A sample
that overflows that queue is lost there, and its record ends through the ordinary
missing path. Ingress holds a reader only by its rcl handle and a weak reference,
never by a `SubscriptionBase` shared pointer, so the node's drop of the old reader is
what releases it. A reader
ingress cannot admit (no C++ introspection type support) stops the edge, and ingress
retires the reader it had.

A reader belongs to the records published after its creation tag. Until the new
reader is bound, ingress may still take from the old one. A sample the old reader
takes of a record published after a later reader's creation tag is `STALE_READER`:
it is counted but never matched, whichever reader the binding has bound by then. The
record waits for the new reader's sample. The same holds for a sample the old reader
takes before the publish is committed, and for an early sample on a cross-federate
edge whose record arrives later: the publish tag decides once it is known. If the new
reader's sample never arrives, the record ends through the ordinary missing path
(`LOST` or `TIMED_OUT`). This is a known loss window: DDS matches a new reader to the
writer some time after it is created, and a record published in between reaches only
the old reader. The reconcile counts some of these records separately, as
`lost_discovery_gap` on the edge. That counter covers exactly the records of which
the old reader took a copy (classified `STALE_READER`) and which then ended missing.
A discovery-gap loss that the old reader never took a copy of (for example a record
published after the old reader was released) is not in that counter. It ends as an
ordinary `LOST` or `TIMED_OUT`, indistinguishable from a network loss. A reader created
off an LF worker after LF started has no tag that orders it against the publishes,
so the binding refuses it with `rebind_untagged` instead of guessing a tag. The
re-created reader of a lone lifecycle subscription is deepened to the edge's depth
like the first one.

On a legacy edge the value a subscriber reads at a tag depends on when the sample
arrives in wall time. The LF link carries only a trigger, and the reaction takes
whatever sample sits in the reader when it runs. A receiver that runs behind its sender
in wall time can take a sample published for a later tag, and one that runs ahead can
miss the sample of its own tag and read it one trigger later.

A node with two subscriptions on one topic gets one admitted edge per subscription when
each subscription has its own callback. The binding picks each edge's subscription by
the creation order of its reaction ID. Two subscriptions bound to the same callback
cannot be told apart and stay legacy (`ambiguous_endpoint`).

A topic that one node publishes from several callbacks in one callback group is
admitted. Those callbacks run as reactions of one LF reactor, which LF executes one
after another in a fixed order. Every publication becomes a record, in that order,
and the subscriber receives all of them at one tag, one callback invocation each.

### Several publishers into one subscription

The admitted edges into one subscription form one fan-in group.
Several nodes that publish on the topic of one subscription callback, two instances of
one class or different nodes, each keep their own edge, record link, carrier and
expectations, and share one reader, one drain reaction and one delivery order. The
group delivers by three keys:

1. The release tag: the publication tag plus the after-delay d, as LF's `after`
   computes it. For d = 0 it is the publication tag itself on an edge inside one process
   and the next microstep of the publication tag on a cross-federate edge.
2. On an equal release tag, the rank of the publisher: the position of its LF instance
   name in the lexicographic order of the group's instance names. Generation fixes the
   ranks once from the whole system, before `--federate` splits it, and ships them as
   `fan_in_rank` in `correspondence.json`, so every federate orders the group the same
   way.
3. On an equal publisher, publication order.

Two publishers whose timers fire at the same tags are therefore delivered alternately,
`a0 b0 a1 b1 ...`, in every run, where stock ROS 2 delivers them in the order the samples
happen to arrive. A missing sample takes its place in that order, see
[Missing messages](../features/missing-messages.md#several-publishers-into-one-subscription).
The binding counts every delivery whose key is below an earlier one of its group
(`order_violations` in the reconcile), and a nonzero count fails the run with exit
code 3.

A group is admitted whole or not at all. When one member is refused for its own reason,
every member stays legacy, with `fan_in_member_refused` and a note that names the refused
one. A group also stays legacy when:

- the members' delays differ (`fan_in_release_mismatch`; for a `join: deferred` group
  the delay plus the chained timeout),
- some members cross federates and others do not (`fan_in_mixed_locality`),
- a member's publisher is the subscribing node itself (`fan_in_self_loop`),
- a member rebinds with a lifecycle node (`fan_in_rebind`),
- two members share one publisher instance (`fan_in_shared_publisher`),
- the subscription is a BoundedNearestToAnchor input (`fan_in_bna_input`; generation
  then stops with `RCLLF-H003`, as for any legacy input of that synchronizer).

The reader depth must hold the sum of the members' bursts, since one reader keeps the
history of every writer (`reader_depth_below_burst`). A writer outside the system
description, for example `ros2 topic pub`, is `UNRESOLVED`: its samples are counted and
never delivered. An experiment with an example package of the development repository
that does not ship in the release compares this order with stock ROS 2.

## Relation to real-time ROS 2 work

See [Design Principles](design-principles.md) for how rcllf relates to
executor-level scheduling work (PiCAS, TIDE, reservation-based analysis) and
its closest prior art.

---

## References

1. M. Lohstroh, C. Menard, S. Bateni, E. A. Lee. "Toward a Lingua Franca for
   Deterministic Concurrent Systems." *ACM Transactions on Embedded Computing Systems*
   20(4), 36:1 to 36:27, 2021. DOI 10.1145/3448128.
2. T. A. Henzinger, B. Horowitz, C. M. Kirsch. "Giotto: A Time-Triggered Language for
   Embedded Programming." *EMSOFT 2001*, pp. 166 to 184. DOI 10.1007/3-540-45449-7_12.
   Journal version: *Proceedings of the IEEE* 91(1), 84 to 99, 2003.
   DOI 10.1109/JPROC.2002.805825.
3. C. M. Kirsch, A. Sokolova. "The Logical Execution Time Paradigm." In *Advances in
   Real-Time Systems*, Springer, 2012, pp. 103 to 120.
4. A. Biondi, M. Di Natale. "Achieving Predictable Multicore Execution of Automotive
   Applications Using the LET Paradigm." *RTAS 2018*, pp. 240 to 250. See also AUTOSAR
   Classic Timing Extensions (LET added in CP R4.4, 2018+) for LET's standardization
   status.
5. E. A. Lee, M. Lohstroh. "Generalizing Logical Execution Time." In *Principles of
   Systems Design* (Henzinger Festschrift), Springer, 2022, pp. 160 to 181.
   DOI 10.1007/978-3-031-22337-2_8.
6. C. Menard, A. Goens, M. Lohstroh, J. Castrillon. "Achieving Determinism in Adaptive
   AUTOSAR." *DATE 2020*, pp. 822 to 827. arXiv:1912.01367.
7. K.-B. Gemlau, L. Köhler, R. Ernst, S. Quinton. "System-Level Logical Execution
   Time: Augmenting the Logical Execution Time Paradigm for Distributed Real-Time
   Automotive Software." *ACM Transactions on Cyber-Physical Systems* 5(2), 14:1 to 14:27,
   2021. DOI 10.1145/3381847.
8. S. A. Edwards, J. Hui. "The Sparse Synchronous Model." *FDL 2020*, pp. 1 to 8.
   DOI 10.1109/FDL50818.2020.9232938.
9. D. Casini, T. Blaß, I. Lütkebohle, B. B. Brandenburg. "Response-Time Analysis of
   ROS 2 Processing Chains Under Reservation-Based Scheduling." *ECRTS 2019*,
   6:1 to 6:23. DOI 10.4230/LIPIcs.ECRTS.2019.6.
10. H. Choi, Y. Xiang, H. Kim. "PiCAS: New Design of Priority-Driven Chain-Aware
    Scheduling for ROS 2." *RTAS 2021*, pp. 251 to 263. DOI 10.1109/RTAS52030.2021.00028.
11. Z. Wang, S. Liu, D. Ji, W. Yi. "Improving Real-Time Performance of Micro-ROS with
    Priority-Driven Chain-Aware Scheduling." *Electronics* 13(9):1658, 2024.
    DOI 10.3390/electronics13091658. (Open-access journal extension of "TIDE: a
    Timing-Deterministic and Efficient Executor for micro-ROS," iThings/GreenCom/
    CPSCom/SmartData/Cybermatics 2023, pp. 487 to 494. Cited here in the journal version
    because it is the openly available source for the mechanism.)
12. "A Survey of Real-Time Support, Analysis, and Advancements in ROS 2." *Dagstuhl
    LITES* 11(1), 2025.
    <https://drops.dagstuhl.de/entities/document/10.4230/LITES.11.1.1>
13. ROS 2 documentation, "About Executors."
    <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html>
14. S. Bateni et al. "Risk and Mitigation of Nondeterminism in Distributed
    Cyber-Physical Systems." *MEMOCODE 2023*, pp. 1 to 11.

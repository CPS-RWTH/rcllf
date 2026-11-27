# Time & Coordination Model

This chapter is the single definition of what rcllf means by *logical time*, *tag*,
*after delay*, *federation*, and *determinism*, and, just as importantly, which of
those words apply to which way of running a converted system.

Lingua Franca (LF) supports more than one coordination scheme, and those schemes have
**disjoint vocabularies**. Each of those words belongs to exactly one regime, and this
chapter says which.

Read this before [Execution Model](architecture.md),
[LF Code Generation](lf-generation.md), and [Federation](../features/federation.md),
all three use the vocabulary defined here.

---

## The three coordination regimes

A *coordination regime* is the mechanism by which logical time advances consistently
across all the reactors in a program. LF defines three, and rcllf generates all three:
the unfederated program is the default, `--federate` generates a centralized
federation, and decentralized federation is experimental and opt-in with
`--coordination decentralized`.

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
    STP (safe-to-process) offset with a logging tardy handler. Example:
    `pubsub_example` with 2 federates, see
    [Federation: Coordination mode](../features/federation.md#coordination-mode).
    Experimental means that it builds and runs and that tardiness shows up as
    described below. The fault and availability story below (the CAL tradeoff
    this regime exists for) is **not** implemented: a stalled federate still
    stalls the whole federation, as under centralized coordination.

The RTI still starts the federation and coordinates its stop, but it does not grant
tag advances. Each federate decides locally when a tag is safe to process, using a **safe-to-process (STP) offset**: a per-federate
allowance for how far physical time must lead a logical tag before that tag may be
processed. Because the decision is local and the network is not omniscient, a message
can arrive after its tag has already been processed (it is **tardy**) and the
program must supply a **tardy handler** to say what to do about it.

That is exactly the point: by accepting the possibility of tardiness, a decentralized
federation can keep making progress when a peer is slow or unreachable. This is the
**consistency-availability-latency (CAL) tradeoff**, and it exists **only** in this
regime. A centralized federation trades availability for consistency by construction.
An unfederated program has no tradeoff to make.

**Vocabulary that belongs to this regime, and nowhere else:** STP offset, tardiness,
tardy message, tardy handler, CAL tradeoff, availability under faults.

### Regime property summary

| Regime | Message routing | Tardiness | Fault / availability story | Status in rcllf |
|---|---|---|---|---|
| Unfederated (single process) | in-process event queue | n/a | n/a | **Validated** |
| Federated, centralized | via RTI | **never tardy** | RTI is coordination bottleneck. All-or-nothing | **Supported** for `--federate per-node`, **Experimental** for other partitions |
| Federated, decentralized | peer-to-peer | **possible** (STP offsets) | CAL tradeoff space. Can stay available under faults | **Experimental** (STP/tardiness only. Fault story not implemented) |

### Keeping the vocabularies apart

The three vocabulary lists above are exclusive:

- Under an RTI (centralized coordination) no message is ever tardy, so STP offsets
  and tardiness do not apply.
- Tardiness, STP offsets, tardy handlers and the CAL tradeoff apply only to
  decentralized federation (`--coordination decentralized`), never to rcllf's
  federation in general. Centralized is the default.
- When these docs say "federation" without a qualifier, they mean centralized
  federation, the default.

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
it is the standard answer to scheduling jitter in automotive multicore software [4].

### Why `after` delays are a LET-style contract, not a latency measurement

Every connection between two nodes has a declared delay d. The default is 0; you set
it with `--after-delay` or in `execution.yaml` (see
[Execution Configuration](../features/execution-config.md#the-after-delay-default)).
Where d is applied depends on the connection:

- A service and a legacy topic edge are an LF connection with an `after` delay. So are
  the goal request and goal response of an action; its result, feedback and cancel
  channels use a fixed `after 1 msec`. This line is from the program generated for
  `service_example`:

  ```lf
  service_example_service_client.process_data_req -> service_example_service_server.process_data_req after 0
  ```

- An admitted topic edge (see
  [Publication correspondence](#publication-correspondence-topic-edges)) is an LF
  connection without `after`. The subscriber applies d when it hands the message to
  the callback.

That delay is a **declared logical-time contract**, in exactly the LET sense. The
value published by the upstream reaction at tag `T` becomes observable to the
downstream reaction at tag `T + d`, not "about d later", not "as soon as DDS
delivers it", but *at that tag*, on every run, regardless of how long the transport
actually took. Write-at-deadline, read-at-release, with the connection playing the
role of the LET interval.

It follows that:

- **The delay is not a measurement.** A measured DDS latency distribution may
  *inform your choice* of the value, for example so that a legacy edge's sample has
  arrived by the time its reaction runs (see
  [Execution Configuration](../features/execution-config.md)). But the value is the contract, and
  the program's semantics are defined by the declared value alone. Describing `after`
  as "modelling DDS transport latency" gets the causality backwards: latency data
  constrains a design choice. It does not define the semantics.
- **If the contract is not satisfiable, that is an observable failure, not a silent
  slowdown.** If the payload has not arrived when the reaction runs, an admitted edge
  waits up to its timeout T and then reports the message missing, and a legacy edge
  follows its take policy (skip, or bounded block). See
  [Take policy at the boundary](#take-policy-at-the-boundary) below.

LF is not LET, though. Lee and Lohstroh [5] show how LF's logical-time model
generalizes LET: a reaction has no LET interval of its own, it is logically
instantaneous, so reactions compose without the designer aligning task periods and
offsets by hand. The LET-style interval appears only where you declare it, as a
delay on a connection.

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
- Where a mechanism is only *partly* under LF's control, the guarantee is narrower.
  For example, in [action_example](../examples/action-example.md) action feedback and
  result run at repeatable tags only when both ends are inside the LF system and the
  server sends from callbacks that LF reactions run. A server whose only clients are
  outside the system takes their goals through a physical action, so its tags follow
  arrival.

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

A modeled message later than T counts as lost, and
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
   It is counted only in `take_stats` `skipped`, with no warning. Within one program Fast DDS delivers in the publish
   call, so only a link between federates is exposed.
2. A legacy take measures its timeout from when it runs. A worker that runs its take
   late accepts a sample that arrived later than tag + T without any report. With a
   known write count the take still gets the counted sample, so the value does not
   change, but the lateness is not reported.
3. Reader overflow before a writer's first take is not seen. The first sample taken
   from a writer only sets the sequence baseline, and DDS raises no
   event for a KeepLast replacement, so a sample replaced before that first take is
   lost silently.
4. A subscription on an arrival-anchored external topic dispatches on arrival, so its
   tags follow wall-clock arrival. Such inputs are outside the claim; a stamp-anchored
   external topic is inside it.
5. After a reported miss on a legacy link, a lone late sample may be delivered one tag
   late, and with several writers the dropped sample may be another
   writer's. The run carries the loss report, but the difference can
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
  `block` mode alike.
- A legacy trigger whose write count is unknown (a program generated with
  [`RCLLF_COUNT_PUBLISHES=0`](../reference/environment-variables.md#rcllf_count_publishes)
  set when running `colcon lf-generate`, or a publisher outside any reaction) does not wait in `skip` mode (the default). `block` polls up
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

!!! info "Setting the take mode"
    `skip` or `block`, and the block timeout, can be set per node or per subscription
    in the execution configuration. The environment variables
    [`ROS2_LF_TAKE_MODE`](../reference/environment-variables.md#ros2_lf_take_mode) and
    [`ROS2_LF_MAXWAIT_MS`](../reference/environment-variables.md#ros2_lf_maxwait_ms)
    override it for one run. The policy selects *which* of skip or block applies where.
    It never changes tag semantics. See
    [Execution Configuration](../features/execution-config.md) for the config file,
    precedence rules, and the timing pitfall of per-subscription blocking.

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

A topic edge that passes admission at generation (an *admitted* edge) runs the
subscriber's callback once per actual publish, with the exact sample that publish
produced, at tag t + d. The generated program counts every publish, the publishing
reaction sends that count to the subscriber over an LF connection without `after`, and
the subscriber's drain reaction joins each record with its DDS sample. Each run ends
with a typed outcome (`MATCHED`, `LOST`, `TIMED_OUT`, ...), and a reconcile summary at
stop checks that every expected publish ended exactly once. This is what puts topic
samples into the modeled input timeline of
[the determinism claim](#the-determinism-claim).

A topic edge that is not admitted (a *legacy* edge) carries only a trigger. Its
callback takes whatever sample sits in the reader when it runs, so the value it sees
depends on wall-clock arrival. A receiver that runs behind its sender can take a
sample published for a later tag, and one that runs ahead can miss the sample of its
own tag and read it one trigger later.

Which edges are admitted, how you see it in the generated code, the logs and the
traces, the reader QoS changes and the limits are on the
[Publication Correspondence](../features/correspondence.md) feature page.

### Several publishers into one subscription

Several admitted edges into one subscription form a fan-in group with one fixed
delivery order: first the release tag (publication tag plus d), then the publisher's
rank (the alphabetical position of its LF instance name), then publication order. Two
publishers whose timers fire at the same tags are therefore delivered
`a0 b0 a1 b1 ...` in every run, where stock ROS 2 delivers them in arrival order. See
[Publication Correspondence: Several publishers](../features/correspondence.md#several-publishers-into-one-subscription)
and, for a missing sample in such a group,
[Missing Messages](../features/missing-messages.md#several-publishers-into-one-subscription).

## Background

**Distributed LET.** The LET lineage has its own answer to distribution: SL-LET
(system-level LET) extends LET across a distributed system with explicit time zones
and interconnect tasks that carry values between them [7]. LF federation answers the
same question with one global tag order, enforced by coordination (an RTI, in
[Regime 2](#regime-2-federated-centralized)), rather than by aligning per-zone LET
schedules. If you know SL-LET, expect tags where you would expect time zones.

**Reactors and LET.** Menard et al. state the difference in one line: "In LET,
logical time strictly matches physical time at each task invocation and termination.
In the reactor model, reactions are logically instantaneous and thus compose without
requiring explicit alignment." [6]

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
    <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html>
14. S. Bateni et al. "Risk and Mitigation of Nondeterminism in Distributed
    Cyber-Physical Systems." *MEMOCODE 2023*, pp. 1 to 11.

# Glossary

Lingua Franca (LF) terms, explained for ROS 2 developers who have never used LF before.
Each entry links to the rcllf concept or page where it matters in practice.

## Reactor

The LF equivalent of a component: a named unit with input **ports**, output **ports**,
**timers**, internal state, and **reactions**. `colcon lf-generate` emits one reactor per
ROS 2 node class. See [Architecture](architecture.md).

## Reaction

A block of code inside a reactor that runs in response to a **trigger**, a timer firing,
a port receiving a value, or `startup`/`shutdown`. Reactions are the LF equivalent of a
ROS 2 callback (timer callback, subscription callback, …). Each generated reactor has a
`startup` reaction that binds it to a live ROS 2 node instance, plus one reaction per timer
or subscription. See [LF Code Generation](lf-generation.md).

## Logical time

A discrete, deterministic clock that every reactor in a program shares. Logical time only
advances between reactions. It does not track wall-clock time directly. Two reactions
scheduled for the same logical **tag** are guaranteed to see a consistent, repeatable
ordering, unlike two ROS 2 callbacks racing on wall-clock time under a normal executor. See
[ADR 0001, LF Is the Sole Time Authority](adr/0001-lf-owns-time.md).

## Physical time

Ordinary wall-clock time. LF programs can read physical time (e.g. for logging or
diagnostics) but must not use it to make scheduling decisions. Only logical time drives
what runs next. In rcllf, `steady_clock::now()` still returns wall time and is
explicitly *not* intercepted. See
[Troubleshooting](../reference/troubleshooting.md#steady_clocknow-returns-wall-time).

## Tag

A pair `(logical time, microstep)` that uniquely and totally orders every event in an LF
program. Two events at the same logical time but different microsteps are still
deterministically ordered. This is what lets reactor-c guarantee reproducible callback
interleaving where a ROS 2 executor cannot.

## `after` delay

An annotation on a connection between two reactor ports (`a.out -> b.in after 5 msec`)
that adds a fixed logical-time offset before the downstream reaction fires. rcllf uses
this to model DDS transport latency as an explicit, deterministic parameter instead of an
unpredictable wall-clock race. See [Execution Configuration](../features/execution-config.md).

## Coordination regime

Which mechanism keeps logical time consistent across all the reactors in a program. LF
defines three, **unfederated** (single process), **federated centralized** (RTI-mediated),
and **federated decentralized** (peer-to-peer, STP-based), and each has its own exclusive
vocabulary. rcllf exposes the first two. Which regime a page is talking about determines
which of the terms below apply. See
[The three coordination regimes](time-and-coordination.md#the-three-coordination-regimes).

## Federate / Federation

**Describes the *centralized* coordination regime, the only one rcllf generates.**

A **federate** is one reactor (or reactor subtree) that runs as its own OS process. A
**federation** is a set of federates coordinated by a shared logical clock, split across
process (or machine) boundaries. In rcllf, federation is opt-in via
`colcon lf-generate --federate`. DDS still carries the data, but the RTI (below)
keeps every federate's logical clock in lockstep. Messages are never tardy in this
regime: a federate waits rather than receiving something late. LF also defines a
decentralized federation mode with different semantics that rcllf does not expose (see
below). See [Federation](../features/federation.md) and
[Regime 2, Federated, centralized](time-and-coordination.md#regime-2-federated-centralized).

## RTI (Run-Time Infrastructure)

**Centralized coordination only.** The daemon that coordinates logical time across
federates in a federation. It does not carry application data (DDS still does that). It
only tells each federate when it is safe to advance its logical clock. It is therefore the
single point of coordination and the coordination bottleneck. A federation cannot make
progress if the RTI or any federate is unreachable. See
[Federation](../features/federation.md) for the all-or-nothing implications. In a
decentralized federation the RTI still starts and stops the federation, but it does not
mediate tag advance.

## Decentralized coordination / STP / Tardiness

**LF defines this regime. rcllf exposes it as experimental** (`--coordination
decentralized`, see [Federation](../features/federation.md)). In a decentralized
federation the RTI only coordinates startup and shutdown: federates exchange messages
peer-to-peer and each decides locally when a
tag is safe to process, using a **safe-to-process (STP) offset**. Because that decision is
local, a message can arrive after its tag was processed (it is **tardy**) and the program
supplies a **tardy handler** for that case. This is what makes the consistency-availability-latency
(CAL) tradeoff possible, and all of this vocabulary applies **only** to this regime, never
to rcllf's RTI-based federation. See
[Regime 3, Federated, decentralized](time-and-coordination.md#regime-3-federated-decentralized).

## LET (Logical Execution Time)

A scheduling model, closely related to LF's design, in which a task's observable inputs
and outputs are fixed to specific logical-time points regardless of how long the task
actually takes to execute on real hardware. rcllf's generated `after` delays give
cross-node communication LET-style semantics: a *declared* logical-time contract, not a
measurement of transport latency. See
[Logical time, tags, and the LET connection](time-and-coordination.md#logical-time-tags-and-the-let-connection)
for the full treatment and citations.

## `lfc`

The Lingua Franca compiler. It takes a `.lf` source file (generated by
`colcon lf-generate`) and produces C++ sources that implement the reactor-c
scheduler and all declared reactors, ready to be compiled into a standalone binary. See
[lfc Path Internals](../design/architecture.md).

## RID (Reaction ID)

A stable integer, assigned by the generator to every `(node class, callback)` pair, that
forms the shared vocabulary between the generated `.lf` file and the `ros2_lf_binding`
library. See [ADR 0002, RID Assignment and Stability](adr/0002-rid-assignment.md).

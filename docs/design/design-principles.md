# Design Principles

This page explains why rcllf exists, the tradeoffs it makes, and how it
compares to other real-time ROS 2 approaches. Read it to decide whether the
approach fits your system.

## Why LF instead of tuning the executor

The rclcpp single-threaded executor takes a **wait-set snapshot**, then
services what was ready in that snapshot (all ready timers, then one message
per ready subscription) before taking a new snapshot. Which handles are ready
at snapshot time depends on DDS delivery timing, OS scheduling, and timer
phasing, so callback interleaving, and therefore the data each callback
observes, varies run to run under identical inputs. See the
[ROS 2 executor documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)
for the primary-source description.

Executor tuning (priorities, callback groups, chain-aware scheduling) bounds
*when* callbacks run. It does not fix *what they observe and in what order*.
There is still no cross-node logical clock, so two nodes' callbacks can
interleave differently between runs even when each node's own dispatch order
is bounded. rcllf replaces the executor with a Lingua Franca reactor
scheduler so that ordering is fixed by the program text: declaration order
within a reactor, dependency order across reactors, never by thread
scheduling or DDS arrival order. Reactors with no dependency between them
are not ordered against each other: their reactions may run in parallel. See
[Three kinds of determinism](time-and-coordination.md#three-kinds-of-determinism)
for exactly which properties that buys and which it does not.

## The tradeoff: a declared latency contract

rcllf's generator emits every inter-node topic connection with an `after`
delay. That delay is a declared logical-time contract in the
[Logical Execution Time (LET)](time-and-coordination.md#what-let-is) sense. A
value published at tag `T` becomes observable downstream at tag `T + delay`,
regardless of how long the transport actually took. The full semantics,
including what happens when the contract is not satisfiable, are on
[Time & Coordination: the `after`-delay contract](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement).

The price of that contract is added latency. Every connection waits out its
declared delay even when the message arrives sooner. Practitioners are trading
wall-clock responsiveness for a repeatable input→output mapping. This is not
free, and it is not automatic. The declared delay must exceed the transport's
worst case for the contract to hold in practice (see
[Execution Configuration](../features/execution-config.md)). rcllf also does not address
wall-clock schedulability. It does not bound *when* a reaction runs, only what
it observes once it does. If you need response-time bounds, executor-level
scheduling work remains the answer, applied to the reactor-c scheduler rather
than to rclcpp's executor.

## How it compares to real-time ROS 2 work

There is a substantial body of work making ROS 2 execution more predictable.
It solves a different problem than LF does, and the two are complementary.

**What that work provides.** Casini et al. give the foundational response-time
analysis of ROS 2 processing chains under the default executor plus
reservation-based scheduling, the field's reference point for what is
analytically bounded. [PiCAS](https://doi.org/10.1109/RTAS52030.2021.00028)
replaces the default rclcpp executor with priority-driven, chain-aware
scheduling that reorders callback dispatch by end-to-end chain priority to
bound latency. [TIDE](https://doi.org/10.3390/electronics13091658) does the
analogous thing for micro-ROS with two chain-aware priority queues and a
priority-based communication daemon. The
[LITES 2025 survey](https://drops.dagstuhl.de/entities/document/10.4230/LITES.11.1.1)
covers the field as a whole.

**What it does not provide.** These are *scheduling* results. They bound
latency and make dispatch order within one executor predictable. None of them
makes execution a deterministic function of the input timeline, and none of
them orders events *across* nodes, since there is no cross-node logical clock to
order them by. Their "determinism" is bounded, predictable latency, which is a
different property from a repeatable input→output mapping.

**The relationship, stated precisely.** Executor-level priority and
chain-aware scheduling is orthogonal to LF's tag-ordered coordination. A
system can want both, and neither substitutes for the other.

**Closest prior art.** Menard et al.'s work on Adaptive AUTOSAR is the nearest
precedent for what rcllf does. It shows an industrial middleware to be
nondeterministic by default (thread-based components, undefined message
processing order) and fixes it by implementing components as reactors with
safe-to-process scheduling over a modified transport binding. The difference
is scope. That work is AUTOSAR-specific and requires the application and
transport to be built for it, while rcllf converts unmodified ROS 2
applications automatically. For the risks that appear once a reactor program
is distributed, Bateni et al. catalogue the nondeterminism hazards of
distributed cyber-physical systems, relevant reading before relying on
[federated, centralized coordination](time-and-coordination.md#regime-2-federated-centralized).

Full citations for all of the above are in
[Time & Coordination: References](time-and-coordination.md#references).

## Summary for both audiences

- **Practitioner:** if your bug reports read "works on my machine" and trace
  back to callback interleaving, LF removes that source of variation at the
  cost of a latency contract you must size correctly. It does not make your
  system faster.
- **Researcher:** rcllf is not a scheduling contribution, and it does not
  compete with PiCAS, TIDE, or reservation-based analysis. It is a
  determinism contribution (firing-order and data determinism via LF's
  tagged-event model) applied automatically to unmodified ROS 2
  applications. See
  [Three kinds of determinism](time-and-coordination.md#three-kinds-of-determinism)
  for the precise claims and their scope.

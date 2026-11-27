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
[ROS 2 executor documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html)
for the primary-source description.

Executor tuning (priorities, callback groups, chain-aware scheduling) bounds
*when* callbacks run. It does not fix *what they observe and in what order*.
There is still no cross-node logical clock, so two nodes' callbacks can
interleave differently between runs even when each node's own dispatch order
is bounded. rcllf replaces the executor with a Lingua Franca (LF) reactor
scheduler so that ordering is fixed by the program text: declaration order
within a reactor, dependency order across reactors, never by thread
scheduling or DDS arrival order. Reactors with no dependency between them
are not ordered against each other: their reactions may run in parallel. See
[Three kinds of determinism](time-and-coordination.md#three-kinds-of-determinism)
for exactly which properties that buys and which it does not.

## The tradeoff: a declared delay

Every topic connection between two nodes has a declared logical delay d. The
default is 0; you set it with `--after-delay` or in `execution.yaml`. A message
published at [tag](glossary.md#tag) `T` reaches the subscriber's callback at
tag `T + d`, no matter how long the transport actually took. This is a
contract in the
[Logical Execution Time (LET)](time-and-coordination.md#what-let-is) sense.
For a topic, the subscriber applies the delay when it hands the message to the
callback (see [Publication correspondence](../features/correspondence.md)).
Services, actions and the remaining topic connections carry it as an LF
`after` delay. The full semantics are on
[Time & Coordination: the `after`-delay contract](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement).

The price of that contract is latency. A subscriber cannot run its callback
for tag `T + d` before the message published at `T` has arrived, so a slow
transport holds it back instead of changing what it sees. You trade
wall-clock responsiveness for a repeatable input to output mapping. rcllf
does not address wall-clock schedulability. It does not bound *when* a
reaction runs, only what it observes once it does. If you need response-time
bounds, executor-level scheduling work remains the answer, applied to the
reactor-c scheduler rather than to rclcpp's executor.

## How it compares to real-time ROS 2 work

Real-time ROS 2 work such as response-time analysis of processing chains,
PiCAS and TIDE makes dispatch *timing* predictable: it bounds latency and
fixes the order within one executor. It does not order events *across*
nodes, because there is no cross-node logical clock. rcllf does the
opposite: it fixes order and observed data across nodes and leaves timing to
the scheduler, so the two approaches complement each other.

### Background

The scheduling work has a long line. Casini et al. give the response-time
analysis of ROS 2 processing chains under reservation-based scheduling.
[PiCAS](https://doi.org/10.1109/RTAS52030.2021.00028) replaces the default
rclcpp executor with priority-driven, chain-aware scheduling.
[TIDE](https://doi.org/10.3390/electronics13091658) does the same for
micro-ROS. The
[LITES 2025 survey](https://drops.dagstuhl.de/entities/document/10.4230/LITES.11.1.1)
covers the field as a whole.

The nearest precedent for what rcllf does is Menard et al.'s work on Adaptive
AUTOSAR. It implements AUTOSAR components as reactors to remove the
middleware's nondeterministic message order. That work needs the application
and transport to be built for it, while rcllf converts unmodified ROS 2 code.
Bateni et al. catalogue the nondeterminism risks of distributed
cyber-physical systems, which is useful reading before relying on
[federated, centralized coordination](time-and-coordination.md#regime-2-federated-centralized).

Full citations are in
[Time & Coordination: References](time-and-coordination.md#references).

## When rcllf fits

- Your bug reports read "works on my machine" and trace back to callback
  interleaving. rcllf removes that source of variation.
- You want two runs with the same inputs to give the same outputs, for
  testing, replay or certification evidence.
- Your nodes are written in C++ with rclcpp and set up their timers,
  subscriptions and clients in the constructor or a lifecycle hook. The
  [Support Matrix](../features/support-matrix.md) lists what converts.

## When it does not fit

- You need a faster system or hard response-time bounds. rcllf does not make
  callbacks faster and adds no wall-clock guarantees.
- Your inputs come from outside the system and their arrival time is the
  signal. Such inputs keep their arrival-time behavior (see
  [the determinism claim](time-and-coordination.md#the-determinism-claim)).
- Your code uses a pattern the analyzer refuses, such as a blocking service
  client ([RCLLF-H009](../reference/diagnostic-codes.md#rcllf-h009)). The
  [Support Matrix](../features/support-matrix.md) lists each one.

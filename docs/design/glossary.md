# Glossary

Lingua Franca (LF) and rcllf terms, explained for ROS 2 developers who have
never used LF. Each entry links to the page where the term matters in practice.

## LF terms

### Reactor

The LF unit of structure: a named block with input and output **ports**,
**timers**, **actions**, internal state and **reactions**. `colcon lf-generate`
turns every ROS 2 node into an outer shell reactor that holds the node's ports,
plus one sub-reactor per [callback group](#callback-group) that holds its
callbacks. See [LF Code Generation](lf-generation.md#node-structure).

### Reaction

A block of code inside a reactor that runs when a **trigger** is present: a
timer fires, a port receives a value, an action is scheduled, or the program
starts or stops. A reaction is the LF counterpart of a ROS 2 callback. Each
generated reaction calls one real callback of your node. The shell reactor
also has a `startup` reaction that binds it to the running ROS 2 node. See
[LF Code Generation](lf-generation.md).

### Logical time

A clock that every reactor in a program shares and that only the LF scheduler
advances. All reactions at one logical time see that same time, however long
they take to run. Reactions at the same **tag** run in an order fixed by the
program, unlike ROS 2 callbacks that race on wall-clock time. In rcllf, the
node's ROS clock reads logical time. See
[ADR 0001: LF owns time](adr/0001-lf-owns-time.md) and
[Time & Coordination](time-and-coordination.md#logical-time-and-tags).

### Physical time

Ordinary wall-clock time. LF waits for physical time to reach a tag before it
runs that tag's reactions, but which reactions run, and in what order, is
decided by logical time only. In rcllf, `std::chrono::steady_clock::now()`
still returns wall time. See
[Troubleshooting](../reference/troubleshooting.md#steady_clocknow-returns-wall-time).

### Tag

A pair (logical time, [microstep](#microstep)) that orders every event in an
LF program. Two events at the same logical time but different microsteps are
still ordered, and the order is the same in every run.

### Microstep

The second part of a [tag](#tag). An event one microstep later happens at the
same logical time, but strictly after. A connection with `after 0` delivers
its value one microstep after it was sent, which is how rcllf orders a service
request before its response at one logical time.

### `after` delay

A declared logical delay on a connection between two ports, for example
`a.out -> b.in after 5 msec`. A value sent at tag T is seen downstream at
T plus the delay, no matter how long the transport actually took. It is a
contract, not a measurement of DDS latency. See
[Time & Coordination](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement)
and [The after-delay default](../features/execution-config.md#the-after-delay-default).

### Physical action

An LF event source that code outside LF schedules, stamped with the physical
time at which it was scheduled. rcllf uses physical actions where a sample from
outside the LF program enters it, for example a declared external topic. A
**logical action** is scheduled from inside a reaction at a logical time.

### Worker

An LF worker thread. The program runs the reactions of one tag on its workers,
so reactions without a dependency between them may run in parallel. The
number is the [`workers`](../features/execution-config.md#workers) setting.

### LET (Logical Execution Time)

A timing model in which a task's inputs and outputs are fixed to logical
points in time, however long the task actually runs. rcllf's declared delays
give communication between nodes this kind of contract. See
[What LET is](time-and-coordination.md#what-let-is).

### Coordination regime

How logical time stays consistent across all reactors of a program. LF has
three: **unfederated** (one process), **federated, centralized** (an RTI grants
every time advance) and **federated, decentralized** (each federate decides
locally). rcllf generates all three. Unfederated is the default, centralized
comes from `--federate`, and decentralized is experimental. Each regime has its
own vocabulary, so a page always says which regime it means. See
[The three coordination regimes](time-and-coordination.md#the-three-coordination-regimes).

### Federate / Federation

A **federate** is a part of the LF program that runs as its own process. A
**federation** is a set of federates that share one logical timeline across
processes or machines. In rcllf it comes from `colcon lf-generate --federate`.
DDS still carries the data. Under centralized coordination the RTI keeps the
federates' logical clocks consistent, and a federate waits instead of
receiving a message late. See [Federation](../features/federation.md) and
[Regime 2](time-and-coordination.md#regime-2-federated-centralized).

### RTI (Run-Time Infrastructure)

The process that coordinates a federation. It carries no application data. In
a centralized federation it tells each federate when it may advance its
logical time, so the federation cannot progress while the RTI or any federate
is unreachable. In a decentralized federation the RTI still starts the
federation and coordinates its stop, but it does not grant time advances. See
[Federation](../features/federation.md).

### Decentralized coordination / STP / Tardiness

The federated regime without central time grants. rcllf generates it as an
experimental option (`--coordination decentralized`, see
[Federation](../features/federation.md)). Each federate decides locally when a
tag is safe to process, using a **safe-to-process (STP) offset**. A message can
then arrive after its tag was processed; it is **tardy**, and a **tardy
handler** deals with it. These terms apply only to this regime. See
[Regime 3](time-and-coordination.md#regime-3-federated-decentralized).

### `lfc`

The Lingua Franca compiler. It takes the generated `.lf` file and produces the
C and C++ sources of the reactor-c program, which the generated colcon package
then compiles. See [Architecture](architecture.md).

## rcllf terms

### Callback group

A ROS 2 grouping that tells the executor which callbacks may run in parallel.
rcllf maps each MutuallyExclusive group to one sub-reactor and each callback of
a Reentrant group to its own sub-reactor. See
[Callback Groups](callback-groups.md).

### RID (Reaction ID)

A number the generator assigns to every (node class, callback) pair. The
generated `.lf` file and the runtime binding both use it to name a callback.
See [Architecture: Reaction IDs](architecture.md#reaction-ids) and
[ADR 0002](adr/0002-rid-assignment.md).

### Admitted edge / Legacy edge

A topic **edge** is one publisher node and one subscriber node on one topic.
An **admitted** edge has [publication correspondence](#publication-correspondence):
each publish runs the subscriber's callback exactly once, with exactly that
sample. A **legacy** edge only triggers the subscriber's reaction, and the
callback takes whatever sample its reader holds. See
[Publication Correspondence](../features/correspondence.md).

### Publication correspondence

The rcllf guarantee for admitted edges: every publish maps to exactly one
subscriber callback run at a fixed tag, and every run ends with a typed outcome
such as `MATCHED` or `LOST`. See
[Publication Correspondence](../features/correspondence.md).

### Stub (`stub_t`)

The type of ports that carry a count instead of a message: an `int32_t`. A
service, action or legacy topic port holds how many requests, responses or
publishes happened, and the receiving reaction runs its callback that many
times. The message itself travels over DDS.

### Take mode

How a legacy subscription takes its sample when its reaction runs. A legacy
subscription that knows how many samples to expect waits for them up to a
timeout. When that count is unknown, `skip` (the default) does not wait for a
sample that has not arrived, and `block` waits up to the timeout. See
[Take policy at the boundary](time-and-coordination.md#take-policy-at-the-boundary).

### Diagnostic code

A code such as `RCLLF-H009` that the analyzer, the generator or the running
program prints for a pattern it cannot convert or that weakens a guarantee. An
error stops generation unless it is acknowledged in `execution.yaml`. Every
code is listed on
[Diagnostic Codes](../reference/diagnostic-codes.md).

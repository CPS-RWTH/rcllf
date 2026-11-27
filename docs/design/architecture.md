# Architecture

How a ROS 2 workspace becomes a Lingua Franca (LF) program: the four pipeline
stages, the process startup sequence, and the scheduling rules that decide when each
ROS 2 callback runs. For a shorter tour, read
[How rcllf Works](../getting-started/how-rcllf-works.md) first. The rest of `design/`
refines one stage of what follows.

```text
ROS 2 sources          extraction            generation            build                 runtime
(C++ + launch)   ──>  colcon lf-analyze ──> colcon lf-generate ──> colcon lf-build   ──> reactor-c
                      system.json           <Stem>.lf + RIDs      lfc + binding         scheduler
```

Every stage is automated and the original ROS 2 node source is never modified. The
program that runs is a reactor program compiled by the LF compiler `lfc`, linked with
the rcllf runtime library `ros2_lf_binding`. No `rclcpp::Executor` runs at any point.
`packages/lf-frame/` holds the binding (`ros2_lf_binding/`, `ros2_lf_msgs/`) and three
libraries a node may opt into (`rcllf_message_filters/`, `rcllf_tf2/`, `rcllf_plant/`).
All LF semantics come from the LF compiler. The binding is a thin dispatch layer keyed
by an integer [reaction ID (RID)](#reaction-ids): it keeps the tables that map each
reaction to its ROS 2 callback and makes no scheduling decision of its own.

This page describes the unfederated regime. Federated deployments change the time and
coordination story only, see
[Time & Coordination Model](time-and-coordination.md#the-three-coordination-regimes).

## Stage 1 (Extraction)

`colcon lf-analyze` statically analyzes the package: its launch files (Python, XML
and YAML), its C++ node sources and its `CMakeLists.txt`. It resolves the topic,
service and action connections across nodes under namespace rules and writes one
`system.json` per launch file. `system.json` is the only input to every later stage.

The parsers and the module layout are in [Extraction](extraction.md), and what
static analysis cannot see is under
[Extraction: What the analyzer cannot see](extraction.md#what-the-analyzer-cannot-see).

## Stage 2 (LF generation)

`colcon lf-generate` turns a `system.json` into an LF program and a self-contained
colcon package `lf_generated_<package>_<system>` (for `pubsub_example` and
`bringup.launch.py`: `lf_generated_pubsub_example_bringup`; the naming rule is in the
[CLI Reference](../reference/cli.md#generated-package-names)).

### Reaction IDs

A **RID** is a stable integer naming one (node, callback) pair. RIDs are assigned in
`system.json` node order, then within each node in reaction declaration order: egress
flush reactions, then timer callbacks before subscription callbacks (the default
`reaction_order: timers_first`, the reverse with `subscriptions_first`), each group in
C++ source order, then service and action reactions. The generated header for
`pubsub_example` (`lf/include/ros2_lf_reaction_ids.h`):

```c
#define RID_PublisherNode_timer_callback 0
#define RID_SubscriberNode_topic_callback 1
#define RID_COUNT 2
```

RIDs stay stable across regenerations as long as `system.json` node order and the
`.cpp` method order are unchanged. They are the only shared vocabulary between the
generated `.lf` and the binding library: no strings, no reflection, no lookup by name
at dispatch time. [ADR 0002](adr/0002-rid-assignment.md) records why.

### The generated `.lf` file

Each node becomes a shell reactor that binds the node at `startup`, plus one
sub-reactor per callback group that holds the timer and subscription reactions. A
`main` reactor instantiates the nodes and wires them together. The full shape, with a
real excerpt from `pubsub_example`, is in
[LF Code Generation](lf-generation.md#node-structure).

### Communication primitives

Each ROS 2 primitive maps to a fixed reaction shape. Every connection between two
nodes has a declared delay d, set with `--after-delay` (default `0`, see
[CLI Reference](../reference/cli.md#colcon-lf-generate)) or per link in
`execution.yaml`.

A **topic** edge that is admitted for
[publication correspondence](../features/correspondence.md) costs one output port on
the publisher and a drain reaction on the subscriber. The publisher's reaction runs the
user callback and sends the number of publishes over an LF connection without `after`.
The subscriber's drain reaction runs the callback once per publish at tag t + d, each
time with the sample that publish produced. A **legacy** topic edge (one that is not
admitted) sends a trigger with `after d`, and the subscriber's reaction takes whatever
sample the reader holds.

A **service** costs three reactions (send on the client, handle on the server,
response callback back on the client) connected as `client.<service>_req ->
server.<service>_req` and `server.<service>_resp -> client.<service>_resp`, each with
`after d`. With d = 0 a request made at tag `(T, m)` is handled at `(T, m+1)` and the
response callback runs at `(T, m+2)`. From the program generated for
`service_example`:

```lf
service_example_service_client.process_data_req -> service_example_service_server.process_data_req after 0
service_example_service_server.process_data_resp -> service_example_service_client.process_data_resp after 0
```

A blocking client (`spin_until_future_complete`, `future.get()` inside a callback) is
refused at analysis with [RCLLF-H009](../reference/diagnostic-codes.md#rcllf-h009). Use
`async_send_request` with a response callback.

An **action** is a set of counted ports, one per channel: goal, result and cancel
requests, goal, result and cancel responses, and feedback. The reaction that ran the
sending callback raises that channel's port by the number of sends, and the receiving
reaction takes exactly that many samples. The goal request and goal response use the
delay d. Every other channel carries a fixed `after 1 msec`, which also keeps the
request and response reactions free of zero-delay cycles. The full reaction table is
on the [action example](../examples/action-example.md#generated-lf-lfc-path) page.

!!! warning "Scope of the action determinism claim"
    Feedback, result and cancel run at repeatable tags when both ends are inside the LF
    system and the server sends only from callbacks that LF reactions run. A server
    that sends from its own thread is refused at generation
    ([RCLLF-H011](../reference/diagnostic-codes.md#rcllf-h011)). A server whose only
    clients are outside the LF system takes their goals through a physical action, so
    its tags follow arrival. See limit L2 in the
    [support matrix](../features/support-matrix.md#l2).

Callback-group routing composes with all of the shapes above. See
[Callback Groups](callback-groups.md) for the MutuallyExclusive and Reentrant mapping
and the `*_sa` sub-reactor used when a Reentrant group also holds service or action
callbacks.

### Output layout

`colcon lf-generate` writes into two places under `packages/`. For `pubsub_example`
and `bringup.launch.py`:

```text
lf-analysis/pubsub_example/bringup/
├── system.json, resolved_model.json, exec_config.json, ...   ← analysis results
└── lf/
    ├── Bringup.lf                         ← LF program
    └── include/ros2_lf_reaction_ids.h     ← RID macros
lf-generated/lf_generated_pubsub_example_bringup/   ← self-contained colcon package
├── CMakeLists.txt                ← runs lfc, links ros2_lf_binding
├── package.xml
├── system.json, exec_config.json, correspondence.json
├── lf/
│   ├── Bringup.lf                ← copy of the LF program
│   └── include/ros2_lf_reaction_ids.h
├── launch/
│   ├── lf_bringup.launch.py      ← what `ros2 lf launch pubsub_example bringup` runs
│   └── rcllf_timer_offsets.py
└── src/
    ├── ros2_lf_startup.cpp       ← start-up before LF's main()
    ├── ros2_lf_reaction_ids_init.cpp   ← fills the RID table
    ├── node_factories.cpp        ← builds the non-component nodes
    ├── nodes/pubsub_example/     ← copies of the node sources, compiled in
    └── rcllf_trace_build_info.cpp
```

## Stage 3 (lfc compilation)

The generated `CMakeLists.txt` drives the whole compile inside one `colcon build`
(which `colcon lf-build` runs). At configure time it runs `lfc --no-compile` to turn
`<Stem>.lf` into C sources under `build/lf_generated_<package>_<system>/lf-src/src-gen/`,
then adds `lfc`'s own generated `CMakeLists.txt` with `add_subdirectory` to build the
program. The files under `src/` compile into that same program, linked with
`ros2_lf_binding`.

The `dev` Docker image bundles `lfc` v0.12.1 at `/opt/lfc/bin/lfc`, Java 21 (required by
lfc), ROS 2 Jazzy with colcon, and the RTI binary for federated runs. No separate
toolchain image is needed.

## Stage 4 (Runtime)

### Startup sequence

Initialization runs before `lfc`'s own `main()`, driven by `ros2_lf_startup.cpp`:

1. **`ros2_lf_reaction_ids_init()`** fills the static RID table: node class, callback
   name, kind, and period or topic for each RID.
2. **`ros2_lf_runtime_init()`** calls `rclcpp::init()` and reads the `system.json`
   named by the `SYSTEM_JSON` environment variable, which the generated launch file sets.
3. **`ros2_lf_register_all_nodes()`** registers the non-component nodes built by the
   generated `node_factories.cpp`.
4. **`ros2_lf_runtime_load_nodes()`** loads the component nodes and constructs the
   non-component ones.
5. **`ros2_lf_runtime_wait_ready()`** waits for DDS discovery to settle and anchors LF
   logical time zero to that instant.
6. **`ros2_lf_bind_node("ClassName")`** runs from each node's `startup` reaction and
   binds that node's entities to their RIDs.

The component manager the binding uses does **not** add loaded components to an
executor. Nodes are constructed and never spun. The LF scheduler is the only
scheduler.

### What `ros2_lf_bind_node` does

Called once per node at the first tag, it walks the node's callback groups and matches
the live ROS 2 entities against the RID table:

- Find the loaded node by its class name.
- Walk all callback groups, collecting timers, subscriptions, service servers and
  clients, and action servers and clients.
- Match each entity to its RID: timers by exact period in nanoseconds, subscriptions
  by topic name, services by service name, actions by type.
- **Cancel every matched timer**, so ROS 2 cannot fire it on its own.

Timer cancellation is what transfers time authority. A timer that escapes matching
stays live and fires outside logical time, so matching is exact and every match is
logged. From a run of `pubsub_example`:

```text
[ros2_lf_binding]: Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)
[ros2_lf_binding]: ros2_lf_bind_node(PublisherNode): 1 timer(s), 0 subscription(s), 0 service(s), 0 client(s) bound
[ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/sensor_data', take_mode=skip timeout_ms=100)
[ros2_lf_binding]: ros2_lf_bind_node(SubscriberNode): 0 timer(s), 1 subscription(s), 0 service(s), 0 client(s) bound
```

### Execution flow

Once startup completes, the LF scheduler owns execution. For the admitted topic edge
of `pubsub_example`:

```text
LF scheduler (owns the logical clock)
    │  fires the timer reaction at tag T
    v
ros2_lf_invoke_timer(self, RID_PublisherNode_timer_callback)
    v
user's timer_callback()  (unmodified .cpp)
    │  publisher_->publish(msg)   ← normal DDS publish, counted by the binding
    v
reaction sets port corr_e0 to the number of publishes
    │  LF connection, no after
    v
subscriber's drain reaction at tag T + d
    │  ros2_lf_corr_drain(self, 0, RID_SubscriberNode_topic_callback, "sensor_data")
    v
user's topic_callback(msg)  (unmodified .cpp), once per publish, with that sample
```

LF owns time. ROS 2 owns transport. User code is untouched. Messages still travel
over DDS. What changes is *who calls the callback*: the LF scheduler at a tag fixed by
the program, instead of an executor's `spin()` loop.

A subscription callback that publishes triggers further downstream callbacks, so
causal chains extend to any length. Before every reaction, LF logical time is pushed
into the `RCL_ROS_TIME` clock of every node the binding constructs, which is what makes
`Clock::now()` inside user callbacks agree with the scheduler, see
[Time & Coordination Model](time-and-coordination.md#clocknow-after-conversion).

The C API, RID and handle table layouts, time source and unit tests are documented in
[ros2_lf_binding Internals](../reference/ros2-lf-binding.md). Its sources live in
`packages/lf-frame/ros2_lf_binding/` with the reactor-c runtime vendored at
`third_party/reactor-c` as a git submodule.

## Scheduling semantics

### Timers before the subscriptions they cause

Each connection has a delay d, a
[declared logical-time contract](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement),
not a latency measurement. At tag `T` a timer reaction fires. Any subscription
callback triggered by the message it publishes runs at `T + d` or later, never
before. That ordering is structural (encoded in the `.lf` connections) rather than an
executor policy.

### Ties at the same logical tag

Declaration order is a **tie-break, not the ordering rule.** LF first orders the
reactions ready at a tag by the reaction dependency graph, which assigns each reaction a
level. The runtime executes level by level. A connection is a precedence edge, so a
reaction reading a port always runs after the reaction that writes it, whatever order
the two were declared in. Declaration order decides nothing there.

It decides the order only among reactions the graph leaves *mutually unordered*, meaning
no dependency path runs between them:

- Within one reactor, the reaction declared first wins. The generator declares a node's
  timer reactions before its subscription reactions, each group in C++ source order.
  `execution.yaml` `reaction_order: subscriptions_first` declares the subscription
  reactions first instead ([Reaction order at one
  tag](../features/execution-config.md#reaction-order-at-one-tag)).
- Across reactors, declaration order decides nothing. Reactions of different reactors
  with no dependency path between them may run on different worker threads at the same
  time, in either order. Only a connection, or `workers: 1`, serializes them; with one
  worker the order is the scheduler's, not a promise.

So reordering two subscription callbacks (or two timer callbacks) in a `.cpp` file
changes their priority, and `reaction_order` changes the priority between a timer and a
subscription. Neither changes anything between connected reactions. RID assignment
follows the same declared order.

Note also that `after 0` is not "no delay". An LF connection without `after` delivers at
the writer's tag (t, m), and `after 0` advances one microstep to (t, m+1). An admitted
topic edge between two nodes of one process is a connection without `after`, so with
d = 0 the subscription callback runs at the publication tag (t, m). A self edge, a
cross-federate d = 0 link and a legacy edge, which keeps `after 0`, run at (t, m+1) or
later. See the
[troubleshooting entry on stale subscriber data](../reference/troubleshooting.md#a-subscriber-callback-sees-stale-data-or-none)
for why these rules do not protect the *payload* of a legacy edge, which travels over
DDS rather than through the LF port.

## Running the pipeline

[First LF Run](../getting-started/first-lf-run.md) walks through the pipeline on
`pubsub_example`, and the [CLI Reference](../reference/cli.md) documents every
command and flag.

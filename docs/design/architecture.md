# Architecture

How a ROS 2 workspace becomes a Lingua Franca program: the four pipeline stages, the
process startup sequence, and the scheduling rules that decide when each ROS 2 callback
runs. Read this before any other design page, the rest of `design/` refines one stage
of what follows.

```text
ROS 2 sources          extraction            generation            lfc build             runtime
(C++ + launch)   ──>  colcon lf-analyze ──> colcon lf-generate ──> colcon build     ──> reactor-c
                      system.json           <Stem>.lf + RIDs      lfc + binding         scheduler
```

Every stage is automated and the original ROS 2 node source is never modified. The lfc
path is the **sole runtime**: `ros2_lf_binding` plus an `lfc`-compiled reactor program.
There is no bridge executor and no `rclcpp::Executor` is in the loop at any point.
`packages/lf-frame/` holds the binding (`ros2_lf_binding/`, `ros2_lf_msgs/`) and three
libraries a node may opt into (`rcllf_message_filters/`, `rcllf_tf2/`, `rcllf_plant/`).
All LF semantics come from the LF compiler. The binding is a thin dispatch layer keyed
by integer RID: it keeps the registration tables that map each reaction to its ROS 2
callback and makes no scheduling decision of its own.

This page describes the unfederated regime. Federated deployments change the time and
coordination story only, see
[Time & Coordination Model](time-and-coordination.md#the-three-coordination-regimes).

## Stage 1 (Extraction)

`colcon lf-analyze` statically analyzes the workspace. Three parsers run independently
and their outputs are joined into one model:

```text
Launch files (.py/.xml)     C++ source (.cpp/.hpp)      CMakeLists.txt
        │                            │                        │
 launch_analysis/parser.py   node_analysis/           node_analysis/
        │                    clang_parser.py          cmake_parser.py
        v                            v                        v
   LaunchGraph              NodeInfo (per node)          CMakeInfo
        └────────────────────────────┼────────────────────────┘
                                     v
                      comm_analysis/topic_mapper.py
                                     v
                          SystemCommunication
                                     v
                       graph/hierarchical.py  ──>  SystemGraph  ──>  system.json
```

The launch layer yields file hierarchy, includes, namespace scoping, conditionals,
remappings and parameter files. The node layer yields publishers, subscribers, timers,
services, actions, callback groups, parameters, lifecycle callbacks and component
registration. The communication layer resolves topic, service and action connections
across nodes under namespace resolution. `SystemGraph` serializes to `system.json`,
which is the sole input to every later stage.

Extraction detail, including parser limits and what defeats static analysis, is in
[Extraction](extraction.md).

## Stage 2 (LF generation)

`colcon lf-generate` turns a `system.json` into a self-contained colcon
package:

```text
system.json
    ├── generation/lf_generator.py           ──>  <Stem>.lf
    ├── generation/binding_header_emitter.py ──>  ros2_lf_reaction_ids.h
    └── verb/generate.py                     ──>  lf_generated_<package>_<stem>/
```

### Reaction IDs

A **RID** is a stable integer naming one (node, callback) pair. RIDs are assigned in
`system.json` node order, then within each node in reaction declaration order: egress
flush reactions, then timer callbacks before subscription callbacks (the default
`reaction_order: timers_first`, the reverse with `subscriptions_first`), each group in C++ source-file definition order (libclang
AST pre-order), then service and action reactions:

```c
#define RID_TimerPublisherNode_on_timer  0
#define RID_RelayNode_on_message         1
#define RID_ListenerNode_on_message      2
#define RID_COUNT                        3
```

They stay stable across regenerations as long as `system.json` node order and the `.cpp`
method order are unchanged. RIDs are the entire shared vocabulary between the generated
`.lf` and the binding library, no strings, no reflection, no lookup by name at dispatch
time.

### The generated `.lf` file

One reactor per ROS 2 node class, plus a `main` reactor wiring them together. Each reactor
carries a `startup` reaction that binds handles, one reaction per timer, and one reaction
per subscription:

```lf
target CCpp {
    build-type: RelWithDebInfo,
    keepalive: true,
}

preamble {=
    #include "ros2_lf_binding/ros2_lf_binding.h"
    #include "ros2_lf_reaction_ids.h"
    typedef int32_t stub_t;  // write count of the port at this tag
=}

reactor TimerPublisherNode {
    output timer_out: stub_t
    timer t_on_timer(0, 1000 msec)

    reaction(startup) {=
        ros2_lf_bind_node(self, "TimerPublisherNode");
    =}

    reaction(t_on_timer) -> timer_out {=
        ros2_lf_invoke_timer(self, RID_TimerPublisherNode_on_timer);
        { stub_t rcllf_c = timer_out->is_present ? timer_out->value : 0; if (__builtin_add_overflow(rcllf_c, 1, &rcllf_c)) { lf_print_error_and_exit("rcllf: write count of port 'timer_out' overflows int"); } lf_set(timer_out, rcllf_c); }
    =}
}

reactor RelayNode {
    input timer_out: stub_t
    output relay_out: stub_t

    reaction(startup) {=
        ros2_lf_bind_node(self, "RelayNode");
    =}

    reaction(timer_out) -> relay_out {=
        for (stub_t rcllf_k = timer_out->is_present ? timer_out->value : 0; rcllf_k > 0; rcllf_k--) {
            if (ros2_lf_invoke_sub(self, RID_RelayNode_on_message, "basic_components/timer_out") > 0) {
                { stub_t rcllf_c = relay_out->is_present ? relay_out->value : 0; if (__builtin_add_overflow(rcllf_c, 1, &rcllf_c)) { lf_print_error_and_exit("rcllf: write count of port 'relay_out' overflows int"); } lf_set(relay_out, rcllf_c); }
            }
        }
    =}
}

main reactor ComboPipeline {
    tp = new TimerPublisherNode()
    rn = new RelayNode()
    ln = new ListenerNode()
    tp.timer_out -> rn.timer_out after 1 msec
    rn.relay_out -> ln.relay_out after 1 msec
}
```

Ports carry a `stub_t` payload that counts the writes of the port at one tag.
LF keeps one value per port per tag, so every writer adds one (a checked add
that exits rather than wrap) and the reader invokes its callback once per counted
write. A reader that publishes counts a write only when the invoke really ran a
callback. The stub only fires the
downstream reaction, the actual message still travels over DDS. Generator options, name mangling and current
limitations are in [LF Code Generation](lf-generation.md).

### Communication primitives

Each ROS 2 primitive maps to a fixed reaction shape. Every cross-side hop
carries an `after` delay, overridable per link by an extracted per-connection
value. The examples in this section show an explicit `1 msec`. Absent a
per-link value the generator emits the `--after-delay` flag's value, which
defaults to `0` (see
[CLI Reference](../reference/cli.md#colcon-lf-generate)).

A **topic** costs one reaction per side: the publisher's timer reaction invokes the user
callback then sets its output port. The subscriber's reaction on that port invokes
`ros2_lf_invoke_sub`.

A **service** costs three reactions (`srv_send` on the client, `srv_server` on the
server, `srv_client_cb` back on the client) connected as
`client.svc_req -> server.svc_req` and `server.svc_resp -> client.svc_resp`, each with the
`after` delay. With a 1 msec delay a request made at tag `T` is handled at `T+1ms` and
the client's response lambda fires at `T+2ms`. The timer callback that calls
`async_send_request` is classified as `srv_send` and bound as a `TIMER` handle, so its
underlying ROS timer is cancelled and reactor-c is the only thing that can fire it.

An **action** is a set of counted ports, one per channel: goal, result and cancel
requests, goal, result and cancel responses, and feedback. The action gate
counts each rcl_action send per thread, action and channel. The reaction that ran the
sending callback raises that channel's port by the count, and the receiving reaction
takes exactly that many samples, each with a bounded wait. Every channel except the goal
request and goal response carries a fixed 1 ms `after`, which also keeps the request
and response reactions free of zero-delay cycles. The full reaction table is on the
[action example](../examples/action-example.md#generated-lf-lfc-path) page.

!!! warning "Scope of the action determinism claim"
    Feedback, result and cancel run at repeatable tags when both ends are inside the LF
    system and the server sends only from callbacks that LF reactions run. A server
    that sends from its own thread is refused at generation (RCLLF-H011). A server whose
    only clients are outside the LF system takes their goals through a physical action,
    so its tags follow arrival. See limit L2 in the
    [support matrix](../features/support-matrix.md#l2).

Blocking service-client styles (`spin_until_future_complete`, `future.get()`) are designed
to wrap the response in a physical action, but are not yet exercised in the lfc path.

Callback-group routing composes with all of the shapes above. See
[Callback Groups](callback-groups.md) for the Mutex / Reentrant / `default_group` mapping
and the `*_sa` sub-reactor used when a Reentrant group also holds service or action
callbacks.

### Output layout

For a system named `ComboPipeline` from package `basic_components`:

```text
lf-analysis/basic_components/combo_pipeline/
├── lf/
│   ├── ComboPipeline.lf                  ← Lingua Franca source
│   └── include/ros2_lf_reaction_ids.h    ← RID macro definitions
└── lf_generated_basic_components_combopipeline/           ← self-contained colcon package
    ├── lf/
    │   ├── ComboPipeline.lf              ← bundled copy
    │   └── include/ros2_lf_reaction_ids.h
    ├── src/
    │   ├── ros2_lf_startup.cpp           ← pre-main init (constructor attribute)
    │   └── ros2_lf_reaction_ids_init.cpp ← populates the RID table at startup
    ├── CMakeLists.txt                    ← invokes lfc, links ros2_lf_binding
    └── package.xml
```

## Stage 3 (lfc compilation)

The generated `CMakeLists.txt` drives the whole compile inside a single `colcon build`.
`lfc` runs as a CMake `execute_process` at configure time, compiling `<Stem>.lf` into C++
sources under `build/lf_generated_<package>_<stem>/lf-src/src-gen/`. An `add_subdirectory` on
`lfc`'s own generated `CMakeLists.txt` then sets up reactor-c headers and builds the main
reactor target. `ros2_lf_startup.cpp` and `ros2_lf_reaction_ids_init.cpp` compile into
that same target alongside `ros2_lf_binding`.

The `dev` Docker image bundles `lfc` v0.12.1 at `/opt/lfc/bin/lfc`, OpenJDK 21 (required
by lfc), ROS 2 Jazzy with colcon and reactor-c headers, and the RTI binary for federated
runs. No separate toolchain image is needed.

## Stage 4 (Runtime)

### Startup sequence

Initialization runs before `lfc`'s own `main()`, driven by `ros2_lf_startup.cpp`:

1. **`ros2_lf_reaction_ids_init()`** fills the static RID table, node class, callback
   name, kind, and period or topic for each RID.
2. **`ros2_lf_runtime_init()`** calls `rclcpp::init()`, creates the
   `LFComponentManager`, and loads every component node named in `system.json`.
3. **`ros2_lf_register_all_nodes()`** populates the registry with non-component node
   instances built by the generated `node_factories.cpp`.
4. **`ros2_lf_bind_node("ClassName")`** runs from each reactor's `startup` reaction,
   binding that node's entities to their RIDs.

`LFComponentManager` is a thin subclass of `rclcpp_components::ComponentManager` that
does **not** add loaded components to an `rclcpp::Executor`. Nodes are
instantiated and left unspun. lfc is the sole scheduler.

### What `ros2_lf_bind_node` does

Called once per reactor during lfc initialization, it walks the node's callback groups and
reconciles the live ROS 2 entities against the RID table:

- Find the loaded node through the class→node registry in `LFComponentManager`.
- Walk all callback groups, collecting timers, subscriptions, service servers and
  clients, and action servers and clients.
- Match each entity to its RID: timers by exact period in nanoseconds, subscriptions
  by topic name, services by service name, actions by waitable type.
- **Cancel every matched timer**, so ROS 2 cannot self-fire it.
- Store the resulting handle in `g_handle_table[rid]`.

Timer cancellation is what transfers time authority. A timer that escapes matching stays
live and fires outside logical time, which is why period matching is tolerance-based and
logged per binding.

### Execution flow

Once startup completes, reactor-c owns execution:

```text
lfc scheduler (owns the logical clock)
    │  fires reaction at logical time T
    v
reaction body in <Stem>.cpp (lfc-generated)
    │  ros2_lf_invoke_timer(self, RID_X)
    v
reaction_dispatch.cpp
    │  g_handle_table[RID_X].timer->execute_callback()
    v
user's on_timer()  (unmodified .cpp)
    │  publisher_->publish(msg)   ← normal DDS publish
    v
DDS middleware → rmw queue
    │
    v
lfc fires the downstream reaction at T + after_delay
    │  ros2_lf_invoke_sub(self, RID_Y, "topic")
    v
user's on_message()  (unmodified .cpp)
```

lfc owns time. ROS 2 owns transport. User code is untouched. Subscriptions stay registered
with the DDS middleware and messages arrive through the normal rmw layer. What changes is
*who calls the callback*, lfc's scheduler at a logical time fixed by the `.lf`
connection graph, instead of an executor's `spin()` loop.

Subscription reactions that publish schedule further downstream subscriptions, so causal
chains extend to arbitrary length. Before every reaction, LF logical time is pushed into
the `RCL_ROS_TIME` clock of every node the binding constructs, which is what makes
`Clock::now()` inside user callbacks agree with the scheduler, see
[Time & Coordination Model](time-and-coordination.md#clocknow-after-conversion).

The C API, RID and handle table layouts, time source and unit tests are documented in
[ros2_lf_binding Internals](../reference/ros2-lf-binding.md). Its sources live in
`packages/lf-frame/ros2_lf_binding/` with the reactor-c runtime vendored at
`third_party/reactor-c` as a git submodule.

## Scheduling semantics

### Timers before the subscriptions they cause

Each connection carries an `after` delay, which is a
[declared logical-time contract](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement),
not a latency measurement. At logical time `T` a timer reaction fires. Any subscription
reaction triggered by the message it publishes is scheduled at `T + after`. That ordering
is structural (encoded in the `.lf` topology) rather than an implicit executor policy.

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
  with no dependency path between them run on different worker threads, at the same time
  and in either order. Only a connection, or `workers: 1`, serializes them; with one
  worker the order is the scheduler's, not a promise.

So reordering two subscription callbacks (or two timer callbacks) in a `.cpp` file
changes their priority, and `reaction_order` changes the priority between a timer and a
subscription. Neither changes anything between connected reactions. RID assignment
follows the same declared order.

Note also that `after 0` is not "no delay". An LF connection without `after` delivers at
the writer's tag (t, m), and `after 0` advances one microstep to (t, m+1). An admitted
topic edge between two nodes of one process is a connection without `after`, so with
d = 0 the subscription callback runs the first record of a tag at the publication tag
(t, m). A second record of one tag, a self edge, a cross-federate d = 0
link and a legacy (non-admitted) edge, which keeps `after 0`, run at (t, m+1) or later.
See the [troubleshooting entry on stale subscriber data](../reference/troubleshooting.md#a-subscriber-callback-sees-stale-data-or-none)
for why neither of these rules protects the *payload*, which travels over DDS rather
than through the LF port.

## Running the pipeline

The combo pipeline (3 nodes) builds and runs from a single compose command:

```bash
docker compose --profile lf run --rm lf-build                # basic_components, 3 nodes
```


`lf-build` runs `docker/lf_build.sh`, which builds `packages/`, analyzes and generates for
`basic_components` and `pubsub_example`, builds each generated package, and smoke-tests
both for 30 seconds. A successful run logs one binding line per RID:

```text
[INFO] [lf_component_manager]: Bound timer RID 0 (TimerPublisherNode::on_timer, period 1000000000 ns)
[INFO] [lf_component_manager]: Bound subscription RID 1 (RelayNode::on_message, topic 'basic_components/timer_out')
[INFO] [lf_component_manager]: Bound subscription RID 2 (ListenerNode::on_message, topic 'basic_components/relay_out')
[INFO] [listener_node]: Received: relayed: timer_publisher tick 0
```

The same service takes other packages. Each gets its own analyze → generate → build →
30 s smoke run:

```bash
docker compose --profile lf run --rm lf-build service_example action_example
```

To drive the stages by hand inside the `dev` container:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace/packages

colcon lf-analyze --package basic_components
colcon lf-generate \
  --system-json lf-analysis/basic_components/combo_pipeline/system.json \
  --no-launch
colcon build --packages-select lf_generated_basic_components_combopipeline
source install/setup.bash

SYSTEM_JSON=lf-analysis/basic_components/combo_pipeline/system.json \
  ros2 run lf_generated_basic_components_combopipeline ComboPipeline
```

`colcon lf` chains all four stages, and `ros2 lf launch <pkg> <stem>` runs a generated
system. Every flag is documented in [CLI Reference](../reference/cli.md).


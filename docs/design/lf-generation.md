# LF Code Generation

`colcon lf-generate` turns an analyzed system (`system.json`) into a Lingua
Franca (LF) program and a colcon package that builds it. The program has one
reactor per node class, a main reactor that creates the nodes and connects
them, and reactions whose bodies call back into the real ROS 2 callbacks
through `ros2_lf_invoke_*` functions. The generated code targets the LF **CCpp**
target: the reactor-c runtime with C++ reaction bodies.

This page explains the shape of the generated program. The steps around it,
and the files each step writes, are on [Architecture](architecture.md).

## Generate a program

Inside the dev container (see
[First LF Run](../getting-started/first-lf-run.md)):

```bash
colcon lf --package pubsub_example
```

`colcon lf` runs analyze, generate and build in one go. Among its output are
the generated LF file and the generated package:

```text
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/lf/Bringup.lf
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/lf/include/ros2_lf_reaction_ids.h
  -> /workspace/packages/lf-generated/lf_generated_pubsub_example_bringup/ (lfc package)
```

The options of the generate step are in the
[CLI reference](../reference/cli.md#colcon-lf-generate). The full layout of the
output is under [Architecture: Output layout](architecture.md#output-layout).

## Generated LF structure

The excerpts on this page come from `Bringup.lf` of `pubsub_example`, trimmed
where marked.

### Node Structure

Every node becomes an outer **shell reactor** plus one sub-reactor per
[callback group](callback-groups.md). The shell holds the node's external
ports and a `startup` reaction that binds the reactor to the running ROS 2
node. All callbacks live in the group sub-reactors. A node without explicit
callback groups gets one sub-reactor, `<Class>_DefaultGroup`.

```lf
reactor PublisherNode {
    output corr_e0: rcllf_corr_t
    default_group = new PublisherNode_DefaultGroup()
    default_group.corr_e0 -> corr_e0

    reaction(startup) -> default_group.rcllf_bound {=
        ros2_lf_bind_node(self, "PublisherNode");
        lf_set(default_group.rcllf_bound, 1);
    =}
}

reactor PublisherNode_DefaultGroup {
    output corr_e0: rcllf_corr_t
    input rcllf_bound: stub_t
    timer t_timer_callback(0, 500 msec)
    physical action ingress_wake
    logical action ingress_drain

    reaction(t_timer_callback) rcllf_bound -> corr_e0 {=
        ros2_lf_invoke_timer(self, RID_PublisherNode_timer_callback);
        { uint32_t n = ros2_lf_corr_emit(self, 0); if (n > 0) { rcllf_corr_t c = {0, n}; lf_set(corr_e0, c); } }
    =}
    ...
}
```

What maps to what:

- **Timers** become LF `timer` declarations with the analyzed period and
  offset 0. `execution.yaml` can set a start offset or a period per timer, see
  [Timer start offsets and periods](../features/execution-config.md#timer-start-offsets-and-periods).
- **Callbacks** become reactions. The body calls the callback by its
  [reaction ID](architecture.md#reaction-ids) (`RID_PublisherNode_timer_callback`).
- **Topic edges** become ports. An admitted edge (see
  [Publication Correspondence](../features/correspondence.md)) is a port
  `corr_e<N>` of type `rcllf_corr_t`, set only when the callback published,
  with the number of publishes. A legacy edge is a port of type `stub_t`
  named after the topic.
- **`rcllf_bound`** is an input of every group sub-reactor. The shell sets it
  once the node is bound, and every callback reaction reads it, so no callback
  runs before its node is bound.
- **`ingress_wake` and `ingress_drain`** let the binding hand a node's
  parameter, lifecycle and type description service requests to an LF worker
  (see [Ingress capacity](../features/execution-config.md#ingress-capacity)).

The subscriber side of an admitted edge is a drain reaction that takes the
publish records and runs the callback once per record:

```lf
reactor SubscriberNode_DefaultGroup {
    input corr_e0: rcllf_corr_t
    input rcllf_bound: stub_t
    logical action corr_drain_e0
    ...
    reaction(corr_drain_e0, corr_e0) rcllf_bound -> corr_drain_e0 {=
        ...
            w = ros2_lf_corr_drain(self, 0, RID_SubscriberNode_topic_callback, "sensor_data");
        ...
    =}
```

### Node with explicit callback groups

`callback_group_example` has one MutuallyExclusive group with two timers, two
Reentrant groups with three timers between them, and a timer and a
subscription in the default group. The generated shell creates one
sub-reactor per MutuallyExclusive group, one per Reentrant callback, and the
default group (from `CallbackGroupNodeSystem.lf`):

```lf
reactor CallbackGroupNode {
    exclusive_group = new CallbackGroupNode_ExclusiveGroup()
    timer_reentrant1a_callback = new CallbackGroupNode_TimerReentrant1aCallback()
    timer_reentrant1b_callback = new CallbackGroupNode_TimerReentrant1bCallback()
    timer_reentrant2_callback = new CallbackGroupNode_TimerReentrant2Callback()
    default_group = new CallbackGroupNode_DefaultGroup()

    reaction(startup) -> exclusive_group.rcllf_bound, ... {=
        ros2_lf_bind_node(self, "CallbackGroupNode");
        lf_set(exclusive_group.rcllf_bound, 1);
        ...
    =}
}
```

Why the groups map this way is on [Callback Groups](callback-groups.md).

### Main Reactor

The main reactor creates the nodes and connects them:

```lf
main reactor Bringup {
    publisher_node = new PublisherNode()
    subscriber_node = new SubscriberNode()

    publisher_node.corr_e0 -> subscriber_node.corr_e0
}
```

- Instance names come from the launch-level node names, made into valid LF
  identifiers.
- An admitted topic edge has no `after`. The subscriber applies the edge's
  declared delay d itself when it runs the callback.
- Service, action and legacy topic connections carry the delay as an LF
  `after`. The default is `after 0`, which is one microstep later at the same
  logical time. The delay is a
  [declared logical-time offset](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement),
  not a latency measurement. Set it with `--after-delay` or in
  [`execution.yaml`](../features/execution-config.md#the-after-delay-default).

### Service Reactions

A service client that uses `async_send_request` with a response callback maps
to three reactions: the client send, the server handler and the client
response callback. All ports are counted `stub_t` ports: the value is the
number of requests or responses, and the receiving reaction runs once per
count. The excerpts come from `service_example`.

| Endpoint | Ports | Reaction body |
|----------|-------|---------------|
| Server | `input <svc>_req: stub_t`, `output <svc>_resp: stub_t` | once per counted request: `ros2_lf_invoke_srv_server(...)`, and one write added to `<svc>_resp` |
| Client (callback that sends) | `output <svc>_req: stub_t` | `ros2_lf_invoke_srv_send(...)`, then `ros2_lf_service_sent(...)` adds the number of requests sent to `<svc>_req` |
| Client (response callback) | `input <svc>_resp: stub_t` | once per counted response: `ros2_lf_invoke_srv_client_cb(...)` |

Connections in the main reactor:

```lf
service_example_service_client.process_data_req -> service_example_service_server.process_data_req after 0
service_example_service_server.process_data_resp -> service_example_service_client.process_data_resp after 0
```

With the default `after 0`, the server handles a request one microstep after it
was sent, and the response callback runs one microstep after that, at the same
logical time. A client that blocks on its response is refused at generation
with [RCLLF-H009](../reference/diagnostic-codes.md#rcllf-h009).

### Action Reactions

Every action channel is a counted port: goal, result and cancel requests, goal,
result and cancel responses, and feedback. The reaction that ran the sending
callback adds the number of sends on each channel to that channel's port, so a
port fires only for a real send. The receiving reaction takes exactly that many
samples. Goal request and goal response use the link delay (`after 0` by
default). Every other channel uses a fixed `after 1 msec`. The reactions of a
real action are shown on the
[action example](../examples/action-example.md#generated-lf-lfc-path) page.

## Name Handling

| ROS 2 concept | LF mapping | Example |
|---------------|------------|---------|
| Node class name | Shell reactor name | `PublisherNode` → `PublisherNode` |
| Callback group | Sub-reactor name | default group → `PublisherNode_DefaultGroup` |
| Node launch name | Instance name (valid LF identifier) | `/robot_1/publisher_node` → `robot_1_publisher_node` |
| Admitted topic edge | Port `corr_e<N>`, type `rcllf_corr_t` | edge 0 on `/sensor_data` → `corr_e0` |
| Legacy topic edge | Port named after the topic, type `stub_t` | `/ns/sensor_data` → `ns_sensor_data` |
| Service / action name | Port stem (full declared name; a stem two names share stops generation) | `/rebind/fib` → `rebind_fib_goal_req` |
| Timer callback | Timer name | `timer_callback` → `t_timer_callback` |
| Launch file stem | Main reactor name | `bringup.launch.py` → `Bringup` |

When the main reactor name collides with a reactor class name, `System` is
appended: `callback_group_node.launch.py` gives the main reactor
`CallbackGroupNodeSystem`, since `CallbackGroupNode` is a class.

## Limitations

- **Blocking service clients** (`spin_until_future_complete`, `future.get()`,
  or a `spin_some`/`spin_once`, condition variable or sleep loop until the
  response callback runs) are refused at generation with
  [RCLLF-H009](../reference/diagnostic-codes.md#rcllf-h009). The response
  arrives as a later LF event, so such a wait would never end.
- **No parameter mapping.** ROS 2 parameters are not turned into LF reactor
  parameters. Nodes read their parameters as usual.
- **One file per system.** All reactors of one launch system go into one
  `.lf` file.
- **Ports carry no message data.** Ports carry counts (`stub_t`, an
  `int32_t`) or publish records (`rcllf_corr_t`). The message itself travels
  over DDS, and the callback takes it from its reader.
- **The generated program is overwritten.** Each `colcon lf-generate` run
  writes the `.lf` file again, so edits to it are lost.

## Generator internals

Generation runs in three stages: `lf_generator.py` converts the analyzed
system graph into an intermediate LF model, `lf_emitter.py` renders that model
to `.lf` text, and `verb/generate.py` is the CLI verb that finds `system.json`
files and runs the stages. The intermediate model lives in `lf_model.py`, and
naming rules in `lf_naming.py`. All four are under
`tools/colcon_lf/colcon_lf/generation/` and `tools/colcon_lf/colcon_lf/verb/`.

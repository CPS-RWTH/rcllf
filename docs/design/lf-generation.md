# LF Code Generation

`colcon lf-generate` converts extracted ROS 2 system models (`system.json`)
into Lingua Franca (`.lf`) source files. Each system produces a single `.lf`
file containing one reactor definition per unique C++ node class, a main
reactor that instantiates the nodes and wires their inter-node connections,
and timer, subscription, service, and action reactions whose bodies dispatch
back into the running ROS 2 callbacks via `ros2_lf_invoke_*`.

The generator also emits a self-contained colcon
package skeleton (CMakeLists, startup glue, RID init, launch file). This is the
canonical path.

The generated code targets the **CCpp** platform (reactor-c runtime with C++ reaction bodies).

## Quick Start

### Docker (recommended)

```bash
# Full pipeline: extract + generate
docker compose --profile lf run --rm analyzer
docker compose run --rm dev bash -c "cd packages && colcon lf-generate"

# Or inside dev container
docker compose run --rm dev
cd packages
colcon build --symlink-install
source install/setup.bash
colcon lf-analyze                    # Extract system.json files
colcon lf-generate      # Generate .lf files from them
```

### Local (with ROS 2 Jazzy)

```bash
cd packages
colcon lf-generate                          # All packages
colcon lf-generate --package pubsub_example  # Single package (all its launch stems)
```

### From a specific system.json

```bash
colcon lf-generate --system-json path/to/system.json --output-dir output/lf/
```

## CLI Options

| Option | Default | Description |
|--------|---------|-------------|
| `--system-json` | auto-discover | Path to a specific `system.json` file |
| `--output-dir` | `lf/` next to `system.json` | Override output directory |
| `--workspace` | auto-detect | Colcon workspace root |
| `--after-delay` | `0` | Default [logical-time offset](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement) on inter-node connections. Precedence, highest first: a per-link `after_delay_ns` in `system.json`, this flag, `execution.yaml`'s `defaults.after_delay` (see [Execution Configuration](../features/execution-config.md#the-after-delay-default)), then the `0` default (`after 0`, one microstep) |
| `--target-lfc` | off | Deprecated no-op: the lfc package is always emitted |
| `--federate {per-process,per-node,<path>.yaml}` | off | Generate federated artifact |
| `--emit-federation-template` | off | Write a starter `federation.yaml` next to each `system.json` |
| `--force` | off | Allow `--emit-federation-template` to overwrite an existing file |

See [Federation](../features/federation.md) for full details on the `--federate` flag.

## Output Structure

Generated files are placed next to the source `system.json`:

```text
packages/lf-analysis/pubsub_example/bringup/
  system.json          # Input (from colcon lf-analyze)
  lf/
    Bringup.lf         # Generated Lingua Franca file
```

## Generated LF Structure

### Node Structure

Every node (regardless of whether it has explicit callback groups or not) emits a **nested reactor structure**: an outer shell reactor holds external ports and a `startup` reaction, while all user reactions live inside per-group sub-reactors.

For a node that uses only the implicit default group, the generator synthesizes a `"default_group"` MutuallyExclusive group covering timers, subscribers, service servers, service clients, action servers, and action clients:

```lf
reactor PublisherNode {
    output sensor_data: SensorData
    default_group = new PublisherNode_DefaultGroup()
    default_group.sensor_data -> sensor_data
    reaction(startup) {= ros2_lf_bind_node(self, "PublisherNode"); =}
}

reactor PublisherNode_DefaultGroup {
    output sensor_data: SensorData
    timer t_timer_callback(0, 500 msec)

    reaction(t_timer_callback) -> sensor_data {=
        lf_print("timer_callback triggered");
    =}
}
```

- **Timers** → LF `timer` declarations with the extracted period and offset 0. `execution.yaml` can set a start offset or a period per timer, see [Timer start offsets and periods](../features/execution-config.md#timer-start-offsets-and-periods)
- **Subscribers** → LF `input` ports
- **Publishers** → LF `output` ports
- **Callbacks** → LF `reaction` blocks triggered by timers or input ports

### Node with Explicit Callback Groups

Nodes with multiple explicit callback groups (MutuallyExclusive or Reentrant) produce additional sub-reactors, one per named group:

```lf
reactor CallbackGroupNode {
    input chatter: String
    exclusive_group = new CallbackGroupNode_ExclusiveGroup()
    reentrant_group = new CallbackGroupNode_ReentrantGroup()
    chatter -> exclusive_group.chatter
}

reactor CallbackGroupNode_ExclusiveGroup {
    input chatter: String
    timer t_timer_a_callback(0, 500 msec)

    reaction(t_timer_a_callback) {=
        lf_print("timer_a_callback triggered");
    =}

    reaction(chatter) {=
        lf_print("chatter_callback triggered");
    =}
}
```

Each callback group becomes a sub-reactor. Port forwarding connections wire the outer ports to the appropriate sub-reactor.

### Main Reactor

The main reactor instantiates all nodes and creates inter-node connections:

```lf
main reactor Bringup {
    publisher_node = new PublisherNode()
    subscriber_node = new SubscriberNode()

    publisher_node.sensor_data -> subscriber_node.sensor_data after 5 msec
}
```

- Instance names come from the launch-level node names (sanitized to valid LF identifiers)
- Connections use `after` delays (the examples on this page pass an explicit
  5 msec. Absent a per-link value the emitted default is `after 0`, see
  `--after-delay` above) as a
  [declared logical-time offset (see chapter)](time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement),
  not a latency measurement

### Service Reactions

Async-callback service clients map to a 3-reaction shape, one client send, one
server handler, one client response lambda:

| Endpoint | Ports emitted | Reaction body |
|----------|---------------|---------------|
| Server | `input <svc>_req: stub_t`, `output <svc>_resp: stub_t` | once per counted request: `ros2_lf_invoke_srv_server(...);` and one write added to `<svc>_resp` |
| Client (timer that calls `async_send_request`) | `output <svc>_req: stub_t` | `ros2_lf_invoke_srv_send(...);` and one write added to `<svc>_req` |
| Client (response lambda) | `input <svc>_resp: stub_t` | once per counted response: `ros2_lf_invoke_srv_client_cb(...);` |

Connections in the main reactor:

```lf
client.<svc>_req  -> server.<svc>_req  after 0
server.<svc>_resp -> client.<svc>_resp after 0
```

With the default `after 0`, the server handles a request one microstep after it was sent
and the response lambda runs one microstep later, at the same logical time.

The classifier that decides whether a client uses the async-callback or
blocking-future style checks for the async pattern first, so a lambda that
calls `future.get()` inside `async_send_request` is still classified as async.

### Action Reactions

Every action channel is a counted port: goal, result and cancel requests,
goal, result and cancel responses, and feedback. The reaction that ran the sending
callback claims the sends the action gate counted on each channel and raises that
channel's port by the count, so a port fires only for a real send. The receiving
reaction takes exactly that many samples. Goal request and goal response use the link
`after` delay, every other channel a fixed 1 ms (`ACTION_ASYNC_AFTER_DELAY_NS` in
`lf_generator.py`). The reaction table and the claims of each reaction are on the
[action example](../examples/action-example.md#generated-lf-lfc-path) page.

## Name Handling

| ROS 2 Concept | LF Mapping | Example |
|-------------|-----------|---------|
| Node class name | Reactor name (PascalCase) | `TimerPublisherNode` → `TimerPublisherNode` |
| Node launch name | Instance name (sanitized) | `/robot_1/publisher_node` → `robot_1_publisher_node` |
| Topic name | Port name (last segment) | `/sensor_data` → `sensor_data` |
| Service / action name | Port stem (full declared name; a stem two names share stops generation) | `/rebind/fib` → `rebind_fib_goal_req` |
| Timer callback | Timer name | `timer_callback` → `t_timer_callback` |
| Message type | Port type (short name) | `std_msgs::msg::String` → `String` |
| Launch file stem | Main reactor name | `bringup.launch.py` → `Bringup` |

When the main reactor name collides with a reactor class name, `System` is appended (e.g., `TimerNode` → `TimerNodeSystem`).

## Limitations

- **Blocking service clients** (`spin_until_future_complete`, `future.get()`, or a
  `spin_some`/`spin_once`, condition variable or sleep loop until the response
  callback runs) are
  refused at generation with RCLLF-H009. The response arrives as a later
  LF event, so a wait inside the callback timed out on every call.
- **Action cancel** is delivered by the 20 ms action poll (`action_cancel_server`
  and `action_cancel_cb` reactions), in one process and across federates.
  A later version replaces the poll with an event.
- **Multi-action-server nodes**, the binding falls back to accept-any because
  `rclcpp_action` does not yet expose `get_action_name()` publicly.
- **No parameter mapping.** ROS 2 parameters are not converted to LF reactor parameters.
- **Single-file output.** All reactor definitions go into one `.lf` file per launch system.
- **Port types are symbolic.** All ports carry `stub_t`
  (a `uint8_t`), presence triggers the downstream reaction. The actual message
  is delivered by DDS.

## Generator internals

Generation runs in three stages: `lf_generator.py` converts the `SystemGraph`
into the `LFSystem` intermediate model, `lf_emitter.py` renders that model to
`.lf` source text, and `verb/generate.py` is the CLI verb that discovers
`system.json` files and orchestrates the run. The intermediate model
(`lf_model.py`) provides the `LFSystem`, `LFReactor`, `LFPort`, `LFTimer`,
`LFReaction`, `LFReactorInstance`, and `LFConnection` dataclasses. Naming
conventions are centralized in `lf_naming.py`. For where generation sits in
the overall pipeline, see [Architecture](architecture.md).

# Your First LF Run

This page takes the `pubsub_example` package from the [Quickstart](index.md) and
runs it under the Lingua Franca runtime, opening the file that each step
produces so you can see what rcllf worked out about your code. Run it inside the
dev container. Every excerpt below is trimmed output from an actual run, with
`...` where lines were cut; your timestamps will differ.

## 1. Source the workspace

The dev container built the runtime and the example packages when it started.
Every command below runs from `packages/`, the colcon workspace:

```bash
cd packages
source install/setup.bash
```

## 2. Analyze: what rcllf reads out of your source

```bash
colcon lf-analyze --package pubsub_example
```

The analyzer prints nothing for about half a minute while it reads the sources, then:

```text
==> Found 4 launch file(s) to analyze

--- pubsub_example/bringup.launch.py ---
  2 nodes, 1 topics, 0 services, 0 actions
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/
...
==> All 4 launch file(s) analyzed successfully
```

The analyzer found the package's four launch files, expanded each (including the
launch files `bringup.launch.py` includes), and parsed the C++ of every node it
found. Each launch file gets its own output directory. `bringup/` is the interesting one because it holds both
nodes.

Open the topic that connects them, in
`lf-analysis/pubsub_example/bringup/system.json`:

```json
"communication": {
  "topics": [
    {
      "topic": "/sensor_data",
      "message_type": "custom_interfaces::msg::SensorData",
      "publishers": [
        "publisher_node"
      ],
      "subscribers": [
        "subscriber_node"
      ],
      ...
      "after_delay_ns": null
    }
  ],
  "services": [],
  "actions": []
}
```

Nobody wrote that down for rcllf. The topic name, its message type, and both
endpoints come from the `create_publisher` and `create_subscription` calls in
`src/publisher_node.cpp` and `src/subscriber_node.cpp`. `after_delay_ns` is
`null` because no logical delay has been chosen for this link, so the link
gets none. Step 3 shows what that means.

The node entry `nodes.publisher_node` in the same file records the timer:

```json
"timers": [
  {
    "period_ms": 500.0,
    "callback": "timer_callback",
    "callback_group": "default_group",
    "member_variable": "timer_",
    "created_in": "constructor",
    "destroyed_in": null,
    "offset_ms": 0.0,
    "callback_expr": null
  }
],
```

`period_ms: 500.0` is the `500ms` argument of the
`this->create_wall_timer(500ms, ...)` call in `PublisherNode`'s constructor, and
`callback: "timer_callback"` is the method bound to it. Both the timer and the
publisher landed in `default_group` because the node never creates a callback
group of its own. [Reference: Data Model](../reference/data-model.md) documents
every field.

## 3. Generate: the same system as a Lingua Franca program

```bash
colcon lf-generate --system-json lf-analysis/pubsub_example/bringup/system.json
```

```text
==> Found 1 system(s) to generate

--- lf-analysis/pubsub_example/bringup/system.json ---
  2 nodes, 1 topics
  -> lf-analysis/pubsub_example/bringup/exec_config.json
  -> lf-analysis/pubsub_example/bringup/resolved_model.json (claim_status=supported)
  -> lf-analysis/pubsub_example/bringup/lf/Bringup.lf
  -> lf-analysis/pubsub_example/bringup/lf/include/ros2_lf_reaction_ids.h
  -> lf-generated/lf_generated_pubsub_example_bringup/src/ros2_lf_reaction_ids_init.cpp
  -> lf-analysis/pubsub_example/bringup/rcllf_trace_manifest.json (sha256 ...)
  -> lf-generated/lf_generated_pubsub_example_bringup/ (lfc package)

==> All 1 system(s) generated successfully
```

The last line is a complete, buildable colcon package. Open its copy of the
program, `lf-generated/lf_generated_pubsub_example_bringup/lf/Bringup.lf` (the same
file as `lf-analysis/pubsub_example/bringup/lf/Bringup.lf`). A *reactor* is LF's
component (the counterpart of a node) and a *reaction* is a block of code the
runtime triggers, the counterpart of a callback. Here is the publisher node as you
wrote it, in `pubsub_example/src/publisher_node.cpp` (the launch file renames the node
`publisher` to `publisher_node`, the name in the logs):

```cpp
PublisherNode::PublisherNode(const rclcpp::NodeOptions & options)
: Node("publisher", options), count_(0)
{
  publisher_ = this->create_publisher<custom_interfaces::msg::SensorData>("sensor_data", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this));
  ...
}

void PublisherNode::timer_callback()
{
  auto message = custom_interfaces::msg::SensorData();
  ...
  ++count_;
  ...
  publisher_->publish(message);
}
```

And here is what rcllf generated for it:

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

Three things to notice. The 500 ms period from `system.json` is now
`timer t_timer_callback(0, 500 msec)`, an LF timer. The node became two nested
reactors, one per callback group, the outer `PublisherNode` and an inner
`PublisherNode_DefaultGroup` holding the callbacks that share that group. And
the reaction body does not reimplement your callback: `ros2_lf_invoke_timer`
calls the real compiled `PublisherNode::timer_callback`, so the code that runs
is still the C++ you wrote. After the call, `ros2_lf_corr_emit` counts what the
callback published on the topic and sends that count on the output port
`corr_e0`, so the subscriber's side knows which messages belong to this
reaction. The `ingress_wake` and `ingress_drain` actions (reactions cut from the
excerpt) take in work that arrives from outside the program, such as a service
response; this example has none.

`RID_PublisherNode_timer_callback` is the handle that makes that possible. It is
defined next door in `lf/include/ros2_lf_reaction_ids.h`:

```c
#define RID_PublisherNode_timer_callback 0
#define RID_SubscriberNode_topic_callback 1
#define RID_COUNT 2
```

One ID per callback in the system, two here, matching the one timer callback
and the one subscription callback in the original sources.

At the bottom of `Bringup.lf` the main reactor wires the two nodes together:

```lf
main reactor Bringup {
    publisher_node = new PublisherNode()
    subscriber_node = new SubscriberNode()

    publisher_node.corr_e0 -> subscriber_node.corr_e0
}
```

This one line replaces the DDS topic as the thing that decides order. The
message itself still travels through ROS 2; the connection carries the count from
`corr_e0`, and the subscriber's reaction delivers the matching messages to your
`topic_callback`. The connection has no `after` clause: the subscriber's reaction
runs at the same *logical time* as the publisher's, and LF runs it after the
publisher's reaction because one reads the port the other writes. Logical time
is LF's model of when things happen, which the runtime advances independently of
the wall clock. No delay is the default, because neither `system.json`'s
`after_delay_ns` (still `null`, see step 2) nor a `--after-delay` flag set one.
See [the `after`-delay contract](../design/time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement)
for what a delay means and [CLI Reference](../reference/cli.md#colcon-lf-generate)
for how to set one.

## 4. Build: hand the program to `lfc`

```bash
colcon build --packages-select lf_generated_pubsub_example_bringup
source install/setup.bash
```

The package's `CMakeLists.txt` runs the Lingua Franca compiler, `lfc`, over
`Bringup.lf`, compiles the node sources alongside it, and links everything
against `ros2_lf_binding`. What comes out is one executable named after the main
reactor:

```text
install/lf_generated_pubsub_example_bringup/lib/lf_generated_pubsub_example_bringup/Bringup
install/lf_generated_pubsub_example_bringup/share/lf_generated_pubsub_example_bringup/launch/lf_bringup.launch.py
```

Both nodes now live in that single binary, scheduled by the LF runtime, and the
generated package installs its own launch file for it. The build summary ends with
`1 package had stderr output: lf_generated_pubsub_example_bringup`: that is `lfc`
printing its `info:` progress lines on stderr, not an error.

## 5. Run: the same callbacks, in a determined order

```bash
ros2 lf launch pubsub_example bringup
```

This resolves to the launch file the previous step installed and runs the
equivalent direct command:

```bash
ros2 launch lf_generated_pubsub_example_bringup lf_bringup.launch.py
```

```text
[Bringup-1] [INFO] [1794837917.694428418] [lf_component_manager]: Found 2 non-component node(s) in registry
[Bringup-1] [INFO] [1794837917.694450858] [lf_component_manager]: Registered non-component node: PublisherNode
[Bringup-1] [INFO] [1794837917.694461978] [lf_component_manager]: Registered non-component node: SubscriberNode
...
[Bringup-1] [INFO] [1794837917.697163647] [lf_component_manager]: ros2_lf_runtime_wait_ready: all subscriptions matched; LF start time anchored
[Bringup-1] [INFO] [1794837917.697982296] [ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/sensor_data', take_mode=skip timeout_ms=100)
[Bringup-1] [INFO] [1794837917.697997026] [ros2_lf_binding]: Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)
...
[Bringup-1] [INFO] [1794837917.698272656] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [1794837917.699085036] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
...
[Bringup-1] [INFO] [1794837918.197757790] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[Bringup-1] [INFO] [1794837918.198174430] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
```

The program logs more `INFO` lines than shown (the topic match at startup, a
summary at the stop); the excerpt keeps the ones this page explains. Stop it with
Ctrl-C. On the stop, the LF runtime prints a few lines like these, which are
normal:

```text
[Bringup-1] WARNING: ---- There are 1 unprocessed future events on the event queue.
[Bringup-1] WARNING: ---- The first future event has timestamp (5500000000, 0) after start tag.
[Bringup-1] WARNING: ---- Cleared 1 unprocessed events from the event queue.
[Bringup-1] ---- Elapsed logical time (in nsec): 5,000,000,000
```

The "future event" is the next timer firing, which the stop made unnecessary.

Read the two `Bound` lines closely. `Bound timer RID 0` says that
`PublisherNode::timer_callback` now runs from the LF timer of step 3, not from the
ROS 2 executor. Its callback is unchanged, but the LF runtime decides when it runs.
That is the substitution the whole pipeline exists to make. The two lines also close
the loop on the header from step 3, reporting RID 0 and RID 1 against the callbacks
they came from.

The log lines themselves are the ones in your `RCLCPP_INFO` calls, unchanged, and the
pairing is what to watch: every `Publishing` is followed by the matching `I heard` at
the same logical time, every time, because the connection from step 3 orders the
subscriber's reaction after the publisher's, regardless of thread scheduling or DDS
delivery. Under a stock executor that interleaving depends on both. Here the program
fixes it.

!!! tip "`colcon lf` runs this pipeline in one command"
    `colcon lf --package pubsub_example` runs analyze, configure, generate and build
    in one go and produces the same `lf_generated_pubsub_example_bringup` package. The
    configure step writes a default `config/execution.yaml` next to each analysis; this
    tutorial generates without one, which gives the same defaults. Start it with
    `ros2 lf launch pubsub_example bringup`. See
    [CLI Reference](../reference/cli.md#colcon-lf-all-in-one).

## Next steps

- [Example Packages](../examples/index.md), the same pipeline against services,
  actions, lifecycle nodes, and composed components.
- [Design: Architecture](../design/architecture.md), how the analyzer and
  generator produce what you just read.
- [Bring Your Own Code](../guides/bring-your-own-code.md), run your own ROS 2
  package the same way.
- [Porting Guide](../guides/porting-guide.md), what to change when the analyzer
  cannot read a node.

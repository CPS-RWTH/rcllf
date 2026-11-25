# Porting Your ROS 2 System

This guide is for someone who already has a working ROS 2 system and wants to
convert it to Lingua Franca part by part: timers, publishers/subscriptions,
composable components, and launch files. Each section below takes a small
real pattern, shows what the analyzer extracts from it, what it becomes under
`colcon lf-generate`, and the hazards or limitations you're likely to hit.

Two related pages to have open alongside this one:

- **[Support Matrix](../features/support-matrix.md)** is the mechanically
  generated, CI-drift-checked truth about what's supported and what each
  diagnostic code means. When this guide says a pattern is "supported," that
  claim is sourced from (and must agree with) that table.
- **[Example Packages](../examples/index.md)** and its per-package pages are the
  worked "run the analysis, see the output" playground for every pattern
  named below, with the full `system.json` walkthrough.

If you'd rather see the whole analyze → configure → generate → build → run →
evaluate pipeline in one place first, start with
[Your First LF Run](../getting-started/first-lf-run.md) for the quickstart, then
[Bring Your Own ROS 2 Code](bring-your-own-code.md) for the stage-by-stage
walkthrough on your own package.

## Timers

### The ROS 2 pattern

A wall timer created with a literal chrono duration, from
`packages/src/timer_example/src/timer_node.cpp`:

```cpp
TimerNode::TimerNode(const rclcpp::NodeOptions & options):
Node("timer_node", options), count_(0)
{
  timer_ = this->create_wall_timer(1000ms, std::bind(&TimerNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Timer node started.");
}
```

### What the analyzer detects

`colcon lf-analyze` records the node, its timer, and the timer's period in
`system.json`. Because `1000ms` is a literal chrono duration, the period
resolves cleanly to a concrete millisecond value on the timer entry. There is
no hazard class for timers. `core.timers` is a `SUPPORTED` feature with no
diagnostic codes.

### What it becomes in LF

A wall timer callback converts directly to an LF `timer` declaration plus a
`reaction` triggered by it, inside the node's (synthesized, if there are no
explicit [callback groups](../design/callback-groups.md)) default-group
sub-reactor:

```lf
reactor TimerNode_DefaultGroup {
    timer t_timer_callback(0, 1000 msec)

    reaction(t_timer_callback) {=
        lf_print("timer_callback triggered");
    =}
}
```

This is a hand-written illustrative snippet matching the shape described in
[LF Code Generation](../design/lf-generation.md#node-structure), not literal
generator output. No `.lf` file is committed to the repo (generated output
is gitignored). Run the pipeline yourself (below) to see the real emission
for your node.

### Hazards & limitations

!!! warning "Parameter-derived timer periods are not statically resolvable"
    If the period comes from a ROS parameter instead of a literal (`declare_parameter("rate_hz", 200.0)` followed by constructing the
    duration from that parameter's value) `colcon lf-analyze`'s clang-based
    extraction cannot resolve it. The timer entry lands in `system.json` with
    `period_ms: null`, and `colcon lf-generate` has no period to emit.

    A parameter's *runtime* value is not visible to static source analysis.
    The workaround is a small script step between `lf-analyze` and
    `lf-generate` that writes the period (from your parameter file, for
    example) into the null `period_ms` of a **copy** of `system.json`, and
    generates from that copy with `--system-json`. Patch the copy, never the
    analysis output itself, so the next `lf-analyze` does not undo it.

### Verify it worked

```bash
cd packages
colcon lf-analyze --package timer_example --launch-file timer_node.launch.py --format json
```

Check `lf-analysis/timer_example/timer_node/system.json` for a timer entry
with a non-null `period_ms`. Then:

```bash
colcon lf-configure --system-json lf-analysis/timer_example/timer_node/system.json
colcon lf-generate --system-json lf-analysis/timer_example/timer_node/system.json
```

Expect a generated colcon package skeleton (CMakeLists, startup glue,
launch file) alongside the `system.json`, with a `.lf` file containing the
`timer` declaration described above.

## Publishers & subscriptions

### The ROS 2 pattern

A publisher/subscriber pair from `packages/src/pubsub_example/src/`. The
publisher (`publisher_node.cpp`):

```cpp
publisher_ = this->create_publisher<custom_interfaces::msg::SensorData>("sensor_data", 10);
timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this));
```

The subscriber (`subscriber_node.cpp`):

```cpp
subscription_ = this->create_subscription<custom_interfaces::msg::SensorData>(
  "sensor_data", 10, std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));
```

wired together by `packages/src/pubsub_example/launch/bringup.launch.py`,
which includes both nodes' individual launch files.

### What the analyzer detects

`colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py`
produces a `system.json` with two node entries and one `TopicConnection` on
`"sensor_data"` linking the publisher's output to the subscriber's input,
matched by resolved topic name. See
[Example Packages: Pubsub Example](../examples/pubsub-example.md) for the full
`system.json` shape.

### What it becomes in LF

A subscription callback converts to an LF `reaction` triggered by an input
port, fed from the publisher's output port via a main-reactor connection:

```lf
main reactor Bringup {
    publisher_node = new PublisherNode()
    subscriber_node = new SubscriberNode()

    publisher_node.sensor_data -> subscriber_node.sensor_data after 5 msec
}
```

Again a hand-written illustrative snippet, see
[LF Code Generation: Main Reactor](../design/lf-generation.md#main-reactor)
for the full generation model, including the `after` delay's meaning as a
declared logical-time offset, not a latency measurement.

### Hazards & limitations

Base publish/subscribe (`core.pubsub`) has no hazard of its own, but the
QoS-adjacent patterns you're likely to have in a real subscription do:

!!! warning "QoS and topic-name hazards"
    `colcon lf-generate` checks each subscription's diagnostics before
    emitting code. Each hazard has a code (`RCLLF-H000` to `RCLLF-H004`,
    one per kind of risky subscription setup, such as an unreliable
    input or a shallow queue). `RCLLF-H000`, `H001`, `H002`, and `H004` **warn**.
    `RCLLF-H003` (`message_filters` outside the admitted two-input
    ExactTime or ApproximateTime form) **rejects generation outright**.
    Any code can be accepted after review by listing it (per code,
    for the whole system) in `acknowledged_hazards`, for H003 that is the
    only way to make generation proceed. See
    [Execution Configuration: Hazard diagnostics](../features/execution-config.md#hazard-diagnostics)
    for what each code means and why, and the
    [Support Matrix](../features/support-matrix.md) for the authoritative,
    mechanically generated list.

    In short: a `best_effort` subscription, a depth-1 queue with an in-model
    publisher, an unparseable QoS profile, or a dynamically computed topic
    name will all still generate (with a warning you should read and either
    fix or acknowledge). A two-input `message_filters` synchronizer with an
    ExactTime or ApproximateTime policy generates without acknowledgement when
    both inputs come from publishers inside the converted launch with one link
    delay; an input from outside it refuses with `RCLLF-H003
    sync_input_legacy:external` (see
    [message_filters Synchronizers](../features/message-filters.md)).
    Any other `message_filters` use will not generate until you rewrite it
    to that form, redesign it in terms of explicit LF `after` delays and
    multi-input reactions, or acknowledge the hazard after review.

### Verify it worked

```bash
cd packages
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py --format json
colcon lf-configure --system-json lf-analysis/pubsub_example/bringup/system.json --show
colcon lf-generate --system-json lf-analysis/pubsub_example/bringup/system.json
```

Expect `lf-analysis/pubsub_example/bringup/system.json` with 2 nodes and 1
topic connection. `lf-configure --show` should resolve without error (no
hazards on the stock example). `lf-generate` should produce the generated
package skeleton with no rejected-generation error. If you're working from a
copy that adds a QoS hazard, expect a warning printed for H000/H001/H002/H004
or a hard failure for H003 (a `message_filters` form outside the admitted
one) until acknowledged.

## Components

### The ROS 2 pattern

A composable node from `packages/src/component_example/src/component_node.cpp`, a timer plus a publisher, registered as a component:

```cpp
ComponentNode::ComponentNode(const rclcpp::NodeOptions & options):
Node("component_node", options), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("component_output", 10);
  timer_ = this->create_wall_timer(1s, std::bind(&ComponentNode::timer_callback, this));
}
// ...
RCLCPP_COMPONENTS_REGISTER_NODE(component_example::ComponentNode)
```

### What the analyzer detects

Structurally this is the same as any other node with a timer and a
publisher, `colcon lf-analyze` extracts the timer and publisher entries the
same way it would for a non-composable node. `RCLCPP_COMPONENTS_REGISTER_NODE`
is a packaging detail the extractor is aware of but which doesn't change the
node's entry shape. See
[Example Packages: component_example](../examples/component-example.md).

### What it becomes in LF

Conversion is identical to a plain node. The reactor you get for
`ComponentNode` looks exactly like the reactor you'd get if it were launched
as a standalone executable with the same timer and publisher.

Where components actually differ is **how the generated system starts**, not
how it's converted, see [Launch files](#launch-files) below and the
[Features Overview](../features/index.md) launch-generation row, which notes
that component nodes rely on `ros2 lf launch` rather than a plain generated
ROS 2 launch file.

### Hazards & limitations

None registered. `core.components` is a `SUPPORTED` feature with no
diagnostic codes, per the [Support Matrix](../features/support-matrix.md).

### Verify it worked

```bash
cd packages
colcon lf-analyze --package component_example --launch-file component_node.launch.py --format json
colcon lf-configure --system-json lf-analysis/component_example/component_node/system.json
colcon lf-generate --system-json lf-analysis/component_example/component_node/system.json
```

Expect the same generated-package shape as any other node: a reactor for
`ComponentNode` with a timer and an output port, no hazard diagnostics.

## Launch files

### The ROS 2 pattern

`packages/src/system_bringup/launch/system_bringup.launch.py` is the
strongest worked example of a non-trivial launch hierarchy: it declares
`namespace` and `use_sim` launch arguments, pushes `pubsub_example` and
`timer_example` under a `robot_1` namespace via `PushRosNamespace` inside a
`GroupAction`, includes `service_example` at root level (no namespace),
passes a `system_params.yaml` override to `parameter_example`, and
conditionally includes a `sim_timer` node gated on `IfCondition(LaunchConfiguration('use_sim'))`.

### What the analyzer detects

- **Namespaced groups**: nodes under `robot_1` get topic names resolved with
  the namespace prefix (e.g. `/robot_1/sensor_data`).
- **Root-level includes**: `service_example` and `parameter_example` stay
  unnamespaced.
- **Conditional inclusion**: a condition is evaluated against the launch
  configurations where it occurs, as `ros2 launch` does. With `use_sim`
  defaulting to `false`, `sim_timer` is not in `system.json`; the analyzer
  reads declared defaults, so change the default to get it. A condition the analyzer cannot
  evaluate keeps the node and prints a WARNING that the generated system
  assumes it starts.
- **Parameter file overrides**: `system_params.yaml` is parsed and attached
  to the `parameter_example` node's entry.
- **Launch arguments**: `namespace` (default `robot_1`) and `use_sim`
  (default `false`) are recorded as `LaunchArgument` entries.

This is already fully narrated with the actual `system.json` shape in
[Example Packages: system_bringup](../examples/system-bringup.md), reuse that
page rather than re-deriving the extraction here.

### What it becomes in LF

Launch-file handling is generator behavior, not a hazard-diagnostic class,
there's no dedicated feature-registry module for it the way there is for
timers or pubsub. The generator classifies extracted nodes into
component-group vs. non-component-node buckets and emits one
`lf_generated_<package>_<name>` package whose single binary loads both kinds, so the
generated system starts the way the original did.

The one change that matters for a porter: **the converted system does not
start via your original `ros2 launch` file.** It starts via

```bash
ros2 lf launch <package> <launch-stem>
```

(e.g. `ros2 lf launch system_bringup system_bringup`) which is the `ros2lf`
CLI's own verb, not a generated ROS 2 launch file, and resolves to the
generated launch artifact from the `lf-generate` step
internally. Use `ros2 lf list` to see what's available to launch after
generating.

### Hazards & limitations

None directly. A launch file itself carries no diagnostic. It inherits
whatever hazards its contained nodes carry (see
[Publishers & subscriptions](#publishers-subscriptions) above).

### Verify it worked

```bash
cd packages
colcon lf-analyze --package system_bringup --launch-file system_bringup.launch.py --format json
colcon lf-configure --system-json lf-analysis/system_bringup/system_bringup/system.json --show
colcon lf-generate --system-json lf-analysis/system_bringup/system_bringup/system.json
ros2 lf list
```

Expect `lf-analysis/system_bringup/system_bringup/system.json` with 6+ node
entries, namespace-prefixed topic names under `robot_1`, no `sim_timer`
entry (`use_sim` defaults to `false`), and parsed content from `system_params.yaml` under
`config/`. After generation, `ros2 lf list` should show the generated system
as launchable. Starting it uses `ros2 lf launch system_bringup system_bringup`
instead of `ros2 launch system_bringup system_bringup.launch.py`.

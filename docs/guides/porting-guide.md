# Porting Your ROS 2 System

This guide is for someone who already has a working ROS 2 system and wants to
convert it to Lingua Franca part by part: timers, publishers/subscriptions,
composable components, and launch files. Each section below takes a small
real pattern, shows what the analyzer extracts from it, what it becomes under
`colcon lf-generate`, and the hazards or limitations you're likely to hit.

Run the commands in the dev container (`docker compose run --rm dev`, see
[Installation](../getting-started/installation.md)). Its first start builds only the runtime
and the learning-path examples; a section that needs more packages says so. A generated
package is named `lf_generated_<package>_<system>`, for example
`lf_generated_timer_example_timernodesystem`. The exact names are the directories under
`packages/lf-generated/`, and the rule is in the [CLI reference](../reference/cli.md).

Two related pages to have open alongside this one:

- **[Support Matrix](../features/support-matrix.md)** is the generated list of what
  is supported, what warns and what is refused. When this guide says a pattern is
  "supported," that claim agrees with that table. What each diagnostic code means is
  in [Diagnostic Codes](../reference/diagnostic-codes.md).
- **[Example Packages](../examples/index.md)** and its per-package pages are the
  worked "run the analysis, see the output" playground for every pattern
  named below, with the full `system.json` walkthrough.

If you'd rather see the whole analyze → configure → generate → build → run →
evaluate pipeline in one place first, start with
[Your First LF Run](../getting-started/first-lf-run.md) for the first tutorial, then
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
resolves cleanly to a concrete millisecond value on the timer entry. Wall timers are a supported feature with no diagnostic codes (see the
[Support Matrix](../features/support-matrix.md)).

### What it becomes in LF

A wall timer callback converts directly to an LF `timer` declaration plus a
`reaction` triggered by it, inside the node's (synthesized, if there are no
explicit [callback groups](../design/callback-groups.md)) default-group
sub-reactor:

```lf
reactor TimerNode_DefaultGroup {
    input rcllf_bound: stub_t
    timer t_timer_callback(0, 1000 msec)
    ...
    reaction(t_timer_callback) rcllf_bound {=
        ros2_lf_invoke_timer(self, RID_TimerNode_timer_callback);
    =}
    ...
}
```

This is an excerpt of the generated `TimerNodeSystem.lf`; `...` marks lines left out (the
startup and ingress reactions every node gets). The reaction body does not copy your callback
into the LF program. It asks the runtime binding to call your original `timer_callback`.
See [LF Code Generation](../design/lf-generation.md#node-structure). The generated file is
gitignored; the verify step below creates it.

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

Expect two places. The LF program is
`lf-analysis/timer_example/timer_node/lf/TimerNodeSystem.lf`, with the
`timer t_timer_callback(0, 1000 msec)` declaration described above. The generated colcon
package is `lf-generated/lf_generated_timer_example_timernodesystem/` (`CMakeLists.txt`,
`package.xml`, `src/`, `launch/`). Its name ends in `timernodesystem` because the launch file
stem `timer_node`, without underscores, equals the name of the node class `TimerNode`; then
`system` is appended.

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
produces a `system.json` with two node entries and one entry under
`communication.topics` for the topic `sensor_data`, listing the publisher as its
publisher and the subscriber as its subscriber, matched by resolved topic name. See
[Example Packages: Pubsub Example](../examples/pubsub-example.md) for the full
`system.json` shape.

### What it becomes in LF

A subscription callback converts to an LF `reaction` triggered by an input
port, fed from the publisher's output port via a main-reactor connection:

```lf
main reactor Bringup {
    publisher_node = new PublisherNode()
    subscriber_node = new SubscriberNode()

    publisher_node.corr_e0 -> subscriber_node.corr_e0
}
```

This is the main reactor of the generated `Bringup.lf`. `corr_e0` is the port that carries
the connection's publish counts ([publication correspondence](../design/time-and-coordination.md#publication-correspondence-topic-edges));
the connection has no `after` delay unless you configure one. See
[LF Code Generation: Main Reactor](../design/lf-generation.md#main-reactor) for the generation
model, including the `after` delay's meaning as a declared logical-time offset, not a latency
measurement.

### Hazards & limitations

Base publish/subscribe has no hazard of its own, but the
QoS-adjacent patterns you're likely to have in a real subscription do:

!!! warning "QoS and topic-name hazards"
    `colcon lf-generate` checks each subscription's diagnostics before
    emitting code. Each hazard has a code, `RCLLF-H` and a number, and the catalog
    [Diagnostic Codes](../reference/diagnostic-codes.md) gives the severity, the meaning and
    the fix of each one. The ones a subscription is likely to hit:

    - [`RCLLF-H000`](../reference/diagnostic-codes.md#rcllf-h000) (unknown QoS reliability)
      is information only.
    - [`RCLLF-H001`](../reference/diagnostic-codes.md#rcllf-h001) (`best_effort` input),
      [`H002`](../reference/diagnostic-codes.md#rcllf-h002) (depth-1 queue with a publisher in
      the system) and [`H004`](../reference/diagnostic-codes.md#rcllf-h004) (topic name computed
      at run time) warn: the system still generates. H001 and H002 print as information when
      the connection is admitted for correspondence.
    - [`RCLLF-H003`](../reference/diagnostic-codes.md#rcllf-h003) (`message_filters` outside the
      admitted two-input form) is an error: generation refuses.

    An error can be accepted after review by listing its code in `acknowledged_hazards` in
    `execution.yaml` (per code, for the whole system), and for H003 that is the only way to make
    generation proceed. Acknowledging skips the check only; the problem stays in the generated
    program, and for some codes the affected callback then never fires. Each catalog entry says
    whether its code can be acknowledged. Warnings and information cannot, and need no
    acknowledgement. See
    [`acknowledged_hazards`](../features/execution-config.md#acknowledged_hazards) for the key
    and [Configure Execution](configure-execution.md) for editing the file.

    A two-input `message_filters` synchronizer with an ExactTime or ApproximateTime policy
    generates without acknowledgement when both inputs come from publishers inside the
    converted launch with one link delay; an input from outside it refuses with
    `RCLLF-H003 sync_input_legacy:external` (see
    [message_filters Synchronizers](../features/message-filters.md)). Any other
    `message_filters` use will not generate until you rewrite it to that form, redesign it in
    terms of explicit LF `after` delays and multi-input reactions, or acknowledge the hazard
    after review.

### Verify it worked

```bash
cd packages
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py --format json
colcon lf-configure --system-json lf-analysis/pubsub_example/bringup/system.json --show
colcon lf-generate --system-json lf-analysis/pubsub_example/bringup/system.json
```

Expect `lf-analysis/pubsub_example/bringup/system.json` with 2 nodes and 1
topic. `lf-configure --show` should resolve without error (no
hazards on the stock example). `lf-generate` should produce the generated
package skeleton with no rejected-generation error. If you're working from a
copy that adds a QoS hazard, expect a line for H000, H001, H002 or H004
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
is a packaging detail that does not change the node's entry shape. See
[Example Packages: component_example](../examples/component-example.md).

### What it becomes in LF

Conversion is identical to a plain node. The reactor you get for
`ComponentNode` looks exactly like the reactor you'd get if it were launched
as a standalone executable with the same timer and publisher.

Where components actually differ is **how the generated system starts**, not
how it's converted: the generated launch file loads them through a component manager. See
[Launch files](#launch-files) below and the
[Features Overview](../features/index.md).

### Hazards & limitations

None. Composable components are a supported feature with no diagnostic codes, per the
[Support Matrix](../features/support-matrix.md).

### Verify it worked

`component_example` is not in the first start's package set. Start the container with
it, then run the steps:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="component_example" dev
```

```bash
cd packages
colcon lf-analyze --package component_example --launch-file component_node.launch.py --format json
colcon lf-configure --system-json lf-analysis/component_example/component_node/system.json
colcon lf-generate --system-json lf-analysis/component_example/component_node/system.json
```

Expect the same generated-package shape as any other node: a reactor for
`ComponentNode` with a timer and an output port, and no hazard diagnostics. The LF program is
`lf-analysis/component_example/component_node/lf/ComponentNodeSystem.lf`, with
`timer t_timer_callback(0, 1000 msec)` and `output component_output`; the package is
`lf-generated/lf_generated_component_example_componentnodesystem/`. To run it, build and
launch it:

```bash
colcon lf-build --package component_example
source install/setup.bash
ros2 lf launch component_example component_node
```

The log shows `Loaded component_example::ComponentNode`, then `Bound timer RID 0
(ComponentNode::timer_callback, period 1000000000 ns)` and one `Publishing: 'Component tick: N'`
line per second.

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
- **Parameter files**: the launch file passes `system_params.yaml` to `parameter_example`
  as `params_file`, but `parameter_node.launch.py` declares no such argument, so the file is
  not applied. `system.json` lists only
  `src/parameter_example/config/params.yaml` under `parameter_files`, and the generated
  program logs `robot_name: ros2_lf_robot` (from that file), not the `system_params.yaml`
  value. A launch file that does consume its parameter file is analyzed as `ros2 launch` would
  evaluate it.
- **Launch arguments**: `namespace` (default `robot_1`), `use_sim` (default `false`) and the
  other arguments are recorded under `launch_graph.root.arguments` in `system.json`.

This is already fully narrated with the actual `system.json` shape in
[Example Packages: system_bringup](../examples/system-bringup.md), reuse that
page rather than re-deriving the extraction here.

### What it becomes in LF

A launch file is not a hazard class. The generator sorts the extracted nodes into component
nodes and plain nodes and emits one `lf_generated_<package>_<system>` package for the launch
file, here `lf_generated_system_bringup_systembringup` (the launch stem `system_bringup`
without underscores equals no node class, so no `system` is appended; the rule is in the
[CLI reference](../reference/cli.md)). Its single binary loads both kinds of node, so the
generated system starts the way the original did.

The one change that matters for a porter: **you start the converted system with
`ros2 lf launch`, not with your original launch file.**

```bash
ros2 lf launch <package> <launch-stem>
```

For example `ros2 lf launch system_bringup system_bringup`. `<launch-stem>` is the launch file
name without `.launch.py`, as `ros2 lf list` prints it. The command finds the generated package
and starts its generated launch file (`lf_systembringup.launch.py` here), which also works with
a plain `ros2 launch`. `ros2 lf list` shows what is available; a system that is generated but
not built yet is listed with `colcon lf-build --package <package>` as the next step.

### Hazards & limitations

None directly. A launch file itself carries no diagnostic. It inherits
whatever hazards its contained nodes carry (see
[Publishers & subscriptions](#publishers-subscriptions) above).

### Verify it worked

`system_bringup` and the `parameter_example` it includes are not in the first start's package
set. Start the container with both:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup parameter_example" dev
```

```bash
cd packages
colcon lf-analyze --package system_bringup --launch-file system_bringup.launch.py --format json
colcon lf-configure --system-json lf-analysis/system_bringup/system_bringup/system.json --show
colcon lf-generate --system-json lf-analysis/system_bringup/system_bringup/system.json
colcon lf-build --package system_bringup
source install/setup.bash
ros2 lf list
```

Expect `lf-analysis/system_bringup/system_bringup/system.json` with 6 node entries
(`/robot_1/publisher_node`, `/robot_1/subscriber_node`, `/robot_1/timer_node`,
`/service_example/service_server`, `/service_example/service_client`, `parameter_node`), the
topic `/robot_1/sensor_data` with the `robot_1` prefix, and no `sim_timer` entry (`use_sim`
defaults to `false`). Before `colcon lf-build`, `ros2 lf list` shows the system with
`- (run: colcon lf-build --package system_bringup)` in the last column; after it, the column
says `ready`:

```text
Package                  Launch Stem                  Nodes  Type         Launch Ready
system_bringup           system_bringup                   6  nodes        ready
```

Start it with `ros2 lf launch system_bringup system_bringup` instead of
`ros2 launch system_bringup system_bringup.launch.py`. The log shows
`Registered non-component node:` for each of the six node classes and
`admitted edge 0 PublisherNode -> SubscriberNode on '/robot_1/sensor_data'`.

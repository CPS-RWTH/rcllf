# timer_example

The simplest ROS 2 node: one wall timer that fires once per second. Start here to see
how a ROS 2 timer becomes a Lingua Franca (LF) timer.

## What it shows

- **Node:** `TimerNode` extends `rclcpp::Node`.
- **Timer:** 1 s period, created with `create_wall_timer`.
- **Callback:** `timer_callback()`, which logs a counter.
- No publishers, subscribers, services or actions.

## Run with stock ROS 2

The first start of the dev container builds this package, so no package selection is
needed:

```bash
docker compose run --rm dev ros2 launch timer_example timer_node.launch.py
```

The first tick comes one period after the node starts, then one per second (trimmed;
your timestamps differ):

```text
[timer_node-1] [INFO] [...] [timer_node]: Timer node started.
[timer_node-1] [INFO] [...] [timer_node]: Timer tick: 0
[timer_node-1] [INFO] [...] [timer_node]: Timer tick: 1
[timer_node-1] [INFO] [...] [timer_node]: Timer tick: 2
```

Stop it with Ctrl+C.

## Run under rcllf

Inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package timer_example && source install/setup.bash
ros2 lf launch timer_example timer_node
```

`ros2 lf launch` names the generated package it starts. Here the launch stem
`timer_node` becomes `timernode`, which equals the node class name, so `system` is
appended ([Generated package names](../reference/cli.md#generated-package-names)):

```text
Launching LF-controlled system: timer_example/timer_node
  Package: lf_generated_timer_example_timernodesystem
  Launch file: lf_timernodesystem.launch.py
...
[TimerNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (TimerNode::timer_callback, period 1000000000 ns)
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 0
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 1
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 2
```

`Bound timer RID 0` means the timer callback is now LF reaction 0
([RID](../design/glossary.md#rid-reaction-id)): the LF scheduler, not the ROS 2
executor, decides when it runs.

## What to look at

- **The first tick.** Under rcllf tick 0 comes at once, at the program's start tag;
  under stock ROS 2 it comes one period after the node starts. To get the stock
  timing, give the timer a start offset equal to its period in
  `lf-analysis/timer_example/timer_node/config/execution.yaml` and run
  `colcon lf --package timer_example` again:

    ```yaml
    schema_version: 1
    nodes:
      timer_node:
        timers:
          timer_callback:
            offset: "1 sec"
    ```

    The generated `.lf` file then declares `timer t_timer_callback(1000 msec, 1000 msec)`,
    and tick 0 comes one second after the `Bound timer RID 0` line, as under stock
    ROS 2. The node key is the key of the node in `system.json`, the timer key is its
    callback name. See
    [Timer start offsets](../features/execution-config.md#timer-start-offsets-and-periods).

- **What the analyzer read.** `lf-analysis/timer_example/timer_node/system.json`
  holds one node, `timer_node`, with `class_name` `TimerNode` and one entry under
  `timers`: `period_ms` 1000.0 and `callback` `timer_callback`. Its publisher,
  subscriber, service and action lists are empty.

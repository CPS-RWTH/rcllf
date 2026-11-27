# Configure Execution

This guide changes how one generated system runs by editing its `execution.yaml`. It
delays a timer, fixes the worker count, makes a subscription wait for its message,
and undoes the change again. Every key and rule is listed in the
[execution.yaml Reference](../features/execution-config.md).

Run the commands in the dev container (`docker compose run --rm dev`), in
`/workspace/packages`. The first start builds `timer_example` and `pubsub_example`.

## 1. Generate the system and find the file

```bash
colcon lf --package timer_example --timeout "3 sec"
```

The second step of the pipeline, `colcon lf-configure`, writes the configuration
files next to the analysis:

```text
============================================================
Step 2/4: colcon lf-configure (scaffold)
============================================================
==> package 'timer_example': 1 analysis [timer_node]
--- /workspace/packages/lf-analysis/timer_example/timer_node/system.json ---
  -> /workspace/packages/lf-analysis/timer_example/timer_node/config/execution.reference.yaml
```

```bash
ls lf-analysis/timer_example/timer_node/config/
cat lf-analysis/timer_example/timer_node/config/execution.yaml
```

```text
execution.reference.yaml
execution.yaml
```

```yaml
schema_version: 1
# See execution.reference.yaml (regenerated every `colcon lf-configure` run) for every
# node/subscription and its effective default. This file is sparse: only keys you
# set here override the built-in default. Presence = set, absence = inherit.
defaults: {}
nodes: {}
subscriptions: {}
acknowledged_hazards: []
```

`execution.yaml` is the file you edit. It starts empty, so every value is the built-in
default. `execution.reference.yaml` lists every node and subscription of the system
with its current value. It is rewritten on every run, so do not edit it.

## 2. Run it unchanged

```bash
ros2 lf launch timer_example timer_node
```

```text
[TimerNodeSystem-1] [INFO] [1795774263.000068486] [timer_node]: Timer tick: 0
[TimerNodeSystem-1] [INFO] [1795774263.999551973] [timer_node]: Timer tick: 1
[TimerNodeSystem-1] [INFO] [1795774264.999550688] [timer_node]: Timer tick: 2
[TimerNodeSystem-1] [INFO] [1795774265.999568851] [timer_node]: Timer tick: 3
[TimerNodeSystem-1] ---- Start execution on Fri Nov 27 10:11:02 2026 ---- plus 999320026 nanoseconds
[TimerNodeSystem-1] ---- Elapsed logical time (in nsec): 3,000,000,000
[TimerNodeSystem-1] ---- Elapsed physical time (in nsec): 3,000,625,306
```

The node creates a 1 s wall timer. Under rcllf it is an LF timer that fires at the
start tag and then every second. The run stops at the tag 3 s, which still runs, so
the timer fires four times. The generated program shows the timer:

```bash
grep -nE "workers|timeout:|timer t_" \
  lf-generated/lf_generated_timer_example_timernodesystem/lf/TimerNodeSystem.lf
```

```text
4:    timeout: 3 sec,
38:    timer t_timer_callback(0, 1000 msec)
```

The two values are the start offset and the period.

## 3. Change the file

Replace `execution.yaml` with:

```yaml
schema_version: 1
defaults:
  workers: 2
nodes:
  timer_node:
    timers:
      timer_callback:
        offset: "1500 msec"
```

`workers` fixes the number of LF worker threads. Without it reactor-c picks one from
the host, so the count differs between machines. `timers` sets the first fire of the
timer `timer_callback` of node `timer_node` to 1.5 s after the start. The names come
from `execution.reference.yaml`.

## 4. Regenerate and check

```bash
colcon lf --package timer_example --timeout "3 sec"
grep -nE "workers|timeout:|timer t_" \
  lf-generated/lf_generated_timer_example_timernodesystem/lf/TimerNodeSystem.lf
```

```text
4:    workers: 2,
5:    timeout: 3 sec,
39:    timer t_timer_callback(1500 msec, 1000 msec)
```

Both keys act at generation, so the generated `.lf` is where to check them.
`colcon lf-configure --show` does not list them; it shows the subscription settings and
the two lists that step 6 uses.

## 5. Run and see the effect

```bash
ros2 lf launch timer_example timer_node
```

```text
[TimerNodeSystem-1] [INFO] [1795774022.423780561] [timer_node]: Timer tick: 0
[TimerNodeSystem-1] [INFO] [1795774023.423667387] [timer_node]: Timer tick: 1
[TimerNodeSystem-1] ---- Start execution on Fri Nov 27 10:07:00 2026 ---- plus 923390796 nanoseconds
[TimerNodeSystem-1] ---- Elapsed logical time (in nsec): 3,000,000,000
[TimerNodeSystem-1] ---- Elapsed physical time (in nsec): 3,000,456,474
```

The program starts at 10:07:00.923. The first tick comes 1.5 s later and the second
one 1 s after that. The tag 3.5 s lies past the stop tag, so the timer fires twice.

A name the node does not have is refused before anything is built. With the timer
spelled `tick` in `/tmp/bad.yaml`:

```yaml
schema_version: 1
nodes:
  timer_node:
    timers:
      tick:
        offset: "5 msec"
```

```bash
colcon lf-generate --package timer_example --exec-config /tmp/bad.yaml
```

```text
  Error checking hazard diagnostics: /tmp/bad.yaml: nodes.'timer_node'.timers.'tick': no timer named 'tick' on 'timer_node' (known timers: timer_callback)

==> 1/1 system(s) failed
```

`--exec-config` reads another file in place of the `execution.yaml` next to the
analysis, which is handy for trying a variant.

## 6. Make a subscription wait

In `pubsub_example` the subscriber reads `sensor_data`. By default its take does not
wait for a message that has not arrived (`take_mode: skip`). Write
`lf-analysis/pubsub_example/bringup/config/execution.yaml`:

```yaml
schema_version: 1
subscriptions:
  subscriber_node:
    sensor_data:
      take_mode: block
      maxwait_ms: 50
```

Check the resolved values before you generate:

```bash
colcon lf-configure --package pubsub_example --show
```

```text
--- /workspace/packages/lf-analysis/pubsub_example/bringup/system.json ---
  Resolved effective policy (post-merge, pre-env), each value with its layer:
    defaults:
      take_mode: "skip"  <- built-in
      take_timeout_ms: 100  <- built-in
    SubscriberNode.sensor_data:
      take_mode: "block"  <- user /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.yaml (subscriptions)
      take_timeout_ms: 50  <- user /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.yaml (subscriptions)
    external_topics: []
    acknowledged_hazards: []
```

Each value names the file and the scope it came from. `take_timeout_ms` is the name
the generated files use for `maxwait_ms`. The other launch files of the package have
their own `execution.yaml` and keep the defaults.

```bash
colcon lf --package pubsub_example --timeout "3 sec"
ros2 lf launch pubsub_example bringup
```

```text
[Bringup-1] [INFO] [1795774117.383770672] [ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/sensor_data', take_mode=block timeout_ms=50)
[Bringup-1] [INFO] [1795774117.384120973] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [1795774117.385165134] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
```

The binding line shows the value the subscription runs with. The publisher still sends
seven messages in 3 s and the subscriber hears all seven, because none arrived late.

## 7. Override for one run

`ROS2_LF_TAKE_MODE` and `ROS2_LF_MAXWAIT_MS` override the file for one run, without a
regeneration:

```bash
ROS2_LF_TAKE_MODE=skip ros2 lf launch pubsub_example bringup
```

```text
[Bringup-1] [WARN] [1795774121.412737127] [ros2_lf_binding]: ROS2_LF_TAKE_MODE=skip overrides take_mode for 1 of 1 configured subscription(s), not for the defaults, which already match
```

They apply to every subscription at once. See
[Environment Variables](../reference/environment-variables.md#ros2_lf_take_mode).

## 8. Undo

Delete the file and run `colcon lf-configure` again. It writes a new empty file:

```bash
rm lf-analysis/pubsub_example/bringup/config/execution.yaml
colcon lf-configure --package pubsub_example
```

```text
--- /workspace/packages/lf-analysis/pubsub_example/bringup/system.json ---
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.yaml (created)
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.reference.yaml
```

Regenerate to build the program without the change.

## Keep a setting with the package

`lf-analysis/` is build output and is not checked in. A setting that belongs to the
package, such as a topic fed from outside the system, goes into
`<package>/lf/<launch stem>.execution.yaml` in the package source.
`pubsub_example/lf/subscriber_node.execution.yaml` is one. The file in `lf-analysis/`
is layered on top of it. See
[Layers](../features/execution-config.md#layers).

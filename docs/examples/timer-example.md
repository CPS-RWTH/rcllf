# timer_example

The simplest ROS 2 node: a single wall timer that fires periodically.

## Pattern

- **Node:** `TimerNode` extends `rclcpp::Node`
- **Timer:** 1s period via `create_wall_timer`
- **Callback:** `timer_callback()`, no parameters (timer callbacks never receive input)
- **No** publishers, subscribers, services, or actions

## What the Extractor Finds

- 1 `TimerInfo` with `period_ms=1000`, `callback="timer_callback"`
- 1 `CallbackInfo` with `parameters=[]`
- Empty publishers, subscribers, services, actions lists

## Run

```bash
docker compose run --rm dev ros2 launch timer_example timer_node.launch.py
```

Under rcllf, inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package timer_example && source install/setup.bash
ros2 lf launch timer_example timer_node
```

The timer becomes a Lingua Franca timer and the ticks count up once per second
(trimmed; your timestamps differ):

```text
[TimerNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (TimerNode::timer_callback, period 1000000000 ns)
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 0
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 1
[TimerNodeSystem-1] [INFO] [...] [timer_node]: Timer tick: 2
```

Tick 0 comes at the start of the program. Under stock ROS 2 the first tick comes one
period (1 s) after the node starts; set the timer's `offset` to `"1 sec"` in
`execution.yaml` to match it
([Timer start offsets](../features/execution-config.md#timer-start-offsets-and-periods)).

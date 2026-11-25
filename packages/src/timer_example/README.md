# timer_example

Demonstrates the minimal ROS2 node pattern: a single wall timer that fires periodically and logs a message.

## Pattern

**Node:** `TimerNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Timer | `create_wall_timer` | Period: 1s |
| Callback | `timer_callback` | Parameters: *(none)* — timer callbacks take no arguments |

This is the simplest possible ROS2 node. There are no publishers, subscribers, services, or actions — just a timer trigger invoking a callback.

## Callback Signature

```cpp
void TimerNode::timer_callback()
// parameters: []
```

Timer callbacks have an empty parameter list. When the extraction tool processes this node, `CallbackInfo.parameters` will be `[]`.

## Run

```bash
docker compose run --rm dev ros2 launch timer_example timer_node.launch.py
```

## What to Expect

```
[INFO] [timer_node]: Timer node started.
[INFO] [timer_node]: Timer tick: 0
[INFO] [timer_node]: Timer tick: 1
...
```

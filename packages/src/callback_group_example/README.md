# callback_group_example

Demonstrates the ROS2 callback group pattern: assigning timers and subscriptions to `MutuallyExclusive` or `Reentrant` callback groups, executed by a `MultiThreadedExecutor`.

## Pattern

**Node:** `CallbackGroupNode` (extends `rclcpp::Node`)

| Callback Group | Type | Assigned Triggers |
|---------------|------|-------------------|
| `exclusive_group_` | `MutuallyExclusive` | Timer A (500ms), Timer B (1s), Subscription (`chatter`) |
| `reentrant_group_` | `Reentrant` | Timer C (750ms) |

| Callback | Parameters | Group |
|----------|-----------|-------|
| `timer_a_callback` | *(none)* | `exclusive_group_` |
| `timer_b_callback` | *(none)* | `exclusive_group_` |
| `timer_c_callback` | *(none)* | `reentrant_group_` |
| `chatter_callback` | `msg` (`String::SharedPtr`) | `exclusive_group_` |

**Executor:** `MultiThreadedExecutor` — allows concurrent execution across groups.

## Callback Signatures

```cpp
// Timer callbacks — no input parameters
void CallbackGroupNode::timer_a_callback()  // parameters: []
void CallbackGroupNode::timer_b_callback()  // parameters: []
void CallbackGroupNode::timer_c_callback()  // parameters: []

// Subscription callback — receives the message
void CallbackGroupNode::chatter_callback(
    const std_msgs::msg::String::SharedPtr msg)
// parameters: [CallbackParameter("msg", "std_msgs::msg::String::SharedPtr")]
```

Callback groups are a **trigger property**, not a callback property. The extraction tool records `callback_group` on each `TimerInfo` and `SubscriberInfo`, not on `CallbackInfo`. The `CallbackInfo.parameters` field captures only the function's input arguments.

## Concurrency Semantics

- `exclusive_group_` (MutuallyExclusive): Timer A, Timer B, and `chatter` callbacks never run simultaneously — they queue behind each other.
- `reentrant_group_` (Reentrant): Timer C can overlap with other groups and itself.

## Run

```bash
docker compose run --rm dev ros2 launch callback_group_example callback_group_node.launch.py
```

## What to Expect

```
[INFO] [callback_group_node]: Callback group node started.
[INFO] [callback_group_node]: Timer A tick: 0
[INFO] [callback_group_node]: Timer C tick: 0
[INFO] [callback_group_node]: Timer B tick: 0
[INFO] [callback_group_node]: Timer A tick: 1
```

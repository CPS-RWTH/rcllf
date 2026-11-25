# lifecycle_example

Demonstrates the ROS2 lifecycle node pattern: managed state transitions with explicit configure/activate/deactivate/cleanup callbacks.

## Pattern

**Node:** `LifecycleExampleNode` (extends `rclcpp_lifecycle::LifecycleNode`)

| State Transition | Callback | Parameters | What It Does |
|-----------------|----------|------------|--------------|
| unconfigured → inactive | `on_configure` | `state` (`State`) | Creates publisher |
| inactive → active | `on_activate` | `state` (`State`) | Starts timer |
| active → inactive | `on_deactivate` | `state` (`State`) | Stops timer |
| inactive → unconfigured | `on_cleanup` | `state` (`State`) | Destroys publisher |

| Element | Type | Detail |
|---------|------|--------|
| Timer | `create_wall_timer` | Period: 1s (created in `on_activate`, destroyed in `on_deactivate`) |
| Callback | `timer_callback` | Parameters: *(none)* |
| Publisher | `create_publisher` (managed) | Topic: `lifecycle_output` |

## Callback Signatures

```cpp
// Lifecycle transition callbacks — all take a State reference
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleExampleNode::on_configure(const rclcpp_lifecycle::State & state)
// parameters: [CallbackParameter("state", "rclcpp_lifecycle::State")]

// Timer callback — no input
void LifecycleExampleNode::timer_callback()
// parameters: []
```

Lifecycle callbacks (`on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`) are extracted into `NodeInfo.lifecycle_callbacks`. Each takes a `State` parameter captured in `CallbackInfo.parameters`.

## Run

```bash
docker compose run --rm dev ros2 launch lifecycle_example lifecycle_node.launch.py
```

## What to Expect

```
[INFO] [lifecycle_node]: LifecycleExampleNode has been created
[INFO] [lifecycle_node]: LifecycleExampleNode has been configured
[INFO] [lifecycle_node]: LifecycleExampleNode has been activated
[INFO] [lifecycle_node]: Publishing: 'Lifecycle tick: 0'
```

# lifecycle_example

Managed lifecycle node with state transitions.

## Pattern

- **Node:** Extends `rclcpp_lifecycle::LifecycleNode`
- **State callbacks:** `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- Publisher created in `on_configure`, timer started in `on_activate`

## What the Extractor Finds

- `NodeInfo.base_class = "rclcpp_lifecycle::LifecycleNode"`
- `NodeInfo.lifecycle_callbacks = ["on_configure", "on_activate", "on_deactivate", "on_cleanup"]`

## Run

```bash
docker compose run --rm dev ros2 launch lifecycle_example lifecycle_node.launch.py
```

Under rcllf, inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package lifecycle_example && source install/setup.bash
ros2 lf launch lifecycle_example lifecycle_node
```

The generated program configures and activates the node before it binds the timer, then
the ticks publish once per second (trimmed; your timestamps differ):

```text
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been configured
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been activated
[LifecycleNode-1] [INFO] [...] [ros2_lf_binding]: Lifecycle node 'LifecycleExampleNode': configured and activated before binding
[LifecycleNode-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (LifecycleExampleNode::timer_callback, period 1000000000 ns)
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 0'
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 1'
```

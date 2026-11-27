# lifecycle_example

A managed lifecycle node: it publishes only after it has been configured and
activated. The example shows how rcllf drives the lifecycle before it hands the node's
timer to the LF scheduler.

## What it shows

- **Node:** `LifecycleExampleNode` extends `rclcpp_lifecycle::LifecycleNode`.
- **State callbacks:** `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`.
- The publisher is created in `on_configure`; the 1 s timer starts in `on_activate`.

## Run with stock ROS 2

The first start of the dev container builds this package:

```bash
docker compose run --rm dev ros2 launch lifecycle_example lifecycle_node.launch.py
```

The launch file configures and activates the node, then a tick is published every
second (trimmed; your timestamps differ):

```text
[lifecycle_node-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been created
[lifecycle_node-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been configured
[lifecycle_node-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been activated
[lifecycle_node-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 0'
[lifecycle_node-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 1'
```

## Run under rcllf

Inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package lifecycle_example && source install/setup.bash
ros2 lf launch lifecycle_example lifecycle_node
```

The generated program configures and activates the node before it binds the timer,
then the ticks publish once per second (trimmed):

```text
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been configured
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: LifecycleExampleNode has been activated
[LifecycleNode-1] [INFO] [...] [ros2_lf_binding]: Lifecycle node 'LifecycleExampleNode': configured and activated before binding
[LifecycleNode-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (LifecycleExampleNode::timer_callback, period 1000000000 ns)
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 0'
[LifecycleNode-1] [INFO] [...] [lifecycle_node]: Publishing: 'Lifecycle tick: 1'
```

## What to look at

- **Activation before binding.** rcllf binds only the timers and subscriptions that
  exist when the program starts. This timer is created in `on_activate`, so the
  generated program activates the node first. For a node that a supervisor activates
  later, set `auto_transition: false`; see
  [Lifecycle auto-transition](../features/execution-config.md#lifecycle-auto-transition).
- **What the analyzer read.** In
  `lf-analysis/lifecycle_example/lifecycle_node/system.json` the node has
  `base_class` `rclcpp_lifecycle::LifecycleNode` and `lifecycle_callbacks`
  `["on_configure", "on_activate", "on_deactivate", "on_cleanup"]`, one timer
  (`period_ms` 1000.0, `callback` `timer_callback`) and one publisher on
  `lifecycle_output`.

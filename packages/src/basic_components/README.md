# basic_components

Small static ROS 2 component nodes (`std_msgs/String` on `basic_components/*`
topics) that the colcon_lf analysis and generation tests use as building
blocks. Each node is registered with `RCLCPP_COMPONENTS_REGISTER_NODE` and
built into one shared library, `basic_components`.

| Component | What it does |
|---|---|
| `TimerPublisherNode` | publishes a string every second |
| `TimerOnlyNode` | a wall timer, no topics |
| `ListenerNode` | one subscription |
| `RelayNode` | subscribes and republishes |
| `TimerAndSubscriptionNode` | a timer and a subscription in one node |
| `DualTimerPublisherNode` | two publishers on `basic_components/dual_in_a` and `dual_in_b` |
| `DualListenerNode` | two subscriptions |
| `DualSubscriptionTimerPublisherNode` | two subscriptions, a timer and a publisher |

## Launch files

- `timer_publisher.launch.py`, `listener_node.launch.py`, `relay_node.launch.py`:
  one component each.
- `combo_pipeline.launch.py`: publisher, relay and listener composed in one
  container.

`lf/listener_node.execution.yaml` and `lf/relay_node.execution.yaml` are
example execution configurations.

```bash
colcon lf-analyze --package basic_components --launch-file combo_pipeline.launch.py
```

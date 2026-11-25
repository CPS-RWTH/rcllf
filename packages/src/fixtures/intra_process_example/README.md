# intra_process_example

Demonstrates ROS2 intra-process communication (zero-copy) using composable components
loaded into a single `component_container_mt`.

## Components

- **IntraProcessPublisherComponent**: 100 Hz timer, publishes
  `std::unique_ptr<std_msgs::msg::Int64>` to `/intra_topic`. Logs the pointer address
  at DEBUG level.
- **IntraProcessSubscriberComponent**: subscribes with `unique_ptr` callback. Logs
  the received pointer address.

When both components share the same container with `use_intra_process_comms=true`, the
pointer address in the subscriber log will match the publisher's, which confirms zero-copy.

## Usage

```bash
ros2 launch intra_process_example bringup.launch.py
# Enable DEBUG logging to see pointer addresses:
RCUTILS_LOGGING_SEVERITY=DEBUG ros2 launch intra_process_example bringup.launch.py
```

## Purpose

Covers the intra-process composable node pattern absent from other examples. It is the
input of the playground case `ipc.intra_process`.

## LF pipeline status

rcllf refuses intra-process communication at analysis with RCLLF-H017: rclcpp hands a
message to a subscription in the same process without the middleware, so the message has
no publisher identity for rcllf to order. `colcon lf-analyze` reports the hazard and
`colcon lf-generate` refuses the system. Run the example with plain `ros2 launch` as
above; see the RCLLF-H017 entry in the troubleshooting guide for the workaround.

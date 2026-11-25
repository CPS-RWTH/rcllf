# tf_broadcaster_example

Demonstrates canonical TF2 transform broadcasting patterns — both static and dynamic.

## Nodes

- **static_tf_broadcaster** — publishes a fixed `world → base_link` transform once at
  startup via `tf2_ros::StaticTransformBroadcaster`.
- **dynamic_tf_broadcaster** — 30 Hz timer, publishes a rotating `base_link → sensor_link`
  transform via `tf2_ros::TransformBroadcaster`. The `sensor_link` orbits `base_link`
  at 0.3 m radius with a slowly increasing yaw angle.

## Purpose

Fills the TF2 gap in `packages/src/`. Documents the LF binding limitation: TF2 uses
internal subscriptions and publishers that are transparent to `rclcpp::Node`'s entity
registry, so the waitable extractor cannot currently model TF2 as LF reactions.

## Usage

```bash
ros2 launch tf_broadcaster_example bringup.launch.py
# Verify transforms:
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo world sensor_link
```

## LF pipeline status

TF2 is out of scope for the LF binding. The analyzer will not include TF2 internals
in `system.json`. Do not run through `colcon lf-build`.

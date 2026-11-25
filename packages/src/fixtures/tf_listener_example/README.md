# tf_listener_example

Demonstrates consuming TF2 transforms via `tf2_ros::Buffer` + `tf2_ros::TransformListener`.
Pair package to `tf_broadcaster_example`.

## Nodes

- **tf_listener** — 10 Hz timer, calls `buffer_.lookupTransform("world", "sensor_link",
  tf2::TimePointZero)` and logs the translation. Tolerates missing transforms by catching
  `tf2::TransformException`.

## Usage

```bash
# Full system (broadcasters + listener):
ros2 launch tf_listener_example bringup.launch.py

# Verify with tf2 tools:
ros2 run tf2_ros tf2_echo world sensor_link
ros2 run tf2_tools view_frames
```

## Purpose

Pair to `tf_broadcaster_example`. Kept as a separate package because
`tf2_ros::TransformListener` holds an internal subscription that belongs to a different
node concern than the broadcaster.

## LF pipeline status

`colcon lf-generate --package tf_listener_example` refuses `tf_listener` with
`RCLLF-R002 tf2_direct_lookup`: `lookupTransform(..., tf2::TimePointZero)` answers with
whatever the Buffer holds when the reaction runs, so the value at a tag depends on
arrival time. The deterministic form uses `rcllf_tf2::AcquisitionLookup`;
see `tf2_acq_example` and [docs/features/tf2.md](../../../../docs/features/tf2.md). The
broadcasters of `tf_broadcaster_example` generate without a refusal.

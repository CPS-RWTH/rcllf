# reentrant_fanout_example

Demonstrates the divergence between stock ROS2 reentrant callback group behaviour
(callbacks may run concurrently) and the LF binding's intentional serialization.

## Nodes

- **fanout_publisher** — 50 Hz timer, publishes incrementing `std_msgs/Int64` on `/work`.
- **fanout_worker** — subscribes to `/work` in a `Reentrant` callback group, sleeps 200 ms
  per callback, and logs its thread ID, sequence number, and start/finish timestamps.
  Spun by a `MultiThreadedExecutor` with 4 threads.

## Expected behaviour under stock ROS2

Because the publisher fires at 50 Hz (every 20 ms) and each callback takes 200 ms,
multiple callbacks will be in-flight simultaneously. The logs will show ≥ 2 distinct
thread IDs active at the same time — the hallmark of reentrant concurrency.

## Expected behaviour under LF binding

The LF runtime serializes all callbacks at the same logical tag, so only one thread
will be active at a time regardless of executor settings. This divergence is documented
and expected — it reflects the deterministic execution model LF enforces.

## Purpose

Provides a verification harness for reentrant callback groups with fan-out.

## Usage

```bash
ros2 launch reentrant_fanout_example bringup.launch.py
```

## LF pipeline

```bash
colcon lf-analyze --package reentrant_fanout_example
```

The analyzer should produce `system.json`. The LF runtime will serialize callbacks
that stock ROS2 runs concurrently — the README divergence note is the expected output.

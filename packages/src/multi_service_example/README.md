# multi_service_example

Demonstrates a single ROS2 server node hosting two services with different request/response
types, and a client node that calls both alternately.

## Nodes

- **multi_service_server** — exposes `/process_data` (`custom_interfaces/srv/ProcessData`)
  and `/add_two_ints` (`example_interfaces/srv/AddTwoInts`).
- **multi_service_client** — 1 Hz timer, alternates between calling `process_data` (even
  ticks) and `add_two_ints` (odd ticks) using async callbacks.

## Purpose

Exercises the LF converter's service-name disambiguation path. The converter's
`clang_parser.py` has a fallback heuristic (around line 1031) that picks the wrong
callback when a node has more than one service client. This package provides a
reproducible fixture to verify and fix that behaviour.

## Usage

```bash
ros2 launch multi_service_example bringup.launch.py
```

## LF pipeline

```bash
colcon lf-analyze --package multi_service_example
colcon lf-generate
colcon lf-build
```

The analyzer should produce `system.json` with both services listed. The generator may
warn about service-disambiguation gaps but must not crash.

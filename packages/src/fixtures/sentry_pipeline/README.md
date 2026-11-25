# sentry_pipeline

Flagship benchmark pipeline.
A 9-node, single-process sensor-fusion / planning / control system used to
compare stock rclcpp executors (single/multi/events threaded) against the
`lfc` (Lingua Franca) runtime variant.

## Topology

Hot path: `ImuNode`, `LidarNode` → `FusionNode` → `PlannerNode` →
`ControllerNode` → `ActuatorSinkNode`.
Side path: `CameraNode` → `DetectorNode` → (`PlannerNode`, `DiagnosticsSinkNode`).

| Node | Path | Rate (param default) | Pubs | Subs |
|---|---|---|---|---|
| `ImuNode` | hot | 200 Hz (`rate_hz`) | `/sentry/imu` | — |
| `LidarNode` | hot | 10 Hz (`rate_hz`) | `/sentry/scan` | — |
| `CameraNode` | side | 25 Hz (`rate_hz`) | `/sentry/image` | — |
| `FusionNode` | hot | reactive | `/sentry/state` | `/sentry/imu`, `/sentry/scan` |
| `DetectorNode` | side | reactive | `/sentry/detections` | `/sentry/image` |
| `PlannerNode` | hot+side merge | 50 Hz (`rate_hz`) | `/sentry/plan` | `/sentry/state`, `/sentry/detections` |
| `ControllerNode` | hot | reactive (no timer) | `/sentry/cmd` | `/sentry/plan` |
| `ActuatorSinkNode` | hot | reactive | — | `/sentry/cmd` |
| `DiagnosticsSinkNode` | side | reactive | — | `/sentry/detections` |

`PlannerNode` is the only node with both a timer and a subscription, so
`PlannerNode->ControllerNode` is the pipeline's one deliberate gated hop
(`lf_eval`'s `timer_gated_nodes()`/hop-gating rule). All other hops are
ungated.

## Scenario mechanism

Node timer rates and QoS depth/reliability are **declared ROS parameters
with literal defaults**, chosen so `clang_parser.py`'s extraction produces
expected `null` values (rate: parameter-derived duration; QoS: a plain
`qos_profile` local with no literal integer/policy word in its construction
site) — not a defect, a designed analyzer limitation resolved
outside the analysis:

- **Timer rate** reaches the stock variant via its own ROS params file, and
  the `lfc` variant via a `system.json` patch step that fills `period_ms` from the scenario's `rate_hz`
  between `lf-analyze` and `lf-generate` — the only field that patch step
  touches.
- **QoS depth/reliability, `busy_us`, `payload_size`** reach both variants
  identically through the `SENTRY_SCENARIO` env-var overlay (a flat
  `KEY=VALUE` file, `<node_name>.<key>=<value>` per line), read once at node
  construction via `scenario_env.hpp`'s `qos_from_overlay`/`int_from_overlay`
  helpers, with `declare_parameter` as the stock-only fallback and a literal
  default as the final tier.

**QoS variable-naming rule (binding for any future node added here):** the
QoS local is always named `qos_profile` (or `qos_profile_<topic_suffix>`
for more than one profile); never embed a QoS policy word (`reliable`,
`best_effort`, `durability`, `keep_last`, `transient`, `volatile`, ...) in a
variable or topic identifier — `_extract_qos_attrs`'s token-substring scan
would otherwise report a non-null, silently wrong value.


If `SENTRY_SCENARIO` is set but the file cannot be opened, every node refuses
to start with an error naming the path. Unset means the declared defaults.

## Composable-node launch (single container, any executor externally)

```bash
ros2 launch sentry_pipeline sentry_pipeline.launch.py
```

Loads all 9 nodes into one `rclcpp_components` container — required so the
executor axis and the `lfc` variant differ only in scheduling/runtime, never
in process topology.

## LF pipeline

```bash
colcon lf-analyze --package sentry_pipeline
colcon lf-generate --package sentry_pipeline
colcon build --packages-select lf_generated_sentry_pipeline_sentrypipeline
ros2 lf launch sentry_pipeline sentry_pipeline
```

Rate is baked in at generation time from the scenario's `system.json` patch,
so the `lfc` variant is regenerated and rebuilt once per scenario (so a
comparison does not favor one variant); QoS/`busy_us`/`payload_size` still reach it at
launch time via `SENTRY_SCENARIO`.

## Messages

`sensor_msgs/msg/Imu`, `sensor_msgs/msg/LaserScan`, `sensor_msgs/msg/Image`,
`geometry_msgs/msg/Twist` (all standard), plus three new minimal messages in
`custom_interfaces`: `FusedState`, `Detections`, `Plan`.

## Message log

Setting `RCLLF_MSGLOG_DIR` to a directory makes every node write what it
publishes to `<dir>/<node name><topic with / replaced by _>.jsonl` (format
`rcllf.msglog/1`, see `include/sentry_pipeline/msg_log.hpp`): an `open` line
at creation, one `pub` line per publication (sequence number, time, CDR bytes
as hex) and a `close` line with the node's own counters. It is off when the
variable is unset. It gives a source-side record of the published values
without a rosbag.

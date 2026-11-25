# Example Packages

`packages/src/` holds example ROS 2 packages that serve as both test fixtures for
the extraction tool and reference implementations of common ROS 2 patterns. The
packages below have a page that shows the pattern, what the extractor finds,
and how to run it; further packages are listed after the table.

| Package | Pattern | Nodes | Launch file |
|---------|---------|-------|-------------|
| [timer_example](timer-example.md) | Wall timer | 1 | `timer_node.launch.py` |
| [pubsub_example](pubsub-example.md) | Publish/Subscribe | 2 | `bringup.launch.py` |
| [service_example](service-example.md) | Service (request/response) | 2 | `bringup.launch.py` |
| [action_example](action-example.md) | Action (goal/feedback/result) | 2 | `bringup.launch.py` |
| [component_example](component-example.md) | Composable node | 1 | `component_node.launch.py` |
| [lifecycle_example](lifecycle-example.md) | Lifecycle state machine | 1 | `lifecycle_node.launch.py` |
| [parameter_example](parameter-example.md) | Parameters + YAML config | 1 | `parameter_node.launch.py` |
| [callback_group_example](callback-group-example.md) | Callback groups + MultiThreadedExecutor | 1 | `callback_group_node.launch.py` |
| [fusion_example](fusion-example.md) | message_filters ExactTime and ApproximateTime | 4 | `bringup.launch.py` |
| [brake_fusion](brake-fusion.md) | stock vs rcllf: deterministic stopping position on a MuJoCo plant | 8 | `t2_separate.launch.py`, `t2_approx.launch.py` |
| custom_interfaces | Message/Service/Action definitions | 0 | N/A |
| [system_bringup](system-bringup.md) | Launch orchestration | 0 (includes others) | `system_bringup.launch.py` |

### custom_interfaces

Custom message, service, and action type definitions shared by the example
packages: `msg/SensorData.msg` (used by `pubsub_example`), `srv/ProcessData.srv`
(used by `service_example`), `action/Analyze.action` (used by `action_example`),
plus `BrakeCmd`, `SyncSample`, `Detections`, `FusedState` and `Plan` messages
for `brake_fusion`, `sync_oracle_showcase` and `sentry_pipeline`.
It is interface-only, with no nodes and no standalone Docker service. The extractor
records it as `CMakeInfo.interfaces` list of all its message, service and action files. Built automatically as a
dependency of the packages that use it.

### More example packages

These have no page of their own; each package directory describes its launch
files, and `colcon lf-analyze --package <name>` shows what rcllf finds.

| Package | Pattern |
|---------|---------|
| `multi_service_example` | one server node hosting two services of different types (`custom_interfaces/srv/ProcessData` and `example_interfaces/srv/AddTwoInts`) and a client; `ros2 launch multi_service_example bringup.launch.py` |
| `reentrant_fanout_example` | reentrant callback group concurrency in stock ROS 2 versus the serialized dispatch of the LF binding: a publisher and a worker node, `ros2 launch reentrant_fanout_example bringup.launch.py` |
| `action_full_example` | full action lifecycle: feedback timer, rejection, cancellation, sequential goals |
| `action_example` (`dual.launch.py`) | one action server and one client in a single launch file of two nodes |
| `handle_matching_example` | a binding corner case: matching publisher and subscription handles |
| `tf2_acq_example`, `sync_oracle_showcase`, `bna_example` | tf2 acquisition lookups, a `message_filters` oracle corpus, and the `BoundedNearestToAnchor` synchronizer policy |


### Test fixtures

Used by the CI runtime tests, not meant as reference code: `basic_components`,
`correspondence_fixture`, `same_tag_fixture`, `take_skip_fixture` and the packages
under `packages/src/fixtures/`.

## Running Examples

Examples run with `ros2 launch` inside the `dev` container (no compose profile needed).
The container's first start builds only the runtime and `pubsub_example`,
`timer_example`, `service_example`, `lifecycle_example`, `fusion_example` and
`brake_fusion`. For any other example, name it in `ENTRYPOINT_PACKAGES`, or pass
`ENTRYPOINT_PACKAGES=all` to build everything:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup" dev
```

Then:

```bash
docker compose run --rm dev ros2 launch <package> <launch-file>
```

For example:

```bash
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py
docker compose run --rm dev ros2 launch service_example bringup.launch.py
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup" dev \
  ros2 launch system_bringup system_bringup.launch.py
```

## Analyzing Examples

Use `colcon lf-analyze` to extract the system model from any example:

```bash
# Inside Docker dev container
cd packages
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py
```

Or analyze all examples at once:

```bash
colcon lf-analyze --format json
```

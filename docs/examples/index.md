# Example Packages

Each example is a small, ordinary ROS 2 package that shows one ROS 2 pattern: a timer,
a topic, a service, an action, and so on. Every example page shows the same four
things: what the example shows, how to run it with stock ROS 2, how to run it under
rcllf, and what to look at in the output and in the generated files. If you have not
run rcllf yet, start with the [Quickstart](../getting-started/index.md); the examples
use the same commands.

| Package | Pattern | Nodes | Launch file | Built at first start | Runs under rcllf |
|---------|---------|-------|-------------|----------------------|------------------|
| [timer_example](timer-example.md) | Wall timer | 1 | `timer_node.launch.py` | yes | yes |
| [pubsub_example](pubsub-example.md) | Publish/Subscribe | 2 | `bringup.launch.py` | yes | yes |
| [service_example](service-example.md) | Service (request/response) | 2 | `bringup.launch.py` | yes | yes |
| [action_example](action-example.md) | Action (goal/feedback/result) | 2 | `bringup.launch.py` | no | yes |
| [component_example](component-example.md) | Composable node | 1 | `component_node.launch.py` | no | yes |
| [lifecycle_example](lifecycle-example.md) | Lifecycle state machine | 1 | `lifecycle_node.launch.py` | yes | yes |
| [parameter_example](parameter-example.md) | Parameters + YAML config | 1 | `parameter_node.launch.py` | no | yes |
| [callback_group_example](callback-group-example.md) | Callback groups + MultiThreadedExecutor | 1 | `callback_group_node.launch.py` | no | yes |
| [fusion_example](fusion-example.md) | message_filters ExactTime and ApproximateTime | 4 | `bringup.launch.py` | yes | yes |
| [brake_fusion](brake-fusion.md) | stock vs rcllf: deterministic stopping position on a MuJoCo plant | 8 | `t2_separate.launch.py`, `t2_approx.launch.py` | yes | yes |
| custom_interfaces | Message/Service/Action definitions | 0 | N/A | yes | N/A |
| [system_bringup](system-bringup.md) | Launch orchestration | 6 (from included packages) | `system_bringup.launch.py` | no | yes |

"Built at first start: no" means the dev container does not build the package unless
you select it; each of those pages gives the command (see
[Running examples](#running-examples)).

### custom_interfaces

Custom message, service, and action type definitions shared by the example
packages: `msg/SensorData.msg` (used by `pubsub_example`), `srv/ProcessData.srv`
(used by `service_example`), `action/Analyze.action` (used by `action_example`),
plus `BrakeCmd`, `SyncSample`, `Detections`, `FusedState` and `Plan` messages used by
`brake_fusion` and other examples. It has no nodes and no launch files, so rcllf
generates nothing for it. colcon builds it as a dependency of the packages that use
it.

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

## Running examples

Examples run with `ros2 launch` inside the `dev` container (no Compose profile
needed). The container's first start builds the runtime and the packages marked
"yes" in the table above. For any other example, name it in `ENTRYPOINT_PACKAGES`;
the start then builds that package and what it depends on:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="action_example" dev \
  ros2 launch action_example bringup.launch.py
```

Name several packages separated by spaces, or pass `ENTRYPOINT_PACKAGES=all` to build
everything. A launch file that includes another package's launch file needs that
package too: `system_bringup` needs
`ENTRYPOINT_PACKAGES="system_bringup parameter_example"`. Packages built once stay in
the workspace, so later starts with a different selection add to them.

To run an example under rcllf, start the container with the same selection and run
`colcon lf --package <name>` and `ros2 lf launch <name> <launch stem>` inside it, as
each page shows.

## Analyzing examples

`colcon lf-analyze` alone writes the system model (`system.json`) and the diagrams
without generating a program:

```bash
# inside the dev container
cd packages
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py
```

The [CLI Reference](../reference/cli.md#colcon-lf-analyze) lists the options.

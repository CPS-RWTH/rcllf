# Extraction Tool (colcon-lf)

`colcon-lf` statically extracts the full structure of a ROS 2 workspace, including launch
files, C++ node source, and inter-node communication, into the `system.json` model
that LF code generation consumes. Read this page for what the extractor pulls out of
each layer. For CLI invocation, see [CLI Reference](../reference/cli.md).

The tool lives in `tools/colcon_lf/` and is a [colcon](https://colcon.readthedocs.io/)
verb extension (`colcon lf-analyze`).

## What it extracts

### Launch layer

Parses Python-based launch files by importing them and walking the
`LaunchDescription` tree. It records node declarations with package,
executable, namespace, and parameters, recursively resolves
`IncludeLaunchDescription` hierarchies, and captures launch arguments with
their defaults and choices, group actions with `PushRosNamespace`,
`IfCondition`/`UnlessCondition` conditions, remappings, parameter-file
references, and composable node containers with their loaded components.

### Node layer

Uses libclang to parse C++ source against `compile_commands.json`. From each
node class it records the triggers (wall timers, subscriptions, service
servers, action servers) and resources (publishers, service clients, action
clients), the callbacks behind them (function names, parameter types, and
intra-callback variable reads and writes), and the callback groups they run
in (`MutuallyExclusive` and `Reentrant`, including indirect assignment via
`SubscriptionOptions`, the Nav2/Autoware pattern). It also detects declared
parameters with types and defaults, `on_set_parameters_callback` registration,
`rclcpp_lifecycle::LifecycleNode` classes with their state-transition
callbacks, and component registration via `RCLCPP_COMPONENTS_REGISTER_NODE`.

### Communication layer

Matches publishers to subscribers, service servers to clients, and action servers to clients across all nodes. Handles:

- Namespace resolution (absolute vs. relative topic names)
- Launch-level remappings
- Orphaned endpoint detection (unmatched publishers/subscribers)

### CMake Layer

Parses `CMakeLists.txt` to map executable/library targets to source files, extract `ament_target_dependencies`, detect component registration, and identify `rosidl_generate_interfaces` definitions.

## Module layout

How the extractor's modules feed each other. For where extraction sits in the
overall pipeline, see [Architecture](architecture.md).

```
Launch Files (.py)          C++ Source (.cpp/.hpp)       CMakeLists.txt
       |                           |                          |
  launch_analysis/            node_analysis/            node_analysis/
    parser.py                  clang_parser.py           cmake_parser.py
       |                           |                          |
       v                           v                          v
  LaunchGraph               NodeInfo (per node)          CMakeInfo
       |                           |                          |
       +---------------------------+--------------------------+
                                   |
                            comm_analysis/
                             topic_mapper.py
                                   |
                                   v
                          SystemCommunication
                                   |
                                graph/
                            hierarchical.py
                                   |
                                   v
                             SystemGraph
                              /        \
                        output/       visualization/
                       JSON files     DOT -> PDF/SVG
```

## Example packages

The tool is tested against 10 example ROS 2 packages in `packages/src/` that exercise every pattern the extraction tool must handle:

| Package | Key Pattern |
|---------|-------------|
| `timer_example` | Minimal node with a wall timer |
| `pubsub_example` | Publisher + subscriber with custom message type |
| `service_example` | Service server + timer-driven client |
| `component_example` | Composable node with `RCLCPP_COMPONENTS_REGISTER_NODE` |
| `lifecycle_example` | `rclcpp_lifecycle::LifecycleNode` with state transitions |
| `action_example` | Action server + client with feedback |
| `parameter_example` | Parameters from YAML config, change callbacks |
| `callback_group_example` | MutuallyExclusive + Reentrant groups, MultiThreadedExecutor |
| `custom_interfaces` | Custom `.msg`, `.srv`, `.action` definitions |
| `system_bringup` | Top-level launch orchestration with namespaces, conditionals, remappings |

## Further reading

- [Data Model Reference](../reference/data-model.md) -- All dataclass definitions
- `CONTRIBUTING.md` (repo root, "Testing" section) -- installation and test instructions

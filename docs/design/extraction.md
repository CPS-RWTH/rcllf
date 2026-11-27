# Extraction

`colcon lf-analyze` reads a ROS 2 package without running it: its launch
files, the C++ source of its nodes and its CMake files. From these it builds
`system.json`, the model of nodes, callbacks and connections that
[LF code generation](lf-generation.md) consumes. This page explains what the
analyzer extracts from each layer, and what it cannot see. The options are in
the [CLI reference](../reference/cli.md#colcon-lf-analyze), and the place of
this step in the pipeline is on [Architecture](architecture.md).

## A first look

Inside the dev container (see
[First LF Run](../getting-started/first-lf-run.md)), `colcon lf` runs the
analyzer as its first step:

```bash
colcon lf --package pubsub_example
```

```text
Step 1/4: colcon lf-analyze
============================================================
==> Found 4 launch file(s) to analyze

--- pubsub_example/bringup.launch.py ---
  2 nodes, 1 topics, 0 services, 0 actions
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/
```

Each launch file gets its own analysis folder. It holds `system.json`, one
JSON file per node under `nodes/`, and the launch graph as
`launch/launch_graph.json` and `launch/launch_graph.svg`. The graph is drawn as
SVG by default; `--format pdf` writes PDF instead.

## What it extracts

### Launch layer

The analyzer reads Python, XML and YAML launch files through the ROS 2 launch
library, which gives the same launch description for all three formats, and
walks that description. It records:

- node declarations with package, executable, namespace and parameters;
- included launch files, resolved recursively;
- launch arguments with their defaults and choices;
- group actions with `PushRosNamespace`, and `IfCondition` and
  `UnlessCondition` conditions;
- remappings and parameter file references;
- composable node containers with their loaded components.

### Node layer

The analyzer parses the C++ source with libclang against
`compile_commands.json`. From each node class it records:

- the triggers (wall timers, subscriptions, service servers, action servers)
  and the resources (publishers, service clients, action clients);
- the callbacks behind them: function names, parameter types, and the member
  variables each callback reads and writes;
- the callback groups they belong to (`MutuallyExclusive` and `Reentrant`),
  also when a group is passed through `SubscriptionOptions`;
- declared parameters with types and defaults, and
  `on_set_parameters_callback` registration;
- `rclcpp_lifecycle::LifecycleNode` classes with their transition callbacks;
- component registration through `RCLCPP_COMPONENTS_REGISTER_NODE`.

### Communication layer

The analyzer matches publishers to subscribers, service servers to clients,
and action servers to clients across all nodes. It resolves namespaces
(absolute and relative names) and launch-level remappings, and reports
endpoints that have no partner.

### CMake layer

The analyzer reads `CMakeLists.txt` to map executable and library targets to
their source files, and to find `ament_target_dependencies`, component
registration and `rosidl_generate_interfaces` definitions.

## What the analyzer cannot see

Static analysis sees only what the source states. Some patterns are invisible
to it, and each one has a diagnostic code. A warning is printed at analyze
time; an error stops generation unless it is acknowledged.

| Pattern | Code |
|---------|------|
| A topic name built at run time (concatenation, a member, a parameter) | [RCLLF-H004](../reference/diagnostic-codes.md#rcllf-h004) (warning) |
| A timer created in an ordinary method or callback | [RCLLF-H006](../reference/diagnostic-codes.md#rcllf-h006) (warning) |
| A callback that is neither `std::bind(&Node::method, this)` nor an inline lambda | [RCLLF-H007](../reference/diagnostic-codes.md#rcllf-h007) (error) |
| A subscription, service, client or publisher created outside the constructor | [RCLLF-H013](../reference/diagnostic-codes.md#rcllf-h013) (error) |
| Work in `main()` outside callbacks, such as a `spin_some` loop that publishes | [RCLLF-H014](../reference/diagnostic-codes.md#rcllf-h014) (error) |
| Endpoints created in the constructor of a template base class | [RCLLF-H015](../reference/diagnostic-codes.md#rcllf-h015) (error) |
| One source file built into several executables with different compile definitions | [RCLLF-H016](../reference/diagnostic-codes.md#rcllf-h016) (error) |

The full list, with what to do about each code, is on
[Diagnostic Codes](../reference/diagnostic-codes.md). The
[Support Matrix](../features/support-matrix.md) lists which ROS 2 features
convert.

## Module layout

How the analyzer's modules feed each other. All paths are under
`tools/colcon_lf/colcon_lf/`.

```text
Launch files (.py/.xml/.yaml)   C++ source (.cpp/.hpp)     CMakeLists.txt
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
                       JSON files     SVG (default) or PDF
```

## Further reading

- [Data Model Reference](../reference/data-model.md): the fields of
  `system.json`.
- [Examples](../examples/index.md): the example packages and the pattern each
  one shows.

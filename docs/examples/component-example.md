# component_example

A composable node: built as a shared library and loaded into a component container at
run time, instead of running as its own executable.

## What it shows

- **Node:** `ComponentNode` with an `explicit ComponentNode(const rclcpp::NodeOptions&)`
  constructor and a 1 s timer that publishes on `component_output`.
- **Registration:** `RCLCPP_COMPONENTS_REGISTER_NODE(component_example::ComponentNode)`.
- **CMake:** `add_library()` (not `add_executable()`) and
  `rclcpp_components_register_nodes()`.
- **Launch:** a `ComposableNodeContainer` that loads the component.

## Run with stock ROS 2

The first start of the dev container does not build this package, so select it with
`ENTRYPOINT_PACKAGES`:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="component_example" dev \
  ros2 launch component_example component_node.launch.py
```

The container loads the component, which publishes a tick every second (trimmed; your
timestamps differ):

```text
[component_container-1] [INFO] [...] [component_container]: Load Library: /workspace/packages/install/component_example/lib/libcomponent_node.so
[component_container-1] [INFO] [...] [component_node]: ComponentNode has been started
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/component_node' in container '/component_container'
[component_container-1] [INFO] [...] [component_node]: Publishing: 'Component tick: 0'
[component_container-1] [INFO] [...] [component_node]: Publishing: 'Component tick: 1'
```

## Run under rcllf

Start the dev container with the same selection
(`docker compose run --rm -e ENTRYPOINT_PACKAGES="component_example" dev`), then:

```bash
cd packages
colcon lf --package component_example && source install/setup.bash
ros2 lf launch component_example component_node
```

The generated program loads the same component and binds its timer (trimmed):

```text
Launching LF-controlled system: component_example/component_node
  Package: lf_generated_component_example_componentnodesystem
  Launch file: lf_componentnodesystem.launch.py
...
[ComponentNodeSystem-1] [INFO] [...] [lf_component_manager]: Found 1 component instance(s) to load
[ComponentNodeSystem-1] [INFO] [...] [lf_component_manager]: Load Library: /workspace/packages/install/component_example/lib/libcomponent_node.so
[ComponentNodeSystem-1] [INFO] [...] [component_node]: ComponentNode has been started
[ComponentNodeSystem-1] [INFO] [...] [lf_component_manager]: Loaded component_example::ComponentNode as 'ComponentNode' (id=1)
[ComponentNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (ComponentNode::timer_callback, period 1000000000 ns)
[ComponentNodeSystem-1] [INFO] [...] [component_node]: Publishing: 'Component tick: 0'
[ComponentNodeSystem-1] [INFO] [...] [component_node]: Publishing: 'Component tick: 1'
```

## What to look at

- **No container under rcllf.** The generated program is one process whose
  `lf_component_manager` loads the component library itself, so no
  `component_container` process starts.
- **What the analyzer read.** In
  `lf-analysis/component_example/component_node/system.json` the node `component_node`
  has `is_component` `true` and `qualified_name` `component_example::ComponentNode`,
  one timer (`period_ms` 1000.0) and one publisher on `component_output`. The
  `launch_graph` entry for the node records `composable_container`
  `component_container`.

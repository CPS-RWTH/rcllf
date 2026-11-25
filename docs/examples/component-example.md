# component_example

Composable node pattern, loaded as a shared library into a container at runtime.

## Pattern

- **Node:** `ComponentNode` with `explicit ComponentNode(const rclcpp::NodeOptions&)` constructor
- **Registration:** `RCLCPP_COMPONENTS_REGISTER_NODE(component_example::ComponentNode)`
- **CMake:** `add_library()` (not `add_executable()`), `rclcpp_components_register_nodes()`
- **Launch:** Uses `ComposableNodeContainer` with `LoadComposableNode`

## What the Extractor Finds

- `NodeInfo.is_component = True`
- `CMakeInfo.libraries` contains the component target with `component_plugin` set
- Launch parser detects `ComposableNodeContainer` and loaded component plugin name

## Run

```bash
docker compose run --rm dev ros2 launch component_example component_node.launch.py
```

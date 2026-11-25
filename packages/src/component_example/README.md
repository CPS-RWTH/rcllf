# component_example

Demonstrates the ROS2 component (composable node) pattern: a node compiled as a shared library and loaded into a container at runtime.

## Pattern

**Component:** `ComponentNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Constructor | `ComponentNode(const rclcpp::NodeOptions &)` | Required for composable nodes |
| Timer | `create_wall_timer` | Period: 1s |
| Callback | `timer_callback` | Parameters: *(none)* |
| Publisher | `create_publisher` | Topic: `component_output`, Type: `std_msgs::msg::String` |
| Registration | `RCLCPP_COMPONENTS_REGISTER_NODE` | Plugin: `component_example::ComponentNode` |

## What Makes It a Component

Unlike regular nodes compiled as executables (`add_executable`), components are compiled as shared libraries (`add_library(... SHARED ...)`). They are loaded into a `ComposableNodeContainer` at runtime via `LoadComposableNode`. The `RCLCPP_COMPONENTS_REGISTER_NODE` macro registers the class as a loadable plugin.

## Callback Signatures

```cpp
// Timer callback — no input parameters
void ComponentNode::timer_callback()
// parameters: []
```

The constructor takes `rclcpp::NodeOptions` (required for component registration), but this is not a callback — it is captured separately as `is_component = True` and constructor signature in the extracted `NodeInfo`.

## Run

```bash
docker compose run --rm dev ros2 launch component_example component_node.launch.py
```

## What to Expect

```
[INFO] [component_node]: ComponentNode has been started
[INFO] [component_node]: Publishing: 'Component tick: 0'
[INFO] [component_node]: Publishing: 'Component tick: 1'
```

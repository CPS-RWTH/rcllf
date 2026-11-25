# parameter_example

Demonstrates the ROS2 parameter pattern: declaring typed parameters with defaults, loading values from a YAML config file, and reacting to parameter changes at runtime.

## Pattern

**Node:** `ParameterNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Parameters | `declare_parameter<T>` | `robot_name` (string), `max_speed` (double), `update_rate` (int), `debug_mode` (bool) |
| Config | YAML file | `config/params.yaml` — loaded via launch file |
| Timer | `create_wall_timer` | Period: 1s |
| Callback | `timer_callback` | Parameters: *(none)* |
| Param callback | `add_on_set_parameters_callback` | Reacts to runtime parameter changes |
| Callback | `on_parameter_change` | Parameters: `parameters` (`vector<rclcpp::Parameter>`) |

## Callback Signatures

```cpp
// Timer callback — no input
void ParameterNode::timer_callback()
// parameters: []

// Parameter change callback — called when any parameter is set
rcl_interfaces::msg::SetParametersResult ParameterNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
// parameters: [
//   CallbackParameter("parameters", "std::vector<rclcpp::Parameter>"),
// ]
```

The parameter change callback is a special ROS2 callback type. The extraction tool detects `add_on_set_parameters_callback` and records the callback name in `NodeInfo`. The input parameter (`parameters`) captures the list of changed parameters.

## Config File

```yaml
# config/params.yaml
parameter_node:
  ros__parameters:
    robot_name: "my_robot"
    max_speed: 2.5
    update_rate: 20
    debug_mode: true
```

## Run

```bash
docker compose run --rm dev ros2 launch parameter_example parameter_node.launch.py
```

## What to Expect

```
[INFO] [parameter_node]: robot_name: my_robot
[INFO] [parameter_node]: max_speed: 2.500000
[INFO] [parameter_node]: update_rate: 20
[INFO] [parameter_node]: debug_mode: true
[INFO] [parameter_node]: Parameter node started.
[INFO] [parameter_node]: Robot: my_robot, Max speed: 2.500000
```

## Headerless node with launch parameters

`headerless_param_node` (`src/headerless_param_node.cpp`) defines its class in the
`.cpp` only, so `colcon lf` builds it through the headerless factory.
`launch/headerless_params.launch.py` gives it `config/e/headerless.yaml` and then an
inline dict; the node logs and publishes `label=inline_label gain=3.500000 count=21` on
`/hl/value`, under `ros2 launch` and in the lfc run alike.

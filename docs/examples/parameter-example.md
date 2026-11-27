# parameter_example

ROS 2 parameters: declaration with defaults, loading from a YAML file in the launch
file, and a callback for changes at run time.

## What it shows

- **Parameters:** `robot_name` (string), `max_speed` (double), `update_rate` (int),
  `debug_mode` (bool).
- **Config:** the launch file loads `config/params.yaml`.
- **Change callback:** `add_on_set_parameters_callback` accepts new values at run time.
- **Timer:** every second the node logs the robot name and maximum speed.

## Run with stock ROS 2

The first start of the dev container does not build this package, so select it with
`ENTRYPOINT_PACKAGES`:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="parameter_example" dev \
  ros2 launch parameter_example parameter_node.launch.py
```

The node prints the values it loaded from `params.yaml`, then one line per second
(trimmed; your timestamps differ):

```text
[parameter_node-1] [INFO] [...] [parameter_node]: robot_name: ros2_lf_robot
[parameter_node-1] [INFO] [...] [parameter_node]: max_speed: 2.500000
[parameter_node-1] [INFO] [...] [parameter_node]: update_rate: 20
[parameter_node-1] [INFO] [...] [parameter_node]: debug_mode: true
[parameter_node-1] [INFO] [...] [parameter_node]: Parameter node started.
[parameter_node-1] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
[parameter_node-1] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
```

## Run under rcllf

Start the dev container with the same selection
(`docker compose run --rm -e ENTRYPOINT_PACKAGES="parameter_example" dev`), then:

```bash
cd packages
colcon lf --package parameter_example && source install/setup.bash
ros2 lf launch parameter_example parameter_node
```

The package has five launch files, so `colcon lf` generates five programs;
`ros2 lf launch` starts the one for `parameter_node.launch.py`. It loads the same
values and binds the timer (trimmed):

```text
Launching LF-controlled system: parameter_example/parameter_node
  Package: lf_generated_parameter_example_parameternodesystem
  Launch file: lf_parameternodesystem.launch.py
...
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: robot_name: ros2_lf_robot
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: max_speed: 2.500000
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: update_rate: 20
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: debug_mode: true
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: Parameter node started.
[ParameterNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (ParameterNode::timer_callback, period 1000000000 ns)
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
[ParameterNodeSystem-1] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
```

## What to look at

- **The values come from the YAML file.** Both runs print `ros2_lf_robot`, the value
  in `config/params.yaml`.
- **What the analyzer read.** In
  `lf-analysis/parameter_example/parameter_node/system.json` the node's `parameters`
  list has four entries, each with its name, type and default (`robot_name` string
  `ros2_lf_robot`, `max_speed` double 2.5, `update_rate` int 20, `debug_mode` bool
  true), and `has_param_callback` is `true`. The top-level `parameter_files` maps
  `src/parameter_example/config/params.yaml` to its parsed content.
- **The other four launch files** (`headerless_params`, `inline_override`,
  `precedence`, `wildcard_scope`) are test cases for how parameter files and inline
  values in a launch file combine (which value wins, which nodes a file applies to).
  Each has its own generated program.

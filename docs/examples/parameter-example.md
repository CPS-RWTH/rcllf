# parameter_example

ROS 2 parameters: declaration, YAML config loading, and runtime change callbacks.

## Pattern

- **Parameters:** `robot_name` (string), `max_speed` (double), `update_rate` (int), `debug_mode` (bool)
- **Config:** `config/params.yaml` loaded via launch file
- **Change callback:** `add_on_set_parameters_callback` for runtime reconfiguration

## What the Extractor Finds

- `NodeInfo.parameters` list with 4 entries (name, type, default_value)
- `NodeInfo.has_param_callback = True`
- `LaunchNodeAction.parameter_files` pointing to `config/params.yaml`
- `SystemGraph.parameter_files` contains parsed YAML content

## Run

```bash
docker compose run --rm dev ros2 launch parameter_example parameter_node.launch.py
```

# system_bringup

Top-level launch orchestration that starts multiple example packages with namespace scoping, conditional inclusion, and parameter overrides.

## Pattern

- **Launch arguments:** `namespace` (default `robot_1`), `use_sim` (default `false`)
- **Namespaced group:** `pubsub_example` and `timer_example` under `robot_1` namespace
- **Root-level includes:** `service_example` and `parameter_example`
- **Conditional node:** `sim_timer` only launched when `use_sim=true`
- **Parameter override:** Passes `system_params.yaml` to parameter_example
- **Acknowledged hazard:** the optional `action_full_example` include (`use_action_full`, off
  by default) sends action results from a thread, which RCLLF-H011 refuses.
  `lf/system_bringup.execution.yaml` acknowledges the code so the rest of the system
  generates. Launched with `use_action_full:=true`, the runtime refuses those sends.

## What the Extractor Finds

- `LaunchGraph` with nested includes and groups
- `LaunchArgument` entries for `namespace` and `use_sim`
- `LaunchGroup` with `namespace="robot_1"` containing namespaced nodes
- No `sim_timer` node: its `IfCondition(use_sim)` is false with the default, so it is
  left out, as are the `use_multi_service`, `use_action_full` and `use_reentrant_fanout`
  includes (all default `false`)
- `parameter_files` dict with parsed YAML content from `system_params.yaml`
- Topic names resolved with namespace prefixes (e.g. `/robot_1/sensor_data`)

## Run

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup" dev \
  ros2 launch system_bringup system_bringup.launch.py
```

`system_bringup` is not in the container's default build, so the first command names it.

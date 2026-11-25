# system_bringup

Top-level launch orchestration package. Contains no nodes — only a launch file that starts multiple example packages together.

## Pattern

This package exercises every advanced launch feature:

| Feature | How It's Used |
|---------|--------------|
| `DeclareLaunchArgument` | `namespace` (default: `robot_1`), `use_sim` (default: `false`) |
| `GroupAction` + `PushRosNamespace` | All nodes run under the declared namespace |
| `IfCondition` | Conditional node inclusion based on `use_sim` |
| `IncludeLaunchDescription` | Includes launches from `pubsub_example`, `timer_example`, `service_example`, `parameter_example` |
| `launch_arguments` pass-through | Namespace forwarded to child launches |
| Topic remappings | e.g., `sensor_data` → `/robot_1/sensor_data` |

## What Gets Launched

```
system_bringup.launch.py
├── pubsub_example/bringup.launch.py
│   ├── publisher_node     → /robot_1/publisher_node
│   └── subscriber_node    → /robot_1/subscriber_node
├── timer_example/timer_node.launch.py
│   └── timer_node         → /robot_1/timer_node
├── service_example/bringup.launch.py
│   ├── service_server     → /robot_1/service_server
│   └── service_client     → /robot_1/service_client
└── parameter_example/parameter_node.launch.py (if use_sim=false)
    └── parameter_node     → /robot_1/parameter_node
```

## Nodes and Callbacks

This package has no source code. For the callback signatures of each included node, see the respective package READMEs:

- [pubsub_example](../pubsub_example/README.md) — `timer_callback` (no params), `topic_callback` (`SensorData::SharedPtr msg`)
- [timer_example](../timer_example/README.md) — `timer_callback` (no params)
- [service_example](../service_example/README.md) — `handle_process_data` (`request`, `response`)
- [parameter_example](../parameter_example/README.md) — `on_parameter_change` (`vector<Parameter>`)

## Run

```bash
docker compose run --rm dev ros2 launch system_bringup system_bringup.launch.py
```

## What to Expect

All nodes start under the `/robot_1` namespace. Topics appear as `/robot_1/sensor_data`, services as `/robot_1/process_data`, etc.

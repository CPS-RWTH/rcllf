# system_bringup

A top-level launch file that starts other example packages together: a namespace for
some of them, launch arguments that switch parts on and off, and a parameter file
handed to an included launch file. rcllf turns the whole launch tree into one LF
program.

## What it shows

`launch/system_bringup.launch.py` declares five launch arguments: `namespace`
(default `robot_1`), `use_sim`, `use_multi_service`, `use_action_full` and
`use_reentrant_fanout` (all default `false`). With the defaults it starts six nodes:

- **In a namespaced group** (`PushRosNamespace(namespace)`): `pubsub_example`
  (publisher and subscriber) and `timer_example`, so the topic becomes
  `/robot_1/sensor_data`.
- **At the root:** `service_example` (server and client) and `parameter_example`,
  included with a `params_file` argument that points to
  `config/system_params.yaml` (`robot_name: "system_robot"`, `max_speed: 3.0`).
- **Only with an argument set to `true`:** a `sim_timer` node (`use_sim`), and the
  `multi_service_example`, `action_full_example` and `reentrant_fanout_example`
  launch files.

## Run with stock ROS 2

The first start of the dev container does not build this package, and the launch file
includes `parameter_example`, which it does not build either. Select both:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup parameter_example" dev \
  ros2 launch system_bringup system_bringup.launch.py
```

With `system_bringup` alone the launch stops with
`package 'parameter_example' not found`. With both, six processes start and log
(trimmed; your timestamps and order differ):

```text
[timer_node-3] [INFO] [...] [robot_1.timer_node]: Timer node started.
[service_server_node-4] [INFO] [...] [service_example.service_server]: Service server ready on 'process_data'
[parameter_node-6] [INFO] [...] [parameter_node]: robot_name: ros2_lf_robot
[parameter_node-6] [INFO] [...] [parameter_node]: Parameter node started.
[publisher_node-1] [INFO] [...] [robot_1.publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[subscriber_node-2] [INFO] [...] [robot_1.subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[timer_node-3] [INFO] [...] [robot_1.timer_node]: Timer tick: 0
[parameter_node-6] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
[service_client_node-5] [INFO] [...] [service_example.service_client]: Sending request: method='sum', 4 values
[service_server_node-4] [INFO] [...] [service_example.service_server]: Request: method='sum', 4 values
[service_client_node-5] [INFO] [...] [service_example.service_client]: Result: 6.00, success=true, message='Computed sum'
```

!!! warning "Known limitation: the parameter override does not reach parameter_example"
    The parameter node prints `ros2_lf_robot`, not `system_robot`, in both runs.
    `parameter_node.launch.py` does not declare a `params_file` argument and always
    loads its own `config/params.yaml`, so the `params_file` value that
    `system_bringup` passes is ignored. To run the node with the system values under
    stock ROS 2, load the file directly:

    ```bash
    ros2 run parameter_example parameter_node --ros-args \
      --params-file $(ros2 pkg prefix system_bringup)/share/system_bringup/config/system_params.yaml
    ```

    It then prints `robot_name: system_robot` and
    `Robot: system_robot, Max speed: 3.000000`.

## Run under rcllf

Start the dev container with the same selection
(`docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup parameter_example" dev`),
then:

```bash
cd packages
colcon lf --package system_bringup && source install/setup.bash
ros2 lf launch system_bringup system_bringup
```

The analysis prints `6 nodes, 1 topics, 1 services, 0 actions`. All six nodes run in
one generated program; seven callbacks are bound as LF reactions, and the nodes
log the same lines as in the stock run (trimmed):

```text
Launching LF-controlled system: system_bringup/system_bringup
  Package: lf_generated_system_bringup_systembringup
  Launch file: lf_systembringup.launch.py
...
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/robot_1/sensor_data', take_mode=skip timeout_ms=100)
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 2 (TimerNode::timer_callback, period 1000000000 ns)
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound service server RID 3 (ServiceServerNode::handle_process_data, service 'process_data')
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound srv_send RID 4 (ServiceClientNode::timer_callback, period 1000000000 ns)
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound service client RID 5 (ServiceClientNode::process_data_response_callback, service 'process_data')
[SystemBringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 6 (ParameterNode::timer_callback, period 1000000000 ns)
[SystemBringup-1] [INFO] [...] [robot_1.timer_node]: Timer tick: 0
[SystemBringup-1] [INFO] [...] [robot_1.publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[SystemBringup-1] [INFO] [...] [parameter_node]: Robot: ros2_lf_robot, Max speed: 2.500000
[SystemBringup-1] [INFO] [...] [service_example.service_client]: Sending request: method='sum', 4 values
[SystemBringup-1] [INFO] [...] [robot_1.subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[SystemBringup-1] [INFO] [...] [service_example.service_server]: Request: method='sum', 4 values
[SystemBringup-1] [INFO] [...] [service_example.service_client]: Result: 6.00, success=true, message='Computed sum'
```

The `Bound` lines appear in any order; the RIDs, not the order, identify the
reactions. All timers fire at once at the start tag, because an LF timer starts at
offset 0 (see [timer_example](timer-example.md)).

## What to look at

- **The launch tree the analyzer read.** In
  `lf-analysis/system_bringup/system_bringup/system.json`, `launch_graph.root` is
  `system_bringup.launch.py`: its `arguments` list holds the five launch arguments
  with their `default_value`, and its `includes` and `groups` hold the included
  launch files, one group with `namespace` `robot_1` around the pubsub and timer
  includes.
- **What the defaults leave out.** The analysis evaluates each condition with the
  default arguments. `use_sim` is `false`, so `sim_timer` is not in `nodes`, and
  the `use_multi_service`, `use_action_full` and `use_reentrant_fanout` includes
  are left out. Pass `--launch-arg use_sim:=true` to `colcon lf-analyze` to change
  that (see the [CLI Reference](../reference/cli.md#colcon-lf-analyze)).
- **Resolved names.** `nodes` has six keys, each the node's full name with its
  namespace, for example `/robot_1/publisher_node` and
  `/service_example/service_server`, and `communication.topics` lists
  `/robot_1/sensor_data` with the namespace already applied.
- **An acknowledged hazard.** The optional `action_full_example` server sends results
  from a thread of its own, which
  [RCLLF-H011](../reference/diagnostic-codes.md#rcllf-h011) refuses.
  `lf/system_bringup.execution.yaml` lists `RCLLF-H011` under
  `acknowledged_hazards`, so that optional include does not stop generation of the
  rest of the system.

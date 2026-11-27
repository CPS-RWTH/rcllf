# fusion_example

Two sensors fused by unmodified `message_filters` synchronizers, the form rcllf
admits.

## Pattern

**Sensor nodes** (`SensorANode`, `SensorBNode`):

- `SensorANode` publishes `custom_interfaces::msg::SensorData` on `fusion/a` every 30 ms
- `SensorBNode` publishes on `fusion/b` every 50 ms
- Each message is stamped with the node clock, which is the LF logical time under rcllf

**Fusion nodes** (`ExactFusionNode`, `ApproxFusionNode`):

- Two `message_filters::Subscriber` members, subscribed with `subscribe(this, "fusion/a")`
  and `subscribe(this, "fusion/b")`
- `ExactFusionNode` uses an `ExactTime` synchronizer and publishes each pair on
  `fusion/exact_pairs`
- `ApproxFusionNode` uses an `ApproximateTime` synchronizer and publishes on
  `fusion/approx_pairs`

## Communication

```text
SensorANode --[fusion/a: SensorData]--> ExactFusionNode  --[fusion/exact_pairs]-->
SensorBNode --[fusion/b: SensorData]--> ApproxFusionNode --[fusion/approx_pairs]-->
```

## What the Extractor Finds

- 4 nodes and 4 topics. `fusion/a` and `fusion/b` each have one publisher and
  both fusion nodes as subscribers
- Each fusion node has one subscription per `Subscriber` input, so its
  synchronizer callback runs inside the LF reaction of the input that completes a set
- No diagnostic (`diagnostics` is empty). Any other `message_filters` form is refused with RCLLF-H003,
  see the [Support Matrix](../features/support-matrix.md)

## Run

```bash
docker compose run --rm dev ros2 launch fusion_example bringup.launch.py
```

Under rcllf, inside the dev container (`docker compose run --rm dev`):

```bash
cd packages && source install/setup.bash
colcon lf --package fusion_example && source install/setup.bash
ros2 lf launch fusion_example bringup
```

`colcon lf` generates and builds one system per launch file. The two launch files that
start a single fusion node carry the line `# rcllf: not-a-system`, so the analysis skips
them; the summary at the end of the run says so:

```text
System                                       Generation              Built  Reason
fusion_example/bringup                       generated               yes
fusion_example/sensor_a_node                 generated               yes
fusion_example/sensor_b_node                 generated               yes
fusion_example/approx_fusion_node.launch.py  skipped (not-a-system)  no     marked rcllf: not-a-system
fusion_example/exact_fusion_node.launch.py   skipped (not-a-system)  no     marked rcllf: not-a-system
```

The launched system prints one line per matched pair (timestamps shown as `[...]`):

```text
[Bringup-1] [INFO] [...] [approx_fusion_node]: PAIR approx a_seq=0 a_ns=... b_seq=0 b_ns=...
[Bringup-1] [INFO] [...] [exact_fusion_node]: PAIR exact a_seq=0 a_ns=... b_seq=0 b_ns=...
[Bringup-1] [INFO] [...] [approx_fusion_node]: PAIR approx a_seq=2 a_ns=... b_seq=1 b_ns=...
```

A run of about 7 s printed 48 `PAIR exact` and 142 `PAIR approx` lines. The same-tag
order of the two fusion nodes' lines can change from run to run: they are not connected,
so LF may run them in parallel (see
[Time and Coordination](../design/time-and-coordination.md)).

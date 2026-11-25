# fusion_example

`message_filters` synchronization in two policies, over the same two sensor
topics.

| Node | Role |
|---|---|
| `SensorANode` | publishes `custom_interfaces/msg/SensorData` on `fusion/a` every 30 ms |
| `SensorBNode` | publishes `SensorData` on `fusion/b` |
| `ExactFusionNode` | `ExactTime` synchronizer on `fusion/a` and `fusion/b`, publishes `fusion/exact_pairs` |
| `ApproxFusionNode` | `ApproximateTime` synchronizer on the same inputs |

`bringup.launch.py` starts both sensors and both fusion nodes; each node also
has its own launch file. `lf/approx_fusion_node.execution.yaml` and
`lf/exact_fusion_node.execution.yaml` configure the two fusion nodes.

The four node sources can also be built as one component library, in which
case `main()` is compiled out (`FUSION_COMPONENT_ONLY`); the standalone
executables keep it. See `docs/examples/fusion-example.md` for what the
analyzer reports.

```bash
colcon lf-analyze --package fusion_example --launch-file bringup.launch.py
```

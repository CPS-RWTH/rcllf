# packages/src/fixtures/: test fixtures, not examples

The packages in this folder are **test fixtures**. The colcon_lf test suite and the
feature playground cases (`colcon_lf/features/modules/`) parse, analyze and generate
them to pin one analyzer or generator behavior each: a launch-file feature, a
lifecycle base-call shape, a QoS hazard, a tf2 pattern, a refusal of the
resolved-model gate. They are not documented, not maintained as user examples and
may show patterns you should not copy (several exist to be refused).

For examples to learn from, start with `packages/src/` and the
[examples documentation](../../../docs/examples/index.md).

| Package | Loaded by |
|---|---|
| `blocking_service_client_example` | playground case `services.blocking_client` |
| `clock_fixture` | playground case `clock.ros_time_sleep` (`ros_sleep` launch) |
| `dynamic_topic_example` | playground case `topics.dynamic_name`, clang parser tests |
| `helper_method_example` | clang parser tests |
| `intra_process_example` | playground case `ipc.intra_process` (refused, RCLLF-H017) |
| `launch_features_example` | launch parser, resolved-model and generation tests |
| `lifecycle_component_example` | playground case `lifecycle.base_call_chaining`, clang parser tests |
| `lifecycle_rebind_example` | parameter twin-topic tests (`reader_param` launch) |
| `main_only_example` | clang parser tests |
| `qos_hazard_example` | playground QoS cases, message_filters scan tests |
| `sentry_pipeline` | resolved-model tests (gate, resolve, canonical form, staleness fingerprint) |
| `tf_broadcaster_example` | tf2 scan tests |
| `tf_listener_example` | tf2 scan and resolved-model gate tests (refused direct lookup) |

colcon finds them like any package under `packages/`: `colcon build` builds them and
`colcon test` runs their lint tests.

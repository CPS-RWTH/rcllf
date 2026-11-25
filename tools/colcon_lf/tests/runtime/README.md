# Runtime smoke layer

Builds real lfc-generated examples with `docker/lf_build.sh` and runs the
actual binary, asserting the runtime output — not just that the static
playground (`tests/playground/test_playground_cases.py`) accepted the
launch file. Covers: `pubsub_example` (publish/subscribe value
correspondence), `lifecycle_example` (reaches `active` and actually
publishes — a past regression), `service_example` (request/response
values match the server's own computation), plus a clean-shutdown check
(SIGINT to the launch process group must end in `process has finished
cleanly`, never `process has died`) on every case.

Run inside the dev container after building the examples:

```bash
docker/lf_build.sh pubsub_example
cd tools/colcon_lf && python -m pytest tests/runtime -v -m runtime
```

A topic edge is admitted to publication correspondence only
with a burst bound k. Since W3 slice S3 the analyzer proves k from the publish
call sites of each publishing callback, so pubsub, service, corr_loop and
corr_relay build without an exec-config. corr_bringup's `on_tick` publishes
/corr/a in a loop (scenario `many`), so `corr_many_execution.yaml` declares
k = 3 for corr_publisher, and `corr_many_k2_execution.yaml` is the negative
case (k = 2 under `many`, `burst_exceeded` at reconcile). The CI
entries in `.github/workflows/ci.yml` name the file per case.

`RCLLF_REQUIRE_RUNTIME_SMOKE=1` turns a not-yet-built case's skip into a
collection error (CI always sets this — same pattern as
`RCLLF_REQUIRE_ROS_TESTS`).

## Limits of this layer

This is a **log-level** smoke layer: it parses `RCLCPP_INFO` lines the
example nodes already print and checks those strings against values
derived from the source formulas. It does not inspect the LF trace, does
not check reaction dispatch order or logical-time semantics, and does not
distinguish "the wrapper forwarded a value" from "the wrapper forwarded
*the corresponding* value" at anything finer than what the log already
prints. Identity/dispatch/provider-completeness correspondence — the part
that needs actual trace evidence, not log text — is covered by the
traced runtime tests.

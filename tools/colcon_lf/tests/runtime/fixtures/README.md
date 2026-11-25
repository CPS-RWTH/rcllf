# Runtime smoke fixtures

Every file here comes from a **real capture**, never hand-written (fixtures are recorded,
not authored). The one negative fixture, `pubsub_bringup_mismatch.log`, is a real capture
with one line altered on purpose (see below).

## `pubsub_bringup_real.log`

A real ~6.5s run of `pubsub_example`'s bringup, built with:

```bash
docker/lf_build.sh pubsub_example
```

then captured and stopped with SIGINT to its process group (the same
sequence `runtime_support.launch_and_capture` automates):

```bash
source /opt/ros/jazzy/setup.bash
cd packages && source install/setup.bash
setsid ros2 launch lf_generated_pubsub_example_bringup lf_bringup.launch.py > out.log 2>&1 &
PID=$!
sleep 6
PGID=$(ps -o pgid= -p $PID | tr -d ' ')
kill -INT -$PGID
```

Captured 2026-11-17 in a container started with `--hostname host01` and
`HOME=/tmp/home`: 12 published messages, 12 heard messages, 1:1
correspondence, ends with `process has finished cleanly`.

## `pubsub_bringup_mismatch.log`

A byte-for-byte copy of `pubsub_bringup_real.log` with exactly one line
altered — the third "I heard" line's `temp` changed from `22.0` to `99.0`
(`sed -i 's/I heard: temp=22.0, humidity=42.0, pressure=1015.0/I heard:
temp=99.0, humidity=42.0, pressure=1015.0/' pubsub_bringup_mismatch.log`,
applied to the copy above). This is the negative fixture for
`test_negative_fixture_catches_value_mismatch` in `test_runtime_smoke.py`:
it proves `find_pubsub_mismatches` reports that exact altered value (heard
position 2, value `(99.0, 42.0, 1015.0)`) rather than passing silently or
failing for an unrelated reason.

## `corr_stale_real.log`

The `corr_pub:` and `corr_sub:` lines of one real `stale`-scenario run of
`correspondence_fixture`'s `corr_bringup`, built with
`LF_EXEC_CONFIG=tools/colcon_lf/tests/runtime/corr_many_execution.yaml
docker/lf_build.sh correspondence_fixture` and captured in the dev container on
2026-10-22 with `CORR_SCENARIO=stale RCLLF_TRACE_CALLBACKS=1` through
`runtime_support.launch_and_capture` (the `test_fixture_dispatches_exactly_what_was_published[stale]`
run), then filtered with `re.search(r"corr_(pub|sub): ", line)`. Ten ticks,
both subscriber nodes heard `t1k1`..`t10k1` in tick order, and at several
ticks `corr_sub_2` logged before `corr_sub_1`. The negative test
`test_stale_check_fails_on_a_wrong_sample_not_on_node_order` derives its wrong
logs from this one in the test body.

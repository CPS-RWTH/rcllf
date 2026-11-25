# tf2_acq_example

Showcase of tf2 acquisition-time lookups under rcllf (see
[docs/features/tf2.md](../../../docs/features/tf2.md)).

- `tf_source_node` broadcasts `world -> sensor_link` on `/tf` every 20 ms with
  `x = t` (1 m/s, t in seconds since its first tick) for ticks 0 to 15, a
  `world -> held_link` (y = 1 m) only at ticks 0 and 10, and one static
  `world -> static_link` (y = 2 m) on `/tf_static` at tick 0.
- `acq_sampler_node` owns a `tf2_ros::Buffer`, a `TransformListener` and an
  `rcllf_tf2::AcquisitionLookup` (wait bound 55 ms, `on_timeout` reject), and
  asks for the transforms of cases T1 to T7 from a 30 ms timer.

Under rcllf both timers start at the same tag t0, the listener's `/tf` and
`/tf_static` reach the Buffer in LF reactions one microstep after the
broadcaster's tag, and every outcome is a function of the tag.

```bash
colcon lf --package tf2_acq_example
ros2 launch lf_generated_tf2_acq_example_bringup lf_bringup.launch.py
```

## Expected values (derived by hand before the first converted run)

Times are ms after t0. `/tf` samples of sensor_link exist at 0, 20, ..., 300
with x = stamp in seconds; tf2 interpolates linearly between the two samples
around a stamp, and accepts a stamp equal to the newest sample.

| Case | Request tag | Stamp | Derivation | Outcome |
|---|---|---|---|---|
| T1 | 60 | 30 | sensor samples 20 (0.02) and 40 (0.04) are in the Buffer; 0.02 + (0.04 - 0.02) x 10/20 | ok at 60, x = 0.030 |
| T2 | 90 | 80 | stamp equals a sample (80 arrived at 80 + 1 microstep) | ok at 90, x = 0.080 |
| T5 | 90 | 90 | held_link has only stamp 0; its next sample (200) is after the deadline 90 + 55 = 145 | reject at 145 |
| T3 | 120 | 125 | newest sample 120 (same tag, next microstep) < 125; the 140 sample arrives at 140 + 1 microstep; 0.12 + 0.02 x 5/20 | ok at 140, x = 0.125, logical wait 20 |
| T6 | 150 | 150 | static_link is static, valid at every stamp | ok at 150, (0, 2) |
| T7 | 180 | 170 | ghost_link is never published; at the deadline 235 tf2 reports an unknown frame, not an extrapolation | failure at 235 |
| T4 | 330 | 320 | the last sensor sample is 300 (tick 15); nothing newer arrives by 385 | reject at 385 |

`corpus/t1_t7_expected.tsv` holds this table; `corpus/t1_t7_events.tsv` is the
same schedule in LF delivery order, which the O1 oracle
(`test/test_o1_oracle.cpp`, a plain `tf2::BufferCore` replay without the
adapter) turns into the same table. The converted run is checked against it by
`tools/colcon_lf/tests/runtime/test_runtime_tf2_acq.py` (20 runs, byte-identical
outcome streams) with the reader in `tools/lf_eval/lf_eval/tf2_acq/`.

## Constants

| Constant | Value | Provenance |
|---|---|---|
| `/tf` period | 20 ms | showcase choice: T3 waits exactly one period |
| wait bound | 55 ms | showcase choice: deadlines at 30k + 55 ms never coincide with a `/tf` tag (multiples of 20) or a request tag (multiples of 30) |
| velocity | 1 m/s along x | showcase choice: x equals the stamp in seconds |
| sample period | 30 ms | showcase choice: not a multiple of 20 ms, so stamps fall between and on `/tf` samples |

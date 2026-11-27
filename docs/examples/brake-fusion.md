# brake_fusion

The showcase and the last step of the examples: an emergency-braking system
whose unchanged ROS 2 node code runs once as stock ROS 2 and once as an rcllf
program. With rcllf, the vehicle stops at the same position in every run.

## Scenario

A vehicle drives at 15 m/s towards an obstacle 40 m ahead, on a MuJoCo plant
stepped every 1 ms (full braking: 8 m/s^2).

- `simulator` steps the plant, publishes `/sim/state` and applies `/brake_cmd`
- `radar` and `radar2` (every 20 ms; `radar2` reads 0.5 m long) and `camera`
  (every 30 ms) measure the range; `wheel_speed` (every 10 ms) the speed
- `radar_tracker` smooths both radar streams into `/radar/track`
- `fusion` pairs radar track and camera samples with a stock `message_filters`
  ApproximateTime synchronizer and publishes the mean range
- `controller` (every 25 ms) computes the time to collision and commands a
  partial brake below 2.45 s and a full brake below 1.141 s

## Communication

```text
simulator --/sim/state--> radar, radar2, camera, wheel_speed
radar, radar2 --/radar/range--> radar_tracker --/radar/track--> fusion
camera --/camera/range--> fusion --/fused/range--> controller
wheel_speed --/wheel/speed--> controller --/brake_cmd--> simulator
```

## What the comparison shows

| | stock (`t2_separate.launch.py`) | rcllf (`t2_approx.launch.py`) |
|---|---|---|
| processes | one per node | one LF program generated from a component container |
| sample stamps | system time | logical time |
| order of the two radars, pairs formed, input of each controller tick | whatever scheduling and transport timing give | fixed by logical time |
| stopping position | safe, but varies from run to run | safe, identical in every run |

The project's runtime test checks both versions against an expected value derived
from the plant and controller parameters alone. A stepped model with exact sensing stops at 35.205 m.
Swept over the sensing errors the controller is designed for, the model gives a
band of 34.64 to 36.49 m that every run must stop in. The rcllf run must also
stop at the same position three times in a row.

## Run

In the dev container (the first start already built `brake_fusion`), from
`packages/`. Run the two versions side by side, in two shells, or one after the other:

<div class="rlf-snippet-grid" markdown>

<div markdown>

**Stock ROS 2** (one process per node)

```bash
source install/setup.bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch brake_fusion t2_separate.launch.py
```

</div>

<div markdown>

**rcllf** (one generated LF program)

```bash
source install/setup.bash
colcon lf-analyze --package brake_fusion
colcon lf-generate \
  --system-json lf-analysis/brake_fusion/t2_approx/system.json \
  --exec-config src/brake_fusion/config/t2/subs.execution.yaml
colcon build --packages-select lf_generated_brake_fusion_t2approx
source install/setup.bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 launch lf_generated_brake_fusion_t2approx lf_t2approx.launch.py
```

</div>

</div>

If you run both at once, give each shell its own `ROS_DOMAIN_ID` so the two
simulators do not hear each other.

On the rcllf side, `colcon lf-analyze` prints two warnings and `colcon lf-generate`
two `[INFO]` lines. All of them are expected for this example:

```text
Warning: n4_cb_state_callback -> n7_cb_fused_callback: spline size > 1 not supported
Warning: n6_cb_settle -> n7_cb_fused_callback: spline size > 1 not supported
...
    ! [INFO] RCLLF-H001 subscription '/radar/range' on node 'radar_tracker': best_effort QoS reliability on admitted correspondence edge(s) ...
    ! [INFO] RCLLF-H001 subscription '/radar/track' on node 'fusion': best_effort QoS reliability on admitted correspondence edge(s) ...
```

The first two come from Graphviz while it draws the system graph and do not affect the
program.
[RCLLF-H001](../reference/diagnostic-codes.md#rcllf-h001) notes that the radar topics
use best-effort QoS: a dropped sample is reported as missing, not silently skipped.

### Compare the stops

Each run prints one line when the vehicle stands still, then keeps running until you
stop it with Ctrl-C:

```text
BF_STOP position=<m> step=<ms>
```

Run each version three times and compare the lines:

- **Stock**: the position can differ from run to run, inside the band of 34.64 to
  36.49 m that the smoke test derives from the plant and controller parameters.
- **rcllf**: the same line in every run, `BF_STOP position=35.38489000000461 step=3743`
  (the value the project's runtime test checks), inside the same band.

### When a run exits early (`worker_lag_exceeded`)

The rcllf program checks that it keeps up with physical time. When a run falls more
than 200 ms behind, it ends with exit code 3 and logs `worker_lag_exceeded` with the
worst lag. Such a run establishes nothing about the stop: the program did not keep
up with logical time, so its `BF_STOP` line, if it printed one, is evidence neither
for nor against the repeatable stop. Leave it out of the
comparison and run again; on a loaded or small machine, close other work first.
The project's runtime test does the same: it reports a lag-only exit 3 as a warning
and checks the stop of the clean runs only.

The rcllf program runs two LF worker threads on every host. `t2_approx.launch.py`
pins that count with its `rcllf_workers` launch argument, which `colcon lf-generate`
turns into the program's `workers` target property ([Execution
config](../features/execution-config.md)). Without the pin, a many-core desk host
runs eight workers: on an idle 24-core host, 2 of 10 such runs fell more than
200 ms behind physical time and ended with exit code 3 (`worker_lag_exceeded`),
while 10 of 10 runs with two workers finished. To try another count for
one run, prefix the launch with `RCLLF_LF_ARGS="-w N"`, which overrides the pin.

Known limitation: the pin fixes the idle many-core case, not small shared
machines. On a machine with 2 virtual CPUs the 2-worker program sometimes falls more
than 200 ms behind and exits with code 3 (`worker_lag_exceeded`), and several runs in
a row can do so. Run it on a machine with more cores, or rerun until a run finishes.

The package README (`packages/src/brake_fusion/README.md`) describes the execution
configs and the tests.

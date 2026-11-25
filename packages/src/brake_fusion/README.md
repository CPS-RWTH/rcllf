# brake_fusion

An emergency-braking showcase: the same ROS 2 node code run once as stock ROS 2
and once as an rcllf program. With rcllf, the vehicle stops at the same position
in every run.

## The scenario

A vehicle drives at 15 m/s towards an obstacle 40 m ahead. The plant is a
MuJoCo model, one body on a slide joint, stepped every 1 ms. Full braking
decelerates it at 8 m/s^2.

| node | what it does |
|---|---|
| `simulator` | steps the MuJoCo plant every 1 ms, publishes `/sim/state`, applies `/brake_cmd` |
| `radar`, `radar2` | measure the range every 20 ms; `radar2` reads 0.5 m long (a second mount with a known bias) |
| `camera` | measures the range every 30 ms, after 2.5 ms of simulated detection work |
| `wheel_speed` | publishes the vehicle speed every 10 ms |
| `radar_tracker` | smooths both radar streams into `/radar/track` |
| `fusion` | pairs radar track and camera samples with a stock `message_filters` ApproximateTime synchronizer, publishes the mean range |
| `controller` | every 25 ms: time to collision from the last two fused samples (checked against the wheel speed); partial brake (0.39) below 2.45 s, full brake below 1.141 s |

Every node writes a JSON-lines log when `BF_LOG_DIR` is set (one
`<node>.jsonl` per node). The simulator also prints one line when the vehicle
stands still:

```text
[simulator]: BF_STOP position=<metres from the start> step=<ms>
```

## Stock versus rcllf

| launch file | runs as |
|---|---|
| `t2_separate.launch.py` | stock ROS 2, one process per node |
| `t2_approx.launch.py` | the rcllf input: all nodes in one component container (`t2_container.launch.py` with the ApproximateTime fusion) |

The node sources are identical in both. What differs is who decides the order
of events.

- **Stock ROS 2.** Each sample carries the system time it was taken, and the
  executors process samples in arrival order. Which radar sample reaches the
  tracker first, which pair the synchronizer forms, and which fused range the
  controller's tick reads all depend on scheduling and transport timing. The
  braking decision therefore shifts by a few milliseconds from run to run, and
  so does the stopping position.
- **rcllf.** The analyzer turns the container launch file into a Lingua Franca
  program. Timers and messages become events at logical times. Samples carry
  the logical time as their stamp, the two radars arrive in a fixed order, and
  every reaction runs in the same order in every run. The controller reads the
  same inputs at every tick, so the vehicle stops at the same position, to the
  last digit, in every run.

Both versions stop safely short of the obstacle. Only the rcllf version stops
at the same place every time.

## Run it

Inside the dev container (see Getting Started), from the workspace's
`packages/` directory, after `colcon build`:

Stock:

```bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 launch brake_fusion t2_separate.launch.py
```

rcllf (analyze, generate with the shipped execution config, build, launch):

```bash
colcon lf-analyze --package brake_fusion
colcon lf-generate --system-json lf-analysis/brake_fusion/t2_approx/system.json \
  --exec-config src/brake_fusion/config/t2/subs.execution.yaml
colcon build --packages-select lf_generated_brake_fusion_t2approx
source install/setup.bash
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 launch lf_generated_brake_fusion_t2approx lf_t2approx.launch.py
```

Watch for the `BF_STOP` line, then stop the launch with Ctrl-C. Run each one a
few times and compare the positions.

`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` keeps Fast DDS on UDP. With the default
shared-memory transport, stock launches in the dev container can hang before
the first sample.

## Execution configs

`config/t2/` holds the rcllf execution configs. `subs.execution.yaml` is the
default above. It sets a 15 ms take timeout on every admitted topic edge and
declares the 1 ms input period of the `/sim/state` subscriptions.
`subs.fed.execution.yaml` is the same setting for a program split into federates.

## Tests

`colcon test --packages-select brake_fusion` runs the node-level tests: the
simulator's pacing and collision event, the brake actuation delay, the startup
timeout of a silent wheel-speed stream, a lost publication, the
ApproximateTime sample ledger, the closing-speed check, the radar tracker, and
the supervised plant timeout.

The runtime smoke `tools/colcon_lf/tests/runtime/test_runtime_brake_fusion.py`
launches both versions and checks the stopping position. The expected value is
derived from the plant and controller parameters alone: a stepped model with
exact sensing stops at 35.205 m. The model is then swept over the sensing
errors the controller is designed for, which gives the band every run must
stop in. The rcllf run must also stop at the same position three times in a
row, and at the position recorded from a real run.

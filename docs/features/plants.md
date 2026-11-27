# Plants

A plant is the simulated physics a closed-loop rcllf system drives, for example the
vehicle of the `brake_fusion` example. rcllf runs every plant behind one boundary, the
`rcllf_plant` package. The application node keeps its own logic and its own clock. The
plant only advances when the node asks for exactly one step, and every step leaves one
log line with a hash of the full plant state. Two runs with the same inputs can then
be compared step by step from their logs alone.

Read this page when you write a node that drives a physics engine, or when you need to
check that two runs of a plant were identical.

## The boundary

| Piece | What it does |
|---|---|
| `rcllf_plant::PlantAdapter` | The interface a backend implements: `describe()`, `reset()`, `step()`, `probe()`. A backend is constructed paused. |
| `rcllf_plant::Transactor` | The only caller of a backend. It runs the backend on its own plant thread, checks every request, and checks every result. |
| `rcllf_plant::MujocoBackend` | MuJoCo 3.14.0 with the `brake_fusion` vehicle (one body on a slide joint, 1 ms step). |

A node does three things:

1. It creates a `Transactor` around a backend.
2. It calls `reset()` once with an episode number, an explicit seed and the initial state.
   An unseeded reset is refused. So is an initial state with a value that is not finite or
   that lies below the backend's `Descriptor::state_min`. The MuJoCo backend sets a
   minimum velocity of 0: the plant is a vehicle that brakes while moving forward, the
   brake only opposes forward motion, and a reported velocity under 1e-9 m/s locks to 0.
   A reset with velocity -1 would otherwise report a stop after one unbraked step. The refusal is
   an `std::invalid_argument` naming the value, and the Transactor stays usable.
3. It calls `step()` once per grid step with `step_id` = previous + 1 and `logical_ns` = `step_id × step_ns`.

The Transactor answers each step with one of these statuses:

| Status | When |
|---|---|
| `OK` | The backend advanced exactly one step. |
| `STALE` | The request is refused before the backend is touched. That happens for a wrong episode, a foreign `step_ns`, a skipped or repeated `step_id`, a `logical_ns` off the grid, or input channels that do not match the descriptor. |
| `MISMATCH` | The backend's step counter is not `step_id`, or its own clock is more than half a step from `logical_ns`. |
| `TIMEOUT` | The step did not finish within `plant_step_timeout_ms`. |
| `BACKEND_FAULT` | The backend threw. |

Any status other than `OK` and `STALE` leaves the Transactor closed: it touches the
backend no more. A node publishes nothing from such a step. `brake_fusion`'s simulator
logs `plant_timeout` or `plant_fault` and leaves with `std::quick_exit(3)`. It does not
call `exit()`, because a destructor would wait for a plant thread that may never return.

Half a step is the largest clock tolerance that still tells one step from zero steps
or two. It absorbs the rounding of a floating-point engine clock. For MuJoCo's `data->time`
that rounding stays far below a nanosecond per step.

## The simulator's plant log

`simulator_node` writes one `plant` line at start and one `plant_step` line per step
into `simulator.jsonl` in the directory named by `BF_LOG_DIR`. Without that variable
no file is written. Each line also starts with `gs` and `wall_ns`, a sequence number
and the wall-clock time of the line:

```json
{"gs":0,"wall_ns":1795772496242489322,"ev":"plant","plant":"mujoco","mujoco_version":"3.14.0","timestep":0.001,"episode":1,"seed":0,"plant_step_timeout_ms":1000,"fault":"","state_hash":"886aa3b9613e7d2b","status":"OK"}
{"gs":2,"wall_ns":1795772496248048455,"ev":"plant_step","step":1,"state_hash":"d8e3366ae9f05787","output_hash":"d8e3366ae9f05787","backend_steps":1,"clock_drift_ns":0,"plant_wall_ns":69049}
```

The hashes are FNV-1a 64 over the IEEE bits of each double. The state hash covers
MuJoCo's qpos and qvel. The output hash covers what the step returned.

| Parameter | Default | Meaning |
|---|---|---|
| `plant` / `$BF_PLANT` | `mujoco` | Only `mujoco` is accepted. Any other value stops the node at start. |
| `plant_step_timeout_ms` | `1000` | The supervised step timeout. One MuJoCo step of this model takes microseconds, and the worst measured scheduling stall is 36.8 ms, so the timeout only fires on a stuck plant. |
| `plant_seed` | `0` | The episode seed, logged in the `plant` line. |

## Run a plant

`brake_fusion` is the shipped package with a plant: a MuJoCo vehicle that brakes
toward a stop, driven by a fusion pipeline. It is part of the first-start build, so
this works in a plain container:

```bash
docker compose run --rm dev bash -c \
  'FASTDDS_BUILTIN_TRANSPORTS=UDPv4 BF_LOG_DIR=/tmp/bf ros2 launch brake_fusion t2_separate.launch.py'
```

Watch for the stop line, then press Ctrl-C:

```text
[simulator_node-1] [INFO] [1795772470.794285017] [simulator]: BF_STOP position=35.426706000004508 step=3705
```

`/tmp/bf` then holds `simulator.jsonl` (the plant log described above) and one
`.jsonl` file per node. To compare two runs, diff the `state_hash` of the same
`step` in the two `simulator.jsonl` files.

## Recording and replaying a run

The binding can record what entered a generated system and replay it later. This is
a feature of the binding for topics that carry their own stamp (`anchor: stamp` in
the [execution config](execution-config.md)), and it is not specific to plants. No
shipped package uses a stamp-anchored topic, so there is no ready-made run to try
here. This section describes the interface only and has not been run for this page.

- `RCLLF_PLANT_RECORD=<dir>` records, per stamp-anchored external topic, one
  `ext.<topic>.jsonl` file (with each message, its disposition `admitted`, `late`,
  `overrun`, `collision`, `after_stop` or `refused`, and its tag) and one `run.jsonl`.
  Nothing is written when the variable is unset or empty.
- `RCLLF_PLANT_REPLAY=<dir>` runs the same generated program against such a
  recording and feeds the recorded messages at their recorded tags.
- If the recording does not fit the program, the replay stops. A `replay_failed`
  record appears in the JSON log and the terminal shows `rcllf replay: FAILED: <why>`.
  It is a log record, not a trace event.

See the [environment variable reference](../reference/environment-variables.md#rcllf_plant_record)
for both variables.

## Testing a timeout

`rcllf_plant_fault` is a test build of the Transactor, built and installed only
with `BUILD_TESTING`, that honours
`RCLLF_PLANT_FAULT`:

- `hang:<k>` blocks plant step k forever;
- `delay:<k>:<ms>` makes plant step k return late.

The production library refuses the variable at start. `brake_fusion`'s
`test_plant_timeout.py` runs the simulator on that build and checks the outcome from
outside the process: `plant_timeout` is logged for step k, no state is published for
step k or later, and the exit code is 3.

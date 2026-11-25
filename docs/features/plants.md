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
into `simulator.jsonl`:

```json
{"ev":"plant","plant":"mujoco","mujoco_version":"3.14.0","timestep":0.001,"episode":1,"seed":0,"plant_step_timeout_ms":1000,"fault":"","state_hash":"886aa3b9613e7d2b","status":"OK"}
{"ev":"plant_step","step":1,"state_hash":"…","output_hash":"…","backend_steps":1,"clock_drift_ns":0,"plant_wall_ns":5123}
```

The hashes are FNV-1a 64 over the IEEE bits of each double. The state hash covers
MuJoCo's qpos and qvel. The output hash covers what the step returned.

| Parameter | Default | Meaning |
|---|---|---|
| `plant` / `$BF_PLANT` | `mujoco` | Only `mujoco` is accepted. Any other value stops the node at start. |
| `plant_step_timeout_ms` | `1000` | The supervised step timeout. One MuJoCo step of this model takes microseconds, and the worst measured scheduling stall is 36.8 ms, so the timeout only fires on a stuck plant. |
| `plant_seed` | `0` | The episode seed, logged in the `plant` line. |

## Comparing runs

Two runs are identical when their `simulator.jsonl` logs have the same plant line,
the same steps 1..N and the same hashes at every step. The project compares them with
a script of its benchmark harness, which is not part of this release; the logs alone
are enough to repeat the check.

## Recording a run

Set `RCLLF_PLANT_RECORD=<dir>` on a generated system to record what entered it and
how each input was decided. Nothing is recorded, created or logged when the variable
is unset or empty. A recording holds everything a replay of the run needs
(see [Replaying a run](#replaying-a-run)).

The directory gets one JSON-lines file per stamp-anchored external topic
(`anchor: stamp`, see [execution config](execution-config.md)), named after the
topic (`/stamped_in` becomes `ext.stamped_in.jsonl`), and one `run.jsonl`:

```json
{"ev":"header","format":"rcllf_plant_record","version":1,"kind":"external","topic":"/stamped_in","type":"geometry_msgs/msg/PointStamped","serialization":"rmw_serialize","rmw":"rmw_fastrtps_cpp","lf_time_start":1794422307092248481,"delay_ns":200000000,"ingress_reserve_ns":50000000,"grid_ns":10000000}
{"ev":"external","rid":0,"id":4,"stamp_ns":1794422308493248481,"arrival_ns":1794422307888580329,"insert_ns":1794422307888584149,"tag_ns":1660000000,"disposition":"admitted","serialize_ns":1670,"cdr_len":52,"cdr":"AAEAAC…"}
{"ev":"correction","rid":0,"id":4,"disposition":"collision"}
{"ev":"external","rid":0,"id":3,"stamp_ns":1794422306887813550,"arrival_ns":1794422307888566219,"insert_ns":null,"tag_ns":null,"disposition":"late","serialize_ns":3060,"cdr_len":52,"cdr":"AAEAAC…"}
```

- An `external` entry is written once per message, as soon as its disposition is
  known: `admitted`, `late`, `overrun`, `collision`, `after_stop` or `refused`. It
  carries the record id, the stamp, the arrival and insert times (absolute
  `CLOCK_REALTIME` ns), the computed tag, the disposition and the message itself:
  `cdr` is the base64 of the rmw serialization, and the header names the type.
- A message that was admitted and then refused, because a later message landed in
  its grid cell, gets a `correction` entry. **A reader takes the last entry per
  record id.** The binding's counters do the same: the record moves from admitted to
  collision. A correction is always written after the entry it corrects, even when
  two ingress threads queue them in the other order.
- `tag_ns` is the tag minus `lf_time_start`, at microstep 0. It is `null` for a late
  message, which never gets a tag. `insert_ns` is `null` when the message was refused
  before the scheduler was entered.

`run.jsonl` starts with a header holding `lf_time_start`, then:

- one `publish` entry per publication on a bounded topic (`output_deadline_ns`):
  the elapsed tag, the microstep and the FNV-1a 64 hash of the serialized message.
  The hash is taken before the output-deadline check, so the check stays next to
  the real publish;
- one `actuation_late` entry per output deadline miss, with the same fields as the
  trace event;
- one `lf_stop` entry, always the last line, with reactor-c's stop tag and why the
  run stopped: `timeout` when the stop tag is `(timeout, 0)`, `request_stop` for a
  later microstep (Ctrl-C, `lf_request_stop`), otherwise `other`. The shutdown
  reaction captures it (`source: shutdown_reaction`) and teardown writes it after
  every other entry; without a shutdown reaction, teardown takes it (`source:
  teardown`). `stop_tag_ns` and `stop_microstep` repeat `tag_ns` and `microstep`.

```json
{"ev":"publish","rid":0,"topic":"/actuation","tag_ns":100000000,"microstep":0,"cdr_len":12,"fnv1a64":"d6db9c79e97a03b4"}
{"ev":"lf_stop","tag_ns":2000000000,"microstep":0,"condition":"timeout","stop_tag_ns":2000000000,"stop_microstep":0,"duration_ns":2000000000,"source":"shutdown_reaction"}
```

Topics anchored at arrival are not recorded, because their tag is the arrival time
and there is no computed tag to keep.

### Cost

A writer thread of its own opens, writes and flushes the files. The ingress thread
does two things for a recorded message, and neither one waits on I/O:

- it serializes the message before taking the scheduler's lock, because a user
  callback may modify the message once it is admitted;
- it appends one line to an in-memory queue after leaving the lock.

The queue has no bound. A recording that outruns the disk grows in memory instead of
slowing the system.

Measured on `stamped_input_example`, a test package of the development repository that does not
ship in the release, 10 runs with recording and 10 runs without, on the
dev image (evidence `plant-m5c/inv3`). The table shows the time from a message's
arrival to its insert into the event queue:

| | First message of a run, p50 / max | Later messages, p50 / max |
|---|---|---|
| Recording off | 88.6 / 97.0 µs | 2.6 / 6.2 µs |
| Recording on | 109.8 / 150.2 µs | 5.0 / 22.3 µs |

Serializing a later message took 2.2 µs at the median. The longest queue append
in any run took 12.0 µs. The first serialization of a type is slow: 61 µs at the
median when the binding did nothing beforehand. The binding therefore serializes
one default message when it attaches a topic, which brings the first message down to
24 µs.

At shutdown the binding logs one summary line:
`rcllf record: N lines in K files under '<dir>' (E write errors); ingress serialize p50=… max=… over n records; enqueue max=… ns excluding the writer start; C corrections without their entry`.
Write errors include failed flushes and closes (a full disk shows up there), so a
nonzero E means the files are incomplete.

## Replaying a run

Set `RCLLF_PLANT_REPLAY=<dir>` to a recording's directory to run the same generated
system again on the inputs the live run admitted, without the outside world.
Nothing changes when the variable is unset or empty.

```bash
RCLLF_PLANT_REPLAY=live/record RCLLF_PLANT_RECORD=replay/record RCLLF_LF_ARGS="-f true" \
  ros2 lf launch <package> <stem>
```


What a replay does:

- No stamp-anchored subscription takes messages from DDS. At the start tag, the
  startup reaction that registers a topic's physical action inserts every record
  whose **last** entry says `admitted` at `(start + tag_ns, 0)`. It uses the same
  insert and the same record lookup as the live run, so the user callback runs at
  the same tag with the same message (the recorded bytes, deserialized).
- `late`, `overrun`, `collision`, `after_stop` and `refused` records are not
  replayed. They never ran live, so they do not run in the replay. One log line per
  topic lists how many of each were left out.
- An admitted record whose tag lies after the live run's stop tag is also left out
  and counted (`admitted_past_stop`). This happens when a stop request ended the
  live run before that tag.
- The stop tag is lowered to the live run's `lf_stop` tag once, at the start tag,
  and read back. A Ctrl-C run therefore replays to the same `(tag, microstep)` as a
  stop request.
- Fast mode (`RCLLF_LF_ARGS="-f true"`) is allowed in a replay, and is the
  intended way to run one. The tags come from the recording, not from the wall
  clock, so a replay does not wait for physical time. Live, fast mode is refused
  for stamp anchors.
- With `RCLLF_PLANT_RECORD` set as well, the replay records itself. Each replayed
  record gets an `external` entry with `"replay_of":<live id>`, and `run.jsonl` gets
  the replay's publications and its `lf_stop` line. Two runs compare file against
  file.

A replay that cannot reproduce the live run is refused before anything runs. It
logs `rcllf replay: FAILED: <why>` and the trace event `replay_failed`, then exits
with code 3. The refusals are:

- a missing or malformed recording (including an unknown disposition or a field of
  the wrong type), or a format version other than 1;
- topic files and `run.jsonl` whose headers name different start times, which means
  they come from different runs;
- a topic, type or anchor constant (T, R, G) that differs from the program's;
- a program without keepalive, which would stop one microstep after its last input;
- a stop tag that cannot be lowered to the live one, such as a replay given a
  shorter timeout;
- a record whose bytes do not deserialize or carry another `header.stamp`;
- any insert that does not succeed.

At shutdown, one `rcllf replay summary:` line per topic gives `inserted`,
`delivered`, `admitted_past_stop` and `not_admitted`. The binding logs
`rcllf replay: FAILED` and the process exits with code 3 if any of these holds:

- a recorded topic has no subscription in the program, even one that received nothing
  live;
- a recorded RID was never replayed;
- `delivered` differs from `inserted`;
- the run stopped at a tag other than the live one, for example because a stop request
  ended it early.

Every tag in a recording is microstep 0 and comes from the grid rule. A replay
therefore puts no two records in one cell, the same as the live run.

**Proof.** `stamped_input_example/test/test_plant_replay_system.py` (in the development repository;
the package does not ship in the release) records a live
run of the `replay` launch on a recorded feed. The feed has three admitted
records, one late, a collision pair and one after the stop. The test records once
with a 6 s timeout and once with Ctrl-C after 2 s, then replays each recording five
times. `plant_replay_compare.py` compares every replay with its live run, using
only the two runs' own files:

- the admitted inputs, with their stamps, tags and bytes, and no other input;
- each stamped reaction's `(tag, microstep)` and order;
- the node's running sums;
- every publication on the bounded `/stamped_out` (tag, microstep, payload hash):
  64 in the timeout run and 30 in the Ctrl-C run;
- the stop tag and its condition.

All 10 replays matched (evidence `plant-m5d/system_run1`). Recorded copies of one
replay per case are checked in every `colcon test` of the development repository
(`test_plant_replay_fixtures.py`).

**Not covered yet.** Topics anchored at arrival are not recorded, so they cannot be
replayed; a replay refuses a program that has one. The plan's logical-action
mechanism is not used. Replay goes through the existing physical action, so it
needs no generator change and the delivery path is the live one (plan
2026-11-09, item 5 status).

## Paired live runs (A8)

Replay shows that a recording reproduces itself. A8 asks the stronger question:
do two *live* runs that receive the same stamped inputs at different physical
times produce the same reactions and outputs? It is plan 2026-11-09's
acceptance A8, on an open-loop rig with no plant in the loop.

**The rig.** The `a8` launch of `stamped_input_example` (a test package of
the development repository that does not ship in the release) runs `a8_controller_node`,
which subscribes to `/scan` (10 Hz) and `/odom` (20 Hz) and publishes one
`/drive` per scan. Both topics are stamp-anchored with T = 60 ms, R = 20 ms and
G = 10 ms; `/drive` carries a 20 ms output bound (`lf/a8.execution.yaml` gives
the basis of each constant). The test process is the feeder. It reads the run's
start from the binding's `external_stamp_anchor` trace line, sets the run's epoch
to start + 2.5 s, stamps every fixture message as epoch + offset and publishes it
at the scheduled arrival time. Every comparison is on tags relative to the run's
start, so A8 tests **time-shift equivalence**, not identical absolute stamps.

| Pair | What differs between the two runs | What must hold |
|---|---|---|
| (a) | each message's on-time arrival: 2 to 18 ms after its stamp in one run, 22 to 38 ms in the other | same elapsed tags, same reaction order within each tag, same `/drive` values and payload hashes; no message lost |
| (b) | at every stamp carrying both a scan and an odom: scan first in one run, odom first in the other | as (a) |
| (c) | one `/odom` pushed 40 ms past T | only that message `late`; nothing changes before its tag; at least one `/drive` differs at or after it, the first differing tag named; the fixture is pre-checked to affect control |
| (d) | scan 7's reaction held 40 ms | exactly one `actuation_late` on `/drive` at that tag, its value unchanged |
| (e) | replays of the (a) and (c) live runs | admitted tags, order and values reproduced |

The verdicts come from the runs' own files (`record/`, the trace and the node's
log), compared by `test/a8_compare.py`. They are also checked against an
independent Python model of the controller (`test/a8_rig.py`). Every run is
also checked on its own: each recorded input, deserialized, equals the
fixture message, and each admitted record has exactly one reaction, at its
rule tag. Two runs that received different inputs, or that both missed a
reaction, therefore fail even when their outputs agree.

**Result** (`test_a8_system.py`, 6/6; evidence `m5d2/run4`). (a): 60 reactions
and 20 drives equal; arrivals differed by at least 5.96 ms per message. (b): all
20 pairs were swapped, with equal results. (c): `/odom` 20 arrived 100.27 ms after
its stamp. It was `late`, its tag 3.58 s; 30 reactions and 10 drives before that
tag were equal, and the first and only differing `/drive` is at 3.58 s (the
controller converges back). (d): lateness 40.38 ms against the 20 ms bound, same
payload hash. (e): 5 replays each of (a) and (c) were equal to their live runs.

**Falsified** on real runs, each failing with the bug's own message. A negative
pair whose lost message changes nothing (`/odom` 21) is rejected by the
pre-check. Without the pre-check, the same pair fails "no /drive difference at
or after the lost tag". The comparison on absolute tags fails "reactions differ
at #0". A 5 ms hold fails (d) with no `actuation_late`. `test_a8_fixtures.py`
checks ten recorded runs and these falsifications in every `colcon test` of
the development repository,
plus two doctored copies: a changed input payload and a reaction missing in
both runs of pair (b).

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

# Fast Mode

Fast mode runs a generated program faster than physical time. This page shows how to
turn it on and off and what changes in a fast run.

## What it does

The reactor-c runtime normally waits until the physical clock reaches a tag before it
runs the reactions of that tag. In fast mode it does not wait: logical time moves on as
soon as the reactions of the current tag are done. The reactions, their order and the
values they see stay the same as in a real-time run. Use it for a simulation or a
regression check that should not wait for the clock.

## Turn it on

```bash
colcon lf --package pubsub_example --fast --timeout "2 sec"
ros2 lf launch pubsub_example bringup
```

```text
[Bringup-1] ---- Elapsed logical time (in nsec): 2,000,000,000
[Bringup-1] ---- Elapsed physical time (in nsec): 3,595,091
```

The program covers 2 s of logical time in under 4 ms of wall time, and the subscriber
hears all five messages the publisher sends in those 2 s. Always give a timeout with
`--fast`. Without one the program runs until it is stopped, and in fast mode that
means as fast as the host allows.

`--fast` adds the reactor-c option `-f true` to the generated launch file.
`--timeout` writes the LF `timeout` target property into the generated `.lf`
(`timeout: 2 sec,`). Both options exist on `colcon lf` and on `colcon lf-generate`.
`--timeout` takes any LF time literal (`2 sec`, `2000 msec`).

## Change it at launch

The generated launch file appends the words of
[`RCLLF_LF_ARGS`](../reference/environment-variables.md#rcllf_lf_args) after its own
arguments, so a built program takes both settings without a regeneration. reactor-c
keeps the last `-f` it reads. To run the same build in real time:

```bash
RCLLF_LF_ARGS="-f false" ros2 lf launch pubsub_example bringup
```

```text
[Bringup-1] ---- Elapsed logical time (in nsec): 2,000,000,000
[Bringup-1] ---- Elapsed physical time (in nsec): 2,000,936,891
```

`-o` sets the timeout, with the value and the unit as two words:

```bash
RCLLF_LF_ARGS="-f true -o 1 sec" ros2 lf launch pubsub_example bringup
```

```text
[Bringup-1] ---- Elapsed logical time (in nsec): 1,000,000,000
[Bringup-1] ---- Elapsed physical time (in nsec): 3,266,462
```

To stop using fast mode for good, run `colcon lf` again without `--fast`.

## What changes in a fast run

- **Timers fire without waiting.** A 1 ms timer fires 1000 times per logical second,
  as fast as its reactions finish. A node's `now()` returns the logical time, so the
  node does not see the wall clock fall behind.
- **Order and values stay the same.** Inputs from inside the program arrive at their
  logical delays, which do not depend on the wall clock.
- **Inputs from outside the program keep their arrival time.** A subscription on an
  [`external_topics`](execution-config.md#external_topics) topic gets the physical
  time of arrival as its tag. In fast mode logical time is usually far ahead of the
  wall clock, so such a message lands at the next free tag. Feed external topics only
  to a program that runs in real time.
- **A topic anchored at `header.stamp` needs real time.**
  `colcon lf-generate --fast` refuses a system with one, and a launch-time `-f true`
  makes the program exit with code 3. See
  [Anchoring a topic at `header.stamp`](execution-config.md#anchoring-a-topic-at-headerstamp).
- **No publish is late.** Logical time runs ahead of physical time, so the
  [output deadline](execution-config.md#publishers-output-deadline) never reports one.
- **reactor-c's physical time line is not a wall-time measure.** It counts from the
  start time and shows how long the run took, not how long the system would take.

## Limits

- **Federations are not supported in fast mode in v0.1.2.** Generate and run a
  [federation](federation.md) in real time.
- **A federation that stops at a logical timeout currently fails.** This holds in real
  time too. See [Stopping a federation](federation.md#stopping-a-federation).
- Fast mode is a reactor-c run-time option, not the LF `fast` target property. lfc
  refuses that property in a program with physical actions, and every generated node
  reactor has one.

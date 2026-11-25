# Fast Mode

Fast mode runs a generated rcllf program faster than physical time. The reactor-c
runtime normally waits until the physical clock reaches a tag before it runs the
reactions of that tag. In fast mode it does not wait: logical time advances as soon
as the reactions of the current tag are done. The reactions, their order and the
values they see are the same as in a real-time run, because every input of a
generated program carries a logical tag and the correspondence of each topic edge is
declared at generation time, not measured at run time. A program that takes 3.4 s of
logical time to brake runs in well under a second of wall time. Read this page when
you want to run a simulation or a regression check without waiting for the clock, or
when you want a program to end at a logical time instead of a wall-clock signal.

## Quick reference

| Setting | At generation | At launch (no regeneration) |
|---|---|---|
| Fast mode | `colcon lf-generate --fast` | `RCLLF_LF_ARGS="-f true"` |
| Logical timeout | `colcon lf-generate --timeout "20 sec"` | `RCLLF_LF_ARGS="-o 20 sec"` |

`--timeout` writes the LF target property into the generated program; `--fast` writes
the reactor-c argument `-f true` into the generated launch files (the one program's and
every federate's):

```lf
target CCpp {
  build-type: RelWithDebInfo,
  keepalive: true,
  timeout: 20 sec
}
```

```python
# generated launch file
arguments=['-f', 'true']
+ shlex.split(os.environ.get("RCLLF_LF_ARGS", "")),
```

Fast mode is not the `fast` target property on purpose. lfc 0.12 refuses that property
in a program with physical actions (`the fast target property is incompatible with
physical actions`), and every rcllf node reactor has one: the wake action that brings
subscription samples into the reactor. The reactor-c runtime itself accepts `-f true`
at start without that check, and the launch files are where the generated program
takes its runtime arguments.

The launch files append the words of `RCLLF_LF_ARGS` after the generated arguments, so
a built program takes both settings at launch time. The value is split like a shell
command line; `-f true` is reactor-c's fast mode and `-o 20 sec` its timeout, with
value and unit as two words. reactor-c keeps the last `-f` it reads, so
`RCLLF_LF_ARGS="-f false"` runs a program generated with `--fast` in real time.

```bash
RCLLF_LF_ARGS="-f true -o 20 sec" ros2 launch lf_generated_brake_fusion_k1container lf_k1container.launch.py
```

`--timeout` accepts any LF time literal (`20 sec`, `20000 msec`, `1500ms`) and writes
the shortest exact form. Both options also exist on `colcon lf`.

## What fast mode changes and what it does not

- **Timers fire without waiting.** A 1 ms timer fires 1000 times per logical second,
  as fast as the reactions complete. Logical time is still the timer's time: a node's
  `now()` under conversion returns the logical tag, so nothing in the application
  sees the wall clock run ahead.
- **Dispatch order and values are unchanged.** The order of reactions at a tag is
  fixed by the generated program, and inputs from inside the program arrive at
  declared logical delays. The pilot below checked this value for value.
- **Inputs from outside the program keep their physical arrival.** A subscription
  whose publisher is not part of the program is a physical action: its tag is the
  physical time of arrival. In fast mode the program's logical time usually runs far
  ahead of the wall clock, so such an input lands at a logical tag that is already
  past, and reactor-c assigns it the next available tag. Run external publishers in
  real time only when the program also runs in real time. A topic anchored at
  `header.stamp` (`anchor: stamp`) has no such fallback: its tag is
  derived from the stamp, so `colcon lf-generate --fast` refuses a system with one and
  the binding exits with code 3 if fast mode reaches it at launch.
- **The timeout ends the program at a logical time.** With `keepalive: true` (the
  generated default) a program runs until it is stopped. `--timeout` ends it at the
  logical time given, in fast mode and in real time alike. A federation ends through
  the same path: every federate reports the stop tag to the RTI and the RTI grants
  it, so no wall-clock signal is needed. In the pilot, `-o 2 sec` ended the one
  program and the two-federate program at an elapsed logical time of exactly
  2 000 000 000 ns, with reactor-c's "process has finished cleanly" line.

## Federations run fast too, with one hazard

A centralized federation runs fast as well: the two-federate brake_fusion program on
one host finished in about a quarter of its real-time wall time (table below).
reactor-c bounds a federate's next event tag to physical time only when a physical
action can reach an output of the federate within a declared bound (`bounded_NET` in
`federate.c`, keyed on `min_delay_from_physical_action_to_federate_output`). lfc
leaves that bound at its "never" default for the generated federates, so the RTI
grants tags as soon as every federate has reported, and the federation advances at
the pace of the slowest reaction chain, not the slowest wall clock. Fast federates
also skip the RTI's start delay (one second after the start handshake), which is part
of the wall-time gain. Two consequences of that skip: reactor-c's own "Elapsed
physical time" line is not a wall-time measure for a fast federation (it counts from
the agreed start time and can be negative), and the figures below come from an
external clock.

**The grant can beat the sample (fixed).** Cross-federate samples are
counted with `take_mode=skip`: the count travels through the RTI with the
tag, the sample through DDS. In real time the RTI's grant and the sample travel at the
same pace and no pilot run ever lost a sample. In fast mode the receiving federate asks
for the next tag as soon as its reactions are done, so the grant can arrive while the
sample is still in the DDS receive path. Before the fix the counted take looked once,
reported the sample as lost (warning P17e, "dropped a late sample ... reported as
lost") and ran the reaction with the sample it had; when the late sample was the only
one owed it was delivered one tag late, and a camera read the plant state
of the previous tag. One of ten W1 pilot runs of the two-federate brake_fusion program
differed from the real-time runs that way, at camera tick 78; a later reproduction differed in the stop
step as well.

Since the fix a counted take waits for the sample it owes, polling the reader for up
to the subscription's `maxwait_ms` of wall time before it declares the miss; the
wait is counted in `take_stats` as `waited`. The bound is the
per-entry value from the execution config (100 ms when the config sets none), for
example 15 ms for fusion's `/radar/range` and `/camera/range` inputs in brake_fusion's
R2 federated configs; `ROS2_LF_MAXWAIT_MS`, when set, only overrides it for
every subscription. The wait therefore covers a sample that is still in the local DDS
receive path. On two hosts a DDS delay longer than that timeout is again a P17e miss,
exactly as before the fix: size `maxwait_ms` above the expected network delay. The
fix was first keyed on reactor-c's `fast` flag (`-f true`); now the same
wait applies in real time, where the RTI's grant over TCP can beat the sample over UDP
under host load (the `brake_fusion` per-node federation lost a `/camera/range` sample
that way in one of ten loaded runs). The one program is unchanged in both modes,
because its samples never leave the process and are in the reader before the tag is
granted.

## Pilot: brake_fusion, real time against fast

The brake_fusion showcase (MuJoCo plant, one 1 ms simulator step per tag, 20 s
horizon) was run as the one rcllf program (K7, timers first) and as two federates on
one host (K8, split per `config/k8/k1_container_2host.federation.yaml`), each in real
time and in fast mode, three runs each, on 4 pinned cores. Fast mode was switched at
launch through `RCLLF_LF_ARGS="-f true"` on the same build. The stop position, the
brake command sequence (tag and value), the per-node record sequence and the wall time
per run were compared value for value; every run matched.

| Variant | Mode | Stop (step, m) | Brake commands (step, level) | Records | Wall time, launch to stop |
|---|---|---|---|---|---|
| One program | real time | 3352, 34.710 | (728, 0.4), (1978, 1.0) | 864 | 3.72 s, 3.74 s |
| One program | fast | 3352, 34.710 | (728, 0.4), (1978, 1.0) | 864 | 1.04 s, 1.04 s |
| Two federates | real time | 3341, 34.604 | (732, 0.4), (1957, 1.0) | 861 | 4.90 s, 4.90 s |
| Two federates | fast | 3341, 34.604 | (732, 0.4), (1957, 1.0) | 861 | 1.27 s, 1.38 s |

The one program produced identical record sequences in all of its runs, real time and
fast alike (3.351 s of logical time in 0.74 s of wall time after the first record,
about 4.5 times faster than real time). The two federates matched in all real-time
runs and in four of five fast runs; the fifth is the hazard above. The two variants
differ from each other because the cross-federate links carry the 5 ms delay of the
federation file, which is the expected difference between the two deployments, not a
fast-mode effect. The wall times are from the external clock of the run script, two
runs per row. A pilot is a few runs, not evidence.

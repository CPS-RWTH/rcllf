# Federation

Federation splits one generated system across several processes, the federates,
which can run on several hosts. This page shows how to generate and run a federation,
how to stop it, and what it can and cannot do.

## Overview

Each [federate](../design/glossary.md#federate-federation) is its own OS process. The
LF [Run-Time Infrastructure (RTI)](../design/glossary.md#rti-run-time-infrastructure)
keeps one logical clock across all of them. DDS still carries every message, so a
plain ROS 2 node can subscribe to a topic a federate publishes. The RTI controls only
when each federate may advance its [tag](../design/glossary.md#tag).

- Federation is opt-in. `colcon lf-generate` without `--federate` builds one process.
- The default is
  [centralized coordination](../design/time-and-coordination.md#regime-2-federated-centralized):
  the RTI grants every tag advance, and messages are never tardy.
- Decentralized coordination is exposed and experimental. See
  [Coordination mode](#coordination-mode).

**Federation is all-or-nothing.** A federation is one logical program that runs in
several processes. Every federate it declares must be present for the program to
make progress. If one federate stalls or dies, the others stall at their next
cross-federate event.

## Quick start

All federation services are in the `federation` Compose profile. They need the `dev`
image, which contains `lfc` (`docker compose build dev`). The `rti` service builds its
own small image (target `lf-rti`) on first use.

### One-shot run

```bash
docker compose --profile federation run --rm lf-build-federated     # generate and build
docker compose --profile federation run --rm lf-pubsub-federated    # RTI and both federates
```

`lf-build-federated` builds `pubsub_example` as two federates. `lf-pubsub-federated`
starts the orchestrator launch file `lf_bringup_federated.launch.py`, which starts the
RTI and then both federates in one container, for 30 s. A working run prints
(trimmed):

```text
[RTI-1] RTI: Number of federates: 2
[RTI-1] RTI: Listening for federates.
[federate__publisher_node-2] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted egress edge 0 (global 0) PublisherNode -> /subscriber_node on '/sensor_data' (d = 0 ns, L = 200000000 ns)
[federate__subscriber_node-3] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted ingress edge 0 PublisherNode -> SubscriberNode on '/sensor_data' (d = 0 ns, T = 100000000 ns, ...)
[RTI-1] RTI: All expected federates have connected. Starting execution.
[federate__publisher_node-2] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[federate__subscriber_node-3] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
...
[RTI-1] WARNING: Failed to accept the socket. Resource temporarily unavailable. Trying again.
...
==> Federated smoke test complete
```

Each federate is its own process. The `/sensor_data` edge is admitted on both sides,
and every `Publishing` line is followed by an `I heard` line with the same values. The
RTI's `Failed to accept the socket` warning is harmless (see
[Troubleshooting](#troubleshooting)).

### One service per federate

```bash
docker compose --profile federation up -d rti                    # RTI in the background
docker compose --profile federation run --rm lf-fed-publisher    # terminal 1
docker compose --profile federation run --rm lf-fed-subscriber   # terminal 2
docker compose --profile federation down rti                     # when done
```

Each `lf-fed-*` service starts one federate against the running RTI, so each terminal
shows one federate's log. Start the second federate within 5 seconds of the first:
before it connects to the RTI, each federate waits that long for its peer's topics and
otherwise exits with code 3 (see [Endpoint match warm-up](#endpoint-match-warm-up)). A federate service runs for `FED_TIMEOUT` seconds (default
60) and then gets SIGINT, which stops the whole federation (see
[Stopping a federation](#stopping-a-federation)). The end of a working run (trimmed,
`FED_TIMEOUT=20` for the publisher and `30` for the subscriber):

```text
publisher:   [federate__publisher_node-1] [INFO] [...] [publisher_node]: Publishing: temp=51.0, humidity=71.0, pressure=1044.0
subscriber:  [federate__subscriber_node-1] [INFO] [...] [subscriber_node]: I heard: temp=51.0, humidity=71.0, pressure=1044.0
publisher:   [INFO] [federate__publisher_node-1]: process has finished cleanly [pid ...]
subscriber:  [INFO] [federate__subscriber_node-1]: process has finished cleanly [pid ...]
rti:         RTI: Federate 1 has resigned.
rti:         RTI: Federate 0 has resigned.
rti:         RTI is exiting.
```

The service whose timeout fired exits with code 124, the other with 0. Both federates
print the same `Elapsed logical time`. For a longer run, raise the timeout:
`FED_TIMEOUT=300 docker compose --profile federation run --rm lf-fed-publisher`.

### Your own package

The pipeline takes the same flags. In the dev container, in `/workspace/packages`:

```bash
colcon lf --package my_robot --federate per-node
source install/setup.bash
ros2 lf list --package my_robot          # the system shows "ready (federated)"
ros2 lf launch my_robot bringup
```

`ros2 lf launch` starts the RTI and every federate from the generated orchestrator
launch file. It runs what the last `colcon lf` produced for that launch file. Run
`colcon lf --package my_robot` without `--federate` to get the one-process program
back.

## How federation differs from plain ROS 2

| Property | ROS 2 | LF federation |
|---|---|---|
| Failure isolation | Each node is independent | All-or-nothing: one stalled federate stalls the federation |
| Time | Local clock (or `/clock`) | One logical time across all federates, kept by the RTI |
| Ordering | No ordering across nodes | The same deterministic order as one process |
| Late joiners | Nodes can come and go | No: every federate is fixed at generation |
| Coordinator | None | The RTI |

## Partition modes

`--federate` selects how nodes are grouped into federates.

| Mode | Argument | Result |
|---|---|---|
| Per node | `--federate per-node` | One federate per ROS 2 node. |
| Per process | `--federate per-process` | One federate per launch process: a component container, or one standalone node. Falls back to per node without launch information. Experimental. |
| Custom | `--federate path/to/federation.yaml` | Your own grouping. Experimental. |

Per node under centralized coordination is supported; per process, a custom YAML,
several hosts and decentralized coordination are experimental (see the
[Support Matrix](support-matrix.md)).

### Custom YAML

`--emit-federation-template` writes a starter file with one federate per node:

```bash
colcon lf-generate --emit-federation-template \
  --system-json lf-analysis/pubsub_example/bringup/system.json
# writes lf-analysis/pubsub_example/bringup/federation.yaml
```

`--force` overwrites an existing template. Group nodes by moving them under fewer
federate names:

```yaml
system: bringup

federates:
  publisher_side:
    nodes:
      - publisher_node

  subscriber_side:
    nodes:
      - subscriber_node

after_overrides: []
```

- Every node appears in exactly one federate.
- Every federate has at least one node.
- Federate names use only `a-z`, `A-Z`, `0-9` and `_`.

### Per-link after-delay overrides

A cross-federate connection defaults to `after 0`. The receiver runs at the
publisher's time, one microstep later, so a split into federates adds no logical
latency. Inside one process an admitted d = 0 edge runs at the publication tag itself,
so a cross-federate d = 0 link lands one microstep later than the same link in one
process. To give a link a delay d > 0:

```yaml
after_overrides:
  - from: publisher_node.sensor_data
    to:   subscriber_node.sensor_data
    after: 8ms   # or '8 msec'
```

`after` takes the same time grammar as `--after-delay` and `execution.yaml`: `8ms`,
`500us`, `100ns`, `2s`, or `8 msec`, `500 usec`, `100 nsec`, `2 sec`. A unit is
required except for `0`. `from` and `to` are the `<instance>.<port>` names of the two
ends of a cross-federate link, as in the top-level federated `.lf` file. An entry that
names no cross-federate link stops generation with an error that lists the valid keys.

## Coordination mode

`--coordination {centralized,decentralized}` on `colcon lf-generate` (and `colcon lf`)
selects the LF coordination mode. `centralized` is the default. A custom
`federation.yaml` may set `coordination:` itself, and the flag wins when both are
given.

`decentralized` is exposed and experimental. It emits LF's
`coordination: decentralized` target property
([regime 3](../design/time-and-coordination.md#regime-3-federated-decentralized)):
federates advance without asking the RTI, each input waits a safe-to-process (STP)
offset, and a message can arrive tardy, after its tag was already processed.

### STP offset

Each cross-federate input gets an STP offset and a reaction that logs a tardy message.
Set the offset per link next to `after`:

```yaml
coordination: decentralized
after_overrides:
  - from: publisher_node.sensor_data
    to:   subscriber_node.sensor_data
    after: 5ms
    stp_offset: 20ms
```

Unset, `stp_offset` is 10 ms, a starting point to tune and not a measured value. Under
centralized coordination it is ignored.

- **Only a federate with its own tag source can see a tardy message.** A federate whose
  only trigger is the network input is never ahead of that input. The two-federate
  `pubsub_example` has this shape, so it shows no tardy message whatever the offset.
- **One message per logical time on a link that is not admitted.** The receiver has
  already processed a time when a second message of that time arrives. Such a link
  exits with code 3 when its publisher writes it twice at one time. Keep one write per
  time, or put the publishers in the subscriber's federate.
- **Admitted links stay admitted.** A tardy record is not fatal: each of its messages
  ends LOST and runs the subscription's `message_lost` handler once, where
  `ros2_lf_message_lost_context()` reports `tardy = 1`. See
  [Missing Messages](missing-messages.md).

### Budget check

A decentralized link holds without a tardy message when its delay and STP offset cover
the worst delivery:

    d + W >= L + eps + X + W_send

`d` is the link's `after` and `W` its `stp_offset` (alias `absent_after`, 10 ms when
unset). An `after_overrides` entry may declare `max_latency` (L, worst network
latency), `clock_error` (eps, clock offset between the hosts) and `max_exec` (X, worst
execution time of the sending reaction). `W_send` is the largest STP offset of the
sending federate's own decentralized inputs. Generation refuses a link that breaks the
inequality and names every number:

```text
ERROR: budget check refused: link 'pub.data -> sub.data' violates d + W >= L + eps + X + W_send: ...
short by 1000000 ns
```

A link with some but not all three terms is refused too. A link with none is reported
as `WARNING: ... declares no budget` and not checked. Under centralized coordination
the terms are refused, because the RTI grants tags there.

### Example

```bash
docker compose --profile federation run --rm lf-build-federated-decentralized
docker compose --profile federation run --rm lf-pubsub-federated-decentralized
```

The build uses `packages/src/pubsub_example/federation.yaml`, which sets
`coordination: decentralized` and `stp_offset: 1usec` on the one cross-federate link.

## Generated output

A federation is N+1 colcon packages under `packages/lf-generated/`. For
`pubsub_example` with `--federate per-node`:

```
lf-generated/
├── lf_generated_pubsub_example_bringup_federated/          # coordinator package
│   ├── CMakeLists.txt                       # runs lfc, builds the federate binaries
│   ├── lf/
│   │   ├── Bringup_federated.lf             # top-level federated reactor
│   │   ├── publisher_node/PublisherNodeSystem.lf
│   │   └── subscriber_node/SubscriberNodeSystem.lf
│   └── launch/lf_bringup_federated.launch.py   # orchestrator: RTI and all federates
├── lf_generated_pubsub_example_bringup_federated_publisher_node/
│   ├── system.json                          # only this federate's nodes
│   └── launch/lf_publishernodesystem.launch.py
└── lf_generated_pubsub_example_bringup_federated_subscriber_node/
    └── ...
```

The coordinator package builds one binary per federate, named `federate__<name>`. A
per-federate package holds the launch file and the configuration and starts the
coordinator's binary.

## Connection semantics

| Connection | After delay |
|---|---|
| Inside one federate | `--after-delay` (0 by default, as in one process) |
| Across federates | 0, or the `after_overrides` value |

Under centralized coordination a federate waits at a tag until the RTI grants it, so
the order across processes is the same as in one process. An admitted link waits for
its samples up to the cutoff, publication tag + d + T, so the take timeout T
(`maxwait_ms`), not d, is the slack that covers the network. See
[Missing Messages](missing-messages.md#counting-the-cutoff-from-the-tag).

When one publisher in federate A feeds several subscribers in federate B, the
generator emits one boundary connection and B fans the value out inside. The same
holds for fan-in. Several publishers in other federates that write one subscription's
topic form one fan-in group when every member is admitted (see
[Several publishers into one subscription](missing-messages.md#several-publishers-into-one-subscription)).
The group is delivered in one order: release tag, then the rank of the publisher's
instance name, then publication order. Under centralized coordination
`--federate per-node` delivers the same sequence, at the same tags, as the one-process
program. A group with members both inside and outside the subscriber's federate is
not admitted (`fan_in_mixed_locality`). Under decentralized coordination a tardy record
of a group ends LOST, and when the group already delivered a later message of that
time the run exits with code 3.

## RTI and services

The RTI is built from reactor-c and is part of the `dev` image, so the orchestrator
launch file can start it. The `rti` Compose service runs it alone.

### Docker Compose services

| Service | Purpose |
|---|---|
| `rti` | The RTI alone. `docker compose --profile federation up -d rti`. |
| `lf-build-federated` | Analyze, generate and build `pubsub_example` as two federates, centralized. |
| `lf-build-federated-decentralized` | The same, decentralized. |
| `lf-pubsub-federated` | Run both federates and their own RTI in one container. |
| `lf-pubsub-federated-decentralized` | The same, for the decentralized build. |
| `lf-fed-publisher` | Run only the publisher federate against a running RTI. |
| `lf-fed-subscriber` | Run only the subscriber federate against a running RTI. |

| Variable | Default | Used by | Meaning |
|---|---|---|---|
| `RTI_PORT` | `15045` | RTI, federates | TCP port of the RTI. |
| `RTI_HOST` | `localhost` | `lf-fed-*` | Host a federate connects to. |
| `RTI_NUM_FEDERATES` | `2` | `rti` | How many federates the RTI waits for. |
| `FED_TIMEOUT` | `60` | `lf-fed-*` | Seconds before the federate gets SIGINT. |
| `FEDERATION_ID` | empty (the lfc default) | `lf-fed-*` | Federation ID. Both sides must match. |

Every other variable is in [Environment Variables](../reference/environment-variables.md).

### Endpoint match warm-up

A best-effort DDS sample that a writer publishes before it matched the reader is lost.
Federates started on two hosts can begin while their cross-federate endpoints are
still unmatched. So each federate waits, before the RTI handshake, until every local
writer of a cross-federate topic has matched its readers and every local reader its
writers. `colcon lf-generate` writes the expected counts into each federate's
`system.json` as `cross_federate_topics`. An endpoint that a lifecycle node creates in
a hook is left out, since it does not exist before tag 0. Each federate logs one line
per endpoint and a summary:

```text
ros2_lf_match_warmup: stage=startup topic=/sensor_data side=writer have=1 want=1 matched=1 match_us=1079 lost=0
ros2_lf_match_warmup: stage=startup targets=1 matched=1 timed_out=0 longest_match_us=1079 timeout_ms=5000
```

The wait is bounded by
[`ROS2_LF_MATCH_WARMUP_MS`](../reference/environment-variables.md#ros2_lf_match_warmup_ms),
by default the discovery budget `ROS2_LF_DISCOVERY_TIMEOUT_MS` (5000). On a timeout
the federate logs a FATAL line that names the topic and the counts and exits with
code 3.

## Stopping a federation

A federation stops cleanly on SIGINT:

- **Ctrl-C on `ros2 lf launch`** (or `ros2 launch` of the orchestrator). Every
  federate asks for an LF stop, they all stop at one agreed tag and resign, and the
  RTI then ends. The orchestrator starts the RTI through `rti_guard.py`, because the
  RTI alone ends with code 2 on SIGINT, before the federates could stop.
- **The `lf-fed-publisher` and `lf-fed-subscriber` Compose services**, which send
  SIGINT when `FED_TIMEOUT` expires (default 60 s).

The `lf-pubsub-federated` smoke service is different: it ends its run with
`timeout 30`, which sends SIGTERM, and ignores the exit code.

An RTI you start yourself (the `rti` service) is not guarded, so do not send it the
Ctrl-C.

**A federation that stops at an LF timeout currently fails.** With `--timeout` the
subscriber federate exits with code 3:

```text
[federate__subscriber_node-3] [ERROR] [...] [ros2_lf_binding.corr]: rcllf corr: reconcile: complete_accounting edge 0: receiver's last COMPLETE 5 above the 4 the sender says landed by the stop tag
[ERROR] [launch]: Caught exception in launch (see debug for traceback): rcllf: federate subscriber_node exited 3, the run failed
```

Stop a federation with SIGINT instead. Fast mode (`--fast`) is not supported for
federations in v0.1.2.

**SIGTERM** (from `timeout` or `kill`) also ends the run. The RTI guard gives the RTI
[`RCLLF_RTI_TERM_GRACE_S`](../reference/environment-variables.md#rcllf_rti_term_grace_s)
(3 s) to grant the stop before it passes the SIGTERM on. The generated launch files
set [`RCLLF_EXIT_WITH_PARENT`](../reference/environment-variables.md#rcllf_exit_with_parent),
so a federate ends when its launch process dies. A signal ends any rcllf process even
when its LF stop cannot complete: after
[`RCLLF_STOP_TIMEOUT_MS`](../reference/environment-variables.md#rcllf_stop_timeout_ms)
(3000) without progress, and at most
[`RCLLF_STOP_MAX_MS`](../reference/environment-variables.md#rcllf_stop_max_ms)
(12 s) after the signal, it exits with code 124. A harness that stops a run should
wait that long before it kills.

## Exit codes

| Code | Process | Meaning |
|---|---|---|
| 0 | federate, RTI | Clean end. |
| 1 | federate | A fatal error, for example a reader QoS the binding refuses. |
| 1 | RTI | A federate reported an error. |
| 2 | RTI | The RTI got SIGINT itself. The orchestrator's guard prevents this. |
| 3 | federate | rcllf refused to start or found a broken run: an endpoint match warm-up or peer check timed out, a peer was lost, or the end-of-run check failed (`burst_exceeded`, `worker_lag_exceeded`, `complete_accounting`, `local_undelivered`). A refused external input, stamp anchor or synchronizer input exits 3 too. |
| 4 | RTI guard | A federate's connection closed before it resigned, although the RTI ended with 0. |
| 124 | federate | The startup bound ([`RCLLF_STARTUP_TIMEOUT_MS`](../reference/environment-variables.md#rcllf_startup_timeout_ms)) or the stop bound expired. A Compose federate service whose `FED_TIMEOUT` fired exits 124 too. |
| 128 + N | any | Killed by signal N. |

`ros2 lf launch` reports a federate that exited with a code other than 0 as
`rcllf: federate <name> exited <code>, the run failed`.

## Several hosts

The RTI uses TCP, so a federation runs across hosts once the network allows it:

```bash
# on the RTI host
RTI_NUM_FEDERATES=2 docker compose --profile federation up rti

# on host A
RTI_HOST=rti.host.example docker compose --profile federation run --rm lf-fed-publisher

# on host B
RTI_HOST=rti.host.example docker compose --profile federation run --rm lf-fed-subscriber
```

Each host needs:

- The federate binaries, built in the dev image.
- A route to the RTI host on `RTI_PORT`.
- The same `ROS_DOMAIN_ID`, and DDS discovery that reaches the other host (a
  discovery server where multicast is blocked).
- Synchronized clocks when an admitted edge crosses hosts. Measure the largest offset
  with chrony or PTP and set it on every host as
  [`ROS2_LF_XFED_CLOCK_BOUND_NS`](../reference/environment-variables.md#ros2_lf_xfed_clock_bound_ns),
  in decimal nanoseconds. Hosts that share one clock set
  [`ROS2_LF_XFED_CLOCK_SYNCHRONIZED=1`](../reference/environment-variables.md#ros2_lf_xfed_clock_synchronized)
  instead. Without either, the receiver refuses the other host's records.

Start all hosts within the startup peer check window, `ROS2_LF_DISCOVERY_TIMEOUT_MS`
(5 s by default).

Pick the take timeout of an admitted cross-host edge above the worst network hop plus
the subscriber's lag, with margin for load. Under centralized coordination a link
delay that is too small costs timing, not correctness: the values and their logical
order stay the same, but the subscriber starts later than its tag. Under
decentralized coordination the STP offset must cover the same hop.

For a long-running deployment, run each federate under a supervisor (systemd, for
example) that restarts the whole federation on failure. Restarting one federate does
not work.

## Troubleshooting

**A federate prints `Connected to localhost:15045` and then nothing happens.**
The RTI waits for the other federates. Start them. A federate service without an RTI
ends after `FED_TIMEOUT` seconds.

**A federate repeats `Writing to socket ... failed. Broken pipe`.**
Its connection to the RTI broke, usually because the RTI exited. Restart the RTI
before the next run.

**The RTI logs `WARNING: Failed to accept the socket. Resource temporarily unavailable. Trying again.`**
Harmless. The RTI's listening socket has a 10 s receive timeout, so the warning
appears once per 10 s in which no federate connects, also while the federation runs.
The RTI keeps waiting for a federate that starts late.

**A federate exits 3 with `startup refused ... check peer_check ... after 5000 ms`.**
A cross-federate edge did not see its peer federate within
`ROS2_LF_DISCOVERY_TIMEOUT_MS`. Start the federates within that window, or raise the
variable for a launch with many federates. A publisher or subscription that a
lifecycle node creates in a hook does not exist before tag 0, so the check skips its
edge and logs `skips its peer check`.

**A federate exits 3 with `match warm-up (startup): startup refused: topic '...': the local publisher matched 0 of 1 readers within 5000 ms`.**
DDS did not match a cross-federate endpoint in time (see
[Endpoint match warm-up](#endpoint-match-warm-up)). Check that the peer federate runs
with the same `ROS_DOMAIN_ID` and discovery range, or raise
`ROS2_LF_MATCH_WARMUP_MS` when the federates start far apart.

**The subscriber exits 3 with `complete_accounting` after an LF timeout.**
See [Stopping a federation](#stopping-a-federation). Stop the federation with SIGINT.

**Ctrl-C does not stop a federate service.**
The federate services use `init: true` and `timeout --foreground`, so Ctrl-C reaches
the federate. Ctrl-Z suspends it instead, and the container then looks frozen.

**The RTI exits before the federation runs.**
The RTI serves one run. Once the federation ends or a federate disconnects, it exits.
Restart it before each run
(`docker compose --profile federation down rti && docker compose --profile federation up -d rti`),
or use `lf-pubsub-federated`, which starts its own RTI.

## Known Limitations

- **Fast mode is not supported for federations in v0.1.2.** Run federations in real
  time.
- **A federation that stops at an LF timeout fails** with exit code 3. Stop it with
  SIGINT (see [Stopping a federation](#stopping-a-federation)).
- **No recovery from a failed federate.** When one federate stalls or dies, the
  federation stalls. Restart the whole federation.
- **Self-concurrency of a [Reentrant callback group](../design/callback-groups.md)** is
  not kept across federates, as in one process.
- **Action channels are counted ports.** Every feedback publish and result send happens
  in an LF reaction, and the receiving reaction takes exactly that many samples, each
  with a bounded wait. A sample that does not arrive is traced as `LOST`. Across
  federates every action channel uses the link's cross-federate delay. A server that
  sends from its own thread is refused with
  [RCLLF-H011](../reference/diagnostic-codes.md#rcllf-h011).
- **Action feedback is not a correspondence edge.** rcl returns no sequence number for
  feedback, so it relies on the counted port and its bounded wait.
- **An action server with only external clients runs at arrival time.** Its reactions
  run at tags set by when requests arrive. A server whose action has clients both
  inside and outside the system takes no goal from the outside client.
- **Long node names can break a federated build.** lfc cuts generated C names to 40
  characters. When a federate's wrapper and its node end in the same long name, the C
  compile fails (`_te__..._self was not declared`). Keep the namespace plus node name
  short (`action_example/cancel_client` works, `action_example/action_cancel_client`
  does not).
- **No physical (`~>`) connections** across federates. Every cross-federate link is a
  logical `->` with an after delay.
- **Decentralized coordination is experimental.** The STP handler only logs a tardy
  message. A stalled federate still stalls the whole federation.
- **Tested systems:** `pubsub_example` (two federates, per node, centralized and
  decentralized), `service_example` and `action_example` (two federates, per node).

# Federation

!!! info "Two coordination modes"
    rcllf's `--federate` flag defaults to
    [**centralized**](../design/time-and-coordination.md#regime-2-federated-centralized)
    coordination. An RTI mediates every cross-federate [tag](../design/glossary.md#tag)
    advance, and messages are never tardy. Pass `--coordination decentralized`
    (or set `coordination: decentralized` in a custom `federation.yaml`) for LF's
    [**decentralized**](../design/time-and-coordination.md#regime-3-federated-decentralized)
    mode instead (peer-to-peer, STP-offset based, tardy messages are possible).
    See [Time & Coordination Model](../design/time-and-coordination.md) for the
    full picture, and [Coordination mode](#coordination-mode) below for the flag,
    the runnable example, and the tardiness actually observed.

## Overview

Federation splits an LF-controlled ROS 2 system across multiple OS processes ([federates](../design/glossary.md#federate-federation)), each running independently but coordinated by the **LF [Run-Time Infrastructure (RTI)](../design/glossary.md#rti-run-time-infrastructure)** daemon. The RTI maintains a shared logical clock across all federates. DDS continues to transport all data unchanged.

**Key properties:**

- LF time authority is preserved across process boundaries (RTI coordinates tags).
- DDS handles all pub/sub, service, and action data transport (unchanged from single-process).
- Each federate is its own OS process / container.
- Federation is **strictly opt-in**. `colcon lf-generate` without `--federate` produces single-process output exactly as before.

**Federation is all-or-nothing.** Unlike plain ROS 2, where any node can come and go, an LF federation is one logical program that happens to run across multiple processes. All federates that the program declares must be present for the program to make progress. If any federate stalls or dies, the others stall waiting at their next cross-federate event.

---

## Quick Start

### Prerequisite

Make sure the `dev` image is built. It bundles both `lfc` and the RTI binary (the
standalone `rti` service instead uses its own `lf-rti` Dockerfile target/image):

```bash
docker compose build dev
```

All federation services live under the `federation` profile.

### One-shot run (everything in a single container)

```bash
docker compose --profile federation run --rm lf-build-federated     # generate + build the federation
docker compose --profile federation run --rm lf-pubsub-federated    # start RTI + both federates internally
```

`lf-pubsub-federated` uses the orchestrator launch file (`lf_bringup_federated.launch.py`) which starts the RTI as a child process, then both federates. Logs are interleaved in one terminal. The container runs for 30 s by default.

What a working run prints (trimmed, from a run of exactly these two commands; your
timestamps and epochs differ):

```text
[RTI-1] RTI: Number of federates: 2
[RTI-1] WARNING: Failed to accept the socket. Resource temporarily unavailable. Trying again.
[federate__publisher_node-2] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted egress edge 0 (global 0) PublisherNode -> /subscriber_node on '/sensor_data' (d = 0 ns, L = 200000000 ns)
[federate__subscriber_node-3] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted ingress edge 0 PublisherNode -> SubscriberNode on '/sensor_data' (d = 0 ns, ...)
[RTI-1] RTI: All expected federates have connected. Starting execution.
[federate__publisher_node-2] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[federate__subscriber_node-3] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[federate__publisher_node-2] [INFO] [...] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[federate__subscriber_node-3] [INFO] [...] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
...
==> Federated smoke test complete
```

Each federate is its own process (`federate__publisher_node-2`, `federate__subscriber_node-3`),
the `/sensor_data` edge is admitted on both sides (egress and ingress), and every
`Publishing` line is followed by the `I heard` line with the same values. The RTI's
`Failed to accept the socket` warning while the federates connect is harmless (see
[Troubleshooting](#troubleshooting)).

### Multi-container run (separate logs per service)

```bash
docker compose --profile federation up -d rti                        # RTI in the background
docker compose --profile federation run --rm lf-fed-publisher        # publisher federate
docker compose --profile federation run --rm lf-fed-subscriber       # subscriber federate
docker compose --profile federation down rti                         # stop RTI when done
```

Each `lf-fed-*` service starts one federate against the running RTI. You can run them in three separate terminals to see each one's logs cleanly. Each federate runs for at most `FED_TIMEOUT` seconds (default 60), so a misconfigured run terminates rather than hanging forever. A working run ends the same way: the federate that reaches its `FED_TIMEOUT` first gets a Ctrl-C, asks the RTI to stop, and both federates stop at the same logical time and resign. The end of a working run (trimmed, from a run with `FED_TIMEOUT=20` for the publisher and `30` for the subscriber):

```text
publisher:   [federate__publisher_node-1] [INFO] [...] [publisher_node]: Publishing: temp=51.0, humidity=71.0, pressure=1044.0
subscriber:  [federate__subscriber_node-1] [INFO] [...] [subscriber_node]: I heard: temp=51.0, humidity=71.0, pressure=1044.0
publisher:   [INFO] [federate__publisher_node-1]: process has finished cleanly [pid ...]
subscriber:  [federate__subscriber_node-1] Fed 1 (subscriber_node_main): Connection to the RTI closed with an EOF.
subscriber:  [INFO] [federate__subscriber_node-1]: process has finished cleanly [pid ...]
rti:         RTI: Federate 1 has resigned.
rti:         RTI: Federate 0 has resigned.
rti:         RTI is exiting.
```

The service whose timeout fired exits with code 124 (the status of `timeout`), the other with 0. Both federates print the same `Elapsed logical time`. For a longer run, raise the timeout:

```bash
FED_TIMEOUT=300 docker compose --profile federation run --rm lf-fed-publisher    # longer run
```

`docker compose --profile federation up rti lf-fed-publisher lf-fed-subscriber` brings all three up at once with mixed logs. Ctrl+C stops them all.

---

### Your own package

The pipeline takes the same flags. In the dev container, from `packages/`:

```bash
colcon lf --package my_robot --federate per-node
source install/setup.bash
ros2 lf list --package my_robot          # the system shows "ready (federated)"
ros2 lf launch my_robot bringup
```

`ros2 lf launch` starts the RTI and every federate from the generated orchestrator
launch file, as in the one-shot run above. It runs what the last `colcon lf` (or
`colcon lf-generate`) produced for that launch file: after a `--federate` run,
`ros2 lf launch my_robot bringup` starts the federation, and `ros2 lf list` shows
`ready (federated)`. Run `colcon lf --package my_robot` without `--federate` to get the
one-process program back. `--coordination decentralized` selects the
other coordination mode (see [Coordination mode](#coordination-mode)).

## How federation differs from plain ROS 2

| Property | ROS 2 | LF Federation |
|---|---|---|
| Failure isolation | Each node independent (crash one, others unaffected) | All-or-nothing (any federate stalling stalls the federation) |
| Time | Local clock (or `/clock`) | RTI-coordinated logical time across all federates |
| Determinism | Best-effort, no ordering guarantees across nodes | Deterministic ordering invariants preserved |
| Late-joiner support | Yes (nodes can come and go) | No (all federates declared at compile time) |
| Coordinator | None (peer-to-peer DDS) | RTI is a central time authority ([centralized coordination](../design/time-and-coordination.md#regime-2-federated-centralized)) |

The federate binaries themselves still use DDS for **data**, so non-LF ROS 2 nodes can subscribe to topics published by a federate. The RTI controls only **logical-time progression** between federates, not data routing.

---

## Partition Modes

The `--federate` flag selects how nodes are grouped into federates.

| Mode | Argument | Description |
|---|---|---|
| Per-node | `--federate per-node` | One federate per ROS 2 node. Maximum split. |
| Per-process | `--federate per-process` | One federate per launch process group (composable container or standalone node). |
| Custom YAML | `--federate path/to/federation.yaml` | Explicit user-defined partition. |

### Per-node

Every node becomes its own federate. Federate name = sanitized node name. For a 2-node `pubsub_example` this produces 2 federates.

### Per-process

Groups nodes by their launch process boundary:

- Composable nodes in the same `ComposableNodeContainer` → one federate (they share an OS process).
- Non-component `Node` actions → one federate each (each is its own executable).

Falls back to `per-node` if no launch graph information is available.

### Custom YAML

Use `--emit-federation-template` to generate a starter YAML, then edit it:

```bash
colcon lf-generate --emit-federation-template \
  --system-json lf-analysis/pubsub_example/bringup/system.json
# Output: lf-analysis/pubsub_example/bringup/federation.yaml
```

Pass `--force` to overwrite an existing template.

The template defaults to one federate per node. Coarsen by moving nodes under fewer federate names:

```yaml
system: pubsub_example

federates:
  publisher_side:
    nodes:
      - publisher_node
      # ... further nodes

  subscriber_side:
    nodes:
      - subscriber_node

after_overrides: []
```

**Rules:**

- Every node must appear in exactly one federate.
- Every federate must have at least one node.
- Federate names must be valid identifiers (`a-z`, `A-Z`, `0-9`, `_`).

#### Per-link after-delay overrides

Cross-federate connections default to `after 0`. The receiver runs at the
publisher's time, one microstep after it at (t, m+1), so a split into federates adds no
logical latency. Inside one process an admitted d = 0 edge runs the first record at the
publication tag (t, m) itself, so a cross-federate d = 0 link lands one
microstep later than the same link in one process. State a d > 0 per link:

```yaml
after_overrides:
  - from: publisher_node.sensor_data
    to:   listener_node.sensor_data
    after: 8ms   # or the LF spelling, '8 msec'
```

`after` takes the same time grammar as `--after-delay` and `execution.yaml`,
so both the terse spelling (`8ms`, `500us`, `100ns`, `2s`) and the
LF spelling (`8 msec`, `500 usec`, `100 nsec`, `2 sec`) work. A unit is
required. Only `0` may omit one, because an unsuffixed number is ambiguous and
guessing a unit is how a wrong delay reaches generated output unnoticed.

`from` and `to` are the `<instance>.<port>` names of the two ends of a
cross-federate link, as they appear in the top-level federated `.lf` file. An
entry that names no cross-federate link stops generation with an error that
quotes the entry and lists the valid keys of the federate that owns its `from`
instance. A misspelled port used to fall back to the default
delay without a word.

---

## Coordination mode

`colcon lf-generate --federate ... --coordination {centralized,decentralized}`
selects the LF coordination mode. `centralized` is the default (regime 2,
above). `decentralized` emits LF's `coordination: decentralized` target
property (regime 3, [Time & Coordination
Model](../design/time-and-coordination.md#regime-3-federated-decentralized)):
peer-to-peer, no RTI-mediated tag advance, STP-offset based, tardy messages
are possible. A custom `federation.yaml`'s own `coordination:` key is used
when `--coordination` is omitted. The flag always wins when both are given.

### STP offset

Each cross-federate **input** port gets a safe-to-process (STP) offset and a
logging-only reaction that fires when a message arrives after its tag was
already safe to process (i.e. it's tardy). Set it per link, alongside
`after_overrides`:

```yaml
coordination: decentralized
after_overrides:
  - from: publisher_node.sensor_data
    to:   subscriber_node.sensor_data
    after: 5ms
    stp_offset: 20ms
```

`stp_offset` uses the same time grammar as `after`. Omitted, it defaults to
10 ms (a starting point to tune, not a
measured value) under decentralized coordination. It is ignored entirely
under centralized coordination (no STP reaction is emitted there, centralized
output is byte-identical to before this feature).

**A federate needs its own independent tag-advancing source (a timer, another
input) to ever observe a violation.** A purely reactive federate, one whose
only trigger is the network input itself, can never be "ahead" of that
input, because the incoming message *is* the next tag it can advance to. The
2-federate `pubsub_example` below is exactly this shape, so its default,
untuned run shows zero violations regardless of how small `stp_offset` is.
This is real LF decentralized-coordination behavior, not a gap in this
generator, see [regime
3](../design/time-and-coordination.md#regime-3-federated-decentralized) for
why.

**One message per logical time on a decentralized link.** A cross-federate link
with `after d` maps every microstep of one time to one tag. Under decentralized
coordination the receiver has already processed that tag when the second message
of the time arrives, so reactor-c judges it tardy and runs it at a later tag or
loses it. A link that is not admitted for publication correspondence therefore
exits with code 3 when its publisher writes it at a second microstep of one time.
Centralized coordination allows two. Keep the
publishers of such a link to one write per time, or put them in one federate
with their subscriber.

**Admitted links under decentralized coordination.** A link admitted for
publication correspondence is admitted under decentralized coordination too. Its
records travel on their own `xcorr_g<N>` link. With d > 0 they are flushed at t + 1 ns
with `after d - 1 ns`, so every write of a time arrives in one message and the
one-message rule above does not limit it; with d = 0 they are sent at the publishing tag
over `after 0`. The link's STP offset is the `stp_offset` of its `after_overrides` entry, else the
10 ms default. A record that arrives after its intended tag t + d (a tardy message)
is not fatal there: each of its messages ends LOST at the arrival tag and runs the
subscription's `message_lost` (and requested-deadline) handler once. Inside that
handler `ros2_lf_message_lost_context()` returns the message's intended tag (elapsed
ns), publisher GID and ordinal, with `tardy = 1`. Under centralized coordination a
record off its tag still exits with code 3. BNA inputs are admitted under
decentralized coordination too (see [BNA synchronizer](bna-synchronizer.md)).

### Budget check

A decentralized link holds without a tardy message when its delay and STP offset
cover the worst delivery:

    d + W >= L + eps + X + W_send

`d` is the link's `after`, `W` its STP offset (`stp_offset`, alias `absent_after`;
10 ms when undeclared). A link may declare the other terms in its `after_overrides`
entry: `max_latency` (`L`, worst network latency), `clock_error` (`eps`, clock offset
between the two hosts) and `max_exec` (`X`, worst execution time of the sending
reaction). `W_send` is the largest STP offset of the sending federate's own
decentralized inputs, generated correspondence links included: the sender itself
waits that long before it runs. Generation refuses a link whose terms violate the
inequality and names the link and every number:

```text
ERROR: budget check refused: link 'pub.data -> sub.data' violates d + W >= L + eps + X + W_send: ...
short by 1000000 ns
```

A link with some but not all three terms is refused too, a link with none is
reported as `WARNING: ... declares no budget` and not checked. Under centralized
coordination the terms are refused, the RTI grants tags there and no budget applies.
The LF `maxwait` is not a term: the generator emits none.

### Example: decentralized `pubsub_example`

```bash
docker compose --profile federation run --rm lf-build-federated-decentralized
docker compose --profile federation run --rm lf-pubsub-federated-decentralized
```

Uses `packages/src/pubsub_example/federation.yaml` (`coordination:
decentralized`, `stp_offset: 1usec` on the one cross-federate link). Verified
(2026-09-24, real container run): the generated `Bringup_federated.lf` carries
`coordination: decentralized,`. The subscriber federate's boundary reactor
carries `reaction(sensor_data) {=} STP(1 usec) {= lf_print_warning(...) =}`
next to its existing forwarding connection. Every heard value matched a
published value in order (same check as
`tools/colcon_lf/tests/runtime/runtime_support.py:find_pubsub_mismatches`, 0
errors, 26/26). With the topological precondition above met (verified with an
added local timer and an artificial send-side delay, not shipped in the
default example, see the STP-offset note above for why a bare 2-federate
reactive pubsub can't otherwise show it), the STP handler fired and reactor-c
itself logged its own downstream-propagation warning for the plain forwarding
connection, which has no handler of its own:

```
Fed 1 (subscriber_node_main): WARNING: STP violation: tardy message on federate
  boundary port 'sensor_data' (-> subscriber_node.sensor_data) at tag (...)
Fed 1 (subscriber_node_main): ERROR: STP violation occurred in a trigger to
  reaction 1, and there is no handler.
```

Both lines are expected. The first is this generator's handler, and the second is
reactor-c's own fallback for the un-annotated forwarding connection reading
the same tardy event. No messages were dropped or reordered.

---

## Generated Output

Federation produces **N+1 sibling colcon packages** under `packages/lf-generated/`. For `pubsub_example` with `--federate per-node`:

```
lf-generated/
├── lf_generated_pubsub_example_bringup_federated/                      # coordinator package
│   ├── package.xml
│   ├── CMakeLists.txt                                   # runs lfc, builds federate binaries
│   ├── lf/
│   │   ├── Bringup_federated.lf                         # top-level federated reactor
│   │   ├── include/ros2_lf_bindings.cmake               # cmake-include applied to each federate
│   │   ├── publisher_node/PublisherNodeSystem.lf        # per-federate reactor (plain, not main)
│   │   └── subscriber_node/SubscriberNodeSystem.lf
│   └── launch/lf_bringup_federated.launch.py            # orchestrator: RTI + all federates
├── lf_generated_pubsub_example_bringup_federated_publisher_node/       # per-federate launch package
│   ├── package.xml
│   ├── CMakeLists.txt                                   # installs launch + system.json only
│   ├── system.json                                      # filtered to this federate's nodes
│   ├── src/{ros2_lf_startup.cpp, ros2_lf_reaction_ids_init.cpp, node_factories.cpp}
│   └── launch/lf_publishernodesystem.launch.py
└── lf_generated_pubsub_example_bringup_federated_subscriber_node/
    └── ...
```

The coordinator package owns the federate binaries. Its CMakeLists invokes lfc on the top-level `Bringup_federated.lf`, which produces one binary per federate (named `federate__<fed_name>`). Per-federate packages are launch + config only. They reference the coordinator's binary by package and executable name.

---

## Connection Semantics

| Connection type | LF syntax | After delay |
|---|---|---|
| Intra-federate | `->` | `--after-delay` (0 default, same as single-process) |
| Cross-federate | `->` | 0 default, or `after_overrides` value |

The RTI enforces logical-time ordering on cross-federate connections
([centralized coordination](../design/time-and-coordination.md#regime-2-federated-centralized)).
A federate blocks at a tag boundary until the RTI grants permission to advance, ensuring determinism across process boundaries.

An admitted link waits for its samples up to the take cutoff, publication tag + d + T, so
the take timeout T, not d, is the slack that has to cover the network. A legacy link
takes at its tag; give it a d that exceeds the worst-case transport latency (5 ms is
comfortable for LAN, not for WAN).

### Fan-out and fan-in across the boundary

When one publisher in federate A feeds multiple subscribers in federate B, the generator emits **one** boundary connection (`A.port -> B.port after d`) and the receiving federate fans the value out to each inner subscriber internally. LF only allows a port name to appear once on the right-hand side of a top-level connection, so duplicating boundary edges per subscriber would fail to compile. The same shape works for fan-in (multiple inner publishers feeding one boundary output port).

Several publishers in other federates that write the topic of one subscription form one
fan-in group when every member is admitted (see
[several publishers into one subscription](../design/time-and-coordination.md#several-publishers-into-one-subscription)).
The subscriber's federate delivers the group in one order: release tag, then the rank
of the publisher's instance name, then publication order. Generation fixes the ranks
from the whole system before it splits it, so under centralized coordination
`--federate per-node` delivers the same sequence as the one-process program
(experiment G-G). The tags and microsteps are the same as well: the
drain reaction takes a record from another federate at its release tag, microstep 0,
as a local drain woken at that tag does, so it runs before a timer of the same tag
under `subscriptions_first` in both builds. This holds for
every record of a fan-in group due at that tag: one drain firing delivers them all, in
rank then publication order, one callback invocation each, and schedules itself again
only for a record released at a later tag, in both builds alike. A
group with members in the subscriber's own federate
and in another federate stays legacy as a whole (`fan_in_mixed_locality`), for example
`--federate per-process` with one publisher in the subscriber's process; the same
system in one process is admitted. Under decentralized coordination the order holds
while every record arrives within its STP window. A tardy record ends `LOST`: its
sample is never delivered, and the event callbacks report it at the later tag the
record came in, outside the group's order. When the group already dispatched a sample
or a missing message of that time that sorts after the late record, the binding counts
this as an order violation and the run exits with code 3, a known open limit. This case is argued from the design and has no experiment.

### Boundary port names

A link that is not admitted for publication correspondence uses a boundary port named after the topic. When that name equals a reactor instance inside the same federate (for example when every topic carries the name of its publishing node), the port takes the name `<instance>_<topic>` instead. lfc would otherwise declare the same C field twice. An admitted link uses an `xcorr_g<N>` port and never collides.

---

## RTI Infrastructure

The RTI is built from the `third_party/reactor-c` submodule and is bundled into:

- The standalone `lf-rti` Docker image (used by the `rti` compose service).
- The `dev` image (so `lf-pubsub-federated` and the orchestrator launch file can spawn the RTI as a child process).

### Docker Compose services

| Service | Purpose |
|---|---|
| `rti` | Standalone RTI daemon. Start with `docker compose --profile federation up -d rti`. |
| `lf-build-federated` | Analyze + generate + build `pubsub_example` as 2 federates, centralized coordination. |
| `lf-build-federated-decentralized` | Same, decentralized coordination (regime 3), via `packages/src/pubsub_example/federation.yaml`. |
| `lf-pubsub-federated` | Run the 2-federate pubsub example with self-managed RTI (single container). |
| `lf-pubsub-federated-decentralized` | Same, for the decentralized build. |
| `lf-fed-publisher` | Run only the publisher federate. Requires an externally-managed RTI. |
| `lf-fed-subscriber` | Run only the subscriber federate. Requires an externally-managed RTI. |


### Configuration

| Environment variable | Default | Used by | Description |
|---|---|---|---|
| `RTI_PORT` | `15045` | RTI, federates | TCP port the RTI listens on. |
| `RTI_HOST` | `localhost` | `lf-fed-*` | Host the federate dials for the RTI. |
| `RTI_NUM_FEDERATES` | `2` | `rti` | How many federates the RTI expects. |
| `FED_TIMEOUT` | `60` | `lf-fed-*` | Seconds a standalone federate runs. When it expires the federate gets SIGINT and the whole federation stops cleanly; a federation that never converges also ends then. |
| `FEDERATION_ID` | `""` | `lf-fed-*` | Optional federation ID. Both sides must match. Empty = lfc default. |
| `ROS2_LF_MATCH_WARMUP_MS` | `ROS2_LF_DISCOVERY_TIMEOUT_MS` (5000) | federates | Bound of the endpoint match warm-up below. An endpoint still unmatched at the bound refuses the start (exit 3). `0` checks once and does not wait. |
| `ROS2_LF_TEST_START_DELAY` | unset | federates | Test only. `<name>:<ms>` delays a federate whose executable name contains `<name>` by `<ms>` (1 to 10000) after its peer check, to force startup skew in a pilot. Logs a WARN. |
| `ROS2_LF_TEST_RESIGN_DELAY` | unset | federates | Test only. `<name>:<ms>` delays the RTI resign of a federate whose executable name contains `<name>` by `<ms>` (1 to 10000), after its end-of-run counts went out. Logs a WARN. |

### Endpoint match warm-up

A best-effort DDS sample that a writer publishes before it has matched the reader is
lost, and matching is not symmetric in time: the reader can see the writer some
100 ms before the writer sees the reader. On one host discovery settles before the
first publication, but federates started on two hosts can begin while their
cross-federate endpoints are still unmatched. The warm-up hardens that start: it
makes sure discovery is done before the first tag.

`colcon lf-generate` therefore writes the key `cross_federate_topics` into each
federate's `system.json`: every topic with endpoints both inside and outside the
federate, with its publisher and subscriber node counts over the whole system. A
node counts once however many endpoints it has on the topic, and an endpoint a
lifecycle node makes in a hook is left out, since it does not exist before tag 0.
Each federate waits, before the RTI handshake, until every local
writer of such a topic has matched that many readers and every local reader that many
writers. The wait is bounded by `ROS2_LF_MATCH_WARMUP_MS`. It runs in fast and real-time
mode alike; a matched federate passes it at once, and the one program has no such
topics. Each federate logs one stats line per endpoint and a summary per stage:

```text
ros2_lf_match_warmup: stage=startup topic=/brake_cmd side=writer have=1 want=1 matched=1 match_us=171420
ros2_lf_match_warmup: stage=startup targets=4 matched=4 timed_out=0 longest_match_us=171420 timeout_ms=5000
```

On a timeout the federate refuses to start: it logs a FATAL line that names the topic
and the counts and exits with status 3 (see Troubleshooting). A sample
published before its reader matched would be lost with no count, so the run does not
start in that state. Without its own variable the bound is the discovery budget,
`ROS2_LF_DISCOVERY_TIMEOUT_MS`, because the match is the end of the same discovery. A federate generated by an earlier version has no
`cross_federate_topics` and skips the warm-up.

### Cutoffs in fast mode

The cutoff of a cross-federate message is its publish tag plus the edge's delay `d`
and timeout `T`. In real time the receiver judges it on the physical clock, which
tracks logical time. A federate in fast mode (`-f true`) skips the wait for the
start time the RTI sets about one second ahead, so its logical time runs ahead of
physical time, and each federate begins tag 0 as soon as it leaves the handshake. A
fast federate therefore judges cutoffs on its own logical tag: a sample is on time
when it is in the table before the receiver's tag passes the cutoff, whatever the
physical gap between the sample and its record. The join still waits at most `T`
of physical time for a sample that has not come. A publisher stamps its records with
the same clock, so the receiver checks a record's stamp against its own logical tag. The
worker lag check (physical minus logical time) is off in a fast federate. Real time and the one program keep
the physical clock. In earlier versions a fast federate that left the handshake more
than `d + T` (21 ms on R2's `/brake_cmd`) after its peer released the early first
sample as late and ended the record timed out.

---

## Multi-Machine Deployment

The RTI uses TCP, so federation works across machines as soon as the network allows it.

```bash
# On the RTI host (any one of the machines, or a dedicated control plane host):
docker run --network host -p 15045:15045 \
  ghcr.io/cps-rwth/rcllf:lf-rti -n 2 -p 15045

# On host A (publisher):
RTI_HOST=rti.host.example docker compose --profile federation run --rm lf-fed-publisher

# On host B (subscriber):
RTI_HOST=rti.host.example docker compose --profile federation run --rm lf-fed-subscriber
```

Each machine needs:

- The dev image (or pre-built federate binaries copied as a portable artifact).
- The federation `system.json` filtered to that machine's federates.
- Network reachability to the RTI host on `RTI_PORT`.
- A consistent `ROS_DOMAIN_ID` so DDS topics flow between hosts (and DDS discovery configured if multicast is blocked, e.g. FastDDS discovery server).

For long-running deployments, wrap each federate in a process supervisor (systemd, k8s) that restarts the **whole federation** on failure. Recovering individual federates does not work.

---

## Measured overhead

Smoke measurements on shared 32-core workstations, 10 runs of 30 s per
row. Latency and CPU are smoke numbers, not a timing benchmark. The correctness columns
are the results. Every row's logical dispatch sequence was identical across its runs, and
every `pubsub_example` run heard a gap-free sequence of the published values.

### One host

`pubsub_example`, `publisher_node -> subscriber_node`:

| build | logical | physical p50 / p99 / max | subscriber lag p50 / p99 | CPU cores |
|---|---|---|---|---|
| single process | 1 ms | 1.019 / 1.087 / 1.110 ms | 0.051 / 0.494 ms | 0.029 |
| 2 federates, centralized | 5 ms | 5.107 / 5.185 / 5.216 ms | 0.131 / 0.253 ms | 0.044 |

The subscriber lag is the time from a dispatch's tag to its start. Federation adds about
0.08 ms of it at p50, the wait for the RTI's tag advance grant. The after delay adds 4 ms
(5 ms across federates instead of the 1 ms `--after-delay` inside a process, the defaults
when these runs were taken; both are 0 now). On one host
the cross-federate delay is almost entirely latency added on purpose.

### Two hosts

RTI and `publisher_node` on one host, `subscriber_node` on the other, one switch, clocks
by NTP with a measured offset of 12 us +- 41 us:

| cross-federate delay | logical | physical p50 / p99 / max | subscriber lag p50 / p99 / max | network hop p50 / p99 / max |
|---|---|---|---|---|
| 5 ms (the earlier default) | 5 ms | 5.092 / 5.261 / 5.329 ms | 0.133 / 0.282 / 0.345 ms | 0.144 / 0.296 / 0.337 ms |
| 0.1 ms | 0.1 ms | 0.477 / 1.725 / 2.083 ms | 0.422 / 2.569 / 2.792 ms | 0.149 / 0.312 / 0.368 ms |

The 5 ms delay (then the default) held at every observed percentile. The slowest of 508 cross-host hops
took 0.337 ms. With 0.1 ms, centralized coordination still delivers the same values in
the same logical order, so nothing breaks logically. What fails is the timing. The
subscriber cannot run at its tag and starts 0.42 ms (p50) to 2.6 ms (p99) late, so the
real latency is about 5 times the logical latency the program states.

### LAN guidance

- Pick the cross-federate delay above the worst network hop plus the subscriber's lag on
  your network, then add margin for load. On the LAN above the sum stayed below 0.7 ms,
  so 5 ms leaves ample margin. Measure your own network the same way before you shrink it.
- In centralized mode a too-small delay costs timing, not correctness. In decentralized
  mode the STP offset must cover the same hop, or a federate with its own timer runs past
  the tag and the message arrives tardy (an STP violation, see [STP offset](#stp-offset)).
- Set `ROS2_LF_XFED_CLOCK_BOUND_NS` on both hosts when an admitted edge crosses hosts,
  and keep the hosts' clocks synchronized.
- Start all hosts within the startup peer check window (5 s by default).

---

## Troubleshooting

**Publisher prints "Connected to localhost:15045" then nothing happens.**
The publisher reached the RTI but the RTI is waiting for the second federate. Start the subscriber. If you forgot to start the RTI, the federate exits after `FED_TIMEOUT` seconds (plus up to 15 s if it does not react to Ctrl-C).

**Subscriber spams "Writing to socket … failed. Broken pipe".**
The federate's connection to the RTI broke, usually because the RTI exited (federation completed or other federate disconnected). Restart the RTI before the next run.

**RTI logs `WARNING: Failed to accept the socket. Resource temporarily unavailable. Trying again.`**
Harmless. The RTI's listening socket has a 10 s receive timeout, so the warning appears once per 10 s in which no federate connects, while the RTI waits for its federates and again while the federation runs. Pinned reactor-c treated that timeout as fatal: the RTI logged `RTI failed to accept the federate.` without the `Trying again.` suffix, started the federation with the federates it had, refused the late one and exited. The image now builds the RTI with a local fix from `third_party/patches/reactor-c/`, so the RTI waits for a federate that starts late. Federates are compiled against the runtime that lfc bundles, which had the same timeout in its peer-to-peer accept loop: a federate that waited more than 10 s for its peer logged `Federate failed to accept the network abstraction.` and stopped listening. The image patches that bundled runtime as well, and a federate now logs `Trying again.` and keeps listening. An image built before that fix still has the old behaviour. With it, start the RTI at most a few seconds before its federates.

**A federate exits with code 3 and "startup refused ... check peer_check ... after 5000 ms".**
A cross-federate edge did not see its peer federate within `ROS2_LF_DISCOVERY_TIMEOUT_MS` (5000 by default). The other federates then wait for the RTI until you stop them. Start the federates on different hosts within that window, and raise the variable for launches with many federates (the 24-federate reference system failed this check in 3 and 7 of 10 runs with the default and in 0 of 10 runs with `ROS2_LF_DISCOVERY_TIMEOUT_MS=30000`). A lifecycle node's publisher or subscription made in a lifecycle hook does not exist before tag 0, so the check skips its edge and logs `skips its peer check`. When the hook makes it, that end is compared with its peer (type and QoS), and a mismatch is refused: a publisher exits 3 with `bind refused ... check peer_check`, a reader is refused with the code `peer_check`.

**A federate exits 3 with "match warm-up (startup): startup refused: topic '...': the local publisher matched 0 of 1 readers within 5000 ms".**
DDS did not match a cross-federate endpoint within `ROS2_LF_MATCH_WARMUP_MS` (see
[Endpoint match warm-up](#endpoint-match-warm-up)). A sample published before the match
would be lost uncounted, so the federate refuses to start. Check that the
peer federate runs and shares the `ROS_DOMAIN_ID` and discovery range, or raise the
variable when the federates start far apart.

**Ctrl+C on the orchestrator launch.**
Ctrl+C stops every federate through an LF stop request, they all stop at one
agreed tag, resign, and the RTI then ends on its own. For the federated
`pubsub_example` `ros2 launch` reports every process, the RTI included, as
finished cleanly (10 of 10 runs). A Ctrl+C during an `action_example`
goal ends the goal ABORTED from the server's `reaction(shutdown)`, with or without
federation. The
orchestrator starts the RTI through `rti_guard.py` next to the launch file,
because reactor-c's RTI ends with exit code 2 on any SIGINT, before the
federates could stop. An RTI you start yourself (the `rti` compose service,
`RTI_HOST` set) is not guarded, so do not send it the Ctrl+C.

**SIGTERM to the orchestrator launch (`timeout`, `kill`).**
`ros2 launch` answers a SIGTERM by exiting without stopping its processes, and
`timeout` sends the SIGTERM to the RTI guard and the federates as well. In earlier versions
the guard passed it on to the RTI at once, the federates' LF stop
then waited for an RTI that was gone, and they outlived the launch; the next
run's peer check then refused to start. Now three things hold. The guard
gives the RTI 3 s (`RCLLF_RTI_TERM_GRACE_S`) to grant the stop before it
forwards a SIGTERM. The generated launch files set `RCLLF_EXIT_WITH_PARENT` to
the launch process's pid, so a federate and the guard get SIGTERM when the
launch process dies, and exit at once if it died before they started. And a
signal ends any rcllf process even when its LF stop cannot complete or its teardown
blocks. The bound is `RCLLF_STOP_TIMEOUT_MS` (3000 by default, `0` turns it off)
counted from the last progress, since a dispatch that returns re-arms it, and the
whole stop is capped at `RCLLF_STOP_MAX_MS` from the signal (default four times the
bound, 12 s). The process then exits with code 124, which its log announces when the
bound is armed (`process end bounded to ... exits with code 124`). A harness that
stops a run should wait at least that cap before it kills, see the
[binding reference](../reference/ros2-lf-binding.md).

**Ctrl+C does not stop a federate.**
Federate services use `init: true` (tini as PID 1) and `exec timeout --foreground` so signals propagate cleanly. If Ctrl+C still feels unresponsive, make sure you are using Ctrl+C and not Ctrl+Z (Ctrl+Z suspends the process, so the container looks frozen because it actually is).

**RTI exits before I can run the federation.**
The RTI is a per-run daemon, not a long-running service. Once all expected federates have connected and the federation completes (or any federate disconnects), the RTI exits. Restart it before each run with `docker compose --profile federation down rti && docker compose --profile federation up -d rti`, or use the all-in-one `lf-pubsub-federated` service which spawns its own RTI.

**`fed-gen/` or `log/` directory appears in the repo root.**
You ran `lfc` or `colcon` directly from the workspace root by accident. Both are gitignored at the root and should be deleted (`rm -rf fed-gen log`). All build invocations from inside the toolchain do this in the right working directory.

---

## Known Limitations

- **Self-concurrency in [reentrant callback groups](../design/callback-groups.md)** is not preserved across federate boundaries (same as single-process limitation from Phase 4).
- **Action channels are counted ports.** The server steps its goals from a timer, so every feedback publish and result send happens in an LF reaction. The binding counts each rcl_action send, and the reaction raises its port by that count. The receiving reaction takes exactly that many samples, each with a bounded wait, and traces `LOST` for a sample that does not arrive. Goal request and goal response use the link delay of the connection (0 in one process). Feedback, result, cancel request and cancel response use 1 ms in one process (`ACTION_ASYNC_AFTER_DELAY_NS`). Across federates every action channel uses the cross-federate delay of the link (0 by default, or an `after_overrides` entry of the federation YAML). `action_example/cancel.launch.py` and `bringup.launch.py` gave identical client output and identical logical tags in 10 of 10 federated runs. A server that sends from its own thread is refused with RCLLF-H011.
- **Action client feedback is not a correspondence edge.** `rcl_action_take_feedback` returns no sequence number, so feedback relies on the counted port and its bounded wait only.
- **An action server with only external clients is driven by arrival time.** A server whose action has no client inside the LF system gets a physical action, so its reactions run at tags set by when requests arrive. Mixed internal and external clients on one action are not supported, and a server whose action also has a client inside the LF system takes no goal from a client outside it.
- **Long node names can break a federated build.** lfc cuts generated C names to 40 characters. When a federate's wrapper and the node inside it end in the same long name, lfc renames one of them and the federate's own setup code still uses the old name, so the C compile fails (`_te__..._self was not declared`). Keep the namespace plus node name short (`action_example/cancel_client` works, `action_example/action_cancel_client` does not).
- **`~>` (physical) connections** for cross-federate links are not yet exposed in the generator. All cross-federate links use logical `->` with an after delay.
- **Fast mode can report late cross-federate samples as lost.** A federation runs fast (`--fast` or `RCLLF_LF_ARGS="-f true"`, see [Fast Mode](fast-mode.md)), but a counted cross-federate port whose sample has not arrived at its tag reports it lost (warning P17e) and the reaction runs without it. Three of five fast pilot runs lost a sample on a 1 ms topic and one of them produced different camera records from then on; no real-time run lost one.
- **Federate failure recovery** is intentionally not handled. When one federate stalls or dies, the federation stalls. Restart the whole federation.
- **Currently validated systems**: `pubsub_example` (2 federates, `per-node`, centralized **and** decentralized coordination), `service_example` (2 federates, `per-node`). `action_example` (2 federates, `per-node`) demonstrates goal, feedback and result crossing the boundary end to end with `bringup.launch.py`, and a cancel of a running goal with `cancel.launch.py` (output checked against values derived from the server code in 10 of 10 runs).
- **Decentralized coordination (regime 3)** is exposed but young. STP offset is only wired at the cross-federate input boundary (a logging-only reaction alongside the existing forwarding connection, not a full application-level tardy-recovery handler), fault/availability behavior (the CAL tradeoff regime 3 exists for) is not implemented. A stalled federate still stalls the whole federation, same as centralized.

# Execution Configuration

Execution configuration sets the ROS→LF boundary's
[take policy](../design/time-and-coordination.md#take-policy-at-the-boundary)
(`skip` vs `block`, and the block timeout), per node or per subscription, via a config
file auto-generated from your extracted system model. This page also covers the QoS
hazard diagnostics `colcon lf-analyze` emits and what `colcon lf-generate` does about
them. Read it when you need to tune take policy for a specific subscription, or when
generation refuses because of a hazard.

## Quick reference

Two files live next to each staged `system.json`, under
`lf-analysis/{package}/{launch_stem}/config/`:

| File | Purpose |
|---|---|
| `execution.yaml` | The file you edit. Sparse: set only the keys you want to override. |
| `execution.reference.yaml` | Read-only, regenerated every run: every node/subscription, its effective default, and what the key does. Never hand-edit this one. |

The one command you run after changing `execution.yaml`:

```bash
colcon lf-configure --show   # print the resolved effective table before generating
```

`colcon lf-generate` then reads `execution.yaml` automatically. No extra flag needed.

### The after-delay default

The `after`-delay on generated connections is a generation-time setting rather than
a runtime one. It is a [declared logical-time contract, not a latency
measurement](../design/time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement).
`colcon lf-generate --after-delay` defaults to `0`. With d = 0 an admitted
edge inside one process runs the subscriber's first record of a tag at the publication
tag, and a legacy or cross-federate `after 0` link runs it one microstep
later. Pass an explicit value to change it:

```bash
colcon lf-generate --after-delay "5 msec"
```

`execution.yaml` can also set a default, under `defaults.after_delay`, as an LF time
literal such as `"1 msec"`:

```yaml
defaults:
  after_delay: "1 msec"
```

This key is **defaults-scope only**: it cannot be set per-node or per-subscription,
because the after-delay is resolved at generation time, before per-instance runtime
config exists. For the same reason it never appears in `exec_config.json`, the file
the C++ runtime reads at startup, which carries only the runtime-scoped keys
(`take_mode`, and `maxwait_ms` under its resolved name `take_timeout_ms`). The other generation-time key,
[`auto_transition`](#lifecycle-auto-transition), is node-scoped rather than
defaults-only.

Precedence, highest first:

1. a [`connections:`](#per-connection-after-delays) entry for the sender -> receiver pair
2. a `connections:` entry for the link's name
3. a per-link `after_delay_ns` in `system.json`
4. an explicit `--after-delay` flag
5. `execution.yaml`'s `defaults.after_delay`
6. the built-in `0` default. An admitted edge inside one process then runs
   the receiver at the publication tag, an `after 0` link one microstep later

### Per-connection after-delays

One delay for every link says nothing about any single link. `connections:` gives a
link its own delay and makes it state why. An entry is keyed by the
topic, service or action name; `links:` narrows it to one sender -> receiver pair
(publisher -> subscriber, or client -> server):

```yaml
connections:
  /sentry/image:
    after_delay: "400 usec"
    reason: measured            # measured | budgeted | default
    source: "run <dir>: camera_node timer callback max 390 us"
  /sentry/detections:
    after_delay: "8200 usec"
    reason: measured
    source: "run <dir>: detector_node image callback max 8113 us"
    links:
      "detector_node -> diagnostics_sink_node":
        after_delay: "20 msec"
        reason: budgeted
        source: "diagnostics are logged, not acted on"
```

- `reason: measured` means the value comes from a trace run, and `source` names
  that run and the statistic. `budgeted` means it is a design budget, and `source`
  names the budget. `default` needs no `source`.
- A name, pair, reason or time literal the system does not have is a hard error.
- The value applies wherever the link carries one. A legacy connection gets
  `after <value>`, and an [admitted edge](#correspondence-bounds) gets correspondence
  delay `d = <value>`, which also sizes its sequence window. A service link uses it
  in both directions. An action link uses it on its goal channels; feedback, result
  and cancel keep 1 msec.
- Cross-federate links keep the delay that federation planning gives them.
- The generated `.lf` carries the reason above each configured connection, for
  example
  `// after-delay 400 usec: measured (run ...) [execution.yaml:connections./sentry/image]`.
  The resolved model's `link:<src>-><dst>/<name>#after_delay` record carries the
  value, the reason and the entry it came from. Its provenance is
  `measured_profiled` for a measured value and `explicitly_configured` otherwise.
  The read-back gate therefore checks the configured value
  against the built program.

A user `execution.yaml` entry replaces the package's entry of the same name, and its
pairs merge pair by pair. `packages/src/fixtures/sentry_pipeline/config/execution.yaml` is a
worked example in which every value cites the run it was measured in.

### Timer start offsets and periods

Every timer starts at the LF start tag (offset 0) with the period the analyzer read
from the source, unless `execution.yaml` says otherwise. rclcpp has no offset
argument, so the analyzer never reads one. This differs from stock ROS 2: there a wall
timer fires first one period after it is created, so under rcllf the same timer fires
once more, one period earlier. Set `offset` equal to the period to get the stock first
tick; a later release matches it by default. The phase of a timer is a first-order
latency knob: in the AEB braking loop a controller timer that starts 17 ms later
stops the car 15 cm earlier (probe SQ-A2). `nodes.<node>.timers` sets the start
offset, the period, or both, per timer:

```yaml
nodes:
  /aeb_controller:
    timers:
      control_callback:         # the timer's callback name, as in system.json
        offset: "17 msec"       # first fire at start tag + 17 ms
        period: "25 msec"       # optional, the period the node creates the timer with
```

- The key under `timers:` is the timer's callback name (`TimerInfo.callback` in
  `system.json`, also listed by the error below). Each timer of a node is its own
  entry. Timers that share one callback name share the entry.
- `offset` and `period` are LF time literals (`"17 msec"`, `"500 usec"`, `0`). An
  entry needs at least one of them or `offset_runtime` (below). Any other key is a
  hard error.
- `offset` must not be negative. An offset of the period or longer is allowed: LF
  fires the timer first at the offset, then every period, so it acts as a start
  delay.
- `period` must be greater than 0. It states the period the node's code creates the
  timer with, for a timer whose period the analyzer cannot read (a parameter, for
  example). It does not change the rclcpp timer: at startup the binding finds the
  timer handle by its exact period in nanoseconds, and refuses to start if none fits
  (`... has no unbound timer handle in node '...' that fits it`). To run a timer at
  another period, change the period in the node's code.
- A timer name the node does not have is a hard error that names the node and lists
  its timers, for example
  `no timer named 'tick' on 'n' (known timers: fast, slow)`.
- Timers exist per node, so there is no `defaults:` form. In the two-file model the
  user file merges field by field: a user `offset` keeps the package's `period` of
  the same timer.
- An offset on a timer whose period the analyzer could not read leaves the period
  unresolved. Generation then still refuses with
  [RCLLF-R001](../reference/resolved-model.md) until the entry also sets `period`. An
  acknowledgement does not lift it.

The emitted program shows the values, for example
`timer t_control_callback(17 msec, 25 msec)` in the generated `.lf`. The resolved
model (`resolved_model.json`) carries one `timer:<node>/<callback>#offset` and one
`#period` record per timer. A configured value has provenance
`explicitly_configured` and names its `execution.yaml` entry, the analyzed one is
`extracted`. The runtime binding matches timer handles by period only, so an offset
needs no runtime change.

#### Offsets set at program start

An offset sweep needs one build per offset with `offset`. `offset_runtime: true`
builds the timer once and takes the offset when the program starts:

```yaml
nodes:
  controller:
    timers:
      control_callback:
        offset_runtime: true    # optional: offset "5 msec" becomes the default
```

```bash
RCLLF_TIMER_OFFSETS="controller/control_callback=12 msec" \
  ros2 launch lf_generated_brake_fusion_k1container lf_k1container.launch.py
```

- The timer's offset becomes a parameter of the main reactor (one program) or of the
  federated reactor (federation), named `<node>__t_<callback>_offset` and passed down
  to the timer. reactor-c sets a main-reactor parameter from the program option
  `--<parameter> <n> <unit>` in `main()`, before the start tag, so the run is the
  same as a build with that offset baked in: same tags, same order.
- The generated launch files read `RCLLF_TIMER_OFFSETS`: entries
  `<node>/<timer callback>=<integer> <unit>`, separated by `,` or `;`. Units are
  `nsec`, `usec`, `msec` and `sec` (and `ns`, `us`, `ms`, `s`). In a federation each
  federate takes the options for its own timers.
- A name that is not a run-time timer of the build is refused at launch, with the
  list of the build's run-time timers. A build without any refuses every name. A
  federate would ignore an option it does not know, so the launch file checks.
- Unset, the timer starts at its default: the `offset` of the entry, else 0.
  Without `offset_runtime` the generated `.lf` is unchanged.
- The options come before `RCLLF_LF_ARGS` (see [Fast mode](fast-mode.md)), which
  passes raw reactor-c arguments.

### Lifecycle auto-transition

A managed lifecycle node idiomatically creates its publishers in `on_configure` and
its timers in `on_activate`, so a node still in `unconfigured` owns none of them and
the discovery walk in `ros2_lf_bind_node` would bind nothing. The wrapper therefore
drives such a node to `active` before binding.

Managed lifecycle nodes registered as components take this path too. A lifecycle
node registered with `rclcpp_components_register_nodes` is built as a
non-component node instead, because the component path cannot drive a lifecycle
at all, and generation says so by name when it re-routes one. Nothing needs to
be configured for this.

The bring-up default is the right one when the wrapper owns the node. Set
`auto_transition: false` only for a node whose activation belongs to in-process
C++ that calls `activate()` from a callback the LF runtime dispatches:

```yaml
nodes:
  lifecycle_node:
    auto_transition: false
```

It may also be set under `defaults` to decline the bring-up system-wide. Like
`after_delay` it is generation-time only, so it never appears in `exec_config.json`.
Unlike `after_delay` it is a property of a node rather than of the system, so it may
be set per node as well as in `defaults`. It cannot be set per subscription, because
a node with no subscriptions (a publisher-only lifecycle node is the common case)
would have nowhere to carry it.

The value must be a real YAML boolean. `auto_transition: "false"` is a string and is
rejected rather than read as true.

The wrapper drives only the transitions a node is actually missing, so a node
already configured is activated without being reconfigured, and one already
`active` is left alone.

!!! note "Later transitions take effect at the next tag"
    The runtime never spins a ROS executor, by design. LF is the sole scheduler.
    A binding thread serves only the node's parameter and lifecycle services and
    hands each request to an LF worker, so `ros2 lifecycle set` and
    `ros2 param set` work. The binding sees each completed transition where it
    runs. A transition a reaction makes at tag T holds for
    every reaction at a tag after T, so all reactions at one tag agree on the
    state whatever their physical order, and the same input gives the same tag in
    every run. A `LifecyclePublisher` holds its activation for the tag in the same
    way. The callback that runs the transition sees the change at
    once, as in stock rclcpp, and every other reaction at T publishes as it would
    have before the change. A transition completed outside LF, on a thread of the node's own,
    is external input. It enters LF as a physical action, the node's ingress
    wake, and holds from the tag after that action's tag. `ros2 lifecycle set`
    reaches the node as an ingress request, so it too takes effect at a physical
    action's tag. Such a tag follows the wall clock, so it differs between runs.
    The binding names the node of a transition by its rcl state machine, so a
    remapped `transition_event` topic or two nodes of the same name do not
    mislead it. Two setups fall back to polling: a lifecycle node built without
    the communication interface (`enable_communication_interface` false), and an
    executable without the binding's publication gate (lfc-generated executables
    always link it). The binding logs a warning for such a node at bind. A polled
    transition is read when a callback returns or at the next boundary step and
    carries no timestamp, so it can apply at a logical tag earlier than its
    physical time.
    Transition states such as `deactivating` are ignored, only the four primary
    states count.

    While a node is not `active` none of its callbacks run. A timer reaction is
    skipped. A subscription sample, service request or response and action
    request the reaction takes is dropped and never delivered later, as in a
    stock node that destroyed the entity. Each change re-matches the node's
    timers and subscriptions against its current entities and its services,
    clients and actions by name, so a timer created in `on_activate` binds at its
    first dispatch and one released in `on_deactivate` stops. A node that keeps
    its timer through `on_deactivate` stops too and resumes on the next
    activation. An entity created later replaces only the one it supersedes. A
    reaction keeps a live entity unless a newer one fits it, and an entity it
    gave up for a newer one never comes back, even while another owner keeps it
    alive. A timer the node hands to another owner stays in the node's callback
    group and keeps firing while the node is `active`, as under a stock executor.

    So `auto_transition: false` yields a node that stays idle until an in-process
    supervisor or `ros2 lifecycle set` activates it. Generation warns by name for
    every lifecycle node that opts out, because a node nobody activates looks like
    a broken one (`0 timer(s), 0 subscription(s)` at startup).

!!! warning "v1, static at launch"
    Execution policies are read once, at process startup, from a generated JSON file.
    There is no runtime reconfiguration yet (the interface shape allows for it later
    (see [Limitations](#limitations)), but v1 ships none of it). Policy *is*
    per-node-instance. See
    [The (binding key, topic) resolution reality](#the-binding-key-topic-resolution-reality)
    for how instances are identified.

### Ingress capacity

A binding thread takes each node's parameter, lifecycle and type description service
requests and stores them until an LF worker runs them. `ingress_capacity`
is how many requests one node may hold stored at the same time. The default is 8. A
request that arrives while the node already holds that many is refused at once. The
binding logs a warning that names the node key, the capacity and the request, and a
mutating request (`set_parameters`, `change_state`) gets a failure reply with the
reason `overflow`, so `ros2 param set` fails fast instead of timing out.

```yaml
defaults:
  ingress_capacity: 8
nodes:
  /robot_1/config_node:
    ingress_capacity: 2
```

The value is an int greater than 0. It may be set under `defaults` and per node, but
not per subscription. Like `auto_transition` it is generation-time only. The generator
bakes it into the node's startup reaction (`ros2_lf_ingress_set_capacity`, emitted
only for a value other than 8), so it never appears in `exec_config.json`.

### Reaction order at one tag

Each node becomes an LF reactor, and each timer or subscription callback a reaction in
it. LF runs the reactions of one tag in an order that comes from the reaction dependency
graph: a connection is a precedence edge, so a reaction that reads a port always runs
after the reaction that writes it. Declaration order inside a reactor decides only among
reactions that the graph leaves unordered at that tag (see [Ties at the same logical
tag](../design/architecture.md#ties-at-the-same-logical-tag)). That is the case for a
timer and a subscription of the same node that fire at the same tag, because no
connection runs between them. `reaction_order` sets which of the two groups the
generator declares first.

| Value | Declared order in the node's reactor |
|---|---|
| `timers_first` (default) | timer reactions, then subscription reactions. A timer sees the state before the messages of its own tag. |
| `subscriptions_first` | subscription reactions, then timer reactions. A timer sees the messages of its own tag. |

```yaml
defaults:
  reaction_order: timers_first
nodes:
  /fusion_ties_consumer:
    reaction_order: subscriptions_first
```

The default was `subscriptions_first` in early versions and is now
`timers_first`. A system generated before that change and regenerated after it gets the
other order for every node that does not set the key; set `reaction_order:
subscriptions_first` under `defaults` to keep the old order. Evaluation releases are
not affected, because each one reproduces from the commit its `eval-*` tag names.

The key may be set under `defaults` and per node, not per subscription. A per-node value
beats `defaults`, which beats the default `timers_first`. Any other value is an
error that lists the two choices, and so is a node key that is not in `system.json`. It is
generation-time only, so it never appears in `exec_config.json`. `resolved_model.json`
reads it back per node as `node:<node>#reaction_order`, with the evidence
`execution.yaml:nodes.<node>.reaction_order`, `execution.yaml:defaults.reaction_order` or
`registry-default:reaction_order`.

What the key does not change:

- Source order within each group. Two subscriptions keep the order of their callbacks
  in the C++ source, and so do two timers.
- A timer callback that also sends a service request or an action goal. Its reaction is
  a sender reaction, not a timer reaction, so it stays with the subscriptions in both
  orders.
- Egress flush reactions (declared first), service and action server reactions
  (declared after the callbacks), and the shutdown close of a correspondence edge
  (declared last).
- Callbacks in a Reentrant callback group. Each one gets its own sub-reactor, so their
  order at a tag comes from the instance order in the parent reactor, not from this key.
  Callbacks of one MutuallyExclusive group, the default group included, share one
  sub-reactor and follow the key.
- Reactions linked by a connection, which the dependency graph orders.

Reaction IDs (the `RID_*` macros) follow the declared order, so they change with the key.
The binding reads them from the generated header, so nothing else needs to change.

The key can decide whether a zero-delay feedback loop is admitted. LF
"allows zero-delay feedback loops as long as there is no causality loop" (Edward Lee),
and a subscription reaches its node's outputs at its tag only through the reactions
declared at or after it in its reactor. So a latch declared after the publishing timer
(`timers_first`) breaks a loop, while the same latch under `subscriptions_first` closes a
causality loop, which `colcon lf-generate` refuses with the loop's links named. Give one
link a logical delay d > 0 to break it. No microstep is added implicitly, and there is no
per-link knob for an LF `after 0` delay. The key still fixes the order in which a
node's own reactions run at one tag: with `timers_first` on the brake_fusion simulator
a brake command at tag t is applied from the plant step at t + 1 ms, the next step of
its integration, while with `subscriptions_first` the latch stores it before the step
of the same tag reads it.

`require_admitted: true` (under `defaults` only, generation-time only, default false)
makes any refused topic edge a generation error that names each edge and its reason.
The brake_fusion T2 configs set it, because their determinism claim needs all eleven
edges. With it set, a `min_input_period_ms` that sizes no bound is a generation error
too. The binding checks only the declared periods an admitted edge's
bound was sized from, so any other declaration would read like a check that never
runs. The message names the node, the topic and the reason (no such node, no
subscription on that topic, or no admitted edge sized from that input).

`workers` (under `defaults` only, generation-time only, a positive integer, unset by
default) sets the `workers` target property, the number of LF worker threads in the
generated program. When it is unset, reactor-c picks min(host cores, reaction graph
breadth), so the same program runs a different number of workers on a desk host than on a
CI runner. Too few workers is the risk on a slow host: a single worker runs every
reaction of a tag in turn, and a 1 ms tag whose reactions together take more than 1 ms
of that host's CPU falls behind physical time a little more at every tag until it
exceeds `max_worker_lag_ms`. brake_fusion's T2 one-program build did this with one
worker on a 2-vCPU CI runner and stayed about 5 ms behind with two.

The analyzed launch file can set the same property itself: a root launch file that
declares `DeclareLaunchArgument('rcllf_workers', default_value='<n>')` gives the
generated program `workers: <n>`, so the count ships with the launch file a user runs
and needs no execution config change. A stock `ros2 launch` of that file ignores the
argument. Only the root launch file counts, the default must be an integer above zero,
and an execution config `workers` that names a different count is a generation error.
Either way, a reactor-c `-w N` on the program's command line, or in `RCLLF_LF_ARGS` at
launch, overrides the property for that run.

`control_failure_policy` (under `defaults` only, generation-time only) decides what the
subscriber of a cross-federate edge does when a COMPLETE record is missing. Every
activation of a publishing reaction that published on a cross-federate edge sends one
COMPLETE record per tag on that edge, which lists everything it published there. An activation that published nothing sends no record and takes no number, so
the receiver has nothing to wait for. The records carry a sequence number per edge. A
gap in those numbers is a CONTROL_FAILURE, because the receiver cannot tell what the
missing activation published. A missing last record has no later number to expose it,
so the run's end check compares the receiver's highest number with the count the
sender sent and fails the run on a difference.

| Value | Effect |
|---|---|
| `report_continue` (default) | Log and trace each CONTROL_FAILURE, fail the run reconcile, keep running |
| `stop` | The same report, then request an LF stop at once |

The default keeps running because a missing COMPLETE leaves earlier and later
activations intact, and the reconcile already marks the run as not valid. `stop` is for
a system that must not act on a partial view of its inputs.

Two more cases have no later number to expose them, and both still end as a
CONTROL_FAILURE.

- **A trailing COMPLETE lost after the last one that arrived.** The sender's reconcile
  record lists `completes_sent` and `completes_at_stop` (the messages that land past the
  stop tag, which no receiver sees). The receiver's record lists `complete_high`, the
  last number it saw. The run merge (`rq2b_oracle.merge_reconciles`) requires
  `complete_high = completes_sent - completes_at_stop` on every cross-federate edge.
  A lower value is reported as `control_failure edge E: N trailing activation(s)`.
- **A sender that stalls.** Under centralized coordination the RTI holds the receiver's
  next tag, so the receiver sees logical time stand still instead of a missing record.
  Each subscriber federate with a timer arms a watchdog at its first reaction. It
  reports `CONTROL_FAILURE stall` (log, trace and reconcile) once per episode in which
  the tag does not change for longer than W. W is the longest timer period P of the
  federate plus d + T of every incoming cross-federate edge plus the worker lag bound L.
  A healthy federate advances within P unless it waits on an upstream window (at most
  d + T per edge) or runs behind its tags (at most L). The watchdog stays off without a
  timer (an idle federate may stand still), with an infinite T, or in a fast federate.
  Under `stop` it requests an LF stop, which completes once the RTI can grant the stop
  tag.

On a cross-federate edge the deadline of every record counts from the publication's
logical tag t, D = t + max(0, d) + T + eps with eps the declared clock bound.
All records of one publication share it. An edge inside one federate has no
deadline: it is decided when its publish returns (see
[missing messages](missing-messages.md#edges-inside-one-federate-have-no-cutoff)).

### Correspondence bounds

A topic edge gets publication correspondence (every published sample is matched to
exactly one dispatch, or reported lost) only when its bounds are known at generation
time. The analyzer proves the burst bound k where it can, and five
generation-time keys supply what it cannot prove or size. A sixth, `cutoff_anchor`,
accepts only `tag`, the one cutoff rule. None of them reaches the
runtime `exec_config.json`.

| Key | Scopes | Value | Meaning |
|---|---|---|---|
| `max_publishes_per_invocation` | `defaults`, `nodes` | int > 0 | Burst bound k. A callback of the node publishes at most k samples per invocation on any one topic. Overrides the bound the analyzer proves. |
| `max_worker_lag_ms` | `defaults` | int > 0 | Worker lag bound L. Without it L is twice the largest edge timeout of an admitted edge, where a relay edge adds its upstream timeout (see [missing messages](missing-messages.md)). A value below that timeout is an error. |
| `reader_depth` | all three | int > 0, default 8 | KEEP_LAST depth the binding gives the reader of an admitted edge at run time. An edge whose burst k exceeds it is refused. |
| `join` | all three | `blocking` (default) or `deferred` | How the drain of an admitted edge waits for its sample. `blocking` dispatches at publish tag + d and waits for an absent sample up to its cutoff, holding the rest of the LF program. `deferred` dispatches at the cutoff tag, publish tag + d + T, never blocks the worker waiting, and delivers only a sample taken in strictly before the cutoff. `deferred` on a self edge is a generation error, since its sample is present at the drain. On a cross-federate edge the subscriber's federate defers its join; the publisher's half has none. See [missing messages](missing-messages.md). On an edge inside one federate, decided when its publish returns, `deferred` is a generation error. |
| `cutoff_anchor` | all three | `tag` (default, the only value) | Kept for configs written by earlier versions. The absence cutoff of a cross-federate edge always counts from the publication's logical tag, t + max(0, d) + T + eps (eps the declared clock bound), the maxwait rule of Paladino et al. (Eq. 1b), so the next hop's `maxwait_ms` must cover the upstream timeout. `sender_start`, which counted from the physical start of the publishing activation, was removed and is a generation error. An edge inside one federate has no cutoff at all, it is decided when its publish returns. d is the link's existing `after_delay` (`connections:` or `defaults.after_delay`), there is no separate delay key. A fast federate uses eps = 0. The key generates byte-identical output to a config without it. See [missing messages](missing-messages.md#counting-the-cutoff-from-the-tag). |
| `min_input_period_ms` | all three | finite number from 0.000001 (1 ns) to about 9.2e12 (the int64 nanosecond range) | Shortest gap between two messages on a subscription. Bounds the rate of a callback fed from outside the system. Converted to whole nanoseconds rounded down, so a bound is never computed from a longer gap than declared. Checked at run time when it sizes a bound: two runs of the callback closer than the period count as a violation, the first logs a WARN `ros2_lf_input_period: first violation` with both sample identities, and shutdown logs `ros2_lf_input_period: ... violations=N` per declaration. |

Without a declaration, k comes from the analyzer. It counts the publish call sites of
each publishing callback on the edge's topic, following calls to member functions of
the node class. A site inside a loop or a lambda, a `goto`, a recursive member call or
a publish on a publisher it cannot name leaves k unproven. A subscription's
requested-deadline-missed handler (the deadline hook) runs instead of the callback when
a message is missing, inside the same drain reaction, so its publishes count toward
that callback with the larger of the two counts. The analyzer finds the handler when
the subscription options are a local variable or come from a function whose body it
can see. A publish the analyzer misses (in a free function, another class or a
handler it cannot find) is still caught at run time, as `burst_exceeded` at reconcile
or as an undeclared effect. The manifest lists k per publishing callback with its
source (`declared` or `proven`).

From these the generator sizes each admitted edge. With d the edge's after-delay and
`N(W)` the most publications the edge can see in any window of length W, it sets
`burst = k` (the largest k of the edge's publishing callbacks), `carrier = N(d)` and `retain = N(d + L) + 1`
on an edge inside one federate. On a cross-federate edge `retain = N(d + L + T) + 1`,
because the sender's clock may run up to eps ahead and the binding refuses eps >= T,
so a sample can arrive up to T before its tag. A `deferred` edge releases its record
at the cutoff tag, publish tag + d + T, so its carrier and retention also cover T:
`carrier = N(d + T)` and `retain = N(d + T + L + T) + 1`. A timer with period P runs
`floor(W / P) + 1` times in W, and a subscription callback runs as often as its inputs
deliver. The values and their inputs are listed per edge in the trace manifest under
`bounds`.

At run time the binding sizes each edge's carrier and retention from these values and
logs them on the `admitted edge` line. More than `burst` records in one reaction fails
the reconcile at shutdown with `burst_exceeded` (exit code 3). The edge's
`maxwait_ms` is fixed at generation time, since L and `retain` are derived from it,
so `ROS2_LF_MAXWAIT_MS` does not change the timeout of an admitted edge.

On a cycle that crosses a federation link, order the timeouts. A loss on a `blocking` cross-federate edge x holds its subscriber's
whole federate F until x's cutoff, publication tag t + d_x + T_x. What F publishes at
the release tag of x leaves F only after the stall, at t + T_x or later, while the
cutoff of an admitted edge y that F publishes on counts from the tag: t + d_y + T_y.
When y leads back to x's publisher federate and `T_x > T_y + d_y`, one loss on x makes
y's samples late (`NOT_READY`, `publisher_late`): with a 200 ms brake edge from the
controller into the simulator's federate and 100 ms sensor edges from there back to
the controller's federate, a lost brake command also loses the sensor samples
published behind it. The rule: on an admitted cycle
through a federation link, the T of a blocking edge is at most the smallest T + d of
the admitted edges leaving its subscriber's federate toward the cycle. Generation
warns, naming both edges, when the rule is broken; it does not refuse. A `deferred`
edge never holds its federate and is exempt.

An edge whose bounds are unknown stays on the legacy path, with the reason in
`correspondence.json` and the manifest.

| Reason | Cause |
|---|---|
| `burst_unbounded` | A publishing callback has no proven bound (for example a publish in a loop) and the node declares no `max_publishes_per_invocation`. |
| `unsupported_api` | The node publishes on the topic with a serialized message, which the binding cannot match to a sample. |
| `period_unknown` | A publishing timer has no known period, or its period is below 1 ns or beyond the int64 nanosecond range. Periods are rounded down to whole nanoseconds. |
| `rate_unbounded` | A publishing callback is fed by an edge that is itself refused, by a publisher outside the system, or by a feedback cycle, and its subscription declares no `min_input_period_ms`. |
| `reader_depth_below_burst` | The publisher's burst k exceeds the edge's `reader_depth`, so one invocation alone could overwrite the reader's queue. |
| `carrier_unsized` | The bound came out as zero (defensive, not expected). |

The reader depth is a detection boundary, not a promise of prevention.
k limits the publishes of one invocation, not the number of invocations before the
reader's next ingress take. Choose `reader_depth` at least k times the number of
publishing invocations expected between two ingress takes of the edge. If more samples
arrive than the depth holds, DDS overwrites the oldest, and the correspondence table
classifies each overwritten sample LOST when a later sample of the same writer is
taken. The binding then runs the subscription's deadline callback and its
`message_lost` callback with the logical total, and traces the `terminal` and
`dispatch_end` events with outcome LOST. An overwrite on an admitted edge is never
silent.

Refusing one edge can make edges downstream of it `rate_unbounded`, so refusal repeats
until nothing changes, and the admitted edges are then numbered from 0. For example, a
timer node at 200 ms with one publish call per tick (k 1), d 1 ms and the default
`maxwait_ms` of 100 gives L 200 ms, `N(201 ms) = 2` and `N(1 ms) = 1`, so retain 3,
carrier 1 and burst 1.

The generated table stores burst, retain and carrier as 32-bit unsigned integers and
the times as signed 64-bit nanoseconds. A bound that does not fit stops generation
with an error that names the edge and the field, so lower `max_publishes_per_invocation`
or `max_worker_lag_ms` in that case. The manifest lists every timer period that
contributes to an edge, as a list per callback.

```yaml
nodes:
  corr_publisher:   # publishes in a loop, at most 3 per tick
    max_publishes_per_invocation: 3
```

## The two-file model

Every package's config lives next to its staged model, under the same
`lf-analysis/{package}/{launch_stem}/config/` directory `colcon lf-analyze` already
writes into:

```
lf-analysis/{package}/{launch_stem}/config/
├── execution.reference.yaml   # WRITE-ONLY, regenerated every lf-configure run
└── execution.yaml             # USER-OWNED, sparse, machine-readable
```

**`execution.reference.yaml`** is documentation-as-a-file and lists every node and every
subscription in your system, its effective default, and a one-line explanation of
what the key does, regenerated in full on every `colcon lf-configure` run and never
read back by any tool. It carries the comments. `execution.yaml` itself stays
comment-free.

**`execution.yaml`** is the file you actually edit. It is **sparse**:
presence of a key means "set it", absence means "inherit from a less specific scope".
The first scaffold run writes just a `schema_version` and an empty `defaults:` block
plus a pointer comment to the reference file, nothing else. No tool ever rewrites
this file except `colcon lf-configure --interactive` (which only appends/edits keys
you drive through the prompts). Stale entries (keys that no longer match anything in
the model) are reported as diagnostics on stdout, never silently edited away. Nothing
in the pipeline deletes files under `config/`. `lf-analyze` re-running does not touch
your `execution.yaml`.

`colcon lf-generate` consumes `execution.yaml` automatically when it is present next
to the system it's generating from. Pass `--exec-config <path>` to point at a
different file. If no file is present at all, generation uses today's built-in
defaults, zero migration burden for existing packages. A malformed file (bad YAML,
unknown key, wrong `schema_version`) is a **hard error**, never a silent fall-through
to defaults.

### Layers

A package can also ship its own file, checked in next to its launch files as
`<package>/lf/<launch stem>.execution.yaml`. The stem is the launch file name without
its `.launch.py`, `.launch.xml` or `.launch.yaml` suffix, the same name
`colcon lf-analyze` gives the analysis directory. It exists because some settings,
above all [`external_topics`](#external_topics), belong to the package source and
must reach every workspace that builds it. It uses the same schema and may hold every
top-level key. Four layers apply, lowest first:

```
built-in  <  package lf/<launch stem>.execution.yaml  <  user file  <  env var
```

The user file is `lf-analysis/{package}/{launch_stem}/config/execution.yaml`, or the
`--exec-config` file in its place. An explicit file replaces the analysis directory's
file but still sits over the package file, so it never drops the package's
declarations. Env vars are read by the binding at startup and override only the
runtime keys `take_mode` and `maxwait_ms`.

Each file is validated on its own, so an error names the file it is in. The layers
then merge **value by value**. A value the user file sets wins over the same value in
the package file, and every value the user file leaves unset comes from the package
file. Setting `nodes: {my_node: {maxwait_ms: 400}}` in the user file keeps the
package's `timers:` of that node. `external_topics` and `acknowledged_hazards` are the
union of both layers.

Scope precedence (next section) applies after this merge, so a narrower scope wins
even from a lower layer. A `maxwait_ms` the package sets under `subscriptions:`
beats one the user file sets under `defaults:` or `nodes:` for that subscription. To
override it, set the same subscription key in the user file.
`colcon lf-configure --show` prints the layer and scope of every resolved value.

## Schema (v1)

```yaml
schema_version: 1
defaults:                       # global scope
  take_mode: skip               # skip | block
  maxwait_ms: 100          # int > 0; only meaningful when resolved mode = block
  auto_transition: true         # drive lifecycle nodes to `active` before binding
  max_publishes_per_invocation: 1  # burst bound k (see Correspondence bounds)
  max_worker_lag_ms: 200        # worker lag bound L, defaults scope only
  ingress_capacity: 8           # stored service requests per node (see Ingress capacity)
  reaction_order: timers_first  # or subscriptions_first (see Reaction order at one tag)
nodes:
  /robot_1/subscriber_node:     # node key: byte-identical to system.json nodes key
    maxwait_ms: 250
  /robot_1/lifecycle_node:
    auto_transition: false      # in-process C++ owns this node's activation
  /robot_1/timer_node:
    timers:                     # per timer, keyed by its callback name
      timer_callback:
        offset: "2 msec"        # first fire, >= 0 (see Timer start offsets)
        period: "5 msec"        # > 0, overrides the analyzed period
  /robot_1/acq_sampler:
    tf2:                        # rcllf_tf2::AcquisitionLookup, wins over the literals
      wait_bound: "125 msec"    # > 0 (see tf2 Lookups)
      on_timeout: failure       # reject | failure | fallback:<name>
subscriptions:
  /robot_1/subscriber_node:
    sensor_data:                # topic key: byte-identical to the model (may be relative)
      take_mode: block
acknowledged_hazards: []        # list of RCLLF-H* codes accepted by the team
external_topics: []             # topics fed from outside the system (see below)
connections: {}                 # per-link after-delays with a reason (see above)
publishers: {}                  # per-publisher output_deadline_ns (see below)
```

There are three scopes for policy keys (`defaults`, `nodes`, `subscriptions`), plus
an environment-variable layer above all of them, described next.

### `maxwait_ms` and its alias

`maxwait_ms` is the take timeout, named after the maxwait rule of
[Missing messages](missing-messages.md). Files written by earlier versions spell it
`take_timeout_ms`, and that name stays accepted as an alias at every scope, in every
layer. Both names set one value: a user file's `maxwait_ms` overrides a package file's
`take_timeout_ms` and the reverse, and a narrower scope wins whichever name it uses.
One scope that sets both names to the same value loads with one warning to keep only
`maxwait_ms`; set to different values it is a hard error naming both keys and both
values:

```text
execution.yaml: defaults: 'maxwait_ms' and its alias 'take_timeout_ms' are both set with different values (maxwait_ms: 200, take_timeout_ms: 100); keep only 'maxwait_ms'
```

The alias is additive, so the schema stays at version 1 and every existing file reads
as before. Generated artifacts keep the resolved name `take_timeout_ms`: the
`exec_config.json` table, the `colcon lf-configure --show` listing and the run
records. The environment variable has the same alias, described next.

### Canonical key forms

Node keys and topic keys are **byte-identical to `system.json`**
(`SystemGraph.nodes` keys, and each subscription's `topic` field). You are not meant
to guess these. `colcon lf-configure` generates `execution.reference.yaml` straight
from the model, so you copy keys from there.

Input is matched **by runtime name**, the name rclcpp gives the node or topic when the
program runs. The same rule holds in `nodes:`, `subscriptions:` and
`publishers:`:

- A node key names the node with the same fully qualified name, so `controller` and
  `/controller` are one node, and so are `robot/controller` and `/robot/controller`.
  ROS 2 has no relative node names.
- A topic key names the model topic that resolves to the same name for that node.
  An absolute name (`/scan`) stays as written, a relative name (`scan`) is joined to
  the node's namespace, and `~/scan` to the node's full name. Launch remappings are not
  applied: keys name the topic as the node's code spells it.

For a node in the root namespace, `sensor_data` and `/sensor_data` are therefore one
topic and either spelling works. For a node in `/robot` they are two topics,
`/robot/sensor_data` and `/sensor_data`, and each key names only its own. Whichever
spelling you write is normalized to the model's form during validation.

This is a behaviour change from the earlier slash-tolerant matching. A config written
for a node in `/robot` that used `/scan:` to mean the node's relative `scan` topic
(`/robot/scan`) is now refused as an unknown topic; the error names what `/scan`
resolves to and lists the node's topics with their runtime names. To fix it, write
the topic as the node's code spells it (`scan:`), or copy the key from
`execution.reference.yaml`.

Two keys of one section that resolve to one runtime name, such as `controller:` and
`/controller:` under `nodes:`, or `scan:` and `/scan:` under one root-namespace node
in `subscriptions:`, are refused with an error that names both spellings. The later
entry used to replace the earlier one silently. A key that matches nothing in the
model is a hard `ValueError` that names the key, the name it resolves to and the
node's topics with theirs.

### Precedence, per-key, not per-block

Precedence resolves **one policy key at a time**, not as "pick a whole scope block".
For each key independently:

```
built-in default  <  defaults:  <  nodes:  <  subscriptions:  <  env var
```

Take the schema example above. `/robot_1/subscriber_node`'s `sensor_data`
subscription resolves to:

| Key | Resolved value | From |
|---|---|---|
| `take_mode` | `block` | `subscriptions:` (most specific scope that sets it) |
| `maxwait_ms` | `250` | `nodes:` (subscription scope never sets this key, so the next scope down wins) A subscription whose admitted edges all lie inside its federate gets no cutoff from it; the value then only sizes the worker lag bound L and the retention. |

That's the point of per-key merge. Setting `take_mode` at the subscription scope does
not implicitly pull `maxwait_ms` down with it. Each key falls back through the
scope chain independently.

`PolicySpec.scopes` is enforced by validation. A key set at a scope it isn't valid
for is a `ValueError`. Both `take_mode` and `maxwait_ms` are valid at all three
scopes. `maxwait_ms` bounds the wait in both modes: a `skip` take waits up to it
for a sample its trigger counted, a `block` take also when the count is
unknown, and an admitted edge's join waits up to it as well. The one
subscription whose take never waits is an [`external_topics`](#external_topics) one, and
`colcon lf-generate` warns when `maxwait_ms` is set there. A take whose
write count is unknown at run time (`RCLLF_COUNT_PUBLISHES=0`, a publisher outside any
reaction) skips at once too; the generator cannot see that per edge, so it does not warn.

### Env var overrides

`ROS2_LF_TAKE_MODE` and `ROS2_LF_MAXWAIT_MS` sit **above every scope**, and each
overrides **only its own key**. Setting `ROS2_LF_TAKE_MODE` does not touch any
subscription's resolved `maxwait_ms`, and vice versa.

`ROS2_LF_TAKE_TIMEOUT_MS` is the alias of `ROS2_LF_MAXWAIT_MS` and works the same way;
the startup `WARN` names the alias when it is the variable that applied. An empty
variable counts as unset. Both set to the same value apply that value. Both set to
different values refuse the override, so every subscription keeps its configured
`maxwait_ms`, and the runtime logs one `WARN` naming both variables and both values.
In the Docker Compose services, a `ROS2_LF_MAXWAIT_MS` set on the host also sets the
alias's default, so it never disagrees on its own. `docker compose run -e
ROS2_LF_MAXWAIT_MS=N` reaches the container next to the alias's default of 100,
so pass both variables or set the host variable.

One exception applies to `ROS2_LF_MAXWAIT_MS`. An admitted correspondence edge
keeps the timeout generated into its binding header (see
[Correspondence bounds](#correspondence-bounds) above), and the variable does not change
it. To give an admitted edge a different timeout, for example a very high one for a
run that should lose nothing, set `maxwait_ms` in `execution.yaml` and regenerate.

At startup, if an env var changes any resolved entry, the runtime logs exactly one
`WARN` naming how many entries it affected, e.g.:

```
WARN: ROS2_LF_TAKE_MODE=block overrides take_mode for 12 of 40 configured subscription(s) and for the defaults, which every subscription without its own entry uses
```

"12 of 40" counts per-subscription entries only. The defaults entry is reported
apart, as "and for the defaults" or "not for the defaults, which already match".

Treat the env vars as a **quick-experiment override**, not a substitute for the config
file. They're useful for "try blocking everywhere for this one run" without editing
YAML, but they apply uniformly and can't target a single subscription the way
`execution.yaml` can.

### Other validation rules

- Unknown policy keys, invalid values (wrong type, out of range, not in `choices`),
  and unrecognized `schema_version` values are all hard `ValueError`s, prefixed with
  the file path (the same idiom federation config errors use).
- A block key written without a value (YAML null, such as `timers:` or `tf2:` with
  nothing under it) means the block is absent, the same as leaving the key out. This
  holds for every block: `defaults`, `nodes`, `subscriptions`, `external_topics`,
  `connections`, `publishers`, `acknowledged_hazards`, and a node's `timers` and
  `tf2`. So `tf2:` with no value is accepted even on a node without an
  `AcquisitionLookup`, while an empty mapping `tf2: {}` is refused. A policy key
  with a null value (`maxwait_ms:` with nothing) is refused as a wrong type.
- Absent `execution.yaml` → built-in defaults, silently. Corrupt or unparseable file,
  or a `schema_version` the tool doesn't understand → **loud init failure**, never a
  silent fallback to defaults. A typo in your config never quietly resolves to "as if
  you'd set nothing."

## The (binding key, topic) resolution reality

`execution.yaml` is authored in terms of **node names**. Internally, resolution
flattens node-name scope down to a **binding key** before it reaches the runtime,
the key the generated code looks up.

A binding key identifies one node *instance* across every generated artifact, the LF
[reactor](../design/glossary.md#reactor) definition, its
[reaction](../design/glossary.md#reaction)-ID range, the `ros2_lf_bind_node` call, the
node registry / component registration, and the `node_class` field of
`exec_config.json`.
All of them are derived from one helper in one generator run, so they cannot drift.
The rule is:

| Instances of the C++ class | Binding key |
|---|---|
| exactly one | the plain class name, e.g. `SubscriberNode` |
| two or more | `Class__<node-key>`, e.g. `FeatureNode__primary_node` |

The suffix is the node's model key with leading slashes stripped, non-identifier
characters replaced by `_`, and runs of `_` collapsed, so `/robot_1/timer_node`
becomes `TimerNode__robot_1_timer_node`. In the rare case where two node keys
normalize to the same suffix (`/a/b_c` and `/a_b/c` both give `a_b_c`), a
deterministic `_h<6 hex>` hash of the raw key is appended so the keys stay distinct.
Keys are derived from `system.json` content only, so they are stable across
regenerations for a fixed topology.

Because keys are 1:1 with instances, **per-instance policy just works**: two
`FeatureNode` instances, one `block` and one `skip`, resolve to two independent rows.

!!! note "Adding an instance re-keys that class's generated rows"
    Going from one instance of a class to two flips that class's key from
    `FeatureNode` to `FeatureNode__<node-key>`, which changes its `exec_config.json`
    rows and its `RID_*` macro names. **Your `execution.yaml` is unaffected**: its
    authoring keys are node names, and a node name doesn't change when a sibling
    appears, so the config re-resolves correctly against the new keys. The churn is
    confined to generated build artifacts, which you regenerate anyway.

The resolved table is emitted as `exec_config.json` next to the staged `system.json`,
keyed by `(node_class, topic)`, where the frozen field name `node_class` carries the
**binding key**, equal to the C++ class name only when that class has one instance:

```json
{ "schema_version": 1,
  "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
  "subscriptions": [
    {"node_class": "SubscriberNode", "topic": "sensor_data",
     "take_mode": "block", "take_timeout_ms": 250},
    {"node_class": "FeatureNode__primary_node", "topic": "feature_input",
     "take_mode": "block", "take_timeout_ms": 250},
    {"node_class": "FeatureNode__secondary_node", "topic": "feature_input",
     "take_mode": "skip", "take_timeout_ms": 100} ] }
```

If resolution ever reports two node instances folding to one binding key, that is a
generator bug (key synthesis), not a configuration error, and the error message says
so.

`topic` here is the model form (possibly relative). The runtime matches it
slash-tolerantly against both the declared topic and the live subscription's resolved
topic name. In a federation, every federate receives the **whole** resolved table.
Rows for other federates are simply never looked up, and the runtime logs loudly,
per occurrence, whenever a bound subscription has no matching row (since resolution
emits one row per model subscription, a miss means a real key-form bug, not an
expected gap).

## `colcon lf-configure`

`colcon lf-configure` scaffolds, validates, and inspects execution config. It runs as
part of the full `colcon lf` chain (analyze → configure → generate → build), and can
also be run standalone.

```bash
colcon lf-configure                          # scaffold + validate + stale report, all packages
colcon lf-configure --package pubsub_example # every analysis of one package
colcon lf-configure --system-json lf-analysis/pubsub_example/bringup/system.json  # single analysis
colcon lf-configure --show                   # print the resolved effective table
colcon lf-configure --interactive            # guided prompt loop over defaults/nodes/subscriptions
```

`--package <name>` and `--system-json <path>` are mutually exclusive for scoping a
run. `--package` resolves to every analysis under `lf-analysis/<name>/` (one per
launch file `lf-analyze` processed) and prints the launch stems it matched, so you
don't have to know the `lf-analysis/<pkg>/<launch-stem>/system.json` layout to scope
a run. `lf-generate` accepts the same pair of flags with the same semantics.

Every run, with no extra flags, scaffolds and then validates. Scaffolding
regenerates `execution.reference.yaml` from the current `system.json` and creates
`execution.yaml` if it doesn't exist yet. It is idempotent and never touches an
existing `execution.yaml`. Validation loads your sparse `execution.yaml`, checks it
against the model and the policy registry, and prints any entries that no longer
correspond to anything in the model (a node was renamed, a subscription was removed)
as diagnostics on stdout, your file is never rewritten to clean these up
automatically. You decide whether to remove them.

`--show` prints the fully resolved, post-merge, pre-env-override table, the same
table written to `exec_config.json`, so you can check what a given node/topic will
actually run with before generating. Each value names the [layer](#layers) and scope
it came from, then the `external_topics` and `acknowledged_hazards` entries with their
layer, then any set env var override:

```text
  Resolved effective policy (post-merge, pre-env), each value with its layer:
    defaults:
      take_mode: "skip"  <- user lf-analysis/my_pkg/bringup/config/execution.yaml (defaults)
      take_timeout_ms: 100  <- built-in
    ListenerNode./chatter:
      take_mode: "skip"  <- user lf-analysis/my_pkg/bringup/config/execution.yaml (defaults)
      take_timeout_ms: 300  <- package src/my_pkg/lf/bringup.execution.yaml (subscriptions)
    external_topics:
      - /chatter  <- package src/my_pkg/lf/bringup.execution.yaml
    acknowledged_hazards: []
    env ROS2_LF_TAKE_MODE=block overrides that key for every subscription at runtime
```

`--interactive` runs a guided prompt loop over
`defaults:` → `nodes:` → `subscriptions:`, driven by the policy registry's one-line
`doc` strings as inline help. It reads and writes only the sparse `execution.yaml`
(no comment round-trip to worry about). This mode is more experimental than the
others. Treat it as a convenience for first-time setup, not the primary editing
path.

### `--exec-config` on `lf-generate`

```bash
colcon lf-generate --exec-config path/to/execution.yaml
```

Overrides which user file `lf-generate` reads for this run. Without the flag,
`lf-generate` looks for `execution.yaml` next to the `system.json` it's generating
from. Either way the user file is layered over the package's
`lf/<launch stem>.execution.yaml` (see [Layers](#layers)). If neither file exists, it
falls back to built-in defaults silently.

## The block-timing footgun

`take_mode: block` does not change *[tag](../design/glossary.md#tag) semantics*. It
never fabricates data or re-delivers a message at an earlier tag, and it is not
[tardiness](../design/time-and-coordination.md#regime-3-federated-decentralized) in
the LF sense (there's still no tardy handler, no STP offset, and the message is not
delivered late at a different tag, it simply isn't observed until it physically
arrives or the timeout elapses). A sample that arrives after the timeout is dropped
and reported when it comes, never delivered at a later tag (see
[late samples](../reference/troubleshooting.md#a-subscriber-callback-sees-stale-data-or-none)).
This holds across federates too: the generated link carries the writer's record
events with its write count, so the receiving federate drops a late sample by
tag as one process does; a link that lost events, a program generated with
`RCLLF_COUNT_PUBLISHES=0`, or a topic whose writer publisher count the analysis
cannot prove exact (a dynamic publisher name, a publisher created outside the
constructor, as a lifecycle publisher made at configure, or one the constructor
creates only on some paths, as under an `if` on a parameter), falls back to the
count rule with a WARN naming the edge and the reason. What Block changes is **physical timing**, and that
has a real cost:

!!! danger "A per-subscription block can stall the whole tag"
    Reactions triggered at the same tag execute in a fixed order (see
    [Firing-order determinism](../design/time-and-coordination.md#three-kinds-of-determinism)).
    If a reaction earlier in that order is polling under `take_mode: block`, every
    reaction scheduled after it **at that same tag** waits for it to either receive
    its payload or hit `maxwait_ms`, up to the full timeout, in wall-clock
    milliseconds, not logical time. A single subscription with a generous timeout can
    therefore stall unrelated reactions that have nothing to do with the slow
    payload.

Keep `maxwait_ms` as tight as the transport genuinely needs, and remember that
setting `block` on a subscription is a decision that affects **every other reaction at
that tag**, not just the one you're tuning. `colcon lf-configure` warns at
resolve-time if a subscription's `maxwait_ms` exceeds the connection's declared
`after` delay, since that combination all but guarantees the block will be felt by
downstream reactions.

## `acknowledged_hazards`

`acknowledged_hazards` is a list of hazard codes (below) that your team has reviewed
and decided to accept for this system. It's per-code, lives in `execution.yaml` (so
it's reviewable in a PR diff and survives CI), and is the only way to make
`lf-generate` proceed past an **error**-severity hazard. There is no global
"allow all hazards" flag.

```yaml
acknowledged_hazards:
  - RCLLF-H003
```

Every acknowledgement is logged loudly at generate time, naming the code and the
node/topic it applies to, so accepting a hazard is never silent even once it's in the
config file.

## `external_topics`

`external_topics` lists the topics whose messages come from outside the generated
system, such as a sensor driver, `ros2 topic pub`, or a rosbag. Each entry is a topic
name, slash-tolerant: `odom` and `/odom` name the same topic.

```yaml
external_topics:
  - /chatter
```

Which messages come from outside is a property of the launch file, so a package
declares it in its checked-in [package layer](#layers),
`<package>/lf/<launch stem>.execution.yaml`. A user file or `--exec-config` file can
add topics but cannot remove the package's, because the lists of both layers are
merged. The examples in `packages/src/` ship such files, for instance
`callback_group_example/lf/callback_group_node.execution.yaml`.

A declared topic enters LF time as a physical action. The binding's ingress thread
takes each message the moment it arrives and schedules the subscription's reaction at
that physical time, the standard LF treatment of sensor input. The callback then runs
once per message on an LF worker, in arrival order, like any other reaction. Every
arrival is traced as an `external_arrival` event (see the
[trace schema](../reference/trace-schema.md#external-arrivals-schema-11)).

The declaration is required, not optional. If some node subscribes to a topic that no
node of the system publishes and the topic is not listed, `colcon lf-generate`
refuses, because the subscription's LF input port would stay unconnected and every
message would be dropped without a word. The error names each subscribing node and the
topic, and prints the lines to add:

```text
  Error: node 'callback_group_node' (class CallbackGroupNode) subscribes to '/chatter', which no node in this system publishes. Under rcllf its callback would never be dispatched.
  Refusing to generate instead of dropping their messages silently. If the messages come from outside the system (a driver, 'ros2 topic pub', a rosbag), declare the topic(s) external in /workspace/packages/src/callback_group_example/lf/callback_group_node.execution.yaml:
    external_topics:
      - /chatter
```

Validation errors, all raised when the file is loaded:

- the value is not a list of strings or `topic:` mappings
- a name no subscription of the system uses (`unknown topic`)
- a name some node of the system publishes. Declaring it would bypass the in-system
  connection, so only topics without an in-system publisher can be external.

### Anchoring a topic at `header.stamp`

A plain entry anchors the topic at arrival: each message's tag is the physical time
the ingress thread took it. A sensor stream whose messages carry a `header.stamp`
can instead be anchored at the stamp, so its reaction runs at a tag derived from the
time the sample was taken, not from when it reached the process:

```yaml
external_topics:
  - topic: /scan
    anchor: stamp
    delay_ns: 200000000           # T: late when arrival > header.stamp + T
    ingress_reserve_ns: 50000000  # budget for the take and the insert (default 0)
    grid_ns: 10000000             # tags rounded up to this grid (default 1)
    basis: measured p99 lateness 83 ms on the streaming rig, 2026-11-11, plus one 100 ms period
```

A `tf2_msgs/TFMessage` topic (`/tf`, `/tf_static`) has no top-level header; its
anchor is the common `header.stamp` of its transforms, and a message whose transforms
carry different stamps is refused as `tf_mixed_stamps` (see
[tf2 Lookups](tf2.md#external-tf)).

The three constants are integers in nanoseconds, at most 2^60 (about 36 years) each,
so the binding's 64-bit tag arithmetic cannot overflow; `grid_ns` is at least 1.

The rules, in the order the binding applies them to a message:

1. **Late is decided on arrival, against T alone.** A message that arrives after
   `header.stamp + delay_ns` is refused as `late`, before the scheduler is involved.
   Arriving exactly at `header.stamp + delay_ns` is on time.
2. **The tag is absolute.** An on-time message is inserted at
   `start + ceil((header.stamp - start) + delay_ns + ingress_reserve_ns, grid_ns)`,
   microstep 0, where `start` is the program's start tag. The reserve is what the
   ingress path may spend between the late test and the insert; the grid groups
   stamps of one sample period into one cell.
3. **Admission is decided under the scheduler's lock.** The binding enters reactor-c's
   critical section, checks the tag against the stop tag and the current tag, and
   inserts the event in one step, so the scheduler cannot advance past the tag between
   the check and the insert. A tag at or before the current tag is refused as
   `overrun` (the ingress path took longer than `ingress_reserve_ns`, which the log
   names as the falsified assumption); a tag after the stop tag as `after_stop`. A
   future tag that reactor-c itself will not schedule (the program is stopping) is
   `refused`, not `overrun`. The scheduler functions must be the executable's own
   copies: the binding checks each one's defining object at startup and refuses to run
   if any is missing or comes from another library.
4. **Two on-time messages in one cell collide.** The event already queued for the cell
   is removed and the new one is not inserted, so neither runs and the outcome does
   not depend on which arrived first. A later message into the same cell is refused
   the same way until the scheduler is past the cell.

Every outcome is one `external_arrival` trace event with `stamp_ns`, `tag_ns`,
`insert_ns` and a `disposition` code (see the
[trace schema](../reference/trace-schema.md#stamp-anchored-arrivals-schema-15)), and
the shutdown summary prints the per-topic counts. `delay_ns` and `basis` are required:
there is no default T, because the value is a property of the sensor and the rig it was
measured on, and the basis records where it came from. `colcon lf-configure --show`
prints each constant with the layer it came from. A user layer that gives a `topic:`
mapping replaces the package's anchor for that topic as a unit; a plain string keeps it.

A stamp anchor needs real time: under `--fast` logical time runs ahead of the stamps
and every message would be refused, so `colcon lf-generate --fast` refuses the system
and the binding refuses a launch-time `-f true` with exit code 3. The message type must
carry a `std_msgs/Header` named `header` whose `stamp` is a `builtin_interfaces/Time`
(checked by type name through introspection at startup, so a look-alike header type
from another package is refused), and
the generated physical action must be plain (no min delay, no min spacing); the binding
refuses to run otherwise instead of ignoring the declaration silently. The arrival
anchor is unchanged for every other topic. `stamped_input_example` in the development repository
(not shipped in the release) is the reference, with a test that derives every tag by hand.

The key is a top-level list, not a policy key, because being external is a property of
the topic within this system. Every subscriber of the topic gets the same treatment.

An external callback may also send a service request or an action goal, the same as
a timer callback. Its reaction stays triggered by the arrival and also sets the
request or goal port, so the server's reaction runs at the same tag and the response
or result comes back through the usual client reactions.
`external_send_example` in the development repository (not shipped in the release) shows both, fed with
`ros2 topic pub /ext_request std_msgs/msg/Float64 "{data: 4.0}"` and
`ros2 topic pub /ext_goal std_msgs/msg/Int32 "{data: 5}"`.

A lifecycle node may create its external subscription in `on_configure` and drop it in
`on_cleanup`. The subscription then binds late, like any subscription of a lifecycle
node. After each `change_state` request, and again when the ingress
thread lets go of a dropped subscription, the binding schedules the reaction of every
such RID without a live subscription. The reaction finds the one the transition
created and admits it to the ingress thread, and the messages that arrive from then on
are dispatched at their arrival time. Messages sent while the node holds no
subscription are not received, the same as under stock ROS 2. A message taken from
a subscription that the node destroyed before its callback ran is dropped too, as
stock drops it with the subscription. The binding logs one line per such message and
never hands it to the next subscription's callback. The ingress thread takes at
most one history depth of messages per wake and nothing from a subscription the node
has already dropped.
Its `launch/lifecycle.launch.py` shows it, with
`auto_transition: false` so that `ros2 lifecycle set /lifecycle_listener configure`
drives it.

At startup the binding waits for every subscription to match a publisher before it
anchors LF time zero (up to `ROS2_LF_DISCOVERY_TIMEOUT_MS`, 5000 ms by default). The
subscriptions on declared topics are left out of that wait, so a launch with nothing
feeding them yet starts without paying the timeout.

`take_mode` does not act on an external subscription. Its message is already taken by
the ingress thread when the reaction runs, which is what `skip` does too, so `skip`
(the default) is the only mode that fits. A resolved `take_mode: block` on an external
subscription is refused at generation, from any layer or scope, with the lines that
fix it:

```text
Error: take_mode 'block' on node 'callback_group_node' (binding key CallbackGroupNode) for external topic '/chatter', set by --exec-config /workspace/block.yaml (defaults). An external_topics message enters as an LF physical action that the binding has already taken when the reaction runs, so there is nothing to wait for and 'block' cannot apply. Set it to skip for this subscription:
    subscriptions:
      callback_group_node:
        /chatter:
          take_mode: skip
```

A set `maxwait_ms` on an external subscription is inert for the same reason, and
`colcon lf-generate` and `colcon lf-configure` warn about it:

```text
  Warning: CallbackGroupNode./chatter: 'maxwait_ms' is set for an external topic; an external_topics message is already taken when the reaction runs, so the take never waits and the timeout is inert. A take on an edge inside the system waits up to the timeout for a counted sample in skip and block mode alike.
```

The runtime overrides `ROS2_LF_TAKE_MODE` and `ROS2_LF_MAXWAIT_MS` do not apply
to external subscriptions either. They keep dispatching on arrival, and the startup
`WARN` that counts the overridden entries names how many it left out, e.g.
`ROS2_LF_TAKE_MODE=block overrides take_mode for 0 of 0 configured subscription(s) and
for the defaults, which every subscription without its own entry uses, leaving out 1
external_topics subscription(s), which dispatch on arrival`.

## `publishers`: output deadline

An actuator command that leaves late is often worth knowing about but never worth
dropping. `publishers:` sets a per-publisher bound on how late after its tag a publish
may leave:

```yaml
publishers:
  /actuation_node:              # node key, as in system.json
    /actuation:                 # a topic this node publishes (matched by runtime name)
      output_deadline_ns: 20000000
      basis: >-
        where the number comes from: a measurement, or a placeholder named as such
```

Both keys are required: there is no default bound, and `basis` states where the bound
comes from (house rule on constants). `output_deadline_ns` is an integer from 0 to 2^60.
The node must be in `system.json` and the topic one it publishes; anything else is
refused at generation, as is a bounded topic that no reaction of the node publishes in
the analysis (the check would never run). Keys match by runtime name, as in
[Canonical key forms](#canonical-key-forms): two spellings of one node (`controller`
and `/controller`), or of one topic of a root-namespace node (`actuation` and
`/actuation`), name the same entry, and writing it twice is refused with an error
naming both spellings, never merged or overwritten. For a node in `/robot`, `actuation`
(`/robot/actuation`) and `/actuation` are two topics and take two bounds.

What happens at run time:

- The generator emits one line right before each `ros2_lf_invoke_*` call whose callback
  publishes the topic,
  `ros2_lf_output_deadline_arm(RID_..., "/actuation", 20000000LL);`. It tells the
  binding the bound of that reaction's publishes of that topic.
- The executable's publication gate, which wraps every `rcl_publish`, compares
  `lateness = lf_time_physical() - lf_time_logical()` with the bound immediately before
  the real publish, so the time the callback spent before its `publish()` counts. A
  publish with `lateness > output_deadline_ns` is late.
- A late publish emits the trace event `actuation_late` (schema 1.6, see the
  [trace schema](../reference/trace-schema.md)) and, once the callback has returned, a
  `WARN` line `rcllf actuation_late: RID <n> topic '<topic>' tag (<elapsed ns>, <microstep>)
  published <lateness> ns after its tag, above output_deadline_ns=<bound>; published
  anyway`. The first ten per reaction and topic are logged, the rest only traced
  and counted. Under `RCLLF_TRACE_CALLBACKS=1` every late publish also prints a JSON line
  `{"event":"actuation_late","rid":..,"topic":..,"tag_ns":..,"microstep":..,"tag_time":..,"physical_ns":..,"lateness_ns":..,"bound_ns":..}`.
- **The message is always published, unchanged.** Lateness never changes a published
  value or the set of reactions that ran. rcllf reports actuation lateness, it neither
  prevents nor shortens it.
- The `WARN` (including the one for a topic the binding could not resolve against its
  node), JSON and summary lines go to the console only (rcutils' console handler,
  bypassing rcl's `/rosout` and log-file handlers), whatever the nodes are called: a
  reaction subscribed to `/rosout` runs the same with the bound on, off or always missed.
- At shutdown, one line per bounded (reaction, topic): `rcllf output deadline summary:
  RID <n> topic '<topic>' output_deadline_ns=<bound> checked=<n> late=<n>
  max_lateness_ns=<ns>` A bound is registered the first time its reaction
  runs, so a bounded reaction that never ran in the run prints no summary line.

This is deliberately not an LF `deadline`. reactor-c runs a deadline's handler *instead
of* the reaction body on a miss, which would drop the message and skip the body's other
effects, forking the program's state on a timing event. No `deadline` appears in the
generated `.lf`.

Under `--fast`, logical time runs ahead of physical time, so the lateness is negative
and no publish is ever late. Only publishes from a reaction are checked; a publish from
a thread the callback started has no reaction tag and is not checked.

`actuation_late_example` in the development repository (not shipped in the release) shows it: `launch/bound` (B = 20 ms), `off` (no
entry) and `tiny` (B = 0) generate three systems whose published values and reactions
are identical, while ticks 3 and 7, which busy-wait 30 ms before publishing, are
`actuation_late` in `bound` and every tick is in `tiny`. Its `rosout_watch_node`
subscribes to `/rosout` (declared in `external_topics`) and sees no output-deadline line
in any of the three.

## Hazard diagnostics

`colcon lf-analyze` runs a set of hazard detectors (QoS and topic-name) after
building the system graph and persists the findings (as facts with code, node, topic and message fields and no severity)
into `system.json`. `colcon lf-generate` reads them and enforces severity. It refuses
to generate when an **error**-severity hazard is present and not covered by
`acknowledged_hazards`, and it prints, but never blocks on, **warning** and
**info**-severity hazards.

If `system.json` predates hazard detection (`schema_version` below 3, or missing the
`diagnostics` field entirely), `lf-generate` refuses and tells you to re-run
`colcon lf-analyze`. A missing `diagnostics` field is treated as unknown, not as "no
hazards". An empty list is the only way to say "analyzed, none found."

Each hazard below explains what it is, why it's hazardous specifically **because**
of LF's logical-tag semantics (not just "bad QoS practice" generically), and what to
do about it.

### RCLLF-H000 {: #rcllf-h000 }

**Unknown QoS** (info)

**What it is:** a subscription whose QoS profile the extractor could not parse into
a concrete reliability/depth/durability value (materialization returned `None`).

**Why it's hazardous under LF:** none of the other hazard checks (H001, H002) can run
against a subscription the extractor can't read. An unknown QoS profile is a blind
spot in hazard detection itself, not a specific runtime risk. It's surfaced at info
severity so you know detection coverage is incomplete for that subscription, not
because the QoS itself is necessarily unsafe.

**What to do:** if the profile is a named QoS preset the extractor doesn't yet
recognize, or a pattern outside what static analysis can parse (e.g. constructed from
a runtime variable), either simplify the construction so it's statically
resolvable, or treat the subscription as unaudited and verify its QoS manually.

**At startup:** the binding knows the effective QoS of every subscription when it
binds the subscription to its reaction. A subscription whose reliability the analysis
did not see logs one info line with the effective value, such as
`rcllf corr: RCLLF-H000 subscription RID 5 (DetectorNode::image_callback) on
'/sentry/image': the analysis saw no reliability, the subscription runs best effort`.
H001 and H002 then decide on that effective value, so an unknown reliability or depth
that turns out best effort or depth 1 is still reported. A publisher whose
reliability the analysis did not see logs the same info line at startup, once per
publisher handle of its node on its topic, such as `rcllf corr:
RCLLF-H000 publisher of node '/unknown_qos_publisher' on '/loose_topic': the analysis
saw no reliability, the publisher runs best effort`. Twin publishers of one node on one
topic cannot be told apart. When the analysis saw the reliability of some of them and
not of others, every handle of the node on that topic gets its own line, and each line
says the analysis saw no reliability on U of the N publishers of the node on this
topic, so a line can name a publisher whose reliability the analysis did see. A
publisher that does not exist yet at startup gets a line saying its effective
reliability is not checked. The publishers a node creates for itself are never taken
for an analyzed one. They are told apart by topic and message type, namely `/rosout`
(`rcl_interfaces/msg/Log`), `/parameter_events` (`rcl_interfaces/msg/ParameterEvent`),
the `transition_event` topic of a lifecycle node (`lifecycle_msgs/msg/TransitionEvent`),
and the `_action/feedback` (a `_FeedbackMessage` type) and `_action/status`
(`action_msgs/msg/GoalStatusArray`) topics of an action server. Two kinds of publisher
are not checked at startup and keep only the static RCLLF-H000. One is a publisher
whose topic is computed at run time, which the analysis gives no topic row. The other
is a user publisher on one of those topics with that message type, which cannot be told
apart from the node's own.

### RCLLF-H001 {: #rcllf-h001 }

**Best-effort input** (a warning for now, promotable to error once extraction is
trustworthy enough to catch every case)

**What it is:** a subscription whose materialized reliability is `best_effort`
(explicitly, or via a named profile like `SensorDataQoS`).

**Why it's hazardous under LF:** a `best_effort` subscriber can silently drop a
message with no delivery guarantee and no retransmission. Under LF's
[take policy](../design/time-and-coordination.md#take-policy-at-the-boundary), a
dropped message isn't a *late* delivery at a later tag (that would be tardiness, and
rcllf's [supported regimes](../design/time-and-coordination.md#the-three-coordination-regimes)
don't have tardiness). It's a payload that never arrives at all, so the reaction
at that tag waits up to `maxwait_ms` for a counted sample and then skips, in
`skip` (default) and `block` mode alike.
Combined with a `skip` take policy, a best-effort input can produce a
**silent logical-clock stall**: the reaction fires, observes nothing, and moves on,
with no error and no distinguishing trace unless you're counting skipped takes.

**On a correspondence edge:** generation repeats H001 once it knows each edge's
status. When every edge into the subscription is admitted, the warning
becomes an info note. A drop is then no longer silent, since the binding times the
sample out and delivers a missing outcome at its tag through the deadline hook. A
subscription with a legacy edge keeps the warning.

The binding also creates the reader of a **cross-federate** admitted edge **best
effort** at run time, whatever reliability the user asked for. Loss on
such an edge is detected by the record that travels over the federation link, not by
DDS retransmission, and a RELIABLE reader is harmful there: after one unrepairable loss
Fast DDS withholds every later sample until the writer's history drops the lost one,
and under centralized coordination one lost message then becomes one lost call per
slot of the writer's history (10 with the default KeepLast(10)), with the second
federate stalled behind the first. The reader of a same-federate admitted edge keeps
the user's reliability: a loss there is one lost call, with no federation link to stall
behind. Startup logs the effective reliability per admitted edge (`reader best effort,
user reliable` on a relaxed edge, `reader reliable` otherwise); a SYSTEM_DEFAULT
request is logged by name (`user system default`) and judged best effort, only an
explicit RELIABLE counts as reliable. A cross-federate reader the binding could not
relax, one created outside the node's construction scope and still RELIABLE, is refused
at startup (`reader_reliability`, exit 1), as the depth refusal does.
`RCLLF_ADMITTED_READER_RELIABLE=1` keeps the user's reliability on admitted readers
(`reader reliable`) and lifts that refusal, for comparison runs only.

**At startup, on the effective QoS:** the analysis sees only the QoS the source spells
out. A scenario overlay or a ROS parameter can make a reader best effort at run time.
The binding therefore checks the actual QoS of every subscription before tag 0. The
reader of an admitted edge gets one warning that names the edge, judged
on the reliability the user declared, since on a cross-federate edge the binding runs
that reader best effort itself. Every
other subscription is checked by its own reaction when the binding binds it,
whether it reads a legacy edge or no edge at all, such as an input from outside
the analyzed graph. Twin subscriptions on one topic are judged apart, each against its
own analyzed reliability. In a federated run the reader's federate checks the reader of
a legacy edge from another federate and names that publisher. A best-effort
subscription whose analyzed reliability is not `best_effort` logs one warning such as
`rcllf corr: RCLLF-H001 subscription RID 10 (ActuatorSinkNode::cmd_callback) on
'/sentry/cmd' runs best effort, the analysis saw no reliability. It reads the legacy
edge from ControllerNode, which never reports a sample DDS drops`. The run still
starts. A subscription the analysis already saw as best effort is not reported again.

**What to do:** if the topic can tolerate occasional loss, this may be intentional.
Review it and add `RCLLF-H001` to `acknowledged_hazards` if so. If it can't, change
the publisher/subscriber QoS to `reliable`, or set `take_mode: block` with a timeout
generous enough for retransmission-driven delivery (mind the
[block-timing footgun](#the-block-timing-footgun) above before doing this on a shared
tag). v1 warns rather than refuses because the extractor doesn't yet reliably catch
every best-effort profile (e.g. some named presets). Refusing on only the subset it
happens to catch, while staying silent on the rest, would be worse than a warning
that's honest about partial coverage.

### RCLLF-H002 {: #rcllf-h002 }

**Shallow queue** (warning)

**What it is:** a subscription with queue depth 1 whose topic has an in-model
publisher (i.e., a publisher this analysis can actually see, joined by the same
slash-tolerant topic-name comparison used throughout the pipeline).

**Why it's hazardous under LF:** this is the
[depth-1 overwrite hazard](../design/time-and-coordination.md#cross-tag-coupling-why-the-honest-claim-is-input-timeline-determinism)
described in the Time & Coordination Model chapter, a depth-1 subscription that
would have been read at tag `T` can be silently overwritten by a newer message
before tag `T+1`'s reaction ever reads it. This doesn't break firing-order
determinism, but it does mean the *values* a reaction observes depend on exactly when
messages physically arrived relative to tag boundaries, the input-timeline
sensitivity that chapter calls out as the "most common misreading of what LF buys
you."

**What to do:** increase queue depth if you need to guarantee older, unconsumed
messages survive to the next tag, or confirm (and acknowledge) that always reading
"whatever is most recent" is the intended semantics for this subscription.

On an edge admitted to publication correspondence the binding raises the reader to the
edge's `reader_depth` (default 8) at runtime. An overwrite beyond that
depth is detected and reported missing, see [Correspondence bounds](#correspondence-bounds). H002
reports the user's declared depth and is still emitted for those edges. On a
cross-federate edge the same reader runs best effort (see
[RCLLF-H001](#rcllf-h001)).

**At startup, on the effective QoS:** like H001, H002 is checked again before tag 0 on
the reader of every admitted and legacy edge, in a federated run by the
reader's federate. A KEEP_LAST reader whose
depth is 1 at run time, while the analysis saw another depth or none, logs one
warning. On a legacy edge it names the depth DDS gave the reader. On an admitted edge
it names the depth the user asked for and the depth rcllf keeps after raising it, for
example `RCLLF-H002 admitted edge 5 PlannerNode -> ControllerNode on '/sentry/plan':
the reader of node '/controller_node' asks for depth 1, the analysis saw no depth.
rcllf keeps 8 sample(s) in the reader`. H002 never refuses the run.

### RCLLF-H003 {: #rcllf-h003 }

**`message_filters` outside the admitted form** (error, refuses generation)

**What it is:** `colcon lf-analyze` scans every node that includes
`message_filters` (comments stripped, header and source together). One form is
admitted and needs no acknowledgement. That form is a single
`message_filters::Synchronizer` with an `ExactTime` or `ApproximateTime` policy
over exactly two plain `message_filters::Subscriber<M>` members, each subscribed
once with `subscribe(this, "topic"[, qos[, options]])` or the matching constructor,
and one `registerCallback` that names a member function. The optional
`rclcpp::SubscriptionOptions` lets the node register its event callbacks (for
example `message_lost_callback`) on a synchronizer input. The same shape
with the rcllf `BoundedNearestToAnchor` policy and literal parameters is admitted
too, with `stamp_provenance` declared on both inputs (and optionally `max_stamp_lag`
on an `lf_clock` input) and its inputs converted to
their correspondence drains (see [BNA synchronizer](bna-synchronizer.md)). It
raises `RCLLF-H010` (info) instead. For the other two policies the analyzer records each input
as an ordinary subscription of the node (callback `<sync_callback>_via_<member>`),
so LF dispatches it and the synchronizer runs the user callback inside that
reaction. Everything else (other policies, more inputs, `Cache`, chained or
custom filters, computed topics, a namespace alias) raises H003 with the exact
reasons in the diagnostic message.

Generation also checks the inputs of an `ExactTime` or
`ApproximateTime` synchronizer. An input that is not an admitted correspondence edge
raises H003 `sync_input_legacy:<reason>` (`external` for a publisher outside the
analyzed system), and inputs with different link delays raise H003
`sync_input_delay_unequal`. Such an input arrives in wall-clock order, so the policy's
tuples would depend on scheduling. See [message_filters Synchronizers](message-filters.md).

**Why it's hazardous under LF:** the analyzer cannot see the rclcpp
subscriptions such a node creates through the library, so they get no reaction
ID and their callbacks would never run. The binding refuses to
start a node that owns a subscription without a reaction ID, so an
unrecognised input fails loudly at bind time rather than losing its messages.

**What to do:** rewrite the node to the admitted form, or redesign the
synchronization in LF terms (explicit `after` delays and reactions gated on
multiple inputs). If you've reviewed the usage and it is not hazardous (for
example dead code behind the include), add `RCLLF-H003` to
`acknowledged_hazards` to let generation proceed. The acknowledgement is logged
loudly, you are the one certifying the review, and a subscription the analyzer
did not see still stops the node at bind time.

### RCLLF-H004 {: #rcllf-h004 }

**Dynamically computed topic name** (warning)

**What it is:** a publisher or subscription whose topic name is built at runtime
(string concatenation, a member or parameter reference, or any computed expression),
so static analysis could not resolve it to a single literal, and neither the C++
`declare_parameter` default nor a launch/YAML parameter overlay recovered it later
in the pipeline. Detection is backed by a structured `topic_dynamic` fact the clang
parser persists into `system.json`. Before this check existed, a partially
resolvable name like `"prefix_" + std::to_string(id)` was silently recorded as
`"prefix_"` with no warning at all.

**Why it's hazardous under LF:** the LF model can only wire connections whose topic
names are known at analysis time. A runtime-computed name means the endpoint appears
in the model **without an edge**, the extracted graph is claiming a completeness it
doesn't have, and everything downstream of the missing edge (generated connections,
the [soundness checker](soundness.md), evaluation metrics) inherits the gap
silently.

**What to do:** dynamic topic names are legitimate ROS 2 style (namespaced
multi-instance nodes, parameterized pipelines), which is why this warns rather than
refuses. If the edge must appear in the generated model, hoist the name into a
string literal or a `constexpr`/`const char[]` constant. If the dynamic name is
intentional, review the endpoint and add `RCLLF-H004` to `acknowledged_hazards` to
accept the deliberately-missing edge.

### RCLLF-H005 {: #rcllf-h005 }

**Lifecycle hook override skips the base call** (warning)

**What it is:** a managed lifecycle node that overrides `on_activate` or
`on_deactivate` without calling
`rclcpp_lifecycle::LifecycleNode::on_activate(state)`. The base implementation is
what activates managed publishers, via `impl_->on_activate()`. An override that
returns `SUCCESS` without chaining leaves every `LifecyclePublisher` deactivated
for the life of the process.

**Why it's hazardous under LF:** every `publish()` is silently dropped, and the
extracted model does not know it. The publisher is still recorded, the generator
still wires its LF connection, and the [soundness checker](soundness.md) still
expects traffic on it, so the model claims an edge that carries no data. This is
the same class of lie as `RCLLF-H004`, one layer down: there the edge is missing
from the model, here the edge is in the model but dead at runtime.

**Why it needs a check at all:** the runtime failure is close to invisible.
`LifecyclePublisher` warns once and then latches its logging off, so a node
dropping 100% of its messages emits a single line at its first publish and then
looks healthy. The idiomatic node also logs `"Publishing: ..."` immediately
*before* calling `publish()`, so the log actively suggests success. This shipped
in this project's own `lifecycle_example` and went unnoticed until a byte-level
A/B on a second example surfaced it.

**What to do:** chain to the base. `return
rclcpp_lifecycle::LifecycleNode::on_activate(state);`, or capture its result
first if the override has more work to do:

```cpp
const auto parent = rclcpp_lifecycle::LifecycleNode::on_activate(state);
if (parent != CallbackReturn::SUCCESS) {
  return parent;
}
// ... your activation work ...
return CallbackReturn::SUCCESS;
```

This warns rather than refuses because the detection deliberately fails open and
a node that manages publisher activation by hand is a legitimate, if rare, shape.
If yours is one, add `RCLLF-H005` to `acknowledged_hazards`. Only `on_activate`
and `on_deactivate` are checked, since the other hooks are no-ops on
`LifecycleNodeInterface` and skipping them costs nothing. A lifecycle node that
owns no publisher never triggers this.

## Limitations

- **No runtime reconfiguration.** Policy is read once at startup. There is no service
  or mechanism to change it while the process is running.
- **Binding keys shift when a class's instance count crosses 1 → 2.** Per-instance
  policy is supported, but the generated key for a class changes form when you add a
  second instance of it. See
  [The (binding key, topic) resolution reality](#the-binding-key-topic-resolution-reality)
  above. Your `execution.yaml` survives the flip. The generated `exec_config.json`
  rows and `RID_*` macros are re-keyed.
- **Services and actions are out of scope.** They keep today's silent-skip behavior.
  The schema reserves a `services:` scope for a later phase, but v1 does not populate
  or honor it.
- **No semantic QoS handling.** These diagnostics detect and warn/refuse. They do not
  implement timeout-based best-effort progress, tardy handlers, or deadline handling.
  That is future work.
- **External subscriptions ignore `take_mode`.** The ingress thread has already taken
  the message when the reaction runs, so there is nothing to block on. A resolved
  `block` is refused at generation, and the env overrides leave them out.
- **A transition from a thread LF does not own is observed late.** The binding
  reads lifecycle states when a user callback it dispatched returns. A transition
  run from a thread of the user's own is seen at the next such return, so its tag
  depends on timing. Transitions from callbacks, in-process supervisors and
  `ros2 lifecycle set` are observed at their own tag.
- **Deactivation does not cancel action goals.** Goals an action client of the
  node sent while active are not cancelled. Their feedback and results are
  dropped while the node is not active. A service response dropped while the node
  is not active leaves its request pending in the client.
- **A `transient_local` sample published in a constructor is not delivered.** Under
  stock ROS 2 a subscriber that joins later receives it; under rcllf the in-system
  subscribers never see it. Publish the first value from a timer
  or callback instead; a later release replays it at the start tag.
- **No `message_filters` conversion.** Two-input ExactTime and ApproximateTime
  synchronizers run unmodified. H003 refuses every other form and offers no
  automatic rewrite to LF-native constructs.

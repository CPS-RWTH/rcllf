# execution.yaml Reference

This page lists every key of `execution.yaml`, the file that sets how a generated
system runs: its type, scope, default, allowed values and an example, plus the file
locations, the precedence and the validation rules. For a walk-through that edits the
file and shows the effect, see [Configure Execution](../guides/configure-execution.md).

## The two-file model

`colcon lf-configure` writes two files next to each analyzed `system.json`:

```
lf-analysis/{package}/{launch_stem}/config/
├── execution.reference.yaml   # regenerated every run, never read back
└── execution.yaml             # yours, sparse
```

`execution.reference.yaml` lists every node and every subscription of the system, the
value each key has for it, and one line per key on what it does. It is rewritten in
full on every `colcon lf-configure` run and no tool reads it. Copy node and topic keys
from it.

`execution.yaml` is the file you edit. It is sparse: a key that is present is set, a
key that is absent is inherited from a wider scope. The first run writes a
`schema_version`, empty blocks and a comment that points to the reference file. No tool
rewrites it except `colcon lf-configure --interactive`. Entries that no longer match
the model (a renamed node, a removed subscription) are reported on stdout and never
removed for you. Re-running `colcon lf-analyze` does not touch it.

`colcon lf-generate` reads `execution.yaml` when it is present next to the system it
generates. Without any file the built-in defaults apply. A malformed file (bad YAML,
an unknown key, a wrong `schema_version`) is an error, never a silent fall back to the
defaults.

### Layers

A package can ship its own file in its source tree, as
`<package>/lf/<launch stem>.execution.yaml`. The stem is the launch file name without
`.launch.py`, `.launch.xml` or `.launch.yaml`, the same name `colcon lf-analyze` gives
the analysis directory. Settings that belong to the package source, above all
[`external_topics`](#external_topics), go there so that every workspace that builds the
package gets them. It uses the same schema. Four layers apply, lowest first:

```
built-in  <  package lf/<launch stem>.execution.yaml  <  user file  <  env var
```

The user file is `lf-analysis/{package}/{launch_stem}/config/execution.yaml`, or the
file given with `--exec-config`. An `--exec-config` file replaces the analysis
directory's file but still sits over the package file. The env vars are read at
startup and change only `take_mode` and `maxwait_ms` (see
[Env var overrides](#env-var-overrides)).

Each file is validated on its own, so an error names the file it is in. The layers
then merge value by value. A value the user file sets wins over the same value in the
package file. Every value the user file leaves unset comes from the package file, so
`nodes: {my_node: {maxwait_ms: 400}}` in the user file keeps the package's `timers:`
for that node. `external_topics` and `acknowledged_hazards` are the union of both
layers.

[Scope precedence](#precedence-per-key-not-per-block) applies after this merge, so a
narrower scope wins even from a lower layer. A `maxwait_ms` the package sets under
`subscriptions:` beats one the user file sets under `defaults:` or `nodes:` for that
subscription. To override it, set the same subscription key in the user file.
`colcon lf-configure --show` prints the layer and scope of every resolved value.

## Schema (v1)

```yaml
schema_version: 1
defaults:                         # every node and subscription
  take_mode: skip                 # skip | block
  maxwait_ms: 100                 # int > 0, how long a take waits (skip and block)
  after_delay: "1 msec"           # LF time literal, defaults only
  auto_transition: true           # drive lifecycle nodes to active before binding
  workers: 2                      # LF worker threads, defaults only
  reaction_order: timers_first    # or subscriptions_first
  ingress_capacity: 8             # stored service requests per node
  max_publishes_per_invocation: 1 # burst bound k
  max_worker_lag_ms: 200          # worker lag bound L, defaults only
  reader_depth: 8                 # reader depth of an admitted edge
  join: blocking                  # or deferred
  require_admitted: false         # defaults only
  control_failure_policy: report_continue  # or stop, defaults only
nodes:
  /robot_1/subscriber_node:       # node key, as in system.json
    maxwait_ms: 250
  /robot_1/lifecycle_node:
    auto_transition: false
  /robot_1/timer_node:
    timers:                       # per timer, keyed by its callback name
      timer_callback:
        offset: "2 msec"          # first fire, >= 0
        period: "5 msec"          # > 0, the period the code creates the timer with
  /robot_1/acq_sampler:
    tf2:                          # see tf2 Lookups
      wait_bound: "125 msec"
      on_timeout: failure         # reject | failure | fallback:<name>
subscriptions:
  /robot_1/subscriber_node:
    sensor_data:                  # topic key, as in system.json
      take_mode: block
      min_input_period_ms: 10
acknowledged_hazards: []          # hazard codes the team accepts
external_topics: []               # topics fed from outside the system
connections: {}                   # per-link after-delays with a reason
publishers: {}                    # per-publisher output deadline
```

There are three scopes for policy keys: `defaults`, `nodes` and `subscriptions`. The
env vars sit above all three. The `tf2:` block is explained in
[tf2 Lookups](tf2.md#the-supported-form).

## Policy keys

"Generation" keys are baked into the generated program, so a change needs a
regeneration. "Runtime" keys are written to `exec_config.json` and read at startup.

| Key | Scopes | Type and values | Default | Applied at | Effect |
|---|---|---|---|---|---|
| `take_mode` | all | `skip` or `block` | `skip` | runtime | Whether a take waits for a message whose write count is unknown. See [below](#take_mode-and-maxwait_ms). |
| `maxwait_ms` | all | int > 0 | `100` | runtime | How long a take waits for its message. Alias `take_timeout_ms`. |
| `after_delay` | defaults | LF time literal | unset (0) | generation | Default logical delay on connections. See [below](#the-after-delay-default). |
| `auto_transition` | defaults, nodes | YAML bool | `true` | generation | Drive lifecycle nodes to `active` before binding. |
| `workers` | defaults | int > 0 | unset | generation | Number of LF worker threads. See [Workers](#workers). |
| `reaction_order` | defaults, nodes | `timers_first` or `subscriptions_first` | `timers_first` | generation | Which callbacks of a node run first at one tag. |
| `ingress_capacity` | defaults, nodes | int > 0 | `8` | generation | Service requests one node may hold stored. |
| `max_publishes_per_invocation` | defaults, nodes | int > 0 | unset (analyzer bound) | generation | Burst bound k of publication correspondence. |
| `max_worker_lag_ms` | defaults | int > 0 | unset (2 × the largest admitted timeout) | generation | Worker lag bound L. |
| `reader_depth` | all | int > 0 | `8` | generation | KEEP_LAST depth of the reader of an admitted edge. |
| `min_input_period_ms` | all | number > 0 | unset | generation | Shortest gap between two messages on a subscription. |
| `join` | all | `blocking` or `deferred` | `blocking` | generation | How an admitted edge waits for its sample. |
| `cutoff_anchor` | all | `tag` | `tag` | generation | Kept for old files. `tag` is the only value. |
| `require_admitted` | defaults | YAML bool | `false` | generation | Refuse generation when any topic edge is not admitted. |
| `control_failure_policy` | defaults | `report_continue` or `stop` | `report_continue` | generation | What a federate does when a COMPLETE record is missing. |
| `stamp_provenance` | subscriptions | `lf_clock` or `declared:<text>` | unset | generation | Where the stamps of a synchronizer input come from. See [BNA Synchronizer](bna-synchronizer.md). |
| `max_stamp_lag` | subscriptions | LF time literal | unset | generation | Largest gap between the publish tag and the header stamp of a synchronizer input. See [BNA Synchronizer](bna-synchronizer.md). |

The other top-level keys are not policy keys: `timers` and `tf2` under a node,
[`acknowledged_hazards`](#acknowledged_hazards), [`external_topics`](#external_topics),
[`connections`](#per-connection-after-delays) and
[`publishers`](#publishers-output-deadline).

### `take_mode` and `maxwait_ms`

A subscription reaction takes its message from the reader when it runs. If the
message has not arrived yet, `maxwait_ms` bounds how long the take waits for it:

- In both modes a take waits up to `maxwait_ms` for a message its trigger counted.
- When the write count is unknown, `skip` gives up at once and `block` still waits up
  to `maxwait_ms`.
- The join of an [admitted edge](#correspondence-bounds) also waits up to
  `maxwait_ms`.

A message that has not arrived by then is reported missing. What the node sees then
is explained in [Missing Messages](missing-messages.md). The take of an
[`external_topics`](#external_topics) subscription never waits, so `maxwait_ms` is
inert there and `colcon lf-generate` warns when it is set. A take whose write count is
unknown at run time (a program generated with
[`RCLLF_COUNT_PUBLISHES=0`](../reference/environment-variables.md#rcllf_count_publishes),
a publisher outside any reaction) gives up at once in `skip` mode too. The generator
cannot see that per edge, so it does not warn.

```yaml
subscriptions:
  subscriber_node:
    sensor_data:
      take_mode: block
      maxwait_ms: 50
```

### `maxwait_ms` and its alias

`maxwait_ms` is named after the maxwait rule of
[Missing Messages](missing-messages.md#counting-the-cutoff-from-the-tag). Older files
spell it `take_timeout_ms`, and that name stays accepted at every scope and in every
layer. Both names set one value: a user file's `maxwait_ms` overrides a package
file's `take_timeout_ms` and the reverse. One scope that sets both to the same value
loads with a warning to keep only `maxwait_ms`. Set to different values it is an
error:

```text
execution.yaml: defaults: 'maxwait_ms' and its alias 'take_timeout_ms' are both set with different values (maxwait_ms: 200, take_timeout_ms: 100); keep only 'maxwait_ms'
```

Generated files use the name `take_timeout_ms`: the `exec_config.json` table, the
`colcon lf-configure --show` listing and the run records.

### The after-delay default

The `after` delay on generated connections is a
[declared logical-time contract, not a latency measurement](../design/time-and-coordination.md#why-after-delays-are-a-let-style-contract-not-a-latency-measurement).
`colcon lf-generate --after-delay` defaults to `0`. With d = 0 an admitted edge inside
one process runs the subscriber at the publication tag, and an `after 0` link (a
cross-federate one, or one that is not admitted) runs it one microstep later.

```bash
colcon lf-generate --after-delay "5 msec"
```

```yaml
defaults:
  after_delay: "1 msec"
```

`after_delay` is a `defaults` key only, because the delay is resolved at generation,
before any per-instance configuration exists. It never appears in `exec_config.json`.
Precedence, highest first:

1. a [`connections:`](#per-connection-after-delays) entry for the sender -> receiver pair
2. a `connections:` entry for the link's name
3. a per-link `after_delay_ns` in `system.json`
4. an explicit `--after-delay` flag
5. `defaults.after_delay`
6. the built-in `0`

### Per-connection after-delays

`connections:` gives one link its own delay and states why. An entry is keyed by the
topic, service or action name. `links:` narrows it to one sender -> receiver pair
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

- `reason: measured` means the value comes from a trace run, and `source` names the
  run and the statistic. `budgeted` means a design budget, and `source` names it.
  `default` needs no `source`.
- A name, pair, reason or time literal the system does not have is an error.
- A connection that is not admitted gets `after <value>`. An
  [admitted edge](#correspondence-bounds) gets the correspondence delay
  `d = <value>`, which also sizes its sequence window. A service link uses the value
  in both directions. An action link uses it on its goal channels. Feedback, result
  and cancel keep 1 msec.
- Cross-federate links keep the delay that federation planning gives them.
- The generated `.lf` carries the reason above each configured connection, for example
  `// after-delay 400 usec: measured (run ...) [execution.yaml:connections./sentry/image]`.
  The resolved model's `link:<src>-><dst>/<name>#after_delay` record carries the value,
  the reason and the entry, so the read-back check compares the configured value with
  the built program.

A user file entry replaces the package's entry of the same name, and its pairs merge
pair by pair.

### Timer start offsets and periods

Every timer starts at the LF start tag (offset 0) with the period the analyzer read
from the source. A stock ROS 2 wall timer fires first one period after it is created,
so under rcllf the same timer fires once more, one period earlier. Set `offset` equal
to the period to get the stock first tick. `nodes.<node>.timers` sets the start
offset, the period, or both, per timer:

```yaml
nodes:
  timer_node:
    timers:
      timer_callback:           # the timer's callback name
        offset: "1500 msec"     # first fire at start tag + 1.5 s
```

- The key under `timers:` is the timer's callback name, as listed in
  `execution.reference.yaml` and in the error below. Timers that share one callback
  name share the entry.
- `offset` and `period` are LF time literals (`"17 msec"`, `"500 usec"`, `0`). An entry
  needs at least one of them or `offset_runtime`. Any other key is an error.
- `offset` must not be negative. An offset of one period or more acts as a start
  delay.
- `period` must be greater than 0. It states the period the node's code creates the
  timer with, for a timer whose period the analyzer cannot read (a parameter, for
  example). It does not change the rclcpp timer: at startup the binding finds the
  timer by its exact period and refuses to start if none fits
  (`... has no unbound timer handle in node '...' that fits it`). To run a timer at
  another period, change the period in the node's code.
- A timer name the node does not have is an error that lists the node's timers:
  `no timer named 'tick' on 'timer_node' (known timers: timer_callback)`.
- There is no `defaults:` form. The user file merges field by field: a user `offset`
  keeps the package's `period` of the same timer.
- An offset on a timer whose period the analyzer could not read leaves the period
  unknown. Generation then refuses with
  [RCLLF-R001](../reference/diagnostic-codes.md#rcllf-r001) until the entry also
  sets `period`. An acknowledgement does not lift it.

The generated `.lf` shows the values, for example
`timer t_timer_callback(1500 msec, 1000 msec)`. `resolved_model.json` carries one
`timer:<node>/<callback>#offset` and one `#period` record per timer. A configured
value has the provenance `explicitly_configured` and names its `execution.yaml` entry.

#### Offsets set at program start

`offset_runtime: true` builds the timer once and takes its offset when the program
starts, so an offset sweep needs one build instead of one per offset:

```yaml
nodes:
  timer_node:
    timers:
      timer_callback:
        offset_runtime: true    # an offset in the entry becomes the default
```

```bash
RCLLF_TIMER_OFFSETS="timer_node/timer_callback=1500 msec" \
  ros2 lf launch timer_example timer_node
```

- The offset becomes a parameter of the main reactor (or of the federated reactor),
  named `<node>__t_<callback>_offset`. reactor-c sets it from the program option
  `--<parameter> <n> <unit>` before the start tag, so the run is the same as a build
  with that offset.
- [`RCLLF_TIMER_OFFSETS`](../reference/environment-variables.md#rcllf_timer_offsets)
  holds entries `<node>/<timer callback>=<integer> <unit>`, separated by `,` or `;`.
  Units are `nsec`, `usec`, `msec` and `sec` (and `ns`, `us`, `ms`, `s`). In a
  federation each federate takes the entries for its own timers.
- A name that is not a run-time timer of the build is refused at launch:
  `RCLLF_TIMER_OFFSETS: no run-time timer 'timer_node/nope' in this build. Run-time timers: timer_node/timer_callback`.
- Unset, the timer starts at the `offset` of the entry, else 0.
- The offsets come before
  [`RCLLF_LF_ARGS`](../reference/environment-variables.md#rcllf_lf_args) on the
  command line.

### Lifecycle auto-transition

A managed lifecycle node usually creates its publishers in `on_configure` and its
timers in `on_activate`, so an `unconfigured` node owns none of them and binding would
find nothing. The wrapper therefore drives such a node to `active` before binding. It
drives only the transitions that are missing, so a node already `active` is left
alone. A lifecycle node registered with `rclcpp_components_register_nodes` is built as
a non-component node, and generation says so by name.

Set `auto_transition: false` only for a node whose activation belongs to an
in-process supervisor or to `ros2 lifecycle set`:

```yaml
nodes:
  lifecycle_node:
    auto_transition: false
```

The value must be a real YAML boolean. `"false"` is a string and is refused. Such a
node stays idle until something activates it, and generation warns by name for every
node that opts out. While a node is not `active` none of its callbacks run: a timer
reaction is skipped, and a message, request or response its reaction takes is dropped.

A transition a reaction makes at tag T holds for every reaction at a later tag, so all
reactions at one tag agree on the state. A transition that `ros2 lifecycle set` or a
thread of the node's own makes enters LF as a physical action, so its tag follows the
wall clock and differs between runs. The
[lifecycle example](../examples/lifecycle-example.md) shows both.

### Ingress capacity

A binding thread takes each node's parameter, lifecycle and type description service
requests and stores them until an LF worker runs them. `ingress_capacity` is how many
one node may hold stored at once. A request that arrives while the node holds that many
is refused at once with a warning that names the node, and a mutating request
(`set_parameters`, `change_state`) gets a failure reply with the reason `overflow`, so
`ros2 param set` fails fast instead of timing out.

```yaml
nodes:
  /robot_1/config_node:
    ingress_capacity: 2
```

### Reaction order at one tag

LF orders the reactions of one tag along the connections between them. Declaration
order inside a reactor decides only among reactions that no connection orders (see
[Ties at the same logical tag](../design/architecture.md#ties-at-the-same-logical-tag)).
A timer and a subscription of the same node that fire at the same tag are such a pair.
`reaction_order` sets which group the generator declares first:

| Value | Order in the node's reactor |
|---|---|
| `timers_first` (default) | Timer reactions, then subscription reactions. A timer sees the state before the messages of its own tag. |
| `subscriptions_first` | Subscription reactions, then timer reactions. A timer sees the messages of its own tag. |

```yaml
nodes:
  /fusion_ties_consumer:
    reaction_order: subscriptions_first
```

The default was `subscriptions_first` in versions before v0.1.0. Set it under
`defaults` to keep that order for an older system. A node key that is not in
`system.json`, or any other value, is an error. `resolved_model.json` records the value
per node as `node:<node>#reaction_order`.

What the key does not change:

- Source order within each group.
- A timer callback that also sends a service request or an action goal. Its reaction
  is a sender reaction and stays with the subscriptions in both orders.
- Egress flush reactions (declared first), service and action server reactions
  (declared after the callbacks), and the shutdown close of a correspondence edge
  (declared last).
- Callbacks in a Reentrant callback group. Each gets its own sub-reactor. Callbacks of
  one MutuallyExclusive group, the default group included, follow the key.
- Reactions linked by a connection.

The key can decide whether a zero-delay feedback loop is accepted. A subscription
reaches its node's outputs at its tag only through the reactions declared at or after
it. So a latch declared after the publishing timer (`timers_first`) breaks a loop, and
the same latch under `subscriptions_first` closes a causality loop, which
`colcon lf-generate` refuses with the loop's links named. Give one link a delay d > 0
to break it.

### Workers

`workers` sets the LF `workers` target property, the number of worker threads of the
generated program. Unset, reactor-c picks the smaller of the host's core count and the
breadth of the reaction graph, so the same program runs with a different number of
workers on different hosts. Too few workers is the risk on a slow host: one worker runs
every reaction of a tag in turn, and a 1 ms tag whose reactions take more than 1 ms of
CPU falls further behind physical time at every tag until it exceeds
`max_worker_lag_ms`.

```yaml
defaults:
  workers: 2
```

The analyzed root launch file can set it too: a
`DeclareLaunchArgument('rcllf_workers', default_value='<n>')` gives the program
`workers: <n>`. A stock `ros2 launch` ignores the argument. The default must be an
integer above zero, and a different `workers` in `execution.yaml` is an error. A
reactor-c `-w N` in
[`RCLLF_LF_ARGS`](../reference/environment-variables.md#rcllf_lf_args) overrides both
for one run.

### Control failure policy

Every activation of a reaction that published on a cross-federate edge sends one
COMPLETE record for its tag, which lists what it published there. The records carry a
sequence number per edge. A gap in the numbers is a CONTROL_FAILURE, because the
receiver cannot tell what the missing activation published. `control_failure_policy`
decides what the receiving federate does:

| Value | Effect |
|---|---|
| `report_continue` (default) | Log and trace it, fail the reconcile at the end of the run, keep running. |
| `stop` | The same report, then request an LF stop at once. |

The default keeps running because a missing COMPLETE leaves earlier and later
activations intact. `stop` is for a system that must not act on a partial view of its
inputs. Two cases have no later number to expose them and still end as a
CONTROL_FAILURE:

- **A lost last record.** The end-of-run check compares the receiver's last number
  with the count the sender says landed by the stop tag.
- **A sender that stalls.** Under centralized coordination the receiver then sees its
  logical time stand still. A subscriber federate with a timer reports
  `CONTROL_FAILURE stall` once per episode in which its tag does not change for longer
  than W, the longest timer period of the federate plus d + T of every incoming
  cross-federate edge plus the worker lag bound L. The watchdog is off without a timer,
  with an infinite T, and in fast mode.

### Correspondence bounds

A topic edge gets [publication correspondence](correspondence.md) (every published
sample is matched to exactly one dispatch, or reported lost) only when its bounds are
known at generation. The analyzer proves the burst bound k where it can. These keys
supply what it cannot prove:

| Key | Meaning |
|---|---|
| `max_publishes_per_invocation` | Burst bound k. A callback of the node publishes at most k samples per invocation on any one topic. Overrides the analyzer's bound. |
| `max_worker_lag_ms` | Worker lag bound L, how far physical time may run ahead of a drain's tag. Unset, L is twice the largest timeout of an admitted edge, where a relay edge adds its upstream timeout. A value below that timeout is an error. |
| `reader_depth` | KEEP_LAST depth of the reader of an admitted edge. An edge whose k exceeds it is refused. |
| `join` | `blocking` dispatches at publish tag + d and waits for an absent sample up to its cutoff, which holds the rest of the program. `deferred` dispatches at publish tag + d + T and never waits, and delivers only a sample taken before that tag. `deferred` is an error on a self edge and on an edge inside one federate. |
| `cutoff_anchor` | `tag`, the only value. The cutoff counts from the publication's tag (see [Missing Messages](missing-messages.md#counting-the-cutoff-from-the-tag)). The old value `sender_start` is an error. |
| `min_input_period_ms` | Shortest gap between two messages on a subscription fed from outside the admitted edges. Bounds the rate of a callback that publishes on an admitted edge. When it sizes a bound it is checked at run time: the first violation logs `ros2_lf_input_period: first violation` with both samples, and shutdown logs `ros2_lf_input_period: ... violations=N`. |
| `require_admitted` | `true` makes any refused topic edge a generation error that names each edge and its reason. With it set, a `min_input_period_ms` that sizes no bound is an error too. |

Here d is the edge's after delay and T its `maxwait_ms`. Without a declaration, k is
the number of publish call sites of the callback on the edge's topic, following calls
to member functions of the node class. A site inside a loop or a lambda, a `goto`, a
recursive call or a publish on a publisher it cannot name leaves k unproven. A
subscription's deadline handler counts toward its callback. A publish the analyzer
misses is still caught at run time, as `burst_exceeded` at the end of the run.

From these the generator sizes each admitted edge, with N(W) the most publications the
edge can see in any window of length W:

| Edge | `burst` | `carrier` | `retain` |
|---|---|---|---|
| inside one federate | k | N(d) | N(d + L) + 1 |
| cross-federate, `blocking` | k | N(d) | N(d + L + T) + 1 |
| `deferred` | k | N(d + T) | N(d + T + L + T) + 1 |

A timer with period P runs floor(W / P) + 1 times in W. For example, a timer at 200 ms
with one publish per tick (k 1), d 1 ms and the default `maxwait_ms` of 100 gives
L 200 ms, so retain N(201 ms) + 1 = 3, carrier N(1 ms) = 1 and burst 1. The trace
manifest lists the values and their inputs per edge under `bounds`. More than `burst`
records in one reaction fails the run at shutdown with `burst_exceeded` (exit code 3).
An admitted edge's `maxwait_ms` is fixed at generation, so
[`ROS2_LF_MAXWAIT_MS`](../reference/environment-variables.md#ros2_lf_maxwait_ms) does
not change it.

On a cycle through a federation link, order the timeouts. A loss on a `blocking`
cross-federate edge x holds its subscriber's federate F until x's cutoff, t + d_x + T_x.
What F publishes then on an admitted edge y leaves F at t + T_x or later, but y's
cutoff is t + d_y + T_y. When y leads back toward x's publisher and
T_x > T_y + d_y, one loss on x makes y's samples miss their cutoff too. The rule: on an
admitted cycle through a federation link, the T of a blocking edge is at most the
smallest T + d of the admitted edges leaving its subscriber's federate toward the
cycle. Generation warns, naming both edges, when the rule is broken. A `deferred` edge
never holds its federate and is exempt.

An edge whose bounds are unknown is not admitted. The reason is in
`correspondence.json` and the manifest:

| Reason | Cause |
|---|---|
| `burst_unbounded` | A publishing callback has no proven bound and the node declares no `max_publishes_per_invocation`. |
| `unsupported_api` | The node publishes a serialized message, which the binding cannot match to a sample. |
| `period_unknown` | A publishing timer has no known period, or one below 1 ns or beyond the int64 nanosecond range. |
| `rate_unbounded` | A publishing callback is fed by a refused edge, by a publisher outside the system, or by a feedback cycle, and its subscription declares no `min_input_period_ms`. |
| `reader_depth_below_burst` | k exceeds the edge's `reader_depth`. |
| `carrier_unsized` | The bound came out as zero (defensive, not expected). |

Refusing one edge can make edges downstream of it `rate_unbounded`, so refusal repeats
until nothing changes. The reader depth is a detection boundary, not a prevention: if
more samples arrive between two takes than the depth holds, DDS overwrites the oldest,
and the overwritten sample is reported LOST. A bound that does not fit its 32-bit or
64-bit field stops generation with an error naming the edge and the field.

```yaml
nodes:
  corr_publisher:   # publishes in a loop, at most 3 per tick
    max_publishes_per_invocation: 3
```

## Canonical key forms

Node keys and topic keys match `system.json`: the keys of its `nodes` and the `topic`
field of each subscription. `execution.reference.yaml` lists them, so copy them from
there.

Keys match by runtime name, the name rclcpp gives the node or topic when the program
runs. The same rule holds in `nodes:`, `subscriptions:` and `publishers:`:

- A node key names the node with the same fully qualified name, so `controller` and
  `/controller` are one node.
- A topic key names the topic that resolves to the same name for that node. `/scan`
  stays as written, `scan` is joined to the node's namespace, and `~/scan` to the
  node's full name. Launch remappings are not applied: write the topic as the node's
  code spells it.

For a node in the root namespace `sensor_data` and `/sensor_data` are one topic. For a
node in `/robot` they are two, `/robot/sensor_data` and `/sensor_data`. Two keys of one
section that resolve to one name (`controller:` and `/controller:`) are refused with
an error that names both spellings. A key that matches nothing is an error that names
the key, the name it resolves to and the node's topics.

## Precedence, per-key, not per-block

Precedence resolves one policy key at a time:

```
built-in default  <  defaults:  <  nodes:  <  subscriptions:  <  env var
```

In the schema example, the `sensor_data` subscription of `/robot_1/subscriber_node`
resolves to:

| Key | Value | From |
|---|---|---|
| `take_mode` | `block` | `subscriptions:` |
| `maxwait_ms` | `250` | `nodes:`, since the subscription scope does not set it |

Setting `take_mode` at the subscription scope does not pull `maxwait_ms` along. Each
key falls back through the scopes on its own. A key set at a scope it is not valid for
(see the [Policy keys](#policy-keys) table) is an error.

## Env var overrides

[`ROS2_LF_TAKE_MODE`](../reference/environment-variables.md#ros2_lf_take_mode) and
[`ROS2_LF_MAXWAIT_MS`](../reference/environment-variables.md#ros2_lf_maxwait_ms) sit
above every scope, and each overrides only its own key, for every subscription. They
are for a quick experiment. To target one subscription, use the file.

- `ROS2_LF_TAKE_TIMEOUT_MS` is the alias of `ROS2_LF_MAXWAIT_MS`. An empty variable
  counts as unset. Both set to different values refuse the override, and the runtime
  logs one `WARN` that names both.
- `ROS2_LF_MAXWAIT_MS` does not change the timeout of an admitted edge, which is fixed
  at generation. Set `maxwait_ms` in the file and regenerate instead.
- Neither applies to an `external_topics` subscription.
- If an env var changes any resolved entry, the runtime logs one `WARN` that counts the
  entries:

```text
[WARN] [ros2_lf_binding]: ROS2_LF_TAKE_MODE=skip overrides take_mode for 1 of 1 configured subscription(s), not for the defaults, which already match
```

The count covers subscription entries. The defaults entry is reported apart.

In the Docker Compose services a `ROS2_LF_MAXWAIT_MS` set on the host also sets the
alias, so the two never disagree. `docker compose run -e ROS2_LF_MAXWAIT_MS=N` reaches
the container next to the alias's default of 100, so pass both variables there.

## Other validation rules

- Unknown keys, wrong types, values out of range or not among the allowed values, and
  an unknown `schema_version` are errors that start with the file path.
- A block key with no value (YAML null, such as `timers:` or `tf2:` with nothing under
  it) counts as absent. This holds for `defaults`, `nodes`, `subscriptions`,
  `external_topics`, `connections`, `publishers`, `acknowledged_hazards`, and a node's
  `timers` and `tf2`. An empty mapping `tf2: {}` is refused. A policy key with no value
  (`maxwait_ms:` alone) is refused as a wrong type.
- No file means the built-in defaults. A file that does not parse, or has a
  `schema_version` the tool does not know, stops the run. A typo never resolves to
  "as if nothing were set".

## Binding keys and `exec_config.json`

`execution.yaml` names nodes. The generated code looks up a binding key instead, which
identifies one node instance in every generated file: the reactor, its reaction IDs,
the `ros2_lf_bind_node` call, the node registration and the `node_class` field of
`exec_config.json`.

| Instances of the C++ class | Binding key |
|---|---|
| exactly one | the class name, e.g. `SubscriberNode` |
| two or more | `Class__<node key>`, e.g. `FeatureNode__primary_node` |

The suffix is the node key with leading slashes stripped, other characters than
letters, digits and `_` replaced by `_`, and runs of `_` collapsed, so
`/robot_1/timer_node` becomes `TimerNode__robot_1_timer_node`. When two node keys give
the same suffix, a `_h<6 hex>` hash of the key is appended. Two `FeatureNode`
instances therefore get two rows, one `block` and one `skip` if you like. Adding a
second instance of a class changes that class's key and its `RID_*` macro names. Your
`execution.yaml` is not affected, since it names nodes.

The resolved table is written as `exec_config.json` next to `system.json` and into the
generated package. The startup log names the file it loaded. For `pubsub_example` with
the subscription set to `block`:

```json
{
  "schema_version": 1,
  "defaults": {
    "take_mode": "skip",
    "take_timeout_ms": 100
  },
  "subscriptions": [
    {
      "node_class": "SubscriberNode",
      "topic": "sensor_data",
      "take_mode": "block",
      "take_timeout_ms": 50
    }
  ]
}
```

It carries only the runtime keys. `topic` is the model's form. The runtime matches it
against both the declared topic and the subscription's resolved name. In a federation
every federate gets the whole table and logs a line for each bound subscription without
a row. The file the runtime reads can be changed with
[`ROS2_LF_EXEC_CONFIG`](../reference/environment-variables.md#ros2_lf_exec_config).

## `colcon lf-configure` and `--exec-config`

`colcon lf-configure` scaffolds, validates and shows the configuration. `colcon lf`
runs it between analyze and generate.

```bash
colcon lf-configure                          # every package
colcon lf-configure --package pubsub_example # every analysis of one package
colcon lf-configure --system-json lf-analysis/pubsub_example/bringup/system.json
colcon lf-configure --show                   # print the resolved values
colcon lf-configure --interactive            # prompt for keys
```

`--package` and `--system-json` are mutually exclusive. `--package` takes every
analysis under `lf-analysis/<name>/` and prints the launch stems it matched.
`lf-generate` accepts the same two flags.

Every run regenerates `execution.reference.yaml`, creates `execution.yaml` if it is
missing, validates it against the model and reports stale entries. It never changes
an existing `execution.yaml`.

`--show` prints the resolved values before any env var, the same table that goes into
`exec_config.json`. Each value names its layer and scope. Then come `external_topics`
and `acknowledged_hazards` with their layer, then any env var that is set. For
`pubsub_example`, with the package file and a user file:

```text
--- /workspace/packages/lf-analysis/pubsub_example/bringup/system.json ---
  Resolved effective policy (post-merge, pre-env), each value with its layer:
    defaults:
      take_mode: "skip"  <- built-in
      take_timeout_ms: 100  <- built-in
    SubscriberNode.sensor_data:
      take_mode: "block"  <- user /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.yaml (subscriptions)
      take_timeout_ms: 50  <- user /workspace/packages/lf-analysis/pubsub_example/bringup/config/execution.yaml (subscriptions)
    external_topics: []
    acknowledged_hazards: []
--- /workspace/packages/lf-analysis/pubsub_example/subscriber_node/system.json ---
  Resolved effective policy (post-merge, pre-env), each value with its layer:
    defaults:
      take_mode: "skip"  <- built-in
      take_timeout_ms: 100  <- built-in
    SubscriberNode.sensor_data:
      take_mode: "skip"  <- built-in
      take_timeout_ms: 100  <- built-in
    external_topics:
      - /sensor_data  <- package /workspace/packages/src/pubsub_example/lf/subscriber_node.execution.yaml
    acknowledged_hazards: []
```

`take_timeout_ms` is the resolved name of `maxwait_ms`. `--show` lists only the
runtime keys. Check the generation keys in the generated `.lf`.

`--interactive` walks through `defaults:`, `nodes:` and `subscriptions:` with the
one-line help of each key, and writes only `execution.yaml`. It is meant for a first
setup.

`colcon lf-generate --exec-config path/to/execution.yaml` reads another user file for
this run, still over the package file. Without the flag it reads the `execution.yaml`
next to `system.json`.

## The block-timing footgun

`take_mode: block` does not change [tag](../design/glossary.md#tag) semantics. It never
makes up data, never delivers a message at another tag and adds no tardy handler. A
message that arrives after the timeout is dropped and reported, never delivered later
(see
[late samples](../reference/troubleshooting.md#a-subscriber-callback-sees-stale-data-or-none)).
This holds across federates too, as long as the link carries the writer's write count.
A link that lost those records, a program generated with `RCLLF_COUNT_PUBLISHES=0`, or
a topic whose publisher count the analysis cannot prove exact (a publisher name built
at run time, a publisher created outside the constructor or only on some paths) falls
back to the count rule with a `WARN` that names the edge and the reason. What `block`
changes is physical timing:

!!! danger "A per-subscription block can stall the whole tag"
    Reactions at one tag run in a fixed order (see
    [Firing-order determinism](../design/time-and-coordination.md#three-kinds-of-determinism)).
    If a reaction early in that order waits under `take_mode: block`, every reaction
    after it at that tag waits too, up to the full `maxwait_ms` of wall-clock time. One
    subscription with a generous timeout can stall reactions that have nothing to do
    with it.

Keep `maxwait_ms` as tight as the transport needs, and treat `block` on one
subscription as a decision about every other reaction at that tag.

## `acknowledged_hazards`

`acknowledged_hazards` lists the hazard codes your team reviewed and accepts for this
system. It is the only way to let `lf-generate` continue past an error-level hazard,
and it works per code. There is no flag that allows all hazards.

```yaml
acknowledged_hazards:
  - RCLLF-H003
```

Every acknowledgement is logged at generation with the code and the node and topic it
applies to.

## `external_topics`

`external_topics` lists the topics whose messages come from outside the generated
system, such as a sensor driver, `ros2 topic pub` or a rosbag. An entry is a topic
name, and `odom` and `/odom` name the same topic.

```yaml
external_topics:
  - /chatter
```

Which messages come from outside is a property of the launch file, so a package
declares them in its [package layer](#layers). A user file can add topics but not
remove the package's. `pubsub_example/lf/subscriber_node.execution.yaml` is one such
file.

A declared topic enters LF time as a physical action. The binding's ingress thread
takes each message when it arrives and schedules the reaction at that physical time.
The callback runs once per message, in arrival order. Every arrival is traced as an
`external_arrival` event (see the
[trace schema](../reference/trace-schema.md#fields-added-in-11-external-arrivals)).

The declaration is required. If a node subscribes to a topic that no node of the
system publishes and the topic is not listed, `colcon lf-generate` refuses, because
every message would be dropped without a word. The error prints the lines to add:

```text
  Error: node 'callback_group_node' (class CallbackGroupNode) subscribes to '/chatter', which no node in this system publishes. Under rcllf its callback would never be dispatched.
  Refusing to generate instead of dropping their messages silently. If the messages come from outside the system (a driver, 'ros2 topic pub', a rosbag), declare the topic(s) external in /workspace/packages/src/callback_group_example/lf/callback_group_node.execution.yaml:
    external_topics:
      - /chatter
```

Validation refuses a value that is not a list of strings or `topic:` mappings, a name
no subscription uses, and a name some node of the system publishes.

At startup the binding waits for every subscription to match a publisher before it
anchors LF time zero (up to
[`ROS2_LF_DISCOVERY_TIMEOUT_MS`](../reference/environment-variables.md#ros2_lf_discovery_timeout_ms)).
External subscriptions are left out of that wait.

An external callback may send a service request or an action goal. The server's
reaction then runs at the same tag. A lifecycle node may create its external
subscription in `on_configure` and drop it in `on_cleanup`. Messages sent while the
node holds no subscription are not received, as under stock ROS 2.

`take_mode` does not act on an external subscription, since its message is already
taken when the reaction runs. A resolved `take_mode: block` on one is refused at
generation with the lines that fix it:

```text
Error: take_mode 'block' on node 'callback_group_node' (binding key CallbackGroupNode) for external topic '/chatter', set by --exec-config /workspace/block.yaml (defaults). An external_topics message enters as an LF physical action that the binding has already taken when the reaction runs, so there is nothing to wait for and 'block' cannot apply. Set it to skip for this subscription:
    subscriptions:
      callback_group_node:
        /chatter:
          take_mode: skip
```

A `maxwait_ms` on an external subscription is inert and both `colcon lf-generate` and
`colcon lf-configure` warn about it. The env overrides leave external subscriptions
out, and their startup `WARN` says how many.

### Anchoring a topic at `header.stamp`

A plain entry gives each message the tag of its arrival. A topic whose messages carry
a `header.stamp` can instead be anchored at the stamp, so its reaction runs at a tag
derived from when the sample was taken:

```yaml
external_topics:
  - topic: /scan
    anchor: stamp
    delay_ns: 200000000           # T: late when arrival > header.stamp + T
    ingress_reserve_ns: 50000000  # time for the take and the insert (default 0)
    grid_ns: 10000000             # tags rounded up to this grid (default 1)
    basis: measured p99 lateness 83 ms, plus one 100 ms period
```

`delay_ns` and `basis` are required. There is no default T, because it is a property
of the sensor and the setup, and `basis` records where the value came from. The three
constants are integers in nanoseconds, each at most 2^60. `grid_ns` is at least 1.
A `tf2_msgs/TFMessage` topic is anchored at the common stamp of its transforms (see
[tf2 Lookups](tf2.md#external-tf)).

The binding applies these rules to each message, in order:

1. **Late is decided on arrival.** A message that arrives after
   `header.stamp + delay_ns` is refused as `late`. Exactly at that time is on time.
2. **The tag is absolute.** An on-time message is inserted at
   `start + ceil((header.stamp - start) + delay_ns + ingress_reserve_ns, grid_ns)`,
   microstep 0, where `start` is the start tag.
3. **Admission is decided under the scheduler's lock.** A tag at or before the
   current tag is refused as `overrun` (the ingress path took longer than
   `ingress_reserve_ns`). A tag after the stop tag is refused as `after_stop`. A tag
   reactor-c does not schedule because the program is stopping is `refused`.
4. **Two on-time messages in one grid cell collide.** Neither runs, so the outcome
   does not depend on which arrived first.

Every outcome is one `external_arrival` trace event with a `disposition` (see the
[trace schema](../reference/trace-schema.md#fields-added-in-15-stamp-anchored-arrivals)), and
shutdown prints the counts per topic. The message type must carry a `std_msgs/Header`
named `header`, checked at startup. A stamp anchor needs real time:
`colcon lf-generate --fast` refuses the system, and a launch-time `-f true` exits with
code 3. A user layer that gives a `topic:` mapping replaces the package's anchor for
that topic.

## `publishers`: output deadline

`publishers:` sets, per publisher, how late after its tag a publish may leave before it
is reported:

```yaml
publishers:
  /actuation_node:              # node key, as in system.json
    /actuation:                 # a topic the node publishes
      output_deadline_ns: 20000000
      basis: >-
        where the number comes from: a measurement, or a placeholder named as such
```

Both keys are required. `output_deadline_ns` is an integer from 0 to 2^60. The node
must be in `system.json` and publish the topic from a reaction, otherwise generation
refuses. Keys match by runtime name as in [Canonical key forms](#canonical-key-forms).

At run time, immediately before the real publish, the binding compares
`lateness = physical time - logical time` with the bound. A publish with
`lateness > output_deadline_ns` is late:

- It emits the trace event `actuation_late` and logs, at most ten times per reaction
  and topic,
  `rcllf actuation_late: RID <n> topic '<topic>' tag (<elapsed ns>, <microstep>) published <lateness> ns after its tag, above output_deadline_ns=<bound>; published anyway`.
  With [`RCLLF_TRACE_CALLBACKS=1`](../reference/environment-variables.md#rcllf_trace_callbacks)
  every late publish also prints a JSON line.
- **The message is always published, unchanged.** rcllf reports the lateness. It does
  not prevent or shorten it.
- These lines go to the console only, never to `/rosout`.
- At shutdown it logs one summary line per bounded reaction and topic that ran:
  `rcllf output deadline summary: RID <n> topic '<topic>' output_deadline_ns=<bound> checked=<n> late=<n> max_lateness_ns=<ns>`.

This is deliberately not an LF `deadline`, whose handler runs instead of the reaction
and would drop the message. Under `--fast` logical time runs ahead of physical time,
so no publish is ever late. A publish from a thread the callback started has no tag
and is not checked.

## Hazard diagnostics

<a id="rcllf-h000"></a><a id="rcllf-h001"></a><a id="rcllf-h002"></a><a id="rcllf-h003"></a><a id="rcllf-h004"></a><a id="rcllf-h005"></a>
`colcon lf-analyze` checks QoS settings and topic names that conflict with logical time
and stores the findings in `system.json`. `colcon lf-generate` refuses to generate
while an error-level hazard is present and not listed in
[`acknowledged_hazards`](#acknowledged_hazards), and prints warnings and notes without
stopping. A `system.json` without the `diagnostics` field was analyzed by an older
version, and `lf-generate` asks you to run `colcon lf-analyze` again. Every code, what
it means and how to resolve it is listed in
[Diagnostic Codes](../reference/diagnostic-codes.md).

## Limitations

- **No reconfiguration at run time.** The policy is read once at startup.
- **Binding keys change when a class gets a second instance.** See
  [Binding keys and `exec_config.json`](#binding-keys-and-exec_configjson).
  `execution.yaml` is not affected.
- **Services and actions have no policy keys.** `take_mode` and `maxwait_ms` apply to
  subscriptions only.
- **QoS policies get no LF meaning.** The hazard checks detect a QoS setting that
  conflicts with logical time. DDS deadline, liveliness and lifespan events never
  reach the callbacks. A missing message is reported by rcllf's own rule instead (see
  [Missing Messages](missing-messages.md)).
- **External subscriptions ignore `take_mode`.** A resolved `block` is refused at
  generation.
- **A transition from a thread LF does not own is seen late.** Its tag depends on
  timing. Transitions from callbacks, in-process supervisors and `ros2 lifecycle set`
  are seen at their own tag.
- **Deactivation does not cancel action goals.** Their feedback and results are
  dropped while the node is not active. A service response dropped then leaves its
  request pending in the client.
- **A `transient_local` sample published in a constructor is not delivered** to the
  subscribers of the system. Publish the first value from a timer or a callback.
- **No `message_filters` conversion.** Two-input ExactTime and ApproximateTime
  synchronizers run unmodified.
  [RCLLF-H003](../reference/diagnostic-codes.md#rcllf-h003) refuses every other form.

# Missing Messages

This page explains what a node sees when a message it is owed does not arrive in time,
and the rule that decides "in time": the cutoff and its `maxwait_ms`. It is the one
page for that rule. The keys that set it are in the
[execution.yaml Reference](execution-config.md#take_mode-and-maxwait_ms).

On an admitted topic edge (see
[publication correspondence](../design/time-and-coordination.md#publication-correspondence-topic-edges)),
every publish creates an expectation: the subscriber is dispatched with exactly that
sample at its release tag. A sample that does not arrive in time is a missing message.
rcllf calls the node's event callbacks for it and reports it in the log and the trace.
That makes [the determinism claim](../design/time-and-coordination.md#the-determinism-claim)
checkable: two runs with the same input timeline in which every message arrives within
its timeout give identical outputs, and a run that differs carries a loss report.

## What the node sees

At the release tag of the missing message, in the reaction that would have run the
subscription callback, the binding calls the subscription's event callbacks:

| Registered in `SubscriptionOptions::event_callbacks` | Called with |
|---|---|
| `deadline_callback` | `QOSDeadlineRequestedInfo{total_count = n_d, total_count_change = 1}` |
| `message_lost_callback` | `QOSMessageLostInfo{total_count = n_l, total_count_change = 1}` |

```cpp
rclcpp::SubscriptionOptions options;
options.event_callbacks.deadline_callback =
  [this](rclcpp::QOSDeadlineRequestedInfo & info) {use_last_known_value(info.total_count);};
options.event_callbacks.message_lost_callback =
  [this](rclcpp::QOSMessageLostInfo & info) {count_gap(info.total_count);};
sub_ = create_subscription<Msg>("/topic", qos, cb, options);
```

- Both run when both are registered, the deadline callback first.
- Only the registered one runs when only one is registered.
- Nothing runs when neither is registered. The log and the trace still report the
  missing message.
- `n_d` and `n_l` are logical counts on this subscription in this run, never the DDS
  status. `n_l` counts missing messages. `n_d` counts missing messages and
  [silent deadline windows](#a-producer-that-falls-silent), so it equals `n_l` until the
  first silent window.
- A publish from inside either callback belongs to the same reaction and tag as a
  publish from the subscription callback.
- After shutdown has begun, no callback runs. If the deadline callback starts the
  shutdown itself (for example with `rclcpp::shutdown()`), the `message_lost_callback`
  of the same message is skipped.

The node needs no change. A node that already handles a missed deadline or a lost
message in its event callbacks keeps doing so, now at a deterministic tag.

## What the log says

A missing message is logged when it is decided, whether or not the node registered a
callback:

```
[WARN] [ros2_lf_binding.corr]: rcllf corr: edge 0: writer 01000000... ordinal 7 TIMED_OUT
  (publish tag 600000000 ns): the sample never reached its reaction and is missing
  (1 missing on this edge so far, 1 since the last line; at most one line per edge
  per 1000 ms, the record at stop lists them all)
```

The first missing message of an edge prints at once. Later ones print at most one line
per edge per second, with the count since the last line, so a 1 kHz edge with a dead
writer logs one line a second.

At shutdown the binding logs one record per process, `rcllf corr: {...}`, with a
`missing_total` and a `missing` list per edge. Its two verdict fields:

| Field | Meaning |
|---|---|
| `reconciled` | The accounting balanced: every token, dispatch and record ended exactly once. The exit code and the `run_end.reconciled` trace field carry the same verdict. |
| `missing_total` | Missing messages of all edges (`NOT_READY`, `TIMED_OUT`, `LOST`, `OVERFLOW`). |

A run can be reconciled and still have missing messages. `"reconciled": true` beside
`"missing_total": 3` means the binding knows exactly which three went missing. A tool
that wants "no fault" checks both fields.

## When a message is missing

A message is missing when its sample does not reach the subscriber before the cutoff.
Each publication gets its cutoff when it is published:

```
cutoff = t + max(0, d) + T + eps
```

- t is the publication's logical tag.
- d is the edge's after delay (see
  [Per-connection after-delays](execution-config.md#per-connection-after-delays)).
- T is the subscription's `maxwait_ms` (100 ms by default).
- eps is the bound on the clock offset between the two hosts. It is 0 between
  federates on one host, and otherwise the value of
  [`ROS2_LF_XFED_CLOCK_BOUND_NS`](../reference/environment-variables.md#ros2_lf_xfed_clock_bound_ns).

A sample counts only when the subscriber took it in, on its own clock, strictly before
the cutoff. When the worker processes it, or when the publisher's record arrives, does
not matter. This is the maxwait rule that Paladino et al. define for the network inputs
of an LF federation, and it is the only cutoff of a cross-federate edge.

A sample that misses the cutoff is never handed to the subscription callback. The
drain at the release tag ends with a missing outcome (`TIMED_OUT`, `LOST`, `NOT_READY`
or `OVERFLOW`, kept in the trace as the cause). Which label appears can depend on
physical timing. What the node sees depends only on "missing".

While a join waits for a sample, it holds its tag, and every later tag of the program
waits too, since LF runs the reactions of a tag level by level. A loss on a
cross-federate edge is detected at its cutoff. That costs latency, never a spurious
loss.

### Counting the cutoff from the tag

The cutoff counts from the publication's tag, not from when the publisher ran. A
publisher that falls behind its tags therefore makes its outputs late downstream, and
the receiver reports them missing. Each hop's `maxwait_ms` must cover the timeouts
upstream of it plus the execution time in between, so timeouts grow per hop.

Example: a chain S -> A -> B of three federates, with 10 ms timers, d = 0 and
T = 50 ms on both edges. S stalls for 60 ms in its tick at tag 1000 ms.

| Sample | Cutoff | Arrives | Result |
|---|---|---|---|
| S's tag 1010 ms output at A | 1010 + 50 = 1060 ms | after 1060 ms, since S runs 60 ms late | missing at A |
| A's tag 1010 ms output at B | 1010 + 50 = 1060 ms | after 1060 ms, since A waited for S until 1060 ms | missing at B |
| the same, with B's T = 100 ms | 1010 + 100 = 1110 ms | about 1060 ms | on time |

With d = 30 ms on the A -> B edge, B dispatches A's tag-t sample at t + 30 ms and cuts
it at t + 30 + 50 = t + 80 ms.

`cutoff_anchor: tag` in `execution.yaml` names this rule and is the only value.

### Edges inside one federate have no cutoff

The cutoff belongs to cross-federate edges, where a sample travels over the network.
An edge inside one federate has none. The publishing thread takes the sample from the
subscriber's in-process DDS reader before its publish returns, so the edge is decided
then: the sample is there, or the publication ends `LOST`, counted as
`local_undelivered`, and the run fails with exit code 3. The reconcile row of such an
edge reads `local_now: true` and `cutoff: none`. This is the LF meaning of a connection
inside one program: a value is present or absent at its tag, never in a race against
physical time.

On such an edge `maxwait_ms` only sizes the worker lag bound and the retention. Worker
lag imposes no cutoff there: under sustained overload the edge delivers its values late
instead of dropping them, and the
[worker lag bound](execution-config.md#correspondence-bounds) `max_worker_lag_ms` ends
the run with `worker_lag_exceeded` and exit code 3.

### Not waiting: `join: deferred`

`join: deferred` in `execution.yaml` makes a subscription's cross-federate edges never
wait. Its drain runs at the cutoff tag, publish tag + d + T, instead of the release
tag. A lost sample then holds no other reaction. The price is latency: every sample of
the edge reaches the callback T later. The cutoff rule is the same. Which samples make
it can still differ between runs when an arrival races the cutoff. `deferred` on an
edge inside one federate is a generation error.

### Feedback loops

Admitted edges may form a loop across nodes, as long as the loop is not a causality
loop.

- **The rule.** A loop of zero-delay edges on which every subscription publishes onward
  into the loop at the tag it receives is a causality loop. `colcon lf-generate`
  refuses it with an error that names the loop's links. No microstep is added for you.
- **The fix.** Give one link a delay d > 0 with a
  [`connections:`](execution-config.md#per-connection-after-delays) entry.
- **What is admitted.** A loop on which one subscription only stores, or publishes only
  through reactions declared before it (see
  [Reaction order at one tag](execution-config.md#reaction-order-at-one-tag)). A self
  edge, where one node publishes and subscribes the topic in one process.

Example: node A publishes `/x` from a timer, node B relays `/x` to `/y`, and A
subscribes to `/y` and publishes `/x` from that callback. Every link has d = 0, so the
loop is refused. With this entry A's `/y` callback runs 1 ms later, and the loop is
admitted:

```yaml
connections:
  /y:
    after_delay: "1 msec"
    reason: default
```

## Several publishers into one subscription

When several nodes publish on the topic of one subscription, their admitted edges form
one fan-in group with one delivery order (see
[Several publishers into one subscription](../design/time-and-coordination.md#several-publishers-into-one-subscription)).
Detection stays per publisher:

- Each publish gets its cutoff on its own publisher's edge, from its own publish tag
  and its own edge's timeout.
- A missing message takes its place in the group's order: the event callbacks run
  where its sample would have been delivered.
- `n_l` and `n_d` count the missing messages of the whole subscription, as ROS reports
  lost messages per subscription. The `missing_hook` trace record names the publisher's
  edge (`edge`), its writer (`gid`) and the publish ordinal (`ordinal`).
- A lost sample holds the group's drain until its cutoff. The samples behind it in the
  order wait in physical time, and their dispatch tags do not change.

## A producer that falls silent

A missing message needs a publish to be named. A producer that never publishes, or
stops, makes no record, so it needs a deadline of its own. A subscription declares its
input periodic when its QoS carries a `.deadline(D)` and it registers a
`deadline_callback`. On such a subscription the binding keeps a logical window of
length D:

- The window starts at the tag of the last message the drain delivered on the
  subscription (received or missing, from any publisher of a fan-in group), or at the
  start tag before the first one.
- When no message is delivered at a tag up to and including `(t_ref + D, 0)`, the
  `deadline_callback` runs at `(t_ref + D, 0)` with
  `QOSDeadlineRequestedInfo{total_count = n, total_count_change = 1}`, and again at
  `(t_ref + 2 D, 0)`, `(t_ref + 3 D, 0)`, and so on while the silence lasts.
  `message_lost_callback` does not run, since nothing was lost.
- Messages released at the window's end tag are delivered first, so a message exactly
  D after the last one closes the window in time.
- **No double report.** A missing message is a delivered record: its own deadline
  callback runs at its tag, and that tag restarts the window. A stream of period
  P < D that loses one sample gets exactly one deadline call.
- No window ends after the stop tag.

This mirrors the DDS requested deadline, which Fast DDS runs per instance as one timer
of period D that each sample restarts. A keyless topic is one instance, which is why a
fan-in group shares one window. One difference: DDS starts the timer at the first
sample, so stock ROS 2 reports nothing for a stream that never starts. rcllf starts the
first window at the start tag and reports it at `t0 + D`. All inputs are logical tags,
so a replay gives the same windows. The first silent window per subscription also logs
a warning.

## Physical DDS events never reach the callbacks

Under plain ROS 2, DDS raises deadline and sample-lost events in physical time, and an
executor runs the callbacks. Under rcllf no node is added to an executor, so a
`.deadline(p)` in the QoS never produces a physical callback, and neither does a DDS
sample-lost status. Only the logical missing message and the silent window above reach
the callbacks. The QoS deadline period is not mapped onto `maxwait_ms`. It sets only the
[silence window](#a-producer-that-falls-silent).

## Trace

On the `RCLLF_TRACE_CALLBACKS` channel (see the
[trace schema](../reference/trace-schema.md#correspondence-json-records)):

- The `dispatch` record of a missing outcome carries `missing_hook`, the first callback
  that ran (`deadline`, `message_lost`, `none` or `shutdown`), and `missing_hooks`, the
  list of every callback that ran, in call order.
- One `missing_hook` record per missing message whose callbacks ran, with `total`
  (`n_l`) and `hooks` (the same list).
- One `silence_deadline` record per silent deadline window, with the window's bounds,
  `period_ns`, `hook` (`deadline`, `inactive` or `shutdown`) and `total` (`n_d`).

# Missing Messages

On an admitted topic edge (see
[publication correspondence](../design/time-and-coordination.md#publication-correspondence-topic-edges)),
every publish creates an expectation that the subscriber is dispatched with exactly
that sample at its release tag. This page explains what your node sees when that
sample never arrives in time. Such a message counts as lost, and the run reports it.
That is what makes [the determinism claim](../design/time-and-coordination.md#the-determinism-claim)
checkable: two runs with the same modeled input timeline in which every modeled
message arrives within its timeout T must give identical outputs, and a run that
differs must carry a loss report. The places where
rcllf does not yet meet that are listed there.

## When a message is missing

Each expectation gets an admission cutoff when it is published:

```
cutoff = t + max(0, after-delay of the edge) + maxwait_ms + eps
```

t is the publication's logical tag and eps the declared bound on the offset of the two
hosts' clocks (0 in a fast federate and between two federates of one boot). This is
the maxwait rule of Paladino et al. (Eq. 1b), and it is the only
cutoff of a cross-federate edge. `maxwait_ms` is the subscription's
[execution configuration](execution-config.md) key (default 100 ms). A sample counts
only when its ingress stamp, the time the subscriber took it in on its own clock, is
strictly before the cutoff. When the worker processes the sample, and when the
publisher's record reaches the subscriber, does not matter. Every record
of one publication shares the cutoff.

Because the cutoff counts from the tag, a sender that runs behind its tags makes its
outputs late downstream. The receiver reports them `LATE` or missing at its own join
(late means lost and reported). So each hop's `maxwait_ms` must cover the
upstream timeouts plus the execution time in between, and timeouts grow per hop.

In earlier versions the default counted from S(a), the physical start of the publishing
activation, and the tag rule was opt-in. That rule and its
`publisher_late` counter are gone. `cutoff_anchor: tag` still parses and changes
nothing, `cutoff_anchor: sender_start` is a generation error.

### Edges inside one federate have no cutoff

The cutoff belongs to the cross-federate edges, where a sample travels over the
network. An edge inside one federate, publisher and subscriber in
one process, has none. The publishing thread takes the sample from the subscriber's
in-process DDS reader before its publish returns (Fast DDS delivers in the publish
call, the premise the binding checks at startup) and retains it in the correspondence
table, so the edge is decided when the publish returns: the sample is there, or the
publication ends `LOST`, counted as `local_undelivered`, and the reconcile fails
(exit code 3). The drain at t + d never waits on a local edge and no sample is late
there. The reconcile row of such an edge reads `local_now: true` and `cutoff: none`.
That is the LF meaning of a connection inside one program: a value is present or
absent at its tag, never in a race against physical time. The subscription's
`maxwait_ms` then only sizes the worker lag bound L and the retention.

### Counting the cutoff from the tag

The tag rule applies to cross-federate edges only, the network inputs on which
Paladino et al. define maxwait. An edge inside one federate has no cutoff at all
(above). With a nonzero after-delay d (`connections:` or
`defaults.after_delay`) the receiver sees the tag-t sample at t + d and cuts it at
t + d + T + eps.

Measured on the S -> A -> B chain of `maxwait_chain_fixture` (a fixture of the development repository,
not shipped in the release) as a per-node federation
(10 ms timers, T = 50 ms on both edges, S stalls 60 ms in its tick at tag 1000 ms).
S's next sample (tag 1010 ms) is cut at 1060.0 ms and reported late, and A's own
tag-1010 output, whose activation waited for S through logical time, is late at B as
well. With B's T raised to 100 ms (A's T plus B's own 50 ms) nothing is late at B.
With d = 30 ms on `/chain/a` B dispatches the tag-t sample at t + 30 ms and cuts it at
t + 80 ms. Under the removed S(a) rule the tag-1010 sample was cut at its start plus
T, about 1110 ms, and was on time.

On a cross-federate edge a lost sample of tag t is detected at `t + max(0, d) + T + eps`.
While its join waits it holds its tag, since LF runs the reactions of a tag level by level, and every later
tag of the program waits too. That costs latency, never a spurious loss.

A subscription can opt out of the wait on its cross-federate edges
with `join: deferred` in its execution config (on an edge inside one federate the join
never waits, and `deferred` there is a generation error). Its drain then runs at the cutoff tag, publish tag + d + T, instead
of the release tag. A deferred edge never blocks the LF worker waiting for its sample,
and its dispatch tag is fixed, so a lost sample there holds no other reaction. The
price is latency: every sample of the edge, delivered or not, reaches the callback T
later. A sample counts only when it was taken in strictly before the cutoff. One taken
in at or after the cutoff is late even when the worker runs the join later, and one
taken in before it is still delivered when the worker runs the join late, or, on a
cross-federate edge, when the publisher's record comes after the cutoff. Which
samples make it can still differ between runs when an arrival races the cutoff. LF
still waits for physical time to reach each tag. A local edge, self edges included, has
no cutoff, and `join: deferred` on it is a generation error.

Admitted edges may form a loop across nodes, as in LF, which "allows
zero-delay feedback loops as long as there is no causality loop" (Edward Lee). An
in-process zero-delay edge runs its first record at the publication tag,
on a loop too. A loop of such edges on which every subscription publishes onward into
the loop at the tag it receives is a causality loop, and `colcon lf-generate` refuses
it with an error that names the loop's links, as the LF compiler would. Give one link a
logical delay d > 0 (a [`connections:`](execution-config.md#per-connection-after-delays)
entry) to break it. No microstep is added implicitly. A loop on which one subscription
only stores, or publishes only before it in its node's reaction order, has no causality
loop and is admitted. A self edge, where one node publishes and
subscribes the topic in one process, is admitted too, since publisher and reader share
one process. Like every edge inside one
federate it is decided when its publish returns (earlier versions
had the drain take the present sample itself, without the cutoff test). Worker
lag alone therefore imposes no cutoff on a local edge: under sustained overload the
edge delivers its values late rather than dropping them for lateness, and the
[worker lag bound](execution-config.md) `max_worker_lag_ms` reports the lateness, so
the process ends with `worker_lag_exceeded` and exit code 3. The one way a local
publication is lost is the `local_undelivered` count above, reported as `LOST`, when
the publish returns without the reader having the sample. On a
cross-federate edge a sample whose arrival stamp is later than the cutoff is never
handed to the subscription callback. The drain at the release tag then ends with a
missing outcome (`TIMED_OUT`, `LOST`, `NOT_READY` or `OVERFLOW`, kept in the trace as
the cause). Which label appears can
depend on physical timing, but what the node sees depends only on "missing".

## What the node sees

At the release tag of the missing message, in the same reaction that would have run
the subscription callback, the binding calls the subscription's event callbacks:

| Registered in `SubscriptionOptions::event_callbacks` | Called with |
|---|---|
| `deadline_callback` | `QOSDeadlineRequestedInfo{total_count = n_d, total_count_change = 1}` |
| `message_lost_callback` | `QOSMessageLostInfo{total_count = n_l, total_count_change = 1}` |

- Both run when both are registered, deadline first.
- Only the registered one runs when only one is registered.
- Nothing runs when neither is registered. The trace still records the missing outcome,
  and the binding logs the loss either way (next section).
- `n_d` and `n_l` are logical counts on this subscription so far in this run, never
  the DDS status. `n_l` counts missing messages. `n_d` counts missing messages and
  [silent deadline windows](#a-producer-that-falls-silent), so it equals `n_l` until
  the first silent window.
- Publishes from inside either callback are attributed to the drain reaction at that
  tag, exactly like publishes from the subscription callback.
- After shutdown has begun, no callback runs (`missing_hook` is `shutdown`). If the
  deadline callback itself begins shutdown (for example by calling
  `rclcpp::shutdown()`), the message_lost callback of the same missing message is
  skipped, and the trace lists only `deadline`.

The node needs no change. A node that already handles a missed deadline or a lost
message in its event callbacks keeps doing so, now at a deterministic tag.

## What the log says

A missing message is reported when it is decided, not only at shutdown,
whether or not the node registered a callback:

```
[WARN] [ros2_lf_binding.corr]: rcllf corr: edge 0: writer 01000000... ordinal 7 TIMED_OUT
  (publish tag 600000000 ns): the sample never reached its reaction and is missing
  (1 missing on this edge so far, 1 since the last line; at most one line per edge
  per 1000 ms, the record at stop lists them all)
```

The line is rate-limited per edge: the first missing message of an edge prints at
once, later ones at most one line per edge per second, carrying the count since the
last line. One line a second keeps an edge at 1 kHz with a dead writer at one line per
second instead of a thousand, and a single loss is never held back.

At shutdown the binding logs one record per process, `rcllf corr: {...}`, with a
`missing_total` and a `missing` list per edge. Its two verdict fields:

| Field | Meaning |
|---|---|
| `reconciled` | the accounting balanced: every token, dispatch and record ended exactly once. The process exit code and the `run_end.reconciled` trace field carry the same verdict |
| `missing_total` | missing messages (`NOT_READY`, `TIMED_OUT`, `LOST`, `OVERFLOW`) of all edges: the separate fault count |

A run can be reconciled and still have lost messages: `"reconciled": true` beside
`"missing_total": 3` says the binding knows exactly which three went missing (each
was warned above and is in the `missing` list of its edge). A tool that wants "no
fault" checks both fields; `colcon lf-build`'s smoke check and the benchmark oracles
check the accounting and judge the missing messages from the record's lists.

## Several publishers into one subscription

When several nodes publish on the topic of one subscription, their admitted edges form
one fan-in group with one delivery order (see
[several publishers into one subscription](../design/time-and-coordination.md#several-publishers-into-one-subscription)).
Detection stays per publisher:

- Each publish gets its expectation and its cutoff on its own publisher's edge,
  computed as above from its own publish tag and its own edge timeout.
- A missing message takes its place in the group's order: the event callbacks run
  where its sample would have been delivered, after every sample of a smaller release
  tag, rank or publication order, and before the larger ones.
- `n_l` and `n_d` count the missing messages of the whole subscription, as ROS reports
  lost messages per subscription. The `missing_hook` trace record names the publisher's
  edge (`edge`), its writer (`gid`) and the publish ordinal (`ordinal`).
- A lost sample holds the group's drain until its cutoff. The samples of the other
  publishers that follow it in the order wait behind it in physical time, and their
  dispatch tags do not change. This is the hold described in
  [When a message is missing](#when-a-message-is-missing), now inside one subscription,
  and the generation warning about it judges every member like any other edge.

```cpp
rclcpp::SubscriptionOptions options;
options.event_callbacks.deadline_callback =
  [this](rclcpp::QOSDeadlineRequestedInfo & info) {use_last_known_value(info.total_count);};
options.event_callbacks.message_lost_callback =
  [this](rclcpp::QOSMessageLostInfo & info) {count_gap(info.total_count);};
sub_ = create_subscription<Msg>("/topic", qos, cb, options);
```

## A producer that falls silent

A missing message needs a publish to be named. A producer that never publishes, or
stops publishing, makes no record, so it also needs a deadline of its own.
A subscription declares its input periodic when its QoS carries a
`.deadline(D)` and it registers a `deadline_callback`. On such a subscription the
binding keeps a logical window of length D:

- The window starts at the tag of the last message the drain delivered on the
  subscription (received or missing, from any publisher of a fan-in group), or at
  the start tag before the first one.
- When no message is delivered at a tag up to and including `(t_ref + D, 0)`, the
  `deadline_callback` runs at `(t_ref + D, 0)` with
  `QOSDeadlineRequestedInfo{total_count = n, total_count_change = 1}`, and again
  at `(t_ref + 2 D, 0)`, `(t_ref + 3 D, 0)`, ... while the silence lasts. One call
  per elapsed window. `message_lost_callback` does not run: nothing was lost.
- Messages released at the window's end tag are delivered first, so a message
  exactly D after the last one closes the window in time.
- **No double report.** A missing message is a delivered record: its own deadline
  callback runs at its tag, and that tag restarts the window. A stream of period
  P < D that loses one sample therefore gets exactly one deadline call.
- No window ends after the stop tag.

This mirrors the DDS requested deadline, which Fast DDS runs per instance as one
timer of period D that each sample restarts and each expiry reports once
(`DataReaderImpl::deadline_missed` in Fast DDS 2.14); a keyless topic is one
instance, which is why a fan-in group shares one window. The difference: DDS starts
an instance's timer only at its first sample, so stock ROS 2 reports nothing for a
stream that never starts. rcllf starts the first window at the start tag and
reports it at `t0 + D`. All inputs are logical tags, so a replay gives the same
windows. The trace marks each window with a `silence_deadline` record (see below),
and the first one per subscription also logs a warning.

## Physical DDS events never reach the callbacks

Under plain ROS 2, DDS raises these events in physical time and an executor runs the
callbacks. Under rcllf no node is ever added to an executor, the binding binds only
action waitables, and ingress refuses every other waitable. A `.deadline(p)` in the
QoS therefore never produces a physical callback, and neither does a DDS sample-lost
status. The runtime tests provoke both for real (a 20 ms QoS deadline against a 200 ms
timer, and a best-effort probe subscription flooded from another process until Fast
DDS reports SAMPLE_LOST) and check that only the logical missing message reaches the
callbacks. The QoS deadline period is also not mapped onto `maxwait_ms`, which
stays the only knob for missing messages; it sets only the
[silence window](#a-producer-that-falls-silent).

## Trace

On the `RCLLF_TRACE_CALLBACKS` channel (see the
[trace schema](../reference/trace-schema.md#correspondence-json-records)):

- The `dispatch` record of a missing outcome carries `missing_hook`, the first
  callback that ran (`deadline`, `message_lost`, `none` or `shutdown`), and
  `missing_hooks`, the list of every callback that ran, in call order.
- One `missing_hook` record per missing message whose callbacks ran, with `total`
  (`n_l`, the lost-message count) and `hooks` (the same list).
- One `silence_deadline` record per silent deadline window, with the window's
  bounds, `period_ns`, `hook` (`deadline`, `inactive` or `shutdown`) and `total`
  (`n_d`, the deadline count).

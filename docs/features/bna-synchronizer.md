# BNA Synchronizer

`BoundedNearestToAnchor` (BNA) is a `message_filters` policy shipped by
`rcllf_message_filters`. It pairs each sample of an anchor input with
the candidate sample whose header stamp is nearest, within a hard window. Every
decision and every sample outcome is traced, so a run can be checked against an
independent oracle. BNA is an rcllf refinement. It is not `ApproximateTime` and does
not try to reproduce what `ApproximateTime` would pick.

!!! note "Status: experimental"
    BNA is generated and smoke-tested but has no evaluation behind it (see the
    [Support Matrix](support-matrix.md)). It is an rcllf policy, so a node that uses
    it does not build against stock ROS without the `rcllf_message_filters` package.

## Try it

`bna_example` has an anchor sensor, a candidate sensor and a BNA fusion node. It is not
in the first-start build, so name it in `ENTRYPOINT_PACKAGES` (see the
[Docker reference](../reference/docker.md)). The program runs until you stop it:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES=bna_example dev bash -c \
  'colcon lf --package bna_example && ros2 lf launch bna_example bringup'
```

Without the selection, `colcon lf` stops with `'rcllf_message_filters/bna_policy.hpp'
file not found` and `Analysis failed`. The `apt-get install` hint printed below that
does not help: the header is part of this workspace and the selection builds it.

The `RCLLF_BNA` lines of the [trace](#trace) appear on the terminal, the first ones
like this:

```text
[Bringup-1] RCLLF_BNA {"ev":"params","sync":0,"epoch":10403944782035913331,"delta":20000000,"lateness":[0,30000000],"capacity":[4,4],"retention":350000000,"timeout":300000000}
[Bringup-1] RCLLF_BNA {"ev":"admit","sync":0,"input":0,"stamp":1795774719673914138,"key":"10403944782035913331:010f7fc4d402a11c0000000000003e03:1","tag":[1795774719673914138,0]}
```

## Opting in

The policy is an explicit choice in the node source. Nothing rewrites an existing
`ApproximateTime` synchronizer.

```cpp
#include <rcllf_message_filters/bna_policy.hpp>

using Policy = rcllf_message_filters::sync_policies::BoundedNearestToAnchor<Anchor, Candidate>;
message_filters::Synchronizer<Policy> sync(
  Policy(delta_ns, lateness0_ns, lateness1_ns, capacity0, capacity1,
    retention_ns, timeout_ns),
  sub_anchor, sub_candidate);
sync.registerCallback(&Node::callback, this);
```

The parameters must be literals, so the analyzer can read them. Each input topic
declares where its stamps come from in `execution.yaml`.

```yaml
subscriptions:
  bna_fusion:
    /bna/anchor:
      stamp_provenance: lf_clock
    /bna/candidate:
      stamp_provenance: lf_clock
```

`lf_clock` means the publisher stamps with `now()`, which under rcllf is the logical
tag of the reaction. You must declare it: rcllf does not detect the stamp source from
the code. Generation reports `RCLLF-H010` (info) for
every admitted BNA synchronizer, as a reminder that the outcomes follow the rules
below and not `ApproximateTime`. Any other `message_filters` form stays under
[RCLLF-H003](../reference/diagnostic-codes.md#rcllf-h003).

### Lost messages and `max_stamp_lag`

A message lost on an input (its `message_lost` handler runs) reaches the engine as a
missing report. By default the report does not move the input's watermark, so an
anchor that waited for the lost candidate waits for the next delivered candidate or
its timeout. An `lf_clock` input may declare how far its stamps lie below their
publish tag:

```yaml
subscriptions:
  bna_fusion:
    /bna/candidate:
      stamp_provenance: lf_clock
      max_stamp_lag: 0 ms
```

With it a lost message counts as a stamp of at least its publish tag minus
`max_stamp_lag`. The watermark rises to that bound at the loss report and pending
anchors decide there. `0 ms` fits a publisher that stamps with `now()` in the reaction
that publishes. A stamp taken in an earlier reaction or forwarded by a relay lies
below the publish tag, so it needs the largest such gap. A lag that is too small
makes the engine decide too early and reject a later in-window sample as late; rcllf
does not check the stamps. Generation refuses `max_stamp_lag` on a `declared:<text>`
input, and without the key the generated code is unchanged.

## Parameters

| Parameter | Meaning |
|---|---|
| `delta` | Window. A candidate is a partner when its stamp is at most `delta` away from the anchor stamp, bounds included. |
| `lateness0`, `lateness1` | How far behind its largest stamp an input may still deliver. The watermark of an input is its largest stamp minus its lateness. |
| `capacity0`, `capacity1` | Samples kept per input. Evicting the oldest one rejects the anchors that could still have used it. |
| `retention` | How long a candidate waits for an anchor, from its admission. |
| `timeout` | How long an anchor waits for its decision, from its admission. |

## Rules

An anchor is decided once both watermarks are past its stamp plus `delta`. It then
takes the nearest candidate within `delta` (the earlier stamp wins a tie), or it is
rejected as missing. A candidate that no pending or later anchor can reach is
superseded. A sample at or below its input's watermark is late. An anchor that is
still undecided at its timeout deadline is rejected, and a candidate still unused at
its retention deadline expires. A deadline lies at the admission tag plus the
timeout or retention, at microstep 0. An admission at a deadline's tag is seen
before that deadline fires.

Every sample ends with exactly one traced outcome. Anchors end as `MATCHED` or
`REJECTED` (reason `missing`, `timeout`, `anchor_evicted`, `candidate_evicted` and
so on), candidates as `SELECTED`, `SUPERSEDED`, `EXPIRED`, `LATE` or `OVERFLOW`.
When the node goes away, undecided anchors end as `REJECTED` with the close reason
(`shutdown`, `epoch` or `cancelled`) and open candidates as `FLUSHED`, and one
`close` record carries that reason and the tag.

## Under rcllf and under stock ROS

Under rcllf each input converts to its
[correspondence drain](missing-messages.md), and the policy admits a sample at the
drain's logical tag and correspondence key. A generated wake reaction applies the
deadlines at their own tags, including those no input carries, such as the timeout
of the last anchor. A missing sample on an input still fires the subscription's
registered `deadline` and `message_lost` handlers, each exactly once, and the
policy records a `missing` trace line for it. It does not move the watermark of that
input, so the waiting anchor is decided by the next sample or by its timeout.

So the bound that rcllf states is a bound in logical time. Each anchor is decided by
its logical deadline tag. How long that takes in wall-clock time is not bounded by
this feature.

Under stock ROS the same source runs with the steady clock as the tag and one timer
thread for the deadlines. All engine calls, timer included, run under one mutex.
The rules are the same, but which samples meet depends on arrival order, so a stock
run is not reproducible.

## Trace

Each record is one JSON line on stderr, prefixed `RCLLF_BNA`. The kinds are
`params`, `admit`, `missing`, `wake`, `decision`, `sample` and `close`. A sample key is
`<run epoch>:<writer>:<ordinal>`, the ordinal counting per writer. `lf_eval.bna` holds the standalone
oracle and the comparator, and `bna_replay` (in `rcllf_message_filters`) replays the
input records of a trace through the C++ engine.

## Federation

Under `--federate`, an input that comes from another federate is a cross-federate
correspondence edge and BNA works as in one process, under centralized and
decentralized coordination alike. Two differences follow from the link. Admissions
move to the link delivery tag, one microstep after the link delivers, while a
deadline stays at microstep 0, so an admission at a deadline's time can fall just
after the deadline. Under decentralized coordination a record that arrives after its
tag is reported lost, and samples sent in one burst share a microstep. The
[federation page](federation.md) describes the coordination modes and the
[budget check](federation.md#budget-check). The `bna_example` README lists the cases.

## Differences from ApproximateTime

1. One anchor input, not a symmetric pivot.
2. A hard window `delta`, bounds included.
3. Nearest to the anchor with a stated tie-break, not the smallest set span with an
   age penalty.
4. A decision waits until the watermarks prove it final, so latency grows by about
   `delta`, the lateness and one input period.
5. A timeout rejects the anchor. BNA never emits a partial or best-so-far set.
6. Evicting a sample for capacity rejects the anchors that could have used it.
7. Every input sample has one traced outcome.
8. Two inputs only.

## Example

`packages/src/bna_example` drives every outcome from a fixed candidate table, and
its README derives the expected outcome stream by hand, for one process and for
federation per node.

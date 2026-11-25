# bna_example

Two sensors fused by the `BoundedNearestToAnchor` (BNA) message_filters policy
of `rcllf_message_filters`. The candidate stamps follow a fixed
table, so every outcome is known before the system runs.

| Node | Publishes | Behaviour |
|---|---|---|
| `anchor_sensor` | `bna/anchor` | Ticks 0 to 11 every 100 ms, stamp `now()` |
| `candidate_sensor` | `bna/candidate` | Ticks 0 to 13, one sample per table entry, stamp `now()` plus the offset |
| `bna_fusion` | `bna/fused` | BNA over anchor (input 0) and candidate, logs `PAIR`, `MISSING` and `LOST` lines |

Policy parameters: delta 20 ms, lateness 0 ms (anchor) and 30 ms (candidate),
capacity 4 and 4, retention 350 ms, timeout 300 ms. Both inputs declare
`stamp_provenance: lf_clock` in `lf/bringup.execution.yaml`, because under
rcllf `now()` is the LF tag of the timer tick.

```bash
colcon lf --package bna_example
ros2 lf launch bna_example bringup 2>&1 | tee run.log
python3 -m lf_eval.bna.schedule run.log    # prints AGREE or the first difference
```

## Candidate table and hand-derived outcomes

Under rcllf a sample of tick k is admitted at k * 100 ms with the default corr
after-delay of 0. The corr action runs one microstep after the
publishing timer, so the j-th sample of one tick lands at microstep 1 + j, the
anchor before the candidate. Times below are in ms after the start. `A n` is anchor ordinal
n (tick n - 1), `C n` candidate ordinal n.

The rules used: an anchor is decided once both watermarks (largest stamp
minus the input's lateness) are past its stamp plus delta. It takes the
nearest candidate within delta, or is rejected as missing. A candidate is
superseded once no pending or later anchor can reach it. A deadline lies at
the admission time plus timeout (anchor) or retention (candidate), microstep
0. So a deadline at an admission's millisecond fires before the admission.

| Tick | Candidate offsets (ms) | Ordinals | What it shows |
|---|---|---|---|
| 0 | 0 | C1 | exact match |
| 1 | +20 | C2 | at the delta bound, matched |
| 2 | +21 | C3 | 1 ms past delta, anchor A3 has no partner |
| 3 | -10, +10 | C4, C5 | a tie, the earlier stamp wins |
| 4 | 0, -40 | C6, C7 | C7 is at or below the candidate watermark (400 - 30), late |
| 5, 6 | none | | anchor A5 waits |
| 7 | -290 | C8 | stamp 410, admitted just after A5's timeout deadline (700) |
| 8 to 11 | 0 | C9 to C12 | C9 is admitted just after A6's timeout deadline (800), then matches resume |
| 12 | none | | |
| 13 | 0 | C13 | a lone candidate after the last anchor, expires |

| Tag | Outcome | Why |
|---|---|---|
| (100, 1) | A1 MATCHED C1 | A2 (100) and C2 (120) lift both watermarks past 0 + 20 |
| (200, 1) | A2 MATCHED C2 | distance 20, inside delta |
| (300, 1) | A3 REJECTED(missing), C3 SUPERSEDED | C3 (221) is 21 from A3, and no later anchor can reach it |
| (400, 1) | A4 MATCHED C4, C5 SUPERSEDED | C4 (290) and C5 (310) are both 10 from A4, the earlier stamp wins |
| (400, 2) | C7 LATE | stamp 360 is not above the watermark 370 |
| (700, 0) | A5 REJECTED(timeout), C6 SUPERSEDED | the deadline fires before C8 is admitted at (700, 1) |
| (700, 1) | C8 SUPERSEDED | C8 (410) has no pending anchor left to reach |
| (800, 0) | A6 REJECTED(timeout) | its deadline fires before C9 is admitted at (800, 1) |
| (800, 1) | A7, A8 REJECTED(missing) | C9 (800) completes both, neither has a candidate within 20 ms |
| (900, 1), (1000, 1), (1100, 1) | A9, A10, A11 MATCHED C9, C10, C11 | |
| (1400, 0) | A12 REJECTED(timeout) | no anchor after A12, so the anchor watermark never passes 1120 |
| (1450, 0) | C12 EXPIRED | retention 350 ms after its admission at 1100 |
| (1650, 0) | C13 EXPIRED | retention 350 ms after its admission at 1300 |

The deadline outcomes at microstep 0 come from tags no input carries, so each
needs a wake reaction at that tag. `lf_eval.bna.schedule` repeats this table
(`HAND`) and checks that the oracle reproduces it.

At d = 0 no admission shares a tag with a deadline any more. The boundary
"admission at the deadline's own tag" that the 1 ms default used to exercise
(C9 completing A6 to A8 as missing) is now covered only by the oracle's unit
tests, not by this example.

With `RCLLF_TEST_CORR_DROP_SAMPLE=<candidate edge>:10` the binding drops C10.
The hook exists only in a binding built with `-DRCLLF_CORR_TEST_HOOKS=ON`; an
ordinary build logs that it is compiled out and drops nothing.
The candidate input sees a missing message at (900, 1), the `MISSING` and
`LOST` handlers each run once, and the outcomes change to A9 MATCHED C9 and
A10 REJECTED(missing), both at (1000, 1) (`HAND_DROP`).

With `max_stamp_lag: 0 ms` added to `/bna/candidate` in
`lf/bringup.execution.yaml` the loss report of C10 carries the stamp
bound 900 (its publish tag minus the lag). The candidate watermark rises to
900 - 30 = 870, past A9's 800 + 20, so A9 MATCHED C9 at the loss report (900, 1).
A10 (900) still needs a watermark above 920, which only C11's admission at
(1000, 1) gives (970), so A10 REJECTED(missing) stays at (1000, 1) (`HAND_RAISE`,
`--stamp-lag 0`).

## Federated

`colcon lf-generate --package bna_example --federate per-node` puts each node
in its own federate under centralized coordination. Both inputs are
cross-federate correspondence edges with the default link `after 0`, which
delivers one microstep after the publishing tag. The drain takes the link's port
as a trigger and drains at that delivery tag, so admissions are at k * 100 ms,
microstep 1 (1 + j for the j-th sample of a tick), as in one process, and the
outcomes are those of one process (`HAND_FEDERATED` is `HAND`). Check a federated
run with `python3 -m lf_eval.bna.schedule run.log --mode federated`.

With `--coordination decentralized` both inputs are admitted
cross-federate correspondence edges as well. Each link waits its STP offset (10 ms
by default) before the fusion federate processes a tag. A publisher's two-sample burst
reaches the receiver at one tag, so both samples share microstep 1 instead of taking 1
and 2: C4 and C5 (tick 3) are both admitted at (300 ms, 1), C6 and C7 (tick 4) at
(400 ms, 1). C4 and C5 are decided at 400 ms, so their outcomes do not move. C7 is LATE
at (400 ms, 1) rather than (400 ms, 2); every other outcome is the centralized one. Check such a run with `--mode decentralized` (the `--drop 10`
and `--stamp-lag 0` variants included).

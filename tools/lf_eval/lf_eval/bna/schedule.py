# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Expected BNA stream of ``packages/src/bna_example``.

The publish schedule below repeats ``include/bna_example/schedule.hpp`` and the
policy parameters of ``src/bna_fusion_node.cpp``. Under rcllf both sensors
stamp with ``now()``, the LF tag of their timer tick, and each corr drain runs
``D`` (0) after the tick, sample ``j`` of one tick at microstep
``1 + j`` in both layouts (:data:`MODES`). The anchor drain runs before the
candidate drain at the same tag. That gives the input
stream without running anything, the oracle gives the outputs, and
:data:`HAND` is the same outcome table derived by hand (README.md).

A real trace is normalised before the comparison: times relative to the start
(the stamp of anchor ordinal 1, tick 0), epoch 0, and each writer gid replaced
by its input's name. Wake events are not predicted (a wake may be requested for
a deadline that is no longer live), so they are left out of the input
comparison. Instead every outcome at a tag no input event carries must have a
wake event at that tag (fix 4).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

from lf_eval.bna import compare, corpus, oracle

MS = 1_000_000
PERIOD = 100 * MS
D = 0  # corr after-delay: the default ``after 0``
# Admission delay and first microstep per generation mode, both at the default
# d = 0. In one process the corr action of a tick is scheduled
# with delay 0 and runs one microstep after the publishing timer, so sample j
# of tick k is admitted at (k * 100 ms, 1 + j). Across federates the link
# ``after 0`` delivers the record at (k * 100 ms, 1) and the drain takes that
# port as a trigger and drains at the delivery tag (lf_generator.py
# _build_corr_sub_reactions), so the admissions are the same. An earlier
# model had the federated drain one microstep later, which holds only for the
# RCLLF_XFED_SPLIT_DRAIN=1 escape; the measured per-node federated runs (a890e782)
# admit at (k * 100 ms, 1) as one process does.
# Per-node under decentralized coordination the receiver drains a
# tick's records at one tag after its STP wait, so both samples of a two-sample
# tick are admitted at (k * 100 ms, 1) (10 identical runs, 2026-11-15).
MODES = {"single": (D, 1), "federated": (D, 1), "decentralized": (D, 1)}
# Modes whose samples of one tick share the first microstep instead of 1 + j.
SAME_TAG_BURST = {"decentralized"}
ANCHOR_TICKS = 12
CANDIDATE_OFFSETS_MS = [
    [0], [20], [21], [-10, 10], [0, -40], [], [], [-290], [0], [0], [0], [0], [], [0],
]
PARAMS = {"ev": "params", "sync": 0, "epoch": 0, "delta": 20 * MS, "lateness": [0, 30 * MS],
          "capacity": [4, 4], "retention": 350 * MS, "timeout": 300 * MS}
NAMES = ("anchor", "candidate")

# Hand-derived outcomes, (kind, key, outcome, tag ms, microstep), keys as
# A<anchor ordinal> and C<candidate ordinal>. See README.md for the reasoning.
# One process at d = 0: admissions at (k * 100 ms, 1 + j), deadlines at
# microstep 0. A deadline at an admission's millisecond therefore fires before
# the admission (it is the "just after the deadline tag" case the federated
# table at 5 ms used to show): A5 times out at (700, 0) before C8 comes in at
# (700, 1), and A6 times out at (800, 0) before C9 at (800, 1), so only A7 and
# A8 are rejected as missing there.
HAND = [
    ("D", "A1", "MATCHED C1", 100, 1), ("S", "C1", "SELECTED", 100, 1),
    ("D", "A2", "MATCHED C2", 200, 1), ("S", "C2", "SELECTED", 200, 1),
    ("D", "A3", "REJECTED(missing)", 300, 1), ("S", "C3", "SUPERSEDED", 300, 1),
    ("D", "A4", "MATCHED C4", 400, 1), ("S", "C4", "SELECTED", 400, 1),
    ("S", "C5", "SUPERSEDED", 400, 1),
    ("S", "C7", "LATE", 400, 2),
    ("D", "A5", "REJECTED(timeout)", 700, 0), ("S", "C6", "SUPERSEDED", 700, 0),
    ("S", "C8", "SUPERSEDED", 700, 1),
    ("D", "A6", "REJECTED(timeout)", 800, 0),
    ("D", "A7", "REJECTED(missing)", 800, 1), ("D", "A8", "REJECTED(missing)", 800, 1),
    ("D", "A9", "MATCHED C9", 900, 1), ("S", "C9", "SELECTED", 900, 1),
    ("D", "A10", "MATCHED C10", 1000, 1), ("S", "C10", "SELECTED", 1000, 1),
    ("D", "A11", "MATCHED C11", 1100, 1), ("S", "C11", "SELECTED", 1100, 1),
    ("D", "A12", "REJECTED(timeout)", 1400, 0),
    ("S", "C12", "EXPIRED", 1450, 0),
    ("S", "C13", "EXPIRED", 1650, 0),
]
# Dropping candidate ordinal 10 (tick 9) turns its admission into a missing
# event at (900 ms, 1). Anchor 9 then waits for candidate 11.
DROP = 10


def _drop(hand: list[tuple], step: int) -> list[tuple]:
    """*hand* with candidate DROP missing: A9 is decided by C11's admission at
    (1000 ms, *step*) instead of C10's, and A10 has no candidate left."""
    out = [r for r in hand if r[1] not in ("A9", "C9", "A10", "C10")]
    at = out.index(("D", "A11", "MATCHED C11", 1100, step))
    out[at:at] = [
        ("D", "A9", "MATCHED C9", 1000, step), ("S", "C9", "SELECTED", 1000, step),
        ("D", "A10", "REJECTED(missing)", 1000, step),
    ]
    return out


HAND_DROP = _drop(HAND, 1)
# With ``max_stamp_lag: 0 ms`` declared on the candidate input the
# loss report of C10 at (900 ms, 1) carries the stamp bound 900 ms (its publish
# tag minus the lag). The candidate watermark rises to 900 - 30 = 870 ms > A9's
# 800 + 20 ms, so A9 is decided with C9 at the loss report instead of at C11's
# admission. A10 (stamp 900 ms) needs a watermark above 920 ms, which only
# C11's admission at (1000 ms, 1) gives (970 ms), and is rejected as missing.
HAND_RAISE = [
    (k, x, o, 900, 1) if x in ("A9", "C9") else (k, x, o, ms, m)
    for k, x, o, ms, m in HAND_DROP
]
# Per-node federation at d = 0 admits at the same tags as one process
# (MODES), so its outcomes are the one-process table.
HAND_FEDERATED = HAND
HAND_FEDERATED_DROP = HAND_DROP
HAND_FEDERATED_RAISE = HAND_RAISE
# Decentralized per-node: C7, the second sample of tick 4 (stamp
# 360 ms, below the candidate watermark 370 ms that C6 set), is admitted with C6
# at (400, 1) and is LATE there instead of at (400, 2). Every other outcome is
# decided at a tag that does not depend on a burst sample's microstep. C10 is
# alone in its tick, so the drop and raise variants move only C7 as well.


def _c7_same_tag(rows):
    return [(k, x, o, ms, 1) if (x, o) == ("C7", "LATE") else (k, x, o, ms, m)
            for k, x, o, ms, m in rows]


HAND_DECENTRALIZED = _c7_same_tag(HAND)
HAND_DECENTRALIZED_DROP = _c7_same_tag(HAND_DROP)
HAND_DECENTRALIZED_RAISE = _c7_same_tag(HAND_RAISE)


def key(i: int, ordinal: int) -> str:
    return f"0:{NAMES[i]}:{ordinal}"


def short(k: str) -> str:
    _, gid, ordinal = oracle.split_key(k)
    return ("A" if gid == NAMES[0] else "C") + str(ordinal)


def inputs(drop: Optional[int] = None, mode: str = "single",
           stamp_lag: Optional[int] = None) -> list[dict]:
    """Expected params, admit and missing events, start time 0, no close.

    *stamp_lag* (ns) is the candidate input's declared ``max_stamp_lag``:
    the loss report then carries the publish tag minus it."""
    delay, step = MODES[mode]
    same = mode in SAME_TAG_BURST
    out = [dict(PARAMS)]
    ordinal = [0, 0]
    for tick in range(max(ANCHOR_TICKS, len(CANDIDATE_OFFSETS_MS))):
        t = tick * PERIOD + delay
        if tick < ANCHOR_TICKS:
            ordinal[0] += 1
            out.append({"ev": "admit", "sync": 0, "input": 0, "stamp": tick * PERIOD,
                        "key": key(0, ordinal[0]), "tag": [t, step]})
        offsets = CANDIDATE_OFFSETS_MS[tick] if tick < len(CANDIDATE_OFFSETS_MS) else []
        for j, off in enumerate(offsets):
            ordinal[1] += 1
            if ordinal[1] == drop:
                out.append({"ev": "missing", "sync": 0, "input": 1,
                            "key": key(1, ordinal[1]), "tag": [t, step + (0 if same else j)]})
                if stamp_lag is not None:
                    out[-1]["stamp"] = tick * PERIOD - stamp_lag
                continue
            out.append({"ev": "admit", "sync": 0, "input": 1, "stamp": tick * PERIOD + off * MS,
                        "key": key(1, ordinal[1]), "tag": [t, step + (0 if same else j)]})
    return out


def table(outputs: list[dict]) -> list[tuple]:
    """Outputs in the :data:`HAND` form."""
    rows = []
    for r in outputs:
        if r["ev"] == "decision":
            what = (f"MATCHED {short(r['partner'])}" if r["outcome"] == "MATCHED"
                    else f"REJECTED({r['reason']})")
            rows.append(("D", short(r["anchor"]), what, r["tag"][0] // MS, r["tag"][1]))
        else:
            rows.append(("S", short(r["key"]), compare.describe(r), r["tag"][0] // MS,
                         r["tag"][1]))
    return rows


def normalise(records: list[dict]) -> list[dict]:
    """Relative times, epoch 0, gids replaced by input names."""
    recs = [r for r in records if "ev" in r]
    gids: dict[str, str] = {}
    start = None
    for r in recs:
        if r["ev"] in ("admit", "missing"):
            _, gid, ordinal = oracle.split_key(r["key"])
            name = NAMES[int(r["input"])]
            if gids.setdefault(gid, name) != name:
                raise ValueError(f"writer {gid} feeds both inputs")
            if r["ev"] == "admit" and name == NAMES[0] and ordinal == 1:
                start = int(r["stamp"])
    if start is None:
        raise ValueError("no admission of anchor ordinal 1, cannot find the start time")
    if len(set(gids.values())) != len(gids):
        raise ValueError(f"more than one writer per input: {gids}")

    def k(text: Optional[str]) -> Optional[str]:
        if text is None:
            return None
        _, gid, ordinal = oracle.split_key(text)
        return f"0:{gids.get(gid, gid)}:{ordinal}"

    out = []
    for r in recs:
        r = dict(r)
        if "tag" in r:
            r["tag"] = [int(r["tag"][0]) - start, int(r["tag"][1])]
        if r["ev"] == "params":
            r["epoch"] = 0
        for f in ("key", "anchor", "partner"):
            if f in r:
                r[f] = k(r[f])
        for f in ("stamp", "partner_stamp"):
            if r.get(f) is not None and r["ev"] in ("admit", "missing", "decision", "sample"):
                r[f] = int(r[f]) - start
        out.append(r)
    return out


def check(records: list[dict], drop: Optional[int] = None,
          mode: str = "single", stamp_lag: Optional[int] = None) -> Optional[str]:
    """First difference between a real trace and the expected stream, or None."""
    actual = normalise(records)
    closes = [r for r in actual if r["ev"] == "close"]
    if len(closes) != 1:
        return f"close_count: expected 1 close record, trace has {len(closes)}"
    expected = inputs(drop, mode, stamp_lag) + [{"ev": "close", "sync": 0, "reason": "shutdown",
                                "tag": closes[0]["tag"]}]
    out = oracle.run(expected)
    if out and closes[0]["tag"] <= out[-1]["tag"]:
        return f"close_too_early: close at {closes[0]['tag']}, last outcome at {out[-1]['tag']}"
    no_wake = [r for r in actual if r["ev"] != "wake"]
    report = compare.compare(expected, out, no_wake, no_wake)
    if report:
        return report
    input_tags = {tuple(r["tag"]) for r in expected if r["ev"] != "params"}
    wakes = {tuple(r["tag"]) for r in actual if r["ev"] == "wake"}
    for r in out:
        t = tuple(r["tag"])
        if t not in input_tags and t not in wakes:
            return f"missing_wake: outcome {describe_row(r)} at {list(t)} has no wake event"
    return None


def describe_row(r: dict) -> str:
    k = r["anchor"] if r["ev"] == "decision" else r["key"]
    return f"{short(k)} {compare.describe(r)}"


def trace_records(text: str) -> list[dict]:
    """``RCLLF_BNA`` records of a run log. The copy the binding also logs
    through rclcpp with ``RCLLF_TRACE_CALLBACKS=1`` is skipped."""
    out = []
    for line in text.splitlines():
        i = line.find("RCLLF_BNA {")
        if i >= 0 and "ros2_lf_binding.trace" not in line:
            out.append(json.loads(line[i + len("RCLLF_BNA "):]))
    return out


def outcomes(records: list[dict]) -> str:
    """Normalised decision and sample records, one JSON line each. Two runs
    of the example agree when these texts are byte-identical."""
    return corpus.dumps(r for r in normalise(records) if r["ev"] in oracle.OUTPUT_EVENTS)


def replay_corpus(records: list[dict]) -> str:
    """Every input event of the trace (params, admit, missing, wake, close),
    unnormalised, as the ``bna_replay --corpus`` input (fix 7)."""
    return corpus.dumps(r for r in records if r.get("ev") in oracle.INPUT_EVENTS)


def replay_report(records: list[dict], replayed: list[dict]) -> Optional[str]:
    """First difference between the outputs of a run and ``bna_replay`` on that
    run's corpus, or None. The trace outputs stand in for the oracle."""
    trace_in = [r for r in records if r.get("ev") in oracle.INPUT_EVENTS]
    trace_out = [r for r in records if r.get("ev") in oracle.OUTPUT_EVENTS]
    got = [r for r in replayed if r.get("ev") in oracle.OUTPUT_EVENTS]
    return compare.compare(trace_in, trace_out, trace_in, got)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", type=Path, help="run log with RCLLF_BNA lines")
    ap.add_argument("--drop", type=int, default=None, help="dropped candidate ordinal")
    ap.add_argument("--mode", choices=sorted(MODES), default="single",
                    help="generation mode of the run (admission delay and microsteps)")
    ap.add_argument("--stamp-lag", type=int, default=None,
                    help="declared max_stamp_lag (ns) of the candidate input")
    ap.add_argument("--outcomes", type=Path, help="write the normalised outcome stream here")
    ap.add_argument("--corpus", type=Path, help="write the replay corpus here")
    ap.add_argument("--replayed", type=Path,
                    help="bna_replay output for this run's corpus, compared to the trace")
    a = ap.parse_args(argv)
    records = trace_records(a.log.read_text(encoding="utf-8", errors="replace"))
    report = check(records, a.drop, a.mode, a.stamp_lag)
    if a.outcomes:
        a.outcomes.write_text(outcomes(records), encoding="utf-8")
    if a.corpus:
        a.corpus.write_text(replay_corpus(records), encoding="utf-8")
    if not report and a.replayed:
        replay = replay_report(records, corpus.read_jsonl(a.replayed))
        report = replay and f"replay: {replay}"
    print(report or "AGREE")
    return 1 if report else 0


if __name__ == "__main__":
    sys.exit(main())

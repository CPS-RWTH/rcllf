# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""BoundedNearestToAnchor on the generated bna_example system.

Expected values come from the example source through
``lf_eval.bna.schedule`` (publish table, 100 ms ticks, the default corr
after-delay 0) and its hand table, never from a previous run.

The schedule, the hand tables and the bna_example README assume d = 0
(admissions at (k * 100 ms, 1 + j)).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools" / "lf_eval"))
from lf_eval.bna import schedule  # noqa: E402

_EXPIRED = re.compile(r'RCLLF_BNA \{"ev":"sample".*"outcome":"EXPIRED"')
_PAIR = re.compile(r"PAIR anchor_tick=(\d+) candidate_tick=(\d+) offset_ns=(-?\d+)")
_HOOK = re.compile(r"\[bna_fusion\]: (MISSING|LOST) input=(\w+) total=(\d+) change=(\d+)")


def _run(monkeypatch, **env) -> rts.LaunchResult:
    case = rts.require_case("bna")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    # Both lone EXPIRED outcomes (1401 ms and 1601 ms) close the schedule.
    return rts.launch_and_capture(
        case, 1.5, until=lambda log: len(_EXPIRED.findall(log)) >= 2, settle_s=0.5)


def _pairs(hand) -> list[tuple[int, int]]:
    """(anchor tick, candidate tick) of each MATCHED row of a hand table."""
    cand_tick = {}
    n = 0
    for tick, offsets in enumerate(schedule.CANDIDATE_OFFSETS_MS):
        for _ in offsets:
            n += 1
            cand_tick[f"C{n}"] = tick
    return [(int(k[1:]) - 1, cand_tick[w.split()[1]]) for d, k, w, _, _ in hand
            if d == "D" and w.startswith("MATCHED")]


def _edge(log: str, topic: str) -> int:
    found = re.findall(rf"rcllf corr: admitted edge (\d+) \S+ -> \S+ on '{topic}'", log)
    assert len(found) == 1, f"expected one admitted edge on {topic}: {found}"
    return int(found[0])


def test_bna_example_agrees_with_the_hand_table(monkeypatch):
    result = _run(monkeypatch)
    log = result.log
    assert result.finished_cleanly, "no clean shutdown:\n" + log[-3000:]
    assert "RCLLF_BNA defect" not in log and "RCLLF_BNA refused" not in log, log[-3000:]
    report = schedule.check(schedule.trace_records(log))
    assert report is None, report
    got = [(int(a), int(c)) for a, c, _ in _PAIR.findall(log)]
    assert got == _pairs(schedule.HAND), f"PAIR lines {got}"
    assert _HOOK.findall(log) == [], "a deadline or message_lost handler ran"


def test_missing_candidate_runs_both_handlers_once_and_reaches_bna(monkeypatch):
    """Fix 6: a missing join on a BNA input still runs the registered
    deadline and message_lost handlers exactly once, then the BNA missing hook."""
    probe = _run(monkeypatch)
    edge = _edge(probe.log, "/bna/candidate")
    result = _run(monkeypatch, RCLLF_TEST_CORR_DROP_SAMPLE=f"{edge}:{schedule.DROP}",
                  ROS2_LF_TAKE_TIMEOUT_MS="100")
    log = result.log
    assert result.finished_cleanly, "no clean shutdown:\n" + log[-3000:]
    hooks = [(k, i, int(t), int(c)) for k, i, t, c in _HOOK.findall(log)]
    assert hooks == [("MISSING", "candidate", 1, 1), ("LOST", "candidate", 1, 1)], hooks
    report = schedule.check(schedule.trace_records(log), drop=schedule.DROP)
    assert report is None, report
    got = [(int(a), int(c)) for a, c, _ in _PAIR.findall(log)]
    assert got == _pairs(schedule.HAND_DROP), f"PAIR lines {got}"

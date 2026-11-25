# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""fusion_example, example 4 of the learning path, as an rcllf program.

Sensor A publishes every 30 ms and sensor B every 50 ms, each message stamped
with ``now()`` and carrying its sequence number. Under rcllf ``now()`` is the
logical time, so the stamps lie exactly on each timer's period grid and the A
and B grids meet every 150 ms (their least common multiple). Expected values,
derived from the source: the ExactTime synchronizer pairs exactly those
coinciding stamps, so consecutive exact pairs are 150 ms apart and advance
A's sequence by 5 and B's by 3, with no gap; the ApproximateTime synchronizer
pairs each message at most once, so both sequences rise strictly, and its two
stamps lie at most one B period (50 ms) apart.

Selected in CI by ``-k fusion_example`` (runtime-smoke group fusion-example of
the public CI); the test names avoid every
other group's ``-k`` words.
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

_RUN_SECS = 6.0
_PERIOD_A_NS = 30_000_000
_PERIOD_B_NS = 50_000_000
_LCM_NS = 150_000_000
# 6 s give about 40 exact pairs; 10 leave room for the startup and the stop.
_MIN_EXACT = 10
_MIN_APPROX = 20

_PAIR = re.compile(
    r"PAIR (exact|approx) a_seq=(\d+) a_ns=(-?\d+) b_seq=(\d+) b_ns=(-?\d+)")


@pytest.fixture(scope="module")
def fusion_run():
    case = rts.require_case("fusion")
    result = rts.launch_and_capture(case, _RUN_SECS)
    pairs = {"exact": [], "approx": []}
    for kind, a_seq, a_ns, b_seq, b_ns in _PAIR.findall(result.log):
        pairs[kind].append((int(a_seq), int(a_ns), int(b_seq), int(b_ns)))
    return result, pairs


def test_fusion_example_exact_pairs_on_the_common_grid(fusion_run):
    result, pairs = fusion_run
    exact = pairs["exact"]
    tail = f"\n--- log tail ---\n{result.log[-1500:]}"
    assert len(exact) >= _MIN_EXACT, f"only {len(exact)} exact pair(s){tail}"
    for a_seq, a_ns, b_seq, b_ns in exact:
        assert a_ns == b_ns, f"exact pair with different stamps a={a_seq} b={b_seq}{tail}"
    steps = [(a2 - a1, an2 - an1, b2 - b1)
             for (a1, an1, b1, _), (a2, an2, b2, _) in zip(exact, exact[1:])]
    want = (_LCM_NS // _PERIOD_A_NS, _LCM_NS, _LCM_NS // _PERIOD_B_NS)
    bad = [(i, s) for i, s in enumerate(steps) if s != want]
    assert not bad, (f"exact pairs off the 150 ms grid (step index, (d a_seq, d ns, d b_seq)), "
                     f"want {want}: {bad[:5]}{tail}")
    assert not result.process_died, f"process had to be SIGKILLed{tail}"
    assert result.finished_cleanly, f"no 'process has finished cleanly' after SIGINT{tail}"


def test_fusion_example_approx_pairs_rise_and_stay_within_one_period(fusion_run):
    result, pairs = fusion_run
    approx = pairs["approx"]
    tail = f"\n--- log tail ---\n{result.log[-1500:]}"
    assert len(approx) >= _MIN_APPROX, f"only {len(approx)} approx pair(s){tail}"
    for (a1, _, b1, _), (a2, _, b2, _) in zip(approx, approx[1:]):
        assert a2 > a1 and b2 > b1, f"approx pairs do not rise: ({a1},{b1}) -> ({a2},{b2}){tail}"
    for a_seq, a_ns, b_seq, b_ns in approx:
        assert abs(a_ns - b_ns) <= _PERIOD_B_NS, (
            f"approx pair a={a_seq} b={b_seq} stamps {abs(a_ns - b_ns)} ns apart{tail}")
    # The stamps sit on each sensor's own grid: a fixed offset plus seq periods.
    a_off = {a_ns - a_seq * _PERIOD_A_NS for a_seq, a_ns, _, _ in approx}
    b_off = {b_ns - b_seq * _PERIOD_B_NS for _, _, b_seq, b_ns in approx}
    assert len(a_off) == 1 and len(b_off) == 1, (
        f"stamps off the period grid: A offsets {sorted(a_off)[:3]}, "
        f"B offsets {sorted(b_off)[:3]}{tail}")

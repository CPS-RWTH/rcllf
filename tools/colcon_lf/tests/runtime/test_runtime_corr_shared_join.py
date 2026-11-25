# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A join of one reactor pushes another reactor's publish past its cutoff.

``correspondence_fixture/corr_shared_join.launch.py`` with its package
execution.yaml and CORR_SCENARIO=two_publishers. corr_publisher's 200 ms timer
publishes ``a<N>`` on /corr/a and ``b<N>`` on /corr/b for ticks 1 to 10.
corr_sub_2 reads /corr/a with take_timeout_ms 150. corr_fanin_relay reads
/corr/a and /corr/b (20 ms each) and republishes ``j-b<N>`` on /corr/j, which
corr_relay_sink reads with 20 ms.

All drains of /corr/a and /corr/b run at the release tag (t - 1) * 200 ms + d.
The fan-in relay's /corr/b drain comes after its /corr/a drain in its reactor,
so it sits at a higher LF level than corr_sub_2's drain. Every edge here is
local and decided when its publish returns: no edge has a cutoff
(``cutoff_elapsed`` -1), a dropped sample is LOST at once, nothing holds the
tag, and ``j-b2`` is MATCHED. ``join: deferred`` is a generation
error on a local edge.

Expected values come from the fixture source and the default ``after 0``
(d = 0), derived by hand; the trace elapsed carries no microstep.
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts
from .test_runtime_correspondence import D_NS, TICK_NS, _edges, _events, _reconcile

pytestmark = pytest.mark.runtime

TICKS = list(range(1, 11))


def _run(monkeypatch, **env) -> str:
    case = rts.require_case("corr_shared_join")
    monkeypatch.setenv("CORR_SCENARIO", "two_publishers")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    result = rts.launch_and_capture(
        case, 1.0, until=lambda log: "corr_relay_sink: payload=j-b10" in log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    return result.log


def _all_edges(log: str) -> dict[tuple[str, str], int]:
    edges = _edges(log)
    want = {("CorrSubscriberA", "/corr/a"), ("CorrFaninRelay", "/corr/a"),
            ("CorrFaninRelay", "/corr/b"), ("CorrRelaySink", "/corr/j")}
    assert set(edges) == want, f"expected the four edges admitted, got {edges}"
    return edges


def _tags(dispatch: list[dict], edge: int) -> list[tuple[int, int, str]]:
    return [(e["ordinal"], e["elapsed"], e["outcome"]) for e in dispatch if e["edge"] == edge]


def _sink_payloads(log: str) -> list[str]:
    return re.findall(r"corr_relay_sink: payload=(\S+)", log)


def _check(log: str, *, dropped: bool) -> None:
    edges = _all_edges(log)
    x = edges[("CorrSubscriberA", "/corr/a")]
    j = edges[("CorrRelaySink", "/corr/j")]
    assert re.search(r"admitted edge \d+ CorrFaninRelay -> CorrRelaySink on '/corr/j' \(d = 0 ns, "
                     r"T = 20000000 ns", log), "/corr/j's T is not its own 20 ms (a chained T is back)"
    dispatch = _events(log, "dispatch")
    lost = {2} if dropped else set()
    want_x = [(t, (t - 1) * TICK_NS + D_NS, "LOST" if t in lost else "MATCHED")
              for t in TICKS]
    assert _tags(dispatch, x) == want_x, f"edge {x} /corr/a to corr_sub_2: {_tags(dispatch, x)}"
    for key in (("CorrFaninRelay", "/corr/a"), ("CorrFaninRelay", "/corr/b")):
        e = edges[key]
        assert _tags(dispatch, e) == [(t, (t - 1) * TICK_NS + D_NS, "MATCHED") for t in TICKS], (
            f"edge {e} {key}: {_tags(dispatch, e)}")
    # Every edge is local and decided at publish, so none has a
    # cutoff (kNoCutoff, traced as -1).
    cuts = {(e["edge"], e["ordinal"]): e["cutoff_elapsed"] for e in dispatch}
    assert cuts and set(cuts.values()) == {-1}, f"a local edge has a cutoff: {cuts}"
    # No /corr/j sample is missing, also not the one published after the hold:
    # the hold drops nothing.
    assert _tags(dispatch, j) == [(t, (t - 1) * TICK_NS + 2 * D_NS, "MATCHED") for t in TICKS], (
        f"edge {j} /corr/j: {_tags(dispatch, j)}")
    heard = _sink_payloads(log)
    assert heard == [f"j-b{t}" for t in TICKS], f"sink heard {heard}"


def test_shared_join_without_loss_matches(monkeypatch):
    log = _run(monkeypatch)
    _check(log, dropped=False)
    rec = _reconcile(log)
    assert rec["reconciled"] is True, rec


def test_lost_sample_is_lost_at_publish_and_the_other_sample_matches(monkeypatch):
    log = _run(monkeypatch, RCLLF_TEST_CORR_DROP_SAMPLE="2")
    edges = _all_edges(log)
    assert edges[("CorrSubscriberA", "/corr/a")] == 0, (
        f"the drop hook acts on edge 0, which must be corr_sub_2's /corr/a, got {edges}")
    _check(log, dropped=True)

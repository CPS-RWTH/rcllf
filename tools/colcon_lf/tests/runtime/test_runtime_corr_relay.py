# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Analyzer-proven burst bound and drain-reaction effects (W3 slice S3).

``correspondence_fixture/corr_relay.launch.py``, built WITHOUT an exec-config:
corr_publisher_one's 200 ms timer publishes ``t<N>k1`` on /corr/a for ticks
1 to 10, corr_relay republishes each as ``r-t<N>k1`` on /corr/r, and its
deadline handler publishes ``missing<total>`` on /corr/r. Every publishing
callback has one publish call site outside any loop, so both edges are
admitted with the proven k = 1. Expected values come from the fixture source
and the default ``after 0`` on each edge: both
/corr/a and the relay's /corr/r land at the publication tag (t, 0). Since
both edges are local and decided at publish: a dropped sample is
LOST at once and no edge has a cutoff (``cutoff_elapsed`` -1).
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts
from .test_runtime_correspondence import D_NS, HOP_MICRO, TICK_NS, _edges, _events, _reconcile

pytestmark = pytest.mark.runtime

TICKS = list(range(1, 11))


def _run(monkeypatch, **env) -> str:
    case = rts.require_case("corr_relay")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    result = rts.launch_and_capture(
        case, 1.0, until=lambda log: "corr_relay_sink: payload=r-t10k1" in log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    return result.log


def _chain(log: str) -> tuple[int, int]:
    edges = _edges(log)
    assert set(edges) == {("CorrRelay", "/corr/a"), ("CorrRelaySink", "/corr/r")}, (
        f"expected both relay edges admitted without an exec-config, got {edges}")
    return edges[("CorrRelay", "/corr/a")], edges[("CorrRelaySink", "/corr/r")]


def _sink_payloads(log: str) -> list[str]:
    return re.findall(r"corr_relay_sink: payload=(\S+)", log)


def _tags(dispatch: list[dict], edge: int) -> list[tuple[int, int, int, str]]:
    return [(e["ordinal"], e["elapsed"], e["microstep"], e["outcome"])
            for e in dispatch if e["edge"] == edge]


def test_relay_admits_with_proven_k_and_reconciles(monkeypatch):
    log = _run(monkeypatch)
    a, r = _chain(log)
    dispatch = _events(log, "dispatch")
    # /corr/a: tick t at (t - 1) * 200 ms, dispatched d later. /corr/r: the
    # relay publishes in that dispatch, so one more d.
    assert _tags(dispatch, a) == [(t, (t - 1) * TICK_NS + D_NS, HOP_MICRO, "MATCHED") for t in TICKS], (
        f"edge {a} /corr/a: {_tags(dispatch, a)}")
    assert _tags(dispatch, r) == [(t, (t - 1) * TICK_NS + 2 * D_NS, 2 * HOP_MICRO, "MATCHED") for t in TICKS], (
        f"edge {r} /corr/r: {_tags(dispatch, r)}")
    heard = _sink_payloads(log)
    assert heard == [f"r-t{t}k1" for t in TICKS], f"sink heard {heard}"
    rec = _reconcile(log)
    assert rec["reconciled"] is True and rec["retained"] == 0 and rec["carrier"] == 0, rec
    assert "undeclared_effect" not in log.lower(), "an undeclared effect was classified"


def test_deadline_handler_publish_is_an_admitted_effect_of_the_drain(monkeypatch):
    log = _run(monkeypatch, RCLLF_TEST_CORR_DROP_SAMPLE="2")
    a, r = _chain(log)
    assert a == 0, f"the drop hook acts on edge 0, which must be /corr/a, got {a}"
    dispatch = _events(log, "dispatch")
    bad = [(o, el, ms, out) for o, el, ms, out in _tags(dispatch, a) if out != "MATCHED"]
    assert bad == [(2, TICK_NS + D_NS, HOP_MICRO, "LOST")], f"edge {a} /corr/a non-MATCHED: {bad}"
    hooks = [(e["edge"], e["ordinal"], e["total"]) for e in _events(log, "missing_hook")]
    assert hooks == [(a, 2, 1)], f"missing_hook events {hooks}"
    # The handler's publish in the drain of tick 2 is /corr/r ordinal 2, an
    # admitted effect: MATCHED one d after the drain, like every relay.
    assert _tags(dispatch, r) == [(t, (t - 1) * TICK_NS + 2 * D_NS, 2 * HOP_MICRO, "MATCHED") for t in TICKS], (
        f"edge {r} /corr/r: {_tags(dispatch, r)}")
    heard = _sink_payloads(log)
    assert heard == [("missing1" if t == 2 else f"r-t{t}k1") for t in TICKS], f"sink heard {heard}"


# T on each edge (registry default, no exec-config, never chained since the causal windows)
# and the relay's work before each publish (fixture env CORR_RELAY_WORK_US).
T_NS = 100_000_000
WORK_US = 5000


def test_slow_deadline_handler_publish_keeps_its_downstream_slack(monkeypatch):
    # The handler publishes /corr/r ordinal 2 after WORK_US of work. Until
    # a cutoff fixed from the tag could end that publish NOT_READY
    # or TIMED_OUT though the sink heard it. Both
    # edges are local and decided at publish, with no cutoff at all
    # (cutoff_elapsed -1), so the slow publish is MATCHED by construction.
    log = _run(monkeypatch, RCLLF_TEST_CORR_DROP_SAMPLE="2", CORR_RELAY_WORK_US=str(WORK_US))
    a, r = _chain(log)
    assert re.search(r"admitted edge \d+ CorrRelay -> CorrRelaySink on '/corr/r' \(d = 0 ns, T = "
                     + str(T_NS) + " ns", log), "/corr/r's T is not its own 100 ms (a chained T is back)"
    dispatch = _events(log, "dispatch")
    bad = [(o, el, ms, out) for o, el, ms, out in _tags(dispatch, a) if out != "MATCHED"]
    assert bad == [(2, TICK_NS + D_NS, HOP_MICRO, "LOST")], f"edge {a} /corr/a non-MATCHED: {bad}"
    assert _tags(dispatch, r) == [(t, (t - 1) * TICK_NS + 2 * D_NS, 2 * HOP_MICRO, "MATCHED") for t in TICKS], (
        f"edge {r} /corr/r: {_tags(dispatch, r)}")
    cut = {(e["edge"], e["ordinal"]): e["cutoff_elapsed"] for e in dispatch}
    assert cut and set(cut.values()) == {-1}, f"a local edge has a cutoff: {cut}"
    heard = _sink_payloads(log)
    assert heard == [("missing1" if t == 2 else f"r-t{t}k1") for t in TICKS], f"sink heard {heard}"
    rec = _reconcile(log)
    assert rec["reconciled"] is True and rec["retained"] == 0 and rec["carrier"] == 0, rec
    # The deliberate drop on edge a is counted beside the accounting verdict.
    assert rec["missing_total"] == 1, rec

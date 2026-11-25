# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Same-tag delivery on zero-delay links, one node versus a chain.

``same_tag_fixture`` (see its README), one process, every connection d = 0 and
no execution config. ``tick`` (200 ms timer, ticks 1 to 10) publishes ``/a``,
``/c`` and ``/d`` once and ``/m`` three times per tick. ONE (``st_one``) runs
``/a -> relay -> /c``, CHAIN (``st_chain``) the same computation split in
``/a -> pre -> /b -> relay -> /c``. ``sink`` subscribes ``/c`` (a fan-in group
of ``relay``, rank 0, and ``tick``, rank 1) and ``/d``; ``msink`` subscribes
``/m``.

An LF connection without ``after`` delivers at the publication tag, and a
reaction fires once per tag, so the expected delivery tags follow from the
fixture source alone. With tick t at T = (t - 1) * 200 ms:

* every stage input (``/a``, ``/b``) at (T, 0), so ``relay`` publishes ``/c``
  at (T, 0) in ONE and in CHAIN alike;
* ``sink`` ``/c``: ``relay``'s record (rank 0), then ``tick``'s (rank 1),
  both at (T, 0), in one drain firing;
* ``sink`` ``/d`` at (T, 0);
* ``msink`` ``/m``: the three records of a tick all at (T, 0), in ordinal order.

The general rule (the rule: every record due at a tag is
delivered at that tag) checked against the ``emit`` trace (publication tags)
is: the records a subscription receives from one logical time are all
delivered at one tag, the earliest publication tag among them, in rank then
ordinal order (the dispatch order of the trace).
Evidence is the ``corr`` JSON trace (``RCLLF_TRACE_CALLBACKS=1``) and the
sink's value log of real runs.
"""

from __future__ import annotations

import json
import os
import re
from collections import defaultdict
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

TICK_NS = 200_000_000
TICKS = list(range(1, 11))

# Publisher class -> role. The relay role is StStage in ONE and StStageOut in
# CHAIN: the same position in the graph, the same LF instance name "relay".
ROLE = {
    "StSource": "tick",
    "StStage": "relay",
    "StStageIn": "pre",
    "StStageOut": "relay",
}

DONE_NEEDLES = ("st_sink: topic=/c value=37", "st_sink: topic=/c value=2010",
                "st_sink: topic=/d value=1010", "st_msink: topic=/m value=3102")


# ---------------------------------------------------------------------------
# Log parsing (pure, so a recorded log can be checked without a run)
# ---------------------------------------------------------------------------

def corr_events(log: str, kind: str) -> list[dict]:
    out = []
    for m in re.findall(r'(\{"event":"corr".*\})', log):
        ev = json.loads(m)
        if ev.get("kind") == kind:
            out.append(ev)
    return out


def admitted_edges(log: str) -> dict[int, tuple[str, str, str]]:
    """edge id -> (publisher class, subscriber class, topic), from the startup lines."""
    return {int(e): (pub, sub, topic) for e, pub, sub, topic in re.findall(
        r"rcllf corr: admitted edge (\d+) (\S+) -> (\S+) on '([^']+)'", log)}


def edge_ranks(log: str) -> dict[int, int]:
    """edge id -> fan-in rank; an edge outside a group has rank 0."""
    return {int(e): int(r) for e, r in re.findall(
        r"rcllf corr: edge (\d+) has rank (\d+) in the fan-in group", log)}


def deliveries(log: str) -> list[dict]:
    """Every dispatch of an admitted edge, with its edge's publisher, subscriber and topic."""
    edges = admitted_edges(log)
    ranks = edge_ranks(log)
    out = []
    for ev in corr_events(log, "dispatch"):
        pub, sub, topic = edges[ev["edge"]]
        out.append({"rank": ranks.get(ev["edge"], 0), "pub": pub, "role": ROLE.get(pub, pub), "sub": sub, "topic": topic,
                    "ordinal": ev["ordinal"], "tag": (ev["elapsed"], ev["microstep"]),
                    "outcome": ev["outcome"], "edge": ev["edge"]})
    return out


def publication_tags(log: str) -> dict[tuple[int, int], tuple[int, int]]:
    """(edge, ordinal) -> publication tag, from the ``emit`` events."""
    out = {}
    for ev in corr_events(log, "emit"):
        for tok in ev.get("tokens", []):
            out.setdefault((ev["edge"], tok["ordinal"]), (ev["elapsed"], ev["microstep"]))
    return out


def sink_view(log: str) -> list[tuple]:
    """What ``sink`` saw, in delivery (dispatch trace) order: (tag, topic, role, ordinal, outcome)."""
    return [(d["tag"], d["topic"], d["role"], d["ordinal"], d["outcome"])
            for d in deliveries(log) if d["sub"] == "StSink"]


def sink_values(log: str) -> dict[str, list[int]]:
    """Values ``sink`` logged per topic, in log order."""
    out: dict[str, list[int]] = defaultdict(list)
    for topic, value in re.findall(r"st_sink: topic=(\S+) value=(-?\d+)", log):
        out[topic].append(int(value))
    return dict(out)


def fmt(tag: tuple[int, int]) -> str:
    return f"(t={tag[0]} ns, m={tag[1]})"


def same_tag_violations(log: str) -> list[str]:
    """Deliveries that break the same-tag rule of a zero-delay connection.

    Per subscription (subscriber class and topic) and logical time, every
    record is delivered at (t, m0), m0 the earliest publication microstep
    among them: no record moves to a later microstep. In dispatch order the
    records run in (rank, ordinal) order.
    """
    pubs = publication_tags(log)
    groups: dict[tuple[str, str, int], list[dict]] = defaultdict(list)
    for d in deliveries(log):
        groups[(d["sub"], d["topic"], d["tag"][0])].append(d)
    errors = []
    for (sub, topic, time_ns), recs in sorted(groups.items()):
        published = [pubs.get((d["edge"], d["ordinal"])) for d in recs]
        if None in published:
            errors.append(f"{sub} {topic} at t={time_ns} ns: a delivered record has no emit event")
            continue
        m0 = min(p[1] for p in published if p[0] == time_ns) if any(
            p[0] == time_ns for p in published) else None
        if m0 is None:
            errors.append(f"{sub} {topic} at t={time_ns} ns: no record published at this time "
                          f"(published {published}), a zero-delay link moved the time")
            continue
        for i, (d, p) in enumerate(zip(recs, published)):
            want = (time_ns, m0)
            if i > 0 and (d["rank"], d["ordinal"]) < (recs[i - 1]["rank"], recs[i - 1]["ordinal"]):
                errors.append(
                    f"{sub} {topic}: record {i + 1} of t={time_ns} ns ({d['role']} rank "
                    f"{d['rank']} ordinal {d['ordinal']}) dispatched after rank "
                    f"{recs[i - 1]['rank']} ordinal {recs[i - 1]['ordinal']}, expected rank "
                    "then ordinal order")
            if d["tag"] != want:
                which = "first record" if i == 0 else f"record {i + 1}"
                errors.append(
                    f"{sub} {topic}: {which} of t={time_ns} ns ({d['role']} ordinal "
                    f"{d['ordinal']}, published at {fmt(p)}) delivered at {fmt(d['tag'])}, "
                    f"expected {fmt(want)}")
    return errors


def expected_sink_view() -> list[tuple]:
    rows = []
    for t in TICKS:
        time_ns = (t - 1) * TICK_NS
        rows += [((time_ns, 0), "/c", "relay", t, "MATCHED"),
                 ((time_ns, 0), "/c", "tick", t, "MATCHED"),
                 ((time_ns, 0), "/d", "tick", t, "MATCHED")]
    return rows


def expected_view(log: str, sub: str, topic: str) -> tuple[list, list]:
    """(got, want) delivery tags of one subscription, from the fixture source."""
    got = sorted((d["tag"], d["role"], d["ordinal"], d["outcome"])
                 for d in deliveries(log) if d["sub"] == sub and d["topic"] == topic)
    want = []
    for t in TICKS:
        time_ns = (t - 1) * TICK_NS
        if topic == "/m":
            want += [((time_ns, 0), "tick", 3 * (t - 1) + k + 1, "MATCHED") for k in range(3)]
        else:
            role = "pre" if topic == "/b" else "tick"
            want.append(((time_ns, 0), role, t, "MATCHED"))
    return got, sorted(want)


# ---------------------------------------------------------------------------
# Runs (one launch per variant per session)
# ---------------------------------------------------------------------------

_LOGS: dict[str, str] = {}


def run_case(case_key: str) -> str:
    case = rts.require_case(case_key)
    if case_key not in _LOGS:
        with pytest.MonkeyPatch.context() as mp:
            mp.setenv("RCLLF_TRACE_CALLBACKS", "1")
            result = rts.launch_and_capture(
                case, 1.0, until=lambda log: all(n in log for n in DONE_NEEDLES))
        log_dir = os.environ.get("RCLLF_SAME_TAG_LOG_DIR")
        if log_dir:  # keep the raw run as evidence
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            (Path(log_dir) / f"{case_key}.log").write_text(result.log, encoding="utf-8")
        assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
        _LOGS[case_key] = result.log
    return _LOGS[case_key]


def check_variant(log: str, stage_edges: set[tuple[str, str, str]]) -> None:
    """Assert one variant's deliveries; *stage_edges* are its (pub, sub, topic) stage links."""
    got_edges = set(admitted_edges(log).values())
    want_edges = {("StSource", "StSink", "/c"), ("StSource", "StSink", "/d"),
                  ("StSource", "StMultiSink", "/m")} | stage_edges
    assert got_edges == want_edges, (
        f"admitted edges {sorted(got_edges)}, expected {sorted(want_edges)} (all d = 0, none legacy)")
    assert re.findall(r"admitted edge \d+ \S+ -> \S+ on '[^']+' \(d = (\d+) ns", log), (
        "no admitted edge line carries its delay")
    delays = set(re.findall(r"admitted edge \d+ \S+ -> \S+ on '[^']+' \(d = (\d+) ns", log))
    assert delays == {"0"}, f"every connection must be d = 0, got d in {delays} ns"

    errors = same_tag_violations(log)
    # The first two violations of every subscription, so one noisy
    # subscription does not hide the others.
    shown: dict[str, list[str]] = defaultdict(list)
    for e in errors:
        key = " ".join(e.split()[:2])
        if len(shown[key]) < 2:
            shown[key].append(e)
    assert not errors, (f"same-tag delivery broken on a d = 0 link ({len(errors)} deliveries):\n  "
                        + "\n  ".join(e for v in shown.values() for e in v))

    for sub, topic in [("StSink", "/d"), ("StMultiSink", "/m")] + [
            (sub, topic) for _, sub, topic in stage_edges if sub != "StSink"]:
        got, want = expected_view(log, sub, topic)
        assert got == want, f"{sub} {topic}: delivered {got[:4]}..., expected {want[:4]}..."
    # /c (FANIN: relay rank 0 then tick rank 1) and /d share the tag. The
    # order between the two subscriptions is LF's reaction order, not this
    # rule, so it is compared per topic in dispatch order.
    view = sink_view(log)
    assert [r for r in view if r[1] == "/c"] == [r for r in expected_sink_view() if r[1] == "/c"], (
        f"sink /c deliveries {[r for r in view if r[1] == '/c'][:6]}..., expected relay (rank 0) "
        "then tick (rank 1), both at the publication tag (T, 0)")
    assert sorted(view) == sorted(expected_sink_view()), (
        f"sink deliveries {sink_view(log)[:6]}..., expected {expected_sink_view()[:6]}...")
    values = sink_values(log)
    assert values.get("/c") == [v for t in TICKS for v in (3 * t + 7, 2000 + t)], (
        f"sink /c values {values.get('/c')} (relay 3t+7 before tick 2000+t, rank order)")
    assert values.get("/d") == [1000 + t for t in TICKS], f"sink /d values {values.get('/d')}"
    heard_m = [int(v) for v in re.findall(r"st_msink: topic=/m value=(\d+)", log)]
    assert heard_m == [3000 + 10 * t + k for t in TICKS for k in range(3)], f"msink /m {heard_m}"


def test_same_tag_one_delivers_at_the_publication_tag():
    check_variant(run_case("same_tag_one"), {("StSource", "StStage", "/a"), ("StStage", "StSink", "/c")})


def test_same_tag_chain_delivers_at_the_publication_tag():
    check_variant(run_case("same_tag_chain"), {
        ("StSource", "StStageIn", "/a"), ("StStageIn", "StStageOut", "/b"),
        ("StStageOut", "StSink", "/c")})


def test_same_tag_modularity_one_and_chain_give_the_sink_identical_tags():
    one, chain = run_case("same_tag_one"), run_case("same_tag_chain")
    v_one, v_chain = sink_view(one), sink_view(chain)
    diff = [(a, b) for a, b in zip(v_one, v_chain) if a != b]
    assert v_one == v_chain, (
        "sink sees different deliveries in ONE and CHAIN (first differences, ONE vs CHAIN): "
        + "; ".join(f"{a} vs {b}" for a, b in diff[:6]))
    assert sink_values(one) == sink_values(chain), (
        f"sink values differ: ONE {sink_values(one)} CHAIN {sink_values(chain)}")

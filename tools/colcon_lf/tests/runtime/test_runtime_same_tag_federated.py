# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Zero-delay record links across federates.

``same_tag_fixture`` ONE (``st_one``) split in two federates under centralized
coordination (``st_one_2fed.federation.yaml``): ``tick`` and ``relay`` in one,
``sink`` and ``msink`` in the other. Every link is d = 0, so the record links
relay -> sink and tick -> sink (``/c``, one fan-in group), tick -> sink
(``/d``) and tick -> msink (``/m``) cross the federation. A centralized d = 0 record link is a plain
zero-delay LF connection, so its records arrive at their publication tag
(t, m), as in one process, and the sink sees exactly the deliveries of the
one-process ONE run (``test_runtime_same_tag``): the expected tags follow from
the fixture source alone and the ONE run must give the same.

A link of ``after 0`` would land every cross-federate record one microstep
after its publication, the sink's ``/d`` at (T, 1) instead of (T, 0).

The federated launch prints each federate's lines under its own process
name, and edge ids are per federate, so edges are matched across federates by
(publisher class, subscriber class, topic), unique in this fixture.
"""

from __future__ import annotations

import json
import os
import re
import time
from collections import defaultdict
from pathlib import Path

import pytest

from . import runtime_support as rts
from .test_runtime_same_tag import (
    DONE_NEEDLES, ROLE, TICKS, expected_sink_view, expected_view, fmt, run_case, sink_values,
    sink_view,
)

pytestmark = pytest.mark.runtime

GEN_PKG = "lf_generated_same_tag_fixture_stone_federated"
BUILD = ("FEDERATE=$PWD/src/same_tag_fixture/st_one_2fed.federation.yaml LF_LAUNCH=st_one "
         "bash ../docker/lf_build.sh same_tag_fixture")
EDGE_RE = re.compile(
    r"\[(federate__\w+?)(?:-\d+)?\].*rcllf corr: admitted (?:ingress |egress )?edge (\d+) "
    r"(?:\(global \d+\) )?(\S+) -> (\S+) on '([^']+)'")
CORR_RE = re.compile(r'\[(federate__\w+?)(?:-\d+)?\].*?(\{"event":"corr".*\})')


def federated_case() -> rts.RuntimeCase:
    install = rts._PACKAGES_ROOT / "install"
    launch = sorted((install / GEN_PKG / "share" / GEN_PKG / "launch").glob("*.launch.py"))
    if not launch:
        if os.environ.get("RCLLF_REQUIRE_RUNTIME_SMOKE") == "1":
            pytest.fail(f"{GEN_PKG} not built ({BUILD})")
        pytest.skip(f"{GEN_PKG} not built ({BUILD})")
    return rts.RuntimeCase(source_pkg="same_tag_fixture", launch_stem="st_one",
                           gen_pkg=GEN_PKG, launch_file=launch[0].name)


def fed_edges(log: str) -> dict[tuple[str, int], tuple[str, str, str]]:
    """(federate, local edge id) -> (publisher class, subscriber class, topic)."""
    return {(fed, int(e)): (pub, sub, topic)
            for fed, e, pub, sub, topic in EDGE_RE.findall(log)}


def fed_events(log: str, kind: str) -> list[tuple[str, dict]]:
    out = []
    for fed, raw in CORR_RE.findall(log):
        ev = json.loads(raw)
        if ev.get("kind") == kind:
            out.append((fed, ev))
    return out


def fed_deliveries(log: str) -> list[dict]:
    edges = fed_edges(log)
    out = []
    for fed, ev in fed_events(log, "dispatch"):
        pub, sub, topic = edges[(fed, ev["edge"])]
        out.append({"pub": pub, "role": ROLE.get(pub, pub), "sub": sub, "topic": topic,
                    "ordinal": ev["ordinal"], "tag": (ev["elapsed"], ev["microstep"]),
                    "outcome": ev["outcome"], "edge": ev["edge"]})
    return out


def fed_publication_tags(log: str) -> dict[tuple[tuple[str, str, str], int], tuple[int, int]]:
    """((pub, sub, topic), ordinal) -> publication tag, from the publishers' events.

    A local edge logs ``emit`` with the token ordinals. An egress edge logs
    ``emit_remote`` with the record count of each tag, so its ordinals are
    the running count (ordinal k is the k-th record sent on the edge). The
    egress side names the subscriber node (``/sink``), so it is mapped to the
    class of the ingress edge with the same publisher and topic."""
    edges = fed_edges(log)
    sub_class = {(pub, topic): sub for pub, sub, topic in edges.values()
                 if not sub.startswith("/")}
    edges = {k: (pub, sub_class.get((pub, topic), sub), topic)
             for k, (pub, sub, topic) in edges.items()}
    out = {}
    for fed, ev in fed_events(log, "emit"):
        for tok in ev.get("tokens", []):
            out.setdefault((edges[(fed, ev["edge"])], tok["ordinal"]),
                           (ev["elapsed"], ev["microstep"]))
    sent: dict[tuple[str, int], int] = defaultdict(int)
    for fed, ev in fed_events(log, "emit_remote"):
        triple = edges[(fed, ev["edge"])]
        for _ in range(ev["records"]):
            sent[(fed, ev["edge"])] += 1
            out.setdefault((triple, sent[(fed, ev["edge"])]), (ev["elapsed"], ev["microstep"]))
    return out


def fed_sink_view(log: str) -> list[tuple]:
    return sorted((d["tag"], d["topic"], d["role"], d["ordinal"], d["outcome"])
                  for d in fed_deliveries(log) if d["sub"] == "StSink")


def cross_violations(log: str) -> list[str]:
    """Cross-federate deliveries that miss their publication tag.

    ``/d`` and ``/m`` have one writer, so every record of a time lands at its
    publication tag (t, m), as in one process: ``/m`` sends three records per
    tick and all three are delivered at (T, 0) (every record due at a
    tag is delivered at that tag, test_runtime_same_tag). ``/c`` is a fan-in
    group whose second writer is ranked one microstep later (checked by the
    sink view)."""
    pubs = fed_publication_tags(log)
    groups: dict[tuple[str, str, int], list[dict]] = defaultdict(list)
    for d in fed_deliveries(log):
        if d["sub"] in ("StSink", "StMultiSink"):
            groups[(d["sub"], d["topic"], d["tag"][0])].append(d)
    errors = []
    for (sub, topic, time_ns), recs in sorted(groups.items()):
        recs.sort(key=lambda d: d["tag"])
        for i, d in enumerate(recs):
            p = pubs.get(((d["pub"], d["sub"], d["topic"]), d["ordinal"]))
            if p is None:
                errors.append(f"{sub} {topic} ordinal {d['ordinal']}: no emit event")
            elif topic in ("/d", "/m") and d["tag"] != p:
                errors.append(
                    f"{sub} {topic}: record {i + 1} of t={time_ns} ns ({d['role']} ordinal "
                    f"{d['ordinal']}, published at {fmt(p)}) delivered at {fmt(d['tag'])}, "
                    f"expected {fmt(p)} over a zero-delay federated link")
    return errors


_FED: dict[str, tuple[str, float]] = {}


def run_federated() -> tuple[str, float]:
    if "log" not in _FED:
        case = federated_case()
        with pytest.MonkeyPatch.context() as mp:
            mp.setenv("RCLLF_TRACE_CALLBACKS", "1")
            t0 = time.monotonic()
            result = rts.launch_and_capture(
                case, 1.0, until=lambda log: all(n in log for n in DONE_NEEDLES),
                until_timeout_s=60.0)
            wall = time.monotonic() - t0
        log_dir = os.environ.get("RCLLF_SAME_TAG_LOG_DIR")
        if log_dir:  # keep the raw run as evidence
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            (Path(log_dir) / "same_tag_one_2fed.log").write_text(result.log, encoding="utf-8")
            (Path(log_dir) / "same_tag_one_2fed.wall_s").write_text(f"{wall:.3f}\n")
        print(f"same_tag_one_2fed wall time {wall:.3f} s (launch to stop)")
        assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
        _FED["log"] = (result.log, wall)
    return _FED["log"]


def test_xfed_zero_delay_records_arrive_at_their_publication_tag():
    log, _ = run_federated()
    triples = set(fed_edges(log).values())
    for want in [("StSource", "StStage", "/a"), ("StSource", "StSink", "/c"),
                 ("StSource", "StSink", "/d"), ("StSource", "StMultiSink", "/m"),
                 ("StStage", "StSink", "/c")]:
        assert want in triples, f"edge {want} not admitted, admitted {sorted(triples)}"
    errors = cross_violations(log)
    assert not errors, (f"cross-federate d = 0 records off their publication tag "
                        f"({len(errors)} deliveries):\n  " + "\n  ".join(errors[:6]))
    for sub, topic in [("StStage", "/a"), ("StSink", "/d"), ("StMultiSink", "/m")]:
        got = sorted((d["tag"], d["role"], d["ordinal"], d["outcome"])
                     for d in fed_deliveries(log) if d["sub"] == sub and d["topic"] == topic)
        _, want = expected_view("", sub, topic)
        assert got == want, f"{sub} {topic}: delivered {got[:4]}..., expected {want[:4]}..."


def test_xfed_sink_matches_the_fixture_source():
    log, _ = run_federated()
    assert fed_sink_view(log) == expected_sink_view(), (
        f"sink deliveries {fed_sink_view(log)[:6]}..., expected {expected_sink_view()[:6]}...")
    values = sink_values(log)
    assert values.get("/c") == [v for t in TICKS for v in (3 * t + 7, 2000 + t)], (
        f"sink /c values {values.get('/c')}")
    assert values.get("/d") == [1000 + t for t in TICKS], f"sink /d values {values.get('/d')}"
    heard_m = [int(v) for v in re.findall(r"st_msink: topic=/m value=(\d+)", log)]
    assert heard_m == [3000 + 10 * t + k for t in TICKS for k in range(3)], f"msink /m {heard_m}"


def test_xfed_sink_equals_the_one_process_run():
    """The split must not move a delivery: the sink's tags and values equal ONE's."""
    log, _ = run_federated()
    one = run_case("same_tag_one")
    v_one, v_fed = sink_view(one), fed_sink_view(log)
    diff = [(a, b) for a, b in zip(v_one, v_fed) if a != b]
    assert v_one == v_fed, (
        "sink sees different deliveries in one process and across federates (ONE vs 2FED): "
        + "; ".join(f"{a} vs {b}" for a, b in diff[:6]))
    assert sink_values(one) == sink_values(log), (
        f"sink values differ: ONE {sink_values(one)} 2FED {sink_values(log)}")


def test_xfed_sigint_stop_keeps_the_end_of_run_counts():
    """The run is stopped by SIGINT; every cross-federate ingress edge still
    gets its sender's end-of-run COMPLETE count (a fixed regression).

    Before that fix (the stop settled before the COMPLETE exchange) the signal
    shut the context down first: every sink edge reported complete_end false
    and complete_end_missing (desk run)."""
    log, _ = run_federated()
    assert "signal_handler(SIGINT" in log, "the federated run was not stopped by SIGINT"
    missing = sorted(set(re.findall(r"complete_end_missing edge \d+[^\n]*", log)))
    assert not missing, "end-of-run counts lost at the SIGINT stop:\n  " + "\n  ".join(missing)
    ingress = []
    for fed, raw in re.findall(r'\[(federate__\w+?)(?:-\d+)?\].*?(\{"event":"reconcile".*\})', log):
        ingress += [(fed, e) for e in json.loads(raw)["edges"] if e.get("role") == "ingress"]
    assert len(ingress) == 4, f"expected 4 ingress edges in the reconcile lines, got {len(ingress)}"
    bad = [(fed, e["edge"], e.get("complete_end")) for fed, e in ingress
           if e.get("complete_end") is not True]
    assert not bad, f"ingress edges without their end-of-run count (federate, edge, complete_end): {bad}"


INGRESS_RE = re.compile(
    r"\[(federate__\w+?)(?:-\d+)?\].*rcllf corr: admitted ingress edge \d+ .*?"
    r"\(d = (-?\d+) ns, T = (\d+) ns, L = (\d+) ns")


def test_xfed_every_federate_starts_its_stall_watch_at_the_start_time():
    """runtime.cpp calls stall_watch_start once per federated run,
    which logs one ``stall_backstop`` event with the stall window
    W = sum over ingress edges (d + T + eps) + L (Table::stall_window_ns).

    Expected from the fixture alone: ``sinks`` has the four ingress edges
    /c (two writers), /d and /m, each d = 0 and the default take_timeout_ms
    T = 100 ms, L = 2 T = 200 ms and no declared clock bound (eps = 0), so
    W = 4 * 100 + 200 = 600 ms. ``source`` has no ingress edge: W = -1 and no
    backstop thread. No
    test failed when the call was removed, since the receiver's own timer
    arms the watch at tag 0 anyway; without the call no federate logs this
    event, and a timer-free federate has no backstop."""
    log, _ = run_federated()
    got: dict[str, list[int]] = defaultdict(list)
    for fed, ev in fed_events(log, "stall_backstop"):
        got[fed].append(ev["window_ns"])
    ms = 1_000_000
    assert dict(got) == {"federate__source": [-1], "federate__sinks": [600 * ms]}, (
        f"stall_backstop window per federate {dict(got)}, expected source [-1] and sinks "
        f"[{600 * ms}] once each: stall_watch_start did not run at the start of every "
        f"federate (runtime.cpp)")
    # The same window from the edges the sinks federate admitted.
    edges = [(int(d), int(t), int(lag)) for fed, d, t, lag in INGRESS_RE.findall(log)
             if fed == "federate__sinks"]
    assert len(edges) == 4, f"sinks admitted {len(edges)} ingress edges, expected 4"
    assert sum(max(0, d) + t for d, t, _ in edges) + max(lag for *_, lag in edges) == 600 * ms

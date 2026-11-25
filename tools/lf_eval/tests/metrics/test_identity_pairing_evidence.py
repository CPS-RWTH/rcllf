# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Evidence of the identity pairing: what counts as a paired sample.

Missing identity never satisfies a gate, an lfc run is paired by its dispatch
trace, a window edge does not cut a publish from its take, an empty hop next
to a populated one is plotted, and an entirely unresolved analysis keeps and
renders its rows. Each test fails with the message the defect produces.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from lf_eval.metrics.determinism import compute_ordering
from lf_eval.metrics.e2e_latency import E2ELatencyMetric, e2e_latency_samples, run_evidence
from lf_eval.metrics.registry import get_metric
from lf_eval.model.chain import Chain, Hop, build_chain
from lf_eval.model.run import Window, apply_window, make_run, windowed_timeline
from lf_eval.model.timeline import build_timeline
from lf_eval.report import generate_report

from tests.conftest import MS, EventBuilder
from tests.test_chain_latency import SYSTEM, STATIC, lfc_events

AB = Chain(nodes=("NodeA", "NodeB"), hops=(Hop("NodeA", "NodeB"),), name="ab")


def _a_to_b(published: bool, n: int = 3, period_ms: int = 100, take_offset: int = 0) -> EventBuilder:
    """NodeA (timer) -> NodeB, *n* releases.

    ``published`` false leaves A's publishes untraced. A non-zero
    ``take_offset`` makes B take stamps A never published (lost publications).
    """
    b = EventBuilder()
    b.pid = 9
    for node in ("NodeA", "NodeB"):
        b.add_node(node)
    b.add_timer("NodeA", float(period_ms))
    b.add_publisher("NodeA", "/a")
    b.add_subscription("NodeB", "/a")
    base = 1000 * MS
    for k in range(n):
        t = base + k * period_ms * MS
        b.invoke(b.timer_callbacks["NodeA"], t, MS, tid=1)
        if published:
            b.rmw_publish("NodeA", "/a", t + MS // 2, 50 + k, tid=1)
        b.rmw_take("NodeB", "/a", t + 3 * MS - 1, 50 + k + take_offset, tid=2)
        b.invoke(b.sub_callbacks[("NodeB", "/a")], t + 3 * MS, MS, tid=2)
    return b


# ── missing identity never satisfies a gate ───────────────────────


def test_unresolved_activations_are_not_paired_by_timestamps():
    """three takes of untraced publications became ``unresolved: 0``."""
    timeline = build_timeline(_a_to_b(published=False).build())
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=[AB])
    rec = res.data["per_run"]["r"]["chains"]["ab"]
    got = (rec["e2e_pairing"]["pairing"], rec["e2e_pairing"]["unresolved"], rec["complete"])
    assert got == ("message_identity", 3, False), (
        f"unresolved activations were paired by timestamps: pairing, unresolved, complete = {got}")
    order = compute_ordering(timeline, AB)
    assert (order.verdict, order.unresolved) == ("INCOMPLETE", 3), (
        f"ordering {order.verdict} with {order.unresolved} unresolved activations")


def _lfc_run_dir(tmp_path: Path) -> Path:
    """An lfc run directory: system.json, the rcllf manifest and the dispatch events."""
    run = tmp_path / "20261102T000000Z_lfc_default_r0"
    run.mkdir()
    (run / "system.json").write_text(json.dumps(SYSTEM))
    (run / "rcllf_trace_manifest.json").write_text(json.dumps(STATIC))
    lines = [json.dumps({"stats": {}})]
    lines += [json.dumps(e.__dict__) for e in lfc_events(20)]
    (run / "semantic_events.jsonl").write_text("\n".join(lines) + "\n")
    return run


def test_lfc_run_pairs_by_the_dispatch_trace(tmp_path):
    """an lfc run is paired by dispatch identity, not by ROS-trace timestamps.

    Expected values by hand from ``lfc_events(20)`` (dispatch d spans
    ``[ts, ts + 5]``): src->mid pairs d1->d2 and d4->d5, 110 - 105 = 5 ns
    each; mid->gate d2->d3 and d5->d6, 5 ns each; gate->sink d7->d8,
    400 - 305 = 95 ns. The sink d8 walks back through gate's tick d7 to its
    newest input d6, then d5, to the source tick d4: 405 - 200 = 205 ns.
    """
    run_dir = _lfc_run_dir(tmp_path)
    chain = build_chain(["src", "mid", "gate", "sink"], name="c")
    # the ROS trace of an lfc run does not see the dispatch flow: an empty timeline
    run = make_run(run_dir.name, build_timeline([]), mode="lfc", path=run_dir)
    res = E2ELatencyMetric().compute([run], chains=[chain])
    rec = res.data["per_run"][run_dir.name]["chains"]["c"]
    hops = {h: (v["pairing"], v.get("identity_source"), v["count"]) for h, v in rec["hops"].items()}
    assert hops == {"src->mid": ("message_identity", "lfc_dispatch", 2),
                    "mid->gate": ("message_identity", "lfc_dispatch", 2),
                    "gate->sink": ("message_identity", "lfc_dispatch", 1)}, (
        f"the lfc run was not paired by its dispatch trace: {hops}")
    assert rec["hops"]["src->mid"]["mean_us"] == pytest.approx(0.005)
    assert rec["hops"]["mid->gate"]["mean_us"] == pytest.approx(0.005)
    assert (rec["e2e_pairing"]["paired"], rec["e2e_pairing"]["unresolved"]) == (1, 0)
    e2e = e2e_latency_samples(run.timeline, chain, evidence=run_evidence(run))
    assert e2e.values_us == pytest.approx([0.205])
    # one sample is too few for a distribution, and the note says so
    assert res.note.startswith("1 chain sample(s) paired"), res.note
    order = get_metric("determinism").compute([run], chains=[chain])
    assert order.summary_rows[0]["ordering"] == "PASS"


def test_windowed_run_pairs_a_message_published_before_the_window():
    """The window cut a publish from its take and reported a traced sample unresolved.

    Window [2002 ms, 3002 ms): NodeB activations k = 10..19 start inside it.
    The publish of k = 10 at 2000.5 ms lies before the window, its take after.
    Every activation is paired, 3 ms - 1 ms = 2 ms each.
    """
    events = sorted(_a_to_b(published=True, n=30).build(), key=lambda e: e.timestamp_ns)
    window = Window(warmup_s=1.002, steady_s=1.0)
    full = build_timeline(events)
    run = make_run("w", windowed_timeline(events, window), mode="stock")
    run.window, run.full_timeline = window, full
    assert len(apply_window(events, window)) < len(events)
    res = E2ELatencyMetric().compute([run], chains=[AB])
    hop = res.data["per_run"]["w"]["chains"]["ab"]["hops"]["NodeA->NodeB"]
    got = (hop["activations"], hop["paired"], hop["unresolved"], hop["complete"])
    assert got == (10, 10, 0, True), (
        f"the window edge left a traced message unresolved: activations, paired, "
        f"unresolved, complete = {got}")
    assert hop["mean_us"] == pytest.approx(2000.0)


# ── an empty hop next to a populated one is plotted ──────────────


def test_plot_with_an_empty_hop_beside_a_populated_one(tmp_path):
    pytest.importorskip("matplotlib")
    b = _a_to_b(published=True)
    b.add_node("NodeC")
    b.add_publisher("NodeB", "/b")
    b.add_subscription("NodeC", "/b")
    timeline = build_timeline(b.build())
    chains = [AB, Chain(nodes=("NodeB", "NodeC"), hops=(Hop("NodeB", "NodeC"),), name="bc")]
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=chains)
    assert res.data["per_run"]["r"]["chains"]["bc"]["hops"]["NodeB->NodeC"]["count"] == 0
    try:
        written = E2ELatencyMetric().plot(res, tmp_path)
    except KeyError as exc:
        pytest.fail(f"plotting a hop without samples raised KeyError {exc}")
    assert tmp_path / "hop_latency_bars.png" in written


# ── an entirely unresolved analysis keeps its rows ───────────────


def test_unresolved_only_analysis_keeps_and_renders_its_rows(tmp_path):
    """every activation unresolved returned ``available False`` with no rows.

    A's publications are traced (so the hop has identity) but B takes three
    stamps A never published, so every B activation is unresolved.
    """
    timeline = build_timeline(_a_to_b(published=True, take_offset=900).build())
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=[AB])
    assert res.available is False
    rows = [(r["chain"], r["n"], r["unresolved"], r["complete"]) for r in res.summary_rows]
    assert rows == [("NodeA->NodeB", 0, 3, False)], (
        f"the unresolved-only analysis lost its summary rows: {rows}")
    payload = {"metrics": {"e2e_latency": res.to_dict()}}
    text = generate_report([], payload, [AB], tmp_path).read_text()
    assert "| NodeA->NodeB |" in text and "unresolved" in text, (
        f"the report does not render the unresolved counters:\n{text}")

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the e2e_latency metric.

The synthetic pipeline has exactly known timings, so latency values are asserted
numerically rather than merely for plausibility:

  NodeA timer starts t0, ends t0+1ms
  NodeB sub starts t0+3ms, ends t0+4ms   => hop A->B = 2 ms
  NodeC sub starts t0+6ms, ends t0+7ms   => hop B->C = 2 ms
  end-to-end = NodeA start -> NodeC end  => 7 ms
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.e2e_latency import compute_e2e, compute_hops
from lf_eval.metrics.registry import get_metric
from lf_eval.model.chain import build_chain
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder


def test_e2e_latency_matches_constructed_timing(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    stats = compute_e2e(timeline, pipeline_chain)
    assert stats is not None
    assert stats.count == 20
    assert stats.mean_us == pytest.approx(7000.0)
    assert stats.min_us == pytest.approx(7000.0)
    assert stats.max_us == pytest.approx(7000.0)
    assert stats.std_us == pytest.approx(0.0)


def test_hop_latencies_match_constructed_timing(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    hops = dict(
        (hop.label, stats) for hop, stats in compute_hops(timeline, pipeline_chain)
    )
    assert set(hops) == {"NodeA->NodeB", "NodeB->NodeC"}
    # upstream end -> downstream start = 2 ms on both hops.
    assert hops["NodeA->NodeB"].mean_us == pytest.approx(2000.0)
    assert hops["NodeB->NodeC"].mean_us == pytest.approx(2000.0)
    assert hops["NodeA->NodeB"].count == 20


def test_e2e_needs_at_least_two_activations():
    b = EventBuilder()
    for node in ("A", "B"):
        b.add_node(node)
    b.add_timer("A", 100.0)
    b.add_subscription("B", "/t")
    b.timer_fire("A", 1000 * MS, MS)
    b.sub_fire("B", "/t", 1002 * MS, MS)
    timeline = build_timeline(b.build())
    chain = build_chain(["A", "B"])
    # A single sample cannot be summarised.
    assert compute_e2e(timeline, chain) is None


def test_e2e_returns_none_when_a_node_is_absent(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    chain = build_chain(["NodeA", "Ghost"])
    assert compute_e2e(timeline, chain) is None


def test_e2e_returns_none_for_degenerate_chain(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    assert compute_e2e(timeline, build_chain(["NodeA"])) is None


def test_max_gap_filters_non_causal_pairs(pipeline_builder, pipeline_chain):
    """A gap window below the real hop latency must reject every pair."""
    timeline = build_timeline(pipeline_builder.build())
    # Real hop gap is 2 ms; allow only 1 ms.
    hops = compute_hops(timeline, pipeline_chain, max_gap_ns=1 * MS)
    assert hops == []


def test_gated_hop_pairs_against_downstream_timer():
    """A gated hop must measure to the downstream node's *timer*, not its sub."""
    b = EventBuilder()
    for node in ("Sensor", "Planner", "Actuator"):
        b.add_node(node)
    b.add_timer("Sensor", 100.0)
    b.add_subscription("Planner", "/raw")
    b.add_timer("Planner", 100.0)
    b.add_subscription("Actuator", "/cmd")

    base = 1000 * MS
    for i in range(10):
        t0 = base + i * 100 * MS
        b.timer_fire("Sensor", t0, MS)
        b.sub_fire("Planner", "/raw", t0 + 2 * MS, MS)      # receives, no publish
        # Planner's own tick (the gate) publishes 40 ms later.
        b.timer_fire("Planner", t0 + 40 * MS, MS)
        b.sub_fire("Actuator", "/cmd", t0 + 43 * MS, MS)
    timeline = build_timeline(b.build())

    from lf_eval.model.chain import Chain, Hop

    chain = Chain(
        nodes=("Sensor", "Planner", "Actuator"),
        hops=(
            Hop("Sensor", "Planner", gated=False),
            Hop("Planner", "Actuator", gated=True),
        ),
        name="gated",
    )
    hops = dict((h.label, s) for h, s in compute_hops(timeline, chain))
    # Sensor ends t0+1ms, Planner's sub starts t0+2ms => 1 ms.
    assert hops["Sensor->Planner"].mean_us == pytest.approx(1000.0)
    # Planner->Actuator is gated: pairing must use Planner's *timer* (the only
    # callback that publishes), ending t0+41ms, to the Actuator sub start at
    # t0+43ms => 2 ms. Using all Planner invocations would blend in its
    # absorbing subscription and yield a meaningless ~21 ms mean.
    assert hops["Planner->Actuator"].mean_us == pytest.approx(2000.0)
    assert hops["Planner->Actuator"].count == 10

    stats = compute_e2e(timeline, chain)
    assert stats is not None
    # Sensor start t0 -> Planner sub end (t0+3ms) -> Planner timer end
    # (t0+41ms) -> Actuator sub end (t0+44ms) = 44 ms.
    assert stats.mean_us == pytest.approx(44_000.0)


def test_metric_reports_unavailable_without_a_chain(pipeline_run):
    result = get_metric("e2e_latency").compute([pipeline_run], chains=[])
    assert result.available is False
    assert "chain" in result.note


def test_metric_reports_unavailable_when_chain_nodes_are_missing(
    pipeline_run, pipeline_system
):
    chain = build_chain(["Ghost1", "Ghost2"], system=pipeline_system)
    result = get_metric("e2e_latency").compute(
        [pipeline_run], system=pipeline_system, chains=[chain]
    )
    assert result.available is False
    assert "observed" in result.note


def test_metric_summary_rows_and_payload(pipeline_run, pipeline_system, pipeline_chain):
    result = get_metric("e2e_latency").compute(
        [pipeline_run], system=pipeline_system, chains=[pipeline_chain]
    )
    assert result.available
    row = result.summary_rows[0]
    assert row["run"] == pipeline_run.name
    assert row["mode"] == "lfc"
    assert row["mean_ms"] == pytest.approx(7.0)
    assert row["n"] == 20

    chain_data = result.data["per_run"][pipeline_run.name]["chains"]["primary"]
    assert chain_data["nodes"] == ["NodeA", "NodeB", "NodeC"]
    assert chain_data["e2e"]["mean_ms"] == pytest.approx(7.0)
    assert set(chain_data["hops"]) == {"NodeA->NodeB", "NodeB->NodeC"}
    # raw samples omitted by default to keep metrics.json small
    assert "raw_us" not in chain_data["e2e"]


def test_keep_raw_includes_samples(pipeline_run, pipeline_system, pipeline_chain):
    result = get_metric("e2e_latency").compute(
        [pipeline_run],
        system=pipeline_system,
        chains=[pipeline_chain],
        include_raw=True,
    )
    e2e = result.data["per_run"][pipeline_run.name]["chains"]["primary"]["e2e"]
    assert len(e2e["raw_us"]) == 20


def test_plot_writes_expected_files(
    pipeline_run, pipeline_system, pipeline_chain, tmp_path
):
    metric = get_metric("e2e_latency")
    result = metric.compute(
        [pipeline_run],
        system=pipeline_system,
        chains=[pipeline_chain],
        include_raw=True,
    )
    written = metric.plot(result, tmp_path)
    names = {p.name for p in written}
    assert "e2e_latency_cdf.png" in names
    assert "hop_latency_bars.png" in names
    for path in written:
        assert path.is_file() and path.stat().st_size > 0


def test_plot_is_noop_when_unavailable(pipeline_run, tmp_path):
    metric = get_metric("e2e_latency")
    result = metric.compute([pipeline_run], chains=[])
    assert metric.plot(result, tmp_path) == []


def test_two_modes_are_compared(pipeline_builder, pipeline_system, pipeline_chain):
    """Runs of different modes must both appear in the payload."""
    timeline = build_timeline(pipeline_builder.build())
    runs = [
        make_run("lfc_run", timeline, mode="lfc"),
        make_run("stock_run", timeline, mode="stock"),
    ]
    result = get_metric("e2e_latency").compute(
        runs, system=pipeline_system, chains=[pipeline_chain]
    )
    modes = {row["mode"] for row in result.summary_rows}
    assert modes == {"lfc", "stock"}

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the determinism metric: ordering, jitter, cross-run consistency."""

from __future__ import annotations

import pytest

from lf_eval.metrics.determinism import (
    compute_consistency,
    compute_ordering,
    compute_timer_jitter,
)
from lf_eval.metrics.registry import get_metric
from lf_eval.model.chain import build_chain
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder


def _overlapping_pipeline(overlap_ms: float = 0.5) -> EventBuilder:
    """A->B where B *starts before* A finishes — an ordering violation."""
    b = EventBuilder()
    for node in ("A", "B"):
        b.add_node(node)
    b.add_timer("A", 100.0)
    b.add_subscription("B", "/t")
    base = 1000 * MS
    for i in range(10):
        t0 = base + i * 100 * MS
        b.timer_fire("A", t0, 2 * MS)  # A runs t0 .. t0+2ms
        # B starts inside A's execution window.
        b.sub_fire("B", "/t", t0 + int((2 - overlap_ms) * MS), MS)
    return b


def test_clean_pipeline_passes_ordering(identity_pipeline_builder, pipeline_chain):
    timeline = build_timeline(identity_pipeline_builder.build())
    result = compute_ordering(timeline, pipeline_chain)
    assert result.verdict == "PASS"
    assert result.violations == 0
    assert result.pairs_checked == 40  # 20 activations x 2 hops
    assert set(result.pairing.values()) == {"message_identity"}


def test_timestamp_paired_ordering_is_never_a_pass(pipeline_builder, pipeline_chain):
    """Review finding: a trace without identity proves no pair."""
    timeline = build_timeline(pipeline_builder.build())
    result = compute_ordering(timeline, pipeline_chain)
    assert (result.violations, result.pairs_checked) == (0, 40)
    assert result.verdict == "INCOMPLETE", (
        f"ordering verdict {result.verdict} from timestamp pairs alone")


def test_overlapping_callbacks_are_flagged():
    timeline = build_timeline(_overlapping_pipeline().build())
    result = compute_ordering(timeline, build_chain(["A", "B"]))
    assert result.verdict == "FAIL"
    assert result.violations == 10
    assert result.violation_rate == pytest.approx(1.0)


def test_violation_details_capture_the_overlap():
    timeline = build_timeline(_overlapping_pipeline(overlap_ms=1.0).build())
    result = compute_ordering(timeline, build_chain(["A", "B"]))
    detail = result.details[0]
    assert detail["upstream"] == "A"
    assert detail["downstream"] == "B"
    assert detail["overlap_ns"] == pytest.approx(1 * MS)
    # Only the first 10 are retained.
    assert len(result.details) <= 10


def test_ordering_result_payload_shape():
    timeline = build_timeline(_overlapping_pipeline().build())
    payload = compute_ordering(timeline, build_chain(["A", "B"])).to_dict()
    assert payload["verdict"] == "FAIL"
    assert payload["violations"] == 10
    assert payload["pairs_checked"] == 10
    assert payload["violation_rate_pct"] == pytest.approx(100.0)
    assert len(payload["first_violations"]) <= 5


def test_ordering_skips_gated_hops():
    """Gated hops are legitimately unordered and must not be checked."""
    from lf_eval.model.chain import Chain, Hop

    timeline = build_timeline(_overlapping_pipeline().build())
    chain = Chain(nodes=("A", "B"), hops=(Hop("A", "B", gated=True),), name="g")
    result = compute_ordering(timeline, chain)
    assert result.pairs_checked == 0
    assert result.verdict == "N/A"


def test_ordering_na_for_missing_nodes(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    result = compute_ordering(timeline, build_chain(["NodeA", "Ghost"]))
    assert result.verdict == "N/A"
    assert result.pairs_checked == 0


# ── timer jitter ─────────────────────────────────────────────────────────


def test_perfect_timer_has_zero_jitter(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    jitter = compute_timer_jitter(timeline, {"NodeA": 100.0})
    assert jitter["NodeA"]["mean_period_ms"] == pytest.approx(100.0)
    assert jitter["NodeA"]["std_ms"] == pytest.approx(0.0)
    assert jitter["NodeA"]["max_jitter_ms"] == pytest.approx(0.0)
    assert jitter["NodeA"]["expected_period_ms"] == pytest.approx(100.0)
    assert jitter["NodeA"]["count"] == 19  # 20 firings => 19 intervals


def test_irregular_timer_reports_jitter():
    b = EventBuilder()
    b.add_node("T")
    b.add_timer("T", 100.0)
    # Intervals: 100, 110, 90, 100 ms
    for start in (0, 100, 210, 300, 400):
        b.timer_fire("T", start * MS, MS)
    timeline = build_timeline(b.build())
    jitter = compute_timer_jitter(timeline, {"T": 100.0})["T"]
    assert jitter["max_jitter_ms"] == pytest.approx(10.0)
    assert jitter["std_ms"] > 0


def test_jitter_falls_back_to_traced_period(pipeline_builder):
    """Without system.json, rcl_timer_init's period must be used."""
    timeline = build_timeline(pipeline_builder.build())
    jitter = compute_timer_jitter(timeline, timer_periods={})
    assert jitter["NodeA"]["expected_period_ms"] == pytest.approx(100.0)


def test_jitter_needs_at_least_three_firings():
    b = EventBuilder()
    b.add_node("T")
    b.add_timer("T", 100.0)
    b.timer_fire("T", 0, MS)
    b.timer_fire("T", 100 * MS, MS)
    timeline = build_timeline(b.build())
    assert compute_timer_jitter(timeline, {"T": 100.0}) == {}


# ── cross-run consistency ────────────────────────────────────────────────


def test_consistency_needs_two_runs_of_one_mode(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    single = [make_run("r1", timeline, mode="lfc")]
    assert compute_consistency(single, [pipeline_chain]) == {}


def test_identical_runs_are_perfectly_consistent(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    runs = [
        make_run("r1", timeline, mode="lfc"),
        make_run("r2", timeline, mode="lfc"),
    ]
    consistency = compute_consistency(runs, [pipeline_chain])
    hop = consistency["lfc"]["hops"]["NodeA->NodeB"]
    assert hop["n_runs"] == 2
    # Identical distributions => KS statistic 0, no variation of means.
    assert hop["max_ks_statistic"] == pytest.approx(0.0)
    assert hop["cov_of_means"] == pytest.approx(0.0)
    assert hop["comparisons"][0]["mean_diff_us"] == pytest.approx(0.0)


def test_divergent_runs_show_a_large_ks_statistic(pipeline_builder, pipeline_chain):
    """A run with systematically different hop latency must be detected."""
    slow = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        slow.add_node(node)
    slow.add_timer("NodeA", 100.0)
    slow.add_subscription("NodeB", "/a_out")
    slow.add_subscription("NodeC", "/b_out")
    base = 1000 * MS
    for i in range(20):
        t0 = base + i * 100 * MS
        slow.timer_fire("NodeA", t0, MS)
        # 20 ms hop instead of 2 ms.
        slow.sub_fire("NodeB", "/a_out", t0 + 21 * MS, MS)
        slow.sub_fire("NodeC", "/b_out", t0 + 30 * MS, MS)

    runs = [
        make_run("fast", build_timeline(pipeline_builder.build()), mode="stock"),
        make_run("slow", build_timeline(slow.build()), mode="stock"),
    ]
    consistency = compute_consistency(runs, [pipeline_chain])
    hop = consistency["stock"]["hops"]["NodeA->NodeB"]
    assert hop["max_ks_statistic"] == pytest.approx(1.0)
    assert hop["cov_of_means"] > 0.5


def _short_run(hop_ms: float, n: int) -> EventBuilder:
    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_subscription("NodeB", "/a_out")
    b.add_subscription("NodeC", "/b_out")
    for i in range(n):
        t0 = 1000 * MS + i * 100 * MS
        b.timer_fire("NodeA", t0, MS)
        b.sub_fire("NodeB", "/a_out", t0 + (1 + hop_ms) * MS, MS)
        b.sub_fire("NodeC", "/b_out", t0 + (10 + hop_ms) * MS, MS)
    return b


def test_runs_below_five_samples_are_compared_with_their_sizes(pipeline_builder, pipeline_chain):
    """KS is defined for any n, so a 3-sample run is compared, not dropped."""
    runs = [
        make_run("full", build_timeline(pipeline_builder.build()), mode="stock"),
        make_run("short", build_timeline(_short_run(20.0, 3).build()), mode="stock"),
    ]
    hops = compute_consistency(runs, [pipeline_chain]).get("stock", {}).get("hops", {})
    assert "NodeA->NodeB" in hops, "run with fewer than 5 samples silently dropped from the KS comparison"
    hop = hops["NodeA->NodeB"]
    assert hop["n_samples_per_run"]["short"] == 3
    cmp = hop["comparisons"][0]
    assert {cmp["run_a"]: cmp["n_a"], cmp["run_b"]: cmp["n_b"]}["short"] == 3
    # every short sample is 19 ms slower than every full sample: KS distance 1
    assert cmp["ks_statistic"] == pytest.approx(1.0)


def test_run_without_samples_is_listed_as_excluded(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    runs = [
        make_run("a", timeline, mode="lfc"),
        make_run("b", timeline, mode="lfc"),
        make_run("none", build_timeline(_short_run(0.0, 0).build()), mode="lfc"),
    ]
    hop = compute_consistency(runs, [pipeline_chain])["lfc"]["hops"]["NodeA->NodeB"]
    assert hop["n_runs"] == 2
    assert hop["excluded_runs"] == {"none": "no paired sample on this hop"}


def test_hop_with_one_sampled_run_lists_its_runs(pipeline_builder, pipeline_chain):
    runs = [
        make_run("a", build_timeline(pipeline_builder.build()), mode="lfc"),
        make_run("none", build_timeline(_short_run(0.0, 0).build()), mode="lfc"),
    ]
    mode = compute_consistency(runs, [pipeline_chain]).get("lfc", {})
    listed = mode.get("hops_without_comparison", {}).get("NodeA->NodeB")
    assert listed == {"a": "only run with a paired sample on this hop",
                      "none": "no paired sample on this hop"}, (
        f"hop without a KS comparison vanished from the output: {mode}")


def test_consistency_groups_by_mode(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    runs = [
        make_run("l1", timeline, mode="lfc"),
        make_run("l2", timeline, mode="lfc"),
        make_run("s1", timeline, mode="stock"),
    ]
    consistency = compute_consistency(runs, [pipeline_chain])
    # stock has only one run => excluded.
    assert set(consistency) == {"lfc"}
    assert consistency["lfc"]["n_runs"] == 2


# ── metric wrapper ───────────────────────────────────────────────────────


def test_metric_reports_ordering_and_notes(pipeline_run, pipeline_system, pipeline_chain):
    result = get_metric("determinism").compute(
        [pipeline_run], system=pipeline_system, chains=[pipeline_chain]
    )
    assert result.available
    row = result.summary_rows[0]
    # pipeline_run has no rmw identity: timestamp pairs give no PASS
    assert row["ordering"] == "INCOMPLETE"
    assert row["violations"] == "0/40"
    assert "single run" in result.note


def test_metric_without_runs_is_unavailable():
    result = get_metric("determinism").compute([])
    assert result.available is False


def test_metric_without_chain_still_reports_jitter(pipeline_run, pipeline_system):
    result = get_metric("determinism").compute(
        [pipeline_run], system=pipeline_system, chains=[]
    )
    assert result.available is True
    jitter = result.data["per_run"][pipeline_run.name]["timer_jitter"]
    assert "NodeA" in jitter
    assert "no chain resolved" in result.note


def test_plot_writes_expected_files(pipeline_system, pipeline_chain, tmp_path):
    timeline = build_timeline(_overlapping_pipeline().build())
    runs = [make_run("stock_run", timeline, mode="stock")]
    metric = get_metric("determinism")
    result = metric.compute(
        runs, system=pipeline_system, chains=[build_chain(["A", "B"])]
    )
    written = metric.plot(result, tmp_path)
    assert any(p.name == "ordering_invariant.png" for p in written)
    for path in written:
        assert path.is_file() and path.stat().st_size > 0


def test_jitter_plot_is_written(pipeline_run, pipeline_system, pipeline_chain, tmp_path):
    metric = get_metric("determinism")
    result = metric.compute(
        [pipeline_run], system=pipeline_system, chains=[pipeline_chain]
    )
    written = metric.plot(result, tmp_path)
    assert any(p.name == "timer_jitter_bars.png" for p in written)


def test_hop_with_few_samples_is_compared_not_dropped():
    """KS is valid at any n, a run with 3 hop samples is compared."""
    runs = []
    for name, hop_ms in (("r1", 2), ("r2", 3)):
        b = EventBuilder()
        for node in ("NodeA", "NodeB", "NodeC"):
            b.add_node(node)
        b.add_timer("NodeA", 100.0)
        b.add_subscription("NodeB", "/a_out")
        b.add_subscription("NodeC", "/b_out")
        for i in range(3):
            t0 = 1000 * MS + i * 100 * MS
            b.timer_fire("NodeA", t0, MS)
            b.sub_fire("NodeB", "/a_out", t0 + (1 + hop_ms) * MS, MS)
        runs.append(make_run(name, build_timeline(b.build()), mode="stock"))
    consistency = compute_consistency(runs, [build_chain(["NodeA", "NodeB"])])
    hops = consistency.get("stock", {}).get("hops", {})
    assert "NodeA->NodeB" in hops, f"a hop with 3 samples per run was dropped: {consistency}"
    cmp = hops["NodeA->NodeB"]["comparisons"][0]
    assert (cmp["n_a"], cmp["n_b"]) == (3, 3)

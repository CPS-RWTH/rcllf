# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the cpu metric.

The ``mean_range`` assertions check the summary statistics;
:func:`test_aggregate_by_mode_*` check that rows
are keyed by the modes discovered from the runs, not by a hardcoded product
of systems and variants.
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.cpu import (
    aggregate_by_mode,
    mean_range,
    per_node_cores,
    trace_cores_used,
    trace_vs_proc,
)
from lf_eval.metrics.registry import get_metric
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder


# ── ported pure functions ────────────────────────────────────────────────


def test_mean_range():
    assert mean_range([1.0, 3.0]) == (2.0, 1.0, 3.0)


def test_mean_range_empty():
    assert mean_range([]) == (0.0, 0.0, 0.0)


def test_trace_vs_proc_is_signed_relative_to_proc():
    assert trace_vs_proc(1.00, 0.70) == pytest.approx(-0.3)
    assert trace_vs_proc(0.50, 0.60) == pytest.approx(0.2)


def test_trace_vs_proc_undefined_without_proc_cpu():
    assert trace_vs_proc(0.0, 0.0) is None
    assert trace_vs_proc(0.0, 0.3) is None


def test_metric_issues_no_tolerance_verdict_on_the_gap(pipeline_builder):
    """/proc CPU and callback wall time are different quantities.

    No tolerance between them is derivable, so a 30 percent gap must come out
    as a number, not as an OK/FLAG verdict from an unfounded 20 percent cut.
    """
    timeline = build_timeline(pipeline_builder.build())
    run = make_run("r", timeline, mode="lfc", cpu_proc={"aggregate_cores": 10.0, "n_cores_machine": 10})
    result = get_metric("cpu").compute([run])
    entry = result.data["per_run"]["r"]
    row = result.data["by_mode"]["lfc"]
    assert "cross_check" not in entry and "cross_check" not in row, (
        f"cpu metric still issues an unfounded 20 percent verdict: {entry.get('cross_check')}"
    )
    # entry["trace_cores"] is rounded to 4 decimals for display, the gap is not,
    # so they agree within half a unit of that rounding over 10 proc cores.
    expected = (entry["trace_cores"] - 10.0) / 10.0
    assert entry["trace_vs_proc"] == pytest.approx(expected, abs=0.00005 / 10.0)


def test_trace_vs_proc_keeps_a_gap_below_four_decimals():
    # Rounding to 4 decimals reported a real gap of
    # 4e-5 as 0.0, losing its sign and size.
    assert trace_vs_proc(1.0, 1.00004) > 0, "a 4e-5 gap was rounded to 0.0"


# ── trace-derived utilisation ────────────────────────────────────────────


def test_trace_cores_used_is_busy_over_span():
    """20 timer firings of 1 ms over a ~1.9 s span."""
    b = EventBuilder()
    b.add_node("N")
    b.add_timer("N", 100.0)
    # 10 firings of 10 ms each inside a 1000 ms span = 0.1 cores.
    for i in range(11):
        b.timer_fire("N", i * 100 * MS, 10 * MS if i < 10 else 0)
    timeline = build_timeline(b.build())
    run = make_run("r", timeline, mode="lfc")
    # busy = 100 ms, span = 1000 ms
    assert trace_cores_used(run) == pytest.approx(0.1, rel=0.01)


def test_trace_cores_used_none_without_span():
    run = make_run("r", build_timeline([]), mode="lfc")
    assert trace_cores_used(run) is None


def test_per_node_cores_sorted_descending():
    b = EventBuilder()
    for node in ("Busy", "Idle"):
        b.add_node(node)
        b.add_timer(node, 100.0)
    for i in range(10):
        b.timer_fire("Busy", i * 100 * MS, 10 * MS)
        b.timer_fire("Idle", i * 100 * MS + 50 * MS, MS)
    b.timer_fire("Busy", 1000 * MS, 0)
    timeline = build_timeline(b.build())
    shares = per_node_cores(make_run("r", timeline, mode="lfc"))
    assert list(shares) == ["Busy", "Idle"]
    assert shares["Busy"] > shares["Idle"]


# ── aggregation (the report_cpu deliverable table) ───────────────────────


def test_aggregate_by_mode_means_and_percentages():
    per_run = {
        "a": {"mode": "stock", "proc_cores": 0.50, "trace_cores": 0.48, "n_cores_machine": 12},
        "b": {"mode": "stock", "proc_cores": 0.60, "trace_cores": 0.55, "n_cores_machine": 12},
    }
    row = aggregate_by_mode(per_run)["stock"]
    assert row["n_runs"] == 2
    assert row["proc_cores_mean"] == 0.55
    assert row["proc_cores_range"] == [0.5, 0.6]
    assert row["pct_one_core"] == 55.0
    assert row["pct_machine"] == pytest.approx(round(0.55 * 100 / 12, 2))
    assert row["trace_vs_proc"] == pytest.approx((0.515 - 0.55) / 0.55, abs=1e-4)


def test_aggregate_by_mode_reports_divergence_as_a_number():
    per_run = {
        "a": {"mode": "lfc", "proc_cores": 1.0, "trace_cores": 0.5, "n_cores_machine": 8},
    }
    assert aggregate_by_mode(per_run)["lfc"]["trace_vs_proc"] == -0.5


def test_aggregate_by_mode_without_proc_data():
    per_run = {
        "a": {"mode": "lfc", "proc_cores": None, "trace_cores": 0.4, "n_cores_machine": 8},
    }
    row = aggregate_by_mode(per_run)["lfc"]
    assert row["proc_cores_mean"] == 0.0
    assert row["trace_vs_proc"] is None
    # Falls back to the trace figure for the headline percentage.
    assert row["pct_one_core"] == 40.0


def test_aggregate_gap_pairs_only_runs_with_both_figures():
    # Run b has no cpu_proc.json: its trace figure must not enter the gap.
    per_run = {
        "a": {"mode": "stock", "proc_cores": 1.0, "trace_cores": 1.0, "n_cores_machine": 8},
        "b": {"mode": "stock", "proc_cores": None, "trace_cores": 9.0, "n_cores_machine": 8},
    }
    gap = aggregate_by_mode(per_run)["stock"]["trace_vs_proc"]
    assert gap == 0.0, f"mode gap mixes runs without /proc data: {gap}"


def test_aggregate_by_mode_separates_modes():
    per_run = {
        "a": {"mode": "lfc", "proc_cores": 0.4, "trace_cores": 0.4, "n_cores_machine": 8},
        "b": {"mode": "stock", "proc_cores": 0.9, "trace_cores": 0.9, "n_cores_machine": 8},
    }
    aggregate = aggregate_by_mode(per_run)
    assert set(aggregate) == {"lfc", "stock"}
    assert aggregate["stock"]["proc_cores_mean"] == 0.9


# ── metric wrapper ───────────────────────────────────────────────────────


def test_metric_uses_proc_json_when_present(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    run = make_run(
        "r",
        timeline,
        mode="lfc",
        cpu_proc={"aggregate_cores": 0.75, "n_cores_machine": 10},
    )
    result = get_metric("cpu").compute([run])
    assert result.available
    entry = result.data["per_run"]["r"]
    assert entry["proc_cores"] == 0.75
    assert entry["n_cores_machine"] == 10
    assert entry["pct_one_core"] == 75.0
    assert "trace_vs_proc" in entry


def test_metric_trace_only_labels_callback_wall_time(pipeline_run):
    result = get_metric("cpu").compute([pipeline_run])
    assert result.available
    assert "CALLBACK WALL TIME, not CPU" in result.note
    assert "lower bound" not in result.note.lower()
    assert result.data["per_run"][pipeline_run.name]["proc_cores"] is None


def test_metric_unavailable_without_any_data():
    run = make_run("empty", build_timeline([]), mode="lfc")
    result = get_metric("cpu").compute([run])
    assert result.available is False
    assert "sample-cpu" in result.note


def test_metric_reports_per_node_breakdown(pipeline_run):
    result = get_metric("cpu").compute([pipeline_run])
    per_node = result.data["per_run"][pipeline_run.name]["per_node_cores"]
    assert set(per_node) == {"NodeA", "NodeB", "NodeC"}


def test_plot_writes_file(pipeline_run, tmp_path):
    metric = get_metric("cpu")
    result = metric.compute([pipeline_run])
    written = metric.plot(result, tmp_path)
    assert [p.name for p in written] == ["cpu_utilisation.png"]
    assert written[0].is_file() and written[0].stat().st_size > 0


def test_plot_noop_when_unavailable(tmp_path):
    metric = get_metric("cpu")
    result = metric.compute([make_run("empty", build_timeline([]), mode="lfc")])
    assert metric.plot(result, tmp_path) == []


def test_metric_mode_summary_keeps_a_gap_below_four_decimals(pipeline_builder):
    """The complete metric output. The run's
    trace figure is 1.00004 cores against 1.0 /proc cores. Rounded to 4 decimals
    for per_run display it reads 1.0, and a mode gap built from that was 0.0."""
    timeline = build_timeline(pipeline_builder.build())
    span = timeline.duration_ns
    busy = timeline.cpu_busy_ns()
    # proc cores chosen so trace/proc - 1 is exactly 4e-5 for this trace
    proc = (busy / span) / 1.00004
    run = make_run("r", timeline, mode="lfc", cpu_proc={"aggregate_cores": proc, "n_cores_machine": 10})
    result = get_metric("cpu").compute([run])
    per_run = result.data["per_run"]["r"]
    row = result.data["by_mode"]["lfc"]
    assert per_run["trace_vs_proc"] == pytest.approx(4e-5, rel=1e-9)
    assert row["trace_vs_proc"] == pytest.approx(4e-5, rel=1e-9), (
        f"mode gap built from 4-decimal per-run values: {row['trace_vs_proc']} "
        f"(per run {per_run['trace_vs_proc']})")
    assert row["proc_cores_mean"] == round(proc, 4)
    assert row["pct_one_core"] == round(proc * 100, 1)

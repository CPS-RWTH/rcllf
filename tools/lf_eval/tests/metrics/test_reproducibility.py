# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Cross-run per-node callback sequence identity (protocol Q1)."""

from __future__ import annotations

from lf_eval.metrics import get_metric
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline
from tests.conftest import MS, EventBuilder


def _run(name, order, mode="stock-multi"):
    """Node F subscribes /imu and /scan and fires them in ``order`` ('I'/'S');
    node T is a plain timer (one callback, never informative)."""
    b = EventBuilder()
    b.add_node("F")
    b.add_subscription("F", "/imu")
    b.add_subscription("F", "/scan")
    b.add_node("T")
    b.add_timer("T", period_ms=10.0)
    for i, c in enumerate(order):
        b.sub_fire("F", "/imu" if c == "I" else "/scan", start_ns=(i + 1) * 10 * MS, duration_ns=MS)
        b.timer_fire("T", start_ns=(i + 1) * 10 * MS + 2 * MS, duration_ns=MS)
    return make_run(name, build_timeline(b.build()), mode=mode)


def _result(runs):
    return get_metric("reproducibility").compute(runs)


def test_identical_runs_are_reproducible():
    res = _result([_run("r0", "IISIS"), _run("r1", "IISIS"), _run("r2", "IISIS")])
    mode = res.data["per_mode"]["stock-multi"]
    assert mode["reproducible"] is True and mode["runs"] == 3
    assert mode["informative_nodes"] == ["F"], "a one-callback node cannot diverge"


def test_a_swapped_pair_is_found_at_its_index():
    res = _result([_run("r0", "IISIS"), _run("r1", "ISIIS")])
    mode = res.data["per_mode"]["stock-multi"]
    assert mode["reproducible"] is False
    (row,) = [r for r in mode["divergences"] if r["node"] == "F"]
    assert (row["run"], row["first_divergence"]) == ("r1", 1), row


def test_modes_are_compared_separately_and_one_run_is_not_enough():
    res = _result([_run("a", "IIS", mode="lfc"), _run("b", "IIS", mode="lfc"), _run("c", "ISI")])
    assert res.data["per_mode"]["lfc"]["reproducible"] is True
    assert res.data["per_mode"]["stock-multi"]["reproducible"] is None, "one run proves nothing"

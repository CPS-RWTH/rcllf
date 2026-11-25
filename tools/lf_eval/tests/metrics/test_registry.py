# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the metric registry — lf_eval's extension point."""

from __future__ import annotations

import pytest

from lf_eval.metrics import metric_names, resolve_metrics
from lf_eval.metrics.registry import (
    Metric,
    MetricResult,
    all_metrics,
    get_metric,
    register,
)


def test_all_v1_metrics_are_registered():
    assert set(metric_names()) >= {"e2e_latency", "determinism", "cpu", "drops"}


def test_get_metric_returns_instance_with_name():
    metric = get_metric("cpu")
    assert isinstance(metric, Metric)
    assert metric.name == "cpu"
    assert metric.summary


def test_every_metric_has_a_summary():
    for metric in all_metrics():
        assert metric.summary, f"{metric.name} has no summary"


def test_unknown_metric_lists_available():
    with pytest.raises(KeyError) as excinfo:
        get_metric("nope")
    message = str(excinfo.value)
    assert "nope" in message
    assert "cpu" in message  # the error must enumerate valid choices


def test_resolve_metrics_defaults_to_all():
    assert len(resolve_metrics(None)) == len(metric_names())
    assert len(resolve_metrics([])) == len(metric_names())


def test_resolve_metrics_selects_named_subset():
    metrics = resolve_metrics(["cpu", "drops"])
    assert [m.name for m in metrics] == ["cpu", "drops"]


def test_registering_a_new_metric_makes_it_discoverable():
    """The documented extension recipe must work end to end."""

    @register("test_only_probe")
    class ProbeMetric(Metric):
        summary = "probe"

        def compute(self, runs, system=None, chains=(), **options):
            return MetricResult(name=self.name, data={"n": len(runs)})

    try:
        assert "test_only_probe" in metric_names()
        metric = get_metric("test_only_probe")
        assert metric.name == "test_only_probe"
        result = metric.compute([])
        assert result.data == {"n": 0}
        assert result.available is True
        # Default plot implementation draws nothing rather than failing.
        assert metric.plot(result, "/tmp") == []
    finally:
        from lf_eval.metrics.registry import _REGISTRY

        _REGISTRY.pop("test_only_probe", None)


def test_duplicate_registration_is_rejected():
    from lf_eval.metrics.registry import _REGISTRY

    @register("test_only_dup")
    class First(Metric):
        summary = "first"

    try:
        with pytest.raises(ValueError, match="already registered"):

            @register("test_only_dup")
            class Second(Metric):
                summary = "second"

    finally:
        _REGISTRY.pop("test_only_dup", None)


def test_registering_a_non_metric_is_rejected():
    with pytest.raises(TypeError):

        @register("test_only_bad")
        class NotAMetric:  # noqa: D401
            pass


def test_base_metric_compute_is_abstract():
    with pytest.raises(NotImplementedError):
        Metric().compute([])


def test_metric_result_to_dict_shape():
    result = MetricResult(
        name="m",
        data={"k": 1},
        summary_rows=[{"a": 1}],
        note="hi",
        plots=["plots/x.png"],
    )
    payload = result.to_dict()
    assert payload == {
        "available": True,
        "note": "hi",
        "data": {"k": 1},
        "summary": [{"a": 1}],
        "plots": ["plots/x.png"],
    }


def test_unavailable_result_defaults():
    result = MetricResult(name="m", available=False, note="no data")
    assert result.to_dict()["available"] is False
    assert result.data == {}
    assert result.summary_rows == []

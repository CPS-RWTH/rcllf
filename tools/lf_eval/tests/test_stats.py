# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the shared statistics helpers."""

from __future__ import annotations

import math

import pytest

from lf_eval.metrics.stats import (
    coefficient_of_variation,
    ks_statistic,
    ks_2samp,
    pair_forward,
    relative_spread,
    summarize,
)


def test_summarize_basic_moments():
    stats = summarize([10.0, 20.0, 30.0], label="hop")
    assert stats is not None
    assert stats.count == 3
    assert stats.mean_us == pytest.approx(20.0)
    assert stats.median_us == pytest.approx(20.0)
    assert stats.min_us == pytest.approx(10.0)
    assert stats.max_us == pytest.approx(30.0)
    assert stats.label == "hop"


def test_summarize_requires_two_samples():
    assert summarize([]) is None
    assert summarize([1.0]) is None


def test_summarize_to_dict_converts_ms_and_hides_raw():
    stats = summarize([1000.0, 3000.0])
    payload = stats.to_dict()
    assert payload["mean_ms"] == pytest.approx(2.0)
    assert "raw_us" not in payload
    assert stats.to_dict(include_raw=True)["raw_us"] == [1000.0, 3000.0]


def test_pair_forward_takes_first_target_strictly_after():
    pairs = pair_forward([10, 20], [5, 15, 25], max_gap_ns=0)
    assert pairs == [(10, 15), (20, 25)]


def test_pair_forward_drops_pairs_beyond_max_gap():
    # 100 -> 400 is a gap of 300; with max_gap 200 it must be dropped.
    assert pair_forward([100], [400], max_gap_ns=200) == []
    assert pair_forward([100], [250], max_gap_ns=200) == [(100, 250)]


def test_pair_forward_ignores_targets_at_the_same_timestamp():
    # bisect_right => a target exactly at the anchor is not "after" it.
    assert pair_forward([100], [100], max_gap_ns=0) == []


def test_pair_forward_handles_unsorted_targets():
    assert pair_forward([10], [50, 20, 30], max_gap_ns=0) == [(10, 20)]


def test_pair_forward_empty_inputs():
    assert pair_forward([], [1, 2]) == []
    assert pair_forward([1, 2], []) == []


def test_ks_statistic_identical_samples_is_zero():
    assert ks_statistic([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]) == pytest.approx(0.0)


def test_ks_statistic_disjoint_samples_is_one():
    assert ks_statistic([1.0, 2.0], [10.0, 11.0]) == pytest.approx(1.0)


def test_ks_statistic_empty_returns_none():
    assert ks_statistic([], [1.0]) is None


def test_ks_2samp_returns_statistic_and_pvalue():
    result = ks_2samp([1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0])
    assert result is not None
    stat, pvalue = result
    assert stat == pytest.approx(0.0)
    # p-value is NaN only in the no-scipy fallback; either way it is a float.
    assert isinstance(pvalue, float)


def test_ks_2samp_without_scipy_falls_back_to_statistic(monkeypatch):
    import builtins

    real_import = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name == "scipy" or name.startswith("scipy."):
            raise ImportError("scipy blocked for this test")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    stat, pvalue = ks_2samp([1.0, 2.0], [10.0, 11.0])
    assert stat == pytest.approx(1.0)
    assert math.isnan(pvalue)


def test_relative_spread_and_cov():
    assert relative_spread([2.0, 2.0, 2.0]) == pytest.approx(0.0)
    assert relative_spread([1.0, 3.0]) == pytest.approx(1.0)  # (3-1)/2
    assert coefficient_of_variation([5.0, 5.0]) == pytest.approx(0.0)
    assert coefficient_of_variation([]) == 0.0
    # mean of zero must not divide by zero
    assert coefficient_of_variation([-1.0, 1.0]) == 0.0

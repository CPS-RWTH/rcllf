# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Shared summary statistics and causal-pairing helpers.

:func:`pair_forward` is the workhorse of the latency metrics: it reproduces the
predecessor script's binary-search pairing (``np.searchsorted`` on sorted
downstream timestamps) but as a reusable function rather than being inlined per
metric.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

@dataclass
class Summary:
    """Distribution summary in microseconds."""

    label: str
    count: int
    mean_us: float
    std_us: float
    median_us: float
    p95_us: float
    p99_us: float
    min_us: float
    max_us: float
    raw_us: list[float] = field(default_factory=list)

    def to_dict(self, include_raw: bool = False) -> dict:
        out = {
            "count": self.count,
            "mean_us": round(self.mean_us, 3),
            "std_us": round(self.std_us, 3),
            "median_us": round(self.median_us, 3),
            "p95_us": round(self.p95_us, 3),
            "p99_us": round(self.p99_us, 3),
            "min_us": round(self.min_us, 3),
            "max_us": round(self.max_us, 3),
            "mean_ms": round(self.mean_us / 1000.0, 4),
            "p99_ms": round(self.p99_us / 1000.0, 4),
        }
        if include_raw:
            out["raw_us"] = [round(v, 3) for v in self.raw_us]
        return out


def summarize(values_us: Sequence[float], label: str = "") -> Optional[Summary]:
    """Summarise a sequence of microsecond values; ``None`` if fewer than 2."""
    if len(values_us) < 2:
        return None
    arr = np.asarray(values_us, dtype=float)
    return Summary(
        label=label,
        count=int(arr.size),
        mean_us=float(np.mean(arr)),
        std_us=float(np.std(arr)),
        median_us=float(np.median(arr)),
        p95_us=float(np.percentile(arr, 95)),
        p99_us=float(np.percentile(arr, 99)),
        min_us=float(np.min(arr)),
        max_us=float(np.max(arr)),
        raw_us=[float(v) for v in arr],
    )


def pair_forward(
    anchors_ns: Sequence[int],
    targets_ns: Sequence[int],
    max_gap_ns: int = 0,
) -> list[tuple[int, int]]:
    """Pair each anchor with the first target strictly after it.

    :param anchors_ns: upstream timestamps (need not be sorted).
    :param targets_ns: downstream timestamps (need not be sorted).
    :param max_gap_ns: pairs further apart than this are dropped. ``0`` or a
        negative value (the default) keeps every pair: no fixed cutoff has a
        basis independent of the system under test.
    :returns: ``(anchor_ns, target_ns)`` pairs, one per matched anchor.
    """
    if not anchors_ns or not targets_ns:
        return []
    sorted_targets = sorted(targets_ns)
    pairs: list[tuple[int, int]] = []
    for anchor in anchors_ns:
        idx = bisect.bisect_right(sorted_targets, anchor)
        if idx >= len(sorted_targets):
            continue
        target = sorted_targets[idx]
        if max_gap_ns > 0 and target - anchor > max_gap_ns:
            continue
        pairs.append((anchor, target))
    return pairs


def ks_2samp(
    a: Sequence[float], b: Sequence[float]
) -> Optional[tuple[float, float]]:
    """Two-sample Kolmogorov-Smirnov test, or ``None`` without scipy.

    Falls back to a pure-numpy KS *statistic* with no p-value when scipy is
    absent; callers must handle a ``None`` p-value.
    """
    try:
        from scipy import stats
    except ImportError:
        stat = ks_statistic(a, b)
        return (stat, float("nan")) if stat is not None else None
    result = stats.ks_2samp(np.asarray(a, float), np.asarray(b, float))
    return (float(result.statistic), float(result.pvalue))


def ks_statistic(a: Sequence[float], b: Sequence[float]) -> Optional[float]:
    """KS statistic (max CDF distance) computed without scipy."""
    if len(a) == 0 or len(b) == 0:
        return None
    arr_a = np.sort(np.asarray(a, dtype=float))
    arr_b = np.sort(np.asarray(b, dtype=float))
    grid = np.concatenate([arr_a, arr_b])
    cdf_a = np.searchsorted(arr_a, grid, side="right") / arr_a.size
    cdf_b = np.searchsorted(arr_b, grid, side="right") / arr_b.size
    return float(np.max(np.abs(cdf_a - cdf_b)))


def relative_spread(values: Sequence[float]) -> float:
    """``(max - min) / mean`` — 0.0 for identical values, NaN-safe."""
    if not values:
        return 0.0
    arr = np.asarray(values, dtype=float)
    mean = float(np.mean(arr))
    if mean == 0:
        return 0.0
    return float((np.max(arr) - np.min(arr)) / abs(mean))


def coefficient_of_variation(values: Sequence[float]) -> float:
    """``std / mean`` — the scale-free dispersion measure."""
    if not values:
        return 0.0
    arr = np.asarray(values, dtype=float)
    mean = float(np.mean(arr))
    if mean == 0:
        return 0.0
    return float(np.std(arr) / abs(mean))

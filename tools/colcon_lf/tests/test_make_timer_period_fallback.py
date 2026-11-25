# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for ``colcon_lf.generation.lf_generator._make_timer``.

``lf_generator.py``'s null-``period_ms`` fallback warns instead of silently
substituting 1000ms; the fallback value itself is 1000ms.

The warning is emitted via ``warnings.warn`` (matching the module's existing
warning at the exec-config layer), NOT ``logging``: pytest's ``caplog`` does
not capture records under the ROS container/CI pytest-plugin stack, which made
logging-based assertions pass on the host and fail in CI.
"""

from __future__ import annotations

import warnings

import pytest

from colcon_lf.generation.lf_generator import _make_timer
from colcon_lf.node_analysis.models import TimerInfo


class TestMakeTimerNullPeriodRefuses:
    def test_null_period_ms_refuses(self):
        """A timer with an unresolved period_ms refuses generation.
        It used to fall back to a 1000 msec placeholder with a warning."""
        timer = TimerInfo(period_ms=None, callback="timer_callback")
        with pytest.raises(ValueError, match="has no period"):
            _make_timer(timer)

    def test_null_period_ms_error_names_the_callback(self):
        """The error identifies which timer callback has no period."""
        timer = TimerInfo(period_ms=None, callback="my_special_timer_cb")
        with pytest.raises(ValueError, match="my_special_timer_cb"):
            _make_timer(timer)

    def test_resolved_period_ms_emits_no_warning(self):
        """A timer with a real (non-null) period never warns — no false positives."""
        timer = TimerInfo(period_ms=200.0, callback="timer_callback")
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            result = _make_timer(timer)
        assert result.period == "200 msec"


class TestMakeTimerFractionalPeriod:
    def test_fractional_ms_period_is_exact_nsec(self):
        """lfc rejects "10.5 msec" (LF times are integers). The
        period is written in nsec from the exact decimal, the value the RID
        table carries and the binding matches exactly."""
        result = _make_timer(TimerInfo(period_ms=10.5, callback="grid_deadline"))
        assert result.period == "10500000 nsec", (
            f"fractional period emitted as {result.period!r}, which lfc rejects"
        )

    def test_8_2_ms_is_exactly_8200000_nsec(self):
        result = _make_timer(TimerInfo(period_ms=8.2, callback="t"))
        assert result.period == "8200000 nsec", result.period

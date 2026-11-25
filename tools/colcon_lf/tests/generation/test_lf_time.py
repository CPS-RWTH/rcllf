# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the LF time-literal parser."""

from __future__ import annotations

import pytest

from colcon_lf.generation.lf_emitter import _ns_to_lf_time
from colcon_lf.generation.lf_time import parse_lf_time_ns


class TestLfSpelling:
    """The long-form spelling used by --after-delay and execution.yaml."""

    @pytest.mark.parametrize(
        "text,expected",
        [
            ("1 msec", 1_000_000),
            ("5 msec", 5_000_000),
            ("500 usec", 500_000),
            ("100 nsec", 100),
            ("2 sec", 2_000_000_000),
            ("999 sec", 999_000_000_000),
        ],
    )
    def test_parses(self, text, expected):
        assert parse_lf_time_ns(text) == expected

    def test_no_space_is_accepted(self):
        assert parse_lf_time_ns("1msec") == 1_000_000

    def test_plural_units(self):
        assert parse_lf_time_ns("2 secs") == 2_000_000_000
        assert parse_lf_time_ns("3 msecs") == 3_000_000

    def test_case_insensitive(self):
        assert parse_lf_time_ns("1 MSEC") == 1_000_000


class TestTerseSpelling:
    """The short spelling federation YAML already used (partition.py)."""

    @pytest.mark.parametrize(
        "text,expected",
        [("5ms", 5_000_000), ("100ns", 100), ("2s", 2_000_000_000), ("50us", 50_000)],
    )
    def test_parses(self, text, expected):
        assert parse_lf_time_ns(text) == expected


class TestZeroAndEmpty:
    def test_bare_zero_needs_no_unit(self):
        assert parse_lf_time_ns("0") == 0

    def test_zero_with_unit(self):
        assert parse_lf_time_ns("0 msec") == 0

    def test_none_is_none(self):
        """None means 'not configured', distinct from zero."""
        assert parse_lf_time_ns(None) is None

    def test_empty_string_is_none(self):
        assert parse_lf_time_ns("") is None
        assert parse_lf_time_ns("   ") is None

    def test_int_passthrough(self):
        assert parse_lf_time_ns(0) == 0
        assert parse_lf_time_ns(1_000_000) == 1_000_000


class TestRejection:
    """A bad value must raise, never silently default (an earlier bug)."""

    @pytest.mark.parametrize(
        "bad",
        ["garbage", "msec", "1 fortnight", "--", "1.5.2 msec", "abc msec"],
    )
    def test_unparseable_raises(self, bad):
        with pytest.raises(ValueError):
            parse_lf_time_ns(bad)

    def test_bare_nonzero_number_raises(self):
        """An unsuffixed delay is ambiguous; guessing a unit hides real errors."""
        with pytest.raises(ValueError, match="needs a unit"):
            parse_lf_time_ns("5")

    def test_unknown_unit_names_the_unit(self):
        with pytest.raises(ValueError, match="unknown time unit"):
            parse_lf_time_ns("1 fortnight")

    def test_negative_int_raises(self):
        with pytest.raises(ValueError, match="negative"):
            parse_lf_time_ns(-1)

    def test_bool_raises(self):
        """bool is an int subclass; must not parse as 0/1 ns."""
        with pytest.raises(ValueError):
            parse_lf_time_ns(True)

    def test_fractional_nanosecond_raises(self):
        with pytest.raises(ValueError, match="whole number"):
            parse_lf_time_ns("0.5 nsec")

    def test_error_includes_source(self):
        with pytest.raises(ValueError, match="execution.yaml"):
            parse_lf_time_ns("garbage", source="execution.yaml")


class TestRoundTrip:
    """parse_lf_time_ns is the inverse of the emitter's _ns_to_lf_time."""

    @pytest.mark.parametrize(
        "ns", [0, 100, 1_000, 500_000, 1_000_000, 5_000_000, 2_000_000_000],
    )
    def test_emitter_output_reparses(self, ns):
        assert parse_lf_time_ns(_ns_to_lf_time(ns)) == ns

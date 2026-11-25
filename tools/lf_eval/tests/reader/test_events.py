# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for event records and field coercion."""

from __future__ import annotations

import pytest

from lf_eval.reader.events import DiscardStats, TraceEvent, coerce_field


def test_pointer_fields_parse_as_int():
    assert coerce_field("callback", "0x58AB64082168") == 0x58AB64082168
    assert coerce_field("node_handle", "0x15B5E960") == 0x15B5E960


def test_string_fields_get_quotes_stripped():
    assert coerce_field("node_name", '"FrontLidarDriver"') == "FrontLidarDriver"
    assert coerce_field("topic_name", '"/sensor_data"') == "/sensor_data"


def test_symbol_field_keeps_inner_punctuation():
    symbol = '"std::_Bind<void (NodeA::*(NodeA*))()>"'
    assert coerce_field("symbol", symbol).startswith("std::_Bind")


def test_integer_fields_parse_as_int():
    assert coerce_field("period", "100000000") == 100_000_000
    assert coerce_field("taken", "0") == 0
    assert coerce_field("queue_depth", "10") == 10


def test_negative_integers_parse():
    assert coerce_field("timestamp", "-5") == -5


def test_already_typed_values_pass_through():
    assert coerce_field("period", 42) == 42
    assert coerce_field("period", 1.5) == 1.5


# ── DiscardStats (M7) ─────────────────────────────────────────────────────


def test_discard_stats_defaults_to_zero_and_known():
    stats = DiscardStats()
    assert stats.total == 0
    assert stats.any_discarded is False
    assert stats.known is True


def test_discard_stats_total_sums_events_and_packets():
    stats = DiscardStats(discarded_events=3, discarded_packets=2)
    assert stats.total == 5
    assert stats.any_discarded is True


def test_discard_stats_unknown_total_is_none():
    stats = DiscardStats(known=False)
    assert stats.total is None
    # Unknown must never be conflated with "confirmed zero".
    assert stats.any_discarded is False


def test_discard_stats_to_dict_reports_none_when_unknown():
    stats = DiscardStats(discarded_events=7, known=False)
    d = stats.to_dict()
    assert d["known"] is False
    assert d["discarded_events"] is None
    assert d["discarded_packets"] is None


def test_discard_stats_to_dict_reports_counts_when_known():
    stats = DiscardStats(discarded_events=1, discarded_packets=0, known=True)
    d = stats.to_dict()
    assert d == {"known": True, "discarded_events": 1, "discarded_packets": 0}


def test_unknown_hex_field_still_parses_as_int():
    assert coerce_field("mystery", "0xFF") == 255


def test_unparseable_pointer_falls_back_to_text():
    assert coerce_field("callback", "0xZZ") == "0xZZ"


def test_trace_event_get_and_getitem():
    event = TraceEvent(name="callback_start", timestamp_ns=5, fields={"callback": 7})
    assert event.get("callback") == 7
    assert event["callback"] == 7
    assert event.get("missing") is None
    assert event.get("missing", "dflt") == "dflt"
    with pytest.raises(KeyError):
        event["missing"]


def test_trace_event_is_frozen():
    event = TraceEvent(name="x", timestamp_ns=1)
    with pytest.raises(Exception):
        event.name = "y"  # type: ignore[misc]

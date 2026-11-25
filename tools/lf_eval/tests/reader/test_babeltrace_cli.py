# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the babeltrace text-output parser.

All sample lines are **verbatim** from a real reference-system trace of a
stock ROS 2 run (only the host name is neutralized), so the parser is
tested against the exact format it must handle in production.
"""

from __future__ import annotations

import pytest

from lf_eval.reader import babeltrace_cli
from lf_eval.reader.babeltrace_cli import _DAY_NS, _split_top_level, parse_line, read_events
from lf_eval.reader.events import DiscardStats

CALLBACK_START = (
    "[00:24:58.422017524] (+?.?????????) host01 ros2:callback_start: "
    "{ cpu_id = 5 }, { callback = 0x58AB64082168, is_intra_process = 0 }"
)
NODE_INIT = (
    "[00:25:00.707905850] (+0.000057250) host01 ros2:rcl_node_init: { cpu_id = 21 }, "
    '{ node_handle = 0x641E26823680, rmw_handle = 0x641E269A38F0, '
    'node_name = "FrontLidarDriver", namespace = "/" }'
)
RMW_TAKE = (
    "[00:24:58.427010060] (+0.004962736) host01 ros2:rmw_take: { cpu_id = 5 }, "
    "{ rmw_subscription_handle = 0x58AB641F3010, message = 0x746EB8000D00, "
    "source_timestamp = 0, taken = 0 }"
)
TIMER_INIT = (
    "[00:25:00.714982635] (+0.000146870) host01 ros2:rcl_timer_init: { cpu_id = 21 }, "
    "{ timer_handle = 0x641E26B164A0, period = 100000000 }"
)
# The gid array is what a naive `\{(.+)\}` regex mis-captures.
RMW_PUBLISHER_INIT = (
    "[00:24:59.509218711] (+0.008779743) host01 ros2:rmw_publisher_init: { cpu_id = 18 }, "
    "{ rmw_publisher_handle = 0x15A36210, gid = [ [0] = 1, [1] = 15, [2] = 202 ] }"
)
CALLBACK_REGISTER = (
    "[00:24:59.512366838] (+0.000014650) host01 ros2:rclcpp_callback_register: "
    '{ cpu_id = 18 }, { callback = 0x15AD8AF0, symbol = "rclcpp::ParameterService::'
    'ParameterService(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>, '
    'rclcpp::QoS const&)::{lambda(std::shared_ptr<rmw_request_id_s>)#1}" }'
)


def test_parses_callback_start():
    event = parse_line(CALLBACK_START)
    assert event is not None
    assert event.name == "callback_start"
    assert event.fields["callback"] == 0x58AB64082168
    assert event.fields["is_intra_process"] == 0
    assert event.cpu_id == 5


def test_timestamp_is_nanoseconds_since_midnight():
    event = parse_line(CALLBACK_START)
    expected = (0 * 3600 + 24 * 60 + 58) * 1_000_000_000 + 422_017_524
    assert event.timestamp_ns == expected


def test_parses_string_fields():
    event = parse_line(NODE_INIT)
    assert event.name == "rcl_node_init"
    assert event.fields["node_name"] == "FrontLidarDriver"
    assert event.fields["namespace"] == "/"
    assert event.fields["node_handle"] == 0x641E26823680


def test_parses_taken_flag_as_zero_not_none():
    event = parse_line(RMW_TAKE)
    assert event.fields["taken"] == 0
    assert event.fields["rmw_subscription_handle"] == 0x58AB641F3010


def test_parses_timer_period():
    event = parse_line(TIMER_INIT)
    assert event.fields["period"] == 100_000_000


def test_array_field_does_not_break_payload_parsing():
    event = parse_line(RMW_PUBLISHER_INIT)
    assert event is not None
    assert event.fields["rmw_publisher_handle"] == 0x15A36210
    # gid array element indices must not overwrite the real handle field.
    assert event.cpu_id == 18


def test_symbol_with_braces_and_commas_is_captured():
    event = parse_line(CALLBACK_REGISTER)
    assert event.name == "rclcpp_callback_register"
    assert event.fields["callback"] == 0x15AD8AF0
    assert "ParameterService" in event.fields["symbol"]


def test_non_event_lines_return_none():
    assert parse_line("") is None
    assert parse_line("some babeltrace warning") is None
    assert parse_line("[bad timestamp] junk") is None


def test_split_top_level_separates_context_and_payload():
    groups = _split_top_level("{ cpu_id = 5 }, { callback = 0x1 }")
    assert len(groups) == 2
    assert "cpu_id" in groups[0]
    assert "callback" in groups[1]


def test_split_top_level_ignores_braces_inside_strings():
    groups = _split_top_level('{ cpu_id = 1 }, { symbol = "a{b}c" }')
    assert len(groups) == 2
    assert 'symbol = "a{b}c"' in groups[1]


def test_midnight_rollover_unwraps_forward():
    late = "[23:59:59.900000000] (+0.0) h ros2:callback_start: { cpu_id = 0 }, { callback = 0x1 }"
    early = "[00:00:00.100000000] (+0.0) h ros2:callback_start: { cpu_id = 0 }, { callback = 0x1 }"
    first = parse_line(late)
    second = parse_line(early, prev_ns=first.timestamp_ns)
    # Time must move forward by 200 ms, not backward by nearly a day.
    assert second.timestamp_ns - first.timestamp_ns == pytest.approx(200_000_000)
    assert second.timestamp_ns > _DAY_NS


def test_no_rollover_for_normal_ordering():
    a = parse_line("[01:00:00.000000000] (+0.0) h ros2:x: { cpu_id = 0 }, { a = 1 }")
    b = parse_line(
        "[01:00:01.000000000] (+0.0) h ros2:x: { cpu_id = 0 }, { a = 1 }",
        prev_ns=a.timestamp_ns,
    )
    assert b.timestamp_ns - a.timestamp_ns == 1_000_000_000


def test_event_without_payload_treats_group_as_context():
    line = "[01:00:00.000000000] (+0.0) h ros2:rcl_init: { cpu_id = 3 }"
    event = parse_line(line)
    assert event is not None
    assert event.cpu_id == 3
    assert event.fields == {}


# ── read_events / discard cross-check honesty (M7) ───────────────────────


def test_read_events_marks_discards_unknown(monkeypatch, tmp_path):
    """The CLI backend cannot detect discards, so it must say so honestly."""
    monkeypatch.setattr(babeltrace_cli, "cli_binary", lambda: "true")
    discards = DiscardStats()
    list(read_events(tmp_path, discards=discards))
    assert discards.known is False


def test_read_events_without_discards_sink_does_not_error(monkeypatch, tmp_path):
    monkeypatch.setattr(babeltrace_cli, "cli_binary", lambda: "true")
    assert list(read_events(tmp_path, discards=None)) == []


def test_read_events_raises_when_no_binary(monkeypatch, tmp_path):
    monkeypatch.setattr(babeltrace_cli, "cli_binary", lambda: None)
    with pytest.raises(RuntimeError):
        list(read_events(tmp_path))

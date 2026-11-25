# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for callback attribution and timeline construction."""

from __future__ import annotations

import pytest

from lf_eval.model.timeline import (
    build_timeline,
    class_from_symbol,
    is_timer_symbol,
)
from lf_eval.reader.events import TraceEvent

from tests.conftest import MS, EventBuilder


def test_handle_chain_attributes_timer_callbacks(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    assert timeline.timer_invocations["NodeA"]
    assert len(timeline.timer_invocations["NodeA"]) == 20


def test_handle_chain_attributes_subscription_topics(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    assert ("NodeB", "/a_out") in timeline.topic_invocations
    assert ("NodeC", "/b_out") in timeline.topic_invocations


def test_timer_period_is_recovered_from_rcl_timer_init(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    timer_cbs = [c for c in timeline.callbacks.values() if c.kind == "timer"]
    assert timer_cbs
    assert timer_cbs[0].period_ns == 100 * MS


def test_invocations_merge_timer_and_subscription():
    b = EventBuilder()
    b.add_node("N")
    b.add_timer("N", 10.0)
    b.add_subscription("N", "/in")
    b.timer_fire("N", 1000, 100)
    b.sub_fire("N", "/in", 2000, 100)
    timeline = build_timeline(b.build())
    assert len(timeline.invocations["N"]) == 2
    assert len(timeline.timer_invocations["N"]) == 1
    assert len(timeline.sub_invocations["N"]) == 1


def test_invocations_are_sorted_by_start(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    for invs in timeline.invocations.values():
        starts = [inv.start_ns for inv in invs]
        assert starts == sorted(starts)


def test_unmatched_start_is_counted():
    b = EventBuilder()
    b.add_node("N")
    cb = b.add_timer("N", 10.0)
    # A start with no matching end.
    b.events.append(TraceEvent("callback_start", 5000, {"callback": cb}))
    timeline = build_timeline(b.build())
    assert timeline.total_callback_starts == 1
    assert timeline.unmatched_starts == 1
    assert not timeline.invocations.get("N")


def test_end_without_start_is_ignored():
    b = EventBuilder()
    b.add_node("N")
    cb = b.add_timer("N", 10.0)
    b.events.append(TraceEvent("callback_end", 5000, {"callback": cb}))
    timeline = build_timeline(b.build())
    assert timeline.total_callback_starts == 0
    assert not timeline.invocations.get("N")


def test_invocation_duration_helpers():
    b = EventBuilder()
    b.add_node("N")
    b.add_timer("N", 10.0)
    b.timer_fire("N", 1000, 3000)
    timeline = build_timeline(b.build())
    inv = timeline.invocations["N"][0]
    assert inv.duration_ns == 3000
    assert inv.duration_us == pytest.approx(3.0)


def test_node_filter_excludes_infrastructure():
    b = EventBuilder()
    b.add_node("AppNode")
    b.add_node("rosbag2_recorder")
    b.add_timer("AppNode", 10.0)
    b.add_timer("rosbag2_recorder", 10.0)
    b.timer_fire("AppNode", 1000, 100)
    b.timer_fire("rosbag2_recorder", 2000, 100)
    timeline = build_timeline(b.build(), node_filter={"AppNode"})
    assert timeline.nodes == ["AppNode"]


def test_node_filter_is_case_insensitive():
    b = EventBuilder()
    b.add_node("AppNode")
    b.add_timer("AppNode", 10.0)
    b.timer_fire("AppNode", 1000, 100)
    timeline = build_timeline(b.build(), node_filter={"appnode"})
    assert timeline.nodes == ["AppNode"]


def test_symbol_fallback_attributes_node_without_handle_chain():
    """No node/timer init events: attribution must fall back to the symbol."""
    events = [
        TraceEvent(
            "rclcpp_callback_register",
            0,
            {
                "callback": 0x100,
                "symbol": "std::_Bind<void (LateNode::*(LateNode*))()>",
            },
        ),
        TraceEvent("callback_start", 1000, {"callback": 0x100}),
        TraceEvent("callback_end", 2000, {"callback": 0x100}),
    ]
    timeline = build_timeline(events)
    assert timeline.nodes == ["LateNode"]
    # No placeholder in the symbol => classified as a timer.
    assert timeline.timer_invocations["LateNode"]


def test_symbol_fallback_classifies_subscription_by_placeholder():
    events = [
        TraceEvent(
            "rclcpp_callback_register",
            0,
            {
                "callback": 0x200,
                "symbol": (
                    "std::_Bind<void (SubNode::*(SubNode*, std::_Placeholder<1>))"
                    "(std::shared_ptr<Msg>)>"
                ),
            },
        ),
        TraceEvent("callback_start", 1000, {"callback": 0x200}),
        TraceEvent("callback_end", 2000, {"callback": 0x200}),
    ]
    timeline = build_timeline(events)
    assert timeline.sub_invocations["SubNode"]


def test_unattributable_callback_is_dropped():
    """A lambda with no Class:: pattern cannot be attributed and is skipped."""
    events = [
        TraceEvent(
            "rclcpp_callback_register",
            0,
            {"callback": 0x300, "symbol": "{lambda()#1}"},
        ),
        TraceEvent("callback_start", 1000, {"callback": 0x300}),
        TraceEvent("callback_end", 2000, {"callback": 0x300}),
    ]
    timeline = build_timeline(events)
    assert timeline.nodes == []
    assert timeline.total_callback_starts == 1


def test_handle_chain_wins_over_symbol():
    """The authoritative node name must not be overwritten by the symbol regex."""
    b = EventBuilder()
    b.add_node("RealName")
    node_handle = b.nodes["RealName"]
    timer_handle = 0x900
    callback = 0x901
    b.events.extend(
        [
            TraceEvent("rcl_timer_init", 0, {"timer_handle": timer_handle, "period": 10 * MS}),
            TraceEvent(
                "rclcpp_timer_link_node",
                0,
                {"timer_handle": timer_handle, "node_handle": node_handle},
            ),
            TraceEvent(
                "rclcpp_timer_callback_added",
                0,
                {"timer_handle": timer_handle, "callback": callback},
            ),
            TraceEvent(
                "rclcpp_callback_register",
                0,
                {"callback": callback, "symbol": "void (WrongName::*())"},
            ),
            TraceEvent("callback_start", 1000, {"callback": callback}),
            TraceEvent("callback_end", 2000, {"callback": callback}),
        ]
    )
    timeline = build_timeline(b.build())
    assert timeline.nodes == ["RealName"]


def test_timeline_span_and_cpu_busy(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    assert timeline.duration_ns > 0
    # 60 invocations x 1 ms each.
    assert timeline.cpu_busy_ns() == 60 * MS


def test_cpu_busy_can_be_restricted_to_nodes(pipeline_builder):
    timeline = build_timeline(pipeline_builder.build())
    assert timeline.cpu_busy_ns(["NodeA"]) == 20 * MS


def test_empty_event_stream_yields_empty_timeline():
    timeline = build_timeline([])
    assert timeline.nodes == []
    assert timeline.duration_ns == 0
    assert timeline.cpu_busy_ns() == 0


def test_class_from_symbol():
    assert class_from_symbol("std::_Bind<void (Foo::*(Foo*))()>") == "Foo"
    assert class_from_symbol("no class here") is None


def test_is_timer_symbol():
    assert is_timer_symbol("void (Foo::*())")
    assert not is_timer_symbol("void (Foo::*(std::_Placeholder<1>))")

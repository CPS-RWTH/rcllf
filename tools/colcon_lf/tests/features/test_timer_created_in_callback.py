# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H006 — timer created outside the constructor/lifecycle hooks.

These tests cover the *diagnostic* half, driven off a hand-built graph. The
clang detection half (`_find_out_of_scope_timers`) needs real rclcpp headers
and lives in `tests/test_clang_parser.py`
(`test_cancel_timer_created_in_goal_response_callback_is_not_extracted` /
`test_cancel_timer_recorded_as_unbound`).

To falsify: change `detect()` in `features/modules/timer_created_in_callback.py`
to read `node_info.timers` instead of `node_info.unbound_timers` and
`test_fires_for_unbound_timer` fails (the fixture's one `timers` entry is a
constructor-created timer, unrelated to the hazard).
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.modules.timer_created_in_callback import CODE_H006, detect
from colcon_lf.features.registry import severity_for
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import NodeInfo, TimerInfo


def _graph(node: NodeInfo, key: str = "n") -> SystemGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    return SystemGraph(
        name="s",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={key: node},
        communication=SystemCommunication(),
    )


def _node(*, unbound_timers=None, timers=None) -> NodeInfo:
    return NodeInfo(
        name="n",
        class_name="N",
        package="test_pkg",
        source_file=Path("/fake/n.cpp"),
        timers=timers or [],
        unbound_timers=unbound_timers or [],
    )


class TestOutOfScopeTimerHazard:
    def test_quiet_with_no_unbound_timers(self):
        node = _node(
            timers=[TimerInfo(period_ms=2000.0, callback="send_goal", created_in="constructor")],
        )
        assert detect(DetectionContext(graph=_graph(node))) == []

    def test_fires_for_unbound_timer(self):
        node = _node(
            timers=[TimerInfo(period_ms=2000.0, callback="send_goal", created_in="constructor")],
            unbound_timers=[
                TimerInfo(
                    period_ms=3000.0,
                    callback="cancel_timer_callback",
                    member_variable="cancel_timer_",
                    created_in="goal_response_callback",
                ),
            ],
        )
        diags = detect(DetectionContext(graph=_graph(node, key="action_client")))
        assert len(diags) == 1
        d = diags[0]
        assert d.code == CODE_H006
        assert d.node == "action_client"
        assert "cancel_timer_" in d.message
        assert "cancel_timer_callback" in d.message
        assert "goal_response_callback" in d.message

    def test_severity_is_warning(self):
        assert severity_for(CODE_H006) == Severity.WARNING

    def test_multiple_unbound_timers_each_get_a_diagnostic(self):
        node = _node(
            unbound_timers=[
                TimerInfo(period_ms=1.0, callback="a", member_variable="a_", created_in="m1"),
                TimerInfo(period_ms=2.0, callback="b", member_variable="b_", created_in="m2"),
            ],
        )
        diags = detect(DetectionContext(graph=_graph(node)))
        assert len(diags) == 2
        assert any("m1" in d.message for d in diags)
        assert any("m2" in d.message for d in diags)

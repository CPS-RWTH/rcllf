# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H008, a sleep on the node's ROS clock.

The token scan runs on a stand-in token stream (spelling and line), the
diagnostic on a hand-built graph. The real-source case is the playground
example ``clock_fixture/ros_sleep``.

To falsify: drop the ``get_clock ( ) ->`` prefix check in
``_scan_ros_sleep_calls`` and ``test_steady_member_sleep_is_not_reported``
fails with the steady sleep reported.
"""

from __future__ import annotations

from types import SimpleNamespace

from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.modules.ros_time_sleep import CODE_H008, detect
from colcon_lf.features.registry import severity_for
from colcon_lf.node_analysis.clang_parser import _scan_ros_sleep_calls
from tests.features.conftest import make_graph, make_node


def _tu(source: str):
    """A stand-in translation unit: one token per whitespace-split word."""
    tokens = []
    for line_no, line in enumerate(source.splitlines(), start=1):
        for word in line.split():
            tokens.append(SimpleNamespace(spelling=word, location=SimpleNamespace(line=line_no)))
    return SimpleNamespace(cursor=SimpleNamespace(get_tokens=lambda: iter(tokens)))


class TestScan:
    def test_finds_sleep_for_and_sleep_until_with_lines(self):
        tu = _tu(
            "void f ( ) {\n"
            "  this -> get_clock ( ) -> sleep_for ( d ) ;\n"
            "  get_clock ( ) -> sleep_until ( t ) ;\n"
            "}\n"
        )
        assert _scan_ros_sleep_calls(tu) == ["sleep_for:2", "sleep_until:3"]

    def test_steady_member_sleep_is_not_reported(self):
        tu = _tu("steady_clock_ . sleep_for ( d ) ;\nclock_ -> sleep_for ( d ) ;\n")
        assert _scan_ros_sleep_calls(tu) == []

    def test_unreadable_tokens_give_nothing(self):
        def boom():
            raise RuntimeError("no tokens")

        tu = SimpleNamespace(cursor=SimpleNamespace(get_tokens=boom))
        assert _scan_ros_sleep_calls(tu) == []


class TestDiagnostic:
    def test_quiet_without_sleep_calls(self):
        graph = make_graph({"clock_sub": make_node("clock_sub")})
        assert detect(DetectionContext(graph=graph)) == []

    def test_fires_once_per_call_with_source_line(self):
        node = make_node("clock_ros_sleep")
        node.detected_ros_sleep_calls = ["sleep_for:55"]
        diags = detect(DetectionContext(graph=make_graph({"clock_ros_sleep": node})))
        assert len(diags) == 1
        d = diags[0]
        assert d.code == CODE_H008
        assert d.node == "clock_ros_sleep"
        assert "get_clock()->sleep_for() at clock_ros_sleep.cpp:55" in d.message
        assert "#197" not in d.message

    def test_severity_is_warning(self):
        assert severity_for(CODE_H008) == Severity.WARNING

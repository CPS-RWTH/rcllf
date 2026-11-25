# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H007, a callback in a form the analyzer cannot model.

The clang half (a lambda gets a synthesized callback, a ``std::function``
member leaves ``callback == ""`` with ``callback_expr`` set) is tested on the
real sources in ``tests/test_clang_parser.py::TestLambdaCallbacks`` and end to
end by the playground case ``handle_matching_example/opaque_callback``.

To falsify: make ``detect()`` in ``features/modules/opaque_callback.py`` return
``[]`` and ``test_fires_for_opaque_timer`` and
``test_fires_for_opaque_subscription`` fail.
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.modules.opaque_callback import CODE_H007, detect
from colcon_lf.features.registry import severity_for
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import NodeInfo, SubscriberInfo, TimerInfo


def _graph(timers: list[TimerInfo], subscribers: list[SubscriberInfo] = ()) -> SystemGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    node = NodeInfo(
        name="n",
        class_name="N",
        package="test_pkg",
        source_file=Path("/fake/n.cpp"),
        timers=timers,
        subscribers=list(subscribers),
    )
    return SystemGraph(
        name="s",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={"/n": node},
        communication=SystemCommunication(),
    )


def test_quiet_for_bound_member_and_lambda_callbacks():
    graph = _graph(
        [TimerInfo(period_ms=100.0, callback="tick"),
         TimerInfo(period_ms=100.0, callback="lambda_timer_1")],
        [SubscriberInfo(topic="t", message_type="M", callback="lambda_subscription_0")],
    )
    assert detect(DetectionContext(graph=graph)) == []


def test_fires_for_opaque_timer():
    graph = _graph(
        [
            TimerInfo(period_ms=100.0, callback="tick"),
            TimerInfo(period_ms=100.0, callback="", member_variable="timer_",
                      callback_expr="tick_"),
        ]
    )
    diagnostics = detect(DetectionContext(graph=graph))
    assert [d.code for d in diagnostics] == [CODE_H007]
    assert diagnostics[0].node == "/n"
    assert "timer 'timer_' (period 100 ms) with callback argument `tick_`" in (
        diagnostics[0].message
    )
    assert severity_for(CODE_H007) == Severity.ERROR


def test_fires_for_opaque_subscription():
    graph = _graph(
        [],
        [SubscriberInfo(topic="chatter", message_type="M", callback="",
                        member_variable="sub_", callback_expr="&on_message")],
    )
    diagnostics = detect(DetectionContext(graph=graph))
    assert [d.code for d in diagnostics] == [CODE_H007]
    assert diagnostics[0].topic == "chatter"
    assert "subscription 'sub_' on 'chatter' with callback argument `&on_message`" in (
        diagnostics[0].message
    )

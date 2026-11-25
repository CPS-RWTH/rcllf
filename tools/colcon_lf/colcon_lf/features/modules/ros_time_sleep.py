# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H008, a sleep on the node's ROS clock.

Under rcllf a node's ``get_clock()`` follows LF logical time, which stands
still while a reaction runs. ``get_clock()->sleep_for(...)`` or
``sleep_until(...)`` inside a callback therefore waits for a time that never
comes, and the program used to hang without a word. The runtime clock gate
(``ros2_lf_binding/src/clock_gate.cpp``) now ends the program with a message
naming the node and callback. This module reports the call at analyze time,
where the source shows it.

WARNING, not ERROR: the scan sees the call, not whether it runs in a
callback. A sleep on a thread the node starts itself waits for logical time
and returns, so generation goes ahead and the runtime gate decides.
"""

from __future__ import annotations

from pathlib import PurePath

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H008 = "RCLLF-H008"


def _message(node_key: str, source: str, call: str, line: str) -> str:
    return (
        f"node '{node_key}' calls get_clock()->{call}() at {source}:{line}. The "
        "node's clock follows LF logical time, which does not advance while a "
        "reaction runs, so inside a callback the sleep never returns and rcllf "
        "stops the program with a FATAL message. Use a timer for a "
        "delay, or an RCL_STEADY_TIME clock for a wall-clock wait."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H008 for every ``get_clock()->sleep_for/sleep_until`` call."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        source = getattr(node_info, "source_file", None)
        source_name = PurePath(str(source)).name if source else "?"
        for entry in getattr(node_info, "detected_ros_sleep_calls", []) or []:
            call, _, line = entry.partition(":")
            diagnostics.append(
                Diagnostic(
                    code=CODE_H008,
                    node=node_key,
                    topic=None,
                    message=_message(node_key, source_name, call, line),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="clock.ros_time_sleep",
        title="Sleep on the node's ROS clock",
        support=Support.WARN,
        codes=(CODE_H008,),
        severity={CODE_H008: Severity.WARNING},
        detect=detect,
        conversion=(
            "warning: get_clock()->sleep_for/sleep_until waits on LF logical "
            "time, which does not advance inside a reaction. The runtime "
            "clock gate stops the program with a message naming the node and "
            "callback when the sleep runs in a callback."
        ),
        playground=PlaygroundCase(
            example="clock_fixture/ros_sleep",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H008,),
        ),
        docs_entry=(
            "Sleep on the node's ROS clock (warning, refused at runtime inside "
            "a callback)"
        ),
    )
)

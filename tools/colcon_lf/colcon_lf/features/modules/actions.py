# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.actions, ROS2 actions, a SUPPORTED (no-hazard) feature.

Action servers and clients are a supported ROS2 to LF conversion. The hazard
for a server that sends from its own thread lives in its own module
(``thread_action_server.py``, RCLLF-H011), so ``detect`` here is ``[]``. See
``timers.py`` for the "presence is structural, not diagnostic" rationale
shared by every SUPPORTED-with-no-codes module in this file set.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """No hazard class for actions today. It always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.actions",
        title="Actions",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "A ROS2 action server's goal, cancel and accepted callbacks and the "
            "client's response, feedback and result callbacks convert to LF "
            "reactions. Each request, response, feedback and result a reaction "
            "sends becomes one LF event on its connection, and the receiving "
            "reaction takes every sample that step sent, so delivery tags repeat "
            "in one process and across federates for LF-origin pairs. "
            "A server that sends from its own thread is refused "
            "(RCLLF-H011)."
        ),
        playground=PlaygroundCase(
            example="action_example/bringup",
            # WARN, not PASS: action_example/bringup's own
            # ActionClientNode creates cancel_timer_ inside
            # goal_response_callback, which RCLLF-H006
            # (timer_created_in_callback.py) now co-fires a WARNING on for
            # this same launch file. core.actions itself still owns no
            # codes, so expected_codes stays empty, a trivially-satisfied
            # subset. This only reflects the outcome another module's
            # diagnostic now produces on the shared example.
            expected_outcome=Outcome.WARN,
            expected_codes=(),
        ),
        docs_entry=(
            "Actions (supported for servers stepped by LF callbacks, with "
            "repeatable delivery tags. A server that sends from its "
            "own thread is refused, RCLLF-H011)"
        ),
    )
)

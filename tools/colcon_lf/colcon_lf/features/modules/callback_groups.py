# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.callback_groups — ROS2 callback groups, a SUPPORTED (no-hazard) feature.

Callback groups (reentrant/mutually-exclusive) are a supported ROS2 → LF
conversion (LF's reaction ordering already provides the relevant
concurrency/exclusivity guarantees). No hazard class exists for this feature
today, so ``detect`` is trivially ``[]`` — see ``timers.py`` for the
"presence is structural, not diagnostic" rationale shared by every
SUPPORTED-with-no-codes module in this file set.
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
    """No hazard class for callback groups today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.callback_groups",
        title="Callback groups",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "Reentrant/mutually-exclusive callback group membership is honored "
            "by the generated reaction ordering — LF's own scheduling already "
            "provides the relevant exclusivity guarantees."
        ),
        playground=PlaygroundCase(
            example="callback_group_example/callback_group_node",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Callback groups (supported)",
    )
)

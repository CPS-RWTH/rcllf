# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.parameters — ROS2 node parameters, a SUPPORTED (no-hazard) feature.

Declared/get/set node parameters are a supported ROS2 → LF conversion (the
parameter API is bridged through the runtime binding unchanged). No hazard
class exists for this feature today, so ``detect`` is trivially ``[]`` — see
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
    """No hazard class for parameters today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.parameters",
        title="Node parameters",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "Declared node parameters and get/set calls are bridged through "
            "the runtime binding unchanged — no LF-specific conversion is needed."
        ),
        playground=PlaygroundCase(
            example="parameter_example/parameter_node",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Node parameters (supported)",
    )
)

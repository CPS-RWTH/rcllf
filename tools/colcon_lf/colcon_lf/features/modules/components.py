# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.components — ROS2 composable components, a SUPPORTED (no-hazard) feature.

Composable (rclcpp_components) nodes are a supported ROS2 → LF conversion.
No hazard class exists for this feature today, so ``detect`` is trivially
``[]`` — see ``timers.py`` for the "presence is structural, not diagnostic"
rationale shared by every SUPPORTED-with-no-codes module in this file set.
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
    """No hazard class for composable components today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.components",
        title="Composable components",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "A composable (rclcpp_components) node converts the same way as a "
            "standalone node — component registration is a packaging detail, "
            "not a structural difference for the LF conversion."
        ),
        playground=PlaygroundCase(
            example="component_example/component_node",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Composable components (supported)",
    )
)

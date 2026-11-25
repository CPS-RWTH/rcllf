# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.timers — ROS2 wall timers, a SUPPORTED (no-hazard) feature.

Timers are a fully supported ROS2 → LF conversion (a timer callback becomes
an LF timer-triggered reaction); there is no known hazard class for this
feature, so this module owns no diagnostic codes. ``detect`` is trivially
``[]`` — module "presence" in a system is a structural fact discoverable
from the extracted graph/launch, not a diagnostic, so there is nothing for a
detector to emit here. The module exists so ``core.timers`` has a registry
entry, a playground case, and a features-table row like every other
capability (Phase 7 design decision, feature-registry-spine).
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
    """No hazard class for timers today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.timers",
        title="Wall timers",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion="A ROS2 wall timer callback converts directly to an LF timer-triggered reaction.",
        playground=PlaygroundCase(
            example="timer_example/timer_node",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Wall timers (supported)",
    )
)

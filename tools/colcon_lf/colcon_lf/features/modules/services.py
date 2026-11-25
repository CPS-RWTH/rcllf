# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.services — ROS2 services, a SUPPORTED (no-hazard) feature.

Service servers/clients are a supported ROS2 → LF conversion. No hazard
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
    """No hazard class for services today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.services",
        title="Services",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "A ROS2 service server callback converts to an LF reaction; the "
            "client-side call/response pair is bridged through the runtime binding."
        ),
        playground=PlaygroundCase(
            example="service_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Services (supported)",
    )
)

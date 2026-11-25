# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.pubsub — ROS2 publish/subscribe, a SUPPORTED (no-hazard) feature.

Publishers and subscriptions are the foundational ROS2 → LF conversion (a
subscription callback becomes an LF reaction triggered by an input port fed
from the publisher's connection). QoS-specific hazard classes on top of
pub/sub already have their own dedicated modules
(``qos.qos_unknown``/``qos.best_effort_input``/``qos.shallow_queue``); this
module covers the base capability itself, which has no hazard class of its
own — ``detect`` is trivially ``[]``. See ``timers.py`` for the same
"presence is structural, not diagnostic" rationale.
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
    """No hazard class for base pub/sub itself — QoS hazards have their own modules."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.pubsub",
        title="Publish/subscribe",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "A ROS2 subscription callback converts to an LF reaction triggered "
            "by an input port fed from the publisher's connection."
        ),
        playground=PlaygroundCase(
            example="pubsub_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Publish/subscribe (supported)",
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.lifecycle — ROS 2 managed lifecycle nodes, a SUPPORTED (no-hazard) feature.

The wrapper builds, constructs, and registers ``rclcpp_lifecycle::LifecycleNode``
classes, then drives ``configure()``/``activate()`` before binding so that
entities created in ``on_configure``/``on_activate`` exist by the time
``ros2_lf_bind_node`` walks the node's callback groups. Transitions
are driven once, at startup.

A lifecycle node registered with ``rclcpp_components_register_nodes`` is built
as a non-component node so that the same bring-up applies. The
component path cannot drive a lifecycle: ``LFComponentManager`` records a
loaded component in ``class_to_id_`` but never in the node registry, so no
transition thunk is ever captured, and such a node used to be bound while still
``unconfigured`` and therefore bound nothing.

External transitions work: the binding's ingress role serves
``change_state`` and the parameter services, and the request runs on an LF
worker. The binding follows every transition: a change
observed at tag T holds from the next tag on, no callback of the node runs
while it is not active (arrivals are taken and dropped), and the node's entries
are re-matched against its current entities on every effective change.

No hazard class exists for this feature today, so ``detect`` is
trivially ``[]`` — see ``timers.py`` for the "presence is structural, not
diagnostic" rationale shared by every SUPPORTED-with-no-codes module in this
file set.
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
    """No hazard class for lifecycle nodes today — always returns no diagnostics."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.lifecycle",
        title="Managed lifecycle nodes",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "The wrapper builds, constructs, and registers the node, then drives "
            "configure() and activate() before binding, so entities created in "
            "on_configure/on_activate are discovered and LF-bound. A lifecycle "
            "node registered as a component is built as a non-component node so "
            "the same bring-up applies. External transitions (ros2 lifecycle set) "
            "are served on an LF worker. Later transitions are "
            "followed: nothing of the node runs while it is not active, and "
            "entities created by a transition bind at their next dispatch."
        ),
        playground=PlaygroundCase(
            example="lifecycle_example/lifecycle_node",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry=(
            "Managed lifecycle nodes, component or not (supported, configured and "
            "activated at startup unless opted out, later transitions take effect "
            "at the next tag)"
        ),
    )
)

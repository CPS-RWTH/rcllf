# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.message_filters, the admitted message_filters form, a SUPPORTED feature.

The analyzer admits an unmodified two-input
``ExactTime`` or ``ApproximateTime`` synchronizer over two
``message_filters::Subscriber`` members, each subscribed once with
``subscribe(this, "topic"[, qos[, options]])``. Each input becomes a subscription of the
node with its own LF reaction, and the synchronizer runs the user callback
inside the reaction of the input that completes a set. The admitted form
raises no diagnostic, so ``detect`` is trivially ``[]``. Every other form is
refused by ``qos.message_filters_sync`` (RCLLF-H003).

Bounds the claim to the oracle domain ``mf-sync/1``: both inputs are
admitted correspondence edges with one link delay, and the converted pairs
equal the pinned library run on the logical arrival order (``lf_eval
sync_oracle``). Generation refuses an input outside it
(``sync_input_legacy:<reason>``, ``sync_input_delay_unequal``).
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

#: The oracle domain the support claim is bounded to, also the
#: ``domain`` of the resolved model's ``sync.message_filters`` record.
DOMAIN_ID = "mf-sync/1"
#: The library versions the domain was validated against (Jazzy snapshot of
#: 2026-09, with a sync probe script); the comparator refuses other streams.
LIBRARY_PINS = {"message_filters": "4.11.17", "rclcpp": "28.1.22"}


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """No hazard class for the admitted form, other forms belong to RCLLF-H003."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.message_filters",
        title="message_filters two-input synchronizers",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "Each message_filters::Subscriber input converts to an LF reaction "
            "triggered by its own input port, and the synchronizer calls the "
            "user callback inside the reaction that completes a set. Both inputs "
            f"must be admitted correspondence edges with one link delay (domain {DOMAIN_ID})."
        ),
        playground=PlaygroundCase(
            # Two timer-driven sensors feed one ExactTime and one
            # ApproximateTime fusion node.
            example="fusion_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        # Release check: how far each evidence reaches.
        docs_entry=(
            "message_filters two-input ExactTime/ApproximateTime (supported on domain "
            f"{DOMAIN_ID}, admitted by the scan, both inputs on "
            "admitted correspondence edges with one delay; an input on a legacy edge is "
            "refused with RCLLF-H003 sync_input_legacy. On that domain the converted stream "
            "equals the pinned library run off-line by oracle O1 on the LF arrival order. "
            "The synchronizer evaluation matched 20 of 20 runs on one schedule, checked against a Python port of "
            "the two policies, not against the library "
            "itself)"
        ),
    )
)

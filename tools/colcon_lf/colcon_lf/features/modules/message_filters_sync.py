# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H003 — unsupported message_filters use.

The analyzer admits unmodified
two-input ``ExactTime`` and ``ApproximateTime`` synchronizers whose inputs are
``message_filters::Subscriber`` members
(``node_analysis/message_filters_scan.py``). Each input becomes a subscription
of the node, the binding dispatches the rclcpp subscription the library
created inside that input's LF reaction, and the synchronizer calls the user
callback synchronously within it. Admitted forms raise no diagnostic.

Every other use (other policies, more than two inputs, Cache, chained filters,
namespace aliases, ...) is listed in ``NodeInfo.message_filters.unsupported``
and refused here at error severity, one diagnostic per node naming each
reason. Without that refusal the unrecognised inputs would have no RID and the
binding would refuse to start (``node_binding.cpp``).

Detection still starts from ``NodeInfo.detected_includes`` (an include scan,
robust to missing third-party headers). A node that includes
message_filters but carries no scan result (a system.json from an older
analyzer) is refused.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H003 = "RCLLF-H003"

MESSAGE_FILTERS_ALLOWLIST_ENTRY = "message_filters"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H003 for every node with an unsupported message_filters use."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        detected = getattr(node_info, "detected_includes", None) or []
        if MESSAGE_FILTERS_ALLOWLIST_ENTRY not in detected:
            continue
        mf = getattr(node_info, "message_filters", None)
        if mf is None:
            reasons = ["no message_filters scan result in system.json "
                       "(re-run colcon lf-analyze)"]
        else:
            reasons = list(mf.unsupported)
        if not reasons:
            continue
        diagnostics.append(
            Diagnostic(
                code=CODE_H003,
                node=node_key,
                topic=None,
                message=(
                    f"node '{node_key}' uses message_filters in a form rcllf "
                    f"does not support: {'; '.join(reasons)}. Supported is a "
                    "two-input ExactTime, ApproximateTime or "
                    "rcllf BoundedNearestToAnchor Synchronizer over two "
                    "message_filters::Subscriber members, each subscribed with "
                    "subscribe(this, \"topic\"[, qos[, options]]). Generation refuses "
                    "unless RCLLF-H003 is listed in execution.yaml's "
                    "acknowledged_hazards, and an unrecognised input then "
                    "fails at bind time."
                ),
            )
        )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="qos.message_filters_sync",
        title="message_filters synchronizers outside the admitted two-input form",
        support=Support.REJECT,
        codes=(CODE_H003,),
        severity={CODE_H003: Severity.ERROR},
        detect=detect,
        conversion=(
            "Two-input ExactTime and ApproximateTime synchronizers over "
            "message_filters::Subscriber members are admitted without "
            "acknowledgement, and so is BoundedNearestToAnchor "
            "with literal parameters (sync.bna). Rewrite other forms into that "
            "shape, or acknowledge via execution.yaml's acknowledged_hazards "
            "to proceed anyway."
        ),
        playground=PlaygroundCase(
            # message_filters_cache_node.cpp feeds a message_filters::Cache,
            # which stays unsupported, so this launch demonstrates H003 in
            # isolation. message_filters_subscriber_node (the admitted
            # two-input ApproximateTime form) raises nothing.
            example="qos_hazard_example/message_filters_cache_node",
            expected_outcome=Outcome.REJECT,
            expected_codes=(CODE_H003,),
        ),
        docs_entry=(
            "message_filters other than two-input ExactTime/ApproximateTime "
            "or BoundedNearestToAnchor (error, refuses generation)"
        ),
    )
)

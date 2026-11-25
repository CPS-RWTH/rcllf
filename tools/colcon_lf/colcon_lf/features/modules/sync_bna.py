# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""sync.bna, the BoundedNearestToAnchor synchronizer, a SUPPORTED feature.

A node that builds ``rcllf_message_filters::sync_policies::BoundedNearestToAnchor``
with literal parameters over two ``message_filters::Subscriber`` inputs is
admitted. Under rcllf each admission is keyed by the logical tag and the
correspondence key of its drain, so the matched pairs and their callback
tags repeat run to run. RCLLF-H010 (INFO) tells the user that this policy is
the rcllf refinement, not ``ApproximateTime``: it decides by bounded nearest
to anchor rules and its outcomes are traced.
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

CODE_H010 = "RCLLF-H010"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H010 for every admitted BoundedNearestToAnchor synchronizer."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        mf = getattr(node_info, "message_filters", None)
        for sync in mf.synchronizers if mf else []:
            if sync.bna_params is None:
                continue
            params = ", ".join(f"{k}={v}" for k, v in sync.bna_params.items())
            diagnostics.append(
                Diagnostic(
                    code=CODE_H010,
                    node=node_key,
                    topic=None,
                    message=(
                        f"node '{node_key}' synchronizer {sync.member_variable} uses "
                        f"BoundedNearestToAnchor ({params} ns), the rcllf refinement, "
                        "not ApproximateTime. Each input topic needs stamp_provenance "
                        "in execution.yaml."
                    ),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="sync.bna",
        title="BoundedNearestToAnchor synchronizer",
        support=Support.SUPPORTED,
        codes=(CODE_H010,),
        severity={CODE_H010: Severity.INFO},
        detect=detect,
        conversion=(
            "Each input converts to its correspondence drain reaction, the policy "
            "admits at the drain's logical tag and key, and a generated wake "
            "reaction applies the anchor timeouts and candidate retention "
            "deadlines."
        ),
        playground=PlaygroundCase(
            example="bna_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(CODE_H010,),
        ),
        docs_entry="BoundedNearestToAnchor synchronizer (supported, RCLLF-H010 info)",
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H015: a node inherits endpoint-creating code from a template base.

The analyzer cannot walk a template base's constructor, so the endpoints
it creates would silently be missing from the generated system.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H015 = "RCLLF-H015"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H015 for every fact recorded on a node."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for fact in getattr(node_info, "template_base_endpoints", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H015,
                    node=node_key,
                    topic=fact.get("topic"),
                    message=(
                        f"node '{node_key}' inherits endpoint-creating code from "
                        f"template base {fact['base']} ({fact.get('calls', '')}); "
                        "not supported, see issue"
                    ),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="inheritance.template_base_endpoints",
        title="Node inherits endpoints from a template base",
        support=Support.REJECT,
        codes=(CODE_H015,),
        severity={CODE_H015: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: endpoints created in a template base class are not extracted; generation refuses the node unless RCLLF-H015 is acknowledged."
        ),
        playground=None,
        docs_entry=(
            "Node inherits endpoints from a template base (refused at generation)"
        ),
    )
)

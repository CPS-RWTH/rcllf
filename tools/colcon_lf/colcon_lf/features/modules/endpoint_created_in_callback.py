# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H013: an endpoint created outside the constructor/lifecycle hooks.

rcllf binds endpoints in a one-shot startup pass, so a subscription, service,
client or publisher created later in a method is never bound and its callback
never runs. The analyzer keeps the fact in ``NodeInfo.unbound_endpoints``.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H013 = "RCLLF-H013"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H013 for every fact recorded on a node."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for fact in getattr(node_info, "unbound_endpoints", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H013,
                    node=node_key,
                    topic=fact.get("topic"),
                    message=(
                        f"node '{node_key}' creates {fact['kind']} "
                        f"'{fact['topic']}' in '{fact['method']}'"
                        + (f" (callback {fact['callback']})" if fact.get("callback") else "")
                        + "; under rcllf its callback never runs. "
                        "Create it in the constructor."
                    ),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="endpoints.created_outside_scope",
        title="Endpoint created outside the constructor/lifecycle hooks",
        support=Support.REJECT,
        codes=(CODE_H013,),
        severity={CODE_H013: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: an endpoint created in an arbitrary member function is never bound by the one-shot startup pass; generation refuses the node unless execution.yaml acknowledges RCLLF-H013 (the late endpoint then delivers nothing)."
        ),
        playground=None,
        docs_entry=(
            "Endpoint created outside the constructor (refused at generation, create it in the constructor)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H016: the compile definitions selecting a source variant are ambiguous.

One .cpp built into several executables with different definitions (#ifdef
mains) that cannot be told apart for this node.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H016 = "RCLLF-H016"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H016 for every fact recorded on a node."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for fact in getattr(node_info, "compile_variant_issues", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H016,
                    node=node_key,
                    topic=fact.get("topic"),
                    message=(
                        f"{fact.get('source', node_key)}: {fact.get('reason', '')} "
                        "(RCLLF-H016: the executable's compile definitions could not "
                        "be determined, so the wrong #ifdef variant may be analyzed)"
                    ),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="build.compile_variant_ambiguous",
        title="Compile variant of a shared source is ambiguous",
        support=Support.REJECT,
        codes=(CODE_H016,),
        severity={CODE_H016: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: the executable's target cannot be separated from sibling targets of the same source file, so the #ifdef variant analyzed may be wrong; generation refuses unless RCLLF-H016 is acknowledged."
        ),
        playground=None,
        docs_entry=(
            "Ambiguous compile variant of a shared source (refused at generation)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H014: main() does node work outside executor callbacks.

rcllf replaces main(), so a spin_some loop that calls a publishing node
method, or a direct publish in main, never runs.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H014 = "RCLLF-H014"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H014 for every fact recorded on a node."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for fact in getattr(node_info, "main_outside_callbacks", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H014,
                    node=node_key,
                    topic=fact.get("topic"),
                    message=(
                        f"main() of {fact.get('source', 'the node')} runs node work "
                        f"outside callbacks ({fact['call']}); rcllf replaces main(), "
                        "so it never runs. Move it into a timer."
                    ),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="main.work_outside_callbacks",
        title="main() runs node work outside callbacks",
        support=Support.REJECT,
        codes=(CODE_H014,),
        severity={CODE_H014: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: node work done directly in main() (a publish, or a publishing method called from a spin_some loop) is dropped because rcllf replaces main(); generation refuses unless RCLLF-H014 is acknowledged."
        ),
        playground=None,
        docs_entry=(
            "Work in main() outside callbacks (refused at generation, move it into a timer)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H017, intra-process communication switched on.

With ``use_intra_process_comms`` on, rclcpp hands a message from a publisher to
a subscription in the same process through the intra-process manager, not
through the middleware. rcllf ingests and orders messages at the middleware
boundary, and the intra-process transport zeroes the publisher GID, so such a
publication has no identity. The binding refuses to start a publication edge of
a node that turns it on (``corr_runtime.cpp``, ``intra_process``); this module
says so at analyze time, before anything is built.

Two places switch it on and both are detected:

* a composable node's launch ``extra_arguments=[{'use_intra_process_comms':
  True}]`` (read from the launch file, with its arguments bound);
* node source that builds its options with ``.use_intra_process_comms(true)``.

ERROR: generation stops. Turn the option off, or leave it unset (the default
is off).
"""

from __future__ import annotations

import re
from pathlib import Path

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H017 = "RCLLF-H017"

_COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.DOTALL)
_SOURCE_RE = re.compile(r"\buse_intra_process_comms\s*\(\s*true\s*\)")

_ADVICE = (
    "Under rcllf a publication must pass the middleware to have an identity, and "
    "the binding refuses to start an edge of a node with intra-process "
    "communication on. Remove the option, or set it to false."
)


def _source_turns_it_on(path: Path | None) -> bool:
    if path is None:
        return False
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return bool(_SOURCE_RE.search(_COMMENT_RE.sub("", text)))


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H017 for a launch or source switch of intra-process comms."""
    diagnostics: list[Diagnostic] = []
    launch_graph = getattr(ctx.graph, "launch_graph", None)
    for launch_node in getattr(launch_graph, "all_nodes", None) or []:
        if getattr(launch_node, "use_intra_process_comms", None) is not True:
            continue
        label = launch_node.name or launch_node.executable
        diagnostics.append(
            Diagnostic(
                code=CODE_H017,
                node=label,
                topic=None,
                message=(
                    f"composable node '{label}' sets extra_arguments "
                    f"use_intra_process_comms to True in its launch file. {_ADVICE}"
                ),
            )
        )
    for node_key, node_info in ctx.graph.nodes.items():
        files = [getattr(node_info, "source_file", None),
                 getattr(node_info, "header_file", None)]
        hit = next((f for f in files if _source_turns_it_on(f)), None)
        if hit is None:
            continue
        diagnostics.append(
            Diagnostic(
                code=CODE_H017,
                node=node_key,
                topic=None,
                message=(
                    f"node '{node_key}' builds its NodeOptions with "
                    f"use_intra_process_comms(true) in {Path(hit).name}. {_ADVICE}"
                ),
            )
        )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="ipc.intra_process",
        title="Intra-process communication",
        support=Support.REJECT,
        codes=(CODE_H017,),
        severity={CODE_H017: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: a message delivered through rclcpp's intra-process manager "
            "has no publisher identity at the middleware boundary, the binding "
            "refuses to start its edges, so generation refuses it unless "
            "execution.yaml acknowledges RCLLF-H017."
        ),
        playground=PlaygroundCase(
            example="intra_process_example/bringup",
            expected_outcome=Outcome.REJECT,
            expected_codes=(CODE_H017,),
        ),
        docs_entry=(
            "Intra-process communication (refused at analyze, turn "
            "use_intra_process_comms off)"
        ),
    )
)

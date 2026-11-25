# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H011, an action server that sends from its own thread.

Under rcllf every action callback runs in an LF reaction, and the binding
admits an action feedback publish or result send only when an LF reaction
runs it. A thread the node starts itself (the stock pattern, ``execute`` on a
``std::thread`` from ``handle_accepted``, or a worker started in the
constructor) sends with no LF origin. The runtime refuses such a send (one
error log and a trace line, the call is not forwarded), so the client never
gets the feedback or the result.

ERROR, like RCLLF-H009: the generated program cannot serve the goal, so
generation stops. Advance goals from an LF timer instead, as
``action_example`` does (one step per timer callback).
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

CODE_H011 = "RCLLF-H011"


def _message(node_key: str, entry: dict[str, str]) -> str:
    action = entry.get("action", "?")
    action_text = (
        "one of its action servers" if action == "?" else f"action '{action}'")
    return (
        f"node '{node_key}' sends feedback or a result on {action_text} from "
        f"thread '{entry.get('thread', '?')}', started in "
        f"{entry.get('started_in', '?')}. Under rcllf an action send needs an LF "
        "origin, so the runtime refuses a send from the node's own thread and the "
        "client never gets it. Advance goals from a timer callback."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H011 for every thread of a node that sends on an action handle."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for entry in getattr(node_info, "thread_action_senders", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H011,
                    node=node_key,
                    topic=entry.get("action", "?"),
                    message=_message(node_key, entry),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="actions.thread_server",
        title="Action server sending from its own thread",
        support=Support.REJECT,
        codes=(CODE_H011,),
        severity={CODE_H011: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: feedback or a result sent from a thread the node starts "
            "itself has no LF origin, the runtime refuses it, so generation "
            "refuses the node unless execution.yaml acknowledges RCLLF-H011."
        ),
        playground=PlaygroundCase(
            example="action_full_example/bringup",
            expected_outcome=Outcome.REJECT,
            expected_codes=(CODE_H011,),
        ),
        docs_entry=(
            "Action server sending feedback or results from its own thread "
            "(refused at generation, advance goals from a timer callback)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H006 — a timer created outside the constructor/lifecycle hooks.

`action_example`'s `ActionClientNode` creates `cancel_timer_`
(`create_wall_timer(3s, ...)`) inside `goal_response_callback`, once the
action server has accepted a goal — not in the constructor, and not in a
managed-lifecycle hook (`node_analysis/models.py`'s `LIFECYCLE_CHAINED_HOOKS`
family). The analyzer used to simply never visit that callback for resource
creation, so the timer, its callback, and the cancel request it drives never
appeared anywhere in the extracted model: no generated system, single-process
or federated, has ever emitted a reaction for it.

**Why extraction is not the fix.** `ros2_lf_bind_node`
(`packages/lf-frame/ros2_lf_binding/src/node_binding.cpp`) drives a lifecycle
node's `configure()`/`activate()` transitions *before* its single, one-shot
callback-group walk that populates the handle table — so
constructor- and lifecycle-hook-created timers already exist by the time that
one pass runs. A timer created inside an arbitrary member function is created
later, at runtime, in response to something the bind pass has already
finished waiting for (here, the action server's goal-response). It is never
in the handle table ("a timer created after startup is not in the
handle table"), so a generated reaction triggered by it could never be bound
— the reaction would be silently, permanently dead code, which is worse than
the current silent drop because it would look supported. `unbound_timers`
(`node_analysis/models.py`) keeps the fact instead: `_find_out_of_scope_timers`
(`clang_parser.py`) still extracts the timer's period/callback/member
variable, but leaves it off `NodeInfo.timers`, so nothing about codegen
changes; this module turns that fact into a visible diagnostic.

WARNING, not ERROR: the rest of the node still analyzes and generates
normally, exactly as before this issue was found — the only change is that
the drop is now loud instead of silent.
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

CODE_H006 = "RCLLF-H006"


def _message(node_key: str, timer) -> str:
    var = timer.member_variable or "(unnamed timer)"
    cb = timer.callback or "(unknown callback)"
    return (
        f"node '{node_key}' creates timer '{var}' (callback '{cb}') inside "
        f"'{timer.created_in}', not in the constructor or a managed-lifecycle "
        "hook. The runtime's one-shot startup bind pass "
        "(ros2_lf_bind_node) can only observe timers that already exist by "
        "then, so this timer is not extracted into the generated system: "
        f"'{cb}' never runs under lfc."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H006 for every timer created outside the constructor/lifecycle hooks."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for timer in getattr(node_info, "unbound_timers", []) or []:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H006,
                    node=node_key,
                    topic=None,
                    message=_message(node_key, timer),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="timers.created_outside_scope",
        title="Timer created outside the constructor/lifecycle hooks",
        support=Support.WARN,
        codes=(CODE_H006,),
        severity={CODE_H006: Severity.WARNING},
        detect=detect,
        conversion=(
            "warning: a timer created inside an arbitrary member function "
            "(not the constructor, not a managed-lifecycle hook) is not "
            "extracted or generated — the runtime's one-shot startup bind "
            "pass cannot observe a timer that does not exist yet, so a "
            "generated reaction for it could never be bound. "
            "The rest of the node analyzes and generates normally."
        ),
        playground=PlaygroundCase(
            # action_client only (not action_example/bringup, which
            # core.actions already covers with an unrelated PASS
            # expectation) — isolates this case to the one node that
            # actually creates cancel_timer_ outside its constructor.
            example="action_example/action_client",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H006,),
        ),
        docs_entry=(
            "Timer created outside the constructor/lifecycle hooks "
            "(warning, dropped and not generated)"
        ),
    )
)

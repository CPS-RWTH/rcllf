# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H007, a timer or subscription callback the analyzer cannot model.

The analyzer turns a timer or subscription callback into a
reaction and reads the callback's body to find the publishers it uses, which
become the reaction's declared effects and the correspondence edges of the
model. It reads two forms, each only as the whole callback argument
(``_callback_form`` in ``clang_parser.py``):

* a member function handed over with ``std::bind(&Node::method, this)``, and
* an inline lambda, which gets a callback named by creation order,
  ``lambda_timer_<i>`` or ``lambda_subscription_<i>``
  (``_synthesize_lambda_callbacks``).

Any other form (a ``std::function`` member or variable, a lambda stored in a
variable first, a free function pointer, or a conditional or other expression
that merely contains a bind or a lambda, such as
``flag ? std::bind(&N::a, this) : std::bind(&N::b, this)``) leaves the
callback empty: the one body it would run is not visible at the creation site.
Without a reaction the runtime would leave the handle unbound and refuse to
start, so this module refuses at generation instead and names the argument it
could not read.

**Why refuse instead of bind.** Binding the handle to a reaction without its
body would make it run, but it would publish through ports the model does not
declare. The LF schedule would then no longer order that output against its
readers, which is exactly the determinism the generated system claims.
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

CODE_H007 = "RCLLF-H007"

_READABLE_FORMS = (
    "The analyzer reads a member function passed via std::bind(&Node::method, this) "
    "or an inline lambda, each written as the whole callback argument"
)


def _timer_message(node_key: str, timer) -> str:
    var = timer.member_variable or "(unnamed timer)"
    period = "unknown period" if timer.period_ms is None else f"period {timer.period_ms:g} ms"
    form = timer.callback_expr or "(unknown form)"
    return (
        f"node '{node_key}' creates timer '{var}' ({period}) with callback "
        f"argument `{form}`, which is neither form the analyzer can model. "
        f"{_READABLE_FORMS}, so this timer would get no reaction. Pass the "
        "callback in one of those forms."
    )


def _subscription_message(node_key: str, sub) -> str:
    var = sub.member_variable or "(unnamed subscription)"
    topic = sub.topic or "(unresolved topic)"
    form = sub.callback_expr or "(unknown form)"
    return (
        f"node '{node_key}' creates subscription '{var}' on '{topic}' with "
        f"callback argument `{form}`, which is neither form the analyzer can "
        f"model. {_READABLE_FORMS}, so this subscription would get no reaction. "
        "Pass the callback in one of those forms."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H007 for every extracted timer or subscription without a callback."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for timer in getattr(node_info, "timers", []) or []:
            if not timer.callback:
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H007,
                        node=node_key,
                        topic=None,
                        message=_timer_message(node_key, timer),
                    )
                )
        for sub in getattr(node_info, "subscribers", []) or []:
            if not sub.callback:
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H007,
                        node=node_key,
                        topic=sub.topic or None,
                        message=_subscription_message(node_key, sub),
                    )
                )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="callbacks.unmodeled_form",
        title="Timer or subscription callback in a form the analyzer cannot model",
        support=Support.REJECT,
        codes=(CODE_H007,),
        severity={CODE_H007: Severity.ERROR},
        detect=detect,
        conversion=(
            "pass the callback as std::bind(&Node::method, this) or as an inline "
            "lambda at the create_wall_timer / create_subscription call. A "
            "std::function member, a lambda stored in a variable first or a free "
            "function pointer hides the body the reaction is modelled from."
        ),
        playground=PlaygroundCase(
            # OpaqueCallbackNode has one timer whose callback is a
            # std::function member and nothing else, so H007 fires in isolation.
            example="handle_matching_example/opaque_callback",
            expected_outcome=Outcome.REJECT,
            expected_codes=(CODE_H007,),
        ),
        docs_entry=(
            "Timer or subscription callback in a form the analyzer cannot model "
            "(error, refuses generation)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H009, a blocking service client.

A client that sends a request and then waits for the answer inside the same
callback (``future.wait_for``, ``spin_until_future_complete``, or a loop on
``spin_some``/``spin_once``, a condition variable or a flag that the response
callback sets) cannot run
under rcllf. The response arrives as a later LF event, which the waiting
reaction holds back, so every call ends in its own timeout. A wait in
``main`` outside any callback has no LF trigger at all, and the request is
never sent. Runs of ``blocking_service_client_example`` showed both, in one
process and with one federate per node.

ERROR, not WARNING: the generated program cannot answer the call, so
generation stops. A client whose call style the analyser could not decide
(``call_style`` is ``None``) keeps the async path and is not reported here.
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

CODE_H009 = "RCLLF-H009"

_WAIT_CALL = {
    "blocking_future": "future.wait()/wait_for()/get()",
    "blocking_spin": "spin_until_future_complete()",
    "blocking_wait": (
        "a wait loop (spin_some/spin_once, a condition variable, or a sleep "
        "polling a flag) until the response callback runs"
    ),
}


def _message(node_key: str, service: str, call_style: str) -> str:
    return (
        f"node '{node_key}' calls service '{service}' and blocks on "
        f"{_WAIT_CALL[call_style]} (call style {call_style}). Under rcllf the "
        "response is a later LF event, so a wait inside a callback times out on "
        "every call, and a wait outside any callback is never triggered. "
        "Use async_send_request with a response callback."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H009 for every service client with a blocking call style."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for client in getattr(node_info, "service_clients", []) or []:
            if client.call_style not in _WAIT_CALL:
                continue
            diagnostics.append(
                Diagnostic(
                    code=CODE_H009,
                    node=node_key,
                    topic=client.service_name,
                    message=_message(node_key, client.service_name, client.call_style),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="services.blocking_client",
        title="Blocking service client",
        support=Support.REJECT,
        codes=(CODE_H009,),
        severity={CODE_H009: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: a service call that waits for its response in the same "
            "callback cannot be answered under rcllf, so generation refuses it "
            "unless execution.yaml acknowledges RCLLF-H009."
        ),
        playground=PlaygroundCase(
            example="blocking_service_client_example/bringup",
            expected_outcome=Outcome.REJECT,
            expected_codes=(CODE_H009,),
        ),
        docs_entry=(
            "Blocking service client (refused at generation, use "
            "async_send_request with a response callback)"
        ),
    )
)

# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""RCLLF-H012 — a request or goal sent from a service handler or a response callback.

A request port is written by the reaction of the timer or subscription
callback that sends the request (review round 5, O1-1/O1-2). A service
handler or a client's response callback runs in a reaction that writes no
request port, so a request it sends reaches its server outside any LF write:
the server takes it in place of a later in-system request, or never serves it,
and its response callback never runs. The binding drops such a send's count
with a warning at run time; the analysis refuses it before generation.

ERROR, not WARNING: the generated program would serve requests out of order.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Support,
    Severity,
)
from colcon_lf.features.registry import FeatureModule, register

CODE_H012 = "RCLLF-H012"


def _handlers(node_info) -> dict[str, str]:
    """Callback name -> what it handles, for service handlers and response callbacks."""
    roles: dict[str, str] = {}
    for srv in getattr(node_info, "service_servers", []) or []:
        if srv.callback:
            roles.setdefault(srv.callback, f"the handler of service '{srv.service_name}'")
    for cli in getattr(node_info, "service_clients", []) or []:
        if cli.response_callback:
            roles.setdefault(
                cli.response_callback, f"the response callback of service '{cli.service_name}'"
            )
    return roles


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H012 for every handler or response callback that sends a request or goal."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        roles = _handlers(node_info)
        for cb in getattr(node_info, "callbacks", []) or []:
            role = roles.get(cb.name)
            if role is None:
                continue
            sent = list(cb.service_clients_used or []) + list(cb.action_clients_used or [])
            for target in sent:
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H012,
                        node=node_key,
                        topic=target,
                        message=(
                            f"node '{node_key}': callback '{cb.name}' ({role}) sends a "
                            f"request or goal on '{target}'. Under rcllf a request is "
                            "carried by the port of the timer or subscription reaction "
                            "that sends it; a send from a service handler or a response "
                            "callback has no port, so its server would serve it in place "
                            "of a later in-system request. Send it from a timer or "
                            "subscription callback instead."
                        ),
                    )
                )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="services.request_from_service_callback",
        title="Request sent from a service handler or response callback",
        support=Support.REJECT,
        codes=(CODE_H012,),
        severity={CODE_H012: Severity.ERROR},
        detect=detect,
        conversion=(
            "error: a request or goal sent from a service handler or a response "
            "callback has no request port, so generation refuses it unless "
            "execution.yaml acknowledges RCLLF-H012."
        ),
        playground=None,
        docs_entry=(
            "Request from a service handler or response callback (refused at "
            "generation, send it from a timer or subscription callback)"
        ),
    )
)

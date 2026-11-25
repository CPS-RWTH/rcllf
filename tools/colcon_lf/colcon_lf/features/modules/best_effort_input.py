# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H001 — best_effort QoS reliability on a subscription.

Warning severity, not error. Refusing generation on the subset of best_effort spellings the parser
happens to catch (while staying silent on ``rclcpp::SensorDataQoS()``, which
*is* best_effort) would be worse than warning — it would teach users the
check is trustworthy when it is not yet. Promote to error once QoS
extraction is broadly trustworthy against real systems.
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

CODE_H001 = "RCLLF-H001"


def resolved_topic(graph, node_key: str, topic: str) -> str:
    """The topic name *node_key*'s subscription *topic* runs on.

    The launch file's namespace and remappings applied, as
    ``comm_analysis.topic_mapper`` resolves the system graph's topics (the
    analysis keeps the code-level name on the subscription). A node no launch
    action names resolves in the root namespace, as there. Empty or unknown
    names are returned unchanged.
    """
    from colcon_lf.comm_analysis.topic_mapper import resolve_name
    from colcon_lf.graph.hierarchical import _node_key
    from colcon_lf.launch_analysis.ros_arguments import launch_identity

    node_info = graph.nodes.get(node_key)
    if node_info is None or not topic.strip():
        return topic
    for launch_node in graph.launch_graph.all_nodes:
        if _node_key(launch_node, node_info) == node_key:
            identity = launch_identity(launch_node, node_info.name)
            return resolve_name(topic, identity.namespace, list(identity.remappings),
                                identity.name)
    return resolve_name(topic, None, [], node_info.name)


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H001 for every subscription with materialized best_effort reliability.

    ``topic`` stays the code-level name (the correspondence edges' ``sub_topic``
    key, see ``h001_by_edge_status``); the message names the remapped topic the
    subscription runs on.
    """
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for sub in node_info.subscribers:
            if sub.qos_reliability == "best_effort":
                shown = resolved_topic(ctx.graph, node_key, sub.topic)
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H001,
                        node=node_key,
                        topic=sub.topic,
                        message=(
                            f"subscription '{shown}' on node '{node_key}': "
                            "best_effort QoS reliability — messages may be "
                            "dropped in transit; LF's deterministic tag "
                            "semantics do not compensate for transport drops."
                        ),
                    )
                )
    return diagnostics


def h001_by_edge_status(diagnostics: list[Diagnostic], corr_edges: list,
                        graph=None) -> list[tuple[Severity, str]]:
    """RCLLF-H001 text per correspondence edge status (W3 slice S3).

    Known only at generation time, once the edges are admitted or refused.
    On a subscription whose every incoming edge is admitted, a transport drop
    is no longer silent: the binding times the sample out and delivers a
    missing outcome at its tag, so the diagnostic becomes an info note. A
    subscription with a legacy edge, or with no edge at all, keeps the
    analyze-time warning text unchanged. With *graph* the info note names the
    remapped topic, as the analyze-time text does.
    """
    out: list[tuple[Severity, str]] = []
    for diag in diagnostics:
        if diag.code != CODE_H001:
            continue
        edges = [e for e in corr_edges if (e.sub_node, e.sub_topic) == (diag.node, diag.topic)]
        if edges and all(e.status == "admitted" for e in edges):
            labels = ", ".join(e.label() for e in edges)
            shown = (resolved_topic(graph, diag.node, diag.topic) if graph is not None
                     else diag.topic)
            out.append((Severity.INFO, (
                f"subscription '{shown}' on node '{diag.node}': best_effort QoS "
                f"reliability on admitted correspondence edge(s) {labels}. Drops become "
                "missing outcomes, see the deadline hook (the subscription's "
                "requested-deadline-missed callback)."
            )))
        else:
            out.append((Severity.WARNING, diag.message))
    return out


MODULE = register(
    FeatureModule(
        feature_id="qos.best_effort_input",
        title="best_effort input subscription",
        support=Support.WARN,
        codes=(CODE_H001,),
        severity={CODE_H001: Severity.WARNING},
        detect=detect,
        conversion=(
            "a warning for now: promote to error once QoS extraction "
            "reliably distinguishes 'reliable' from 'unknown' across named "
            "profiles, not just the literal .best_effort() spelling."
        ),
        playground=PlaygroundCase(
            # best_effort_subscriber_node.cpp subscribes on "best_effort_topic"
            # with rclcpp::QoS(10).best_effort() — a literal spelling H001's
            # detect() matches directly.
            example="qos_hazard_example/best_effort_subscriber_node",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H001,),
        ),
        docs_entry="best_effort input subscription (warning)",
    )
)

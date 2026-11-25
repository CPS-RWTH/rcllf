# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H002 — depth-1 subscription queue on a topic with an in-model publisher.

Per ``00_design.md`` section 3.1: fires when ``qos_depth == 1`` on a
subscription whose topic has a publisher elsewhere in the model. The join is
by slash-tolerant topic name (``TopicConnection`` does not carry QoS, per
``02_model_pipeline_survey.md`` section 1 / review nit) — the same
three-way comparison used throughout the exec-config canonicalization
(``node_binding.cpp:234-238``).
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

CODE_H002 = "RCLLF-H002"


def _topic_matches(a: str, b: str) -> bool:
    """Slash-tolerant topic equality (same three-way comparison as node_binding.cpp)."""
    return a == b or a == f"/{b}" or f"/{a}" == b


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H002 for depth-1 subscriptions whose topic has an in-model publisher."""
    published_topics: list[str] = []
    for node_info in ctx.graph.nodes.values():
        for pub in node_info.publishers:
            published_topics.append(pub.topic)

    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for sub in node_info.subscribers:
            if sub.qos_depth != 1:
                continue
            if any(_topic_matches(sub.topic, pub_topic) for pub_topic in published_topics):
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H002,
                        node=node_key,
                        topic=sub.topic,
                        message=(
                            f"subscription '{sub.topic}' on node '{node_key}': "
                            "queue depth 1 with an in-model publisher — a "
                            "burst of publishes between reactions will drop "
                            "all but the most recent message."
                        ),
                    )
                )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="qos.shallow_queue",
        title="Depth-1 subscription queue",
        support=Support.WARN,
        codes=(CODE_H002,),
        severity={CODE_H002: Severity.WARNING},
        detect=detect,
        conversion=(
            "warning: a shallow queue is a legitimate choice for "
            "latest-value-only consumers; flagged so it is a deliberate "
            "choice, not an accidental default."
        ),
        playground=PlaygroundCase(
            # H002 needs the depth-1 subscriber (shallow_queue_topic) joined
            # against an in-model publisher (hazard_publisher_node). The
            # standalone shallow_queue_subscriber_node.launch.py does not
            # include hazard_publisher_node, so it alone would never fire H002;
            # the package's `bringup` launch does include both, but it also
            # starts message_filters_cache_node, whose RCLLF-H003 (ERROR)
            # makes the generate hazard-gate
            # (verb/generate.py:_check_hazard_gate) refuse on the whole graph —
            # the launch's real end-to-end outcome there is REJECT, not WARN.
            #
            # The
            # purpose-built `shallow_queue_with_publisher` launch starts exactly
            # hazard_publisher_node + shallow_queue_subscriber_node (no
            # message_filters), so H002 fires in isolation and this case can
            # assert the module's own support level, WARN. Both nodes use a
            # bare-int QoS depth, which materializes to reliability "reliable",
            # so neither H000 nor H001 co-fires either.
            example="qos_hazard_example/shallow_queue_with_publisher",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H002,),
        ),
        docs_entry="Depth-1 subscription queue (warning)",
    )
)

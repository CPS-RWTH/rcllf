# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H004 — topic name not resolvable to a single static string literal.

This module turns
the analyzer's most dangerous *silent* extraction failure into a first-class,
persisted diagnostic.

Before Phase 7, a topic argument that was not a plain string literal took one of
two paths in ``node_analysis/clang_parser.py``:

- **Nothing resolvable at all** — the pub/sub entry was still emitted with
  ``topic=""`` and a human-readable string was appended to ``parse_warnings``.
  ``parse_warnings`` are printed to analyze's stderr only; they never reach
  ``system.json``, so no downstream consumer (``lf_eval``, a CI gate, the
  Phase 7 topology checker) could ask "did this system have unresolved
  topics?" from the artifact alone. Worse, every ``topic=""`` entry collided in
  ``comm_analysis/topic_mapper.py``'s ``pub_map``/``sub_map``, so two unrelated
  nodes with unresolvable names appeared *falsely connected*.
- **Partially resolvable** — ``_first_string_literal`` walked the whole
  argument subtree and returned the *first* literal it found, so
  ``"prefix_" + std::to_string(id)`` recorded ``"prefix_"`` as though it were
  the complete, correct topic name, with **no warning of any kind**.

``clang_parser._resolve_topic_argument`` now classifies the argument expression
(``classify_topic_expression``), discards a partial literal, keeps the
``parse_warning``, and sets ``topic_dynamic=True`` on the emitted
``PublisherInfo``/``SubscriberInfo``. That flag is a plain dataclass field, so
``output/serializer.py``'s ``asdict`` walk persists it into ``system.json``
automatically. This module reads it back and emits one WARNING-severity
diagnostic per flagged endpoint — the surfacing mechanism for edges the static
graph is known to be missing.

Warning, not error: dynamic topic names are legitimate ROS2 style (namespaced
multi-instance nodes, parameterized pipelines). What is *not* legitimate is
letting the extracted graph claim completeness it does not have.
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

CODE_H004 = "RCLLF-H004"


def _is_unresolved(topic: str, topic_dynamic: bool) -> bool:
    """Whether an endpoint's topic name is still unknown at the end of analysis.

    ``topic_dynamic`` is set by the clang parser at *extraction* time, but two
    later pipeline stages can still resolve a ``${param}`` placeholder into a
    real name: ``clang_parser._resolve_placeholders_from_params`` (from a C++
    ``declare_parameter`` default) and ``graph.hierarchical._resolve_topic_placeholders``
    (from a launch/YAML parameter overlay). Both clear the flag. This predicate
    still treats any topic that no longer looks like a ``${...}`` placeholder
    and is non-empty as resolved, for a ``system.json`` written before the
    second stage cleared it. Net
    effect: H004 fires only for endpoints whose name really is unknown in the
    finished graph, never for one that was recovered downstream.
    """
    if not topic_dynamic:
        return False
    name = topic.strip()
    if not name:
        return True
    return name.startswith("${") and name.endswith("}")


def _message(kind: str, topic: str, node_key: str) -> str:
    """Build the diagnostic text for one flagged endpoint."""
    where = f"'{topic}'" if topic else "(name unresolved)"
    return (
        f"{kind} {where} on node '{node_key}': the topic name is built at "
        "runtime (concatenation, member/parameter reference, or computed "
        "expression), so it could not be resolved statically — this "
        "connection may be missing from the extracted graph. Use a string "
        "literal or a `constexpr`/`const char[]` constant if this edge must "
        "appear in the generated model."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H004 for every pub/sub whose topic name is not statically resolvable."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for pub in node_info.publishers:
            if _is_unresolved(pub.topic, getattr(pub, "topic_dynamic", False)):
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H004,
                        node=node_key,
                        topic=pub.topic or None,
                        message=_message("publisher", pub.topic, node_key),
                    )
                )
        for sub in node_info.subscribers:
            if _is_unresolved(sub.topic, getattr(sub, "topic_dynamic", False)):
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H004,
                        node=node_key,
                        topic=sub.topic or None,
                        message=_message("subscription", sub.topic, node_key),
                    )
                )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="topics.dynamic_name",
        title="Dynamically computed topic name",
        support=Support.WARN,
        codes=(CODE_H004,),
        severity={CODE_H004: Severity.WARNING},
        detect=detect,
        conversion=(
            "warning: the LF model can only wire connections whose topic names "
            "are known at analysis time. A runtime-computed name is not "
            "convertible; the endpoint appears in the model without an edge. "
            "Hoist the name to a `constexpr`/`const char[]` constant, or accept "
            "the missing edge deliberately."
        ),
        playground=PlaygroundCase(
            # dynamic_topic_example is purpose-built
            # for this code: dynamic_publisher_node builds its topic name as
            # `"chatter_" + std::to_string(instance_id)` (the silent partial-
            # literal case), while static_subscriber_node uses a plain literal
            # so the package still contributes one valid edge to the graph.
            # Both use a bare-int QoS depth (materializes to "reliable"), so no
            # QoS-hazard module co-fires and this launch's end-to-end outcome is
            # a clean WARN.
            example="dynamic_topic_example/bringup",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H004,),
        ),
        docs_entry="Dynamically computed topic name (warning)",
    )
)

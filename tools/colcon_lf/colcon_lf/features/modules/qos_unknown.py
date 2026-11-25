# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H000 — unknown/unparsed QoS reliability on a subscription or publisher.

Per ``00_design.md`` section 3.1: fires when a subscription's or publisher's
``qos_reliability`` is still ``None`` after the defaults-materialization pass
in ``node_analysis/clang_parser.py`` (i.e. genuinely un-parseable — not a
bare-int default, which materializes to ``"reliable"``). Info severity: this
makes an analysis blind spot *visible* rather than silently treated as safe.

Extended to publishers:
previously only subscriptions were checked, even though ``PublisherInfo``
carries the same ``qos_reliability`` field and is subject to the identical
extraction gap. Under the Phase 6 benchmark's coding pattern (QoS built at
runtime from a `qos_profile` local, overlay-driven), this is expected to
fire on every publisher and subscriber, every run — a standing, expected
diagnostic confirming QoS hazard detection is running against nulls, not a
transient signal.
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

CODE_H000 = "RCLLF-H000"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H000 for every pub/sub whose QoS reliability is unparsed."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        for sub in node_info.subscribers:
            if sub.qos_reliability is None:
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H000,
                        node=node_key,
                        topic=sub.topic,
                        message=(
                            f"subscription '{sub.topic}' on node '{node_key}': "
                            "QoS reliability could not be determined during "
                            "analysis (unrecognized QoS expression) — treat "
                            "the effective reliability as unknown, not safe."
                        ),
                    )
                )
        for pub in node_info.publishers:
            if pub.qos_reliability is None:
                diagnostics.append(
                    Diagnostic(
                        code=CODE_H000,
                        node=node_key,
                        topic=pub.topic,
                        message=(
                            f"publisher '{pub.topic}' on node '{node_key}': "
                            "QoS reliability could not be determined during "
                            "analysis (unrecognized QoS expression) — treat "
                            "the effective reliability as unknown, not safe."
                        ),
                    )
                )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="qos.qos_unknown",
        title="Unknown/unparsed QoS reliability",
        support=Support.SUPPORTED,
        codes=(CODE_H000,),
        severity={CODE_H000: Severity.INFO},
        detect=detect,
        conversion=(
            "Extraction gap, not a code hazard: extend "
            "`_extract_qos_attrs`/defaults-materialization in clang_parser.py "
            "to recognize the QoS expression."
        ),
        playground=PlaygroundCase(
            # pubsub_example's pub/sub QoS is a bare-int depth (10) with no
            # named profile or chained method calls, which materializes to
            # reliability "reliable" (not None) — so this launch does NOT
            # trigger RCLLF-H000. Kept as the qos.qos_unknown playground case
            # anyway to demonstrate the SUPPORTED, no-diagnostic path (a
            # clean pass); see qos_hazard_example for a launch that does
            # exercise QoS hazard detection end-to-end.
            example="pubsub_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry="Unknown QoS reliability (info)",
    )
)

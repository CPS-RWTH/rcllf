# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-R001/R002/R003 — the resolved-model fail-closed gate.

Unlike every other module in ``features/modules/``, these three codes are
never emitted by ``detect()`` against a ``DetectionContext`` at *analyze*
time: whether a timer period or a tf2/take mechanism is resolved can change
at *generate* time (a benchmark system.json patch, a ``--params-file``, or an
``execution.yaml`` P3 timer-period override all postdate analysis), so the
gate reads ``resolved_model.resolve()``'s records directly
(``verb/generate.py::_check_resolved_model_gate``), not a persisted
``Diagnostic``.

This module's only job is to make these three codes' ERROR severity visible
through the same ``features.registry.severity_for()`` lookup every other
code uses, and to give the docs-table generator (``docs_table.py``) a row
for them. ``detect`` is therefore a permanent no-op — it never fires via
the standard hazard scan, and `run_detection` never calls it in anger.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import DetectionContext, Diagnostic, Severity, Support
from colcon_lf.features.registry import FeatureModule, register

CODE_R001 = "RCLLF-R001"
CODE_R002 = "RCLLF-R002"
CODE_R003 = "RCLLF-R003"


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Always empty — see module docstring."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="resolved_model.gate",
        title="Resolved-model fail-closed gate (timer period, tf2/take, staleness)",
        support=Support.REJECT,
        codes=(CODE_R001, CODE_R002, CODE_R003),
        severity={
            CODE_R001: Severity.ERROR,
            CODE_R002: Severity.ERROR,
            CODE_R003: Severity.ERROR,
        },
        detect=detect,
        conversion=(
            "RCLLF-R001 (unresolved timer period): supply the value "
            "explicitly, set nodes.<node>.timers.<timer>.period in "
            "execution.yaml, or acknowledge it. RCLLF-R002 (unsupported "
            "mechanism): for tf2 the subcode names the conversion "
            "(tf2_direct_lookup / tf2_blocking_lookup: use "
            "rcllf_tf2::AcquisitionLookup; tf2_listener_unadmitted: one "
            "TransformListener(*buffer, this) member; tf2_clock_mismatch: "
            "node-clock Buffer, /tf broadcast by a node of this system); take() has "
            "none, acknowledge to proceed as unsupported. RCLLF-R003 (stale "
            "resolved model): re-run colcon lf-generate."
        ),
        # No PlaygroundCase: these codes fire from resolved_model.resolve()'s
        # records at generate time, not from analyze-time diagnostics, so
        # they cannot plug into the playground harness's analyze-then-check
        # flow. tests/resolved_model/test_gate.py is the regression vehicle.
        playground=None,
        docs_entry=(
            "resolved configuration model with timer period (R001), tf2/take "
            "mechanism (R002 with its tf2 subcodes) and staleness (R003) (error, refuses "
            "generation unless acknowledged)"
        ),
    )
)

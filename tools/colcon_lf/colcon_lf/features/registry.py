# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``FeatureModule`` + ``REGISTRY`` + ``run_detection`` — the feature-module registry.

A frozen dataclass with
``detect`` as a callable, driven off a module-owned ``codes`` tuple and a
``severity`` map that covers exactly those codes. Severity is
the module's policy, resolved at *read* time — never persisted alongside a
``Diagnostic``.

Invariants (stated in the design, asserted here):
    - a module's emitted diagnostic codes are a subset of ``module.codes``.
    - ``module.severity`` covers *exactly* ``module.codes`` (no more, no less).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    PlaygroundCase,
    Severity,
    Support,
)


@dataclass(frozen=True)
class FeatureModule:
    """One feature/hazard-detection module — the extension axis for ``features/``."""

    feature_id: str  # "qos.best_effort_input"
    title: str
    support: Support
    codes: tuple[str, ...]  # diagnostic codes this module OWNS
    severity: dict[str, Severity]  # code -> severity; REGISTRY owns policy
    detect: Callable[[DetectionContext], list[Diagnostic]]
    conversion: str | None  # rationale for warn/reject; hook name later
    playground: PlaygroundCase | None  # Phase 7 fills in; shape fixed now
    docs_entry: str  # features-table cell; docs anchor derived from code

    def __post_init__(self) -> None:
        """Assert the codes/severity invariant at construction time."""
        codes_set = set(self.codes)
        severity_keys = set(self.severity.keys())
        if severity_keys != codes_set:
            missing = codes_set - severity_keys
            extra = severity_keys - codes_set
            raise AssertionError(
                f"FeatureModule '{self.feature_id}': severity map must cover "
                f"exactly `codes` (missing={sorted(missing)}, extra={sorted(extra)})"
            )


REGISTRY: dict[str, FeatureModule] = {}


def register(module: FeatureModule) -> FeatureModule:
    """Register *module* in ``REGISTRY`` keyed by ``feature_id``. Returns *module*."""
    if module.feature_id in REGISTRY:
        raise AssertionError(f"duplicate feature_id in REGISTRY: '{module.feature_id}'")
    REGISTRY[module.feature_id] = module
    return module


def severity_for(code: str) -> Severity:
    """Resolve *code*'s severity from the current ``REGISTRY`` (read-time policy)."""
    for module in REGISTRY.values():
        if code in module.severity:
            return module.severity[code]
    raise KeyError(f"no registered feature module owns diagnostic code '{code}'")


def run_detection(ctx: DetectionContext) -> list[Diagnostic]:
    """Run every registered module's ``detect`` against *ctx*.

    Asserts the emitted-codes-subset-of-module.codes invariant
    for every diagnostic a module returns.
    """
    diagnostics: list[Diagnostic] = []
    for module in REGISTRY.values():
        emitted = module.detect(ctx)
        for diag in emitted:
            if diag.code not in module.codes:
                raise AssertionError(
                    f"module '{module.feature_id}' emitted diagnostic code "
                    f"'{diag.code}' not in its declared codes {module.codes!r}"
                )
        diagnostics.extend(emitted)
    return diagnostics


# Import modules for their registration side-effects (populates REGISTRY).
# Done at the bottom of this module to avoid a circular import (modules import
# `register`/`FeatureModule` from this module).
from colcon_lf.features.modules import (  # noqa: E402,F401
    actions,
    best_effort_input,
    blocking_service_client,
    callback_groups,
    compile_variant_ambiguous,
    components,
    correspondence,
    dynamic_topic,
    endpoint_created_in_callback,
    intra_process,
    lifecycle,
    lifecycle_activation,
    main_outside_callbacks,
    message_filters,
    message_filters_sync,
    opaque_callback,
    parameters,
    pubsub,
    qos_unknown,
    request_from_service_callback,
    resolved_model_gate,
    ros_time_sleep,
    services,
    shallow_queue,
    thread_action_server,
    sync_bna,
    template_base_endpoints,
    timer_created_in_callback,
    timers,
)

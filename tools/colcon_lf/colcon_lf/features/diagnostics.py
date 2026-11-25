# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Diagnostic shape + severity/support enums + detection context.

A
``Diagnostic`` persists **code + facts only** (code, node, topic, message) —
no severity. Severity is resolved from the feature ``REGISTRY`` at *read*
time, so demoting/promoting a hazard's severity never requires re-analysis
(``system.json`` stays valid data, not frozen policy).
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph


class Support(enum.Enum):
    """How well a feature module's underlying capability is supported today."""

    SUPPORTED = "supported"
    WARN = "warn"
    REJECT = "reject"
    STUB = "stub"


class Severity(enum.Enum):
    """Diagnostic severity — resolved from the REGISTRY at read time, never persisted."""

    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class Outcome(enum.Enum):
    """Expected end-to-end result of running the playground CI harness on a case.

    Semantics:

    - ``PASS``: analyze succeeds with no WARNING/ERROR-severity diagnostics
      (INFO is allowed — e.g. RCLLF-H000 may still fire); the generate
      hazard-gate proceeds.
    - ``WARN``: at least one WARNING-severity diagnostic is present; the
      generate hazard-gate still proceeds (warnings never block).
    - ``REJECT``: at least one ERROR-severity diagnostic is present; the
      generate hazard-gate refuses to proceed unless the code is listed in
      ``execution.yaml``'s ``acknowledged_hazards``.

    A case's ``expected_codes`` are asserted as a **subset** of the codes
    actually observed on that launch, not an exact set — co-firing modules
    sharing a launch file must not break each other's playground case.
    """

    PASS = "pass"
    WARN = "warn"
    REJECT = "reject"


@dataclass(frozen=True)
class PlaygroundCase:
    """A playground example a feature module is demonstrated/CI-asserted against.

    Shape fixed in Phase 5 so Phase 7's auto-generated playground table needs
    no rework; ``playground`` on ``FeatureModule`` is ``None``
    until Phase 7 fills it in. See ``Outcome`` for ``expected_outcome``
    semantics.
    """

    example: str  # package/launch path, e.g. "pubsub_example/bringup"
    expected_outcome: Outcome
    expected_codes: tuple[str, ...] = ()


@dataclass
class Diagnostic:
    """One detected hazard/fact. Persists code + facts only — no severity.

    ``node``/``topic`` are optional because not every diagnostic is
    subscription-scoped (e.g. a node-level hazard could omit ``topic``).
    """

    code: str  # "RCLLF-H001"
    node: str | None
    topic: str | None
    message: str

    def to_dict(self) -> dict:
        """Serialize to the facts-only shape persisted in ``system.json``."""
        return {
            "code": self.code,
            "node": self.node,
            "topic": self.topic,
            "message": self.message,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Diagnostic":
        """Deserialize from the facts-only shape persisted in ``system.json``."""
        return cls(
            code=data["code"],
            node=data.get("node"),
            topic=data.get("topic"),
            message=data["message"],
        )


@dataclass
class DetectionContext:
    """Wraps ``SystemGraph`` for feature-module detection.

    A dedicated wrapper (rather than passing ``SystemGraph`` directly) so a
    module's ``detect`` signature never needs to change when a future
    detector needs more than the graph — e.g.
    per-node source paths. Today it only carries the graph.
    """

    graph: "SystemGraph"

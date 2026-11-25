# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``Record``/``ResolvedModel`` dataclasses, canonicalization and hashing.

Schema:

    {"schema": "rcllf.resolved_model", "schema_version": 1,
     "system": {"name": ..., "system_json_schema_version": ...},
     "claim_status": "supported"|"partial"|"unsupported",
     "records": [Record, ...],           # sorted by contract_id
     "mutation_channels": {name: state},
     "model_hash": "sha256:<hex>",       # outside the hash itself
     "inputs": [{"role", "path", "sha256"}, ...]}   # outside the hash

Canonicalization rules (design section "Canonicalization and hash"):

1. dict keys sorted at every depth (``json.dumps(sort_keys=True)``).
2. lists sorted by a declared key: records by ``contract_id``.
3. floats via ``json``'s shortest round-trip ``repr``; ints stay ints;
   NaN/inf rejected (``allow_nan=False``).
4. no timestamps, hostnames or tool versions in the body.
5. no absolute paths in the body — callers relativize before constructing a
   ``Record``/``ResolvedModel``.
6. YAML values keep their loaded type (``10`` int, ``10.0`` float).

``model_hash`` is computed over everything *except* ``inputs`` and
``model_hash`` itself, so re-hashing the persisted document must strip those
two keys first (``canonical_bytes`` does this from the dataclass form; a
loaded-from-JSON dict can call ``canonical_bytes_from_dict``).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Any

SCHEMA = "rcllf.resolved_model"
SCHEMA_VERSION = 1

# Provenance values (design "Properties, sources, provenance" table).
PROVENANCE_EXTRACTED = "extracted"
PROVENANCE_EXPLICITLY_CONFIGURED = "explicitly_configured"
PROVENANCE_MEASURED_PROFILED = "measured_profiled"
PROVENANCE_UNRESOLVED = "unresolved"
VALID_PROVENANCE = frozenset({
    PROVENANCE_EXTRACTED,
    PROVENANCE_EXPLICITLY_CONFIGURED,
    PROVENANCE_MEASURED_PROFILED,
    PROVENANCE_UNRESOLVED,
})

# compatibility_mode values.
COMPAT_PRESERVED = "preserved"
COMPAT_UNSUPPORTED = "unsupported"
VALID_COMPATIBILITY_MODE = frozenset({COMPAT_PRESERVED, COMPAT_UNSUPPORTED})

# claim_status values.
CLAIM_SUPPORTED = "supported"
CLAIM_PARTIAL = "partial"
CLAIM_UNSUPPORTED = "unsupported"
VALID_CLAIM_STATUS = frozenset({CLAIM_SUPPORTED, CLAIM_PARTIAL, CLAIM_UNSUPPORTED})


@dataclass
class Record:
    """One resolved temporal-contract property.

    ``contract_id`` is ``<kind>:<subject path>#<property>`` (e.g.
    ``timer:/imu_node/timer_callback#period``), stable across configurations.
    """

    contract_id: str
    property: str
    subject: dict[str, str]
    value: Any
    provenance: str
    evidence: str
    unit: str | None = None
    domain: str | None = None
    compatibility_mode: str = COMPAT_PRESERVED
    reason: str | None = None  # set when provenance == unresolved

    def __post_init__(self) -> None:
        if self.provenance not in VALID_PROVENANCE:
            raise ValueError(
                f"Record {self.contract_id!r}: invalid provenance {self.provenance!r} "
                f"(expected one of {sorted(VALID_PROVENANCE)})"
            )
        if self.compatibility_mode not in VALID_COMPATIBILITY_MODE:
            raise ValueError(
                f"Record {self.contract_id!r}: invalid compatibility_mode "
                f"{self.compatibility_mode!r} (expected one of "
                f"{sorted(VALID_COMPATIBILITY_MODE)})"
            )
        if isinstance(self.value, float) and (
            self.value != self.value or self.value in (float("inf"), float("-inf"))
        ):
            raise ValueError(f"Record {self.contract_id!r}: value must not be NaN/inf")

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "contract_id": self.contract_id,
            "property": self.property,
            "subject": dict(self.subject),
            "value": self.value,
            "provenance": self.provenance,
            "evidence": self.evidence,
            "compatibility_mode": self.compatibility_mode,
        }
        if self.unit is not None:
            out["unit"] = self.unit
        if self.domain is not None:
            out["domain"] = self.domain
        if self.reason is not None:
            out["reason"] = self.reason
        return out

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "Record":
        return cls(
            contract_id=raw["contract_id"],
            property=raw["property"],
            subject=dict(raw["subject"]),
            value=raw["value"],
            provenance=raw["provenance"],
            evidence=raw["evidence"],
            unit=raw.get("unit"),
            domain=raw.get("domain"),
            compatibility_mode=raw.get("compatibility_mode", COMPAT_PRESERVED),
            reason=raw.get("reason"),
        )


@dataclass
class ResolvedModel:
    """The full ``resolved_model.json`` document."""

    system: dict[str, Any]  # {"name": ..., "system_json_schema_version": ...}
    claim_status: str
    records: list[Record]
    mutation_channels: dict[str, str]
    inputs: list[dict[str, str]] = field(default_factory=list)
    model_hash: str = ""
    schema: str = SCHEMA
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.claim_status not in VALID_CLAIM_STATUS:
            raise ValueError(
                f"claim_status {self.claim_status!r} not in {sorted(VALID_CLAIM_STATUS)}"
            )

    def sorted_records(self) -> list[Record]:
        return sorted(self.records, key=lambda r: r.contract_id)

    def body_dict(self) -> dict[str, Any]:
        """The hashed portion: everything except ``inputs`` and ``model_hash``."""
        return {
            "schema": self.schema,
            "schema_version": self.schema_version,
            "system": dict(self.system),
            "claim_status": self.claim_status,
            "records": [r.to_dict() for r in self.sorted_records()],
            "mutation_channels": dict(self.mutation_channels),
        }

    def to_dict(self) -> dict[str, Any]:
        """The full persisted document, including ``inputs`` and ``model_hash``."""
        out = self.body_dict()
        out["model_hash"] = self.model_hash
        out["inputs"] = list(self.inputs)
        return out

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "ResolvedModel":
        return cls(
            system=dict(raw["system"]),
            claim_status=raw["claim_status"],
            records=[Record.from_dict(r) for r in raw["records"]],
            mutation_channels=dict(raw["mutation_channels"]),
            inputs=list(raw.get("inputs", [])),
            model_hash=raw.get("model_hash", ""),
            schema=raw.get("schema", SCHEMA),
            schema_version=raw.get("schema_version", SCHEMA_VERSION),
        )


def _canonical_dumps(body: dict[str, Any]) -> bytes:
    """Serialize *body* per the canonicalization rules, as UTF-8 bytes.

    ``sort_keys=True`` handles rule 1 (keys sorted at every depth) and, given
    ``Record.to_dict()``'s already-deterministic field set, rule 2's "subject
    fields fixed". ``allow_nan=False`` implements rule 3's NaN/inf rejection;
    the same call's default float formatting is already Python's shortest
    round-trip ``repr``, and ints are untouched, so ``10`` and ``10.0`` never
    collide (rule 6).
    """
    try:
        text = json.dumps(
            body,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    except ValueError as exc:
        raise ValueError(f"resolved model body is not hashable: {exc}") from exc
    return text.encode("utf-8")


def canonical_bytes(model: ResolvedModel) -> bytes:
    """The exact byte string ``model_hash`` is computed over."""
    return _canonical_dumps(model.body_dict())


def model_hash(model: ResolvedModel) -> str:
    """``"sha256:" + hexdigest`` of :func:`canonical_bytes`."""
    digest = hashlib.sha256(canonical_bytes(model)).hexdigest()
    return f"sha256:{digest}"


def finalize(model: ResolvedModel) -> ResolvedModel:
    """Return a copy of *model* with ``model_hash`` computed and set."""
    model.model_hash = model_hash(model)
    return model


def to_json(model: ResolvedModel, *, indent: int | None = 2) -> str:
    """Pretty (or compact, ``indent=None``) JSON for the persisted file.

    Does not require ``model.model_hash`` to already be set; call
    :func:`finalize` first if it should be populated.
    """
    return json.dumps(model.to_dict(), indent=indent, sort_keys=True, ensure_ascii=False)


def from_json(text: str) -> ResolvedModel:
    return ResolvedModel.from_dict(json.loads(text))

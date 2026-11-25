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
"""Claims, their three statuses and the evidence classes they accept.

A claim is `supported` only when event-stream integrity is `complete`,
coverage is `covered` and reconciliation is `reconciled`. Heuristic
timestamp-paired results (legacy `e2e_latency`, `determinism`) are physical
profiles: assigning one to a correspondence, chain or logical-reproducibility
field raises :class:`EvidenceRejected`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

EVIDENCE_IDENTITY = "identity"
EVIDENCE_HEURISTIC = "heuristic_timestamp_pairing"
EVIDENCE_CLASSES = (EVIDENCE_IDENTITY, EVIDENCE_HEURISTIC)

#: claim or profile field -> accepted evidence classes
ACCEPTED_EVIDENCE: dict[str, frozenset[str]] = {
    "correspondence": frozenset({EVIDENCE_IDENTITY}),
    "chain": frozenset({EVIDENCE_IDENTITY}),
    "logical_reproducibility": frozenset({EVIDENCE_IDENTITY}),
    "physical_hop_latency": frozenset({EVIDENCE_IDENTITY}),
    "drop_accounting": frozenset({EVIDENCE_IDENTITY}),
    "physical_profile": frozenset({EVIDENCE_IDENTITY, EVIDENCE_HEURISTIC}),
}

#: legacy metrics whose results are heuristic physical profiles
HEURISTIC_METRICS = ("e2e_latency", "determinism")

INTEGRITY = ("complete", "incomplete")
COVERAGE = ("covered", "uncovered", "unsupported")
RECONCILIATION = ("reconciled", "unreconciled", "not_evaluated")
OVERALL = ("supported", "incomplete", "unsupported", "contradicted")

REPORT_SCHEMA = "rcllf.semantic_report/1"
SCHEMA_FILE = Path(__file__).with_name("semantic_report.v1.json")


class EvidenceRejected(ValueError):
    """A result of the wrong evidence class was assigned to a claim field."""


def evidence_of(result: Any) -> str:
    """Evidence class of a metric result (MetricResult or its dict form)."""
    if isinstance(result, dict):
        return result.get("evidence") or ""
    return getattr(result, "evidence", "") or ""


def assign(claim_field: str, result: Any) -> Any:
    """Return *result* if its evidence class may fill *claim_field*, else raise."""
    if claim_field not in ACCEPTED_EVIDENCE:
        raise KeyError(f"unknown claim field {claim_field!r}")
    ev = evidence_of(result)
    if ev not in ACCEPTED_EVIDENCE[claim_field]:
        need = " or ".join(sorted(ACCEPTED_EVIDENCE[claim_field]))
        raise EvidenceRejected(f"{claim_field} requires {need} evidence, got {ev or 'none'}")
    return result


@dataclass
class Claim:
    """One claim subject with its three statuses and codes."""

    subject: str
    kind: str  # correspondence | chain | physical_hop_latency | drop_accounting
    integrity: str = "complete"
    integrity_codes: list[str] = field(default_factory=list)
    coverage: str = "covered"
    coverage_codes: list[str] = field(default_factory=list)
    reconciliation: str = "reconciled"
    reconciliation_codes: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        assert self.integrity in INTEGRITY, self.integrity
        assert self.coverage in COVERAGE, self.coverage
        assert self.reconciliation in RECONCILIATION, self.reconciliation
        # Reconciliation is not evaluated when either status above fails.
        if self.integrity != "complete" or self.coverage != "covered":
            self.reconciliation = "not_evaluated"
            self.reconciliation_codes = []

    @property
    def overall(self) -> str:
        if self.coverage == "unsupported":
            return "unsupported"
        if self.reconciliation == "unreconciled":
            return "contradicted"
        if (self.integrity, self.coverage, self.reconciliation) == ("complete", "covered", "reconciled"):
            return "supported"
        return "incomplete"

    @property
    def codes(self) -> list[str]:
        return list(self.integrity_codes) + list(self.coverage_codes) + list(self.reconciliation_codes)

    def to_dict(self) -> dict[str, Any]:
        return {
            "subject": self.subject,
            "kind": self.kind,
            "integrity": self.integrity,
            "integrity_codes": list(self.integrity_codes),
            "coverage": self.coverage,
            "coverage_codes": list(self.coverage_codes),
            "reconciliation": self.reconciliation,
            "reconciliation_codes": list(self.reconciliation_codes),
            "overall": self.overall,
        }


def status_text(state: str, codes: Sequence[str]) -> str:
    return f"{state}({', '.join(codes)})" if codes else state


# --- clock labels -----------------------------------------------------------------


def _manifest_hosts(run_dir: Path) -> list[dict[str, Any]]:
    path = Path(run_dir) / "trace_manifest.json"
    try:
        return list(json.loads(path.read_text()).get("hosts") or [])
    except (OSError, ValueError):
        return []


def _sync_bound(hosts: Sequence[dict[str, Any]]) -> Optional[float]:
    """Largest measured |offset| + error over every host's start and end sample."""
    bound = 0.0
    for h in hosts:
        for key in ("clock_sync_start", "clock_sync_end"):
            s = h.get(key) or {}
            if "offset_s" not in s or "error_s" not in s:
                return None
            bound = max(bound, abs(float(s["offset_s"])) + float(s["error_s"]))
    return bound


def clock_label(run_dirs: Iterable[Path] = (), boot_ids: Iterable[str] = ()) -> dict[str, Any]:
    """`same_host` when one boot_id is present, else `cross_host` with the
    measured sync bound from the run manifests, or `uncertainty: unknown`."""
    hosts: list[dict[str, Any]] = []
    for d in run_dirs:
        hosts.extend(_manifest_hosts(d))
    ids = {h.get("boot_id") for h in hosts if h.get("boot_id")} | {b for b in boot_ids if b}
    if len(ids) <= 1:
        return {"clock_domain": "same_host"}
    bound = _sync_bound(hosts) if hosts else None
    if bound is None:
        return {"clock_domain": "cross_host", "uncertainty": "unknown"}
    return {"clock_domain": "cross_host", "uncertainty_s": bound}


def label_heuristic(result: Any, run_dirs: Iterable[Path] = (), boot_ids: Iterable[str] = ()) -> Any:
    """Mark a legacy timestamp-paired result as a heuristic physical profile."""
    result.evidence = EVIDENCE_HEURISTIC
    result.clock = clock_label(run_dirs, boot_ids)
    return result


def claims_block(claims: Sequence[Claim], extra: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    """The `semantic` block of metrics.json (schema `rcllf.semantic_report/1`)."""
    doc: dict[str, Any] = {"schema": REPORT_SCHEMA, "claims": [c.to_dict() for c in claims]}
    doc.update(extra or {})
    return doc


def render_claims_table(block: dict[str, Any]) -> str:
    """Markdown "Claims" table for the report."""
    rows = block.get("claims") or []
    if not rows:
        return "_no semantic claims (no rcllf trace)_\n"
    out = ["| Subject | Kind | Integrity | Coverage | Reconciliation | Overall |",
           "|---|---|---|---|---|---|"]
    for c in rows:
        out.append(
            f"| {c['subject']} | {c['kind']} | {status_text(c['integrity'], c['integrity_codes'])} | "
            f"{status_text(c['coverage'], c['coverage_codes'])} | "
            f"{status_text(c['reconciliation'], c['reconciliation_codes'])} | **{c['overall']}** |"
        )
    return "\n".join(out) + "\n"


def validate_block(block: dict[str, Any]) -> list[str]:
    """Check *block* against semantic_report.v1.json without a jsonschema dependency.

    Supports the subset the schema file uses (type, required, enum, items,
    properties, const). Returns a list of error strings, empty when valid.
    """
    schema = json.loads(SCHEMA_FILE.read_text())
    errors: list[str] = []

    def check(node: Any, sch: dict[str, Any], path: str) -> None:
        t = sch.get("type")
        types = {"object": dict, "array": list, "string": str, "boolean": bool, "integer": int, "number": (int, float)}
        if t and not isinstance(node, types[t]):
            errors.append(f"{path}: expected {t}")
            return
        if "const" in sch and node != sch["const"]:
            errors.append(f"{path}: expected {sch['const']!r}")
        if "enum" in sch and node not in sch["enum"]:
            errors.append(f"{path}: {node!r} not in {sch['enum']}")
        for req in sch.get("required", []):
            if isinstance(node, dict) and req not in node:
                errors.append(f"{path}: missing {req}")
        for key, sub in (sch.get("properties") or {}).items():
            if isinstance(node, dict) and key in node:
                check(node[key], sub, f"{path}.{key}")
        if "items" in sch and isinstance(node, list):
            for i, item in enumerate(node):
                check(item, sch["items"], f"{path}[{i}]")

    check(block, schema, "$")
    return errors

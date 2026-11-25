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
"""Claims, evidence classes and clock labels."""

from __future__ import annotations

import json

import pytest

from lf_eval.metrics.registry import MetricResult
from lf_eval.report import generate_report
from lf_eval.semantic import claims as c


def _heuristic(tmp_path=None):
    return c.label_heuristic(MetricResult(name="e2e_latency"), [tmp_path] if tmp_path else [])


def test_heuristic_result_rejected_for_correspondence():
    with pytest.raises(c.EvidenceRejected) as exc:
        c.assign("correspondence", _heuristic())
    assert str(exc.value) == (
        "correspondence requires identity evidence, got heuristic_timestamp_pairing"
    )


@pytest.mark.parametrize("claim_field", ["chain", "logical_reproducibility", "physical_hop_latency"])
def test_heuristic_result_rejected_for_every_identity_field(claim_field):
    with pytest.raises(c.EvidenceRejected):
        c.assign(claim_field, _heuristic().to_dict())


def test_heuristic_result_accepted_as_profile_and_identity_as_claim():
    assert c.assign("physical_profile", _heuristic()) is not None
    ident = MetricResult(name="semantic_correspondence", evidence=c.EVIDENCE_IDENTITY)
    assert c.assign("correspondence", ident) is ident


def test_unlabelled_result_rejected():
    with pytest.raises(c.EvidenceRejected, match="got none"):
        c.assign("chain", MetricResult(name="x"))


def test_no_end_run_renders_no_supported_claim(tmp_path):
    claim = c.Claim("edge 0 /chatter", "correspondence", integrity="incomplete",
                    integrity_codes=["no_end"], reconciliation="reconciled")
    assert claim.overall == "incomplete"
    assert claim.reconciliation == "not_evaluated"
    block = c.claims_block([claim])
    assert c.validate_block(block) == []
    report = generate_report([], {"metrics": {}, "semantic": block}, [], tmp_path).read_text()
    assert "## Claims" in report
    assert "incomplete(no_end)" in report
    assert "supported" not in report


@pytest.mark.parametrize(
    ("kw", "overall"),
    [
        ({}, "supported"),
        ({"coverage": "unsupported", "coverage_codes": ["provider_absent"]}, "unsupported"),
        ({"reconciliation": "unreconciled", "reconciliation_codes": ["summary_mismatch"]}, "contradicted"),
        ({"coverage": "uncovered", "coverage_codes": ["path_bypass"]}, "incomplete"),
    ],
)
def test_overall_status(kw, overall):
    assert c.Claim("s", "chain", **kw).overall == overall


def _manifest(run_dir, boot_id, sync=None):
    run_dir.mkdir()
    s = sync or {"source": "unavailable"}
    (run_dir / "trace_manifest.json").write_text(json.dumps(
        {"hosts": [{"host": boot_id, "boot_id": boot_id, "clock_sync_start": s, "clock_sync_end": s}]}))
    return run_dir


def test_two_boot_ids_label_cross_host_unknown(tmp_path):
    runs = [_manifest(tmp_path / "a", "boot-a"), _manifest(tmp_path / "b", "boot-b")]
    res = c.label_heuristic(MetricResult(name="e2e_latency"), runs)
    assert res.to_dict()["clock"] == {"clock_domain": "cross_host", "uncertainty": "unknown"}
    report = generate_report([], {"metrics": {"e2e_latency": res.to_dict()}}, [], tmp_path).read_text()
    assert "## Physical profiles (heuristic pairing)" in report
    assert "clock_domain: cross_host, uncertainty: unknown" in report


def test_cross_host_with_measured_sync_carries_the_bound(tmp_path):
    sync = {"source": "chrony", "offset_s": -0.001, "error_s": 0.004}
    runs = [_manifest(tmp_path / "a", "boot-a", sync), _manifest(tmp_path / "b", "boot-b", sync)]
    assert c.clock_label(runs) == {"clock_domain": "cross_host", "uncertainty_s": pytest.approx(0.005)}


def test_single_boot_id_is_same_host(tmp_path):
    assert c.clock_label([_manifest(tmp_path / "a", "boot-a")]) == {"clock_domain": "same_host"}


def test_schema_rejects_unknown_overall():
    block = c.claims_block([c.Claim("s", "chain")])
    block["claims"][0]["overall"] = "probably"
    assert c.validate_block(block) == ["$.claims[0].overall: 'probably' not in "
                                       "['supported', 'incomplete', 'unsupported', 'contradicted']"]

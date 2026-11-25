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
"""One test per D4 code, each a named mutation of a recorded golden (T7)."""

from __future__ import annotations

import pytest

from ..dev_only import requires_golden
from .golden_support import GOLDEN, Golden, text


@pytest.fixture
def g():
    return Golden("corr_many").copy()


def _integ(g, code, edge="0", kind="correspondence"):
    c = g.claim(edge, kind)
    assert code in c["integrity_codes"], f"expected incomplete({code}), got {text(c, 'integrity')}"
    assert c["overall"] == "incomplete", c
    assert c["reconciliation"] == "not_evaluated", c


def _cov(g, status, code, edge="0", kind="correspondence"):
    c = g.claim(edge, kind)
    assert (c["coverage"], code in c["coverage_codes"]) == (status, True), (
        f"expected {status}({code}), got {text(c, 'coverage')}")
    return c


def test_golden_is_supported(g):
    c = g.claim()
    assert c["overall"] == "supported", c


def test_no_run_manifest(g):
    g.run_manifest = None
    _integ(g, "no_run_manifest")


def test_discarded_rcllf_fails_only_rcllf_claims(g):
    g.run_manifest["channels"]["rcllf"]["discarded"] = 3
    _integ(g, "discarded:rcllf")
    g2 = Golden("corr_many").copy()
    g2.run_manifest["channels"]["ros2"]["discarded"] = 3
    assert g2.claim()["overall"] == "supported", "a ros2 drop must not fail a correspondence claim (Q2)"
    assert "discarded:ros2" in g2.claim("0", "physical_hop_latency")["integrity_codes"]


def test_discards_unknown(g):
    g.run_manifest["channels"]["rcllf"]["discarded"] = "unknown"
    _integ(g, "discards_unknown:rcllf")


def test_crosscheck_discard(g):
    g.stats.discarded_events = {"rcllf": 2}
    _integ(g, "crosscheck_discard")


def test_missing_producer(g):
    g.static["executables"] = g.static["executables"] + ["GhostExecutable"]
    _integ(g, "missing_producer:GhostExecutable")


def test_no_end(g):
    g.drop(lambda e: e.name == "run_end", resequence=False)
    _integ(g, "no_end")


def test_seq_gap(g):
    g.drop(lambda e: e.name == "publication" and e.seq == 5, resequence=False)
    _integ(g, "seq_gap:publication:5")


def test_seq_duplicate(g):
    g.events.append(g.first("publication"))
    _integ(g, "seq_duplicate")


def test_schema_version(g):
    g.first("run_begin").fields["schema_major"] = 2
    _integ(g, "schema_version")


def test_manifest_mismatch_missing_build_info(g):
    g.first("run_begin").fields["static_manifest_hash"] = ""
    _integ(g, "manifest_mismatch")


def test_read_error(g):
    g.stats.read_error = "RuntimeError: truncated packet"
    _integ(g, "read_error")


def test_provider_absent(g):
    g.static["provider_marker"] = None
    c = _cov(g, "unsupported", "provider_absent")
    assert c["overall"] == "unsupported"


def test_stock_variant_is_provider_absent(g):
    g.metadata = {"mode": "stock_single"}
    assert _cov(g, "unsupported", "provider_absent")["overall"] == "unsupported"


def test_provider_disabled_per_event(g):
    rules = g.run_manifest["channels"]["rcllf"]["rules"]
    rules[:] = [{"pattern": f"rcllf:{ev}", "type": "tracepoint", "enabled": True, "filter": None,
                 "exclusions": []} for ev in ("run_begin", "run_end", "backlog", "expectation", "terminal",
                                              "sample", "dispatch_begin", "dispatch_end")]
    c = _cov(g, "uncovered", "provider_disabled:publication")
    assert c["coverage_codes"] == ["provider_disabled:publication"], c
    assert c["overall"] == "incomplete"


def test_provider_filtered(g):
    g.run_manifest["channels"]["rcllf"]["rules"].insert(0, {
        "pattern": "rcllf:publication", "type": "tracepoint", "enabled": True,
        "filter": "ordinal > 3", "exclusions": []})
    g.run_manifest["channels"]["rcllf"]["rules"][1]["exclusions"] = ["rcllf:publication"]
    _cov(g, "uncovered", "provider_filtered:publication")


def test_provider_silent(g):
    end = g.first("run_end")
    end.fields["attempts"][3] = end.fields["totals"][3] + 4  # sample category
    _cov(g, "uncovered", "provider_silent:sample")


def test_legacy_edge(g):
    g.static["edges"][0] = dict(g.static["edges"][0], status="legacy", reason="publisher_callback")
    g.static["edges"][0].pop("id", None)
    c = [c for c in g.analyze()["claims"] if c["kind"] == "correspondence"][0]
    assert (c["coverage"], c["coverage_codes"]) == ("unsupported", ["legacy:publisher_callback"]), c


def test_uninstrumented_api(g):
    gid = g.first("expectation").gid()
    pub = next(e for e in g.events if e.name == "run_end_publisher" and e.gid() == gid)
    pub.fields["unsupported_api"] = 1
    _cov(g, "uncovered", "uninstrumented_api")


def test_path_bypass_from_identity_faults(g):
    g.first("run_end").fields["identity_faults"] = 1
    c = _cov(g, "uncovered", "path_bypass")
    assert c["reconciliation"] == "not_evaluated"


def test_path_bypass_from_sample_without_publication(g):
    victim = next(e for e in g.events if e.name == "publication" and e.fields["origin"] == 0)
    g.drop(lambda e: e is victim)
    _cov(g, "uncovered", "path_bypass")


def test_ros2_provider_missing(g):
    del g.run_manifest["channels"]["ros2"]
    _cov(g, "uncovered", "ros2_provider_missing", kind="physical_hop_latency")
    assert g.claim()["overall"] == "supported", "correspondence does not read ros2"


def test_clock_domain_for_cross_host_physical_claim(g):
    other = g.copy()
    for e in other.events:
        e.epoch += 1
        if e.name == "run_begin":
            e.fields["boot_id"] = "another-host-boot-id"
    g.events += [e for e in other.events if e.name == "run_begin"]
    _cov(g, "unsupported", "clock_domain", kind="physical_hop_latency")


@requires_golden(GOLDEN / "stock_sentry")
def test_recorded_stock_sentry_run_is_provider_absent():
    """Output case 8 on a real traced stock run (lf-bench, stock-single, sentry_pipeline)."""
    from pathlib import Path

    from lf_eval.semantic.reconcile import analyze_run

    block = analyze_run(Path(__file__).parent / "golden" / "stock_sentry")
    claims = [c for c in block["claims"] if c["kind"] == "correspondence"]
    assert len(claims) == 8, claims
    for c in claims:
        assert (c["overall"], c["coverage_codes"]) == ("unsupported", ["provider_absent"]), c


def test_publisher_rids_reads_rids_then_falls_back_to_rid():
    """`rids` list, an older manifest's single `rid`, or nothing."""
    from lf_eval.semantic.completeness import publisher_rids

    assert publisher_rids({"publisher": {"rids": [3, 4]}}) == [3, 4]
    assert publisher_rids({"publisher": {"rid": 3}}) == [3]
    assert publisher_rids({"publisher": {"rids": [5], "rid": 3}}) == [5], "rids wins"
    assert publisher_rids({"publisher": {}}) == []
    assert publisher_rids({}) == []


def test_single_rid_manifest_and_rids_list_give_identical_claims(g):
    """The golden carries the `publisher.rids` list;
    rewriting it to the older single `publisher.rid` must not change a claim."""
    assert all(len(e["publisher"].get("rids", [])) == 1 for e in g.static["edges"]), (
        "golden is not a recent one with one publishing RID per edge")
    base = g.analyze()["claims"]
    for e in g.static["edges"]:
        e["publisher"]["rid"] = e["publisher"].pop("rids")[0]
    assert g.analyze()["claims"] == base


def test_manifest_publisher(g):
    """An admitted edge whose manifest names no publishing RID fails closed."""
    g.static["edges"][0]["publisher"].pop("rids")
    c = _cov(g, "unsupported", "manifest_publisher")
    assert c["overall"] != "supported", c

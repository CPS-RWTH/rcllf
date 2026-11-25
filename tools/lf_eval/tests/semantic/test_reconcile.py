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
"""Reconciliation, chains and identity metrics on recorded goldens (T8)."""

from __future__ import annotations

import json
import random

from lf_eval.semantic.model import build_epochs

from .golden_support import Golden, text


def _metric(block, name):
    return block["metrics"][name]["data"]


def test_golden_reconstructs_every_chain_without_timestamps():
    g = Golden("corr_many")
    block = g.analyze()
    chains = _metric(block, "semantic_chain")["chains"]
    assert len(chains) == 10
    hops = [h for c in chains for h in c["hops"]]
    assert len(hops) == 60 and all(h["outcome"] == "MATCHED" for h in hops), hops[:3]
    assert all(h["logical_delta_ns"] == 1_000_000 for h in hops), "d = 1 ms on every hop"


def test_permuted_timestamps_leave_identity_results_identical():
    """Falsification: identity results never read a timestamp."""
    g = Golden("corr_many")
    base = g.analyze()
    p = g.copy()
    ts = [e.ts for e in p.events]
    random.Random(88).shuffle(ts)
    for e, t in zip(p.events, ts):
        e.ts = t
    perm = p.analyze()
    for name in ("semantic_correspondence", "semantic_chain"):
        a, b = _metric(base, name), _metric(perm, name)
        if a != b:
            first = next((x for x, y in zip(a.get("chains", []), b.get("chains", [])) if x != y), None)
            raise AssertionError(f"{name} changed under timestamp permutation; first differing chain: "
                                 f"{json.dumps(first)[:300] if first else 'n/a'}")
    assert json.dumps(base["claims"], sort_keys=True) == json.dumps(perm["claims"], sort_keys=True)


def test_orphan_dispatch():
    g = Golden("corr_many").copy()
    b = next(e for e in g.events if e.name == "dispatch_begin" and e.fields["edge"] == 0)
    b.fields["ordinal"] = 10_000
    c = g.claim()
    assert "orphan_dispatch" in c["reconciliation_codes"], text(c, "reconciliation")
    assert c["overall"] == "contradicted"


def test_missing_terminal():
    g = Golden("corr_many").copy()
    victim = next(e for e in g.events if e.name == "terminal" and e.fields["edge"] == 0)
    g.drop(lambda e: e is victim)
    c = g.claim()
    assert {"missing_terminal", "summary_mismatch"} <= set(c["reconciliation_codes"]), text(c, "reconciliation")
    assert c["overall"] == "contradicted"


def test_summary_mismatch():
    g = Golden("corr_many").copy()
    end = next(e for e in g.events if e.name == "run_end_edge" and e.fields["edge"] == 0)
    end.fields["matched"] += 1
    c = g.claim()
    assert c["reconciliation_codes"] == ["summary_mismatch"], text(c, "reconciliation")


def test_in_process_reconcile_failure():
    g = Golden("corr_many").copy()
    g.first("run_end").fields["reconciled"] = 0
    assert "in_process" in g.claim()["reconciliation_codes"]


def test_arrival_lag_is_arrival_minus_release_tag_per_key():
    g = Golden("corr_many")
    lag = _metric(g.analyze(), "semantic_arrival_lag")["0"]
    ep = max(build_epochs(g.events).values(), key=lambda e: len(e.expectations))
    release = {(b.gid(), b.fields["ordinal"]): b.fields["tag_time"]
               for b in ep.dispatch_begin.values() if b.fields["edge"] == 0}
    arrival = {(s.gid(), s.fields["ordinal"]): s.fields["arrival_physical_ns"]
               for s in ep.samples if s.fields["edge"] == 0}
    assert len(lag) == 30
    for row in lag:
        k = (row["gid"], row["ordinal"])
        assert row["lag_ns"] == arrival[k] - release[k], row


def test_dropped_seq_golden_is_a_gap():
    """Output case 5 from its own recorded run (RCLLF_TEST_TRACE_DROP_SEQ=publication:5)."""
    c = Golden("corr_drop_seq").claim()
    got = text(c, "integrity") if c["overall"] != "supported" else "supported"
    assert c["integrity_codes"] == ["seq_gap:publication:5"], f"expected incomplete(seq_gap:publication:5), got {got}"


def test_skip_run_end_golden_is_no_end():
    """Output case 6 (RCLLF_TEST_TRACE_SKIP_RUN_END) from its recorded run."""
    c = Golden("corr_skip_run_end").claim()
    got = text(c, "integrity") if c["overall"] != "supported" else "supported"
    assert "no_end" in c["integrity_codes"], f"expected incomplete(no_end), got {got}"


def test_no_publication_golden_is_provider_disabled():
    """Output case 3 from its recorded run (rcllf:publication not enabled)."""
    c = Golden("corr_no_publication").claim()
    got = text(c, "coverage") if c["overall"] != "supported" else "supported"
    assert c["coverage_codes"] == ["provider_disabled:publication"], (
        f"expected uncovered(provider_disabled:publication), got {got}")


# --- review fixes (2026-09-30): fail-closed on close records, producer faults, epochs ---

def test_missing_run_end_edge_is_incomplete():
    g = Golden("corr_many").copy()
    g.drop(lambda e: e.name == "run_end_edge" and e.fields["edge"] == 0, resequence=False)
    c = g.claim()
    got = text(c, "integrity") if c["overall"] != "supported" else "supported"
    assert "missing_close:edge" in c["integrity_codes"], f"expected incomplete(missing_close:edge), got {got}"


def test_missing_run_end_publisher_is_incomplete():
    g = Golden("corr_many").copy()
    victim = g.first("run_end_publisher")
    g.drop(lambda e: e is victim, resequence=False)
    c = g.claim()
    got = text(c, "integrity") if c["overall"] != "supported" else "supported"
    assert "missing_close:publisher" in c["integrity_codes"], (
        f"expected incomplete(missing_close:publisher), got {got}")


def test_producer_orphan_and_count_mismatch_counters_fail_the_edge():
    for field, code in (("orphan_dispatches", "producer_orphan_dispatch"),
                        ("count_mismatch", "producer_count_mismatch")):
        g = Golden("corr_many").copy()
        end = next(e for e in g.events if e.name == "run_end_edge" and e.fields["edge"] == 0)
        end.fields[field] = 1
        c = g.claim()
        assert code in c["reconciliation_codes"], f"{field}=1: expected {code}, got {text(c, 'reconciliation')}"
        assert c["overall"] != "supported"


def test_second_epoch_without_run_end_fails_every_claim():
    """A run with two traced processes: the non-primary epoch's integrity counts too."""
    import copy as _copy
    g = Golden("corr_many").copy()
    other = max(e.epoch for e in g.events) + 1
    begin = _copy.deepcopy(g.first("run_begin"))
    begin.epoch = other
    g.events.append(begin)  # second process started, never closed: no run_end
    c = g.claim()
    got = f"{text(c, 'integrity')} / {text(c, 'coverage')}" if c["overall"] != "supported" else "supported"
    assert "no_end" in c["integrity_codes"], f"expected incomplete(no_end) from the second epoch, got {got}"
    assert c["coverage"] == "unsupported" and "multi_epoch" in c["coverage_codes"], got


def test_untracked_slot_sharing_a_gid_does_not_hide_the_tracked_record():
    import copy as _copy
    g = Golden("corr_many").copy()
    # Edge 0's writer: the first tracked record by trace order can be a writer
    # without publications (/corr/b in scenario many), whose claim has no codes.
    gid = _edge0_gid(g)
    tracked = next(e for e in g.events
                   if e.name == "run_end_publisher" and int(e.fields["tracked"]) and e.gid() == gid)
    tracked.fields["unsupported_api"] = 1
    shadow = _copy.deepcopy(tracked)
    shadow.fields.update(tracked=0, unsupported_api=0)
    g.events.append(shadow)  # a later untracked slot with the same gid
    g.first("run_end").fields["n_publishers"] += 1
    codes = {x for c in g.analyze()["claims"] for x in c["coverage_codes"]}
    assert "uninstrumented_api" in codes, f"tracked record overwritten by untracked shadow, got {sorted(codes)}"


def _edge0_gid(g):
    return next(e for e in g.events if e.name == "expectation" and e.fields["edge"] == 0).gid()


def test_undeclared_publication_from_publisher_close_counter():
    """D7: a refused UNDECLARED_EFFECT publish on the edge's writer
    (counted in run_end_publisher) means the edge is not reconciled."""
    g = Golden("corr_many").copy()
    gid = _edge0_gid(g)
    end = next(e for e in g.events if e.name == "run_end_publisher" and e.gid() == gid)
    end.fields["refused_undeclared_effect"] = 1
    c = g.claim()
    assert "undeclared_publication" in c["reconciliation_codes"], text(c, "reconciliation")
    assert c["overall"] != "supported"


def test_undeclared_publication_from_publication_event():
    """Same, seen as an UNDECLARED_EFFECT `publication` event on the writer's GID."""
    import copy as _copy

    from lf_eval.semantic import schema

    g = Golden("corr_many").copy()
    gid = _edge0_gid(g)
    pubs = [e for e in g.events if e.name == "publication" and e.gid() == gid]
    extra = _copy.deepcopy(pubs[-1])
    extra.fields["origin"] = schema.ORIGINS["UNDECLARED_EFFECT"]
    extra.fields["ordinal"] = max(int(p.fields["ordinal"]) for p in pubs) + 1
    g.events.insert(g.events.index(pubs[-1]) + 1, extra)
    g.resequence()
    c = g.claim()
    assert "undeclared_publication" in c["reconciliation_codes"], text(c, "reconciliation")
    assert c["overall"] != "supported"


def test_sample_after_its_dispatch_pairs_by_key():
    """Ingress may emit `sample` after the dispatch_begin of the same key."""
    g = Golden("corr_many")
    base = g.analyze()
    p = g.copy()
    key = lambda e: (e.fields["edge"], e.gid(), e.fields["ordinal"])  # noqa: E731
    samples = {key(e): e for e in p.events if e.name == "sample"}
    pos = {(e.name, key(e)): i for i, e in enumerate(p.events) if e.name in ("sample", "dispatch_begin")}
    assert all(pos[("sample", k)] < pos[("dispatch_begin", k)] for k in samples
               if ("dispatch_begin", k) in pos), "the golden must start with sample first"
    rest = [e for e in p.events if e.name != "sample"]
    moved, out = 0, []
    for e in rest:
        out.append(e)
        s = samples.get(key(e)) if e.name == "dispatch_begin" else None
        if s is not None:
            s.ts = e.ts + 1
            out.append(samples.pop(key(e)))
            moved += 1
    p.events[:] = out + list(samples.values())
    assert moved == 60, moved  # 30 per edge, two edges
    perm = p.analyze()
    for name in ("semantic_correspondence", "semantic_chain", "semantic_arrival_lag"):
        assert _metric(base, name) == _metric(perm, name), name
    assert json.dumps(base["claims"], sort_keys=True) == json.dumps(perm["claims"], sort_keys=True)
    assert all(c["overall"] == "supported" for c in perm["claims"] if c["kind"] == "correspondence")

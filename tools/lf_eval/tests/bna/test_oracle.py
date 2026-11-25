# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""BNA oracle against the hand-derived expectations and the section 3.4 invariants."""

from __future__ import annotations

from collections import Counter
from pathlib import Path

import pytest

from lf_eval.bna import compare, corpus, oracle

CORPORA = Path(__file__).parent / "corpora"
CASES = sorted(p.stem for p in CORPORA.glob("*.bna"))


def test_every_plan_case_has_a_corpus():
    want = {"N1", "T1", "T2", "W1", "W2", "W3", "LA1", "TO1", "TO2", "TO3", "TO4", "CAP1",
            "CAP2", "CAP3", "CAP4", "R1", "RET1", "RET2", "SUP1", "SH1", "EP1", "CA1", "D1",
            "M1"}
    assert want <= set(CASES)


@pytest.mark.parametrize("case", CASES)
def test_oracle_matches_hand_expectation(case):
    events, expect = corpus.load_bna(CORPORA / f"{case}.bna")
    assert expect, f"{case} has no expect lines"
    assert [corpus.compact(r) for r in oracle.run(events)] == expect


def check_invariants(events: list[dict], out: list[dict]) -> None:
    """Section 3.4 on one corpus and its outputs."""
    p = oracle.Params.from_record(events[0])
    admits = [e for e in events if e["ev"] == "admit"]
    by_key = Counter(e["key"] for e in admits)
    outs = Counter(r["anchor"] if r["ev"] == "decision" else r["key"] for r in out)
    assert outs == by_key, "exactly one terminal outcome per admitted sample"
    for r in out:
        if r["ev"] == "sample":
            assert r["input"] == 1, "anchors only ever get decision records"
    used = Counter()
    decided = []
    for r in out:
        if r["ev"] == "decision" and r["outcome"] == "MATCHED":
            used[r["anchor"]] += 1
            used[r["partner"]] += 1
            assert abs(r["partner_stamp"] - r["stamp"]) <= p.delta
        if r["ev"] == "decision" and (r["outcome"] == "MATCHED" or r["reason"] == "missing"):
            decided.append((r["stamp"], oracle.split_key(r["anchor"])))
    assert all(n == 1 for n in used.values()), "no key in two tuples"
    assert decided == sorted(decided), "anchors decided in (s, k) order"
    tau = {}
    for e in admits:
        tau.setdefault(e["key"], []).append(e["tag"])
    for r in out:
        k = r["anchor"] if r["ev"] == "decision" else r["key"]
        span = p.timeout if r["ev"] == "decision" else p.retention
        latest = max(t[0] for t in tau[k]) + span
        assert tuple(r["tag"]) <= (latest, 0), f"{k} decided after its deadline"


@pytest.mark.parametrize("case", CASES)
def test_invariants_on_corpora(case):
    events, _ = corpus.load_bna(CORPORA / f"{case}.bna")
    check_invariants(events, oracle.run(events))


@pytest.mark.parametrize("seed", range(200))
def test_invariants_on_random_corpora(seed):
    events = corpus.random_corpus(seed)
    check_invariants(events, oracle.run(events))


def test_random_corpora_reach_every_outcome():
    seen = set()
    for seed in range(200):
        for r in oracle.run(corpus.random_corpus(seed)):
            seen.add(r["outcome"] + (f"({r['reason']})" if r["reason"] else ""))
    for want in ("MATCHED", "REJECTED(missing)", "REJECTED(timeout)", "REJECTED(late)",
                 "REJECTED(anchor_evicted)", "REJECTED(candidate_evicted)",
                 "REJECTED(candidate_expired)", "SELECTED", "LATE", "SUPERSEDED", "OVERFLOW",
                 "EXPIRED", "FLUSHED", "FAULT(duplicate)", "FAULT(order)"):
        assert want in seen, want


def _tuples(out):
    return sorted((r["anchor"], r["outcome"], r["reason"], r["partner"]) for r in out
                  if r["ev"] == "decision" and not r["anchor"].startswith("1:zz"))


@pytest.mark.parametrize("seed", range(20))
def test_order_insensitivity(seed):
    base = _tuples(oracle.run(corpus.permuted_corpus(seed, 0)))
    assert base and all(t[1] in ("MATCHED", "REJECTED") for t in base)
    assert all(t[2] in ("", "missing") for t in base)
    for perm in range(1, 10):
        assert _tuples(oracle.run(corpus.permuted_corpus(seed, perm))) == base


def test_missing_changes_no_decision():
    events, _ = corpus.load_bna(CORPORA / "M1.bna")
    without = [e for e in events if e["ev"] != "missing"]
    assert oracle.run(events) == oracle.run(without)


def test_wake_events_change_no_outcome():
    for seed in range(50):
        events = corpus.random_corpus(seed)
        assert oracle.run(events) == oracle.run([e for e in events if e["ev"] != "wake"])


def test_tags_must_not_go_back():
    events, _ = corpus.parse_bna("params epoch=1 delta=1 lateness=0,0 capacity=1,1 "
                                 "retention=5 timeout=5\nadmit 0 1 1:a:1 @2\nwake @1\n")
    with pytest.raises(oracle.OracleError, match="before"):
        oracle.run(events)


# -- comparator messages ----------------------------------------------------

def _out(case):
    events, _ = corpus.load_bna(CORPORA / f"{case}.bna")
    return events, oracle.run(events)


def test_compare_equal_is_none():
    events, out = _out("N1")
    assert compare.compare(events, out, events, list(out)) is None


def test_compare_decision_and_tag_messages():
    events, out = _out("T1")
    wrong = [dict(r) for r in out]
    wrong[0].update(partner="1:b:2", partner_stamp=105_000_000)
    assert compare.compare_outputs(events, out, wrong) == (
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle MATCHED "
        "1:b:1@95000000, implementation MATCHED 1:b:2@105000000")
    late = [dict(r) for r in out]
    late[0]["tag"] = [132_000_000, 0]
    assert compare.compare_outputs(events, out, late) == (
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle tag [131000000,0], "
        "implementation tag [132000000,0]")


def test_compare_checks_sample_input_and_stamps():
    events, out = _out("T1")
    i = next(n for n, r in enumerate(out) if r["ev"] == "sample")
    x = out[i]
    for change in ({"stamp": x["stamp"] + 1}, {"input": 1 - x["input"]}):
        wrong = [dict(r) for r in out]
        wrong[i].update(change)
        y = wrong[i]
        assert compare.compare_outputs(events, out, wrong) == (
            f"outcome_mismatch at sample {x['key']}: oracle input {x['input']} stamp "
            f"{x['stamp']}, implementation input {y['input']} stamp {y['stamp']}")
    wrong = [dict(r) for r in out]
    wrong[0]["stamp"] += 1
    assert compare.compare_outputs(events, out, wrong) == (
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle anchor stamp 100000000, "
        "implementation anchor stamp 100000001")


def test_compare_counts_outcomes_per_admission():
    events, out = _out("LA1")
    double = [out[0], dict(out[0], ev="sample", input=0, key="1:a:2", outcome="LATE",
                           reason=""), out[1]]
    assert compare.compare_outputs(events, out, double) == (
        "outcome_count_mismatch at sample 1:a:2: 1 admission(s), implementation 2 "
        "outcome(s) (REJECTED(late), LATE)")


def test_compare_inputs_reports_params_first():
    events, _ = _out("N1")
    other = [dict(events[0], delta=0)] + events[1:]
    assert compare.compare_inputs(events, other) == (
        "params_mismatch: delta expected 10000000, implementation 0")
    shifted = events[:3] + [dict(events[3], tag=[52_000_000, 0])] + events[4:]
    assert compare.compare_inputs(events, shifted).startswith("input_mismatch at event 2: ")


def test_stamped_missing_raises_the_watermark():
    # A loss report with a stamp bound decides a pending anchor at
    # the report; without the stamp the anchor waits for its timeout.
    MS = 1_000_000
    params = {"ev": "params", "sync": 0, "epoch": 1, "delta": 10 * MS, "lateness": [0, 0],
              "capacity": [4, 4], "retention": 300 * MS, "timeout": 100 * MS}
    base = [params,
            {"ev": "admit", "sync": 0, "input": 0, "stamp": 0, "key": "1:a:1", "tag": [0, 1]},
            {"ev": "admit", "sync": 0, "input": 1, "stamp": 0, "key": "1:c:1", "tag": [0, 2]},
            {"ev": "admit", "sync": 0, "input": 0, "stamp": 30 * MS, "key": "1:a:2",
             "tag": [10 * MS, 1]},
            {"ev": "missing", "sync": 0, "input": 1, "key": "1:c:2", "tag": [20 * MS, 1]},
            {"ev": "close", "sync": 0, "reason": "shutdown", "tag": [500 * MS, 0]}]
    plain = [(r["outcome"], r["tag"]) for r in oracle.run(base) if r["ev"] == "decision"]
    # Timeouts: the candidate watermark never passes A1's 0 + 10 ms.
    assert plain == [("REJECTED", [100 * MS, 0]), ("REJECTED", [110 * MS, 0])]
    stamped = [dict(r) for r in base]
    stamped[4]["stamp"] = 20 * MS
    got = [(r["outcome"], r["tag"]) for r in oracle.run(stamped) if r["ev"] == "decision"]
    assert got == [("MATCHED", [20 * MS, 1]), ("REJECTED", [110 * MS, 0])]
    # A stamp bound at anchor + delta (watermark not past it) decides nothing yet.
    stamped[4]["stamp"] = 10 * MS
    got = [(r["outcome"], r["tag"]) for r in oracle.run(stamped) if r["ev"] == "decision"]
    assert got == plain
    assert any(oracle.run(corpus.random_corpus(seed, stamped_missing=True))
               != oracle.run([{k: v for k, v in r.items() if not (r["ev"] == "missing"
                                                                  and k == "stamp")}
                              for r in corpus.random_corpus(seed, stamped_missing=True)])
               for seed in range(50))

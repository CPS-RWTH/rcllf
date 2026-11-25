# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Converted message_filters synchronizers against oracle O1 (S5).

Each corpus of ``sync_oracle_showcase`` runs through the generated system with
five synchronizer instances (ExactTime q 1/2/3, ApproximateTime q 2/3). The
expected stream of each instance is O1: the stock header run off-line on the
lf_arrival_order (``test/sync_oracle.cpp``), whose own output the corpus
``expect`` block pins by hand. ``lf_eval.sync_oracle`` compares config,
domain, arrival order, tuples and row classes, in that order.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools" / "lf_eval"))
from lf_eval.sync_oracle.compare import (  # noqa: E402
    check_domain, compare, compare_order, parse_jsonl, short)
from lf_eval.sync_oracle.corpus import load_corpus  # noqa: E402
from lf_eval.sync_oracle.order import parse_run  # noqa: E402

# (node, policy, queue) of launch/corpus.launch.py.
INSTANCES = (("exact_q1", "exact", 1), ("exact_q2", "exact", 2), ("exact_q3", "exact", 3),
             ("approx_q2", "approx", 2), ("approx_q3", "approx", 3))
IN_DOMAIN = ("p1", "p2", "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8",
             "a1", "a2", "a3", "a4", "f1")
_SYNC = re.compile(r"RCLLF_SYNC (\{.*\})\s*$")


def _share() -> Path:
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix("sync_oracle_showcase"))


def _corpus_path(name: str) -> Path:
    return _share() / "share" / "sync_oracle_showcase" / "corpora" / f"{name}.yaml"


def _o1(name: str, policy: str, queue: int) -> list[dict]:
    exe = _share() / "lib" / "sync_oracle_showcase" / "sync_oracle"
    out = subprocess.run([str(exe), "--corpus", str(_corpus_path(name)), "--policy", policy,
                          "--queue", str(queue)], capture_output=True, text=True, check=True)
    return parse_jsonl(out.stdout)


def _run(monkeypatch, case_key: str, corpus: str, **env) -> str:
    case = rts.require_case(case_key)
    monkeypatch.setenv("SYNC_CORPUS", str(_corpus_path(corpus)))
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    # The player logs a done record after its last tick (50 ms each, at most 15
    # ticks); the settle lets the synchronizers' records of that tag flush.
    result = rts.launch_and_capture(
        case, 2.0, until=lambda log: '"ev":"done"' in log, settle_s=1.0)
    return result.log


def _streams(log: str) -> dict[str, list[str]]:
    """The RCLLF_SYNC records per node, as logged (the byte-identity unit)."""
    out: dict[str, list[str]] = {}
    for line in log.splitlines():
        m = _SYNC.search(line)
        if m:
            node = re.search(r'"node":"([^"]+)"', m.group(1))
            out.setdefault(node.group(1) if node else "", []).append(m.group(1))
    return out


@pytest.mark.parametrize("corpus", IN_DOMAIN)
def test_every_instance_matches_o1(monkeypatch, corpus):
    path = _corpus_path(corpus)
    meta = load_corpus(path)
    run = parse_run(_run(monkeypatch, "sync_corpus", corpus))
    reports = {}
    for node, policy, queue in INSTANCES:
        o1 = _o1(corpus, policy, queue)
        if policy == meta["policy"] and queue == meta["queue"]:
            # O1 itself reproduces the hand derivation of the corpus.
            assert [short(e) for e in o1 if e.get("ev") in ("signal", "drop")] == \
                meta["expect"], f"{corpus}: O1 {node} differs from the expect block"
        reports[node] = compare(meta, o1, run, node)
    assert reports == {n: None for n, _, _ in INSTANCES}


def test_lost_sample_is_reported_out_of_domain(monkeypatch):
    """Drop P1's second input-0 sample on exact_q1's edge: hand-derived row 2
    (rows alternate input 0, input 1 per tick, so input 0's ordinal 2 is r2)."""
    rts.require_case("sync_corpus")
    probe = parse_run(_run(monkeypatch, "sync_corpus", "p1"))
    [edge] = [k for k, e in probe.edges.items()
              if e["sub"].endswith("exact_q1") and e["topic"].endswith("in0")]
    run = parse_run(_run(monkeypatch, "sync_corpus", "p1",
                         RCLLF_TEST_CORR_DROP_SAMPLE=f"{edge}:2"))
    meta = load_corpus(_corpus_path("p1"))
    assert compare(meta, _o1("p1", "exact", 1), run, "exact_q1") == \
        f"out_of_domain: LOST on edge {edge} ordinal 2 (row 2)"
    assert compare(meta, _o1("p1", "exact", 3), run, "exact_q3") is None


def test_n1_burst_is_out_of_domain(monkeypatch):
    run = parse_run(_run(monkeypatch, "sync_corpus", "n1"))
    meta = load_corpus(_corpus_path("n1"))
    player = run.sync.get("corpus_player", [])
    for node, _, _ in INSTANCES:
        report = check_domain(run, node, meta, player) or compare_order(run, node, meta)
        assert report and report.startswith("out_of_domain"), (node, report)


def _dispatches(log: str) -> list[tuple]:
    """(edge, ordinal, outcome, elapsed, microstep) of every correspondence
    dispatch, sorted: the threads log in physical order, the tags are logical."""
    out = []
    for line in log.splitlines():
        if '"kind":"dispatch"' not in line:
            continue
        rec = json.loads(line[line.index("{"):])
        out.append((rec["edge"], rec["ordinal"], rec["outcome"], rec["elapsed"], rec["microstep"]))
    return sorted(out)


def _join(log: str) -> list[str]:
    return [short(r) for r in parse_run(log).sync.get("join", [])
            if r.get("ev") in ("signal", "drop")]


def test_forkjoin_join_does_not_depend_on_branch_work(monkeypatch):
    """P3: the join pairs the same rows whichever branch does the work."""
    meta = load_corpus(_corpus_path("p3"))
    plain = _run(monkeypatch, "sync_forkjoin", "p3")
    swapped = _run(monkeypatch, "sync_forkjoin", "p3", SYNC_WORK_SWAP="1")
    # forkjoin.launch.py: branch_0 works 0 ms and branch_1 8 ms per row, swapped 8 and 0.
    work = [sorted(set(re.findall(r'"node":"(branch_\d)","ev":"branch","work_ms":(\d+)', log)))
            for log in (plain, swapped)]
    assert work == [[("branch_0", "0"), ("branch_1", "8")],
                    [("branch_0", "8"), ("branch_1", "0")]], work
    assert _join(plain) == meta["expect"]
    assert _join(swapped) == meta["expect"]


@pytest.mark.parametrize("case_key,corpus", [("sync_corpus", "f1"), ("sync_corpus", "b8"),
                                             ("sync_forkjoin", "p3")])
def test_streams_are_identical_across_runs(monkeypatch, case_key, corpus):
    """Output verification 7: RCLLF_SYNC_RUNS runs (default 2, 20 for the
    evidence run) give byte-identical synchronizer records and the same
    (elapsed tag, microstep) for every dispatch."""
    runs = int(os.environ.get("RCLLF_SYNC_RUNS", "2"))
    log = _run(monkeypatch, case_key, corpus)
    first, first_tags = _streams(log), _dispatches(log)
    assert first, "no RCLLF_SYNC records"
    assert first_tags, "no correspondence dispatch records"
    for k in range(1, runs):
        log = _run(monkeypatch, case_key, corpus)
        assert _streams(log) == first, f"run {k}: synchronizer records differ"
        assert _dispatches(log) == first_tags, f"run {k}: dispatch tags differ"

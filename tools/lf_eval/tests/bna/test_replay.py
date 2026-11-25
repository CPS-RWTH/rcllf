# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""C++ BNA engine (bna_replay) against the oracle, and the falsification mutants.

Needs the ``bna_replay`` binary of ``rcllf_message_filters``. It is looked up
in ``RCLLF_BNA_REPLAY``, then in ``packages/install`` and ``packages/build``.
Without it the module skips (run it in the dev container).
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from lf_eval.bna import compare, corpus, oracle

ROOT = Path(__file__).resolve().parents[4]
CORPORA = Path(__file__).parent / "corpora"
CASES = sorted(p.stem for p in CORPORA.glob("*.bna"))


def _find_replay():
    env = os.environ.get("RCLLF_BNA_REPLAY")
    cands = [Path(env)] if env else []
    cands += [ROOT / "packages/install/rcllf_message_filters/lib/rcllf_message_filters/bna_replay",
              ROOT / "packages/build/rcllf_message_filters/bna_replay"]
    return next((p for p in cands if p.is_file() and os.access(p, os.X_OK)), None)


REPLAY = _find_replay()
pytestmark = pytest.mark.skipif(REPLAY is None, reason="bna_replay not built")


def replay(events: list[dict], tmp_path: Path, mutant: str = "none") -> list[dict]:
    src = tmp_path / "corpus.jsonl"
    src.write_text(corpus.dumps(events), encoding="utf-8")
    proc = subprocess.run([str(REPLAY), "--corpus", str(src), "--mutant", mutant],
                          capture_output=True, text=True, timeout=60, check=False)
    assert proc.returncode == 0, proc.stderr
    out = tmp_path / "out.jsonl"
    out.write_text(proc.stdout, encoding="utf-8")
    return corpus.read_jsonl(out)


def first_report(events: list[dict], tmp_path: Path, mutant: str = "none"):
    got = replay(events, tmp_path, mutant)
    return compare.compare(events, oracle.run(events), got, got)


@pytest.mark.parametrize("case", CASES)
def test_replay_agrees_on_corpora(case, tmp_path):
    events, _ = corpus.load_bna(CORPORA / f"{case}.bna")
    assert first_report(events, tmp_path) is None


@pytest.mark.parametrize("seed", range(200))
def test_replay_agrees_on_random_corpora(seed, tmp_path):
    assert first_report(corpus.random_corpus(seed), tmp_path) is None


@pytest.mark.parametrize("seed", range(200))
def test_replay_agrees_on_random_stamped_corpora(seed, tmp_path):
    # Missing reports with a stamp bound raise the watermark.
    assert first_report(corpus.random_corpus(seed, stamped_missing=True), tmp_path) is None


def test_replay_agrees_on_permuted_corpora(tmp_path):
    for seed in range(5):
        for perm in range(3):
            assert first_report(corpus.permuted_corpus(seed, perm), tmp_path) is None


def test_replay_refuses_backward_tags(tmp_path):
    events, _ = corpus.parse_bna("params epoch=1 delta=1 lateness=0,0 capacity=1,1 "
                                 "retention=5 timeout=5\nadmit 0 1 1:a:1 @2\nwake @1\n")
    src = tmp_path / "c.jsonl"
    src.write_text(corpus.dumps(events), encoding="utf-8")
    proc = subprocess.run([str(REPLAY), "--corpus", str(src)], capture_output=True,
                          text=True, timeout=60, check=False)
    assert proc.returncode == 1
    assert "line 3: BNA event tag goes back in time" in proc.stderr


# -- mutants: each must be caught on its target corpus with this exact report.
# Measured with bna_replay in the dev container.
MUTANTS = {
    "tie_later_stamp": ("T1",
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle MATCHED 1:b:1@95000000, "
        "implementation MATCHED 1:b:2@105000000"),
    "watermark_inclusive": ("W1",
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle REJECTED(timeout), "
        "implementation MATCHED 1:b:1@105000000"),
    "late_exclusive": ("W2",
        "outcome_mismatch at sample 1:b:2: oracle LATE, implementation FLUSHED"),
    "reuse": ("T1",
        "outcome_count_mismatch at sample 1:b:1: 1 admission(s), "
        "implementation 2 outcome(s) (SELECTED, SUPERSEDED)"),
    "timeout_lazy": ("TO1",
        "decision_mismatch at anchor 1:a:1 (stamp 100000000): oracle tag [148000000,0], "
        "implementation tag [201000000,0]"),
    "overflow_newest": ("CAP1",
        "outcome_mismatch at sample 1:b:1: oracle OVERFLOW, implementation FLUSHED"),
    "skew_exclusive": ("N1",
        "decision_mismatch at anchor 1:a:3 (stamp 60000000): oracle MATCHED 1:b:2@50000000, "
        "implementation REJECTED(missing)"),
    "expiry_on_admit": ("RET2",
        "outcome_mismatch at sample 1:b:1: oracle EXPIRED, implementation FLUSHED"),
    "late_anchor_double": ("LA1",
        "outcome_count_mismatch at sample 1:a:2: 1 admission(s), "
        "implementation 2 outcome(s) (REJECTED(late), LATE)"),
    "no_order_check": ("D1",
        "outcome_mismatch at sample 1:b:5: oracle FAULT(duplicate), implementation FLUSHED"),
}


@pytest.mark.parametrize("mutant", sorted(MUTANTS))
def test_mutant_is_caught_with_exact_message(mutant, tmp_path):
    case, message = MUTANTS[mutant]
    events, _ = corpus.load_bna(CORPORA / f"{case}.bna")
    assert first_report(events, tmp_path, mutant) == message

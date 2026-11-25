# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""tf2 acquisition-time lookups on the generated tf2_acq_example.

Expected values are the showcase's hand table corpus/t1_t7_expected.tsv,
derived from the source before the first converted run (README "Expected
values"): /tf every 20 ms with x = t at 1 m/s, requests on the 30 ms sampler
grid, wait bound 55 ms, on_timeout reject. The gtest O1 (test_o1_oracle)
replays the same schedule through a plain tf2::BufferCore and equals the
table; this test checks the converted run against it.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tools" / "lf_eval"))
from lf_eval import tf2_acq  # noqa: E402

HAND = tf2_acq.read_hand_table(
    REPO / "packages" / "src" / "tf2_acq_example" / "corpus" / "t1_t7_expected.tsv")
# Request tag and stamp per case, so a run that asked for the wrong stamp fails
# even where the transform would match (T6, a static frame).
REQUESTS = tf2_acq.read_requests(
    REPO / "packages" / "src" / "tf2_acq_example" / "corpus" / "t1_t7_events.tsv")
RUNS = 20  # plan constant: the claim-matrix convention


def _run(monkeypatch, **env):
    case = rts.require_case("tf2_acq")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    # The last outcome (T4) is decided at 385 ms logical time.
    result = rts.launch_and_capture(
        case, 2.0, until=lambda log: log.count('"event":"tf_outcome"') >= len(HAND),
        settle_s=0.3)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-3000:]
    assert "LF takes /tf and /tf_static" in result.log, "listener group not detached"
    origin, ids = tf2_acq.showcase_ids(result.log)
    return tf2_acq.records(result.log), ids, origin


def test_tf2_acq_t1_to_t7_equal_the_hand_table_in_20_identical_runs(monkeypatch):
    streams = []
    for i in range(RUNS):
        recs, ids, origin = _run(monkeypatch)
        diff = tf2_acq.compare(recs, ids, origin, HAND, REQUESTS)
        assert diff is None, f"run {i + 1}: {diff}"
        streams.append("\n".join(tf2_acq.outcome_stream(recs, origin)))
    assert len(set(streams)) == 1, "outcome streams differ between runs:\n" + "\n--\n".join(
        sorted(set(streams)))
    out = REPO / "output" / "experiments" / "tf2_acq"
    out.mkdir(parents=True, exist_ok=True)
    (out / "outcome_stream.tsv").write_text(streams[0] + "\n")


def test_tf2_acq_latest_on_timeout_mutant_is_caught(monkeypatch):
    """Plan output verification 3: the mutant answers T5 with the latest
    held_link transform, which the comparator names at the T5 row."""
    recs, ids, origin = _run(monkeypatch, RCLLF_TF2_TEST_MUTANT="latest_on_timeout")
    assert tf2_acq.compare(recs, ids, origin, HAND, REQUESTS) == "T5: outcome ok != reject"


def test_tf2_acq_runs_on_the_pinned_tf2_ros():
    rts.require_case("tf2_acq")
    found = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", "ros-jazzy-tf2-ros"],
        capture_output=True, text=True, check=False).stdout.strip()
    assert tf2_acq.check_pin(found) is None, found

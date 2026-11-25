# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A flushed cross-federate message is a COMPLETE record.

CI: runtime-smoke group corr-relay-burst (hooks on), ``-k flush_complete``. The
names avoid ``xfed`` so the same-tag group's ``-k xfed`` does not select them.

``pubsub_example`` in one federate per node under decentralized coordination
with ``after 5ms`` (``pubsub_flush_federation.yaml``). Decentralized d > 0
sends through the flush: the records of time t leave at t + 1 ns over
``after d - 1 ns`` and land on (t + d, 0). The publisher's 500 ms timer
publishes one sample per tick, so the k-th tick's flush is completion number
k and the subscriber federate sees numbers 1, 2, 3, ... at tick k's time
+ 5 ms.

The flush message carries a COMPLETE flag and a number, so a dropped flush
message is reported as CONTROL_FAILURE. The drop case withholds the
2nd flushed message with ``RCLLF_TEST_XCORR_DROP_COMPLETE`` (binding built
with ``-DRCLLF_CORR_TEST_HOOKS=ON``) and expects CONTROL_FAILURE for 2.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

GEN_PKG = "lf_generated_pubsub_example_bringup_federated"
BUILD = ("FEDERATE=../tools/colcon_lf/tests/runtime/pubsub_flush_federation.yaml "
         "bash ../docker/lf_build.sh pubsub_example")
D_NS = 5_000_000
PERIOD_NS = 500_000_000
EDGE_RE = re.compile(
    r"\[(\w+?)(?:-\d+)?\].*rcllf corr: admitted (ingress |egress )edge (\d+) "
    r"(?:\(global \d+\) )?(\S+) -> (\S+) on '([^']+)'")
CORR_RE = re.compile(r'\[(\w+?)(?:-\d+)?\].*?(\{"event":"corr".*\})')
FAILURE_RE = re.compile(
    r"CONTROL_FAILURE edge (\d+): sender activation (\d+) has no COMPLETE record")


def _case() -> rts.RuntimeCase:
    install = rts._PACKAGES_ROOT / "install"
    launch = sorted((install / GEN_PKG / "share" / GEN_PKG / "launch").glob("*.launch.py"))
    flush = list((rts._PACKAGES_ROOT / "lf-generated" / GEN_PKG / "lf").rglob("*.lf"))
    if not launch or not any("xcorr_flush" in p.read_text(errors="replace") for p in flush):
        msg = f"{GEN_PKG} not built with the flush fixture ({BUILD})"
        if os.environ.get("RCLLF_REQUIRE_RUNTIME_SMOKE") == "1":
            pytest.fail(msg)
        pytest.skip(msg)
    return rts.RuntimeCase(source_pkg="pubsub_example", launch_stem="bringup", gen_pkg=GEN_PKG,
                           launch_file=launch[0].name)


def _events(log: str, kind: str) -> list[dict]:
    out = []
    for _, raw in CORR_RE.findall(log):
        ev = json.loads(raw)
        if ev.get("kind") == kind:
            out.append(ev)
    return out


def _run(monkeypatch, label: str, drop: str, until) -> str:
    monkeypatch.setenv("RCLLF_TEST_XCORR_DROP_COMPLETE", drop)
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    result = rts.launch_and_capture(_case(), 3.0, until=until, until_timeout_s=30.0)
    out = os.environ.get("RCLLF_FLUSH_COMPLETE_LOG_DIR")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        (Path(out) / f"{label}.log").write_text(result.log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-3000:]
    return result.log


def _completes(log: str) -> list[dict]:
    return [ev for ev in _events(log, "complete") if not ev["duplicate"]]


def test_flush_complete_one_per_time(monkeypatch):
    log = _run(monkeypatch, "flush_healthy", "",
               lambda lg: len(_completes(lg)) >= 4)
    edges = EDGE_RE.findall(log)
    assert any(role == "egress " for _, role, *_ in edges), \
        "the sensor_data flush edge was not admitted:\n" + log[-3000:]
    emits = _events(log, "emit_remote")
    assert emits, "no flushed message was sent"
    # The flush of time t runs at (t + 1 ns, 0) and sends the records of t.
    for ev in emits:
        assert (ev["elapsed"] % PERIOD_NS, ev["microstep"]) == (1, 0), ev
    completes = _completes(log)
    assert len(completes) >= 4, (
        f"the subscriber federate saw {len(completes)} COMPLETE records from the flush "
        "(want one per 500 ms tick)")
    seqs = [ev["seq"] for ev in completes]
    assert seqs == list(range(1, len(seqs) + 1)), f"completion numbers {seqs}"
    # Tick k (k = 0, 1, ...) publishes at k * 500 ms; its flush lands on
    # (k * 500 ms + 5 ms, 0) and carries completion number k + 1.
    got = [(ev["seq"], ev["elapsed"], ev["microstep"]) for ev in completes]
    want = [(k + 1, k * PERIOD_NS + D_NS, 0) for k in range(len(completes))]
    assert got == want, f"COMPLETE (seq, elapsed, microstep)\n got {got}\nwant {want}"
    assert not FAILURE_RE.findall(log), "a healthy flush run reported CONTROL_FAILURE"
    assert not _events(log, "control_failure")


def test_flush_complete_dropped_message_is_a_control_failure(monkeypatch):
    log = _run(monkeypatch, "flush_drop2", "0:2",
               lambda lg: any(ev["seq"] >= 4 for ev in _completes(lg)))
    failures = [int(seq) for _, seq in FAILURE_RE.findall(log)]
    assert failures == [2], (
        f"a flushed message was dropped and no CONTROL_FAILURE was reported (got {failures})\n"
        + log[-3000:])
    assert "TEST FAULT HOOK: COMPLETE 2 of egress edge 0 not sent" in log, \
        "the drop hook did not withhold flushed message 2:\n" + log[-3000:]
    traced = [ev["seq"] for ev in _events(log, "control_failure")]
    assert traced == [2], f"control_failure trace events {traced}"
    seqs = [ev["seq"] for ev in _completes(log)]
    assert seqs[:3] == [1, 3, 4], f"completion numbers {seqs}"

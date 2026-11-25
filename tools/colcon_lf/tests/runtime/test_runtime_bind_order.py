# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Bind before dispatch at tag 0 under stress.

``callback_group_example`` generates five group sub-reactors whose timers all
fire at tag 0. ``RCLLF_TEST_BIND_DELAY_MS=50`` holds the node's bind startup
reaction for 50 ms, so without the generated ``rcllf_bound`` precedence edge
the group timers dispatch while their RIDs are still unbound and the binding
prints ``ros2_lf_invoke_timer: RID n unbound at (0, 0)``. With the edge, every
run has zero ``unbound`` events and every ``callback_start`` follows the
``bind`` trace line. Every run also shuts down cleanly on SIGINT.

A failing run's full launch log is kept under
``output/experiments/bind_order_failures/`` (or ``RCLLF_BIND_ORDER_LOG_DIR``)
and named in the failure message, so a rare failure can be read
after the fact.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

RUNS = int(os.environ.get("RCLLF_BIND_ORDER_RUNS", "20"))
LOG_DIR = Path(
    os.environ.get(
        "RCLLF_BIND_ORDER_LOG_DIR",
        Path(rts.__file__).resolve().parents[4] / "output" / "experiments" / "bind_order_failures",
    )
)


def _keep_log(log: str, run: int) -> Path:
    """Write a failing run's launch log where it survives the test session."""
    path = LOG_DIR / time.strftime("%Y%m%d-%H%M%S") / f"run-{run}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(log, encoding="utf-8")
    return path


def _check_run(log: str) -> list[str]:
    problems = []
    unbound = [ln for ln in log.splitlines() if "unbound at (" in ln or '"event":"unbound"' in ln]
    if unbound:
        problems.append(f"{len(unbound)} unbound dispatch line(s), first: {unbound[0].strip()}")
    bind = log.find('"event":"bind"')
    if bind < 0:
        problems.append("no bind trace line")
        return problems
    first_start = log.find('"event":"callback_start"')
    if first_start < 0:
        problems.append("no callback_start trace line")
    elif first_start < bind:
        problems.append("a callback_start precedes the bind trace line")
    return problems


def test_bind_precedes_every_tag0_dispatch(monkeypatch):
    case = rts.require_case("bind_order")
    monkeypatch.setenv("RCLLF_TEST_BIND_DELAY_MS", "50")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    failures = []
    for run in range(RUNS):
        result = rts.launch_and_capture(case, 1.5)
        problems = _check_run(result.log)
        # The SIGINT at 1.5 s lands inside the 5 s discovery wait
        # (/chatter has no publisher), so ingress registers on a shut-down
        # context. That used to abort with "failed to create guard condition".
        if result.process_died or not result.finished_cleanly:
            problems.append("no 'process has finished cleanly' after SIGINT")
        if problems:
            kept = _keep_log(result.log, run + 1)
            failures.append(f"run {run + 1}: " + "; ".join(problems) + f" (log: {kept})")
    assert not failures, f"{len(failures)}/{RUNS} runs broke bind-before-dispatch:\n" + "\n".join(failures)

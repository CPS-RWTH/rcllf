# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The RTI guard reports a federate lost without a resign.

A lost federate's later control records never reach any receiver. The RTI
marks it NOT_CONNECTED (rti_remote.c) but sends no new grant, so in the
killed.log run the receiver federate stalled at logical 660 ms until its wall
timeout and ended on SIGINT with "process has finished cleanly". Nothing in
the records or payloads names the loss. The guard reads the RTI's own lines
instead and fails the run at the moment the RTI reports the closed link.

Fixtures are RTI logs of real runs (tests/fixtures/rti_guard):
- resigned.log: an idle two-federate run on an Orin NX board at an earlier revision,
  both federates resigned.
- killed.log: a workstation container run at an earlier revision in which the
  federate__host_a process was SIGKILLed 2 s after start (KILL_S=2).
The stand-in RTI prints a fixture and exits 0, as the real RTI did in both.
"""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from colcon_lf.federation import rti_guard

GUARD = Path(__file__).resolve().parents[2] / "colcon_lf/federation/rti_guard.py"
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures/rti_guard"
# Seconds for the stand-in to print a 6-line log and exit; a cap, not a wait.
WAIT_CAP_S = 60


def _guard_over(log: Path) -> subprocess.CompletedProcess:
    # The stand-in writes the log without a final flush call: the guard's
    # relay must pass it through whole, partial last line included.
    rti = f"import sys; sys.stdout.write(open({str(log)!r}).read())"
    return subprocess.run([sys.executable, str(GUARD), sys.executable, "-c", rti],
                          capture_output=True, text=True, timeout=WAIT_CAP_S)


def test_a_run_whose_federates_all_resigned_passes():
    log = FIXTURES / "resigned.log"
    res = _guard_over(log)
    assert res.returncode == 0, res.stderr
    assert res.stdout == log.read_text()
    assert "rti_guard: federate" not in res.stderr


def test_a_federate_killed_mid_run_fails_the_run():
    log = FIXTURES / "killed.log"
    res = _guard_over(log)
    assert res.returncode == rti_guard.LOST_FEDERATE_EXIT, \
        f"guard passed a run with a killed federate (rc {res.returncode}): {res.stderr}"
    assert res.stdout == log.read_text()
    assert "closed its RTI connection without resigning" in res.stderr
    assert "not counted by any receiver" in res.stderr and "#598" not in res.stderr


@pytest.mark.parametrize("lines, lost", [
    # A resign ends the RTI's read loop (rti_remote.c handle_federate_resign
    # sets NOT_CONNECTED), so a close after it is not a loss.
    (["RTI: Federate 0 has resigned.", "RTI: Connection to federate 0 is closed. Exiting the thread."],
     {}),
    (["RTI: Connection to federate 1 is closed. Exiting the thread."],
     {1: "closed its RTI connection without resigning"}),
    (["ERROR: RTI: Federate 0 reports an error and has exited.",
      "RTI: Connection to federate 0 is closed. Exiting the thread."],
     {0: "reported an error and exited"}),
])
def test_lost_federates_from_rti_lines(lines, lost):
    watch = rti_guard.LostFederates()
    for line in lines:
        watch.feed(line)
    assert watch.lost == lost


def test_the_guard_keeps_a_failing_rti_code():
    # The RTI's own nonzero code (1: a federate reported an error) wins over
    # the lost-federate code, so the launch log keeps the RTI's cause.
    rti = ("import sys; print('RTI: Connection to federate 1 is closed. Exiting the thread.'); "
           "sys.exit(1)")
    res = subprocess.run([sys.executable, str(GUARD), sys.executable, "-c", rti],
                         capture_output=True, text=True, timeout=WAIT_CAP_S)
    assert res.returncode == 1
    assert "federate 1 closed its RTI connection without resigning" in res.stderr


def test_a_stalled_stdout_never_lets_a_lost_federate_pass():
    # The relay used to write a chunk before parsing it,
    # and the guard decided after a bounded join. With nobody reading the
    # guard's stdout, the closed-link line behind 80 KB of output (more than
    # one 64 KiB pipe buffer) was unparsed when the guard decided. Falsified on
    # an earlier revision: the guard then aborted at interpreter shutdown (rc -6, the
    # relay thread held the stdout lock), never the lost-federate exit 4.
    filler = "RTI: filler line of the stalled-stdout test.\n" * 1800
    assert len(filler) > 65536
    rti = (f"import sys; sys.stdout.write({filler!r}); "
           "print('RTI: Connection to federate 0 is closed. Exiting the thread.')")
    r, w = os.pipe()

    def drain_late():
        # Start reading only after the guard's drain cap, so the guard decides
        # while its stdout is stalled, then let it exit.
        time.sleep(rti_guard.RELAY_DRAIN_S + 3)
        while os.read(r, 65536):
            pass

    drain = threading.Thread(target=drain_late, daemon=True)
    try:
        guard = subprocess.Popen([sys.executable, str(GUARD), sys.executable, "-c", rti],
                                 stdout=w, stderr=subprocess.PIPE, text=True)
        os.close(w)
        drain.start()
        _, err = guard.communicate(timeout=WAIT_CAP_S)
        drain.join(timeout=WAIT_CAP_S)
    finally:
        os.close(r)
    assert guard.returncode == rti_guard.LOST_FEDERATE_EXIT, \
        f"guard passed with a stalled stdout (rc {guard.returncode}): {err}"


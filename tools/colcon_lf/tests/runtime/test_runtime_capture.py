# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The capture harness itself: polling the log must never rewrite it.

``launch_and_capture`` hands the child its log file and polls that same file
while the child writes. Both share one open file description, so one file
offset: a poll that seeks to 0 moves the child's write position too, and a
line written in that window lands at the start of the file, over the launch
banner. On CI this put a tick-5 ``corr_sub`` line first in the stale
scenario's log. These tests need no ROS: ``Popen`` runs a line writer instead
of ``ros2 launch``.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

# Writes numbered lines as fast as it can, one write(2) each, for 1.5 s, so
# about seven 0.2 s polls overlap it, then exits on its own.
_WRITER = (
    "import sys, time\n"
    "end, i = time.monotonic() + 1.5, 0\n"
    "while time.monotonic() < end:\n"
    "    sys.stdout.write(f'line {i}\\n')\n"
    "    sys.stdout.flush()\n"
    "    i += 1\n"
    "sys.stdout.write(f'wrote {i} lines, process has finished cleanly\\n')\n"
)


def test_polling_never_rewrites_the_captured_log(monkeypatch):
    real_popen = subprocess.Popen

    def writer_popen(_argv, **kwargs):
        kwargs.pop("cwd", None)
        return real_popen([sys.executable, "-c", _WRITER], **kwargs)

    monkeypatch.setattr(rts.subprocess, "Popen", writer_popen)
    case = rts.RuntimeCase("none", "none", "none", "none")
    polls = []

    def until(log: str) -> bool:
        polls.append(len(log))
        return False  # poll until the writer exits, as a slow run does

    for _ in range(3):
        result = rts.launch_and_capture(
            case, 0.0, until=until, until_timeout_s=20.0, settle_s=0.0, allow_exit=True)
        lines = result.log.splitlines()
        assert result.finished_cleanly, f"writer summary missing, last line {lines[-1:]}"
        n = int(lines[-1].split()[1])
        want = [f"line {i}" for i in range(n)] + [lines[-1]]
        first_bad = next((i for i, (a, b) in enumerate(zip(lines, want)) if a != b), None)
        assert first_bad is None and len(lines) == len(want), (
            f"capture rewritten while polling: line {first_bad} is "
            f"{lines[first_bad] if first_bad is not None else None!r}, "
            f"{len(lines)} of {len(want)} lines kept")
    assert len(polls) >= 9, f"too few polls overlapped the writer: {len(polls)}"

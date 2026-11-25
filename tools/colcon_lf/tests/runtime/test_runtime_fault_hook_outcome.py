# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A run with compiled-out fault hooks is skipped only when it is healthy.

``check_fault_hooks`` skips a run that set a correspondence fault hook on a
binding built without ``-DRCLLF_CORR_TEST_HOOKS=ON``. Such a run injected no
fault, so a crash, a died process or a hang in it is the binary's own failure
and must fail the test, not hide behind the skip.

No ROS install is needed: a fake ``ros2`` on ``PATH`` prints what
``ros2 launch`` prints (the binding's compiled-out line, launch's
``process has died`` / ``process has finished cleanly`` lines) and exits the
way the scenario asks. The binding line is the exact text of
``announce_test_hooks`` in ``corr_runtime.cpp``.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

import pytest

from . import runtime_support as rts
from . import semantic_session as ss

_HOOK_LINE = ("[ERROR] [1762900000.000000000] [rcllf.corr]: rcllf corr: RCLLF_TEST_CORR_DROP_SAMPLE "
              "is set but the test hooks are compiled out (build ros2_lf_binding with "
              "-DRCLLF_CORR_TEST_HOOKS=ON): no fault is injected")
_DIED = "[ERROR] [corr_node-1]: process has died [pid 4242, exit code -11, cmd 'corr_node']."
_CLEAN = "[INFO] [corr_node-1]: process has finished cleanly [pid 4242]"

# Each scenario: what the fake launcher prints, then how it ends.
_FAKE_ROS2 = f"""#!/bin/bash
echo "{_HOOK_LINE}"
case "$FAKE_ROS2_MODE" in
  crash_exit)   echo "{_DIED}"; exit 1 ;;
  clean_exit)   echo "{_CLEAN}"; exit 0 ;;
  crash_sigint) echo "{_DIED}"; trap 'exit 0' INT ;;
  silent_sigint) trap 'exit 0' INT ;;
  clean_sigint) trap 'echo "{_CLEAN}"; exit 0' INT ;;
  hang)         trap '' INT ;;
esac
for _ in $(seq 600); do sleep 0.05; done
exit 0
"""

_CASE = rts.RuntimeCase(source_pkg="fake_src", launch_stem="bringup", gen_pkg="fake_gen",
                        launch_file="fake.launch.py")


@pytest.fixture
def fake_ros2(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    script = bindir / "ros2"
    script.write_text(_FAKE_ROS2)
    script.chmod(script.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.delenv(rts.REQUIRE_ENV, raising=False)  # the skip branch is under test

    def use(mode: str) -> None:
        monkeypatch.setenv("FAKE_ROS2_MODE", mode)

    return use


def _outcome(call) -> tuple[str, str]:
    try:
        call()
    except pytest.skip.Exception as exc:
        return "skipped", str(exc)
    except pytest.fail.Exception as exc:
        return "failed", str(exc)
    return "returned", ""


def _capture(**kw):
    return lambda: rts.launch_and_capture(_CASE, 0.3, **kw)


@pytest.mark.parametrize("mode, kw, want", [
    ("crash_exit", {"allow_exit": True}, "process has died"),
    ("crash_sigint", {}, "process has died"),
    ("silent_sigint", {}, "no 'process has finished cleanly'"),
    ("hang", {"stop_timeout_s": 0.5}, "had to be SIGKILLed"),
])
def test_launch_and_capture_fails_a_broken_run_with_hooks_compiled_out(fake_ros2, mode, kw, want):
    fake_ros2(mode)
    got, msg = _outcome(_capture(**kw))
    assert got == "failed", (
        f"a {mode} run with the hooks compiled out was {got}, want failed: {msg}")
    assert want in msg and "compiled out" in msg, msg


@pytest.mark.parametrize("mode, kw", [("clean_exit", {"allow_exit": True}), ("clean_sigint", {})])
def test_launch_and_capture_skips_a_healthy_run_with_hooks_compiled_out(fake_ros2, mode, kw):
    fake_ros2(mode)
    got, msg = _outcome(_capture(**kw))
    assert got == "skipped", f"a healthy {mode} run with the hooks compiled out was {got}: {msg}"
    assert "-DRCLLF_CORR_TEST_HOOKS=ON" in msg, msg


class _NoLttng:
    """Stands in for the private session daemon: every lttng call succeeds."""

    def __init__(self) -> None:
        self.env = dict(os.environ)

    def lttng(self, *args: str, check: bool = True) -> subprocess.CompletedProcess:
        return subprocess.CompletedProcess(["lttng", *args], 0, stdout="", stderr="")


def _traced(tmp_path: Path):
    return lambda: ss.traced_launch(_NoLttng(), _CASE, tmp_path / "run", secs=0.3,
                                    write_manifest=False, copy_static=False)


@pytest.mark.parametrize("mode, want", [
    ("crash_exit", "process has died"),
    ("crash_sigint", "process has died"),
    ("silent_sigint", "no 'process has finished cleanly'"),
])
def test_traced_launch_fails_a_broken_run_with_hooks_compiled_out(fake_ros2, tmp_path, mode, want):
    fake_ros2(mode)
    got, msg = _outcome(_traced(tmp_path))
    assert got == "failed", (
        f"a {mode} traced run with the hooks compiled out was {got}, want failed: {msg}")
    assert want in msg and "compiled out" in msg, msg


def test_traced_launch_skips_a_healthy_run_with_hooks_compiled_out(fake_ros2, tmp_path):
    fake_ros2("clean_sigint")
    got, msg = _outcome(_traced(tmp_path))
    assert got == "skipped", f"a healthy traced run with the hooks compiled out was {got}: {msg}"


def test_run_failure_reads_the_launcher_rc_only_when_given():
    assert rts.run_failure(_CLEAN, returncode=1) == "the launcher exited with rc=1"
    assert rts.run_failure(_CLEAN, returncode=None) is None
    assert rts.run_failure("", need_clean=True) == "no 'process has finished cleanly' in the log"
    assert rts.run_failure(_CLEAN, sigkilled=True) == "the process had to be SIGKILLed"

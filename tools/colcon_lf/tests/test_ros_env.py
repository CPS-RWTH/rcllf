# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The colcon lf-* verb probe skips only when the verb is provably absent."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from . import ros_env

INVALID_CHOICE = (
    "usage: colcon [-h] [--log-base LOG_BASE] {build,info,list,test} ...\n"
    "colcon: error: argument verb_name: invalid choice: 'lf-analyze' "
    "(choose from 'build', 'info', 'list', 'test')"
)
TRACEBACK = (
    "Traceback (most recent call last):\n"
    '  File "colcon_lf/verb/analyze.py", line 1, in <module>\n'
    "ImportError: cannot import name 'broken' from 'colcon_lf'"
)

# Real stderr of an installed colcon_lf whose lf-analyze verb fails to import: colcon
# logs the load failure, then also reports the dropped verb as an invalid choice.
# Capture command: tests/fixtures/colcon_probe/README.md.
BROKEN_INSTALL = (
    Path(__file__).parent / "fixtures" / "colcon_probe" / "lf_analyze_import_broken.stderr"
).read_text()


@pytest.fixture
def fake_colcon(tmp_path, monkeypatch):
    """Put a ``colcon`` on PATH that prints *stderr* and exits with *rc*."""

    def install(stderr: str, rc: int) -> None:
        (tmp_path / "stderr.txt").write_text(stderr)
        colcon = tmp_path / "colcon"
        colcon.write_text(f"#!/bin/sh\ncat '{tmp_path / 'stderr.txt'}' >&2\nexit {rc}\n")
        colcon.chmod(0o755)
        monkeypatch.setenv("PATH", f"{tmp_path}{os.pathsep}{os.environ['PATH']}")
        monkeypatch.delenv(ros_env.REQUIRE_ENV, raising=False)
        ros_env.colcon_lf_verbs_available.cache_clear()

    yield install
    ros_env.colcon_lf_verbs_available.cache_clear()


def _skip_condition(mark: pytest.MarkDecorator) -> bool:
    return bool(mark.mark.args[0])


def test_invalid_choice_from_colcon_skips(fake_colcon):
    fake_colcon(INVALID_CHOICE, 2)
    assert _skip_condition(ros_env.requires_colcon_lf_verbs("probe test"))


def test_invalid_choice_errors_when_ci_requires_the_tests(fake_colcon, monkeypatch):
    fake_colcon(INVALID_CHOICE, 2)
    monkeypatch.setenv(ros_env.REQUIRE_ENV, "1")
    with pytest.raises(RuntimeError, match="not a registered colcon verb"):
        ros_env.requires_colcon_lf_verbs("probe test")


def test_broken_verb_errors_with_the_probe_stderr(fake_colcon):
    fake_colcon(TRACEBACK, 1)
    with pytest.raises(RuntimeError, match="cannot import name 'broken'"):
        ros_env.requires_colcon_lf_verbs("probe test")


def test_invalid_choice_after_a_failed_lf_verb_load_errors(fake_colcon):
    fake_colcon(BROKEN_INSTALL, 2)
    with pytest.raises(RuntimeError, match="lf-analyze import broken"):
        ros_env.requires_colcon_lf_verbs("probe test")


def test_unrelated_extension_load_failure_still_skips(fake_colcon):
    other = (
        "ERROR:colcon.colcon_core.extension_point:Exception loading extension "
        "'colcon_core.event_handler.desktop_notification': No module named 'notify2'\n"
    )
    fake_colcon(other + INVALID_CHOICE, 2)
    assert _skip_condition(ros_env.requires_colcon_lf_verbs("probe test"))


def test_working_verb_runs(fake_colcon):
    fake_colcon("", 0)
    assert not _skip_condition(ros_env.requires_colcon_lf_verbs("probe test"))


# Tests that read a launch file guard on the ROS launch packages.
# Under ``PYTHONPATH=$PWD`` they do not import and the analysis tests failed with
# a KeyError far from the cause instead of skipping.

def test_missing_launch_packages_skip(monkeypatch):
    monkeypatch.setattr(ros_env, "launch_importable", lambda: False)
    monkeypatch.delenv(ros_env.REQUIRE_ENV, raising=False)
    mark = ros_env.requires_launch_ros("launch test")
    assert _skip_condition(mark)
    assert "PYTHONPATH" in mark.mark.kwargs["reason"]


def test_missing_launch_packages_error_when_ci_requires_the_tests(monkeypatch):
    monkeypatch.setattr(ros_env, "launch_importable", lambda: False)
    monkeypatch.setenv(ros_env.REQUIRE_ENV, "1")
    with pytest.raises(RuntimeError, match="launch / launch_ros Python packages do not import"):
        ros_env.requires_launch_ros("launch test")


def test_importable_launch_packages_run(monkeypatch):
    monkeypatch.setattr(ros_env, "launch_importable", lambda: True)
    monkeypatch.setenv(ros_env.REQUIRE_ENV, "1")
    assert not _skip_condition(ros_env.requires_launch_ros("launch test"))


@pytest.fixture
def fake_launch_packages(tmp_path, monkeypatch):
    """Put stand-in ``launch`` / ``launch_ros`` packages first on sys.path.

    Every ``launch*`` module, real or fake, is out of ``sys.modules`` while the
    test runs and the real ones are back afterwards, so no fake leaks into a
    later test (an order dependence it was once mistaken for).
    """
    import sys

    def ours(name):
        return name.split(".")[0] in ("launch", "launch_ros")

    saved = {m: sys.modules[m] for m in list(sys.modules) if ours(m)}
    for name in saved:
        del sys.modules[name]
    monkeypatch.syspath_prepend(str(tmp_path))

    def make(launch_ros_body):
        (tmp_path / "launch").mkdir()
        (tmp_path / "launch" / "__init__.py").write_text("")
        (tmp_path / "launch" / "launch_description_sources.py").write_text("")
        (tmp_path / "launch_ros").mkdir()
        (tmp_path / "launch_ros" / "__init__.py").write_text(launch_ros_body)

    yield make
    for name in [m for m in list(sys.modules) if ours(m)]:
        del sys.modules[name]
    sys.modules.update(saved)


def test_launch_packages_that_fail_to_import_are_not_importable(fake_launch_packages):
    # The package is found on sys.path but its own import fails (a missing
    # dependency): the parser's import fails the same way, so the guard must
    # skip, not run the test into a "launch package not available" error.
    fake_launch_packages("raise ImportError(\"No module named 'rclpy'\")\n")
    assert not ros_env.launch_importable(), (
        "launch_importable() is True for a launch_ros that fails to import: the "
        "guard checks that the package is found, not that it imports"
    )


def test_launch_packages_that_import_are_importable(fake_launch_packages):
    fake_launch_packages("")
    assert ros_env.launch_importable()


def test_fake_launch_packages_do_not_outlive_their_test():
    import sys

    fakes = [m for m, mod in sys.modules.items()
             if m.split(".")[0] in ("launch", "launch_ros")
             and "pytest-of-" in str(getattr(mod, "__file__", "") or "")]
    assert not fakes, f"stand-in launch modules leaked into later tests: {fakes}"

# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared skip rule for tests that need real rclcpp headers.

On a host without a ROS 2 install, or with ROS 2 but an unbuilt workspace (no
generated message headers, no compile_commands.json), the libclang-based parser
cannot resolve the node classes, so these tests would fail for reasons unrelated
to any change. They skip there instead. CI must never skip them (Phase-5 lesson:
host-skipped tests made CI the first place a real parser bug surfaced), so CI
sets ``RCLLF_REQUIRE_ROS_TESTS=1``, which turns a missing header tree into a
collection error rather than a skip.
"""

from __future__ import annotations

from functools import lru_cache
import importlib.util
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest

KNOWN_DISTROS = ("jazzy", "humble", "iron", "rolling")
REQUIRE_ENV = "RCLLF_REQUIRE_ROS_TESTS"


_PACKAGES_ROOT = Path(__file__).resolve().parents[3] / "packages"

# colcon-core logs an extension that fails to import, e.g.
# "Exception loading extension 'colcon_core.verb.lf-analyze': <error>", then drops it
# from argparse, so the same run also reports the verb as an invalid choice.
_LOAD_FAILURE = re.compile(r"Exception loading extension '([^']+)'")


def ros_headers_available() -> bool:
    return any(Path(f"/opt/ros/{d}/include/rclcpp").is_dir() for d in KNOWN_DISTROS)


def workspace_built() -> bool:
    return (_PACKAGES_ROOT / "install").is_dir() and (_PACKAGES_ROOT / "build").is_dir()


@lru_cache(maxsize=None)
def colcon_lf_verbs_available() -> bool:
    """True if the ``colcon`` on PATH knows ``colcon lf-analyze``.

    The verbs exist only once colcon_lf is pip-installed (docker/entrypoint.sh or
    the CI build-workspace action do it). Ask colcon itself: the in-process entry
    points are no proof, since pytest puts tools/colcon_lf on sys.path and an old
    ``colcon_lf.egg-info`` there lists the verbs to this process only.

    False only when the verb is provably absent: no ``colcon`` on PATH, or colcon's
    argparse rejects ``lf-analyze`` as an invalid choice and no lf-* verb or
    colcon_lf module failed to load. Any other failure is an installed but broken
    verb, which must fail the run, so it raises with the probe's output instead of
    skipping.
    """
    colcon = shutil.which("colcon")
    if colcon is None:
        return False
    probe = subprocess.run([colcon, "lf-analyze", "--help"], capture_output=True, text=True,
                           timeout=120)
    if probe.returncode == 0:
        return True
    absent = "invalid choice: 'lf-analyze'" in probe.stderr
    if absent and not _colcon_lf_load_failed(probe.stderr):
        return False
    raise RuntimeError(
        f"`{colcon} lf-analyze --help` exited {probe.returncode}: the verb is registered "
        f"but broken.\nstdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
    )


def _colcon_lf_load_failed(stderr: str) -> bool:
    """True if colcon reports a failed load of an lf-* extension or of colcon_lf."""
    for name in _LOAD_FAILURE.findall(stderr):
        if re.fullmatch(r"lf(-.*)?", name.rsplit(".", 1)[-1]):
            return True
    return bool(_LOAD_FAILURE.search(stderr)) and "colcon_lf" in stderr


def requires_colcon_lf_verbs(what: str) -> pytest.MarkDecorator:
    """Module-level ``pytestmark`` for tests that shell the ``colcon lf-*`` CLI."""
    missing = "colcon lf-analyze is not a registered colcon verb (colcon_lf not pip-installed)"
    if not colcon_lf_verbs_available() and os.environ.get(REQUIRE_ENV) == "1":
        raise RuntimeError(f"{REQUIRE_ENV}=1 but {missing}: {what} would be skipped")
    return pytest.mark.skipif(
        not colcon_lf_verbs_available(),
        reason=f"{missing}: {what} shell the colcon CLI. Run "
        "`pip install -e tools/colcon_lf` or use the dev container entrypoint.",
    )


def launch_importable() -> bool:
    """True if the ROS ``launch`` and ``launch_ros`` Python packages import.

    It imports them rather than only finding them: a package on the path whose
    own import fails (a missing dependency) fails the launch parser too.
    """
    try:
        importlib.import_module("launch.launch_description_sources")
        importlib.import_module("launch_ros")
    except ImportError:
        return False
    return True


def requires_launch_ros(what: str) -> pytest.MarkDecorator:
    """Module-level ``pytestmark`` for tests that read a launch file.

    The ROS Python packages come from the ``PYTHONPATH`` that
    ``/opt/ros/<distro>/setup.bash`` sets, and ``PYTHONPATH=$PWD pytest``
    replaces it. The launch parser then records "launch package not available"
    in the launch file's errors and returns no nodes, so a test of the analysis
    fails far from the cause, with a ``KeyError`` on a topic or a node.
    """
    missing = (
        "the ROS launch / launch_ros Python packages do not import (is "
        "/opt/ros/<distro>/lib/python3.*/site-packages on PYTHONPATH?)"
    )
    if not launch_importable() and os.environ.get(REQUIRE_ENV) == "1":
        raise RuntimeError(f"{REQUIRE_ENV}=1 but {missing}: {what} would be skipped")
    return pytest.mark.skipif(
        not launch_importable(),
        reason=f"{missing}. {what} read launch files; keep the ROS path when you "
        "set PYTHONPATH, e.g. PYTHONPATH=$PWD:$PYTHONPATH.",
    )


def requires_ros_headers(what: str) -> pytest.MarkDecorator:
    """Module-level ``pytestmark`` for tests needing rclcpp headers."""
    missing = []
    if not ros_headers_available():
        missing.append("no /opt/ros/<distro>/include/rclcpp")
    if not workspace_built():
        missing.append(f"workspace not built ({_PACKAGES_ROOT}/build or install missing)")
    if missing and os.environ.get(REQUIRE_ENV) == "1":
        raise RuntimeError(
            f"{REQUIRE_ENV}=1 but {'; '.join(missing)}: {what} would be skipped, "
            "and this environment must run them"
        )
    return pytest.mark.skipif(
        bool(missing),
        reason=f"{'; '.join(missing)}. {what} need real rclcpp headers and a built "
        "workspace; run inside the dev container after `colcon build` for full coverage.",
    )

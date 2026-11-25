# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A topic named by a launch parameter is resolved end to end.

Fixture: ``tests/fixtures/launch_param_topic/src/param_topic_pkg``. Its node
declares ``output_topic`` with no C++ default and publishes on the value it reads
back, so the clang parser records ``${output_topic}`` with ``topic_dynamic``
set. Only the launch file names the topic (``fixture_cmd``). The launch overlay
in ``graph/hierarchical.py`` must fill in the name and clear the flag, otherwise
the resolved model records the publisher as unresolved (RCLLF-H004), see 49f9a16.

The test runs the real ``colcon lf-analyze`` and ``colcon lf-generate`` commands
on a tmp copy. ``fixtures/launch_param_topic/recorded_run.json`` holds the
publisher entry and the resolved-model topic record from a real run in the dev
container. Re-record it from such a run, never by hand.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from .ros_env import (
    REQUIRE_ENV,
    requires_colcon_lf_verbs,
    requires_launch_ros,
    ros_headers_available,
)

if not ros_headers_available() and os.environ.get(REQUIRE_ENV) == "1":
    raise RuntimeError(f"{REQUIRE_ENV}=1 but no rclcpp headers")
# Skip marks, not a module-level pytest.skip: launch_testing's collection hook
# imports every test module, and a skip raised there drops the whole tests/
# package from the run.
pytestmark = [
    pytest.mark.skipif(not ros_headers_available(), reason="needs real rclcpp headers"),
    requires_colcon_lf_verbs("the launch parameter topic tests"),
    # lf-analyze reads the launch file in this process's environment.
    requires_launch_ros("the launch parameter topic tests"),
]

FIXTURE = Path(__file__).parent / "fixtures" / "launch_param_topic"
_LAUNCH_VALUE = "fixture_cmd"  # parameters=[{'output_topic': ...}] in bringup.launch.py


def _run(args: list[str], cwd: Path) -> None:
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, (
        f"{' '.join(args)} returned {result.returncode}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )


@pytest.fixture(scope="module")
def outputs(tmp_path_factory) -> Path:
    """Analyze and generate the fixture once, return the analysis directory."""
    ws = tmp_path_factory.mktemp("launch_param_topic") / "ws"
    shutil.copytree(FIXTURE / "src", ws / "src")
    # Install the node's own headers, as a colcon build would.
    shutil.copytree(
        FIXTURE / "src" / "param_topic_pkg" / "include",
        ws / "install" / "param_topic_pkg" / "include",
    )
    _run(["colcon", "lf-analyze", "--package", "param_topic_pkg",
          "--workspace", str(ws), "--output-dir", str(ws / "out")], cwd=ws)
    analysis = ws / "out" / "param_topic_pkg" / "bringup"
    _run(["colcon", "lf-generate", "--system-json", str(analysis / "system.json"),
          "--workspace", str(ws), "--output-dir", str(ws / "gen")], cwd=ws)
    return analysis


def _recorded() -> dict:
    return json.loads((FIXTURE / "recorded_run.json").read_text())


def test_system_json_publisher_takes_launch_value(outputs):
    pubs = json.loads((outputs / "system.json").read_text())["nodes"]["param_topic_node"][
        "publishers"]
    assert len(pubs) == 1
    pub = pubs[0]
    assert (pub["topic"], pub["topic_dynamic"]) == (_LAUNCH_VALUE, False), (
        f"publisher topic is {pub['topic']!r} with topic_dynamic={pub['topic_dynamic']}: "
        "the endpoint is still dynamic although the launch parameter "
        f"output_topic={_LAUNCH_VALUE!r} names it"
    )
    assert pub == _recorded()["system_json_publisher"]


def test_resolved_model_topic_record_is_extracted(outputs):
    model = json.loads((outputs / "resolved_model.json").read_text())
    topic_records = [r for r in model["records"] if r["property"] == "topic"]
    assert len(topic_records) == 1
    record = topic_records[0]
    assert (record["value"], record["provenance"]) == (f"/{_LAUNCH_VALUE}", "extracted"), (
        f"resolved model records the publisher topic as {record['value']!r} with "
        f"provenance {record['provenance']!r} (reason {record.get('reason')}): the "
        "topic is still unresolved although the launch file names it"
    )
    assert record == _recorded()["resolved_model_topic_record"]

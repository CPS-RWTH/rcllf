# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A reader whose twin topic comes from a launch parameter has one answer.

Example: ``packages/src/fixtures/lifecycle_rebind_example``, launch ``reader_param``. The
``param_reader_node`` stores ``declare_parameter("twin_topic")`` in a member in
its constructor and subscribes on it in ``on_configure``; the launch file sets
``twin_topic: /rebind/feed``. Before the fix the system.json row kept the twin
topic ``''`` while the parameter row held ``/rebind/feed``, correspondence
admitted the one edge it saw for rebind, and the binding refused the reader
(rebind_identity, "analysis found 1 and the node holds 2").

The ``reader`` launch, whose twin topic comes from an environment variable,
must stay unresolved so it keeps exercising rebind_identity.

The test runs the real ``colcon lf-analyze`` and ``colcon lf-generate`` commands
on a tmp copy of the example.
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
    requires_colcon_lf_verbs("the parameter twin topic tests"),
    # lf-analyze reads the launch file in this process's environment.
    requires_launch_ros("the parameter twin topic tests"),
]

PKG = "lifecycle_rebind_example"
EXAMPLE = Path(__file__).resolve().parents[3] / "packages" / "src" / "fixtures" / PKG


def _run(args: list[str], cwd: Path) -> None:
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=900)
    assert result.returncode == 0, (
        f"{' '.join(args)} returned {result.returncode}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )


@pytest.fixture(scope="module")
def ws(tmp_path_factory) -> Path:
    """Analyze the example and generate the reader_param launch once."""
    ws = tmp_path_factory.mktemp("param_twin_topic") / "ws"
    shutil.copytree(EXAMPLE, ws / "src" / PKG)
    _run(["colcon", "lf-analyze", "--package", PKG,
          "--workspace", str(ws), "--output-dir", str(ws / "out")], cwd=ws)
    system_json = ws / "out" / PKG / "reader_param" / "system.json"
    _run(["colcon", "lf-generate", "--system-json", str(system_json),
          "--workspace", str(ws), "--output-dir", str(ws / "gen")], cwd=ws)
    return ws


def _twin_row(ws: Path, launch: str) -> dict:
    nodes = json.loads((ws / "out" / PKG / launch / "system.json").read_text())["nodes"]
    rows = [s for n in nodes.values() for s in n["subscribers"]
            if s["callback"].endswith("on_twin")]
    assert len(rows) == 1, f"{launch}: expected one on_twin row, got {rows}"
    return rows[0]


def test_system_json_twin_row_takes_the_launch_value(ws):
    row = _twin_row(ws, "reader_param")
    got = (row["topic"], row["topic_dynamic"], row.get("topic_param"))
    assert got == ("/rebind/feed", False, "twin_topic"), (
        f"on_twin row (topic, topic_dynamic, topic_param) is {got} although the launch "
        "parameter twin_topic='/rebind/feed' names it: the row and the parameter row "
        "give two answers in one analysis")
    topics = json.loads((ws / "out" / PKG / "reader_param" / "system.json").read_text())[
        "communication"]["topics"]
    feed = next(t for t in topics if t["topic"] == "/rebind/feed")
    assert sorted(feed["subscribers"]) == ["reader", "reader"], feed


def test_environment_twin_stays_unresolved(ws):
    row = _twin_row(ws, "reader")
    got = (row["topic"], row["topic_dynamic"], row.get("topic_param"))
    assert got == ("", True, None), f"reader launch on_twin row is {got}"


def test_correspondence_keeps_the_twin_reader_legacy(ws):
    corr = next(p for p in ws.rglob("correspondence.json") if "readerparam" in str(p))
    got = sorted((e["status"], e.get("reason")) for e in json.loads(corr.read_text())
                 if e["topic"] == "/rebind/feed")
    assert got == [("legacy", "ambiguous_endpoint")] * 2, (
        f"/rebind/feed edges are {got}: the reader holds two subscriptions on the topic, "
        "so an edge admitted for rebind is refused at bind (rebind_identity)")

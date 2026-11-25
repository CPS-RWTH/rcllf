# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``colcon lf-analyze`` stops when the ROS launch packages do not import.

The launch parser cannot evaluate any launch file without ``launch``; it
returns the file with no nodes and the error "launch package not available".
The verb used to print that as a WARNING, write a 0-node system.json and exit
0, so generate and build ran on an empty system. The packages are hidden here
by mapping them to None in ``sys.modules``, which makes their import raise
ImportError whether or not ROS is installed.
"""

from __future__ import annotations

import sys

import pytest

pytest.importorskip("colcon_core.verb", reason="colcon_core not available")

LAUNCH = '''\
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package="pkg", executable="talker", name="talker")])
'''


@pytest.fixture
def hidden_launch(tmp_path, monkeypatch):
    from colcon_lf.launch_analysis.parser import clear_parse_launch_cache

    for name in ("launch", "launch.launch_description_sources", "launch_ros"):
        monkeypatch.setitem(sys.modules, name, None)
    launch = tmp_path / "src" / "pkg" / "launch" / "bringup.launch.py"
    launch.parent.mkdir(parents=True)
    launch.write_text(LAUNCH)
    clear_parse_launch_cache()
    yield launch
    clear_parse_launch_cache()


def _assert_refused(rc, err, out_root):
    from colcon_lf.verb import analyze

    # getattr: on the unfixed code the constant does not exist yet, and the
    # failure must be the exit code below, not an ImportError.
    hint = getattr(analyze, "LAUNCH_UNAVAILABLE_HINT", "<no hint defined>")
    written = sorted(p.name for p in out_root.rglob("*.json")) if out_root.exists() else []
    assert rc == 1, (
        f"lf-analyze exited {rc} with the ROS launch packages hidden and wrote "
        f"{written}: an empty system goes on to generate and build"
    )
    assert "launch package not available" in err, err
    assert hint in err, err
    assert "PYTHONPATH" in hint
    assert written == [], f"refused analysis still wrote {written}"


def test_analyze_refuses_without_launch_packages(hidden_launch, tmp_path, capsys):
    from colcon_lf.verb import analyze

    out = tmp_path / "out"
    rc = analyze._analyze_one(
        package="pkg", launch_file=hidden_launch.name, launch_path=hidden_launch,
        workspace=tmp_path, output_dir=str(out), output_fmt="json")
    _assert_refused(rc, capsys.readouterr().err, out)


def test_launch_only_refuses_without_launch_packages(hidden_launch, tmp_path, capsys):
    from colcon_lf.verb import analyze

    out = tmp_path / "out"
    rc = analyze._parse_launch_only(
        package="pkg", launch_file=hidden_launch.name, launch_path=hidden_launch,
        workspace=tmp_path, output_dir=str(out))
    _assert_refused(rc, capsys.readouterr().err, out)


def test_other_launch_errors_stay_warnings(capsys):
    """Only the missing-package error is fatal; an unevaluable include is not.

    A control: it passes on the unfixed code too, where no launch error is
    fatal (the fallback below stands in for the absent helper).
    """
    from colcon_lf.launch_analysis.models import LaunchFileInfo
    from colcon_lf.verb import analyze

    refuse = getattr(analyze, "_refuse_without_launch", lambda _root: False)
    info = LaunchFileInfo(file_path="x.launch.py", package="pkg",
                          errors=["include not evaluated: foo"])
    assert refuse(info) is False, (
        "an unevaluable include stopped lf-analyze; only 'launch package not "
        "available' is fatal")
    assert capsys.readouterr().err == ""

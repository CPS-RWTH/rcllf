# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""``ros2 lf launch`` forwards only the launch arguments the generated file declares."""

from __future__ import annotations

import types

from ros2lf.verb.launch import LaunchVerb, declared_launch_arguments

_PKG = "lf_generated_pubsub_example_bringup"
_FILE = "lf_bringup.launch.py"


def _generated(tmp_path, text):
    launch_dir = tmp_path / "install" / _PKG / "share" / _PKG / "launch"
    launch_dir.mkdir(parents=True)
    (launch_dir / _FILE).write_text(text)


def test_declared_arguments_are_read_from_the_generated_file(tmp_path):
    _generated(tmp_path, "DeclareLaunchArgument('rcllf_workers', default_value='2')\n")
    assert declared_launch_arguments(tmp_path, _PKG, _FILE) == {"rcllf_workers"}


def test_an_undeclared_argument_is_refused_with_the_rerun_command(tmp_path):
    _generated(tmp_path, "# declares nothing\n")
    args = types.SimpleNamespace(package="pubsub_example")
    text = LaunchVerb._refuse_undeclared(tmp_path, _PKG, _FILE, args, ["extra:=true"])
    assert "launch argument 'extra' is fixed when colcon lf analyzes" in text
    assert "this system accepts: (none)" in text
    assert "colcon lf --package pubsub_example --launch-arg extra:=VALUE" in text


def test_a_declared_argument_is_forwarded(tmp_path):
    _generated(tmp_path, 'DeclareLaunchArgument("rcllf_workers")\n')
    args = types.SimpleNamespace(package="pubsub_example")
    assert LaunchVerb._refuse_undeclared(
        tmp_path, _PKG, _FILE, args, ["rcllf_workers:=3"]) is None

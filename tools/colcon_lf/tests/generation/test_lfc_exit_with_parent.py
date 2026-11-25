# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The single-process lfc process must die with ros2 launch.

ros2 launch exits on a SIGTERM without stopping its processes.
The federate launch files pass the launch pid in ``RCLLF_EXIT_WITH_PARENT``
so the binding arms PDEATHSIG; the single-process launch file did not, and
the lfc process stayed behind as an orphan. This test runs the generated
launch file's ``generate_launch_description`` against stand-in ``launch``
modules and checks the env the node gets.
"""
from __future__ import annotations

import os
import sys
import types

import pytest

from .test_lfc_share_system_json import emitted  # noqa: F401  (fixture)


class _Recorder:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


def _stub_modules(monkeypatch):
    launch = types.ModuleType("launch")
    launch.LaunchDescription = _Recorder
    actions = types.ModuleType("launch_ros.actions")
    actions.Node = _Recorder
    # The launch registers an exit handler on its node.
    launch_actions = types.ModuleType("launch.actions")
    launch_actions.RegisterEventHandler = _Recorder
    handlers = types.ModuleType("launch.event_handlers")
    handlers.OnProcessExit = _Recorder
    launch.actions, launch.event_handlers = launch_actions, handlers
    launch_ros = types.ModuleType("launch_ros")
    launch_ros.actions = actions
    ament = types.ModuleType("ament_index_python.packages")
    ament.get_package_share_directory = lambda pkg: f"/opt/share/{pkg}"
    ament_root = types.ModuleType("ament_index_python")
    ament_root.packages = ament
    for name, mod in {
        "launch": launch, "launch.actions": launch_actions,
        "launch.event_handlers": handlers, "launch_ros": launch_ros, "launch_ros.actions": actions,
        "ament_index_python": ament_root, "ament_index_python.packages": ament,
    }.items():
        monkeypatch.setitem(sys.modules, name, mod)


def test_launch_node_exits_with_the_launch_process(emitted, monkeypatch):  # noqa: F811
    _, pkg_dir, _ = emitted
    (launch,) = (pkg_dir / "launch").glob("lf_*.launch.py")
    _stub_modules(monkeypatch)
    monkeypatch.delenv("RCLLF_EXIT_WITH_PARENT", raising=False)
    scope: dict = {}
    exec(compile(launch.read_text(), str(launch), "exec"), scope)
    node, handler = scope["generate_launch_description"]().args[0]
    # A nonzero exit of the node fails the launch.
    (on_exit,) = handler.args
    assert on_exit.kwargs["target_action"] is node, "exit handler not bound to the node"
    env = node.kwargs["additional_env"]
    # The pid of the process that evaluates the launch file, i.e. ros2 launch,
    # which the binding compares with getppid() (runtime.cpp).
    assert env.get("RCLLF_EXIT_WITH_PARENT") == str(os.getpid()), (
        "single-process launch file does not set RCLLF_EXIT_WITH_PARENT; "
        "the lfc process outlives a killed ros2 launch"
    )
    assert set(env) == {"SYSTEM_JSON", "ROS2_LF_EXEC_CONFIG", "RCLLF_EXIT_WITH_PARENT"}


def test_nonzero_program_exit_fails_the_launch(emitted, monkeypatch):  # noqa: F811
    # `ros2 launch` exits 0 whatever its
    # processes return unless a handler raises, so a failed reconcile (exit 1)
    # would pass every run check that reads the launch exit code.
    _, pkg_dir, _ = emitted
    (launch,) = (pkg_dir / "launch").glob("lf_*.launch.py")
    _stub_modules(monkeypatch)
    scope: dict = {}
    exec(compile(launch.read_text(), str(launch), "exec"), scope)
    fail = scope["_exit_fails_the_launch"]
    fail(types.SimpleNamespace(returncode=0), None)
    with pytest.raises(RuntimeError, match="exited 1, the run failed"):
        fail(types.SimpleNamespace(returncode=1), None)

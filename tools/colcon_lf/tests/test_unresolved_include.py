# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""An unresolved #include in a node source fails `colcon lf-analyze`.

Fixture: ``tests/fixtures/unresolved_include/src/blind_pkg``. Its node header
includes ``<vendor_sdk/sdk.h>``, which no include path holds, the way the MuJoCo
headers sat only in ``build/aeb_mujoco_sim`` before 8e4fc3c. The missing header
is a fatal libclang error, after which clang stops instantiating templates, so
every ``create_*`` call drops out of the AST. Before the fix the analyzer wrote
a ``system.json`` with the node and no entities, and returned 0.

The fixture is copied into ``tmp_path`` with its own headers installed and
analyzed without a build, so there is no compile_commands.json and the parser
takes the fallback include path.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("clang.cindex", reason="libclang not available")
pytest.importorskip("colcon_core.verb", reason="colcon_core not available")
pytest.importorskip("launch_ros", reason="launch_ros not available")

from .ros_env import REQUIRE_ENV, ros_headers_available  # noqa: E402

if not ros_headers_available():
    if os.environ.get(REQUIRE_ENV) == "1":
        raise RuntimeError(f"{REQUIRE_ENV}=1 but no /opt/ros/<distro>/include/rclcpp")
    pytest.skip("needs real rclcpp headers", allow_module_level=True)

FIXTURE = Path(__file__).parent / "fixtures" / "unresolved_include"


def _workspace(tmp_path: Path) -> Path:
    """Copy the fixture and install its own headers, as a colcon build would.

    Only <vendor_sdk/sdk.h> stays unreachable, so the node class is found and
    its entities are what goes missing (the vendored-SDK shape).
    """
    ws = tmp_path / "ws"
    shutil.copytree(FIXTURE / "src", ws / "src")
    shutil.copytree(
        FIXTURE / "src" / "blind_pkg" / "include",
        ws / "install" / "blind_pkg" / "include",
    )
    return ws


def _analyze(ws: Path) -> int:
    from colcon_lf.node_analysis.clang_parser import clear_parse_node_cache
    from colcon_lf.verb.analyze import AnalyzeVerb

    clear_parse_node_cache()
    args = argparse.Namespace(
        package="blind_pkg", launch_file=None, workspace=str(ws),
        output_dir=str(ws / "out"), format="json",
        viz_only=False, launch_only=False,
    )
    return AnalyzeVerb().main(context=SimpleNamespace(args=args))


def _entity_counts(system_json: Path) -> str:
    nodes = json.loads(system_json.read_text())["nodes"]
    return "; ".join(
        f"node {key!r}: {len(n['timers'])} timers, {len(n['publishers'])} "
        f"publishers, {len(n['subscribers'])} subscribers"
        for key, n in nodes.items()
    )


def test_unresolved_include_names_node_file_header_and_fails(tmp_path, capsys):
    ws = _workspace(tmp_path)

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"{_entity_counts(system_json)}: the node was silently emitted with no "
        "entities"
    )
    # No dead system.json for lf-generate to pick up.
    assert not system_json.exists()
    pkg = ws / "src" / "blind_pkg"
    assert (
        f"  Unresolved include: node 'blind_node' "
        f"(source {pkg / 'src' / 'blind_node.cpp'}): "
        f"'vendor_sdk/sdk.h' not found, "
        f"included from {pkg / 'include' / 'blind_pkg' / 'blind_node.hpp'}:8\n"
    ) in err
    assert (
        f"  Error: node 'blind_node' (source {pkg / 'src' / 'blind_node.cpp'}) "
        "has unresolved include(s) 'vendor_sdk/sdk.h', so timers, publishers "
        "or subscriptions can be missing from the analysis. Make the headers "
        "reachable (build the package with "
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON, or install them under "
        "install/<pkg>/include) and re-run colcon lf-analyze.\n"
    ) in err


def _sdk_in_build_tree(ws: Path) -> None:
    """Unpack the vendor SDK header under build/blind_pkg, as a build would."""
    sdk_inc = ws / "build" / "blind_pkg" / "vendor_sdk-1.0" / "include" / "vendor_sdk"
    sdk_inc.mkdir(parents=True)
    shutil.copy(FIXTURE / "sdk" / "sdk.h", sdk_inc / "sdk.h")


def _system_nodes(system_json: Path) -> str:
    return _entity_counts(system_json) if system_json.exists() else "no system.json"


def test_unresolved_include_with_visible_entities_fails(tmp_path, capsys):
    """A node with an unresolved include fails even though entities are visible.

    The include sits after the constructor, so clang still records the timer,
    publisher and subscription. What a missing header hides cannot be told
    from what is left, so the node is refused, not passed with what remains.
    """
    ws = _workspace(tmp_path)
    _sdk_in_build_tree(ws)
    cpp = ws / "src" / "blind_pkg" / "src" / "blind_node.cpp"
    cpp.write_text(cpp.read_text() + "\n#include <vendor_extra/extra.h>\n")

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"{_system_nodes(system_json)}: a node with an unresolved include "
        "passed because some entities were visible"
    )
    assert not system_json.exists()
    assert (
        f"  Error: node 'blind_node' (source {cpp}) has unresolved include(s) "
        "'vendor_extra/extra.h', so timers, publishers or subscriptions can "
        "be missing from the analysis."
    ) in err


def test_error_diagnostic_fails(tmp_path, capsys):
    """An error-level libclang diagnostic other than a missing header fails.

    An unknown type is not a missing file, so it only reached the parse
    warnings block and colcon lf-analyze returned 0.
    """
    ws = _workspace(tmp_path)
    _sdk_in_build_tree(ws)
    cpp = ws / "src" / "blind_pkg" / "src" / "blind_node.cpp"
    cpp.write_text(cpp.read_text().replace(
        "namespace blind_pkg\n{\n",
        "namespace blind_pkg\n{\n\nunknown_vendor_t g_stray_handle;\n",
        1,
    ))

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"{_system_nodes(system_json)}: an error-level libclang diagnostic "
        "was only printed as a parse warning"
    )
    assert not system_json.exists()
    assert (
        f"  Error: node 'blind_node' (source {cpp}): libclang reported "
        "error(s) 'unknown type name 'unknown_vendor_t'', so timers, "
        "publishers or subscriptions can be missing from the analysis."
    ) in err


def test_header_in_package_build_tree_resolves(tmp_path, capsys):
    """Fallback search: a header unpacked under build/<pkg>/*/include is found."""
    ws = _workspace(tmp_path)
    _sdk_in_build_tree(ws)

    rc = _analyze(ws)

    err = capsys.readouterr().err
    assert rc == 0, err
    assert "Unresolved include" not in err
    system = json.loads(
        (ws / "out" / "blind_pkg" / "bringup" / "system.json").read_text()
    )
    (node,) = system["nodes"].values()
    assert node["unresolved_includes"] == []
    assert [t["period_ms"] for t in node["timers"]] == [10.0]
    assert [p["topic"] for p in node["publishers"]] == ["state"]
    assert [s["topic"] for s in node["subscribers"]] == ["cmd"]


def test_unresolved_include_hiding_node_class_fails(tmp_path, capsys):
    """The node's own header is unreachable, so clang never sees the class.

    Before this check the graph builder skipped the launched node with a log
    warning and colcon lf-analyze returned 0 with the node missing.
    """
    ws = _workspace(tmp_path)
    pkg = ws / "src" / "blind_pkg"
    (pkg / "include").rename(pkg / "hidden_include")
    shutil.rmtree(ws / "install" / "blind_pkg" / "include")

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    nodes = (
        sorted(json.loads(system_json.read_text())["nodes"])
        if system_json.exists() else None
    )
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"nodes {nodes}: the launched node 'blind_node' was silently dropped "
    )
    assert not system_json.exists()
    cpp = pkg / "src" / "blind_node.cpp"
    assert (
        f"  Error: node 'blind_node' (source {cpp}): no ROS2 node class found, "
        f"because of unresolved include(s): 'blind_pkg/blind_node.hpp' not "
        f"found, included from {cpp}:4. Make the headers reachable (build the "
        f"package with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON, or install them "
        f"under install/<pkg>/include) and re-run colcon lf-analyze.\n"
    ) in err


def _bringup_with(ws: Path, extra_nodes: list[str]) -> None:
    """Launch blind_node (with its SDK reachable) plus *extra_nodes*."""
    _sdk_in_build_tree(ws)
    (ws / "src" / "blind_pkg" / "launch" / "bringup.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n\n\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        Node(package='blind_pkg', executable='blind_node', name='blind_node'),\n"
        + "".join(f"        {node},\n" for node in extra_nodes)
        + "    ])\n"
    )


def test_unresolvable_launched_nodes_fail(tmp_path, capsys):
    """A launched node of a C++ package that does not resolve fails.

    It was skipped with a log warning that colcon never printed, so the node
    vanished from system.json and colcon lf-analyze returned 0.
    """
    ws = _workspace(tmp_path)
    _bringup_with(ws, [
        "Node(package='turtlesim', executable='turtlesim_node', name='turtle')",
        "Node(package='blind_pkg', executable='not_a_target', name='ghost')",
    ])

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    nodes = (
        sorted(json.loads(system_json.read_text())["nodes"])
        if system_json.exists() else None
    )
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"nodes {nodes}: the launched node 'ghost' was silently dropped"
    )
    assert not system_json.exists()
    assert (
        "  Error: node 'ghost' (package 'blind_pkg', executable "
        "'not_a_target'): the executable does not resolve to a source file "
        "in the package's CMakeLists.txt\n"
    ) in err
    assert (
        "  Warning: node 'turtle' (package 'turtlesim', executable "
        "'turtlesim_node') not analyzed: package 'turtlesim' is not under "
        f"{ws / 'src'}\n"
    ) in err


def test_nodes_outside_the_analysis_are_reported_not_failed(tmp_path, capsys):
    """Nodes of a prebuilt or a Python package are reported, and analysis passes.

    Nav2's launch files start rviz2, robot_state_publisher, a component
    container and Python nodes. Failing on them failed 19 of its 25 launch
    files, so they are named on a Warning line and left out of system.json.
    """
    ws = _workspace(tmp_path)
    py_pkg = ws / "src" / "py_pkg"
    (py_pkg / "py_pkg").mkdir(parents=True)
    (py_pkg / "package.xml").write_text(
        (FIXTURE / "src" / "blind_pkg" / "package.xml").read_text()
        .replace("blind_pkg", "py_pkg")
        .replace("<buildtool_depend>ament_cmake</buildtool_depend>", "")
        .replace("ament_cmake", "ament_python")
    )
    (py_pkg / "setup.py").write_text("")
    _bringup_with(ws, [
        "Node(package='turtlesim', executable='turtlesim_node', name='turtle')",
        "Node(package='py_pkg', executable='py_node', name='py_node')",
    ])

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    assert rc == 0, f"colcon lf-analyze returned {rc}, stderr:\n{err}"
    assert sorted(json.loads(system_json.read_text())["nodes"]) == ["blind_node"]
    assert (
        "  Warning: node 'turtle' (package 'turtlesim', executable "
        "'turtlesim_node') not analyzed: package 'turtlesim' is not under "
        f"{ws / 'src'}\n"
    ) in err
    assert (
        "  Warning: node 'py_node' (package 'py_pkg', executable 'py_node') "
        "not analyzed: package 'py_pkg' has build type 'ament_python', so it "
        "is not a C++ package\n"
    ) in err


def test_cmake_package_without_cmakelists_fails(tmp_path, capsys):
    """An ament_cmake package without its CMakeLists.txt is broken, not Python.

    Classifying by the missing file made it a Warning line, and its launched
    node vanished from system.json while colcon lf-analyze returned 0.
    """
    ws = _workspace(tmp_path)
    cm_pkg = ws / "src" / "cm_pkg"
    shutil.copytree(ws / "src" / "blind_pkg", cm_pkg, ignore=shutil.ignore_patterns("launch"))
    (cm_pkg / "CMakeLists.txt").unlink()
    (cm_pkg / "package.xml").write_text(
        (cm_pkg / "package.xml").read_text().replace("blind_pkg", "cm_pkg")
    )
    _bringup_with(ws, ["Node(package='cm_pkg', executable='cm_node', name='cm_node')"])

    rc = _analyze(ws)

    err = capsys.readouterr().err
    system_json = ws / "out" / "blind_pkg" / "bringup" / "system.json"
    nodes = (
        sorted(json.loads(system_json.read_text())["nodes"])
        if system_json.exists() else None
    )
    assert rc == 1, (
        f"colcon lf-analyze returned {rc} and wrote {system_json.name} with "
        f"nodes {nodes}: the node 'cm_node' of the ament_cmake package "
        "'cm_pkg' was dropped as if it were not a C++ package"
    )
    assert not system_json.exists()
    assert (
        "  Error: node 'cm_node' (package 'cm_pkg', executable 'cm_node'): "
        "package 'cm_pkg' has build type 'ament_cmake' but no "
        f"{cm_pkg / 'CMakeLists.txt'}, so the executable cannot be resolved "
        "to a source file\n"
    ) in err

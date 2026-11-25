# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for ros2lf.api.discovery — the lf_generated_<package>_<name> artifact layout.

``ros2 lf list``/``ros2 lf launch`` only recognized stock-mode
launch/wrapper artifacts (``lf-analysis/<pkg>/<stem>/launch/`` +
``wrapper/install/setup.bash``). ``--target-lfc`` writes a different layout
under ``lf-generated/lf_generated_<package>_<name>/`` (see
``colcon_lf/verb/generate.py`` and ``colcon_lf/generation/lf_naming.py``'s
``launch_stem_to_reactor_name``) and installs into the workspace
``install/`` tree via a normal colcon build — discovery never looked there.

These fabricate both layouts on ``tmp_path`` without ROS2/colcon.
"""

from __future__ import annotations

import json

from ros2lf.api.discovery import discover_systems, find_system, stale_wrapper_artifacts


def _write_system_json(base, pkg, stem, *, node_count=2):
    system_dir = base / "lf-analysis" / pkg / stem
    system_dir.mkdir(parents=True, exist_ok=True)
    nodes = {f"Node{i}": {} for i in range(node_count)}
    data = {
        "nodes": nodes,
        "launch_graph": {
            "all_nodes": [{"is_component": False} for _ in nodes]
        },
    }
    (system_dir / "system.json").write_text(json.dumps(data))
    return system_dir


def test_stock_layout_is_ignored(tmp_path):
    """Stock wrapper artifacts no longer count as a system."""
    workspace = tmp_path
    system_dir = _write_system_json(workspace, "pubsub_example", "bringup")
    launch_dir = system_dir / "launch"
    launch_dir.mkdir()
    (launch_dir / "lf_bringup.launch.py").write_text("# stub")
    wrapper_install = system_dir / "wrapper" / "install"
    wrapper_install.mkdir(parents=True)
    (wrapper_install / "setup.bash").write_text("# stub")

    entry = find_system(workspace, "pubsub_example", "bringup")
    assert entry is not None
    assert entry.layout is None
    assert entry.has_launch is False
    assert entry.wrapper_built is False


def test_lfc_layout_unbuilt(tmp_path):
    """--target-lfc source artifact exists but was never colcon-built."""
    workspace = tmp_path
    _write_system_json(workspace, "pubsub_example", "bringup")

    # lf-generate --target-lfc writes the source launch file under
    # lf-generated/lf_generated_<name>/launch/lf_<name>.launch.py, where
    # <name> = launch_stem_to_reactor_name(stem).lower() -- here "bringup".
    gen_launch_dir = (
        workspace / "lf-generated" / "lf_generated_pubsub_example_bringup" / "launch"
    )
    gen_launch_dir.mkdir(parents=True)
    (gen_launch_dir / "lf_bringup.launch.py").write_text("# stub")

    entry = find_system(workspace, "pubsub_example", "bringup")
    assert entry is not None
    assert entry.has_launch is True
    assert entry.wrapper_built is False
    assert entry.layout == "lfc"


def test_lfc_layout_built(tmp_path):
    """--target-lfc artifact colcon-built: installed share launch file exists."""
    workspace = tmp_path
    _write_system_json(workspace, "pubsub_example", "bringup")

    gen_launch_dir = (
        workspace / "lf-generated" / "lf_generated_pubsub_example_bringup" / "launch"
    )
    gen_launch_dir.mkdir(parents=True)
    (gen_launch_dir / "lf_bringup.launch.py").write_text("# stub")

    installed_launch_dir = (
        workspace
        / "install"
        / "lf_generated_pubsub_example_bringup"
        / "share"
        / "lf_generated_pubsub_example_bringup"
        / "launch"
    )
    installed_launch_dir.mkdir(parents=True)
    (installed_launch_dir / "lf_bringup.launch.py").write_text("# stub")

    entry = find_system(workspace, "pubsub_example", "bringup")
    assert entry is not None
    assert entry.has_launch is True
    assert entry.wrapper_built is True
    assert entry.layout == "lfc"


def test_underscore_stem_name_transform(tmp_path):
    """lifecycle_node -> PascalCase 'LifecycleNode' -> lowered 'lifecyclenode'.

    Mirrors colcon_lf.generation.lf_naming.launch_stem_to_reactor_name.
    """
    workspace = tmp_path
    _write_system_json(workspace, "basic_components", "lifecycle_node")

    gen_launch_dir = (
        workspace
        / "lf-generated"
        / "lf_generated_basic_components_lifecyclenode"
        / "launch"
    )
    gen_launch_dir.mkdir(parents=True)
    (gen_launch_dir / "lf_lifecyclenode.launch.py").write_text("# stub")

    installed_launch_dir = (
        workspace
        / "install"
        / "lf_generated_basic_components_lifecyclenode"
        / "share"
        / "lf_generated_basic_components_lifecyclenode"
        / "launch"
    )
    installed_launch_dir.mkdir(parents=True)
    (installed_launch_dir / "lf_lifecyclenode.launch.py").write_text("# stub")

    entry = find_system(workspace, "basic_components", "lifecycle_node")
    assert entry is not None
    assert entry.layout == "lfc"
    assert entry.has_launch is True
    assert entry.wrapper_built is True


def test_neither_layout_present(tmp_path):
    """No launch artifacts at all: has_launch False, layout None."""
    workspace = tmp_path
    _write_system_json(workspace, "pubsub_example", "bringup")

    entry = find_system(workspace, "pubsub_example", "bringup")
    assert entry is not None
    assert entry.has_launch is False
    assert entry.wrapper_built is False
    assert entry.layout is None


def test_generated_name_taken_from_lf_file(tmp_path):
    """parameter_node generates ParameterNodeSystem.lf -> lf_generated_parameternodesystem."""
    workspace = tmp_path
    system_dir = _write_system_json(workspace, "parameter_example", "parameter_node")
    (system_dir / "lf").mkdir()
    (system_dir / "lf" / "ParameterNodeSystem.lf").write_text("// stub")
    pkg = "lf_generated_parameter_example_parameternodesystem"
    share = workspace / "install" / pkg / "share" / pkg / "launch"
    share.mkdir(parents=True)
    (share / "lf_parameternodesystem.launch.py").write_text("# stub")

    entry = find_system(workspace, "parameter_example", "parameter_node")
    assert entry is not None
    assert entry.generated_package == pkg
    assert entry.launch_filename == "lf_parameternodesystem.launch.py"
    assert entry.wrapper_built is True
    assert entry.layout == "lfc"


def test_digit_leading_stem_keeps_bare_package_name(tmp_path):
    """11_x generates Launch_11X.lf but the package keeps 11x."""
    workspace = tmp_path
    stem = "11_load_composable_nodes"
    pkg = "lf_generated_launch_features_example_11loadcomposablenodes"
    launch = "lf_11loadcomposablenodes.launch.py"
    system_dir = _write_system_json(workspace, "launch_features_example", stem)
    entry = find_system(workspace, "launch_features_example", stem)
    assert entry.generated_package == pkg
    assert entry.launch_filename == launch

    (system_dir / "lf").mkdir()
    (system_dir / "lf" / "Launch_11LoadComposableNodes.lf").write_text("// stub")
    share = workspace / "install" / pkg / "share" / pkg / "launch"
    share.mkdir(parents=True)
    (share / launch).write_text("# stub")
    entry = find_system(workspace, "launch_features_example", stem)
    assert entry.generated_package == pkg
    assert entry.launch_filename == launch
    assert entry.layout == "lfc"


def test_discover_systems_includes_lfc_layout(tmp_path):
    """discover_systems() (used by `ros2 lf list`) also picks up lfc-only systems."""
    workspace = tmp_path
    _write_system_json(workspace, "pubsub_example", "bringup")

    gen_launch_dir = (
        workspace / "lf-generated" / "lf_generated_pubsub_example_bringup" / "launch"
    )
    gen_launch_dir.mkdir(parents=True)
    (gen_launch_dir / "lf_bringup.launch.py").write_text("# stub")

    entries = discover_systems(workspace)
    assert len(entries) == 1
    assert entries[0].layout == "lfc"
    assert entries[0].has_launch is True


def test_stale_wrapper_layout_cannot_shadow_built_lfc_package(tmp_path):
    """A workspace that ran the old wrapper path once keeps
    lf-analysis/COLCON_IGNORE, launch/lf_bringup.launch.py and wrapper/. Discovery
    must still resolve the built lf_generated_* package and name the leftovers.

    Layout recorded from a real run of the old ``colcon lf --package
    pubsub_example``.
    """
    workspace = tmp_path
    system_dir = _write_system_json(workspace, "pubsub_example", "bringup")
    (workspace / "lf-analysis" / "COLCON_IGNORE").write_text("# legacy\n")
    (system_dir / "launch").mkdir()
    stale_launch = system_dir / "launch" / "lf_bringup.launch.py"
    stale_launch.write_text("# lf_wrapper_bringup, never built")
    (system_dir / "wrapper").mkdir()
    (system_dir / "wrapper" / "package.xml").write_text("<package/>")
    pkg = "lf_generated_pubsub_example_bringup"
    (workspace / "lf-generated" / pkg / "launch").mkdir(parents=True)
    (workspace / "lf-generated" / pkg / "launch" / "lf_bringup.launch.py").write_text("# lfc")
    share = workspace / "install" / pkg / "share" / pkg / "launch"
    share.mkdir(parents=True)
    (share / "lf_bringup.launch.py").write_text("# lfc")

    entry = find_system(workspace, "pubsub_example", "bringup")
    assert entry is not None and entry.layout == "lfc", (
        "ros2 lf launch would pick the stale wrapper launch"
    )
    assert entry.generated_package == pkg
    assert entry.has_launch and entry.wrapper_built
    assert stale_wrapper_artifacts(workspace, "pubsub_example", "bringup") == [
        stale_launch, system_dir / "wrapper"
    ]
    assert stale_wrapper_artifacts(workspace, "other_pkg") == []

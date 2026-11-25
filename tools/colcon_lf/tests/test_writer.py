# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for output/writer.py's config-file writing.

Regression for a real bug found on this branch: `graph/hierarchical.py`
started keying `SystemGraph.parameter_files` by resolved *absolute* path
(to fix a same-basename cache collision), but `writer.py`'s
`config_dir / filename` silently discards `config_dir` when `filename` is
absolute (pathlib's `/` operator) — so `colcon lf-analyze` overwrote the
real source params.yaml files it had just read, in place, with a
`yaml.dump` round-trip. Falsified against the pre-fix code (see the commit
message): a real `colcon lf-analyze --package parameter_example` run left
`packages/src/parameter_example/config/{a,b}/params.yaml` and
`config/params.yaml` modified on disk.
"""

from __future__ import annotations

import hashlib

import pytest

from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.output.writer import _sanitize_config_filename, write

from .ros_env import requires_ros_headers

pytestmark = requires_ros_headers("real colcon lf-analyze against parameter_example")


def _make_graph(parameter_files: dict) -> SystemGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="test_system",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        parameter_files=parameter_files,
    )


def _hash_tree(root) -> dict:
    return {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(root.rglob("*"))
        if p.is_file() and "__pycache__" not in p.parts
    }


class TestSanitizeConfigFilename:
    def test_flattens_path_separators(self):
        assert _sanitize_config_filename("pkg/config/a/params.yaml") == (
            "pkg__config__a__params.yaml"
        )

    def test_same_basename_different_dirs_stay_distinct(self):
        a = _sanitize_config_filename("pkg/config/a/params.yaml")
        b = _sanitize_config_filename("pkg/config/b/params.yaml")
        assert a != b

    def test_never_collides_with_execution_yaml(self):
        assert _sanitize_config_filename("execution.yaml") != "execution.yaml"

    def test_absolute_path_outside_workspace_flattened_safely(self):
        safe = _sanitize_config_filename("/tmp/outside/params.yaml")
        assert "/" not in safe

    def test_result_always_has_yaml_extension(self):
        assert _sanitize_config_filename("pkg/config/weird_no_ext").endswith(".yaml")


class TestWriteConfigFilesNeverEscapesOutputDir:
    def test_same_basename_files_land_under_config_with_distinct_names(self, tmp_path):
        graph = _make_graph({
            "pkg/config/a/params.yaml": {"node_a": {"ros__parameters": {"x": 1}}},
            "pkg/config/b/params.yaml": {"node_b": {"ros__parameters": {"x": 2}}},
        })
        out = write(graph, tmp_path / "out")
        config_files = sorted(p.name for p in (out / "config").iterdir())
        assert len(config_files) == 2
        assert len(set(config_files)) == 2  # distinct names, no collision

    def test_no_file_written_outside_output_dir(self, tmp_path):
        # Simulates the exact pre-fix hazard: a key that looks like an
        # absolute path elsewhere on disk. Even so, nothing may be written
        # outside `tmp_path`.
        outside = tmp_path / "outside_workspace"
        outside.mkdir()
        target = outside / "params.yaml"
        target.write_text("original: true\n")
        before = target.read_bytes()

        graph = _make_graph({str(target): {"node": {"ros__parameters": {"x": 1}}}})
        out = write(graph, tmp_path / "out")

        assert target.read_bytes() == before  # untouched
        assert len(list((out / "config").iterdir())) == 1  # written safely instead


@pytest.mark.parametrize("launch_file", ["parameter_node.launch.py", "precedence.launch.py"])
def test_real_analyze_leaves_source_tree_byte_identical(
    launch_file, packages_src, packages_root, tmp_path
):
    """Regression (a): a real `lf-analyze`-equivalent run over
    parameter_example must not modify anything under packages/src.
    """
    from colcon_lf.graph.hierarchical import build_system_graph

    pkg_dir = packages_src / "parameter_example"
    before = _hash_tree(pkg_dir)

    launch_path = pkg_dir / "launch" / launch_file
    graph = build_system_graph(launch_path, workspace_root=packages_root)
    write(graph, tmp_path / "analysis-out")

    after = _hash_tree(pkg_dir)
    changed = {p for p in before if before.get(p) != after.get(p)}
    assert before == after, (
        "colcon lf-analyze-equivalent run modified files under "
        f"packages/src/parameter_example: {changed or (set(before) ^ set(after))}"
    )


def test_real_analyze_system_json_has_no_absolute_parameter_file_path(
    packages_src, packages_root, tmp_path
):
    """Regression (c): system.json's parameter_files keys are workspace-relative."""
    import json

    from colcon_lf.graph.hierarchical import build_system_graph

    launch_path = packages_src / "parameter_example" / "launch" / "precedence.launch.py"
    graph = build_system_graph(launch_path, workspace_root=packages_root)
    out = write(graph, tmp_path / "analysis-out")

    system_json = json.loads((out / "system.json").read_text())
    workspace_str = str(packages_root.resolve())
    for key in system_json.get("parameter_files", {}):
        assert not key.startswith("/"), f"absolute parameter_files key: {key!r}"
        assert workspace_str not in key, f"machine-specific path leaked: {key!r}"

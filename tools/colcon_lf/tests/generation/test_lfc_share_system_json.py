# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The generated launch file must not depend on the cwd.

The standalone lfc launch file used to bake the relative
``lf-analysis/<pkg>/<stem>/system.json`` path. Launched from any directory
other than the colcon workspace, the runtime read no file and loaded 0
components. The package now installs system.json and exec_config.json and
the launch file finds them through the package share directory.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.verb.generate import _emit_lfc_artifacts

from .test_headerless_noncomponent_lfc import _system_json


@pytest.fixture
def emitted(tmp_path: Path, monkeypatch):
    main_cpp = tmp_path / "driver.cpp"
    main_cpp.write_text("int main() { return 0; }\n")
    sj_path = tmp_path / "lf-analysis" / "share_example" / "bringup" / "system.json"
    sj_path.parent.mkdir(parents=True)
    sj_path.write_text(json.dumps(_system_json(str(main_cpp))))
    (sj_path.parent / "exec_config.json").write_text('{"schema_version": 1}\n')
    graph = deserialize_system_graph(sj_path.read_text())

    from colcon_lf.generation.lf_emitter import emit_lf_system
    from colcon_lf.generation.lf_generator import generate_lf_system
    lf_system = generate_lf_system(graph, target_lfc=True)
    out_dir = sj_path.parent / "lf"
    out_dir.mkdir()
    (out_dir / f"{lf_system.name}.lf").write_text(emit_lf_system(lf_system))

    # Generate from another directory, as the bug needs a cwd mismatch.
    monkeypatch.chdir(tmp_path)
    pkg_name = _emit_lfc_artifacts(
        system_json=Path(os.path.relpath(sj_path, tmp_path)),
        out_dir=out_dir, lf_system=lf_system, graph=graph,
    )
    return sj_path, tmp_path / "lf-generated" / pkg_name, pkg_name


def test_package_carries_and_installs_both_json_files(emitted):
    sj_path, pkg_dir, _ = emitted
    assert json.loads((pkg_dir / "system.json").read_text()) == json.loads(sj_path.read_text())
    assert (pkg_dir / "exec_config.json").read_text() == '{"schema_version": 1}\n'
    cmake = (pkg_dir / "CMakeLists.txt").read_text()
    assert "install(FILES system.json" in cmake
    assert "install(FILES exec_config.json" in cmake


def test_launch_file_resolves_through_the_share_directory(emitted):
    _, pkg_dir, pkg_name = emitted
    (launch,) = (pkg_dir / "launch").glob("lf_*.launch.py")
    text = launch.read_text()
    assert "lf-analysis/" not in text, "launch file still bakes the cwd-relative path"
    assert f"get_package_share_directory('{pkg_name}'), 'system.json'" in text
    # exec_config.json is looked up next to the system.json it got.
    assert 'os.path.join(os.path.dirname(system_json), "exec_config.json")' in text

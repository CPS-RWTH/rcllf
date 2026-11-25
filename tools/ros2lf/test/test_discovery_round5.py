# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Review round 5: ros2 lf list shows federated and refused systems."""

from __future__ import annotations

import json
from types import SimpleNamespace

from ros2lf.api.discovery import find_system
from ros2lf.verb.list import ListVerb


def _system(ws, pkg, stem, **files):
    d = ws / "lf-analysis" / pkg / stem
    d.mkdir(parents=True)
    (d / "system.json").write_text(json.dumps(
        {"nodes": {"A": {}}, "launch_graph": {"all_nodes": [{"is_component": False}]}}))
    for name, body in files.items():
        (d / name.replace("__", ".")).write_text(body)
    return d


def test_federated_system_is_found_and_ready(tmp_path, capsys):
    """Before the fix: '- (run: colcon lf-generate)' for a built federated system."""
    _system(tmp_path, "pkg", "bringup",
            lfc_packages__txt="lf_generated_pkg_bringup_federated federated\n")
    base = "lf_generated_pkg_bringup_federated"
    launch = tmp_path / "lf-generated" / base / "launch"
    launch.mkdir(parents=True)
    (launch / "lf_bringup_federated.launch.py").write_text("")
    share = tmp_path / "install" / base / "share" / base / "launch"
    share.mkdir(parents=True)
    (share / "lf_bringup_federated.launch.py").write_text("")
    entry = find_system(tmp_path, "pkg", "bringup")
    assert entry.federated and entry.wrapper_built
    assert entry.generated_package == base
    assert entry.launch_filename == "lf_bringup_federated.launch.py"
    ListVerb().main(args=SimpleNamespace(workspace=str(tmp_path), package=None))
    assert "ready (federated)" in capsys.readouterr().out


def test_refused_system_shows_hazard_code(tmp_path, capsys):
    """Before the fix: '- (run: colcon lf-generate)' for a refused system."""
    _system(tmp_path, "pkg", "mf3", generate_status__json=json.dumps(
        {"status": "refused", "code": "RCLLF-H003", "reason": "x"}))
    ListVerb().main(args=SimpleNamespace(workspace=str(tmp_path), package=None))
    assert "refused (RCLLF-H003)" in capsys.readouterr().out

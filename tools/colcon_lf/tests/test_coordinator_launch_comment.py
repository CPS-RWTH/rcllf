# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The orchestrator launch names the coordination it was built for.

The comment used to say the tool generates no decentralized federations, in
every build, while ``--coordination decentralized`` is offered and admitted.
"""

from __future__ import annotations

import ast

import pytest

from colcon_lf.verb.generate import _emit_coordinator_package


def _launch_header(tmp_path, **kwargs) -> str:
    lf_file = tmp_path / "lf" / "Sys.lf"
    lf_file.parent.mkdir(parents=True)
    lf_file.write_text("", encoding="utf-8")
    _emit_coordinator_package(
        pkg_dir=tmp_path, pkg_name="lf_generated_sys_fed", system_stem="sys",
        federated_reactor_name="Sys", federate_names=["a"],
        federate_launch_stems={"a": "A"},
        federate_dep_info={"a": {"all_dep_packages": []}}, lf_file=lf_file, **kwargs,
    )
    (launch,) = (tmp_path / "launch").glob("*.launch.py")
    text = launch.read_text(encoding="utf-8")
    ast.parse(text)  # the interpolated comment must keep the file valid Python
    return "\n".join(line for line in text.splitlines() if line.startswith("#"))


@pytest.mark.parametrize("mode", ["centralized", "decentralized"])
def test_header_names_the_selected_coordination(tmp_path, mode):
    header = _launch_header(tmp_path, coordination=mode)
    assert "does\n# not generate decentralized" not in header
    assert "not generate decentralized" not in header.replace("\n# ", " ")
    assert f"--coordination {mode}" in header
    other = {"centralized": "decentralized", "decentralized": "centralized"}[mode]
    assert f"--coordination {other}" not in header


def test_coordination_is_required(tmp_path):
    """No default mode, so a caller cannot silently get
    the centralized header for a decentralized build (the symptom)."""
    with pytest.raises(TypeError, match="coordination"):
        _launch_header(tmp_path)


@pytest.mark.parametrize("mode", [None, "", "Decentralized", "central"])
def test_unknown_coordination_is_refused(tmp_path, mode):
    with pytest.raises(ValueError, match="unknown coordination"):
        _launch_header(tmp_path, coordination=mode)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""One generation path.

``colcon lf`` and ``colcon lf-generate`` always emit the lfc package,
``colcon lf-build`` builds ``lf_generated_*`` with ``colcon build
--packages-select``, and ``--target-lfc`` is an accepted no-op.
"""

from __future__ import annotations

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from colcon_lf.verb import build as build_verb
from colcon_lf.verb.build import BuildVerb, find_generated_packages


def _analysis(ws: Path, pkg: str, stem: str, lf_name: str) -> None:
    lf_dir = ws / "lf-analysis" / pkg / stem / "lf"
    lf_dir.mkdir(parents=True)
    (lf_dir.parent / "system.json").write_text("{}")
    (lf_dir / f"{lf_name}.lf").write_text("// stub")


def _generated(ws: Path, name: str) -> None:
    d = ws / "lf-generated" / name
    d.mkdir(parents=True)
    (d / "package.xml").write_text("<package/>")


def _marker(ws: Path, pkg: str, stem: str, line: str) -> None:
    d = ws / "lf-analysis" / pkg / stem
    d.mkdir(parents=True, exist_ok=True)
    (d / "system.json").write_text("{}")
    (d / "lfc_packages.txt").write_text(line + "\n")


def test_find_generated_packages_legacy_lf_name(tmp_path):
    _analysis(tmp_path, "parameter_example", "parameter_node", "ParameterNodeSystem")
    _generated(tmp_path, "lf_generated_parameter_example_parameternodesystem")
    assert find_generated_packages(tmp_path, None) == [
        "lf_generated_parameter_example_parameternodesystem",
    ]


def test_find_generated_packages_federated_only(tmp_path):
    """Fresh workspace after lf-generate --federate: no lf/ next to system.json."""
    _marker(tmp_path, "pubsub_example", "bringup", "lf_generated_bringup_federated federated")
    for name in ("lf_generated_bringup_federated",
                 "lf_generated_bringup_federated_publisher_node",
                 "lf_generated_bringup_federated_subscriber_node"):
        _generated(tmp_path, name)
    _generated(tmp_path, "lf_generated_other_federated")  # another system, not selected
    assert find_generated_packages(tmp_path, "pubsub_example") == [
        "lf_generated_bringup_federated",
        "lf_generated_bringup_federated_publisher_node",
        "lf_generated_bringup_federated_subscriber_node",
    ]


def test_single_marker_ignores_stale_federated_dirs(tmp_path):
    _marker(tmp_path, "pubsub_example", "bringup", "lf_generated_bringup single")
    _generated(tmp_path, "lf_generated_bringup")
    _generated(tmp_path, "lf_generated_bringup_federated")
    assert find_generated_packages(tmp_path, None) == ["lf_generated_bringup"]


def test_lf_build_package_filter_matching_nothing_fails(tmp_path, capsys):
    (tmp_path / "src").mkdir()
    (tmp_path / "lf-analysis").mkdir()
    (tmp_path / "lf-generated").mkdir()
    ctx = SimpleNamespace(args=SimpleNamespace(package="nope", workspace=str(tmp_path)))
    assert BuildVerb().main(context=ctx) == 1
    assert "no generated lf_generated_* package found for package 'nope'" in capsys.readouterr().err


def test_record_generated_packages_writes_marker(tmp_path):
    from colcon_lf.verb.generate import record_generated_packages

    sj = tmp_path / "system.json"
    sj.write_text("{}")
    record_generated_packages(sj, "lf_generated_bringup_federated", federated=True)
    assert (tmp_path / "lfc_packages.txt").read_text() == "lf_generated_bringup_federated federated\n"


def test_lf_build_runs_colcon_build_packages_select(tmp_path, monkeypatch):
    (tmp_path / "src").mkdir()
    _analysis(tmp_path, "pubsub_example", "bringup", "Bringup")
    _generated(tmp_path, "lf_generated_pubsub_example_bringup")
    calls = []

    def fake_run(cmd, cwd=None, **kw):
        calls.append((cmd, cwd))
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(build_verb.subprocess, "run", fake_run)
    # No lfc install here: the runtime gate reports and skips instead of
    # running patch_lfc_runtime.py through the faked subprocess.run.
    monkeypatch.setenv("LFC", str(tmp_path / "no-lfc" / "bin" / "lfc"))
    ctx = SimpleNamespace(args=SimpleNamespace(package="pubsub_example", workspace=str(tmp_path)))
    assert BuildVerb().main(context=ctx) == 0
    assert calls == [
        (["colcon", "build", "--packages-select", "lf_generated_pubsub_example_bringup", "--continue-on-error"], tmp_path),
    ]


def test_colcon_lf_generates_through_the_lfc_path(monkeypatch):
    """The all-in-one verb reaches _generate_one, which always emits the lfc package."""
    import colcon_lf.verb.generate as gen
    from colcon_lf.verb.lf import LfVerb

    seen = {}

    def fake_main(self, *, context):
        seen["args"] = context.args
        return 0

    monkeypatch.setattr(gen.GenerateVerb, "main", fake_main)
    args = SimpleNamespace(package="pubsub_example", workspace=None, after_delay=None)
    assert LfVerb()._run_generate(args) == 0
    # target_lfc is a deprecated no-op; generation cannot opt out any more.
    import inspect
    sig = inspect.signature(gen._generate_one)
    assert sig.parameters["target_lfc"].default is True
    src = inspect.getsource(gen._generate_one)
    assert "if generate_launch and not target_lfc" not in src
    assert "_emit_lfc_artifacts(" in src


def test_target_lfc_flag_prints_deprecation(capsys, monkeypatch, tmp_path):
    import colcon_lf.verb.generate as gen

    args = SimpleNamespace(
        target_lfc=True, federate=None, system_json=str(tmp_path / "missing.json"),
        package=None, output_dir=None, workspace=str(tmp_path), after_delay=None,
        launch=True, emit_federation_template=False, force=False, exec_config=None,
    )
    gen.GenerateVerb().main(context=SimpleNamespace(args=args))
    assert "--target-lfc is deprecated and has no effect" in capsys.readouterr().err


def test_stock_wrapper_emitters_are_gone():
    with pytest.raises(ImportError):
        import colcon_lf.generation.launch_emitter  # noqa: F401
    with pytest.raises(ImportError):
        import colcon_lf.generation.launch_generator  # noqa: F401

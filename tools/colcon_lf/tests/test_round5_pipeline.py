# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Review round 5, WP-D: not-a-system marker, generate status, lf-build keeps going,
colcon lf table."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("colcon_core.verb", reason="colcon_core not available")

from colcon_lf.verb import build as build_verb  # noqa: E402
from colcon_lf.verb import generate as generate_verb  # noqa: E402
from colcon_lf.verb.analyze import _discover_targets, _marked_launch_files  # noqa: E402


def _pkg(src: Path, name: str, launches: dict[str, str]) -> None:
    pkg = src / name
    (pkg / "launch").mkdir(parents=True)
    (pkg / "package.xml").write_text(f"<package><name>{name}</name></package>")
    for fname, body in launches.items():
        (pkg / "launch" / fname).write_text(body)


def test_marked_launch_file_is_not_a_target(tmp_path):
    """Before the fix a standalone helper launch file was analyzed as a system (H003 refusal)."""
    src = tmp_path / "src"
    _pkg(src, "p", {
        "bringup.launch.py": "x = 1\n",
        "helper.launch.py": "# rcllf: not-a-system\nx = 1\n",
        "other.launch.xml": "<launch>\n<!-- rcllf: not-a-system -->\n</launch>\n",
    })
    targets = [rel for _p, rel, _f in _discover_targets(src, "p", None)]
    assert targets == ["bringup.launch.py"], targets
    assert _marked_launch_files(src, "p") == [
        ("p", "helper.launch.py"), ("p", "other.launch.xml")]
    # An explicit --launch-file still selects a marked file.
    explicit = _discover_targets(src, "p", "helper.launch.py")
    assert [rel for _p, rel, _f in explicit] == ["helper.launch.py"]


def test_generate_records_refusal_with_hazard_code(tmp_path, monkeypatch):
    system_json = tmp_path / "system.json"
    system_json.write_text("{}")

    def fake(**kw):
        print("  Error generating LF: Node 'X': RCLLF-H003 sync_input_legacy", file=sys.stderr)
        return 1

    monkeypatch.setattr(generate_verb, "_generate_one", fake)
    assert generate_verb._generate_recorded(system_json=system_json) == 1
    record = json.loads((tmp_path / "generate_status.json").read_text())
    assert record["status"] == "refused" and record["code"] == "RCLLF-H003", record
    assert "RCLLF-H003" in record["reason"]
    monkeypatch.setattr(generate_verb, "_generate_one", lambda **kw: 0)
    assert generate_verb._generate_recorded(system_json=system_json) == 0
    assert json.loads((tmp_path / "generate_status.json").read_text())["status"] == "generated"


def _analysis(ws: Path, stem: str, status: str | dict | None, name: str | None = None) -> None:
    name = name or f"lf_generated_pkg_{stem}"
    d = ws / "lf-analysis" / "pkg" / stem
    d.mkdir(parents=True)
    (d / "system.json").write_text("{}")
    (d / "lfc_packages.txt").write_text(f"{name} single\n")
    if status is not None:
        rec = status if isinstance(status, dict) else {"status": status}
        (d / "generate_status.json").write_text(json.dumps(rec))
    g = ws / "lf-generated" / name
    g.mkdir(parents=True)
    (g / "package.xml").write_text("<package/>")


def test_lf_build_skips_refused_systems(tmp_path):
    _analysis(tmp_path, "ok", "generated")
    _analysis(tmp_path, "bad", "refused")
    assert build_verb.find_generated_packages(tmp_path, "pkg") == ["lf_generated_pkg_ok"]


# Recorded from a real colcon build (packages/log/latest_build/events.log).
_EVENTS = """\
[7.292963] (ros2_lf_msgs) JobEnded: {'identifier': 'ros2_lf_msgs', 'rc': 0}
[8.950964] (lf_generated_a) JobEnded: {'identifier': 'lf_generated_a', 'rc': 1}
[9.1] (lf_generated_b) JobEnded: {'identifier': 'lf_generated_b', 'rc': 0}
"""


def _events(ws: Path) -> None:
    log = ws / "log" / "latest_build"
    log.mkdir(parents=True)
    (log / "events.log").write_text(_EVENTS)


def test_failed_packages_reads_events_log(tmp_path):
    _events(tmp_path)
    assert build_verb._failed_packages(
        tmp_path, ["lf_generated_a", "lf_generated_b"]) == ["lf_generated_a"]


def test_lf_build_continues_past_a_failing_package_and_names_it(tmp_path, monkeypatch, capsys):
    (tmp_path / "src").mkdir()
    _analysis(tmp_path, "a", None, name="lf_generated_a")
    _analysis(tmp_path, "b", None, name="lf_generated_b")
    _events(tmp_path)
    cmds = []

    def fake_run(cmd, cwd=None, **kw):
        cmds.append(cmd)
        return subprocess.CompletedProcess(cmd, 1)

    monkeypatch.setattr(build_verb.subprocess, "run", fake_run)
    monkeypatch.setenv("LFC", str(tmp_path / "no-lfc" / "bin" / "lfc"))
    ctx = SimpleNamespace(args=SimpleNamespace(package=None, workspace=str(tmp_path)))
    assert build_verb.BuildVerb().main(context=ctx) == 1
    assert "--continue-on-error" in cmds[0], cmds
    err = capsys.readouterr().err
    assert "failed: lf_generated_a" in err and "built:  lf_generated_b" in err, err


def _lf_with_stubs(monkeypatch, calls, gen_rc, build_rc, ws):
    from colcon_lf.verb.lf import LfVerb

    verb = LfVerb()
    monkeypatch.setattr(verb, "_run_analyze", lambda a: calls.append("analyze") or 0)
    monkeypatch.setattr(verb, "_run_configure", lambda a: calls.append("configure") or 0)
    monkeypatch.setattr(verb, "_run_generate", lambda a: calls.append("generate") or gen_rc)
    monkeypatch.setattr(verb, "_run_build", lambda a: calls.append("build") or build_rc)
    return verb, SimpleNamespace(args=SimpleNamespace(package=None, workspace=str(ws)))


def test_colcon_lf_builds_generated_systems_after_a_refusal_and_exits_nonzero(
        tmp_path, monkeypatch, capsys):
    """Before the fix a refusal stopped the chain: 'Generation failed.' and nothing was built."""
    (tmp_path / "src").mkdir()
    _analysis(tmp_path, "ok", "generated")
    _analysis(tmp_path, "bad", {"status": "refused", "code": "RCLLF-H003", "reason": "r"})
    _pkg(tmp_path / "src", "pkg", {"h.launch.py": "# rcllf: not-a-system\n"})
    calls: list[str] = []
    verb, ctx = _lf_with_stubs(monkeypatch, calls, 1, 0, tmp_path)
    assert verb.main(context=ctx) == 1
    assert calls == ["analyze", "configure", "generate", "build"], calls
    out = capsys.readouterr().out
    assert "pkg/bad" in out and "refused" in out and "RCLLF-H003" in out, out
    assert "skipped (not-a-system)" in out, out


def test_colcon_lf_exit_zero_when_nothing_failed(tmp_path, monkeypatch):
    (tmp_path / "src").mkdir()
    verb, ctx = _lf_with_stubs(monkeypatch, [], 0, 0, tmp_path)
    assert verb.main(context=ctx) == 0


def test_colcon_lf_banner_precedes_the_failure_message():
    """O2-15: 'Analysis failed.' (stderr) appeared before the buffered 'Step 1/4' banner."""
    code = (
        "import sys; from types import SimpleNamespace as N;"
        "from colcon_lf.verb.lf import LfVerb;"
        "v=LfVerb(); v._run_analyze=lambda a: 1;"
        "sys.exit(v.main(context=N(args=N(package=None, workspace=None))))"
    )
    res = subprocess.run([sys.executable, "-c", code], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    assert res.returncode == 1
    assert res.stdout.index("Step 1/4") < res.stdout.index("Analysis failed."), res.stdout


def test_colcon_lf_forwards_federate_and_coordination(monkeypatch):
    import colcon_lf.verb.generate as gen
    from colcon_lf.verb.lf import LfVerb

    seen = {}
    monkeypatch.setattr(gen.GenerateVerb, "main",
                        lambda self, *, context: seen.update(vars(context.args)) or 0)
    args = SimpleNamespace(package="p", workspace=None, after_delay=None,
                           federate="per-node", coordination="decentralized")
    LfVerb()._run_generate(args)
    assert seen["federate"] == "per-node" and seen["coordination"] == "decentralized", seen


def test_colcon_lf_declares_federate_and_coordination_flags():
    import argparse

    from colcon_lf.verb.lf import LfVerb

    parser = argparse.ArgumentParser()
    LfVerb().add_arguments(parser=parser)
    ns = parser.parse_args(["--federate", "per-process", "--coordination", "centralized"])
    assert (ns.federate, ns.coordination) == ("per-process", "centralized")
    with pytest.raises(SystemExit):
        parser.parse_args(["--coordination", "bogus"])

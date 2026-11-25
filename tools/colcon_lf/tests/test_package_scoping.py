# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for ``--package`` scoping on lf-configure / lf-generate.

The resolver is pure filesystem logic over the ``lf-analysis/{package}/
{launch_stem}/system.json`` layout that ``verb/analyze.py``'s ``_output_path``
writes, so everything here runs on a bare host with ``tmp_path`` fixtures — no
libclang, no ROS headers.  The end-to-end CLI behaviour is pinned separately by
``tests/tutorials/test_tutorial_commands.py`` (dev-container only).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest

pytest.importorskip("colcon_core.verb", reason="colcon_core not available")


def _make_analysis_tree(root: Path, layout: dict[str, list[str]]) -> Path:
    """Build an ``lf-analysis/`` tree: ``{package: [launch_stem, ...]}``.

    Also creates the sibling ``src/`` marker so ``_resolve_workspace`` accepts
    *root* as a colcon workspace.
    """
    (root / "src").mkdir(parents=True, exist_ok=True)
    base = root / "lf-analysis"
    for package, stems in layout.items():
        for stem in stems:
            d = base / package / stem
            d.mkdir(parents=True, exist_ok=True)
            (d / "system.json").write_text("{}", encoding="utf-8")
    return base


class TestFindPackageSystemJsons:
    def test_single_launch_stem(self, tmp_path):
        from colcon_lf.verb.configure import find_package_system_jsons

        base = _make_analysis_tree(tmp_path, {"pubsub_example": ["bringup"]})
        found = find_package_system_jsons(base, "pubsub_example")
        assert found == [base / "pubsub_example" / "bringup" / "system.json"]

    def test_multiple_launch_stems_all_returned_sorted(self, tmp_path):
        from colcon_lf.verb.configure import find_package_system_jsons

        base = _make_analysis_tree(
            tmp_path, {"launch_features_example": ["03_conditions", "01_basic", "02_args"]}
        )
        found = find_package_system_jsons(base, "launch_features_example")
        assert [p.parent.name for p in found] == ["01_basic", "02_args", "03_conditions"]

    def test_unknown_package_is_empty(self, tmp_path):
        from colcon_lf.verb.configure import find_package_system_jsons

        base = _make_analysis_tree(tmp_path, {"pubsub_example": ["bringup"]})
        assert find_package_system_jsons(base, "no_such_package") == []

    def test_package_dir_without_system_json_is_empty(self, tmp_path):
        """An analysis dir that exists but holds no system.json is still 'no analyses'."""
        from colcon_lf.verb.configure import find_package_system_jsons

        base = _make_analysis_tree(tmp_path, {"pubsub_example": ["bringup"]})
        (base / "half_analyzed" / "bringup").mkdir(parents=True)
        assert find_package_system_jsons(base, "half_analyzed") == []

    def test_does_not_recurse_past_launch_stem(self, tmp_path):
        """Only ``{package}/*/system.json`` — never a deeper nested artifact.

        ``lf-generate`` writes ``lf/`` and ``include/`` subdirectories
        next to the system.json; a deep rglob would pick up stray copies.
        """
        from colcon_lf.verb.configure import find_package_system_jsons

        base = _make_analysis_tree(tmp_path, {"pubsub_example": ["bringup"]})
        nested = base / "pubsub_example" / "bringup" / "wrapper" / "generated"
        nested.mkdir(parents=True)
        (nested / "system.json").write_text("{}", encoding="utf-8")

        found = find_package_system_jsons(base, "pubsub_example")
        assert found == [base / "pubsub_example" / "bringup" / "system.json"]


class TestFindSystemJsonsPrecedence:
    """``--system-json`` > ``--package`` > discover-everything."""

    def test_package_scopes_discovery(self, tmp_path):
        from colcon_lf.verb.configure import _find_system_jsons

        _make_analysis_tree(
            tmp_path, {"pkg_a": ["bringup"], "pkg_b": ["bringup", "other"]}
        )
        found = _find_system_jsons(None, str(tmp_path), "pkg_b")
        assert len(found) == 2
        assert all(p.parent.parent.name == "pkg_b" for p in found)

    def test_no_package_discovers_everything(self, tmp_path):
        from colcon_lf.verb.configure import _find_system_jsons

        _make_analysis_tree(
            tmp_path, {"pkg_a": ["bringup"], "pkg_b": ["bringup", "other"]}
        )
        assert len(_find_system_jsons(None, str(tmp_path), None)) == 3

    def test_system_json_wins_over_package(self, tmp_path):
        from colcon_lf.verb.configure import _find_system_jsons

        base = _make_analysis_tree(
            tmp_path, {"pkg_a": ["bringup"], "pkg_b": ["bringup"]}
        )
        exact = base / "pkg_a" / "bringup" / "system.json"
        found = _find_system_jsons(str(exact), str(tmp_path), "pkg_b")
        assert found == [exact]

    def test_unknown_package_returns_empty_not_everything(self, tmp_path):
        """Regression guard: a typo'd package must never silently fan out to all."""
        from colcon_lf.verb.configure import _find_system_jsons

        _make_analysis_tree(tmp_path, {"pkg_a": ["bringup"], "pkg_b": ["bringup"]})
        assert _find_system_jsons(None, str(tmp_path), "pkg_typo") == []

    def test_generate_alias_agrees_with_configure(self, tmp_path):
        """verb/generate.py's alias must resolve identically (one rule, two names)."""
        from colcon_lf.verb.configure import _find_system_jsons as cfg_find
        from colcon_lf.verb.generate import _find_system_jsons as gen_find

        _make_analysis_tree(tmp_path, {"pkg_a": ["bringup", "other"]})
        assert gen_find(None, str(tmp_path), "pkg_a") == cfg_find(
            None, str(tmp_path), "pkg_a"
        )


class TestPackageScopeError:
    def test_names_the_package_and_the_fix(self, tmp_path):
        from colcon_lf.verb.configure import package_scope_error

        msg = package_scope_error("pubsub_example", tmp_path)
        assert "pubsub_example" in msg
        assert "colcon lf-analyze --package pubsub_example" in msg

    def test_works_without_a_resolved_workspace(self):
        from colcon_lf.verb.configure import package_scope_error

        msg = package_scope_error("pubsub_example", None)
        assert "lf-analysis/pubsub_example" in msg


class TestReportPackageScope:
    """The fan-out notice must always show what was matched (user decision)."""

    def test_silent_without_package(self, tmp_path, capsys):
        from colcon_lf.verb.configure import report_package_scope

        report_package_scope(None, [tmp_path / "a" / "system.json"])
        assert capsys.readouterr().out == ""

    def test_singular_for_one_analysis(self, tmp_path, capsys):
        from colcon_lf.verb.configure import report_package_scope

        report_package_scope("pkg_a", [tmp_path / "pkg_a" / "bringup" / "system.json"])
        out = capsys.readouterr().out
        assert "pkg_a" in out
        assert "1 analysis" in out
        assert "bringup" in out

    def test_plural_and_lists_stems(self, tmp_path, capsys):
        from colcon_lf.verb.configure import report_package_scope

        report_package_scope(
            "pkg_b",
            [
                tmp_path / "pkg_b" / "bringup" / "system.json",
                tmp_path / "pkg_b" / "other" / "system.json",
            ],
        )
        out = capsys.readouterr().out
        assert "2 analyses" in out
        assert "bringup" in out and "other" in out


class TestArgparseSurface:
    """``--package`` and ``--system-json`` must be mutually exclusive."""

    @pytest.mark.parametrize("verb_path", ["configure.ConfigureVerb", "generate.GenerateVerb"])
    def test_both_flags_accepted_separately(self, verb_path):
        import importlib

        mod_name, cls_name = verb_path.split(".")
        mod = importlib.import_module(f"colcon_lf.verb.{mod_name}")
        verb = getattr(mod, cls_name)()

        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)

        assert parser.parse_args(["--package", "pkg_a"]).package == "pkg_a"
        assert parser.parse_args(["--system-json", "x.json"]).system_json == "x.json"
        bare = parser.parse_args([])
        assert bare.package is None and bare.system_json is None

    @pytest.mark.parametrize("verb_path", ["configure.ConfigureVerb", "generate.GenerateVerb"])
    def test_combining_flags_is_an_error(self, verb_path):
        import importlib

        mod_name, cls_name = verb_path.split(".")
        mod = importlib.import_module(f"colcon_lf.verb.{mod_name}")
        verb = getattr(mod, cls_name)()

        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)

        with pytest.raises(SystemExit):
            parser.parse_args(["--package", "pkg_a", "--system-json", "x.json"])


class TestChainVerbNamespace:
    """``colcon lf``'s FakeContext shim must satisfy every arg the verbs read.

    The shim bypasses argparse, so a missing attribute is an AttributeError at
    runtime rather than a defaulted flag.  An older ``_run_generate`` omitted
    ``federate``/``target_lfc``/``force``/``emit_federation_template``/
    ``exec_config`` — and ``args.federate`` is the first line
    ``GenerateVerb.main`` touches, so the whole chain raised.
    """

    # NOTE: read the real verb class's source *before* monkeypatching the
    # module attribute to the recorder — otherwise `inspect.getsource` picks up
    # the recorder's trivial `main`, `reads` comes back empty, and the
    # assertions below pass vacuously (caught while proving these tests
    # actually fail against the earlier namespace handling).
    @staticmethod
    def _args_read_by_main(verb_cls) -> set[str]:
        import inspect
        import re

        return set(re.findall(r"args\.(\w+)", inspect.getsource(verb_cls.main)))

    @staticmethod
    def _namespace_for(runner_name: str):
        """Call ``LfVerb._run_<stage>`` with a stub that records the namespace."""
        from colcon_lf.verb.lf import LfVerb

        captured = {}

        class _Recorder:
            def main(self, *, context):
                captured["args"] = context.args
                return 0

        verb = LfVerb()
        chain_args = argparse.Namespace(
            package="pkg_a", workspace=None, after_delay="1 msec"
        )
        return verb, chain_args, captured, _Recorder

    def test_generate_namespace_is_complete(self, monkeypatch):
        from colcon_lf.verb import generate as generate_mod

        reads = self._args_read_by_main(generate_mod.GenerateVerb)
        assert "federate" in reads, "sanity: main() should read args.federate"

        verb, chain_args, captured, recorder = self._namespace_for("generate")
        monkeypatch.setattr(generate_mod, "GenerateVerb", recorder)

        assert verb._run_generate(chain_args) == 0
        missing = reads - set(vars(captured["args"]))
        assert not missing, f"FakeContext namespace missing: {sorted(missing)}"

    def test_generate_namespace_forwards_package(self, monkeypatch):
        from colcon_lf.verb import generate as generate_mod

        verb, chain_args, captured, recorder = self._namespace_for("generate")
        monkeypatch.setattr(generate_mod, "GenerateVerb", recorder)

        verb._run_generate(chain_args)
        assert captured["args"].package == "pkg_a"

    def test_analyze_namespace_is_complete(self, monkeypatch):
        from colcon_lf.verb import analyze as analyze_mod

        reads = self._args_read_by_main(analyze_mod.AnalyzeVerb)
        verb, chain_args, captured, recorder = self._namespace_for("analyze")
        monkeypatch.setattr(analyze_mod, "AnalyzeVerb", recorder)

        assert verb._run_analyze(chain_args) == 0
        missing = reads - set(vars(captured["args"]))
        assert not missing, f"FakeContext namespace missing: {sorted(missing)}"

    def test_build_namespace_is_complete(self, monkeypatch):
        from colcon_lf.verb import build as build_mod

        reads = self._args_read_by_main(build_mod.BuildVerb)
        verb, chain_args, captured, recorder = self._namespace_for("build")
        monkeypatch.setattr(build_mod, "BuildVerb", recorder)

        assert verb._run_build(chain_args) == 0
        missing = reads - set(vars(captured["args"]))
        assert not missing, f"FakeContext namespace missing: {sorted(missing)}"

    def test_configure_stage_scopes_to_package(self, tmp_path, capsys):
        """``colcon lf --package X`` must configure only X's analyses."""
        from colcon_lf.verb.lf import LfVerb

        _make_analysis_tree(
            tmp_path, {"pkg_a": ["bringup"], "pkg_b": ["bringup", "other"]}
        )
        seen: list[Path] = []

        import colcon_lf.verb.configure as configure_mod

        real_configure_one = configure_mod.configure_one

        def _spy(*, system_json, **kwargs):
            seen.append(system_json)
            return 0

        configure_mod.configure_one = _spy
        try:
            verb = LfVerb()
            rc = verb._run_configure(
                argparse.Namespace(package="pkg_b", workspace=str(tmp_path))
            )
        finally:
            configure_mod.configure_one = real_configure_one

        assert rc == 0
        assert len(seen) == 2
        assert all(p.parent.parent.name == "pkg_b" for p in seen)
        assert "pkg_b" in capsys.readouterr().out


def test_every_verb_shares_one_workspace_resolver():
    """All verbs resolve the workspace with one function, so they cannot drift.

    Configure, analyze, build and generate share one ``_resolve_workspace``.
    """
    from colcon_lf.verb import analyze, build, configure, generate
    from colcon_lf.verb.workspace import resolve_workspace

    copies = {m.__name__: m._resolve_workspace for m in (analyze, build, configure, generate)}
    assert all(fn is resolve_workspace for fn in copies.values()), copies


def test_shared_workspace_resolver_order(tmp_path, monkeypatch):
    """``./packages`` wins over ``.``, and a missing --workspace dir is None."""
    from colcon_lf.verb.workspace import resolve_workspace

    (tmp_path / "src").mkdir()
    (tmp_path / "packages" / "src").mkdir(parents=True)
    monkeypatch.chdir(tmp_path)
    assert resolve_workspace(None) == tmp_path / "packages"
    assert resolve_workspace(str(tmp_path)) == tmp_path
    assert resolve_workspace(str(tmp_path / "missing")) is None

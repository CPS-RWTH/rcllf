# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.verb.analyze (colcon lf-analyze).

Phase 8 — TDD assertions:
1. Verb instantiates and has add_arguments/main methods
2. Arguments are all optional (package, launch-file, output-dir)
3. Default output goes to lf-analysis/{package}/{launch_stem}/
4. Explicit --output-dir uses {output_dir}/{package}/{launch_stem}/
5. Missing launch file returns non-zero exit code
6. Integration: single launch file produces expected output
7. Integration: package-only discovers all launch files
8. Integration: --format json produces no viz/ directory
9. Integration: --format svg produces viz files

Unit tests (no Docker): instantiation, argument parsing, output path logic
Integration tests (Docker): end-to-end pipeline execution
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

colcon_core = pytest.importorskip(
    "colcon_core.verb", reason="colcon_core not available"
)


# ---------------------------------------------------------------------------
# Unit tests (no Docker)
# ---------------------------------------------------------------------------

class TestAnalyzeVerbStructure:
    def test_instantiation(self):
        from colcon_lf.verb.analyze import AnalyzeVerb

        verb = AnalyzeVerb()
        assert hasattr(verb, "add_arguments")
        assert hasattr(verb, "main")

    def test_add_arguments_all_optional(self):
        from colcon_lf.verb.analyze import AnalyzeVerb

        verb = AnalyzeVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)
        # No required args — parse with nothing
        args = parser.parse_args([])
        assert args.package is None
        assert args.launch_file is None
        assert args.output_dir is None
        assert args.format == "all"

    def test_add_arguments_explicit(self):
        from colcon_lf.verb.analyze import AnalyzeVerb

        verb = AnalyzeVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)
        args = parser.parse_args([
            "--package", "pubsub_example",
            "--launch-file", "bringup.launch.py",
            "--workspace", "/workspace/packages",
            "--output-dir", "/tmp/out",
            "--format", "json",
        ])
        assert args.package == "pubsub_example"
        assert args.launch_file == "bringup.launch.py"
        assert args.workspace == "/workspace/packages"
        assert args.output_dir == "/tmp/out"
        assert args.format == "json"

    def test_format_choices(self):
        from colcon_lf.verb.analyze import AnalyzeVerb

        verb = AnalyzeVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)
        for fmt in ["json", "pdf", "svg", "all"]:
            args = parser.parse_args(["--format", fmt])
            assert args.format == fmt

    def test_output_path_default(self):
        from colcon_lf.verb.analyze import _output_path

        result = _output_path(
            workspace=Path("/workspace/packages"),
            package="pubsub_example",
            launch_stem="bringup",
            output_dir=None,
        )
        assert result == Path("/workspace/packages/lf-analysis/pubsub_example/bringup")

    def test_output_path_explicit(self):
        from colcon_lf.verb.analyze import _output_path

        result = _output_path(
            workspace=Path("/workspace/packages"),
            package="pubsub_example",
            launch_stem="bringup",
            output_dir="/tmp/analysis",
        )
        assert result == Path("/tmp/analysis/pubsub_example/bringup")

    def test_discover_single_target(self, tmp_path):
        from colcon_lf.verb.analyze import _discover_targets

        pkg_dir = tmp_path / "pkg"
        launch_dir = pkg_dir / "launch"
        launch_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").touch()
        (launch_dir / "test.launch.py").touch()

        targets = _discover_targets(tmp_path, "pkg", "test.launch.py")
        assert len(targets) == 1
        assert targets[0][0] == "pkg"
        assert targets[0][1] == "test.launch.py"

    def test_discover_package_all_launch_files(self, tmp_path):
        from colcon_lf.verb.analyze import _discover_targets

        pkg_dir = tmp_path / "pkg"
        launch_dir = pkg_dir / "launch"
        launch_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").touch()
        (launch_dir / "a.launch.py").touch()
        (launch_dir / "b.launch.py").touch()
        (launch_dir / "helper_utils.py").touch()

        targets = _discover_targets(tmp_path, "pkg", None)
        assert len(targets) == 2
        names = [t[1] for t in targets]
        assert "a.launch.py" in names
        assert "b.launch.py" in names

    def test_discover_all_packages(self, tmp_path):
        from colcon_lf.verb.analyze import _discover_targets

        for pkg in ["alpha", "beta"]:
            pkg_dir = tmp_path / pkg
            d = pkg_dir / "launch"
            d.mkdir(parents=True)
            (pkg_dir / "package.xml").touch()
            (d / "main.launch.py").touch()
        # Package without launch dir should be skipped
        (tmp_path / "gamma").mkdir()

        targets = _discover_targets(tmp_path, None, None)
        assert len(targets) == 2
        pkgs = [t[0] for t in targets]
        assert "alpha" in pkgs
        assert "beta" in pkgs

    def test_package_name_comes_from_package_xml(self, tmp_path):
        # ros2/examples: directory minimal_publisher, package
        # examples_rclcpp_minimal_publisher. The launch file names the package.
        from colcon_lf.verb.analyze import _discover_targets, _find_packages

        pkg_dir = tmp_path / "rclcpp" / "topics" / "minimal_publisher"
        (pkg_dir / "launch").mkdir(parents=True)
        (pkg_dir / "package.xml").write_text(
            '<?xml version="1.0"?>\n<package format="3">\n'
            "  <name>examples_rclcpp_minimal_publisher</name>\n</package>\n"
        )
        (pkg_dir / "launch" / "pub.launch.py").touch()

        assert _find_packages(tmp_path) == {
            "examples_rclcpp_minimal_publisher": pkg_dir
        }
        targets = _discover_targets(tmp_path, "examples_rclcpp_minimal_publisher", None)
        assert [(t[0], t[1]) for t in targets] == [
            ("examples_rclcpp_minimal_publisher", "pub.launch.py")
        ]

    def test_source_and_launch_package_names_come_from_package_xml(self, tmp_path):
        # The node's package in system.json becomes the generated package's
        # find_package(); a directory name there fails the build.
        from colcon_lf.launch_analysis.parser import _guess_package as launch_pkg
        from colcon_lf.node_analysis.clang_parser import _guess_package as source_pkg

        pkg_dir = tmp_path / "minimal_publisher"
        (pkg_dir / "launch" / "topics").mkdir(parents=True)
        (pkg_dir / "package.xml").write_text(
            "<package><name>examples_rclcpp_minimal_publisher</name></package>"
        )
        assert source_pkg(pkg_dir / "member_function.cpp") == (
            "examples_rclcpp_minimal_publisher"
        )
        assert launch_pkg(pkg_dir / "launch" / "topics" / "pub.launch.py") == (
            "examples_rclcpp_minimal_publisher"
        )
        (pkg_dir / "package.xml").write_text("")
        assert source_pkg(pkg_dir / "member_function.cpp") == "minimal_publisher"

    def test_compile_deps_found_by_package_name(self, tmp_path):
        # The generated package compiles the node source, so it needs the
        # node package's <depend>s (std_msgs) even when the directory differs.
        from colcon_lf.generation.package_deps import _read_package_deps

        pkg_dir = tmp_path / "rclcpp" / "topics" / "minimal_publisher"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").write_text(
            "<package><name>examples_rclcpp_minimal_publisher</name>"
            "<build_depend>rclcpp</build_depend><build_depend>std_msgs</build_depend>"
            "<buildtool_depend>ament_cmake</buildtool_depend></package>"
        )
        assert _read_package_deps(tmp_path, "examples_rclcpp_minimal_publisher") == [
            "rclcpp", "std_msgs"
        ]
        assert _read_package_deps(tmp_path, "no_such_package") == []

    def test_launch_files_in_launch_subdirectories(self, tmp_path):
        # ros2/demos demo_nodes_cpp keeps its launch files in launch/topics/.
        from colcon_lf.verb.analyze import _discover_targets

        pkg_dir = tmp_path / "demo_nodes_cpp"
        (pkg_dir / "launch" / "topics").mkdir(parents=True)
        (pkg_dir / "package.xml").touch()
        nested = pkg_dir / "launch" / "topics" / "talker_listener_launch.py"
        nested.touch()

        expected = [("demo_nodes_cpp", "topics/talker_listener_launch.py", nested)]
        for package in ("demo_nodes_cpp", None):
            assert _discover_targets(tmp_path, package, None) == expected
        for name in ("talker_listener_launch.py", "topics/talker_listener_launch.py"):
            assert _discover_targets(tmp_path, "demo_nodes_cpp", name) == expected


class TestPackageResolver:
    """Verify _PackageResolver finds packages in nested source trees."""

    def test_finds_nested_package(self, tmp_path):
        pkg_dir = tmp_path / "src" / "group" / "subgroup" / "my_pkg"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").write_text(
            '<package><name>my_pkg</name></package>'
        )

        from colcon_lf.launch_analysis.parser import _PackageResolver
        resolver = _PackageResolver(tmp_path)
        result = resolver.find_package_share("my_pkg")
        assert result is not None
        assert result == pkg_dir

    def test_install_takes_priority(self, tmp_path):
        install_share = tmp_path / "install" / "my_pkg" / "share" / "my_pkg"
        install_share.mkdir(parents=True)
        pkg_dir = tmp_path / "src" / "nested" / "my_pkg"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").write_text(
            '<package><name>my_pkg</name></package>'
        )

        from colcon_lf.launch_analysis.parser import _PackageResolver
        resolver = _PackageResolver(tmp_path)
        result = resolver.find_package_share("my_pkg")
        assert result == install_share

    def test_flat_src_still_works(self, tmp_path):
        pkg_dir = tmp_path / "src" / "flat_pkg"
        pkg_dir.mkdir(parents=True)

        from colcon_lf.launch_analysis.parser import _PackageResolver
        resolver = _PackageResolver(tmp_path)
        result = resolver.find_package_share("flat_pkg")
        assert result == pkg_dir

    def test_unknown_package_returns_none(self, tmp_path):
        (tmp_path / "src").mkdir()

        from colcon_lf.launch_analysis.parser import _PackageResolver
        resolver = _PackageResolver(tmp_path)
        assert resolver.find_package_share("nonexistent") is None




class TestLfAnalysisColconIgnore:
    """Duplicate crawl (RESOLVES lf-analysis-dup-crawl): lf-bench builds its lfc copy under
    ``lf-analysis/lf-generated/`` with the name ``colcon lf`` gives the package
    under ``lf-generated/``. Without ``lf-analysis/COLCON_IGNORE`` a workspace
    crawl finds the name twice and colcon refuses the whole build.
    """

    def test_output_path_marks_lf_analysis(self, tmp_path):
        from colcon_lf.verb.analyze import _output_path

        out = _output_path(tmp_path, "sentry_pipeline", "sentry", None)
        assert out == tmp_path / "lf-analysis" / "sentry_pipeline" / "sentry"
        assert (tmp_path / "lf-analysis" / "COLCON_IGNORE").is_file(), (
            "lf-analysis/ has no COLCON_IGNORE: colcon finds lf-bench's lfc copy "
            "a second time (Duplicate package names)"
        )

    def test_output_dir_is_marked_too(self, tmp_path):
        from colcon_lf.verb.analyze import _output_path

        _output_path(tmp_path, "p", "s", str(tmp_path / "custom"))
        assert (tmp_path / "custom" / "COLCON_IGNORE").is_file()

    def test_existing_marker_is_kept(self, tmp_path):
        from colcon_lf.verb.analyze import _output_path

        (tmp_path / "lf-analysis").mkdir()
        (tmp_path / "lf-analysis" / "COLCON_IGNORE").write_text("mine\n")
        _output_path(tmp_path, "p", "s", None)
        assert (tmp_path / "lf-analysis" / "COLCON_IGNORE").read_text() == "mine\n"


class TestOutputRootOwnership:
    """The COLCON_IGNORE marker goes only into a directory the tool owns. An
    ``--output-dir`` that is the workspace, an ancestor of it, or holds a
    package the tool did not generate would hide real packages from colcon.
    """

    @staticmethod
    def _package(dirpath, name):
        dirpath.mkdir(parents=True, exist_ok=True)
        (dirpath / "package.xml").write_text(
            f"<package format=\"3\"><name>{name}</name></package>\n")

    def test_workspace_root_is_refused(self, tmp_path):
        from colcon_lf.verb.analyze import OutputRootError, _output_path

        with pytest.raises(OutputRootError, match="is the workspace"):
            _output_path(tmp_path, "p", "s", str(tmp_path))
        assert not (tmp_path / "COLCON_IGNORE").exists()

    def test_workspace_ancestor_is_refused(self, tmp_path):
        from colcon_lf.verb.analyze import OutputRootError, _output_path

        workspace = tmp_path / "ws" / "packages"
        workspace.mkdir(parents=True)
        with pytest.raises(OutputRootError, match="contains the workspace"):
            _output_path(workspace, "p", "s", str(tmp_path / "ws"))
        assert not (tmp_path / "ws" / "COLCON_IGNORE").exists()

    def test_dir_with_a_foreign_package_is_refused(self, tmp_path):
        from colcon_lf.verb.analyze import OutputRootError, _output_path

        out = tmp_path / "elsewhere"
        self._package(out / "src" / "my_pkg", "my_pkg")
        with pytest.raises(OutputRootError, match="my_pkg"):
            _output_path(tmp_path / "ws", "p", "s", str(out))
        assert not (out / "COLCON_IGNORE").exists()

    def test_name_in_a_comment_does_not_hide_a_foreign_package(self, tmp_path):
        from colcon_lf.verb.analyze import OutputRootError, _output_path

        out = tmp_path / "elsewhere" / "my_pkg"
        out.mkdir(parents=True)
        (out / "package.xml").write_text(
            "<package format=\"3\"><!-- <name>lf_generated_x</name> -->"
            "<name>my_pkg</name></package>\n")
        with pytest.raises(OutputRootError, match="my_pkg"):
            _output_path(tmp_path / "ws", "p", "s", str(tmp_path / "elsewhere"))
        assert not (tmp_path / "elsewhere" / "COLCON_IGNORE").exists()

    @pytest.mark.parametrize("text, reason", [
        ("<package><name>lf_generated_x</name>", "cannot be parsed"),
        ("<package><description>x</description></package>", "has no name"),
    ])
    def test_unreadable_manifest_is_refused(self, tmp_path, text, reason):
        from colcon_lf.verb.analyze import OutputRootError, _output_path

        out = tmp_path / "elsewhere" / "pkg"
        out.mkdir(parents=True)
        (out / "package.xml").write_text(text)
        with pytest.raises(OutputRootError, match=reason):
            _output_path(tmp_path / "ws", "p", "s", str(tmp_path / "elsewhere"))
        assert not (tmp_path / "elsewhere" / "COLCON_IGNORE").exists()

    def test_dir_with_only_generated_packages_is_accepted(self, tmp_path):
        from colcon_lf.verb.analyze import _output_path

        self._package(tmp_path / "lf-analysis" / "lf-generated" / "g",
                      "lf_generated_pubsub_example_bringup")
        self._package(tmp_path / "lf-analysis" / "old", "lf_wrapper_bringup")
        _output_path(tmp_path, "p", "s", None)
        assert (tmp_path / "lf-analysis" / "COLCON_IGNORE").is_file()


_EMPTY_LAUNCH = (
    "from launch import LaunchDescription\n\n\n"
    "def generate_launch_description():\n"
    "    return LaunchDescription([])\n"
)


class TestLaunchFileSelection:
    """Nested launch files keep distinct identities or are refused by name.

    ``dup_pkg`` has ``launch/left/demo.launch.py`` and
    ``launch/right/demo.launch.py`` (both would write ``dup_pkg/demo/``) and a
    unique ``launch/extra/solo.launch.py``.
    """

    @staticmethod
    def _workspace(tmp_path: Path) -> Path:
        ws = tmp_path / "ws"
        pkg = ws / "src" / "dup_dir"  # directory name differs from <name>
        for rel in ("left/demo.launch.py", "right/demo.launch.py",
                    "extra/solo.launch.py"):
            (pkg / "launch" / rel).parent.mkdir(parents=True, exist_ok=True)
            (pkg / "launch" / rel).write_text(_EMPTY_LAUNCH)
        (pkg / "package.xml").write_text("<package><name>dup_pkg</name></package>")
        return ws

    @staticmethod
    def _analyze(ws: Path, launch_file, launch_only: bool) -> int:
        from colcon_lf.verb.analyze import AnalyzeVerb

        args = argparse.Namespace(
            package="dup_pkg", launch_file=launch_file, workspace=str(ws),
            output_dir=str(ws / "out"), format="json",
            viz_only=False, launch_only=launch_only,
        )
        return AnalyzeVerb().main(context=SimpleNamespace(args=args))

    @staticmethod
    def _written(ws: Path) -> list[str]:
        out = ws / "out"
        return sorted(p.relative_to(out).as_posix() for p in out.rglob("*.json")) \
            if out.exists() else []

    @pytest.mark.parametrize("launch_only", [False, True])
    def test_duplicate_basenames_are_refused_before_any_output(
            self, tmp_path, capsys, launch_only):
        ws = self._workspace(tmp_path)
        assert self._analyze(ws, None, launch_only) == 1
        err = capsys.readouterr().err
        assert ("Error: launch files launch/left/demo.launch.py and "
                "launch/right/demo.launch.py of package 'dup_pkg' would write the "
                "same output directory dup_pkg/demo/") in err
        assert "Nothing was written" in err
        assert not (ws / "out").exists()

    def test_ambiguous_bare_name_names_both_files(self, tmp_path, capsys):
        ws = self._workspace(tmp_path)
        assert self._analyze(ws, "demo.launch.py", True) == 1
        err = capsys.readouterr().err
        assert ("launch file name 'demo.launch.py' is ambiguous in package 'dup_pkg': "
                "launch/left/demo.launch.py, launch/right/demo.launch.py") in err
        assert not (ws / "out").exists()

    def test_mistyped_relative_path_does_not_fall_back(self, tmp_path, capsys):
        # Before the fix, typo/solo.launch.py fell back to the unique
        # extra/solo.launch.py by its basename and wrote dup_pkg/solo/.
        ws = self._workspace(tmp_path)
        assert self._analyze(ws, "typo/solo.launch.py", True) == 1
        assert self._written(ws) == []
        err = capsys.readouterr().err
        assert ("Error: launch file 'typo/solo.launch.py' not found in package "
                "'dup_pkg': no file ") in err
        assert "typo/solo.launch.py (a path is taken relative" in err

        from colcon_lf.verb.analyze import LaunchSelectionError, _discover_targets

        with pytest.raises(LaunchSelectionError, match="'typo/solo.launch.py' not found"):
            _discover_targets(ws / "src", "dup_pkg", "typo/solo.launch.py")
        # A path that leaves launch/ is not a launch file of the package.
        with pytest.raises(LaunchSelectionError, match="not found"):
            _discover_targets(ws / "src", "dup_pkg", "../launch/extra/solo.launch.py")

    def test_valid_relative_path_selects_exactly_that_file(self, tmp_path, capsys):
        from colcon_lf.verb.analyze import _discover_targets

        ws = self._workspace(tmp_path)
        left = ws / "src" / "dup_dir" / "launch" / "left" / "demo.launch.py"
        assert _discover_targets(ws / "src", "dup_pkg", "left/demo.launch.py") == [
            ("dup_pkg", "left/demo.launch.py", left)]
        assert self._analyze(ws, "left/demo.launch.py", True) == 0
        assert "--- dup_pkg/left/demo.launch.py ---" in capsys.readouterr().out
        assert self._written(ws) == ["dup_pkg/demo/launch/launch_graph.json"]
        graph = (ws / "out" / "dup_pkg" / "demo" / "launch" / "launch_graph.json").read_text()
        assert "left/demo.launch.py" in graph and "right/" not in graph

    def test_unique_bare_name_finds_the_file_in_a_subdirectory(self, tmp_path, capsys):
        from colcon_lf.verb.analyze import _discover_targets

        ws = self._workspace(tmp_path)
        solo = ws / "src" / "dup_dir" / "launch" / "extra" / "solo.launch.py"
        assert _discover_targets(ws / "src", "dup_pkg", "solo.launch.py") == [
            ("dup_pkg", "extra/solo.launch.py", solo)]
        assert self._analyze(ws, "solo.launch.py", True) == 0
        assert self._written(ws) == ["dup_pkg/solo/launch/launch_graph.json"]

    def test_bare_name_counts_the_top_level_file(self, tmp_path, capsys):
        # Before the fix, launch/solo.launch.py silently won over
        # launch/extra/solo.launch.py for the bare name solo.launch.py.
        from colcon_lf.verb.analyze import _discover_targets

        ws = self._workspace(tmp_path)
        launch = ws / "src" / "dup_dir" / "launch"
        (launch / "solo.launch.py").write_text(_EMPTY_LAUNCH)
        assert self._analyze(ws, "solo.launch.py", True) == 1
        err = capsys.readouterr().err
        assert ("Error: launch file name 'solo.launch.py' is ambiguous in package "
                "'dup_pkg': launch/extra/solo.launch.py, launch/solo.launch.py. Pass "
                "the path relative to launch/: --launch-file extra/solo.launch.py or "
                "--launch-file ./solo.launch.py") in err
        assert not (ws / "out").exists()
        # ./<name> selects the file directly in launch/, and only that file.
        assert _discover_targets(ws / "src", "dup_pkg", "./solo.launch.py") == [
            ("dup_pkg", "solo.launch.py", launch / "solo.launch.py")]
        assert self._analyze(ws, "./solo.launch.py", True) == 0
        assert self._written(ws) == ["dup_pkg/solo/launch/launch_graph.json"]
        graph = (ws / "out" / "dup_pkg" / "solo" / "launch" / "launch_graph.json").read_text()
        assert "launch/solo.launch.py" in graph and "extra/" not in graph
        # A unique top-level file keeps the bare-name shorthand, even one the
        # discovery patterns would not list.
        (launch / "plain.py").write_text(_EMPTY_LAUNCH)
        assert _discover_targets(ws / "src", "dup_pkg", "plain.py") == [
            ("dup_pkg", "plain.py", launch / "plain.py")]

    def test_unambiguous_files_keep_their_output_names(self):
        from colcon_lf.verb.analyze import _launch_stem

        for name, stem in (("bringup.launch.py", "bringup"),
                           ("topics/talker_listener_launch.py", "talker_listener"),
                           ("a.launch.xml", "a"), ("b.launch.yaml", "b")):
            assert _launch_stem(name) == stem

    def test_scan_keeps_both_files_and_names_the_clash(self, tmp_path):
        from colcon_lf.scan import scan_workspace

        ws = self._workspace(tmp_path)
        doc = scan_workspace(ws, package="dup_pkg")
        entries = {e["launch_file"]: e for e in doc["packages"][0]["launch_files"]}
        assert sorted(entries) == ["extra/solo.launch.py", "left/demo.launch.py",
                                   "right/demo.launch.py"]
        for name in ("left/demo.launch.py", "right/demo.launch.py"):
            assert any("would write the same output directory dup_pkg/demo/" in r
                       for r in entries[name]["reasons"]), entries[name]["reasons"]
        assert not any("same output directory" in r
                       for r in entries["extra/solo.launch.py"]["reasons"])
        with pytest.raises(ValueError, match="not found in package 'dup_pkg'"):
            scan_workspace(ws, package="dup_pkg", launch_file="typo/solo.launch.py")


class TestPackageSelectionErrors:
    """``--package`` alone tells an unknown package from one without launch files."""

    @staticmethod
    def _analyze(ws: Path, package: str) -> int:
        from colcon_lf.verb.analyze import AnalyzeVerb

        args = argparse.Namespace(
            package=package, launch_file=None, workspace=str(ws),
            output_dir=str(ws / "out"), format="json",
            viz_only=False, launch_only=False,
        )
        return AnalyzeVerb().main(context=SimpleNamespace(args=args))

    @staticmethod
    def _workspace(tmp_path: Path) -> Path:
        ws = tmp_path / "ws"
        pkg = ws / "src" / "no_launch_pkg"
        pkg.mkdir(parents=True)
        (pkg / "package.xml").write_text("<package><name>no_launch_pkg</name></package>")
        return ws

    def test_unknown_package_is_reported_as_not_found(self, tmp_path, capsys):
        ws = self._workspace(tmp_path)
        assert self._analyze(ws, "nosuchpkg") == 1
        err = capsys.readouterr().err
        assert "Error: Package 'nosuchpkg' not found under" in err
        assert "No launch files found" not in err

    def test_package_without_launch_files_says_so(self, tmp_path, capsys):
        ws = self._workspace(tmp_path)
        assert self._analyze(ws, "no_launch_pkg") == 1
        err = capsys.readouterr().err
        assert "Error: No launch files found in package 'no_launch_pkg'" in err

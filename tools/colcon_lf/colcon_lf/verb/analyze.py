# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Colcon verb: ``colcon lf-analyze`` — full ROS2 extraction pipeline.

Usage::

    # Analyze all launch files in all packages
    colcon lf-analyze

    # Analyze all launch files in one package
    colcon lf-analyze --package pubsub_example

    # Analyze a single launch file
    colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py

    # Custom output directory
    colcon lf-analyze --output-dir /tmp/analysis --format json
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from xml.etree import ElementTree

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from colcon_lf.verb.workspace import resolve_workspace as _resolve_workspace
from colcon_lf.workspace import find_package_dirs


class AnalyzeVerb(VerbExtensionPoint):
    """Extract a ROS2 system model from launch files."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            "--package",
            default=None,
            help="ROS2 package to analyze (default: all packages)",
        )
        parser.add_argument(
            "--launch-file",
            default=None,
            help="Launch file name (default: all launch files in package)",
        )
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect)",
        )
        parser.add_argument(
            "--output-dir",
            default=None,
            help=(
                "Base output directory. Default: lf-analysis/{package}/ "
                "inside the workspace"
            ),
        )
        parser.add_argument(
            "--format",
            choices=["json", "pdf", "svg", "all"],
            default="all",
            help="Output format: json, pdf, svg, or all (default: all)",
        )
        parser.add_argument(
            "--viz-only",
            action="store_true",
            default=False,
            help=(
                "Skip extraction; re-generate visualizations from existing "
                "system.json files in the output directory."
            ),
        )
        parser.add_argument(
            "--launch-arg",
            action="append",
            default=[],
            metavar="NAME:=VALUE",
            help=(
                "Launch argument value, as ros2 launch takes it (repeatable). "
                "The generated system is fixed to these values; an argument the "
                "launch file does not declare is refused."
            ),
        )
        parser.add_argument(
            "--launch-only",
            action="store_true",
            default=False,
            help=(
                "Run only the launch file parsing stage. Writes a "
                "launch_graph.json per launch file without C++ analysis."
            ),
        )

    def main(self, *, context) -> int:  # noqa: D102
        from colcon_lf.launch_analysis.parser import (
            LaunchArgumentError,
            launch_argument_overrides,
            parse_launch_arg,
        )
        args = context.args
        try:
            overrides = dict(parse_launch_arg(a) for a in (
                getattr(args, "launch_arg", None) or []))
        except LaunchArgumentError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        with launch_argument_overrides(overrides):
            return self._run(args)

    def _run(self, args) -> int:
        if args.viz_only:
            return self._viz_only(args)

        if args.launch_only:
            return self._launch_only(args)

        # --- Resolve workspace ---
        workspace = _resolve_workspace(args.workspace)
        if workspace is None:
            print(
                "Error: Cannot find colcon workspace. "
                "Use --workspace or run from the workspace directory.",
                file=sys.stderr,
            )
            return 1

        src_root = workspace / "src"
        if not src_root.is_dir():
            print(f"Error: src/ not found in {workspace}", file=sys.stderr)
            return 1

        # --- Discover targets ---
        try:
            targets = _discover_targets(src_root, args.package, args.launch_file)
            _refuse_output_collisions(targets)
            _refuse_generated_name_collisions(targets, src_root)
        except LaunchSelectionError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        if not targets:
            if args.package and (
                    args.launch_file or args.package not in _find_packages(src_root)):
                print(
                    f"Error: Package {args.package!r} not found under {src_root}",
                    file=sys.stderr,
                )
            elif args.package:
                print(
                    f"Error: No launch files found in package {args.package!r}",
                    file=sys.stderr,
                )
            else:
                print("Error: No launch files found in workspace", file=sys.stderr)
            return 1

        for skipped_pkg, skipped_rel in (
                [] if args.launch_file else _marked_launch_files(src_root, args.package)):
            print(f"==> Skipped {skipped_pkg}/launch/{skipped_rel}: "
                  "skipped (not-a-system)")
        print(f"==> Found {len(targets)} launch file(s) to analyze\n")

        # --- Process each target ---
        failures = 0
        for package, launch_file, launch_path in targets:
            rc = _analyze_one(
                package=package,
                launch_file=launch_file,
                launch_path=launch_path,
                workspace=workspace,
                output_dir=args.output_dir,
                output_fmt=args.format,
            )
            if rc != 0:
                failures += 1

        if failures:
            print(
                f"\n==> {failures}/{len(targets)} launch file(s) failed",
                file=sys.stderr,
            )
            return 1

        print(f"\n==> All {len(targets)} launch file(s) analyzed successfully")
        return 0

    def _launch_only(self, args) -> int:
        """Run only launch file parsing, write launch_graph.json."""
        workspace = _resolve_workspace(args.workspace)
        if workspace is None:
            print(
                "Error: Cannot find colcon workspace. "
                "Use --workspace or run from the workspace directory.",
                file=sys.stderr,
            )
            return 1

        src_root = workspace / "src"
        if not src_root.is_dir():
            print(f"Error: src/ not found in {workspace}", file=sys.stderr)
            return 1

        try:
            targets = _discover_targets(src_root, args.package, args.launch_file)
            _refuse_output_collisions(targets)
            _refuse_generated_name_collisions(targets, src_root)
        except LaunchSelectionError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        if not targets:
            print("Error: No launch files found", file=sys.stderr)
            return 1

        print(f"==> Found {len(targets)} launch file(s) to parse (launch-only)\n")

        failures = 0
        warnings = 0
        for package, launch_file, launch_path in targets:
            rc = _parse_launch_only(
                package=package,
                launch_file=launch_file,
                launch_path=launch_path,
                workspace=workspace,
                output_dir=args.output_dir,
            )
            if rc == 1:
                failures += 1
            elif rc == 2:
                warnings += 1

        if failures:
            print(
                f"\n==> {failures}/{len(targets)} launch file(s) failed",
                file=sys.stderr,
            )
            return 1

        if warnings:
            print(
                f"\n==> All {len(targets)} launch file(s) parsed "
                f"({warnings} with warnings)"
            )
            return 0

        print(f"\n==> All {len(targets)} launch file(s) parsed successfully")
        return 0

    def _viz_only(self, args) -> int:
        """Re-generate visualizations from existing system.json files."""
        import json

        workspace = _resolve_workspace(args.workspace)
        if workspace is None:
            print(
                "Error: Cannot find colcon workspace. "
                "Use --workspace or run from the workspace directory.",
                file=sys.stderr,
            )
            return 1

        # Find all system.json files in the output directory.
        base = Path(args.output_dir) if args.output_dir else workspace / "lf-analysis"
        if not base.is_dir():
            print(f"Error: Output directory not found: {base}", file=sys.stderr)
            return 1

        system_jsons = sorted(base.rglob("system.json"))
        if not system_jsons:
            print(f"Error: No system.json files found in {base}", file=sys.stderr)
            return 1

        fmt = args.format if args.format != "json" else "all"
        viz_formats = _viz_formats(fmt)

        print(f"==> Re-generating visualizations for {len(system_jsons)} system(s)\n")
        for sj in system_jsons:
            rel = sj.parent.relative_to(base)
            print(f"--- {rel} ---")
            with open(sj) as f:
                system_data = json.load(f)
            try:
                _generate_visualizations_from_dict(
                    system_data, sj.parent, viz_formats,
                )
            except Exception as exc:
                print(f"  Warning: Visualization failed: {exc}", file=sys.stderr)
            print(f"  -> {sj.parent}/viz/")

        print(f"\n==> Done")
        return 0


# ---------------------------------------------------------------------------
# Single launch file analysis
# ---------------------------------------------------------------------------

def _analyze_one(
    *,
    package: str,
    launch_file: str,
    launch_path: Path,
    workspace: Path,
    output_dir: str | None,
    output_fmt: str,
) -> int:
    """Analyze a single launch file. Returns 0 on success."""
    print(f"--- {package}/{launch_file} ---")

    from colcon_lf.graph.hierarchical import SkippedNodesError, build_system_graph

    not_analyzed: list[str] = []
    try:
        graph = build_system_graph(
            launch_path, workspace_root=workspace, not_analyzed=not_analyzed,
        )
    except SkippedNodesError as exc:
        for reason in not_analyzed:
            print(f"  Warning: {reason}", file=sys.stderr)
        for reason in exc.reasons:
            print(f"  Error: {reason}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"  Error: {exc}", file=sys.stderr)
        return 1

    if _refuse_without_launch(graph.launch_graph.root):
        return 1

    n_nodes = len(graph.nodes)
    n_topics = len(graph.communication.topics)
    n_services = len(graph.communication.services)
    n_actions = len(graph.communication.actions)
    print(
        f"  {n_nodes} nodes, {n_topics} topics, "
        f"{n_services} services, {n_actions} actions"
    )
    for reason in not_analyzed:
        print(f"  Warning: {reason}", file=sys.stderr)
    # Launch-analysis errors (an include or a parameter file the analyzer
    # could not evaluate) stay in system.json under their launch file; they
    # are printed too, since the system they describe is incomplete.
    for path, err in _launch_errors(graph.launch_graph.root):
        print(f"  WARNING: {path}: {err}", file=sys.stderr)

    # Surface parse warnings from libclang (e.g. missing headers) so the user
    # knows why some nodes may have incomplete extraction.
    all_warnings: dict[str, list[str]] = {}  # warning -> [node_name, ...]
    for node_name, node_info in graph.nodes.items():
        for w in node_info.parse_warnings:
            all_warnings.setdefault(w, []).append(node_name)
    if all_warnings:
        print("  Parse warnings (may cause incomplete extraction):", file=sys.stderr)
        for warning, affected_nodes in sorted(all_warnings.items()):
            nodes_str = ", ".join(affected_nodes[:3])
            if len(affected_nodes) > 3:
                nodes_str += f" (+{len(affected_nodes) - 3} more)"
            print(f"    ! {warning}", file=sys.stderr)
            print(f"      Affects: {nodes_str}", file=sys.stderr)
            # Give a rosdep hint if it looks like a missing header
            if "file not found" in warning:
                header = warning.split("'")[1] if "'" in warning else ""
                if header:
                    pkg_guess = header.split("/")[0].replace("_", "-")
                    print(
                        f"      Hint: try `sudo apt-get install ros-jazzy-{pkg_guess}` "
                        f"or add it to the Dockerfile",
                        file=sys.stderr,
                    )

    # An unresolved #include, or any other error a compiler would stop on,
    # leaves the AST incomplete. clang drops whatever depends on the broken
    # part, so a timer, publisher or subscription can vanish while the rest
    # of the node looks fine. Name every such node and refuse to emit it,
    # even when some entities are visible.
    for node_name, node_info in sorted(graph.nodes.items()):
        for inc in node_info.unresolved_includes:
            print(
                f"  Unresolved include: node {node_name!r} "
                f"(source {node_info.source_file}): "
                f"'{inc['header']}' not found, "
                f"included from {inc['included_from']}:{inc['line']}",
                file=sys.stderr,
            )
    incomplete = incomplete_reasons(graph)
    if incomplete:
        for reason in incomplete:
            print(f"  Error: {reason}", file=sys.stderr)
        return 1

    # --- Run feature/hazard detection (RCLLF-H000..H003) and print grouped
    # diagnostics, same style as the parse-warnings block above. Diagnostics
    # are persisted as facts only (code, node, topic, message) — no severity
    # — into system.json (schema_version 3); severity is resolved from the
    # features REGISTRY at read time.
    from colcon_lf.features.diagnostics import DetectionContext
    from colcon_lf.features.registry import run_detection, severity_for

    diagnostics = run_detection(DetectionContext(graph=graph))
    graph.diagnostics = diagnostics

    if diagnostics:
        by_code: dict[str, list] = {}
        for diag in diagnostics:
            by_code.setdefault(diag.code, []).append(diag)
        print("  Hazard diagnostics:", file=sys.stderr)
        for code, diags in sorted(by_code.items()):
            severity = severity_for(code)
            print(
                f"    ! [{severity.value.upper()}] {code} "
                f"({len(diags)} occurrence{'s' if len(diags) != 1 else ''})",
                file=sys.stderr,
            )
            for diag in diags[:3]:
                print(f"      {diag.message}", file=sys.stderr)
            if len(diags) > 3:
                print(f"      (+{len(diags) - 3} more)", file=sys.stderr)

    # --- Determine output path ---
    launch_stem = _launch_stem(launch_file)
    try:
        output_subdir = _output_path(workspace, package, launch_stem, output_dir)
    except OutputRootError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    # --- Write JSON (always) ---
    from colcon_lf.output.writer import write

    write(graph, output_subdir)

    # --- Write launch/ subdirectory (launch_graph.json + SVG) ---
    from colcon_lf.output.serializer import serialize, serialize_to_dict
    from colcon_lf.visualization.launch_graph_viz import launch_graph_viz
    from colcon_lf.visualization.dot_renderer import render as render_dot

    launch_dir = output_subdir / "launch"
    launch_dir.mkdir(parents=True, exist_ok=True)

    launch_json = launch_dir / "launch_graph.json"
    launch_json.write_text(serialize(graph.launch_graph), encoding="utf-8")

    try:
        graph_data = serialize_to_dict(graph.launch_graph)
        dot = launch_graph_viz({"launch_graph": graph_data})
        render_dot(dot, launch_dir / "launch_graph", formats=["svg"])
    except Exception as exc:
        print(f"  Warning: Launch visualization failed: {exc}", file=sys.stderr)

    # --- Visualizations ---
    if output_fmt in ("pdf", "svg", "all"):
        viz_formats = _viz_formats(output_fmt)
        try:
            _generate_visualizations(graph, output_subdir, viz_formats)
        except Exception as exc:
            print(f"  Warning: Visualization failed: {exc}", file=sys.stderr)

    print(f"  -> {output_subdir}/")
    return 0


def incomplete_reasons(graph) -> list[str]:
    """One reason per node whose AST is incomplete.

    An unresolved ``#include``, or any other error a compiler would stop on,
    leaves the AST incomplete, so a timer, publisher or subscription can be
    missing. ``colcon lf-analyze`` refuses such a graph; ``ros2 lf scan``
    reports the launch file as not convertible with the same reasons.
    """
    incomplete: list[str] = []
    for node_name, node_info in sorted(graph.nodes.items()):
        if node_info.unresolved_includes:
            headers = ", ".join(
                f"'{inc['header']}'" for inc in node_info.unresolved_includes
            )
            incomplete.append(
                f"node {node_name!r} (source {node_info.source_file}) has "
                f"unresolved include(s) {headers}, so timers, publishers or "
                f"subscriptions can be missing from the analysis. Make the "
                f"headers reachable (build the package with "
                f"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON, or install them under "
                f"install/<pkg>/include) and re-run colcon lf-analyze."
            )
        not_found = {
            f"'{inc['header']}' file not found"
            for inc in node_info.unresolved_includes
        }
        errors = [w for w in node_info.parse_warnings if w not in not_found]
        if errors:
            diags = ", ".join(f"'{w}'" for w in errors)
            incomplete.append(
                f"node {node_name!r} (source {node_info.source_file}): "
                f"libclang reported error(s) {diags}, so timers, publishers "
                f"or subscriptions can be missing from the analysis. Fix the "
                f"source or its include flags (compile_commands.json) and "
                f"re-run colcon lf-analyze."
            )
    return incomplete


#: Prefix of the launch parser's error when ``launch`` does not import
#: (``launch_analysis/parser.py``, ``parse_launch``).
LAUNCH_UNAVAILABLE = "launch package not available"
LAUNCH_UNAVAILABLE_HINT = (
    "Hint: the ROS 'launch' Python packages do not import; keep the ROS path "
    "on PYTHONPATH (source /opt/ros/<distro>/setup.bash, then extend it: "
    "PYTHONPATH=$PWD:$PYTHONPATH)"
)


def _refuse_without_launch(root) -> bool:
    """Print an error and return True when the ROS launch packages are missing.

    Without them the launch parser evaluates nothing: every launch file comes
    back with no nodes and one error. Writing that as a 0-node system.json and
    exiting 0 let generate and build run on an empty system, so
    the analysis stops here instead.
    """
    missing = [err for _, err in _launch_errors(root)
               if err.startswith(LAUNCH_UNAVAILABLE)]
    if not missing:
        return False
    print(f"  Error: {missing[0]}", file=sys.stderr)
    print(f"  {LAUNCH_UNAVAILABLE_HINT}", file=sys.stderr)
    return True


def _launch_errors(file_info) -> list[tuple[str, str]]:
    """``(launch file, error)`` for *file_info* and every file it includes."""
    found = [(str(file_info.file_path), err) for err in file_info.errors]
    # A group's own errors are already in its file's; its includes are not.
    pending = [file_info]
    while pending:
        scope = pending.pop()
        for child in scope.includes:
            found.extend(_launch_errors(child))
        pending.extend(scope.groups)
    return found


# ---------------------------------------------------------------------------
# Launch-only analysis
# ---------------------------------------------------------------------------

def _parse_launch_only(
    *,
    package: str,
    launch_file: str,
    launch_path: Path,
    workspace: Path,
    output_dir: str | None,
) -> int:
    """Parse a single launch file and write launch_graph.json + SVG. Returns 0 on success."""
    print(f"--- {package}/{launch_file} ---")

    try:
        from colcon_lf.launch_analysis.parser import parse_launch
        from colcon_lf.output.serializer import serialize, serialize_to_dict

        graph = parse_launch(launch_path, workspace_root=workspace)
    except Exception as exc:
        print(f"  Error: {exc}", file=sys.stderr)
        return 1

    if _refuse_without_launch(graph.root):
        return 1

    n_nodes = len(graph.all_nodes)
    launch_errors = _launch_errors(graph.root)
    n_errors = len(launch_errors)
    print(f"  {n_nodes} nodes, {n_errors} errors")

    for path, err in launch_errors:
        # Truncate very long error messages (e.g. package search paths)
        msg = err if len(err) <= 200 else err[:200] + "..."
        print(f"  WARNING: {path}: {msg}", file=sys.stderr)

    # --- Write to launch/ subdirectory ---
    launch_stem = _launch_stem(launch_file)
    try:
        output_subdir = _output_path(workspace, package, launch_stem, output_dir)
    except OutputRootError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    launch_dir = output_subdir / "launch"
    launch_dir.mkdir(parents=True, exist_ok=True)

    out_file = launch_dir / "launch_graph.json"
    out_file.write_text(serialize(graph), encoding="utf-8")

    # --- Generate SVG visualization ---
    try:
        from colcon_lf.visualization.launch_graph_viz import launch_graph_viz
        from colcon_lf.visualization.dot_renderer import render

        graph_data = serialize_to_dict(graph)
        # launch_graph_viz expects {"launch_graph": {"root": ...}} (system.json format)
        dot = launch_graph_viz({"launch_graph": graph_data})
        render(dot, launch_dir / "launch_graph", formats=["svg"])
    except Exception as exc:
        print(f"  Warning: Visualization failed: {exc}", file=sys.stderr)

    print(f"  -> {launch_dir}/")
    return 2 if n_errors else 0  # 2 = warnings (parsed but with errors)


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

class LaunchSelectionError(ValueError):
    """The launch files selected for analysis cannot be analyzed as asked."""


def _launch_stem(launch_file: str) -> str:
    """Output directory name of a launch file: ``launch/x/bringup.launch.py`` -> ``bringup``."""
    return (Path(launch_file).name
            .replace(".launch.xml", "")
            .replace(".launch.yaml", "")
            .replace(".launch.py", "")
            .replace("_launch.py", "")
            .replace(".py", ""))


def _discover_targets(
    src_root: Path,
    package: str | None,
    launch_file: str | None,
) -> list[tuple[str, str, Path]]:
    """Discover (package, launch_file, path) tuples to analyze.

    Finds packages by ``package.xml`` presence (recursive), matching how
    colcon discovers packages.  Handles both flat layouts
    (``src/timer_example/``) and nested mono-repos
    (``src/navigation2/nav2_bringup/``).  Launch files are searched in
    ``launch/`` and its subdirectories (ros2/demos keeps them in
    ``launch/topics/``).  ``launch_file`` in each tuple is the path relative
    to ``launch/`` (``topics/talker_listener_launch.py``).

    An explicit *launch_file* with a directory part (``./<name>`` for a
    file directly in ``launch/``) must name ``launch/<launch_file>``
    exactly.  A bare file name must match exactly one file, counting
    ``launch/<name>`` and every subdirectory.  A missing file or an
    ambiguous bare name raises
    :class:`LaunchSelectionError`.  An unknown package returns ``[]``.
    """
    pkg_dirs = _find_packages(src_root)

    if package and launch_file:
        pkg_dir = pkg_dirs.get(package)
        if pkg_dir is None:
            return []
        launch_dir = pkg_dir / "launch"
        path = launch_dir / launch_file
        if Path(launch_file).name != launch_file:
            rel = _launch_rel(path, launch_dir)
            if rel is not None and path.is_file():
                return [(package, rel, path)]
            raise LaunchSelectionError(
                f"launch file {launch_file!r} not found in package {package!r}: "
                f"no file {path} (a path is taken relative to the package's "
                f"launch/ directory)")
        # A bare name counts every file of that name, launch/<name> included.
        found = {f for f in _launch_files(pkg_dir, include_marked=True)
                 if f.name == launch_file}
        if path.is_file():
            found.add(path)
        matches = sorted(found)
        if len(matches) == 1:
            return [(package, _launch_rel(matches[0], launch_dir), matches[0])]
        if not matches:
            raise LaunchSelectionError(
                f"launch file {launch_file!r} not found in package {package!r} "
                f"(searched {launch_dir} and its subdirectories)")
        names = [_launch_rel(f, launch_dir) for f in matches]
        # A file directly in launch/ is selected as ./<name>.
        options = [n if "/" in n else f"./{n}" for n in names]
        raise LaunchSelectionError(
            f"launch file name {launch_file!r} is ambiguous in package {package!r}: "
            + ", ".join(f"launch/{n}" for n in names)
            + ". Pass the path relative to launch/: "
            + " or ".join(f"--launch-file {o}" for o in options))

    if package:
        pkg_dir = pkg_dirs.get(package)
        if pkg_dir is None:
            return []
        return [(package, _launch_rel(f, pkg_dir / "launch"), f)
                for f in _launch_files(pkg_dir)]

    # All packages
    targets: list[tuple[str, str, Path]] = []
    for pkg_name, pkg_dir in sorted(pkg_dirs.items()):
        targets.extend((pkg_name, _launch_rel(f, pkg_dir / "launch"), f)
                       for f in _launch_files(pkg_dir))
    return targets


def _launch_rel(path: Path, launch_dir: Path) -> str | None:
    """*path* relative to *launch_dir* in POSIX form, or None if outside it."""
    try:
        rel = path.relative_to(launch_dir)
    except ValueError:
        return None
    return None if ".." in rel.parts else rel.as_posix()


def _output_collisions(
    targets: list[tuple[str, str, Path]],
) -> list[tuple[str, str, list[str]]]:
    """Groups of *targets* that would write the same output directory.

    Returns ``(package, stem, [launch files relative to launch/])`` per group
    of two or more, for example ``left/demo.launch.py`` and
    ``right/demo.launch.py`` (both ``<package>/demo``).
    """
    groups: dict[tuple[str, str], list[str]] = {}
    for package, launch_file, _path in targets:
        groups.setdefault((package, _launch_stem(launch_file)), []).append(launch_file)
    return [(pkg, stem, names) for (pkg, stem), names in groups.items()
            if len(names) > 1]


def _collision_message(package: str, stem: str, names: list[str]) -> str:
    files = " and ".join(f"launch/{n}" for n in names)
    return (f"launch files {files} of package {package!r} would write the same "
            f"output directory {package}/{stem}/")


def _refuse_output_collisions(targets: list[tuple[str, str, Path]]) -> None:
    """Raise :class:`LaunchSelectionError` before any output is written if two
    selected launch files share an output directory."""
    clashes = _output_collisions(targets)
    if clashes:
        raise LaunchSelectionError(
            "; ".join(_collision_message(*c) for c in clashes)
            + ". Nothing was written. Rename one of the files, or analyze one "
            "with --launch-file <path relative to launch/> (each such run "
            "replaces that directory).")


def _generated_name_collisions(
    all_targets: list[tuple[str, str, Path]],
) -> list[tuple[str, list[tuple[str, str]]]]:
    """Groups of launch files that map to one ``lf_generated_*`` package name.

    The name is ``lf_generated_<package>_<stem>`` with the stem's underscores
    dropped and lower-cased (``lf_naming.generated_lfc_package_name``), so
    ``mt_main`` and ``mtmain`` of one package, or package ``a_b`` with stem
    ``c`` and package ``a`` with stem ``b_c``, name the same package: the
    second would overwrite the first and ``ros2 lf launch`` would start the
    wrong system. Returns ``(generated name, [(package, launch file)])`` per
    group of two or more distinct (package, stem) pairs; two files of one
    stem are ``_output_collisions``.
    """
    from colcon_lf.generation.lf_naming import (
        generated_lfc_package_name,
        generated_package_stem,
        launch_stem_to_reactor_name,
    )
    groups: dict[str, dict[tuple[str, str], list[str]]] = {}
    for package, launch_file, _path in all_targets:
        stem = _launch_stem(launch_file)
        try:
            name = generated_lfc_package_name(
                package, generated_package_stem(launch_stem_to_reactor_name(stem)))
        except ValueError:
            continue  # an invalid stem is refused when the system is generated
        groups.setdefault(name, {}).setdefault((package, stem), []).append(launch_file)
    found = []
    for name, by_stem in groups.items():
        if len(by_stem) > 1:
            found.append((name, [(pkg, f) for (pkg, _stem), files in sorted(by_stem.items())
                                 for f in files]))
    return found


def _refuse_generated_name_collisions(
    targets: list[tuple[str, str, Path]],
    src_root: Path,
) -> None:
    """Raise :class:`LaunchSelectionError` before any output is written if a
    selected launch file shares its generated package name with another launch
    file of the workspace, selected or not."""
    selected = {(pkg, launch_file) for pkg, launch_file, _p in targets}
    everything = _discover_targets(src_root, None, None)
    clashes = [(name, members) for name, members in _generated_name_collisions(everything)
               if any(m in selected for m in members)]
    if clashes:
        raise LaunchSelectionError(
            "; ".join(
                "launch files " + " and ".join(
                    f"launch/{f} of package {p!r}" for p, f in members)
                + f" would both generate the package {name!r}"
                for name, members in clashes)
            + ". Nothing was written. Rename one of the launch files so that the "
            "name, without underscores, differs.")


_LAUNCH_PATTERNS = ("*.launch.py", "*_launch.py", "*.launch.xml", "*.launch.yaml")


_NOT_A_SYSTEM_RE = re.compile(r"^\s*(?:#|<!--)\s*rcllf:\s*not-a-system\b", re.MULTILINE)


def is_not_a_system(path: Path) -> bool:
    """True if the launch file carries the ``rcllf: not-a-system`` marker.

    The marker is a comment line: ``# rcllf: not-a-system`` in a ``.py`` or
    ``.yaml`` launch file, ``<!-- rcllf: not-a-system -->`` in XML. Only the
    first 4 KiB are read. Such a file is a helper (a standalone node launcher),
    not a system, so analyze skips it and lists it as skipped.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            head = handle.read(4096)
    except OSError:
        return False
    return _NOT_A_SYSTEM_RE.search(head) is not None


def _launch_files(pkg_dir: Path, include_marked: bool = False) -> list[Path]:
    """Return the launch files under ``pkg_dir/launch``, recursively.

    Files marked ``rcllf: not-a-system`` are left out unless *include_marked*.
    """
    launch_dir = pkg_dir / "launch"
    if not launch_dir.is_dir():
        return []
    found: set[Path] = set()
    for pattern in _LAUNCH_PATTERNS:
        found.update(launch_dir.rglob(pattern))
    if not include_marked:
        found = {f for f in found if not is_not_a_system(f)}
    return sorted(found)


def _marked_launch_files(
    src_root: Path, package: str | None,
) -> list[tuple[str, str]]:
    """``(package, launch file relative to launch/)`` of each marked file."""
    out: list[tuple[str, str]] = []
    for pkg_name, pkg_dir in sorted(_find_packages(src_root).items()):
        if package and pkg_name != package:
            continue
        for f in _launch_files(pkg_dir, include_marked=True):
            if is_not_a_system(f):
                out.append((pkg_name, _launch_rel(f, pkg_dir / "launch") or f.name))
    return out


def _find_packages(src_root: Path) -> dict[str, Path]:
    """Find all ROS2 packages under *src_root* by locating ``package.xml``.

    Returns ``{package_name: package_dir}``, named by the manifest's
    ``<name>`` (see :func:`colcon_lf.workspace.find_package_dirs`).
    """
    return find_package_dirs(src_root)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _output_path(
    workspace: Path,
    package: str,
    launch_stem: str,
    output_dir: str | None,
) -> Path:
    """Compute the output directory for one analysis run.

    Default: ``{workspace}/lf-analysis/{package}/{launch_stem}/``
    With --output-dir: ``{output_dir}/{package}/{launch_stem}/``
    """
    if output_dir is not None:
        root = Path(output_dir)
    else:
        root = workspace / "lf-analysis"
    check_owned_output_root(root, workspace)
    mark_colcon_ignore(root)
    return root / package / launch_stem


COLCON_IGNORE_TEXT = (
    "# Analyzer output, not packages: keeps colcon out of lf-analysis/. lf-bench\n"
    "# builds its lfc copies under lf-analysis/lf-generated/ with --paths, and a\n"
    "# crawl would find them a second time next to packages/lf-generated/.\n"
)


# Package name prefixes reserved for colcon-lf: ``colcon lf`` and lf-bench
# generate lf_generated_*, the legacy wrapper path lf_wrapper_*. A package with
# one of these names is taken as tool output, with no per-package marker.
GENERATED_PACKAGE_PREFIXES = ("lf_generated_", "lf_wrapper_")


class OutputRootError(ValueError):
    """An output root the tool does not own, so it must not mark it COLCON_IGNORE."""


def _manifest_name(manifest: Path) -> str:
    """The top-level ``<name>`` of a package.xml, or OutputRootError."""
    try:
        package = ElementTree.parse(manifest).getroot()
    except (ElementTree.ParseError, OSError) as exc:
        raise OutputRootError(f"{manifest} cannot be parsed ({exc})") from exc
    name = package.find("name")
    if name is None or not (name.text or "").strip():
        raise OutputRootError(f"{manifest} has no name")
    return name.text.strip()


def check_owned_output_root(root: Path, workspace: Path) -> None:
    """Raise OutputRootError when a COLCON_IGNORE in ``root`` would hide real packages.

    Refused: the workspace itself, an ancestor of it, and a directory with a
    ``package.xml`` below it that cannot be parsed, has no top-level ``<name>``,
    or names a package outside the prefixes reserved for colcon-lf
    (``GENERATED_PACKAGE_PREFIXES``). The prefixes are the only ownership test.
    """
    root_r = root.resolve()
    workspace_r = workspace.resolve()
    if root_r == workspace_r:
        raise OutputRootError(
            f"output directory {root} is the workspace: a COLCON_IGNORE there "
            "hides every package from colcon. Pick a subdirectory such as lf-analysis/."
        )
    if root_r in workspace_r.parents:
        raise OutputRootError(
            f"output directory {root} contains the workspace {workspace}: a "
            "COLCON_IGNORE there hides every package from colcon. Pick a "
            "subdirectory such as lf-analysis/."
        )
    if not root_r.is_dir():
        return
    for manifest in sorted(root_r.rglob("package.xml")):
        try:
            name = _manifest_name(manifest)
        except OutputRootError as exc:
            raise OutputRootError(
                f"output directory {root} holds an unreadable package manifest, "
                f"{exc}: a COLCON_IGNORE there could hide a package from colcon. "
                "Pick a directory without packages."
            ) from exc
        if not name.startswith(GENERATED_PACKAGE_PREFIXES):
            raise OutputRootError(
                f"output directory {root} holds package {name} ({manifest}), which "
                "colcon-lf did not generate (the name prefixes lf_generated_ and "
                "lf_wrapper_ are reserved for colcon-lf): a COLCON_IGNORE there "
                "hides it from colcon. Pick a directory without packages."
            )


def mark_colcon_ignore(root: Path) -> None:
    """Create ``root/COLCON_IGNORE`` (and ``root``) when it is missing."""
    marker = root / "COLCON_IGNORE"
    if not marker.exists():
        root.mkdir(parents=True, exist_ok=True)
        marker.write_text(COLCON_IGNORE_TEXT, encoding="utf-8")


def _viz_formats(output_fmt: str) -> list[str]:
    """Map --format value to graphviz format list."""
    if output_fmt == "all":
        return ["svg"]
    return [output_fmt]


def _generate_visualizations(
    graph: object,
    output_subdir: Path,
    formats: list[str],
) -> None:
    """Generate all visualization files from the system graph."""
    from colcon_lf.output.serializer import serialize_to_dict

    system_data = serialize_to_dict(graph)
    _generate_visualizations_from_dict(system_data, output_subdir, formats)


def _generate_visualizations_from_dict(
    system_data: dict,
    output_subdir: Path,
    formats: list[str],
) -> None:
    """Generate all visualization files from a system_data dict."""
    from colcon_lf.visualization.dot_renderer import render
    from colcon_lf.visualization.launch_graph_viz import launch_graph_viz
    from colcon_lf.visualization.node_internals_viz import node_internals_viz
    from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
    from colcon_lf.visualization.system_graph_viz import system_graph_viz

    viz_dir = output_subdir / "viz"

    dot = launch_graph_viz(system_data)
    render(dot, viz_dir / "launch_graph", formats=formats)

    dot = comm_graph_viz(system_data)
    render(dot, viz_dir / "comm_graph", formats=formats)

    dot = system_graph_viz(system_data)
    render(dot, viz_dir / "system_graph", formats=formats)

    nodes_data = system_data.get("nodes", {})
    for node_key, node_data in nodes_data.items():
        safe_name = node_key.replace("/", "_").lstrip("_")
        dot = node_internals_viz(node_data)
        render(dot, viz_dir / "nodes" / f"{safe_name}_internals", formats=formats)

    from colcon_lf.visualization.topology_svg import render_topology_svg
    render_topology_svg(system_data, viz_dir / "topology.svg")

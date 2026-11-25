# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Colcon verb: ``colcon lf-configure`` — scaffold + validate execution policy.

Usage::

    # Scaffold + validate + stale-report for all analyzed systems
    colcon lf-configure

    # Every analysis of one package (all launch stems)
    colcon lf-configure --package pubsub_example

    # Single system
    colcon lf-configure --system-json lf-analysis/pubsub_example/bringup/system.json

    # Print the resolved effective table (post-merge, pre-env)
    colcon lf-configure --show

    # Interactive prompt loop over the sparse execution.yaml
    colcon lf-configure --interactive
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from colcon_lf.verb.workspace import resolve_workspace as _resolve_workspace


class ConfigureVerb(VerbExtensionPoint):
    """Scaffold, validate, and resolve execution-policy configuration."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def add_arguments(self, *, parser):  # noqa: D102
        add_package_argument(parser)
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )
        parser.add_argument(
            "--show",
            action="store_true",
            default=False,
            help=(
                "Print the resolved effective policy table (post-merge, pre-env). "
                "Read-only: writes no file. With --interactive the prompt loop "
                "still creates and writes the sparse execution.yaml."
            ),
        )
        parser.add_argument(
            "--interactive",
            action="store_true",
            default=False,
            help=(
                "Interactively prompt for policy values (defaults -> nodes -> "
                "subscriptions), reading and writing only the sparse "
                "execution.yaml."
            ),
        )

    def main(self, *, context) -> int:  # noqa: D102
        args = context.args

        system_jsons = _find_system_jsons(
            args.system_json, args.workspace, args.package
        )
        if not system_jsons:
            if args.package is not None:
                print(
                    package_scope_error(
                        args.package, _resolve_workspace(args.workspace)
                    ),
                    file=sys.stderr,
                )
            else:
                print("Error: No system.json files found.", file=sys.stderr)
            return 1

        report_package_scope(args.package, system_jsons)

        failures = 0
        for sj in system_jsons:
            try:
                rc = configure_one(
                    system_json=sj,
                    show=args.show,
                    interactive=args.interactive,
                )
            except Exception as exc:
                print(f"  Error: {exc}", file=sys.stderr)
                rc = 1
            if rc != 0:
                failures += 1

        if failures:
            print(
                f"\n==> {failures}/{len(system_jsons)} system(s) failed",
                file=sys.stderr,
            )
            return 1

        return 0


# ---------------------------------------------------------------------------
# Shared entry point — used by ConfigureVerb.main AND `colcon lf`'s chain.
#
# `colcon lf` folds a scaffold-only configure step into its pipeline by
# calling this function directly (see verb/lf.py), NOT via a FakeContext
# shim (00_design.md section 4 / survey 02 section 4 flags the FakeContext
# pattern as a wart, not something to extend).
# ---------------------------------------------------------------------------

def configure_one(
    *,
    system_json: Path,
    show: bool = False,
    interactive: bool = False,
    scaffold_only: bool = False,
) -> int:
    """Run the configure pipeline for one system.json. Returns 0 on success.

    Modes (not mutually exclusive except as noted):
      scaffold_only=True  — only regenerate the reference file / create the
                             sparse file if absent; no validation printout,
                             no --interactive. Used by `colcon lf`. Combined
                             with show=True it raises ValueError: show promises
                             to write nothing and scaffold_only only writes.
      (default)            — scaffold + validate + stale-report.
      show=True            — also print the resolved effective table.
      interactive=True      — run last, after everything else; prompt loop.
    """
    if show and scaffold_only:
        raise ValueError(
            "configure_one: show=True and scaffold_only=True contradict each other: "
            "--show writes no file, a scaffold-only run only writes the config files; "
            "pass one of them")

    from colcon_lf.output.serializer import deserialize_system_graph

    print(f"--- {system_json} ---")

    with open(system_json) as f:
        raw = f.read()
    graph = deserialize_system_graph(raw)

    config_dir = system_json.parent / "config"

    from colcon_lf.exec_config.scaffold import scaffold

    # --show only reads: it scaffolds nothing, so it can be pointed at a whole
    # workspace without creating an execution.yaml in every analysis.
    read_only = show and not interactive
    result = scaffold(config_dir, graph, write=not read_only)
    if read_only:
        if not result.sparse_path.is_file():
            print(f"  (no {result.sparse_path.name} yet: built-in and package defaults; "
                  f"run without --show to create it)")
    else:
        if result.sparse_created:
            print(f"  -> {result.sparse_path} (created)")
        print(f"  -> {result.reference_path}")

    if scaffold_only:
        return 0

    from colcon_lf.exec_config.model import ExecConfig, package_exec_config_path

    try:
        config = ExecConfig.load_layered(
            result.sparse_path, package_exec_config_path(graph), graph,
        )
    except ValueError as exc:
        print(f"  Error: {exc}", file=sys.stderr)
        return 1

    if result.stale_nodes or result.stale_subscription_topics:
        print("  Stale entries in execution.yaml (no longer in the model):")
        for node_key in result.stale_nodes:
            print(f"    ! nodes/subscriptions: '{node_key}'")
        for node_key, topic_key in result.stale_subscription_topics:
            print(f"    ! subscriptions.'{node_key}'.'{topic_key}'")

    from colcon_lf.exec_config.resolve import print_warnings, resolve

    resolved = resolve(config, graph)
    print_warnings(resolved.warnings)

    if show:
        _print_resolved_table(resolved, config)

    if interactive:
        from colcon_lf.exec_config.interactive import run_interactive

        run_interactive(result.sparse_path, graph)

    return 0


def _print_resolved_table(resolved, config) -> None:
    """Print the resolved effective table (post-merge, pre-env).

    Every value names the layer that set it: ``built-in``, the
    package-owned ``lf/<launch>.execution.yaml``, or the user file, with the
    scope it was set at. Set env overrides are listed last.
    """
    print("  Resolved effective policy (post-merge, pre-env), each value with its layer:")
    print("    defaults:")
    for key in sorted(resolved.defaults):
        print(
            f"      {key}: {json.dumps(resolved.defaults[key])}"
            f"  <- {resolved.default_origins.get(key, 'built-in')}"
        )
    for row in resolved.subscriptions:
        print(f"    {row.node_class}.{row.topic}:")
        for key in sorted(row.values):
            print(
                f"      {key}: {json.dumps(row.values[key], sort_keys=True)}"
                f"  <- {row.origins.get(key, 'built-in')}"
            )
    for label, items, key in (
        ("external_topics", config.external_topics, "external_topics"),
        ("acknowledged_hazards", config.acknowledged_hazards, "acknowledged_hazards"),
    ):
        print(f"    {label}:" + ("" if items else " []"))
        for item in items:
            print(f"      - {item}  <- {config.origin((key, item))}")
            anchor = config.external_anchors.get(item) if key == "external_topics" else None
            if anchor is None:
                continue
            # Every constant of a stamp anchor with its origin,
            # so a default of 0 / 1 is never silent.
            origin = config.origin((key, item, "anchor"))
            print(f"        anchor: {anchor['anchor']}  <- {origin}")
            for sub in ("delay_ns", "ingress_reserve_ns", "grid_ns", "basis"):
                if sub in anchor:
                    print(f"        {sub}: {json.dumps(anchor[sub])}  <- {origin}")
    # The binding reads these per key at startup (take_mode.cpp). An admitted
    # correspondence edge gets its timeout from the generated binding header and
    # reads the env var only when that value is negative, which the generator
    # never emits (corr_runtime.cpp, add_edge call), so the timeout override
    # reaches legacy subscriptions only.
    if os.environ.get("ROS2_LF_TAKE_MODE"):
        print(
            f"    env ROS2_LF_TAKE_MODE={os.environ['ROS2_LF_TAKE_MODE']} overrides "
            f"that key for every subscription at runtime"
        )
    line = maxwait_env_line(os.environ)
    if line:
        print(f"    {line}")


def _binding_maxwait_ms(value: str) -> int:
    """The value the binding's parse_take_timeout_ms reads (take_mode.cpp)."""
    match = re.fullmatch(r"\s*([+-]?\d+)", value)
    if match is None or int(match.group(1)) < 0:
        return 100
    return int(match.group(1))


def maxwait_env_line(environ) -> str | None:
    """The configure line for ROS2_LF_MAXWAIT_MS and its alias.

    Mirrors the binding's maxwait_env() (take_mode.cpp): an empty variable counts
    as unset, and two set variables whose values disagree refuse the override.
    """
    primary = environ.get("ROS2_LF_MAXWAIT_MS") or ""
    alias = environ.get("ROS2_LF_TAKE_TIMEOUT_MS") or ""
    if not primary and not alias:
        return None
    if primary and alias and _binding_maxwait_ms(primary) != _binding_maxwait_ms(alias):
        return (
            f"env ROS2_LF_MAXWAIT_MS={primary} and its alias "
            f"ROS2_LF_TAKE_TIMEOUT_MS={alias} disagree: the binding refuses the "
            f"maxwait override, so every subscription keeps its configured maxwait_ms"
        )
    if primary and alias:
        source = f"ROS2_LF_MAXWAIT_MS={primary} (and its alias ROS2_LF_TAKE_TIMEOUT_MS={alias})"
    elif primary:
        source = f"ROS2_LF_MAXWAIT_MS={primary}"
    else:
        source = f"ROS2_LF_TAKE_TIMEOUT_MS={alias} (alias of ROS2_LF_MAXWAIT_MS)"
    return (
        f"env {source} overrides maxwait_ms at runtime for legacy subscriptions "
        f"only. An admitted correspondence edge keeps its generated timeout, so "
        f"set maxwait_ms in execution.yaml and regenerate to change it"
    )


# ---------------------------------------------------------------------------
# Discovery (verb/generate.py aliases _find_system_jsons; _resolve_workspace is verb/workspace.py)
# ---------------------------------------------------------------------------

def _find_system_jsons(
    system_json_arg: str | None,
    workspace_arg: str | None,
    package_arg: str | None = None,
) -> list[Path]:
    """Locate system.json files to process.

    Precedence: ``--system-json`` (exact file) > ``--package`` (all of that
    package's launch-stem analyses) > discover everything under the workspace's
    ``lf-analysis/``.  ``--system-json`` and ``--package`` are mutually
    exclusive at the argparse layer (see ``add_package_argument``), so the
    ordering here only matters for direct callers.
    """
    if system_json_arg is not None:
        p = Path(system_json_arg)
        if p.is_file():
            return [p]
        return []

    workspace = _resolve_workspace(workspace_arg)
    if workspace is None:
        return []

    base = workspace / "lf-analysis"
    if not base.is_dir():
        return []

    if package_arg is not None:
        return find_package_system_jsons(base, package_arg)

    return sorted(base.rglob("system.json"))


# ---------------------------------------------------------------------------
# `--package` convenience scoping
#
# `colcon lf-analyze` writes to `lf-analysis/{package}/{launch_stem}/` (see
# verb/analyze.py `_output_path`), so a package's analyses are exactly the
# `system.json` files one level below `lf-analysis/{package}/`.  Both
# lf-configure and lf-generate import these helpers, and `colcon lf`'s chain
# routes its own `--package` through them, so the resolution rule lives in
# exactly one place.
# ---------------------------------------------------------------------------

def find_package_system_jsons(analysis_root: Path, package: str) -> list[Path]:
    """All of *package*'s launch-stem analyses under *analysis_root*.

    Empty list when the package has no analysis directory or it holds no
    ``system.json`` — callers distinguish that from "nothing analyzed at all"
    via ``package_scope_error``.
    """
    pkg_dir = analysis_root / package
    if not pkg_dir.is_dir():
        return []
    return sorted(pkg_dir.glob("*/system.json"))


def package_scope_error(package: str, workspace: Path | None) -> str:
    """The error message for a ``--package`` that resolved to no analyses.

    Deliberately distinct from the generic "No system.json files found": the
    fix is to analyze *this package*, not to wonder whether anything was
    analyzed at all.
    """
    where = f"{workspace / 'lf-analysis' / package}" if workspace else f"lf-analysis/{package}"
    return (
        f"Error: No analyses found for package {package!r} (looked in {where}). "
        f"Run 'colcon lf-analyze --package {package}' first."
    )


def add_package_argument(parser) -> None:
    """Add the ``--package`` flag, mutually exclusive with ``--system-json``.

    Called *instead of* a bare ``--system-json`` ``add_argument`` so argparse
    itself rejects the combination, rather than the verb hand-rolling the
    check after the fact.
    """
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--system-json",
        default=None,
        help=(
            "Path to a system.json file. If omitted, discovers all "
            "system.json files under the workspace lf-analysis/ directory."
        ),
    )
    group.add_argument(
        "--package",
        default=None,
        help=(
            "ROS2 package to process: resolves to every analysis under "
            "lf-analysis/<package>/ (all launch stems). Convenience "
            "alternative to --system-json; the two are mutually exclusive."
        ),
    )


def report_package_scope(package: str | None, system_jsons: list[Path]) -> None:
    """Print what a ``--package`` scope matched, so fan-out is never silent."""
    if package is None:
        return
    n = len(system_jsons)
    noun = "analysis" if n == 1 else "analyses"
    stems = ", ".join(sorted(sj.parent.name for sj in system_jsons))
    print(f"==> package {package!r}: {n} {noun} [{stems}]")

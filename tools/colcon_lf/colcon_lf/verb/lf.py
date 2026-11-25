# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Colcon verb: ``colcon lf`` — all-in-one: analyze → generate → build.

Runs the full LF pipeline for the workspace or a specific package.

Usage::

    colcon lf                         # All packages
    colcon lf --package pubsub_example  # One package
"""

from __future__ import annotations

import sys
from pathlib import Path

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


class LfVerb(VerbExtensionPoint):
    """Run the full LF pipeline: analyze, generate, build."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            "--package",
            default=None,
            help="Process only this package.",
        )
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )
        parser.add_argument(
            "--launch-arg",
            action="append",
            default=[],
            metavar="NAME:=VALUE",
            help="Launch argument value for lf-analyze (repeatable); see colcon lf-analyze.",
        )
        parser.add_argument(
            "--after-delay",
            default=None,
            help=(
                "Default after delay for connections, e.g. '5 msec'. Overrides "
                "execution.yaml's defaults.after_delay (default: '0', one microstep)."
            ),
        )
        parser.add_argument(
            "--fast",
            action="store_true",
            default=False,
            help="Fast mode, '-f true' in the launch files; see colcon lf-generate --fast.",
        )
        parser.add_argument(
            "--timeout",
            default=None,
            metavar="<LF time>",
            help="Emit 'timeout: <LF time>'; see colcon lf-generate --timeout.",
        )
        parser.add_argument(
            "--federate",
            default=None,
            metavar="{per-process,per-node,<path>.yaml}",
            help=(
                "Generate federated artifacts instead of single-process ones; "
                "see colcon lf-generate --federate."
            ),
        )
        parser.add_argument(
            "--coordination",
            default=None,
            choices=["centralized", "decentralized"],
            help="Coordination mode for --federate output; see colcon lf-generate --coordination.",
        )

    def main(self, *, context) -> int:  # noqa: D102
        args = context.args

        # Step 1: Analyze
        _banner("Step 1/4: colcon lf-analyze", first=True)
        rc = self._run_analyze(args)
        if rc != 0:
            _flush()
            print("Analysis failed.", file=sys.stderr)
            _flush()
            return rc

        # Step 2: Configure (scaffold-only — never blocks the chain)
        _banner("Step 2/4: colcon lf-configure (scaffold)")
        rc = self._run_configure(args)
        if rc != 0:
            _flush()
            print("Configure (scaffold) failed.", file=sys.stderr)
            _flush()
            return rc

        # Step 3: Generate. A refused or failed system does not stop the others:
        # the systems that did generate are built, and the exit code stays
        # non-zero.
        _banner("Step 3/4: colcon lf-generate")
        gen_rc = self._run_generate(args)
        if gen_rc != 0:
            _flush()
            print("Generation failed for at least one system.", file=sys.stderr)
            _flush()

        # Step 4: Build the generated lf_generated_* packages
        _banner("Step 4/4: colcon lf-build")
        build_rc = self._run_build(args)
        if build_rc != 0:
            _flush()
            print("Build failed.", file=sys.stderr)
            _flush()

        rows = _system_rows(args)
        _print_table(rows)
        rc = 1 if (gen_rc != 0 or build_rc != 0) else 0
        if rc == 0:
            print()
            print("=" * 60)
            print(
                "LF pipeline complete. Use 'ros2 lf list' to see available systems, "
                "'ros2 lf launch <package> <stem>' to run one."
            )
            print("=" * 60)
        return rc

    def _run_analyze(self, args) -> int:
        from colcon_lf.verb.analyze import AnalyzeVerb

        analyze = AnalyzeVerb()

        class FakeContext:
            pass

        ctx = FakeContext()
        ctx.args = _make_namespace(
            package=args.package,
            workspace=args.workspace,
            launch_file=None,
            format="json",
            output_dir=None,
            viz_only=False,
            launch_only=False,
            launch_arg=list(getattr(args, "launch_arg", None) or []),
        )
        return analyze.main(context=ctx)

    def _run_configure(self, args) -> int:
        # Scaffold-only: calls the shared function directly, not a
        # FakeContext shim (verb/generate.py's discovery helper is reused
        # so --package-scoped analyze output is picked up the same way
        # lf-generate would find it).
        from colcon_lf.verb.configure import (
            _find_system_jsons,
            configure_one,
            report_package_scope,
        )

        system_jsons = _find_system_jsons(None, args.workspace, args.package)
        if not system_jsons:
            # Nothing analyzed yet is not fatal here; lf-generate will
            # report the same condition next and fail the chain there.
            return 0

        report_package_scope(args.package, system_jsons)

        failures = 0
        for sj in system_jsons:
            try:
                rc = configure_one(system_json=sj, scaffold_only=True)
            except Exception as exc:
                print(f"  Error: {exc}", file=sys.stderr)
                rc = 1
            if rc != 0:
                failures += 1
        return 1 if failures else 0

    def _run_generate(self, args) -> int:
        from colcon_lf.verb.generate import GenerateVerb

        gen = GenerateVerb()

        class FakeContext:
            pass

        ctx = FakeContext()
        # Every arg `GenerateVerb.main` reads must be set here: the FakeContext
        # shim bypasses argparse, so a missing attribute is an AttributeError
        # at runtime, not a defaulted flag. `federate`/`target_lfc`/`force`/
        # `emit_federation_template`/`exec_config` were once missing here —
        # `args.federate` is the first line main() touches, so the whole
        # `colcon lf` chain raised instead of generating. Defaults below match
        # each flag's argparse default in verb/generate.py.
        ctx.args = _make_namespace(
            system_json=None,
            package=args.package,
            output_dir=None,
            workspace=args.workspace,
            after_delay=args.after_delay,
            launch=True,
            target_lfc=False,  # deprecated no-op: lfc is the only path
            federate=getattr(args, "federate", None),
            coordination=getattr(args, "coordination", None),
            emit_federation_template=False,
            force=False,
            exec_config=None,
            params_file=[],  # resolved-model gate; matches argparse default
            fast=bool(getattr(args, "fast", False)),
            timeout=getattr(args, "timeout", None),
        )
        return gen.main(context=ctx)

    def _run_build(self, args) -> int:
        from colcon_lf.verb.build import BuildVerb

        build = BuildVerb()

        class FakeContext:
            pass

        ctx = FakeContext()
        ctx.args = _make_namespace(
            package=args.package,
            workspace=args.workspace,
        )
        return build.main(context=ctx)


def _make_namespace(**kwargs):
    """Create a simple namespace object with the given attributes."""
    from types import SimpleNamespace

    return SimpleNamespace(**kwargs)


def _flush() -> None:
    """Flush both streams so a banner and a message keep their order."""
    sys.stdout.flush()
    sys.stderr.flush()


def _banner(title: str, first: bool = False) -> None:
    _flush()
    if not first:
        print()
    print("=" * 60)
    print(title)
    print("=" * 60)
    _flush()


def _system_rows(args) -> list[tuple[str, str, str, str]]:
    """``(system, generation, built, reason)`` per analyzed or skipped system."""
    import json

    from colcon_lf.verb.analyze import _marked_launch_files
    from colcon_lf.verb.build import _failed_packages, find_generated_packages
    from colcon_lf.verb.workspace import resolve_workspace

    workspace = resolve_workspace(args.workspace)
    if workspace is None:
        return []
    package = args.package
    generated_pkgs = find_generated_packages(workspace, package)
    failed_pkgs = set(_failed_packages(workspace, generated_pkgs))
    rows: list[tuple[str, str, str, str]] = []
    analysis = workspace / "lf-analysis"
    for system_json in sorted(analysis.glob("*/*/system.json")):
        system_dir = system_json.parent
        pkg, stem = system_dir.parent.name, system_dir.name
        if package and pkg != package:
            continue
        status, reason = "generated", ""
        status_file = system_dir / "generate_status.json"
        try:
            record = json.loads(status_file.read_text(encoding="utf-8"))
            status = record.get("status", "failed")
            reason = " ".join(filter(None, [record.get("code"), record.get("reason")]))
        except (OSError, ValueError):
            status, reason = "not generated", "no generate record"
        built = "no"
        if status == "generated":
            marker = system_dir / "lfc_packages.txt"
            names = marker.read_text(encoding="utf-8").split() if marker.is_file() else []
            base = names[0] if names else ""
            members = [base] if base else []
            if len(names) > 1 and names[1] == "federated":
                members += [p.name for p in (workspace / "lf-generated").glob(f"{base}_*")]
            if members and all((workspace / "install" / m / "share" / m / "package.xml").is_file()
                           for m in members) \
                    and not failed_pkgs.intersection(members):
                built = "yes"
            elif failed_pkgs.intersection(members):
                reason = "build failed: " + " ".join(sorted(failed_pkgs.intersection(members)))
            else:
                reason = reason or "not built"
        rows.append((f"{pkg}/{stem}", status, built, reason))
    src_root = workspace / "src"
    if src_root.is_dir():
        for pkg, rel in _marked_launch_files(src_root, package):
            rows.append((f"{pkg}/{rel}", "skipped (not-a-system)", "no", "marked rcllf: not-a-system"))
    return rows


def _print_table(rows: list[tuple[str, str, str, str]]) -> None:
    _flush()
    print()
    if not rows:
        return
    headers = ("System", "Generation", "Built", "Reason")
    widths = [max(len(r[i]) for r in [headers] + rows) for i in range(3)]
    fmt = "{:<%d}  {:<%d}  {:<%d}  {}" % tuple(widths)
    print(fmt.format(*headers))
    for row in rows:
        reason = row[3] if len(row[3]) <= 110 else row[3][:107] + "..."
        print(fmt.format(*row[:3], reason).rstrip())
    _flush()

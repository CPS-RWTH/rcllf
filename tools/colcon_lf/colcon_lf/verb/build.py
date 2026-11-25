# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Colcon verb: ``colcon lf-build`` — build the generated lfc packages.

``colcon lf-generate`` writes one ``lf_generated_<name>`` colcon package per
analyzed system into ``<workspace>/lf-generated/`` (plus
``lf_generated_<name>_federated*`` packages for ``--federate``). This verb
builds them with ``colcon build --packages-select`` from the workspace root,
so they land in the workspace's own install space next to
``ros2_lf_binding``.

Usage::

    colcon lf-build                           # Build every generated package
    colcon lf-build --package pubsub_example  # Only those generated from one package
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from colcon_lf.verb.workspace import resolve_workspace as _resolve_workspace


class BuildVerb(VerbExtensionPoint):
    """Build the generated lf_generated_* packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            "--package",
            default=None,
            help="Only build the packages generated from this source package.",
        )
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )

    def main(self, *, context) -> int:  # noqa: D102
        args = context.args
        workspace = _resolve_workspace(args.workspace)
        if workspace is None:
            print("Error: Could not find workspace.", file=sys.stderr)
            return 1

        # Packages generated before the emitted CMake re-checked the
        # lfc runtime build without a check; refuse an unpatched runtime here too.
        from colcon_lf.generation.lfc_runtime import check_lfc_runtime, lfc_runtime_error
        runtime = check_lfc_runtime(workspace)
        if runtime["status"] == "failed":
            print(lfc_runtime_error(runtime), file=sys.stderr)
            return 1
        if runtime["status"] == "skipped":
            print(f"Note: lfc runtime not checked: {runtime['reason']}", file=sys.stderr)

        packages = find_generated_packages(workspace, args.package)
        if not packages:
            if args.package:
                print(
                    f"Error: no generated lf_generated_* package found for package "
                    f"'{args.package}'. Run 'colcon lf-generate --package {args.package}' first.",
                    file=sys.stderr,
                )
                return 1
            print("No generated lf_generated_* packages found to build.")
            return 0

        print(f"==> Building {len(packages)} generated package(s): {' '.join(packages)}\n")
        rc = _colcon_build(workspace, packages)
        if rc != 0:
            failed = _failed_packages(workspace, packages)
            built = [p for p in packages if p not in failed]
            print("\n==> colcon build of the generated packages failed", file=sys.stderr)
            if failed:
                print(f"    failed: {' '.join(failed)}", file=sys.stderr)
                print(f"    built:  {' '.join(built) or '-'}", file=sys.stderr)
            return 1
        print(f"\n==> All {len(packages)} generated package(s) built")
        return 0


def find_generated_packages(workspace: Path, package_filter: str | None) -> list[str]:
    """Names of the ``lf_generated_*`` packages generated from analyzed systems.

    Per analyzed system ``lf-analysis/<pkg>/<stem>/``, lf-generate records the
    package it wrote in ``lfc_packages.txt`` (``<base> single|federated``). A
    federated record also selects the per-federate ``<base>_<federate>``
    packages. Without a record (older output), the ``lf/<Name>.lf`` next to
    system.json names ``lf_generated_<name>``. Only packages that exist under
    ``lf-generated/`` are returned.
    """
    analysis = workspace / "lf-analysis"
    generated = workspace / "lf-generated"
    if not analysis.is_dir() or not generated.is_dir():
        return []
    names: list[str] = []

    def add(path: Path) -> None:
        if (path / "package.xml").is_file() and path.name not in names:
            names.append(path.name)

    for system_json in sorted(analysis.glob("*/*/system.json")):
        system_dir = system_json.parent
        if package_filter and system_dir.parent.name != package_filter:
            continue
        status_file = system_dir / "generate_status.json"
        if status_file.is_file():
            try:
                if json.loads(status_file.read_text(encoding="utf-8")).get(
                        "status") in ("refused", "failed"):
                    continue  # the last generate run produced nothing for it
            except (OSError, ValueError):
                pass
        marker = system_dir / "lfc_packages.txt"
        if marker.is_file():
            parts = marker.read_text(encoding="utf-8").split()
            if not parts:
                continue
            base = parts[0]
            add(generated / base)
            if len(parts) > 1 and parts[1] == "federated":
                for sub in sorted(generated.glob(f"{base}_*")):
                    add(sub)
            continue
        from colcon_lf.generation.lf_naming import (
            generated_lfc_package_name, generated_package_stem,
        )

        source_package = system_dir.parent.name
        for lf_file in sorted((system_dir / "lf").glob("*.lf")):
            add(generated / generated_lfc_package_name(
                source_package, generated_package_stem(lf_file.stem),
            ))
    return names


def _colcon_build(workspace: Path, packages: list[str]) -> int:
    # --continue-on-error: one failing generated package must not stop the
    # independent ones from building (the failure still gives a non-zero rc).
    cmd = ["colcon", "build", "--packages-select", *packages, "--continue-on-error"]
    print("$ " + " ".join(cmd))
    # Flush first: piped to a file, stdout is block-buffered and the banner
    # would land after colcon's own output.
    sys.stdout.flush()
    return subprocess.run(cmd, cwd=workspace).returncode


def _failed_packages(workspace: Path, packages: list[str]) -> list[str]:
    """Packages of *packages* whose job ended non-zero in the last colcon build.

    Reads ``log/latest_build/events.log`` (``(<pkg>) JobEnded: {..., 'rc': N}``).
    """
    events = workspace / "log" / "latest_build" / "events.log"
    try:
        text = events.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    failed: list[str] = []
    for match in re.finditer(r"\(([^)\s]+)\) JobEnded: \{[^}]*'rc': (-?\d+)", text):
        name, rc = match.group(1), int(match.group(2))
        if rc != 0 and name in packages and name not in failed:
            failed.append(name)
    return failed

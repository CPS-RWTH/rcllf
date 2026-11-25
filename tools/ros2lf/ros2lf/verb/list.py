# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Verb: ``ros2 lf list`` — discover available LF-analyzed systems."""

from ros2cli.verb import VerbExtension

from ros2lf.api.discovery import (
    discover_systems,
    find_workspace,
)


class ListVerb(VerbExtension):
    """List available LF-analyzed systems."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )
        parser.add_argument(
            "--package",
            default=None,
            help="Filter by package name.",
        )

    def main(self, *, args):  # noqa: D102
        workspace = find_workspace(getattr(args, "workspace", None))
        if workspace is None:
            print("Error: Could not find workspace. "
                  "Run from a colcon workspace or use --workspace.")
            return 1

        systems = discover_systems(
            workspace, getattr(args, "package", None)
        )

        if not systems:
            print("No LF-analyzed systems found.")
            print("Run 'colcon lf-analyze' first to extract system models.")
            return 0

        # Table header
        fmt = "{:<24} {:<28} {:>5}  {:<12} {:<14}"
        print(fmt.format(
            "Package", "Launch Stem", "Nodes", "Type", "Launch Ready"
        ))
        print("-" * 88)

        for s in systems:
            if s.has_components and not s.has_non_components:
                node_type = "components"
            elif s.has_non_components and not s.has_components:
                node_type = "nodes"
            elif s.has_components and s.has_non_components:
                node_type = "mixed"
            else:
                node_type = "unknown"

            if s.generate_status in ("refused", "failed"):
                ready = (f"refused ({s.refusal_code})" if s.refusal_code
                         else "failed (see: colcon lf-generate)")
            elif not s.has_launch:
                ready = "- (run: colcon lf-generate)"
            elif not s.wrapper_built:
                ready = f"- (run: colcon lf-build --package {s.package})"
            else:
                ready = "ready (federated)" if s.federated else "ready"

            print(fmt.format(
                s.package, s.stem, s.node_count, node_type, ready
            ))

        return 0

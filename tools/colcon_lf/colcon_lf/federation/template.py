# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Emit a starter federation YAML template for a system.

Entry point: ``emit_federation_template(graph, system_json, output_path, force)``
"""

from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph


def emit_federation_template(
    graph: "SystemGraph",
    system_json: Path,
    output_path: Path | None = None,
    force: bool = False,
) -> int:
    """Write a starter federation.yaml next to system.json.

    Returns 0 on success, 1 on error.
    """
    if output_path is None:
        output_path = system_json.parent / "federation.yaml"

    if output_path.exists() and not force:
        print(
            f"  ERROR: {output_path} already exists; pass --force to overwrite.",
            file=sys.stderr,
        )
        return 1

    node_names = sorted(graph.nodes.keys())
    if not node_names:
        print("  WARNING: No nodes found in system.json; template will be empty.")

    from colcon_lf.generation.lf_naming import sanitize_lf_identifier

    date_str = datetime.now().strftime("%Y-%m-%d")
    system_name = system_json.parent.name  # directory name as system identifier

    lines = [
        f"# Federation map for {system_name}",
        f"# Auto-generated {date_str} from {system_json}",
        "# Edit federate names and move nodes between federates as needed.",
        "#",
        "# Rules:",
        "#   - Every node must appear in exactly one federate",
        "#   - Every federate must have at least one node",
        "#   - Federate names must be valid identifiers (a-z, A-Z, 0-9, _)",
        "",
        f"system: {system_name}",
        "",
        "# Default: each node is its own federate (equivalent to --federate per-node).",
        "# Edit by moving nodes under fewer federate names to coarsen the split.",
        "federates:",
    ]

    for node_name in node_names:
        fed_name = sanitize_lf_identifier(node_name)
        lines.append(f"  {fed_name}:")
        lines.append(f"    nodes:")
        lines.append(f"      - {node_name}")
        lines.append("")

    lines += [
        "# Optional: coordination mode for this federation. 'centralized' (default,",
        "# RTI-mediated, never tardy) or 'decentralized' (peer-to-peer, STP-offset",
        "# based, tardy messages are possible). --coordination on the command line",
        "# overrides this. See docs/design/time-and-coordination.md.",
        "# coordination: centralized",
        "",
        "# Optional: per-link cross-federate after-delay overrides. Each value is a",
        "# declared logical-time offset (LF `after`, LET-style contract), not a",
        "# latency measurement; a measured value may inform the choice but does not",
        "# define the semantics. Defaults to 0 when omitted. A unit is required",
        "# (only 0 may omit one): terse (ms, us, ns, s) or LF (msec, usec, nsec, sec).",
        "# 'stp_offset' only applies under decentralized coordination (defaults to",
        "# 10ms when omitted there; ignored under centralized coordination).",
        "# Example:",
        "#   after_overrides:",
        "#     - from: FrontLidarDriver.points",
        "#       to:   PointCloudFusion.front_input",
        "#       after: 8ms",
        "#       stp_offset: 15ms",
        "after_overrides: []",
        "",
    ]

    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"  -> {output_path} (federation template, {len(node_names)} nodes)")
    return 0

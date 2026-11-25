#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Re-generate visualizations from an existing system.json file.

No Docker or ROS2 required — just Python + graphviz.

Usage:
    # Re-visualize a single analysis output
    python3 scripts/reviz.py path/to/system.json

    # Specify output format
    python3 scripts/reviz.py path/to/system.json --format svg

    # Custom output directory
    python3 scripts/reviz.py path/to/system.json --output-dir /tmp/viz
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Add the colcon_lf tool to the Python path.
_TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools" / "colcon_lf"
sys.path.insert(0, str(_TOOLS_DIR))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Re-generate visualizations from system.json",
    )
    parser.add_argument(
        "system_json",
        help="Path to system.json file",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory for viz/ (default: same directory as system.json)",
    )
    parser.add_argument(
        "--format",
        choices=["pdf", "svg", "all"],
        default="all",
        help="Output format (default: all)",
    )
    args = parser.parse_args()

    system_json = Path(args.system_json)
    if not system_json.exists():
        print(f"Error: {system_json} not found", file=sys.stderr)
        return 1

    with open(system_json) as f:
        system_data = json.load(f)

    output_dir = Path(args.output_dir) if args.output_dir else system_json.parent
    viz_dir = output_dir / "viz"

    formats = ["svg"] if args.format == "all" else [args.format]

    from colcon_lf.visualization.dot_renderer import render
    from colcon_lf.visualization.launch_graph_viz import launch_graph_viz
    from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
    from colcon_lf.visualization.system_graph_viz import system_graph_viz
    from colcon_lf.visualization.node_internals_viz import node_internals_viz

    dot = launch_graph_viz(system_data)
    render(dot, viz_dir / "launch_graph", formats=formats)
    print(f"  launch_graph -> {viz_dir}/")

    dot = comm_graph_viz(system_data)
    render(dot, viz_dir / "comm_graph", formats=formats)
    print(f"  comm_graph -> {viz_dir}/")

    dot = system_graph_viz(system_data)
    render(dot, viz_dir / "system_graph", formats=formats)
    print(f"  system_graph -> {viz_dir}/")

    nodes_data = system_data.get("nodes", {})
    for node_key, node_data in nodes_data.items():
        safe_name = node_key.replace("/", "_").lstrip("_")
        dot = node_internals_viz(node_data)
        render(dot, viz_dir / "nodes" / f"{safe_name}_internals", formats=formats)
    print(f"  {len(nodes_data)} node internals -> {viz_dir}/nodes/")

    from colcon_lf.visualization.topology_svg import render_topology_svg
    render_topology_svg(system_data, viz_dir / "topology.svg")
    print(f"  topology -> {viz_dir}/")

    print(f"\nDone. Output in {viz_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())

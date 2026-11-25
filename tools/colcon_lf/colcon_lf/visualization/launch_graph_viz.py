# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Launch file hierarchy visualization — DAG from root to leaf nodes."""

from __future__ import annotations

from typing import Any

import graphviz


def launch_graph_viz(system_data: dict[str, Any]) -> graphviz.Digraph:
    """Build a Digraph showing the launch file include hierarchy.

    Parameters
    ----------
    system_data:
        Parsed ``system.json`` dict.
    """
    dot = graphviz.Digraph("launch_graph", graph_attr={"rankdir": "TB"})
    dot.attr("node", shape="box", style="rounded")

    root = system_data.get("launch_graph", {}).get("root", {})
    if root:
        _walk_launch(dot, root, parent_id=None, counter=[0])

    return dot


def _walk_launch(
    dot: graphviz.Digraph,
    file_info: dict[str, Any],
    parent_id: str | None,
    counter: list[int],
) -> None:
    """Recursively add launch file nodes and their children."""
    file_path = file_info.get("file_path", "unknown")
    package = file_info.get("package", "?")
    errors = file_info.get("errors", [])
    # Use just the filename for display
    filename = file_path.rsplit("/", 1)[-1] if "/" in file_path else file_path
    node_id = f"launch_{counter[0]}"
    counter[0] += 1

    if errors:
        # Show failed/unresolved launch files as red dashed boxes
        error_text = errors[0][:60] if errors else ""
        label = f"{package}\\n{filename}\\n{error_text}"
        dot.node(node_id, label, shape="box", style="dashed,filled", fillcolor="lightsalmon", color="red")
    else:
        dot.node(node_id, f"{package}\\n{filename}", shape="box", style="rounded,filled", fillcolor="lightblue")

    if parent_id is not None:
        edge_attrs: dict[str, str] = {"label": "includes"}
        if errors:
            edge_attrs["style"] = "dashed"
            edge_attrs["color"] = "red"
        dot.edge(parent_id, node_id, **edge_attrs)

    # Direct nodes launched by this file (not inside any group)
    for node in file_info.get("nodes", []):
        _render_node(dot, node, parent_id=node_id, counter=counter)

    # Included launch files
    for inc in file_info.get("includes", []):
        _walk_launch(dot, inc, parent_id=node_id, counter=counter)

    # Groups — rendered hierarchically as subgraphs
    for group in file_info.get("groups", []):
        _walk_group(dot, group, parent_id=node_id, counter=counter)


def _walk_group(
    dot: graphviz.Digraph,
    group: dict[str, Any],
    parent_id: str,
    counter: list[int],
) -> None:
    """Render a GroupAction and all its children under *parent_id*."""
    ns = group.get("namespace") or ""
    cond = group.get("condition")
    g_id = f"group_{counter[0]}"
    counter[0] += 1

    label = "GroupAction"
    if ns:
        label += f"\\nns={ns}"
    if cond:
        label += f"\\n[{cond}]"
    dot.node(g_id, label, shape="box", style="dashed,filled", fillcolor="lightyellow")
    dot.edge(parent_id, g_id)

    # Nodes owned by this group
    for node in group.get("nodes", []):
        _render_node(dot, node, parent_id=g_id, counter=counter)

    # Includes owned by this group
    for inc in group.get("includes", []):
        _walk_launch(dot, inc, parent_id=g_id, counter=counter)

    # Nested groups
    for child_group in group.get("groups", []):
        _walk_group(dot, child_group, parent_id=g_id, counter=counter)


def _render_node(
    dot: graphviz.Digraph,
    node: dict[str, Any],
    parent_id: str,
    counter: list[int],
) -> None:
    """Add a single ROS node ellipse and edge to *parent_id*."""
    n_id = f"node_{counter[0]}"
    counter[0] += 1
    pkg = node.get("package", "?")
    exe = node.get("executable", "?")
    name = node.get("name", exe)
    label = f"{pkg}\\n{name}"
    if node.get("condition"):
        label += f"\\n[{node['condition']}]"
    dot.node(n_id, label, shape="ellipse", style="filled", fillcolor="lightyellow")
    dot.edge(parent_id, n_id)

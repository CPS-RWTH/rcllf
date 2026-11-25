# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Write SystemGraph analysis outputs to the filesystem.

Output directory structure::

    {output_dir}/
      system.json                 # Full SystemGraph
      nodes/
        {node_name}.json          # Individual NodeInfo per node
      config/
        {filename}.yaml           # Copied parameter YAML files
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

from colcon_lf.graph.models import SystemGraph
from colcon_lf.output.serializer import serialize


# Characters unsafe in filenames across platforms (Windows + POSIX)
_UNSAFE_CHARS = re.compile(r'[<>:"|?*\x00-\x1f]')


def _sanitize_filename(name: str) -> str:
    """Sanitize a node key for use as a filename."""
    safe = name.replace("/", "_").lstrip("_")
    safe = _UNSAFE_CHARS.sub("_", safe)
    return safe or "unnamed"


def _sanitize_config_filename(key: str) -> str:
    """Map a ``SystemGraph.parameter_files`` key to a safe, flat filename
    under ``config/``.

    *key* is a workspace-relative (or, rarely, absolute) POSIX path
    (``graph/hierarchical.py``'s ``_parameter_file_key``); every path
    separator becomes ``__`` so two files sharing a basename in different
    directories keep distinct, collision-free names here too (the same
    collision the key itself was fixed to avoid). Critically, the result
    never contains a path separator, so joining it under ``config_dir``
    below can never escape that directory (`config_path = config_dir /
    filename` previously discarded ``config_dir`` entirely for an absolute
    ``filename`` and overwrote the real source file — the bug this
    replaces). Never collides with the reserved ``execution.yaml`` name.
    """
    flat = key.replace("\\", "/").replace("/", "__").lstrip("_")
    flat = _UNSAFE_CHARS.sub("_", flat)
    if not flat.lower().endswith((".yaml", ".yml")):
        flat += ".yaml"
    if flat == "execution.yaml":
        flat = "_" + flat
    return flat or "unnamed.yaml"


def write(graph: SystemGraph, output_dir: str | Path) -> Path:
    """Write a SystemGraph to *output_dir* as JSON files.

    Creates the directory structure and returns the output path.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- system.json ---
    system_path = output_dir / "system.json"
    system_path.write_text(serialize(graph))

    # --- nodes/{name}.json ---
    nodes_dir = output_dir / "nodes"
    nodes_dir.mkdir(exist_ok=True)
    for node_key, node_info in graph.nodes.items():
        safe_name = _sanitize_filename(node_key)
        node_path = nodes_dir / f"{safe_name}.json"
        node_path.write_text(serialize(node_info))

    # --- config/{filename}.yaml ---
    if graph.parameter_files:
        config_dir = output_dir / "config"
        config_dir.mkdir(exist_ok=True)
        resolved_config_dir = config_dir.resolve()
        seen_names: dict[str, str] = {}  # safe filename -> original key
        for filename, content in graph.parameter_files.items():
            safe_name = _sanitize_config_filename(filename)
            if safe_name in seen_names and seen_names[safe_name] != filename:
                raise ValueError(
                    f"config filename collision writing analysis output: "
                    f"{filename!r} and {seen_names[safe_name]!r} both map to "
                    f"{safe_name!r}"
                )
            seen_names[safe_name] = filename
            config_path = config_dir / safe_name
            # Defense in depth: _sanitize_config_filename never emits a path
            # separator, so this can't actually escape config_dir, but a
            # silent escape here is exactly the class of bug being fixed,
            # so it is checked rather than assumed.
            if config_path.resolve().parent != resolved_config_dir:
                raise ValueError(
                    f"refusing to write parameter file outside config_dir: "
                    f"{filename!r} -> {config_path}"
                )
            config_path.write_text(yaml.dump(content, default_flow_style=False))

    return output_dir

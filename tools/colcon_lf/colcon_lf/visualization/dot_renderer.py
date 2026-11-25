# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Render graphviz Digraph objects to PDF/SVG files."""

from __future__ import annotations

from pathlib import Path

import graphviz


def render(
    graph: graphviz.Digraph,
    output_path: str | Path,
    formats: list[str] | None = None,
) -> list[Path]:
    """Render *graph* to one or more file formats.

    Parameters
    ----------
    graph:
        A graphviz ``Digraph`` (or ``Graph``) object.
    output_path:
        Base output path **without** extension (e.g. ``viz/launch_graph``).
    formats:
        List of output formats, e.g. ``["pdf", "svg"]``.
        Defaults to ``["pdf", "svg"]``.

    Returns
    -------
    List of rendered file paths.
    """
    if formats is None:
        formats = ["pdf", "svg"]

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rendered: list[Path] = []
    for fmt in formats:
        graph.format = fmt
        out = graph.render(
            filename=output_path.stem,
            directory=str(output_path.parent),
            cleanup=True,
        )
        rendered.append(Path(out))

    return rendered

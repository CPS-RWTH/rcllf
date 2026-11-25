# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for topology_svg.py — correctness and collision detection.

The collision test works by:
1. Rendering a topology SVG from a multi-node system.json fixture.
2. Parsing all inter-node routing path elements.
3. Extracting vertical segments (x stays constant, y changes).
4. Grouping segments by their x coordinate.
5. Asserting no two segments at the same x have overlapping y-ranges.

This catches regressions in the gap-based interval lane assignment.
"""
from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from colcon_lf.visualization.topology_svg import render_topology_svg

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

COMBO_PIPELINE = {
    "name": "combo_pipeline",
    "nodes": {
        "timer_publisher": {
            "name": "timer_publisher",
            "class_name": "TimerPublisherNode",
            "package": "basic_components",
            "timers": [{"period_ms": 1000, "callback": "on_timer"}],
            "subscribers": [],
            "publishers": [{"topic": "/basic_components/timer_out"}],
            "callbacks": [{"name": "on_timer", "publishers_used": ["/basic_components/timer_out"]}],
        },
        "relay_node": {
            "name": "relay_node",
            "class_name": "RelayNode",
            "package": "basic_components",
            "timers": [],
            "subscribers": [{"topic": "/basic_components/timer_out", "callback": "on_message"}],
            "publishers": [{"topic": "/basic_components/relay_out"}],
            "callbacks": [{"name": "on_message", "publishers_used": ["/basic_components/relay_out"]}],
        },
        "listener_node": {
            "name": "listener_node",
            "class_name": "ListenerNode",
            "package": "basic_components",
            "timers": [],
            "subscribers": [{"topic": "/basic_components/relay_out", "callback": "on_message"}],
            "publishers": [],
            "callbacks": [{"name": "on_message", "publishers_used": []}],
        },
    },
    "communication": {
        "topics": [
            {
                "topic": "/basic_components/timer_out",
                "publishers": ["timer_publisher"],
                "subscribers": ["relay_node"],
            },
            {
                "topic": "/basic_components/relay_out",
                "publishers": ["relay_node"],
                "subscribers": ["listener_node"],
            },
        ],
        "services": [],
        "actions": [],
    },
}

# A fanout fixture: one publisher → many subscribers (exercises multi-edge gaps)
FANOUT = {
    "name": "fanout",
    "nodes": {
        "source": {
            "name": "source",
            "class_name": "Source",
            "package": "pkg",
            "timers": [{"period_ms": 500, "callback": "tick"}],
            "subscribers": [],
            "publishers": [
                {"topic": "/pkg/a"},
                {"topic": "/pkg/b"},
                {"topic": "/pkg/c"},
            ],
            "callbacks": [{"name": "tick", "publishers_used": ["/pkg/a", "/pkg/b", "/pkg/c"]}],
        },
        "sink_a": {
            "name": "sink_a",
            "class_name": "SinkA",
            "package": "pkg",
            "timers": [],
            "subscribers": [{"topic": "/pkg/a", "callback": "on_a"}],
            "publishers": [],
            "callbacks": [{"name": "on_a", "publishers_used": []}],
        },
        "sink_b": {
            "name": "sink_b",
            "class_name": "SinkB",
            "package": "pkg",
            "timers": [],
            "subscribers": [{"topic": "/pkg/b", "callback": "on_b"}],
            "publishers": [],
            "callbacks": [{"name": "on_b", "publishers_used": []}],
        },
        "sink_c": {
            "name": "sink_c",
            "class_name": "SinkC",
            "package": "pkg",
            "timers": [],
            "subscribers": [{"topic": "/pkg/c", "callback": "on_c"}],
            "publishers": [],
            "callbacks": [{"name": "on_c", "publishers_used": []}],
        },
    },
    "communication": {
        "topics": [
            {"topic": "/pkg/a", "publishers": ["source"], "subscribers": ["sink_a"]},
            {"topic": "/pkg/b", "publishers": ["source"], "subscribers": ["sink_b"]},
            {"topic": "/pkg/c", "publishers": ["source"], "subscribers": ["sink_c"]},
        ],
        "services": [],
        "actions": [],
    },
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _render(system_data: dict, tmp_path: Path) -> str:
    """Render system_data to SVG and return the SVG text."""
    out = tmp_path / "topology.svg"
    render_topology_svg(system_data, out)
    return out.read_text()


def _parse_routing_paths(svg_text: str) -> list[str]:
    """Return the 'd' attribute of every <path> that is an inter-node edge.

    Inter-node routing paths are identified by:
    - fill="none" (not chevrons which have fill="rgb(166,166,166)")
    - no stroke-dasharray (internal reaction→port connections use dashes)
    - at least one L command
    """
    root = ET.fromstring(svg_text)
    paths = []
    for elem in root.iter('{http://www.w3.org/2000/svg}path'):
        d = elem.get('d', '')
        fill = elem.get('fill', 'none')
        if fill != 'none':
            continue
        # Dashed paths are intra-node (reaction↔port connections), skip them
        if elem.get('stroke-dasharray'):
            continue
        if 'L' not in d:
            continue
        paths.append(d)
    return paths


def _extract_vertical_segments(d: str) -> list[tuple[float, float, float]]:
    """Extract vertical segments (x, y_start, y_end) from an SVG path 'd' attribute.

    Parses 'M x,y L x2,y2 ...' sequences and returns segments where x is unchanged.
    """
    # Tokenise: split on M and L commands, keep coordinates
    tokens = re.split(r'[ML]', d.replace(' ', ','))
    coords: list[tuple[float, float]] = []
    for tok in tokens:
        tok = tok.strip(',').strip()
        if not tok:
            continue
        parts = tok.split(',')
        if len(parts) >= 2:
            try:
                coords.append((float(parts[0]), float(parts[1])))
            except ValueError:
                pass

    segments = []
    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i + 1]
        if abs(x0 - x1) < 0.5 and abs(y0 - y1) > 1:
            segments.append((round(x0, 1), min(y0, y1), max(y0, y1)))
    return segments


MARGIN = 2.0  # must match topology_svg.py


def _extract_horizontal_segments(d: str) -> list[tuple[float, float, float]]:
    """Extract horizontal segments (y, x_start, x_end) from an SVG path 'd' attribute."""
    tokens = re.split(r'[ML]', d.replace(' ', ','))
    coords: list[tuple[float, float]] = []
    for tok in tokens:
        tok = tok.strip(',').strip()
        if not tok:
            continue
        parts = tok.split(',')
        if len(parts) >= 2:
            try:
                coords.append((float(parts[0]), float(parts[1])))
            except ValueError:
                pass

    segments = []
    for i in range(len(coords) - 1):
        x0, y0 = coords[i]
        x1, y1 = coords[i + 1]
        if abs(y0 - y1) < 0.5 and abs(x0 - x1) > 1:
            segments.append((round(y0, 1), min(x0, x1), max(x0, x1)))
    return segments


def _check_no_overlapping_segments(svg_text: str) -> list[str]:
    """Return collision descriptions for both vertical and horizontal overlaps.

    Vertical overlap: two segments share the same x and have overlapping y-ranges.
    Horizontal overlap: two segments share the same y and have overlapping x-ranges,
    BUT we only flag horizontal overlaps on the arrival side (the last horizontal
    segment of each path), not the departure side (shared wire bundles at source
    are acceptable).
    """
    routing_paths = _parse_routing_paths(svg_text)

    from collections import defaultdict
    collisions = []

    # ── Vertical overlaps ────────────────────────────────────────────────────
    by_x: dict[float, list[tuple[float, float, int]]] = defaultdict(list)
    for path_idx, d in enumerate(routing_paths):
        for x, y_lo, y_hi in _extract_vertical_segments(d):
            by_x[x].append((y_lo, y_hi, path_idx))

    for x, segs in by_x.items():
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                y_lo_i, y_hi_i, pi = segs[i]
                y_lo_j, y_hi_j, pj = segs[j]
                if y_lo_j < y_hi_i - MARGIN and y_lo_i < y_hi_j - MARGIN:
                    if pi != pj:
                        collisions.append(
                            f"VERTICAL x={x}: path[{pi}] y=[{y_lo_i},{y_hi_i}] "
                            f"overlaps path[{pj}] y=[{y_lo_j},{y_hi_j}]"
                        )

    # ── Horizontal overlaps (arrival segments only) ──────────────────────────
    # For each path, the "arrival" horizontal segment is the last one in the path.
    # We only flag overlaps between edges that have vertical segments — purely
    # horizontal edges (direct same-y connections in a linear pipeline) are
    # expected to share y and are not a routing error.
    arrival_segs: dict[float, list[tuple[float, float, int]]] = defaultdict(list)
    for path_idx, d in enumerate(routing_paths):
        h_segs = _extract_horizontal_segments(d)
        v_segs = _extract_vertical_segments(d)
        # Only check arrival overlaps for edges that route through a vertical segment
        if h_segs and v_segs:
            y, x_lo, x_hi = h_segs[-1]  # last horizontal = arrival
            arrival_segs[y].append((x_lo, x_hi, path_idx))

    for y, segs in arrival_segs.items():
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                x_lo_i, x_hi_i, pi = segs[i]
                x_lo_j, x_hi_j, pj = segs[j]
                if x_lo_j < x_hi_i - MARGIN and x_lo_i < x_hi_j - MARGIN:
                    if pi != pj:
                        collisions.append(
                            f"HORIZONTAL y={y}: path[{pi}] x=[{x_lo_i},{x_hi_i}] "
                            f"overlaps path[{pj}] x=[{x_lo_j},{x_hi_j}]"
                        )

    return collisions


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_render_produces_valid_svg(tmp_path):
    """render_topology_svg writes a parseable SVG."""
    svg_text = _render(COMBO_PIPELINE, tmp_path)
    assert svg_text.startswith('<?xml')
    root = ET.fromstring(svg_text)
    assert root.tag == '{http://www.w3.org/2000/svg}svg'


def test_render_empty_nodes(tmp_path):
    """Empty nodes dict produces a placeholder SVG."""
    data = {"nodes": {}, "communication": {"topics": [], "services": [], "actions": []}}
    svg_text = _render(data, tmp_path)
    assert 'no nodes' in svg_text


def test_combo_pipeline_node_count(tmp_path):
    """All three nodes appear in the SVG."""
    svg_text = _render(COMBO_PIPELINE, tmp_path)
    assert 'TimerPublisherNode' in svg_text
    assert 'RelayNode' in svg_text
    assert 'ListenerNode' in svg_text


def test_combo_pipeline_no_edge_collisions(tmp_path):
    """No two inter-node routing segments share the same x and overlap in y."""
    svg_text = _render(COMBO_PIPELINE, tmp_path)
    collisions = _check_no_overlapping_segments(svg_text)
    assert collisions == [], f"Edge collisions detected:\n" + "\n".join(collisions)


def test_fanout_no_edge_collisions(tmp_path):
    """Three edges from one source to three sinks have non-overlapping vertical segments."""
    svg_text = _render(FANOUT, tmp_path)
    collisions = _check_no_overlapping_segments(svg_text)
    assert collisions == [], f"Edge collisions detected:\n" + "\n".join(collisions)


def test_fanout_has_three_routing_paths(tmp_path):
    """Fanout produces 3 inter-node routing paths (one per topic)."""
    svg_text = _render(FANOUT, tmp_path)
    paths = _parse_routing_paths(svg_text)
    # 3 inter-node edges expected; clock-hand paths (non-orthogonal) are excluded
    # by checking that each path has exclusively horizontal/vertical segments
    orthogonal = []
    for d in paths:
        coords = []
        for tok in re.split(r'[ML]', d.replace(' ', ',')):
            tok = tok.strip(',').strip()
            if tok:
                parts = tok.split(',')
                if len(parts) >= 2:
                    try:
                        coords.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        pass
        is_orthogonal = all(
            abs(coords[i][0] - coords[i+1][0]) < 0.5 or abs(coords[i][1] - coords[i+1][1]) < 0.5
            for i in range(len(coords) - 1)
        ) if len(coords) >= 2 else False
        if is_orthogonal:
            orthogonal.append(d)
    assert len(orthogonal) == 3, f"Expected 3 orthogonal routing paths, got {len(orthogonal)}: {orthogonal}"


@pytest.mark.parametrize("rel_path", [
    "packages/lf-analysis/basic_components/combo_pipeline/system.json",
    "packages/lf-analysis/launch_features_example/all_features/system.json",
    "packages/lf-analysis/system_bringup/system_bringup/system.json",
])
def test_real_system_json_no_collisions(tmp_path, rel_path):
    """Render real system.json files and check for no edge collisions."""
    # __file__ is tools/colcon_lf/tests/test_topology_svg.py → up 4 levels = repo root
    real_json = Path(__file__).resolve().parent.parent.parent.parent / rel_path
    if not real_json.exists():
        pytest.skip(f"Real system.json not found: {real_json}")
    with open(real_json) as f:
        data = json.load(f)
    svg_text = _render(data, tmp_path)
    collisions = _check_no_overlapping_segments(svg_text)
    assert collisions == [], f"Edge collisions in {rel_path}:\n" + "\n".join(collisions)


def test_feedback_loop_renders_without_hanging(tmp_path):
    """A -> B -> A used to raise layers forever."""
    import copy
    import threading

    data = copy.deepcopy(COMBO_PIPELINE)
    topics = data["communication"]["topics"]
    back = copy.deepcopy(topics[0])
    back["topic"] = "/feedback"
    back["publishers"], back["subscribers"] = topics[0]["subscribers"][:1], topics[0]["publishers"][:1]
    topics.append(back)
    out: list[str] = []
    worker = threading.Thread(target=lambda: out.append(_render(data, tmp_path)), daemon=True)
    worker.start()
    worker.join(timeout=20)
    assert not worker.is_alive(), "render_topology_svg did not terminate on a feedback loop"
    assert out and out[0].startswith("<?xml")

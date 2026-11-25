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
"""LF reactor-style topology SVG renderer.

Produces a static topology.svg for `colcon lf-analyze` in Docker/CI environments.

This renderer uses a simple BFS layout with interval-based lane assignment.
It works for linear pipelines but has known edge overlap issues on complex
graphs with heavy fan-in/fan-out (e.g. the Autoware reference system).
No further investment planned — use the web UI for complex systems.
"""
from __future__ import annotations

from collections import defaultdict, deque
from pathlib import Path

# ── Constants matching reference SVG exactly ──
NODE_STROKE = 'rgb(190,190,190)'
NODE_FILL = 'rgb(242,242,242)'
NODE_RX = 8
HDR = 32        # header height
ROW_H = 28      # port/reaction row height
PAD = 12        # bottom padding
PORT_W = 8      # port triangle width
PORT_H = 8      # port triangle height
REACT_W = 45    # minimum reaction chevron width
REACT_H = 22    # reaction chevron height
REACT_NOTCH = 6  # left notch depth
TIMER_R = 14.5
TIMER_LABEL_H = 14
TIMER_ROW_H = int(TIMER_R * 2 + TIMER_LABEL_H + 8)  # 51

LAYER_GAP = 180  # horizontal gap between BFS layers
NODE_GAP = 20    # vertical gap between nodes in same layer
PAD_LEFT = 20
PAD_TOP = 20
# Extra canvas margin to account for port labels extending outside node bounds
PORT_LABEL_MARGIN = 160
PORT_LABEL_MAX_CHARS = 22  # truncate long labels to keep diagram readable


def _port_label(t: str) -> str:
    """Return 'namespace/topic', truncated if too long."""
    label = t.strip('/')
    if len(label) > PORT_LABEL_MAX_CHARS:
        label = '…' + label[-(PORT_LABEL_MAX_CHARS - 1):]
    return label


def _node_height(node: dict) -> int:
    timer_h = len(node['timers']) * TIMER_ROW_H
    # Sub reactions after timers: only non-timer-triggered subs need center rows
    timer_cb_set = {t['callback'] for t in node['timers']}
    sub_reaction_h = sum(
        ROW_H for s in node['subscribers']
        if s.get('callback') not in timer_cb_set
    )
    content_h = timer_h + sub_reaction_h
    port_rows = max(len(node['subscribers']), len(node['publishers']))
    port_h = port_rows * ROW_H
    return HDR + max(content_h, port_h, ROW_H) + PAD


def _calc_react_w(node: dict) -> int:
    """Dynamic chevron width to fit order number + callback name."""
    max_cb_len = max((len(c['name']) for c in node['callbacks']), default=0)
    return max(REACT_W, REACT_NOTCH + 14 + max_cb_len * 6 + REACT_NOTCH + 4)


def _node_width(node: dict, node_key: str = '') -> int:
    rw = _calc_react_w(node)
    has_timer = len(node['timers']) > 0
    if has_timer:
        # Must fit: left_pad(8) + timer_diam + gap(16) + react_w + right_pad(16)
        # timer_cx = center_x - rw/2 - TIMER_R - 16
        # timer leftmost = timer_cx - TIMER_R = center_x - rw/2 - 2*TIMER_R - 16
        # Require timer leftmost >= PORT_W + 8 (clear of port triangle)
        # => w/2 - rw/2 - 2*TIMER_R - 16 >= PORT_W + 8
        # => w >= rw + 4*TIMER_R + 48
        center_w = max(int(rw + 4 * TIMER_R + 48), int(TIMER_R * 2 + 16 + rw + 16))
    else:
        center_w = rw + 32
    header = f"{node_key or node['class_name']} : {node['class_name']}"
    header_w = len(header) * 7 + 12
    return max(center_w, header_w, 130)


class _SVG:
    def __init__(self) -> None:
        self.parts: list[str] = []

    def rect(self, x, y, w, h, rx=0, fill=NODE_FILL, stroke=NODE_STROKE, sw=1):
        self.parts.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def line(self, x1, y1, x2, y2, stroke='black', sw=1, dash=''):
        d = f' stroke-dasharray="{dash}"' if dash else ''
        self.parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def path(self, d, stroke='black', sw=1, fill='none', dash=''):
        da = f' stroke-dasharray="{dash}"' if dash else ''
        self.parts.append(
            f'<path d="{d}" stroke="{stroke}" stroke-width="{sw}" fill="{fill}"{da}/>')

    def text(self, x, y, content, anchor='start', size=12, weight=400,
             fill='black', family='sans-serif'):
        self.parts.append(
            f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" '
            f'font-size="{size}" font-weight="{weight}" '
            f'fill="{fill}" font-family="{family}">{content}</text>')

    def polygon(self, pts, fill='black', stroke='black', sw=1):
        self.parts.append(
            f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def ellipse(self, cx, cy, rx, ry, fill=NODE_FILL, stroke='black', sw=1):
        self.parts.append(
            f'<ellipse cx="{cx:.2f}" cy="{cy:.2f}" rx="{rx}" ry="{ry}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def build(self) -> str:
        return '\n'.join(self.parts)


def _draw_chevron(svg: _SVG, chev_x: float, react_y: float, react_w: int,
                  cb_idx: int, cb_name: str) -> None:
    """Draw a reaction chevron with order number and callback name."""
    d = (f'M{chev_x:.1f},{react_y:.1f}'
         f'L{chev_x+react_w-REACT_NOTCH:.1f},{react_y:.1f}'
         f'L{chev_x+react_w:.1f},{react_y+REACT_H/2:.1f}'
         f'L{chev_x+react_w-REACT_NOTCH:.1f},{react_y+REACT_H:.1f}'
         f'L{chev_x:.1f},{react_y+REACT_H:.1f}'
         f'L{chev_x+REACT_NOTCH:.1f},{react_y+REACT_H/2:.1f}Z')
    svg.path(d, stroke='rgb(115,115,115)', fill='rgb(166,166,166)', sw=1)
    mid_y = react_y + REACT_H / 2 + 4
    svg.text(chev_x + REACT_NOTCH + 2, mid_y,
             str(cb_idx + 1), size=10, weight=700, fill='black', family='monospace')
    svg.text(chev_x + REACT_NOTCH + 14, mid_y,
             cb_name, size=8, weight=400, fill='black', family='monospace')


def _render_node(svg: _SVG, node: dict, nx: float, ny: float, node_key: str = '') -> None:
    w = _node_width(node, node_key)
    react_w = _calc_react_w(node)
    center_x = nx + w / 2
    chev_x = center_x - react_w / 2
    timer_cx = chev_x - TIMER_R - 16

    cb_names = [c['name'] for c in node['callbacks']]
    timer_cb_set = {t['callback'] for t in node['timers']}

    # Node box
    svg.rect(nx, ny, w, _node_height(node), rx=NODE_RX)

    # Header: "instance_key : ClassName"
    header_text = f"{node_key or node['class_name']} : {node['class_name']}"
    svg.text(nx + 6, ny + HDR / 2 + 5, header_text, size=12, weight=700)
    svg.line(nx, ny + HDR, nx + w, ny + HDR, stroke=NODE_STROKE)

    def port_cy(i: int) -> float:
        return ny + HDR + i * ROW_H + ROW_H / 2

    # Input ports (left edge, triangle tip at nx pointing right into node)
    for i, sub in enumerate(node['subscribers']):
        cy = port_cy(i)
        pts = f'{nx},{cy-PORT_H//2} {nx+PORT_W},{cy} {nx},{cy+PORT_H//2}'
        svg.polygon(pts)
        svg.text(nx - 2, cy - PORT_H // 2 - 2, _port_label(sub['topic']),
                 anchor='end', size=10, family='monospace')

    # Output ports (right edge, triangle tip at nx+w pointing right out of node)
    for i, pub in enumerate(node['publishers']):
        cy = port_cy(i)
        pts = f'{nx+w-PORT_W},{cy-PORT_H//2} {nx+w+PORT_W//2},{cy} {nx+w-PORT_W},{cy+PORT_H//2}'
        svg.polygon(pts)
        svg.text(nx + w + PORT_W // 2 + 2, cy - PORT_H // 2 - 2, _port_label(pub['topic']),
                 anchor='start', size=10, family='monospace')

    # ── Center column: timers first, then sub reactions ──
    # Align first timer center to port_cy(0) so timer→output lines stay straight
    ey = ny + HDR + ROW_H / 2 - TIMER_R
    for timer in node['timers']:
        timer_cy = ey + TIMER_R
        svg.ellipse(timer_cx, timer_cy, TIMER_R, TIMER_R)
        svg.path(
            f'M{timer_cx:.1f},{timer_cy-TIMER_R*0.8:.1f}'
            f'L{timer_cx:.1f},{timer_cy:.1f}'
            f'L{timer_cx+TIMER_R*0.4:.1f},{timer_cy+TIMER_R*0.4:.1f}',
            sw=1.5)
        period = timer.get('period_ms')
        period_label = f'(0, {int(period)} ms)' if period is not None else '(0, ? ms)'
        svg.text(timer_cx - TIMER_R, timer_cy + TIMER_R + 11,
                 period_label, size=10, family='monospace')

        cb_idx = cb_names.index(timer['callback']) if timer['callback'] in cb_names else -1
        if cb_idx >= 0:
            react_y = timer_cy - REACT_H / 2
            _draw_chevron(svg, chev_x, react_y, react_w, cb_idx,
                          node['callbacks'][cb_idx]['name'])

            # Timer → reaction (dashed)
            svg.line(timer_cx + TIMER_R, timer_cy, chev_x + REACT_NOTCH, timer_cy, dash='3, 1')

            # Reaction → output ports (dashed)
            cb = node['callbacks'][cb_idx]
            for pub_topic in cb.get('publishers_used', []):
                pub_idx = next((j for j, p in enumerate(node['publishers'])
                                if p['topic'] == pub_topic), -1)
                if pub_idx >= 0:
                    op_cy = port_cy(pub_idx)
                    rx_tip = chev_x + react_w
                    op_x = nx + w - PORT_W
                    if abs(timer_cy - op_cy) < 1:
                        svg.line(rx_tip, op_cy, op_x, op_cy, dash='3, 1')
                    else:
                        mid = (rx_tip + op_x) / 2
                        svg.path(
                            f'M{rx_tip:.1f},{timer_cy:.1f} L{mid:.1f},{timer_cy:.1f}'
                            f' L{mid:.1f},{op_cy:.1f} L{op_x:.1f},{op_cy:.1f}',
                            dash='3, 1')
        ey += TIMER_ROW_H

    # Subscription-triggered reactions — placed after all timer rows in center column
    sub_react_y = ey + TIMER_R  # continues from where timers ended
    for i, sub in enumerate(node['subscribers']):
        if sub.get('callback') in timer_cb_set:
            continue
        cb_idx = cb_names.index(sub['callback']) if sub.get('callback') in cb_names else -1
        if cb_idx < 0:
            continue

        # The port is on the port grid; the reaction is in the center column below timers
        cy = port_cy(i)
        react_y = sub_react_y - REACT_H / 2
        _draw_chevron(svg, chev_x, react_y, react_w, cb_idx,
                      node['callbacks'][cb_idx]['name'])

        # Input port → reaction: route from port at cy down to reaction center
        react_mid_y = sub_react_y
        if abs(cy - react_mid_y) < 1:
            svg.line(nx + PORT_W, cy, chev_x + REACT_NOTCH, cy, dash='3, 1')
        else:
            mid_x = (nx + PORT_W + chev_x + REACT_NOTCH) / 2
            svg.path(
                f'M{nx+PORT_W:.1f},{cy:.1f} L{mid_x:.1f},{cy:.1f}'
                f' L{mid_x:.1f},{react_mid_y:.1f} L{chev_x+REACT_NOTCH:.1f},{react_mid_y:.1f}',
                dash='3, 1')

        # Reaction → output ports (dashed)
        cb = node['callbacks'][cb_idx]
        for pub_topic in cb.get('publishers_used', []):
            pub_idx = next((j for j, p in enumerate(node['publishers'])
                            if p['topic'] == pub_topic), -1)
            if pub_idx >= 0:
                op_cy = port_cy(pub_idx)
                rx_tip = chev_x + react_w
                op_x = nx + w - PORT_W
                if abs(react_mid_y - op_cy) < 1:
                    svg.line(rx_tip, op_cy, op_x, op_cy, dash='3, 1')
                else:
                    mid = (rx_tip + op_x) / 2
                    svg.path(
                        f'M{rx_tip:.1f},{react_mid_y:.1f} L{mid:.1f},{react_mid_y:.1f}'
                        f' L{mid:.1f},{op_cy:.1f} L{op_x:.1f},{op_cy:.1f}',
                        dash='3, 1')

        sub_react_y += ROW_H


def _topic_suffix(topic: str) -> str:
    """Return the last path component of a topic (strip namespace)."""
    return topic.strip('/').split('/')[-1]


def _layout_nodes(system: dict) -> tuple[dict, dict]:
    nodes = system['nodes']
    topics = system['communication']['topics']
    name_to_key = {v['name']: k for k, v in nodes.items()}
    name_to_key.update({k: k for k in nodes})

    succs: dict = {k: set() for k in nodes}
    preds: dict = {k: set() for k in nodes}
    for topic in topics:
        for pub in topic['publishers']:
            pk = name_to_key.get(pub, pub)
            for sub in topic['subscribers']:
                sk = name_to_key.get(sub, sub)
                if pk in nodes and sk in nodes:
                    succs[pk].add(sk)
                    preds[sk].add(pk)

    layer: dict = {k: 0 for k in nodes}
    sources = [k for k in nodes if not preds[k]] or [list(nodes.keys())[0]]
    queue: deque = deque(sources)
    # Longest-path layering; a feedback loop (A -> B -> A) would raise layers
    # forever, so a layer never exceeds the node count (found with the
    # corr_loop fixture: `colcon lf-analyze` spun at 100% CPU here).
    max_layer = max(len(nodes) - 1, 0)
    while queue:
        cur = queue.popleft()
        for s in succs[cur]:
            if layer[s] <= layer[cur] and layer[cur] < max_layer:
                layer[s] = layer[cur] + 1
                queue.append(s)

    by_layer: dict = defaultdict(list)
    for k in nodes:
        by_layer[layer[k]].append(k)

    positions = {}
    x = PAD_LEFT + PORT_LABEL_MARGIN
    for _, keys in sorted(by_layer.items()):
        layer_w = max(_node_width(nodes[k], k) for k in keys)
        y = PAD_TOP
        for k in keys:
            positions[k] = (x, y, _node_width(nodes[k], k), _node_height(nodes[k]))
            y += _node_height(nodes[k]) + NODE_GAP
        x += layer_w + LAYER_GAP

    return positions, name_to_key


def _build_port_index(nodes: dict, positions: dict) -> dict:
    """Build lookup: '{node_key}:pub/sub:{topic_suffix}' -> (x, y).

    Uses topic suffix matching so namespaced topics like /robot_1/sensor_data
    match node pub/sub entries that store just 'sensor_data'.
    """
    all_ports: dict = {}
    for key, node in nodes.items():
        nx, ny, w, _ = positions[key]
        for i, sub in enumerate(node['subscribers']):
            cy = ny + HDR + i * ROW_H + ROW_H / 2
            suffix = _topic_suffix(sub['topic'])
            all_ports[f'{key}:sub:{suffix}'] = (nx, cy)
        for i, pub in enumerate(node['publishers']):
            cy = ny + HDR + i * ROW_H + ROW_H / 2
            suffix = _topic_suffix(pub['topic'])
            all_ports[f'{key}:pub:{suffix}'] = (nx + w, cy)
    return all_ports


def render_topology_svg(system_data: dict, output_path: Path) -> Path:
    """Render a LF reactor-style topology SVG from system_data.

    Args:
        system_data: Parsed system.json dict.
        output_path: Full path to write the .svg file (directory created if needed).

    Returns:
        The output_path that was written.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    nodes = system_data['nodes']

    if not nodes:
        output_path.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<svg xmlns="http://www.w3.org/2000/svg" width="200" height="60" '
            'style="background:rgb(245,245,245)">\n'
            '<text x="10" y="35" font-size="12" fill="#999">(no nodes)</text>\n'
            '</svg>'
        )
        return output_path

    topics = system_data['communication']['topics']
    positions, name_to_key = _layout_nodes(system_data)
    all_ports = _build_port_index(nodes, positions)

    # Canvas size: account for port labels on both sides
    max_x = max(nx + w for nx, ny, w, h in positions.values()) + PORT_LABEL_MARGIN
    max_y = max(ny + h for nx, ny, w, h in positions.values()) + PAD_TOP

    svg = _SVG()

    # Collect all inter-node edges
    edge_list = []
    for topic in topics:
        t_suffix = _topic_suffix(topic['topic'])
        for pub_id in topic['publishers']:
            pk = name_to_key.get(pub_id, pub_id)
            for sub_id in topic['subscribers']:
                sk = name_to_key.get(sub_id, sub_id)
                src = all_ports.get(f'{pk}:pub:{t_suffix}')
                dst = all_ports.get(f'{sk}:sub:{t_suffix}')
                if src and dst:
                    edge_list.append((src, dst))

    # ── Gap-based interval lane assignment ──────────────────────────────────
    # Each edge is routed with an orthogonal path: horizontal → vertical → horizontal.
    # The vertical segment travels through one or more "gaps" (the space between
    # adjacent BFS layers). Overlaps happen when two edges share the same gap and
    # their vertical y-ranges overlap.
    #
    # Algorithm:
    # 1. Enumerate all unique gap x-positions (= right edges of all node columns).
    # 2. For every gap, collect every edge whose x1 <= gap_x < x2, i.e. the edge
    #    passes through (or starts in) this gap.
    # 3. Within each gap, assign each edge an x-lane using a greedy interval-
    #    coloring: sort by y_min, place each edge in the lowest lane that has no
    #    y-overlap with already-placed edges in that lane.
    # 4. Collect per-edge per-gap lane assignments, then for each edge determine
    #    the set of gap x positions it must pass through and draw multi-segment
    #    orthogonal paths accordingly.

    # All distinct right-edge x values (= gap starts)
    gap_xs = sorted({round(nx + w) for nx, ny, w, h in positions.values()})

    # For each gap, collect edges that pass through it and assign lanes
    # edge_lanes[edge_idx][gap_x] = lane_x (absolute x coordinate in that gap)
    edge_lanes: list[dict] = [{} for _ in edge_list]

    for gap_x in gap_xs:
        # Edges that pass through this gap: x1 <= gap_x < x2 (strict left, non-strict right)
        passing = []
        for idx, (src, dst) in enumerate(edge_list):
            x1, y1 = src
            x2, y2 = dst
            # An edge "passes through" a gap if its horizontal span covers it
            # (x1 is at right edge of source, x2 is at left edge of dest)
            if x1 <= gap_x + 0.5 and x2 > gap_x + 0.5:
                y_lo = min(y1, y2)
                y_hi = max(y1, y2)
                passing.append((y_lo, y_hi, idx))

        if not passing:
            continue

        # Sort by y_lo for greedy interval coloring
        passing.sort(key=lambda t: (t[0], t[1]))

        # Greedy interval coloring: assign each edge to the first lane
        # where its [y_lo, y_hi] doesn't overlap with anything already placed.
        # lanes[lane_idx] = y_hi of the last edge assigned there
        lane_y_max: list[float] = []
        edge_to_lane: dict[int, int] = {}
        MARGIN = 2.0  # minimum vertical separation between edges in same lane

        for y_lo, y_hi, idx in passing:
            placed = False
            for lane_idx, last_y_hi in enumerate(lane_y_max):
                if y_lo >= last_y_hi + MARGIN:
                    edge_to_lane[idx] = lane_idx
                    lane_y_max[lane_idx] = y_hi
                    placed = True
                    break
            if not placed:
                lane_idx = len(lane_y_max)
                edge_to_lane[idx] = lane_idx
                lane_y_max.append(y_hi)

        num_lanes = len(lane_y_max)
        # Map lane index → absolute x within this gap
        # Spread lanes across 15%–85% of LAYER_GAP; if only 1 lane, centre at 50%
        for idx, lane_idx in edge_to_lane.items():
            if num_lanes == 1:
                frac = 0.5
            else:
                frac = 0.15 + 0.70 * lane_idx / (num_lanes - 1)
            edge_lanes[idx][gap_x] = gap_x + LAYER_GAP * frac

    # Draw each edge as a multi-segment orthogonal path through its assigned lanes
    for idx, (src, dst) in enumerate(edge_list):
        x1, y1 = src
        x2, y2 = dst

        # Collect the gap x positions this edge passes through, in order
        my_gaps = sorted(g for g in edge_lanes[idx])

        if not my_gaps:
            # Fallback: direct line (same column or no gaps found)
            if abs(y1 - y2) < 1:
                svg.path(f'M{x1:.1f},{y1:.1f} L{x2:.1f},{y2:.1f}')
            else:
                mid_x = (x1 + x2) / 2
                svg.path(f'M{x1:.1f},{y1:.1f} L{mid_x:.1f},{y1:.1f} '
                         f'L{mid_x:.1f},{y2:.1f} L{x2:.1f},{y2:.1f}')
            continue

        if abs(y1 - y2) < 1:
            # Truly horizontal edge — draw direct with tiny y offset per lane
            lane_idx_first = edge_to_lane.get(idx, 0) if my_gaps else 0
            y_off = 0.0
            svg.path(f'M{x1:.1f},{y1+y_off:.1f} L{x2:.1f},{y2+y_off:.1f}')
            continue

        # Build orthogonal path through each gap segment
        # The path goes: x1,y1 → lane0,y1 → lane0,y2 → ... → x2,y2
        # For multi-gap edges the y position changes at each segment.
        # We keep y=y1 until the last gap, then drop to y=y2.
        # (This is safe because the interval assignment guarantees no overlap
        # in each individual gap's vertical segment.)
        segments = [f'M{x1:.1f},{y1:.1f}']
        prev_x = x1
        for gi, gap_x in enumerate(my_gaps):
            lx = edge_lanes[idx][gap_x]
            is_last_gap = (gi == len(my_gaps) - 1)
            if is_last_gap:
                # Final gap: descend from y1 to y2
                segments.append(f'L{lx:.1f},{y1:.1f}')
                segments.append(f'L{lx:.1f},{y2:.1f}')
            else:
                # Intermediate gap: stay at y1 (horizontal pass-through)
                segments.append(f'L{lx:.1f},{y1:.1f}')
            prev_x = lx
        segments.append(f'L{x2:.1f},{y2:.1f}')
        svg.path(' '.join(segments))

    # Draw nodes on top
    for key, node in nodes.items():
        nx, ny, w, h = positions[key]
        _render_node(svg, node, nx, ny, node_key=key)

    content = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{max_x:.0f}" height="{max_y:.0f}" '
        f'style="background:rgb(245,245,245)">\n'
        + svg.build() + '\n</svg>'
    )
    output_path.write_text(content)
    return output_path

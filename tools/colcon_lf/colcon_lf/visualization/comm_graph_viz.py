# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Inter-node communication graph — topics, services, actions between nodes."""

from __future__ import annotations

from typing import Any

import graphviz


def comm_graph_viz(system_data: dict[str, Any]) -> graphviz.Digraph:
    """Build a Digraph showing inter-node communication.

    Parameters
    ----------
    system_data:
        Parsed ``system.json`` dict.
    """
    name = system_data.get("name", "comm_graph")
    dot = graphviz.Digraph("comm_graph")
    dot.attr(rankdir="LR", label=name, labelloc="t", fontsize="18")
    dot.attr("node", shape="box", style="rounded,filled", fillcolor="lightblue")

    comm = system_data.get("communication", {})

    # Collect all node names
    all_nodes: set[str] = set()
    for topic in comm.get("topics", []):
        all_nodes.update(topic.get("publishers", []))
        all_nodes.update(topic.get("subscribers", []))
    for svc in comm.get("services", []):
        server = svc.get("server", "")
        if server:
            all_nodes.add(server)
        all_nodes.update(svc.get("clients", []))
    for act in comm.get("actions", []):
        server = act.get("server", "")
        if server:
            all_nodes.add(server)
        all_nodes.update(act.get("clients", []))

    # Create node boxes
    for node_name in sorted(all_nodes):
        dot.node(node_name, node_name)

    # Topic edges (solid) — including orphaned endpoints
    for topic in comm.get("topics", []):
        topic_name = topic.get("topic", "?")
        msg_type = _short_type(topic.get("message_type", ""))
        label = f"{topic_name}\\n{msg_type}{_fields_suffix(topic.get('fields'))}"
        pubs = topic.get("publishers", [])
        subs = topic.get("subscribers", [])
        if pubs and subs:
            for pub in pubs:
                for sub in subs:
                    dot.edge(pub, sub, label=label, color="black")
        elif pubs and not subs:
            # Orphaned publisher — show with a dashed arrow to nowhere
            topic_id = f"topic_{topic_name.replace('/', '_')}"
            dot.node(topic_id, topic_name, shape="ellipse", style="dashed",
                     fillcolor="lightyellow", fontsize="10")
            for pub in pubs:
                dot.edge(pub, topic_id, label=msg_type, style="dashed",
                         color="gray50")
        elif subs and not pubs:
            # Orphaned subscriber — show with a dashed arrow from nowhere
            topic_id = f"topic_{topic_name.replace('/', '_')}"
            dot.node(topic_id, topic_name, shape="ellipse", style="dashed",
                     fillcolor="lightyellow", fontsize="10")
            for sub in subs:
                dot.edge(topic_id, sub, label=msg_type, style="dashed",
                         color="gray50")

    # Service edges (dashed)
    for svc in comm.get("services", []):
        svc_name = svc.get("service", "?")
        svc_type = _short_type(svc.get("service_type", ""))
        req_str = _fields_suffix(svc.get("request_fields"))
        res_str = _fields_suffix(svc.get("response_fields"))
        label = f"{svc_name}\\n{svc_type}"
        if req_str:
            label += f"\\nreq:{req_str}"
        if res_str:
            label += f"\\nres:{res_str}"
        server = svc.get("server", "")
        if server:
            for client in svc.get("clients", []):
                dot.edge(client, server, label=label, style="dashed", color="blue")
        else:
            # Orphaned client(s) with no server
            svc_id = f"svc_{svc_name.replace('/', '_')}"
            dot.node(svc_id, f"{svc_name}\\n(no server)", shape="diamond",
                     style="dashed", fillcolor="mistyrose", fontsize="10")
            for client in svc.get("clients", []):
                dot.edge(client, svc_id, label=label, style="dashed", color="gray50")

    # Action edges (bold)
    for act in comm.get("actions", []):
        act_name = act.get("action", "?")
        act_type = _short_type(act.get("action_type", ""))
        goal_str = _fields_suffix(act.get("goal_fields"))
        result_str = _fields_suffix(act.get("result_fields"))
        fb_str = _fields_suffix(act.get("feedback_fields"))
        label = f"{act_name}\\n{act_type}"
        if goal_str:
            label += f"\\ngoal:{goal_str}"
        if result_str:
            label += f"\\nresult:{result_str}"
        if fb_str:
            label += f"\\nfeedback:{fb_str}"
        server = act.get("server", "")
        if server:
            for client in act.get("clients", []):
                dot.edge(client, server, label=label, style="bold",
                         color="purple", penwidth="2")
        else:
            # Orphaned action client(s) with no server
            act_id = f"act_{act_name.replace('/', '_')}"
            dot.node(act_id, f"{act_name}\\n(no server)", shape="diamond",
                     style="dashed", fillcolor="mistyrose", fontsize="10")
            for client in act.get("clients", []):
                dot.edge(client, act_id, label=label, style="dashed",
                         color="gray50", penwidth="2")

    return dot


def _short_type(type_str: str) -> str:
    """Shorten a C++ type like 'custom_interfaces::msg::SensorData' to 'SensorData'."""
    return type_str.rsplit("::", 1)[-1] if "::" in type_str else type_str


def _fields_suffix(fields: list[dict] | None) -> str:
    """Render up to 8 fields as a newline-separated suffix string."""
    if not fields:
        return ""
    shown = fields[:8]
    rows = "\\n".join(f"  {f['name']}: {f['type']}" for f in shown)
    suffix = f"\\n{rows}"
    if len(fields) > 8:
        suffix += f"\\n  ...+{len(fields) - 8} more"
    return suffix

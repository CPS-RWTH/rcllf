# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Per-node internals visualization — triggers, callbacks, resources, variables."""

from __future__ import annotations

from typing import Any

import graphviz

# Lifecycle state machine: method -> (from_state, to_state)
_LC_METHOD_TRANSITIONS: dict[str, tuple[str, str]] = {
    "on_configure":  ("Unconfigured", "Inactive"),
    "on_activate":   ("Inactive",     "Active"),
    "on_deactivate": ("Active",       "Inactive"),
    "on_cleanup":    ("Inactive",     "Unconfigured"),
    "on_shutdown":   ("*",            "Finalized"),
    "on_error":      ("*",            "Unconfigured"),
}
_LC_STATE_COLORS = {
    "Unconfigured": "lightyellow",
    "Inactive":     "lightblue",
    "Active":       "lightgreen",
    "Finalized":    "lightgray",
    "*":            "white",
}


def node_internals_viz(node_data: dict[str, Any]) -> graphviz.Digraph:
    """Build a Digraph showing one node's internal structure.

    Parameters
    ----------
    node_data:
        Parsed ``nodes/{name}.json`` dict (a serialized NodeInfo).
    """
    name = node_data.get("name", "node")
    dot = graphviz.Digraph(f"node_{name}")
    dot.attr(label=f"{node_data.get('class_name', name)} ({node_data.get('package', '?')})", labelloc="t")
    dot.attr("node", fontsize="10")

    # Track callback_group_name -> trigger node ids for group box rendering
    cbg_triggers: dict[str, list[str]] = {}

    # --- Triggers ---
    for timer in node_data.get("timers", []):
        period = timer.get("period_ms", "?")
        cb = timer.get("callback", "?")
        t_id = f"timer_{cb}"
        label = f"Timer\\n{period}ms"
        cbg = timer.get("callback_group")
        if cbg:
            label += f"\\n[{cbg}]"
            cbg_triggers.setdefault(cbg, []).append(t_id)
        created_in = timer.get("created_in")
        if created_in and created_in != "constructor":
            label += f"\\n[{created_in}]"
        destroyed_in = timer.get("destroyed_in")
        if destroyed_in:
            label += f"\\n\u2717 {destroyed_in}"
        dot.node(t_id, label, shape="diamond", style="filled", fillcolor="lightyellow")
        dot.edge(t_id, f"cb_{cb}", label="triggers")

    for sub in node_data.get("subscribers", []):
        topic = sub.get("topic", "?")
        cb = sub.get("callback", "?")
        s_id = f"sub_{topic}"
        msg_type = _short_type(sub.get("message_type", ""))
        label = f"Sub: {topic}\\n{msg_type}"
        qos = sub.get("qos_depth")
        if qos is not None:
            label += f"\\nQoS: {qos}"
        cbg = sub.get("callback_group")
        if cbg:
            label += f"\\n[{cbg}]"
            cbg_triggers.setdefault(cbg, []).append(s_id)
        created_in = sub.get("created_in")
        if created_in and created_in != "constructor":
            label += f"\\n[{created_in}]"
        destroyed_in = sub.get("destroyed_in")
        if destroyed_in:
            label += f"\\n\u2717 {destroyed_in}"
        dot.node(s_id, label, shape="diamond", style="filled", fillcolor="lightcyan")
        dot.edge(s_id, f"cb_{cb}", label="triggers")

    for srv in node_data.get("service_servers", []):
        svc_name = srv.get("service_name", "?")
        cb = srv.get("callback", "?")
        s_id = f"srvsrv_{svc_name}"
        label = f"Service: {svc_name}"
        cbg = srv.get("callback_group")
        if cbg:
            label += f"\\n[{cbg}]"
            cbg_triggers.setdefault(cbg, []).append(s_id)
        dot.node(s_id, label, shape="diamond", style="filled", fillcolor="lightpink")
        dot.edge(s_id, f"cb_{cb}", label="triggers")

    for act in node_data.get("action_servers", []):
        act_name = act.get("action_name", "?")
        a_id = f"actsrv_{act_name}"
        label = f"Action: {act_name}"
        cbg = act.get("callback_group")
        if cbg:
            label += f"\\n[{cbg}]"
            cbg_triggers.setdefault(cbg, []).append(a_id)
        dot.node(a_id, label, shape="diamond", style="filled", fillcolor="plum")
        goal_cb = act.get("goal_callback")
        if goal_cb:
            dot.edge(a_id, f"cb_{goal_cb}", label="goal")
        cancel_cb = act.get("cancel_callback")
        if cancel_cb:
            dot.edge(a_id, f"cb_{cancel_cb}", label="cancel")
        accepted_cb = act.get("accepted_callback")
        if accepted_cb:
            dot.edge(a_id, f"cb_{accepted_cb}", label="accepted")

    # --- Lifecycle state machine (if lifecycle node) ---
    lc_cb_names = set(node_data.get("lifecycle_callbacks", []))
    if lc_cb_names:
        _render_lifecycle_state_machine(dot, lc_cb_names, node_data.get("callbacks", []))

    # --- Callbacks ---
    for cb in node_data.get("callbacks", []):
        cb_name = cb.get("name", "?")
        if cb_name in lc_cb_names:
            continue  # Rendered as part of state machine
        params = cb.get("parameters", [])
        param_str = ", ".join(p.get("name", "") for p in params)
        label = f"{cb_name}({param_str})"
        # Annotate action feedback/result
        feedback = cb.get("feedback_published", [])
        result = cb.get("result_published", [])
        if feedback:
            label += "\\n[feedback: " + ", ".join(feedback) + "]"
        if result:
            label += "\\n[result: " + ", ".join(result) + "]"
        dot.node(f"cb_{cb_name}", label, shape="box", style="filled", fillcolor="lightgreen")

        # Arrows to publishers used
        for pub_topic in cb.get("publishers_used", []):
            dot.edge(f"cb_{cb_name}", f"pub_{pub_topic}", label="publish")

        # Arrows to service clients used
        for svc in cb.get("service_clients_used", []):
            dot.edge(f"cb_{cb_name}", f"svccli_{svc}", label="call")

        # Arrows to action clients used
        for act in cb.get("action_clients_used", []):
            dot.edge(f"cb_{cb_name}", f"actcli_{act}", label="send_goal")

        # Spawned callbacks (red dashed — thread spawn)
        for spawned in cb.get("spawns_callbacks", []):
            dot.edge(f"cb_{cb_name}", f"cb_{spawned}",
                     label="spawns", style="dashed", color="red")

        # Variable read/write
        for var in cb.get("variables_written", []):
            dot.edge(f"cb_{cb_name}", f"var_{var}", label="writes", style="dashed")
        for var in cb.get("variables_read", []):
            dot.edge(f"var_{var}", f"cb_{cb_name}", label="reads", style="dashed")

    # --- Resources ---
    for pub in node_data.get("publishers", []):
        topic = pub.get("topic", "?")
        msg_type = _short_type(pub.get("message_type", ""))
        is_lc = pub.get("is_lifecycle", False)
        pub_label = f"{'LifecyclePub' if is_lc else 'Pub'}: {topic}\\n{msg_type}"
        qos = pub.get("qos_depth")
        if qos is not None:
            pub_label += f"\\nQoS: {qos}"
        created_in = pub.get("created_in")
        if created_in and created_in != "constructor":
            pub_label += f"\\n[{created_in}]"
        destroyed_in = pub.get("destroyed_in")
        if destroyed_in:
            pub_label += f"\\n\u2717 {destroyed_in}"
        dot.node(f"pub_{topic}", pub_label, shape="box", style="rounded,filled",
                 fillcolor="lightsalmon" if not is_lc else "peachpuff")

    for cli in node_data.get("service_clients", []):
        svc = cli.get("service_name", "?")
        dot.node(f"svccli_{svc}", f"SvcClient: {svc}", shape="box", style="rounded,filled", fillcolor="lightpink")
        bound = cli.get("response_callback")
        if bound:
            dot.edge(f"svccli_{svc}", f"cb_{bound}", label="on_response", style="dashed", color="purple")

    for cli in node_data.get("action_clients", []):
        act = cli.get("action_name", "?")
        dot.node(f"actcli_{act}", f"ActClient: {act}", shape="box", style="rounded,filled", fillcolor="plum")
        for field, lbl in [("goal_response_callback", "on_goal"), ("feedback_callback", "on_feedback"), ("result_callback", "on_result")]:
            bound = cli.get(field)
            if bound:
                dot.edge(f"actcli_{act}", f"cb_{bound}", label=lbl, style="dashed", color="purple")

    # --- Variables ---
    all_vars = set()
    for cb in node_data.get("callbacks", []):
        all_vars.update(cb.get("variables_written", []))
        all_vars.update(cb.get("variables_read", []))
    for var in sorted(all_vars):
        dot.node(f"var_{var}", var, shape="plaintext", fontcolor="gray40")

    # --- Parameters (listed as a record) ---
    params = node_data.get("parameters", [])
    if params:
        rows = "\\l".join(
            f"{p.get('name', '?')}: {p.get('type', '?')} = {p.get('default_value', '?')}"
            for p in params
        )
        dot.node("params", f"Parameters\\l{rows}\\l", shape="note", style="filled", fillcolor="lemonchiffon")

    # --- Parameter callback trigger ---
    param_cb = node_data.get("param_callback")
    if param_cb:
        dot.node("param_change", "on_set_parameters", shape="diamond",
                 style="filled", fillcolor="lemonchiffon")
        if params:
            dot.edge("param_change", "params", label="validates", style="dashed", color="goldenrod")
        dot.edge("param_change", f"cb_{param_cb}", label="triggers", color="goldenrod")

    # --- Callback groups ---
    for i, cbg_info in enumerate(node_data.get("callback_groups", [])):
        cbg_name = cbg_info.get("name", "?")
        cbg_type = cbg_info.get("type", "?")
        cbg_id = f"cbg_{i}"
        dot.node(cbg_id, f"{cbg_name}\\n({cbg_type})",
                 shape="box", style="filled", fillcolor="#E8E8E8", fontsize="9")
        for trigger_id in cbg_triggers.get(cbg_name, []):
            dot.edge(cbg_id, trigger_id, style="dotted", color="#AAAAAA", arrowhead="none")

    return dot


def _short_type(type_str: str) -> str:
    """Shorten a C++ type like 'custom_interfaces::msg::SensorData' to 'SensorData'."""
    return type_str.rsplit("::", 1)[-1] if "::" in type_str else type_str


def _render_lifecycle_state_machine(
    dot: graphviz.Digraph,
    lc_cb_names: set[str],
    callbacks: list[dict],
) -> None:
    """Render lifecycle callbacks as a state machine inside a subgraph cluster."""
    cb_map = {cb.get("name"): cb for cb in callbacks}

    with dot.subgraph(name="cluster_lifecycle") as lc:
        lc.attr(label="Lifecycle State Machine", style="dashed", color="steelblue", fontcolor="steelblue")

        # Emit state nodes (only states reachable from present callbacks)
        states_needed: set[str] = set()
        for method in lc_cb_names:
            trans = _LC_METHOD_TRANSITIONS.get(method)
            if trans:
                states_needed.update(trans)
        states_needed.discard("*")

        for state in states_needed:
            color = _LC_STATE_COLORS.get(state, "white")
            lc.node(f"lc_state_{state}", state, shape="ellipse",
                    style="filled", fillcolor=color, fontsize="10")

        # Emit method boxes + transition edges
        for method in sorted(lc_cb_names, key=lambda m: list(_LC_METHOD_TRANSITIONS).index(m)
                              if m in _LC_METHOD_TRANSITIONS else 99):
            trans = _LC_METHOD_TRANSITIONS.get(method)
            cb = cb_map.get(method, {})
            params = cb.get("parameters", [])
            param_str = ", ".join(p.get("name", "") for p in params)
            label = f"{method}({param_str})"

            # Annotate resources created/destroyed by this method
            resource_notes = []
            if cb.get("variables_written"):
                resource_notes.append("writes: " + ", ".join(cb["variables_written"]))
            if resource_notes:
                label += "\\n" + "\\n".join(resource_notes)

            lc.node(f"cb_{method}", label, shape="box", style="filled", fillcolor="lightcyan", fontsize="9")

            if trans:
                from_state, to_state = trans
                if from_state != "*" and f"lc_state_{from_state}" in [n for n in states_needed]:
                    lc.edge(f"lc_state_{from_state}", f"cb_{method}", style="dashed", color="steelblue")
                if to_state != "*" and f"lc_state_{to_state}" in [n for n in states_needed]:
                    lc.edge(f"cb_{method}", f"lc_state_{to_state}", color="steelblue")

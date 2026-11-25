# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Unified system graph — nested node clusters with internal structure + communication.

Each ROS2 node is a subgraph cluster containing individual shapes for timers,
callbacks, publishers, subscribers, variables, and parameters.  Intra-node
edges show the trigger-callback-resource flow.  Inter-node edges connect
publishers to subscribers via topic/service/action interface boxes.

Layout is left-to-right (``rankdir=LR``) so data flows from triggers on the
left to published outputs on the right, through interface boxes, to the next
node's subscribers.
"""

from __future__ import annotations

from typing import Any

import graphviz

# Lifecycle state machine ordering and transitions
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
}


def _norm_topic(topic: str) -> str:
    """Normalize a topic/service/action name by stripping leading ``/``."""
    return topic.lstrip("/")


# ---------------------------------------------------------------------------
# Topological rank computation
# ---------------------------------------------------------------------------

def _compute_node_depths(
    comm: dict[str, Any],
    nodes_data: dict[str, Any],
) -> dict[str, int]:
    """Compute the longest-path depth for each node from source nodes.

    Returns a mapping from PascalCase node names (matching ``node_any_map``
    keys) to integer depth values.  Source nodes (no incoming edges) get
    depth 0.
    """
    # Build a lowercase -> canonical name map from nodes_data.
    # The keys in nodes_data are PascalCase (e.g. "BehaviorPlanner") and
    # match the keys used in node_any_map.  The comm data uses lowercase
    # names (e.g. "behaviorplanner"), so we map those back to the dict key.
    lc_to_name: dict[str, str] = {}
    for key, data in nodes_data.items():
        # Map both the lowercase key and the lowercase "name" field to the
        # PascalCase dict key (which is what node_any_map uses).
        lc_to_name[key.lower().replace("_", "")] = key
        info_name = data.get("name", "")
        if info_name:
            lc_to_name[info_name.lower().replace("_", "")] = key

    def _resolve(lc_name: str) -> str | None:
        clean = lc_name.lower().replace("_", "")
        return lc_to_name.get(clean)

    # Build adjacency list (publisher -> [subscribers]) using PascalCase names.
    successors: dict[str, set[str]] = {}
    all_nodes: set[str] = set()

    for topic in comm.get("topics", []):
        pub_names = [_resolve(p) for p in topic.get("publishers", [])]
        sub_names = [_resolve(s) for s in topic.get("subscribers", [])]
        for pn in pub_names:
            if pn is None:
                continue
            all_nodes.add(pn)
            successors.setdefault(pn, set())
            for sn in sub_names:
                if sn is None:
                    continue
                all_nodes.add(sn)
                successors.setdefault(sn, set())
                successors[pn].add(sn)

    for svc in comm.get("services", []):
        server = _resolve(svc.get("server", ""))
        for client_name in svc.get("clients", []):
            client = _resolve(client_name)
            if client and server:
                all_nodes.update([client, server])
                successors.setdefault(client, set())
                successors.setdefault(server, set())
                successors[client].add(server)

    for act in comm.get("actions", []):
        server = _resolve(act.get("server", ""))
        for client_name in act.get("clients", []):
            client = _resolve(client_name)
            if client and server:
                all_nodes.update([client, server])
                successors.setdefault(client, set())
                successors.setdefault(server, set())
                successors[client].add(server)

    # Include nodes from nodes_data that have no comm edges (isolated nodes).
    # Use the dict key (PascalCase) which matches node_any_map keys.
    for key in nodes_data:
        all_nodes.add(key)
        successors.setdefault(key, set())

    # Compute in-degree.
    in_degree: dict[str, int] = {n: 0 for n in all_nodes}
    for src, dsts in successors.items():
        for dst in dsts:
            in_degree[dst] = in_degree.get(dst, 0) + 1

    # Longest-path via topological relaxation (Kahn's algorithm variant).
    depth: dict[str, int] = {n: 0 for n in all_nodes}
    queue = [n for n in all_nodes if in_degree[n] == 0]

    while queue:
        next_queue: list[str] = []
        for node in queue:
            for succ in successors.get(node, set()):
                new_depth = depth[node] + 1
                if new_depth > depth[succ]:
                    depth[succ] = new_depth
                in_degree[succ] -= 1
                if in_degree[succ] == 0:
                    next_queue.append(succ)
        queue = next_queue

    return depth


def _inject_rank_constraints(
    dot: graphviz.Digraph,
    comm: dict[str, Any],
    nodes_data: dict[str, Any],
    depths: dict[str, int],
    node_any_map: dict[str, str],
    node_cluster_map: dict[str, str],
) -> None:
    """Add invisible cluster-level edges to enforce left-to-right topology.

    Uses ``ltail``/``lhead`` on invisible edges so that Graphviz positions
    entire node clusters according to the communication pipeline order.
    Also adds ``rank=same`` groups for nodes at the same depth.
    """
    if not depths or not node_cluster_map:
        return

    # Build a resolver that maps any comm-layer name to a key present in
    # node_cluster_map.  Comm data uses names like "actionserver_node",
    # nodes_data keys are "/action_example/action_server", and
    # node_cluster_map keys are launch names like "action_server".
    # We need to bridge all three.
    _name_to_cluster_key: dict[str, str] = {}
    for key, data in nodes_data.items():
        info_name = data.get("name", "")
        # Find which node_cluster_map key corresponds to this nodes_data entry.
        cluster_key: str | None = None
        for ck in node_cluster_map:
            if ck == key or key.endswith(f"/{ck}"):
                cluster_key = ck
                break
        if not cluster_key and info_name:
            cluster_key = node_cluster_map.get(info_name)
            if cluster_key:
                cluster_key = info_name
        if not cluster_key:
            continue
        norm = key.lower().replace("_", "").replace("/", "")
        _name_to_cluster_key[norm] = cluster_key
        if info_name:
            _name_to_cluster_key[info_name.lower().replace("_", "")] = cluster_key

    def _resolve(name: str) -> str | None:
        norm = name.lower().replace("_", "").replace("/", "")
        return _name_to_cluster_key.get(norm)

    # Collect unique (pub_cluster, sub_cluster) pairs from the communication
    # graph and add invisible edges with ltail/lhead.
    seen_cluster_pairs: set[tuple[str, str]] = set()

    for topic in comm.get("topics", []):
        for pub_name in topic.get("publishers", []):
            pub_canon = _resolve(pub_name)
            if not pub_canon or pub_canon not in node_cluster_map:
                continue
            for sub_name in topic.get("subscribers", []):
                sub_canon = _resolve(sub_name)
                if not sub_canon or sub_canon not in node_cluster_map:
                    continue
                pub_cluster = node_cluster_map[pub_canon]
                sub_cluster = node_cluster_map[sub_canon]
                if pub_cluster == sub_cluster:
                    continue
                pair = (pub_cluster, sub_cluster)
                if pair in seen_cluster_pairs:
                    continue
                seen_cluster_pairs.add(pair)
                # Invisible edge between clusters to enforce ordering.
                dot.edge(
                    node_any_map[pub_canon],
                    node_any_map[sub_canon],
                    style="invis", weight="10",
                    ltail=pub_cluster, lhead=sub_cluster,
                )

    # Same for services (client -> server)
    for svc in comm.get("services", []):
        server_canon = _resolve(svc.get("server", ""))
        if not server_canon or server_canon not in node_cluster_map:
            continue
        for client_name in svc.get("clients", []):
            client_canon = _resolve(client_name)
            if not client_canon or client_canon not in node_cluster_map:
                continue
            c_cluster = node_cluster_map[client_canon]
            s_cluster = node_cluster_map[server_canon]
            if c_cluster == s_cluster:
                continue
            pair = (c_cluster, s_cluster)
            if pair not in seen_cluster_pairs:
                seen_cluster_pairs.add(pair)
                dot.edge(
                    node_any_map[client_canon],
                    node_any_map[server_canon],
                    style="invis", weight="10",
                    ltail=c_cluster, lhead=s_cluster,
                )

    # Same for actions (client -> server)
    for act in comm.get("actions", []):
        server_canon = _resolve(act.get("server", ""))
        if not server_canon or server_canon not in node_cluster_map:
            continue
        for client_name in act.get("clients", []):
            client_canon = _resolve(client_name)
            if not client_canon or client_canon not in node_cluster_map:
                continue
            c_cluster = node_cluster_map[client_canon]
            s_cluster = node_cluster_map[server_canon]
            if c_cluster == s_cluster:
                continue
            pair = (c_cluster, s_cluster)
            if pair not in seen_cluster_pairs:
                seen_cluster_pairs.add(pair)
                dot.edge(
                    node_any_map[client_canon],
                    node_any_map[server_canon],
                    style="invis", weight="10",
                    ltail=c_cluster, lhead=s_cluster,
                )

    # rank=same groups for nodes at the same topological depth.
    by_depth: dict[int, list[str]] = {}
    for node_name, d in depths.items():
        if node_name in node_any_map:
            by_depth.setdefault(d, []).append(node_name)

    for d in sorted(by_depth):
        nodes_at_depth = sorted(by_depth[d])
        if len(nodes_at_depth) > 1:
            with dot.subgraph() as rank_sub:
                rank_sub.attr(rank="same")
                for node_name in nodes_at_depth:
                    rank_sub.node(node_any_map[node_name])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def system_graph_viz(system_data: dict[str, Any]) -> graphviz.Digraph:
    """Build a Digraph showing the full system with nested node internals.

    Parameters
    ----------
    system_data:
        Parsed ``system.json`` dict.
    """
    dot = graphviz.Digraph("system_graph")
    dot.attr(rankdir="LR", compound="true", newrank="true", splines="false")
    dot.attr("node", fontsize="9", fontname="Helvetica")
    dot.attr("edge", fontsize="8", fontname="Helvetica")
    dot.attr("graph", fontname="Helvetica")

    root = system_data.get("launch_graph", {}).get("root", {})
    nodes_data = system_data.get("nodes", {})
    comm = system_data.get("communication", {})

    # Maps for inter-node communication edges.
    # Keys use normalized topic names (no leading /).
    pub_port_map: dict[tuple[str, str], str] = {}
    sub_port_map: dict[tuple[str, str], str] = {}
    svc_srv_map: dict[tuple[str, str], str] = {}
    svc_cli_map: dict[tuple[str, str], str] = {}
    act_srv_map: dict[tuple[str, str], str] = {}
    act_cli_map: dict[tuple[str, str], str] = {}
    node_any_map: dict[str, str] = {}
    node_cluster_map: dict[str, str] = {}  # node_name -> cluster subgraph name

    port_maps = (pub_port_map, sub_port_map, svc_srv_map, svc_cli_map,
                 act_srv_map, act_cli_map, node_any_map, node_cluster_map)

    if root:
        _build_launch_cluster(dot, root, nodes_data, counter=[0],
                              port_maps=port_maps, is_root=True)

    # Inject topological rank constraints before adding comm edges so that
    # Graphviz places node clusters in left-to-right pipeline order.
    depths = _compute_node_depths(comm, nodes_data)
    _inject_rank_constraints(dot, comm, nodes_data, depths,
                             node_any_map, node_cluster_map)

    _add_comm_edges(dot, comm, port_maps)

    return dot


# ---------------------------------------------------------------------------
# Launch hierarchy
# ---------------------------------------------------------------------------

def _build_launch_cluster(
    dot: graphviz.Digraph,
    file_info: dict[str, Any],
    nodes_data: dict[str, Any],
    counter: list[int],
    port_maps: tuple,
    *,
    is_root: bool = False,
    outer_cluster: str | None = None,
) -> None:
    """Build a subgraph cluster for a launch file.

    When *is_root* is True the outermost launch-file cluster border is
    omitted so that Graphviz has more freedom to spread nodes horizontally.
    Nested includes and namespace groups still get their own clusters.

    *outer_cluster* is the outermost non-root cluster ID that encloses
    the current subtree.  It is passed down to ``_add_node_cluster`` so
    that ``node_cluster_map`` stores the outermost cluster — which is the
    one that ``ltail``/``lhead`` invisible edges should reference.
    """
    file_path = file_info.get("file_path", "unknown")
    package = file_info.get("package", "?")
    filename = file_path.rsplit("/", 1)[-1] if "/" in file_path else file_path

    def _populate(container: graphviz.Digraph, oc: str | None) -> None:
        for launch_node in file_info.get("nodes", []):
            _add_node_cluster(container, launch_node, nodes_data, counter,
                              port_maps, outer_cluster=oc)

        for group in file_info.get("groups", []):
            ns = group.get("namespace", "")
            g_cluster = f"cluster_{counter[0]}"
            counter[0] += 1
            with container.subgraph(name=g_cluster) as gsub:
                gsub.attr(label=f"ns: {ns}", style="dashed", color="gray")
                for launch_node in group.get("nodes", []):
                    _add_node_cluster(gsub, launch_node, nodes_data, counter,
                                      port_maps, outer_cluster=oc or g_cluster)
                for inc in group.get("includes", []):
                    _build_launch_cluster(gsub, inc, nodes_data, counter,
                                          port_maps,
                                          outer_cluster=oc or g_cluster)

        for inc in file_info.get("includes", []):
            _build_launch_cluster(container, inc, nodes_data, counter,
                                  port_maps, outer_cluster=oc)

    if is_root:
        _populate(dot, outer_cluster)
    else:
        # Flatten: populate children directly without a launch-file cluster box.
        _populate(dot, outer_cluster)


# ---------------------------------------------------------------------------
# Node cluster (shapes inside shapes)
# ---------------------------------------------------------------------------

def _add_node_cluster(
    parent: graphviz.Digraph,
    launch_node: dict[str, Any],
    nodes_data: dict[str, Any],
    counter: list[int],
    port_maps: tuple,
    *,
    outer_cluster: str | None = None,
) -> None:
    """Add a node as a subgraph cluster containing its internal shapes."""
    (pub_port_map, sub_port_map, svc_srv_map, svc_cli_map,
     act_srv_map, act_cli_map, node_any_map, node_cluster_map) = port_maps

    exe = launch_node.get("executable") or "?"
    pkg = launch_node.get("package") or "?"
    name = launch_node.get("name") or exe
    prefix = f"n{counter[0]}"
    cluster_name = f"cluster_node_{counter[0]}"
    counter[0] += 1

    node_info = _find_node_info(name, exe, nodes_data)
    condition = launch_node.get("condition")

    # All names that the communication layer might use to reference this node.
    comm_names = [name]
    if node_info:
        info_name = node_info.get("name", name)
        if info_name != name:
            comm_names.append(info_name)

    # Register the outermost enclosing cluster for topology rank constraints.
    # If there's no outer launch-file cluster, use the node cluster itself.
    effective_cluster = outer_cluster or cluster_name
    for cn in comm_names:
        node_cluster_map[cn] = effective_cluster

    with parent.subgraph(name=cluster_name) as sub:
        cond_suffix = f"\\n[{condition}]" if condition else ""

        if node_info:
            class_name = node_info.get("class_name", name)
            sub.attr(
                label=f"{name}\\n{class_name} ({pkg}){cond_suffix}",
                style="filled,rounded" + (",dashed" if condition else ""),
                fillcolor="white", color="black", penwidth="2",
            )
        else:
            sub.attr(
                label=f"{pkg}/{name}{cond_suffix}",
                style="filled,rounded" + (",dashed" if condition else ""),
                fillcolor="lightyellow", color="gray",
            )
            ph_id = f"{prefix}_ph"
            sub.node(ph_id, f"{pkg}/{name}", shape="box",
                     style="filled", fillcolor="lightyellow")
            for cn in comm_names:
                node_any_map[cn] = ph_id
            return

        # ----- Triggers -----

        # Collect callback names for internal ordering constraints.
        cb_names = [
            cb.get("name", "") for cb in node_info.get("callbacks", [])
            if cb.get("name")
        ]

        for i, timer in enumerate(node_info.get("timers", [])):
            period = timer.get("period_ms", "?")
            cb = timer.get("callback", "")
            cbg = timer.get("callback_group")
            mem_var = timer.get("member_variable")
            t_id = f"{prefix}_timer_{i}"
            label = f"Timer\\n{period}ms"
            if mem_var:
                label += f"\\n({mem_var})"
            if cbg:
                label += f"\\n[{cbg}]"

            sub.node(t_id, label, shape="box", style="filled",
                     fillcolor="#FFFFCC")
            if cb:
                sub.edge(t_id, f"{prefix}_cb_{cb}", style="bold")

        for i, s in enumerate(node_info.get("subscribers", [])):
            topic = s.get("topic", "?")
            msg_type = _short_type(s.get("message_type", ""))
            cb = s.get("callback", "")
            cbg = s.get("callback_group")
            mem_var = s.get("member_variable")
            s_id = f"{prefix}_sub_{i}"
            label = f"Sub: {topic}\\n{msg_type}"
            qos = s.get("qos_depth")
            if qos is not None:
                label += f"\\nQoS: {qos}"
            if mem_var:
                label += f"\\n({mem_var})"
            if cbg:
                label += f"\\n[{cbg}]"

            sub.node(s_id, label, shape="box", style="filled",
                     fillcolor="lightcyan")
            if cb:
                sub.edge(s_id, f"{prefix}_cb_{cb}", style="bold")
            for cn in comm_names:
                sub_port_map[(cn, _norm_topic(topic))] = s_id

        for i, srv in enumerate(node_info.get("service_servers", [])):
            svc_name = srv.get("service_name", "?")
            svc_type = _short_type(srv.get("service_type", ""))
            cb = srv.get("callback", "")
            cbg = srv.get("callback_group")
            mem_var = srv.get("member_variable")
            s_id = f"{prefix}_srvsrv_{i}"
            srv_label = f"Srv: {svc_name}\\n{svc_type}"
            if mem_var:
                srv_label += f"\\n({mem_var})"
            if cbg:
                srv_label += f"\\n[{cbg}]"
            sub.node(s_id, srv_label, shape="box",
                     style="filled", fillcolor="lightpink")
            if cb:
                sub.edge(s_id, f"{prefix}_cb_{cb}", style="bold")
            for cn in comm_names:
                svc_srv_map[(cn, _norm_topic(svc_name))] = s_id

        for i, act in enumerate(node_info.get("action_servers", [])):
            act_name = act.get("action_name", "?")
            act_type = _short_type(act.get("action_type", ""))
            cbg = act.get("callback_group")
            mem_var = act.get("member_variable")
            a_id = f"{prefix}_actsrv_{i}"
            act_label = f"Action: {act_name}\\n{act_type}"
            if mem_var:
                act_label += f"\\n({mem_var})"
            if cbg:
                act_label += f"\\n[{cbg}]"
            sub.node(a_id, act_label, shape="box",
                     style="filled", fillcolor="plum")
            for cb_field, lbl in [("goal_callback", "goal"),
                                   ("cancel_callback", "cancel"),
                                   ("accepted_callback", "accepted")]:
                bound = act.get(cb_field)
                if bound:
                    sub.edge(a_id, f"{prefix}_cb_{bound}",
                             label=lbl, style="dashed", color="purple")
            for cn in comm_names:
                act_srv_map[(cn, _norm_topic(act_name))] = a_id

        # ----- Lifecycle state machine (if lifecycle node) -----

        lc_cb_names = set(node_info.get("lifecycle_callbacks", []))
        if lc_cb_names:
            _render_lifecycle_sm_in_cluster(sub, prefix, lc_cb_names, node_info.get("callbacks", []))

        # ----- Callbacks (nodes only; variable edges drawn after resources) -----

        cb_pub_used: dict[str, list[str]] = {}
        cb_svc_used: dict[str, list[str]] = {}
        cb_act_used: dict[str, list[str]] = {}

        cb_spawns: dict[str, list[str]] = {}

        for cb in node_info.get("callbacks", []):
            cb_name = cb.get("name", "?")
            if cb_name in lc_cb_names:
                cb_pub_used[cb_name] = cb.get("publishers_used", [])
                cb_svc_used[cb_name] = cb.get("service_clients_used", [])
                cb_act_used[cb_name] = cb.get("action_clients_used", [])
                continue  # Already rendered in lifecycle state machine
            params = cb.get("parameters", [])
            param_str = ", ".join(p.get("name", "") for p in params)
            cb_id = f"{prefix}_cb_{cb_name}"
            label = f"{cb_name}({param_str})"
            feedback = cb.get("feedback_published", [])
            result = cb.get("result_published", [])
            if feedback:
                label += "\\n[fb: " + ", ".join(feedback) + "]"
            if result:
                label += "\\n[result: " + ", ".join(result) + "]"
            sub.node(cb_id, label, shape="box",
                     style="filled,rounded", fillcolor="lightgreen")

            cb_pub_used[cb_name] = cb.get("publishers_used", [])
            cb_svc_used[cb_name] = cb.get("service_clients_used", [])
            cb_act_used[cb_name] = cb.get("action_clients_used", [])
            cb_spawns[cb_name] = cb.get("spawns_callbacks", [])

        # ----- Resources -----

        # Publishers
        pub_id_by_topic: dict[str, str] = {}
        for i, pub in enumerate(node_info.get("publishers", [])):
            topic = pub.get("topic", "?")
            msg_type = _short_type(pub.get("message_type", ""))
            mem_var = pub.get("member_variable")
            is_lc = pub.get("is_lifecycle", False)
            p_id = f"{prefix}_pub_{i}"
            label = f"{'LifecyclePub' if is_lc else 'Pub'}: {topic}\\n{msg_type}"
            qos = pub.get("qos_depth")
            if qos is not None:
                label += f"\\nQoS: {qos}"
            if mem_var:
                label += f"\\n({mem_var})"
            created_in = pub.get("created_in")
            if created_in and created_in != "constructor":
                label += f"\\n[{created_in}]"
            sub.node(p_id, label, shape="box",
                     style="filled", fillcolor="lightsalmon" if not is_lc else "peachpuff")
            pub_id_by_topic[topic] = p_id
            for cn in comm_names:
                pub_port_map[(cn, _norm_topic(topic))] = p_id

        # Callback -> publisher edges
        for cb_name, topics in cb_pub_used.items():
            for t in topics:
                if t in pub_id_by_topic:
                    sub.edge(f"{prefix}_cb_{cb_name}", pub_id_by_topic[t])

        # Service clients
        svc_cli_id: dict[str, str] = {}
        for i, cli in enumerate(node_info.get("service_clients", [])):
            svc = cli.get("service_name", "?")
            svc_type = _short_type(cli.get("service_type", ""))
            mem_var = cli.get("member_variable")
            c_id = f"{prefix}_svccli_{i}"
            label = f"SvcClient: {svc}\\n{svc_type}"
            if mem_var:
                label += f"\\n({mem_var})"
            sub.node(c_id, label, shape="box",
                     style="filled", fillcolor="lightpink")
            svc_cli_id[svc] = c_id
            for cn in comm_names:
                svc_cli_map[(cn, _norm_topic(svc))] = c_id

        for cb_name, svcs in cb_svc_used.items():
            for s in svcs:
                if s in svc_cli_id:
                    sub.edge(f"{prefix}_cb_{cb_name}", svc_cli_id[s])

        # Service client -> bound response callback edges
        for i, cli in enumerate(node_info.get("service_clients", [])):
            svc = cli.get("service_name", "?")
            c_id = svc_cli_id.get(svc, f"{prefix}_svccli_{i}")
            bound = cli.get("response_callback")
            if bound:
                sub.edge(c_id, f"{prefix}_cb_{bound}",
                         label="on_response", style="dashed", color="purple",
                         constraint="false")

        # Action clients
        act_cli_id: dict[str, str] = {}
        for i, cli in enumerate(node_info.get("action_clients", [])):
            act = cli.get("action_name", "?")
            act_type = _short_type(cli.get("action_type", ""))
            mem_var = cli.get("member_variable")
            c_id = f"{prefix}_actcli_{i}"
            label = f"ActClient: {act}\\n{act_type}"
            if mem_var:
                label += f"\\n({mem_var})"
            sub.node(c_id, label, shape="box",
                     style="filled", fillcolor="plum")
            act_cli_id[act] = c_id
            for cn in comm_names:
                act_cli_map[(cn, _norm_topic(act))] = c_id

        for cb_name, acts in cb_act_used.items():
            for a in acts:
                if a in act_cli_id:
                    sub.edge(f"{prefix}_cb_{cb_name}", act_cli_id[a])

        # Action client -> bound callback edges (SendGoalOptions)
        # constraint=false: these are response-direction edges (server replies
        # trigger client callbacks) and should not extend the cluster rightward.
        for i, cli in enumerate(node_info.get("action_clients", [])):
            act = cli.get("action_name", "?")
            c_id = act_cli_id.get(act, f"{prefix}_actcli_{i}")
            for field, lbl in [("goal_response_callback", "on_goal"),
                                ("feedback_callback", "on_feedback"),
                                ("result_callback", "on_result")]:
                bound = cli.get(field)
                if bound:
                    sub.edge(c_id, f"{prefix}_cb_{bound}",
                             label=lbl, style="dashed", color="purple",
                             constraint="false")

        # Spawned callback edges (thread spawn — red dashed)
        for cb_name, spawned_list in cb_spawns.items():
            for spawned in spawned_list:
                sub.edge(f"{prefix}_cb_{cb_name}", f"{prefix}_cb_{spawned}",
                         label="spawns", style="dashed", color="red")

        # ----- Variables + callback variable edges -----

        all_vars: set[str] = set()
        for cb in node_info.get("callbacks", []):
            all_vars.update(cb.get("variables_written", []))
            all_vars.update(cb.get("variables_read", []))
        for var in sorted(all_vars):
            sub.node(f"{prefix}_var_{var}", var, shape="ellipse",
                     style="filled", fillcolor="white",
                     fontcolor="gray40", fontsize="8")

        # Draw variable read/write edges
        for cb in node_info.get("callbacks", []):
            cb_name = cb.get("name", "?")
            cb_id = f"{prefix}_cb_{cb_name}"
            for var in cb.get("variables_written", []):
                sub.edge(cb_id, f"{prefix}_var_{var}", style="dashed",
                         color="gray50", label="w", constraint="false")
            for var in cb.get("variables_read", []):
                sub.edge(f"{prefix}_var_{var}", cb_id, style="dashed",
                         color="gray50", label="r", constraint="false")

        # ----- Parameters -----

        params = node_info.get("parameters", [])
        if params:
            rows = "\\l".join(
                f"{p.get('name', '?')}: {p.get('type', '?')}"
                f" = {p.get('default_value', '?')}"
                for p in params
            )
            sub.node(f"{prefix}_params", f"Parameters\\l{rows}\\l",
                     shape="note", style="filled", fillcolor="lemonchiffon",
                     fontsize="8")

        # ----- Parameter callback trigger -----

        param_cb = node_info.get("param_callback")
        if param_cb:
            pc_id = f"{prefix}_param_change"
            sub.node(pc_id, "on_set_parameters", shape="box",
                     style="filled", fillcolor="lemonchiffon", fontsize="8")
            if params:
                sub.edge(pc_id, f"{prefix}_params",
                         label="validates", style="dashed", color="goldenrod")
            sub.edge(pc_id, f"{prefix}_cb_{param_cb}",
                     label="triggers", color="goldenrod")

        # ----- Internal ordering: inputs → callbacks → outputs -----
        # Invisible weighted edges enforce left-to-right flow within the
        # node cluster so subscribers are left and publishers are right.

        input_ids = (
            [f"{prefix}_sub_{i}" for i in range(len(node_info.get("subscribers", [])))]
            + [f"{prefix}_timer_{i}" for i in range(len(node_info.get("timers", [])))]
            + [f"{prefix}_srvsrv_{i}" for i in range(len(node_info.get("service_servers", [])))]
            + [f"{prefix}_actsrv_{i}" for i in range(len(node_info.get("action_servers", [])))]
        )
        output_ids = (
            list(pub_id_by_topic.values())
            + list(svc_cli_id.values())
            + list(act_cli_id.values())
        )

        if input_ids and cb_names:
            sub.edge(input_ids[0], f"{prefix}_cb_{cb_names[0]}",
                     style="invis", weight="100")
        if cb_names and output_ids:
            sub.edge(f"{prefix}_cb_{cb_names[0]}", output_ids[0],
                     style="invis", weight="100")

        # Stack all inputs in one column and all outputs in another.
        if len(input_ids) > 1:
            with sub.subgraph() as rank_sub:
                rank_sub.attr(rank="same")
                for sid in input_ids:
                    rank_sub.node(sid)
        if len(output_ids) > 1:
            with sub.subgraph() as rank_sub:
                rank_sub.attr(rank="same")
                for pid in output_ids:
                    rank_sub.node(pid)

        # ----- Register fallback for comm edge resolution -----

        fallback = None
        if node_info.get("callbacks"):
            fallback = (f"{prefix}_cb_"
                        f"{node_info['callbacks'][0].get('name', '?')}")
        elif node_info.get("timers"):
            fallback = f"{prefix}_timer_0"
        elif node_info.get("publishers"):
            fallback = f"{prefix}_pub_0"
        if fallback:
            for cn in comm_names:
                node_any_map[cn] = fallback


# ---------------------------------------------------------------------------
# Inter-node communication edges (interface boxes between clusters)
# ---------------------------------------------------------------------------

_iface_counter = [0]


def _fields_suffix(fields: list[dict] | None) -> str:
    """Build a graphviz label suffix listing interface fields (max 8)."""
    if not fields:
        return ""
    shown = fields[:8]
    rows = "\\l".join(f"  {f['name']}: {f['type']}" for f in shown)
    suffix = f"\\l{rows}\\l"
    if len(fields) > 8:
        suffix += f"  ...+{len(fields) - 8} more\\l"
    return suffix


def _add_comm_edges(
    dot: graphviz.Digraph,
    comm: dict[str, Any],
    port_maps: tuple,
) -> None:
    """Add interface boxes and edges between node clusters."""
    (pub_port_map, sub_port_map, svc_srv_map, svc_cli_map,
     act_srv_map, act_cli_map, node_any_map, node_cluster_map) = port_maps
    _iface_counter[0] = 0

    for topic in comm.get("topics", []):
        topic_name = topic.get("topic", "?")
        msg_type = _short_type(topic.get("message_type", ""))
        norm = _norm_topic(topic_name)
        pubs = topic.get("publishers", [])
        subs = topic.get("subscribers", [])

        if not pubs and not subs:
            continue

        # Create interface box for this topic
        iface_id = f"iface_topic_{_iface_counter[0]}"
        _iface_counter[0] += 1
        short = _short_type(msg_type)
        fields_str = _fields_suffix(topic.get("fields"))
        dot.node(iface_id, f"{topic_name}\\n{short}{fields_str}",
                 shape="box", style="filled", fillcolor="#F0F0F0",
                 color="#666666", fontsize="8")

        for pub_name in pubs:
            pub_id = _lookup(pub_port_map, pub_name, norm, node_any_map)
            dot.edge(pub_id, iface_id, color="#333333", penwidth="1.5")

        for sub_name in subs:
            sub_id = _lookup(sub_port_map, sub_name, norm, node_any_map)
            dot.edge(iface_id, sub_id, color="#333333", penwidth="1.5")

    for svc in comm.get("services", []):
        svc_name = svc.get("service", "?")
        svc_type = _short_type(svc.get("service_type", ""))
        norm = _norm_topic(svc_name)
        server = svc.get("server", "")
        clients = svc.get("clients", [])

        iface_id = f"iface_svc_{_iface_counter[0]}"
        _iface_counter[0] += 1
        req_str = _fields_suffix(svc.get("request_fields"))
        res_str = _fields_suffix(svc.get("response_fields"))
        svc_label = f"{svc_name}\\n{svc_type}"
        if req_str:
            svc_label += f"\\nreq:{req_str}"
        if res_str:
            svc_label += f"\\nres:{res_str}"
        dot.node(iface_id, svc_label,
                 shape="box", style="filled,dashed", fillcolor="#E8E8FF",
                 color="blue", fontsize="8", fontcolor="blue")

        server_id = _lookup(svc_srv_map, server, norm, node_any_map)
        # request: Client -> Interface -> Server (bold = client-to-server)
        for client in clients:
            client_id = _lookup(svc_cli_map, client, norm, node_any_map)
            dot.edge(client_id, iface_id, style="bold", color="blue",
                     penwidth="1.5", label="request", fontsize="7", fontcolor="blue")
        dot.edge(iface_id, server_id, style="bold", color="blue",
                 penwidth="1.5", label="request", fontsize="7", fontcolor="blue")

        # response: Server -> Interface -> Client (dashed = server-to-client)
        # constraint=false so backward edges don't fight left-to-right layout.
        dot.edge(server_id, iface_id, style="dashed", color="blue",
                 penwidth="1.0", label="response", fontsize="7", fontcolor="blue",
                 constraint="false")
        for client in clients:
            client_id = _lookup(svc_cli_map, client, norm, node_any_map)
            dot.edge(iface_id, client_id, style="dashed", color="blue",
                     penwidth="1.0", label="response", fontsize="7", fontcolor="blue",
                     constraint="false")

    for act in comm.get("actions", []):
        act_name = act.get("action", "?")
        act_type = _short_type(act.get("action_type", ""))
        norm = _norm_topic(act_name)
        server = act.get("server", "")
        clients = act.get("clients", [])

        iface_id = f"iface_act_{_iface_counter[0]}"
        _iface_counter[0] += 1
        goal_str = _fields_suffix(act.get("goal_fields"))
        result_str = _fields_suffix(act.get("result_fields"))
        fb_str = _fields_suffix(act.get("feedback_fields"))
        act_label = f"{act_name}\\n{act_type}"
        if goal_str:
            act_label += f"\\ngoal:{goal_str}"
        if result_str:
            act_label += f"\\nresult:{result_str}"
        if fb_str:
            act_label += f"\\nfeedback:{fb_str}"
        dot.node(iface_id, act_label,
                 shape="box", style="filled,bold", fillcolor="#F0E0FF",
                 color="purple", fontsize="8", fontcolor="purple")

        server_id = _lookup(act_srv_map, server, norm, node_any_map)
        # goal + cancel: Client -> Interface -> Server (bold = client-to-server direction)
        for client in clients:
            client_id = _lookup(act_cli_map, client, norm, node_any_map)
            dot.edge(client_id, iface_id, style="bold", color="purple",
                     penwidth="1.5", label="goal", fontsize="7", fontcolor="purple")
            dot.edge(client_id, iface_id, style="bold", color="purple",
                     penwidth="1.0", label="cancel", fontsize="7", fontcolor="purple")
        dot.edge(iface_id, server_id, style="bold", color="purple",
                 penwidth="1.5", label="goal", fontsize="7", fontcolor="purple")
        dot.edge(iface_id, server_id, style="bold", color="purple",
                 penwidth="1.0", label="cancel", fontsize="7", fontcolor="purple")

        # feedback + result: Server -> Interface -> Client (dashed = server-to-client direction)
        # constraint=false so backward edges don't fight left-to-right layout.
        dot.edge(server_id, iface_id, style="dashed", color="purple",
                 penwidth="1.5", label="feedback", fontsize="7", fontcolor="purple",
                 constraint="false")
        dot.edge(server_id, iface_id, style="dashed", color="purple",
                 penwidth="1.0", label="result", fontsize="7", fontcolor="purple",
                 constraint="false")
        for client in clients:
            client_id = _lookup(act_cli_map, client, norm, node_any_map)
            dot.edge(iface_id, client_id, style="dashed", color="purple",
                     penwidth="1.5", label="feedback", fontsize="7", fontcolor="purple",
                     constraint="false")
            dot.edge(iface_id, client_id, style="dashed", color="purple",
                     penwidth="1.0", label="result", fontsize="7", fontcolor="purple",
                     constraint="false")


def _lookup(
    port_map: dict[tuple[str, str], str],
    node_name: str,
    norm_name: str,
    node_any_map: dict[str, str],
) -> str:
    """Look up a shape id from a port map, trying normalized keys.

    Handles the mismatch between short names in node data (``"analyze"``)
    and fully-qualified names in comm data (``"action_example/analyze"``).
    """
    # Exact match
    result = port_map.get((node_name, norm_name))
    if result:
        return result
    # Normalized match
    for (n, t), shape_id in port_map.items():
        if n == node_name and _norm_topic(t) == norm_name:
            return shape_id
    # Short-name match: comm says "action_example/analyze", node says "analyze"
    # Check if the port map's topic is a suffix of the comm topic or vice versa
    for (n, t), shape_id in port_map.items():
        if n != node_name:
            continue
        nt = _norm_topic(t)
        if norm_name.endswith("/" + nt) or nt.endswith("/" + norm_name):
            return shape_id
        # Last segment match (e.g. "analyze" == "analyze")
        if norm_name.rsplit("/", 1)[-1] == nt.rsplit("/", 1)[-1]:
            return shape_id
    return node_any_map.get(node_name, node_name)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_node_info(
    name: str, executable: str, nodes_data: dict[str, Any],
) -> dict[str, Any] | None:
    """Find the NodeInfo dict for a launch node by name or executable."""
    for key, data in nodes_data.items():
        if data.get("name") == name or key == name or key.endswith(f"/{name}"):
            return data
    for key, data in nodes_data.items():
        if executable in key or executable == data.get("name"):
            return data
    return None


def _short_type(type_str: str) -> str:
    """Shorten a C++ type like 'custom_interfaces::msg::SensorData' to 'SensorData'."""
    return type_str.rsplit("::", 1)[-1] if "::" in type_str else type_str


def _render_lifecycle_sm_in_cluster(
    sub: graphviz.Digraph,
    prefix: str,
    lc_cb_names: set[str],
    callbacks: list[dict],
) -> None:
    """Render lifecycle callbacks as a state machine inside the node cluster."""
    cb_map = {cb.get("name"): cb for cb in callbacks}

    with sub.subgraph(name=f"cluster_{prefix}_lc") as lc:
        lc.attr(label="Lifecycle", style="dashed", color="steelblue",
                fontcolor="steelblue", fontsize="9")

        # Collect needed states
        states_needed: set[str] = set()
        for method in lc_cb_names:
            trans = _LC_METHOD_TRANSITIONS.get(method)
            if trans:
                states_needed.update(t for t in trans if t != "*")

        for state in states_needed:
            color = _LC_STATE_COLORS.get(state, "white")
            lc.node(f"{prefix}_lc_state_{state}", state, shape="ellipse",
                    style="filled", fillcolor=color, fontsize="9")

        # Emit method boxes in state-machine order
        for method in sorted(lc_cb_names,
                              key=lambda m: list(_LC_METHOD_TRANSITIONS).index(m)
                              if m in _LC_METHOD_TRANSITIONS else 99):
            trans = _LC_METHOD_TRANSITIONS.get(method)
            cb = cb_map.get(method, {})
            params = cb.get("parameters", [])
            param_str = ", ".join(p.get("name", "") for p in params)
            cb_id = f"{prefix}_cb_{method}"
            lc.node(cb_id, f"{method}({param_str})", shape="box",
                    style="filled,rounded", fillcolor="lightcyan", fontsize="9")

            if trans:
                from_state, to_state = trans
                if from_state != "*" and from_state in states_needed:
                    lc.edge(f"{prefix}_lc_state_{from_state}", cb_id,
                            style="dashed", color="steelblue")
                if to_state != "*" and to_state in states_needed:
                    lc.edge(cb_id, f"{prefix}_lc_state_{to_state}",
                            color="steelblue")

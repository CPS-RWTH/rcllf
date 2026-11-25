# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Inter-node communication mapper for ROS2 systems.

Matches publishers to subscribers, service servers to clients, and action
servers to clients — accounting for ROS2 namespace resolution and launch-file
remappings.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from colcon_lf.comm_analysis.models import (
    NODE_KEY,
    NODE_NAME,
    ActionConnection,
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
from colcon_lf.launch_analysis.ros_arguments import launch_identity
from colcon_lf.node_analysis.interface_parser import fields_to_dicts, parse_interface
from colcon_lf.node_analysis.models import NodeInfo

if TYPE_CHECKING:
    from colcon_lf.launch_analysis.models import LaunchNodeAction


def resolve_name(
    raw: str,
    namespace: str | None,
    remappings: list[tuple[str, str]],
    node_name: str | None = None,
) -> str:
    """Resolve a ROS2 topic/service/action name to its fully-qualified form.

    Resolution order:

    1. Absolute names (start with ``/``) are used as-is.
    2. Private names (``~`` or ``~/...``) expand to ``/<ns>/<node_name>/...``
       when *node_name* is given.
    3. Other relative names get the namespace prepended: ``/<ns>/<name>``.
       With no namespace they become ``/<name>``.
    4. Remappings are applied after namespace resolution.  The first matching
       rule wins; both the raw name and the resolved name are checked against
       the remapping source so that simple names like ``"sensor_data"`` and
       fully-qualified names like ``"/sensor_data"`` both match.
    """
    ns = ("/" + namespace.strip("/")) if namespace else ""

    if raw.startswith("/"):
        resolved = raw
    elif node_name and (raw == "~" or raw.startswith("~/")):
        resolved = f"{ns}/{node_name}{raw[1:]}"
    else:
        resolved = f"{ns}/{raw}" if ns else f"/{raw}"

    for src, dst in remappings:
        # Resolve the remapping source with the same namespace (no further remapping).
        src_abs = resolve_name(src, namespace, [], node_name)
        if resolved == src_abs or raw == src:
            resolved = resolve_name(dst, namespace, [], node_name)
            break

    return resolved


# Backward-compat alias: `resolve_name` was made public (the
# resolved-model resolver re-derives endpoint names through this same
# function); existing test imports of the private name keep working.
_resolve_name = resolve_name


def build_communication(
    nodes: list[tuple[LaunchNodeAction | None, NodeInfo]],
    workspace_root: Path | None = None,
    node_keys: list[str] | None = None,
) -> SystemCommunication:
    """Build a :class:`SystemCommunication` from a list of ``(launch_action, node_info)`` pairs.

    Parameters
    ----------
    nodes:
        Each entry pairs a :class:`~colcon_lf.launch_analysis.models.LaunchNodeAction`
        (which carries the resolved namespace and remappings from the launch file) with
        the corresponding :class:`~colcon_lf.node_analysis.models.NodeInfo` parsed from
        the C++ source.  Pass ``None`` as the launch action when no launch context is
        available; topic names are then resolved as absolute names with no namespace.
    node_keys:
        The ``SystemGraph.nodes`` key of each entry of *nodes*. Endpoints are
        then listed by node key, one per node instance. Without
        it they are listed by ``NodeInfo.name``, which two instances of one
        class share.

    Returns
    -------
    SystemCommunication
        All matched (and unmatched) topic/service/action connections.
        Unmatched endpoints are included with empty counterpart lists so that
        orphaned publishers/subscribers are visible in the output.

    Notes
    -----
    **Unresolved topic names**:
    publishers/subscribers whose ``topic`` is empty are skipped entirely rather
    than joined. ``clang_parser`` emits ``topic=""`` when a topic-name argument
    cannot be resolved to a static string; every such endpoint used to key into
    ``pub_map``/``sub_map`` under the same ``"/"`` resolved name, so two
    *unrelated* nodes with runtime-computed topic names appeared **falsely
    connected** in ``communication.topics``. A fabricated edge is strictly worse
    than a missing one: the missing edge is reported as ``RCLLF-H004`` by
    ``features/modules/dynamic_topic.py`` (the endpoints stay visible on their
    ``NodeInfo``, flagged ``topic_dynamic=True``), whereas a fabricated edge
    silently poisons the generated LF model and the topology diff.
    """
    # Accumulation maps
    pub_map: dict[str, list[str]] = {}               # resolved_topic -> [node_name]
    sub_map: dict[str, list[str]] = {}               # resolved_topic -> [node_name]
    pub_type: dict[str, str] = {}                    # resolved_topic -> message_type
    sub_type: dict[str, str] = {}                    # resolved_topic -> message_type (fallback)
    svc_servers: dict[str, tuple[str, str]] = {}     # resolved_svc -> (node_name, svc_type)
    svc_clients: dict[str, list[str]] = {}           # resolved_svc -> [node_name]
    svc_client_types: dict[str, str] = {}            # resolved_svc -> svc_type (from client)
    act_servers: dict[str, tuple[str, str]] = {}     # resolved_action -> (node_name, act_type)
    act_clients: dict[str, list[str]] = {}           # resolved_action -> [node_name]
    act_client_types: dict[str, str] = {}            # resolved_action -> act_type (from client)

    keys = node_keys if node_keys is not None else [ni.name for _, ni in nodes]
    for (launch_action, node_info), node_key in zip(nodes, keys, strict=True):
        ns: str | None = None
        remappings: list[tuple[str, str]] = []
        node_name = node_key
        # The ROS node name, which a private "~/x" name expands with.
        ros_name = node_info.name
        if launch_action is not None:
            # arguments= remaps first, as rcl applies them.
            identity = launch_identity(launch_action, node_info.name)
            ns, remappings, ros_name = identity.namespace, list(identity.remappings), identity.name

        for pub in node_info.publishers:
            if not pub.topic.strip():
                continue  # unresolved name — see the "Unresolved topic names" note
            topic = resolve_name(pub.topic, ns, remappings, ros_name)
            pub_map.setdefault(topic, []).append(node_name)
            pub_type[topic] = pub.message_type

        for sub in node_info.subscribers:
            if not sub.topic.strip():
                continue  # unresolved name — see the "Unresolved topic names" note
            topic = resolve_name(sub.topic, ns, remappings, ros_name)
            sub_map.setdefault(topic, []).append(node_name)
            if sub.message_type and topic not in sub_type:
                sub_type[topic] = sub.message_type

        for svc in node_info.service_servers:
            service = resolve_name(svc.service_name, ns, remappings, ros_name)
            svc_servers[service] = (node_name, svc.service_type)

        for cli in node_info.service_clients:
            service = resolve_name(cli.service_name, ns, remappings, ros_name)
            svc_clients.setdefault(service, []).append(node_name)
            if cli.service_type and service not in svc_client_types:
                svc_client_types[service] = cli.service_type

        for srv in node_info.action_servers:
            action = resolve_name(srv.action_name, ns, remappings, ros_name)
            act_servers[action] = (node_name, srv.action_type)

        for cli in node_info.action_clients:
            action = resolve_name(cli.action_name, ns, remappings, ros_name)
            act_clients.setdefault(action, []).append(node_name)
            if cli.action_type and action not in act_client_types:
                act_client_types[action] = cli.action_type

    # Build topic connections (include unmatched pub-only or sub-only topics)
    all_topics = set(pub_map) | set(sub_map)
    topics = [
        TopicConnection(
            topic=t,
            # Use publisher type first, fall back to subscriber type
            message_type=pub_type.get(t, "") or sub_type.get(t, ""),
            publishers=pub_map.get(t, []),
            subscribers=sub_map.get(t, []),
        )
        for t in sorted(all_topics)
    ]

    # Build service connections — include orphaned clients (no server)
    all_services = set(svc_servers) | set(svc_clients)
    services = []
    for svc in sorted(all_services):
        if svc in svc_servers:
            services.append(ServiceConnection(
                service=svc,
                service_type=svc_servers[svc][1],
                server=svc_servers[svc][0],
                clients=svc_clients.get(svc, []),
            ))
        else:
            # Orphaned client(s) with no server
            services.append(ServiceConnection(
                service=svc,
                service_type=svc_client_types.get(svc, ""),
                server="",
                clients=svc_clients.get(svc, []),
            ))

    # Build action connections — include orphaned clients (no server)
    all_actions = set(act_servers) | set(act_clients)
    actions = []
    for act in sorted(all_actions):
        if act in act_servers:
            actions.append(ActionConnection(
                action=act,
                action_type=act_servers[act][1],
                server=act_servers[act][0],
                clients=act_clients.get(act, []),
            ))
        else:
            # Orphaned client(s) with no server
            actions.append(ActionConnection(
                action=act,
                action_type=act_client_types.get(act, ""),
                server="",
                clients=act_clients.get(act, []),
            ))

    # Populate interface field definitions if workspace_root is provided
    if workspace_root is not None:
        from colcon_lf.node_analysis.interface_parser import ActionDef, MessageDef, ServiceDef
        for conn in topics:
            defn = parse_interface(conn.message_type, workspace_root)
            if isinstance(defn, MessageDef):
                conn.fields = fields_to_dicts(defn.fields)
        for conn in services:
            defn = parse_interface(conn.service_type, workspace_root)
            if isinstance(defn, ServiceDef):
                conn.request_fields = fields_to_dicts(defn.request)
                conn.response_fields = fields_to_dicts(defn.response)
        for conn in actions:
            defn = parse_interface(conn.action_type, workspace_root)
            if isinstance(defn, ActionDef):
                conn.goal_fields = fields_to_dicts(defn.goal)
                conn.result_fields = fields_to_dicts(defn.result)
                conn.feedback_fields = fields_to_dicts(defn.feedback)

    return SystemCommunication(
        topics=topics, services=services, actions=actions,
        endpoint_ids=NODE_KEY if node_keys is not None else NODE_NAME,
    )

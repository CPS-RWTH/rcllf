# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Join ``SystemGraph.nodes`` against the launch model for name, namespace, remaps, parameters.

``NodeInfo`` carries ``name``/``class_name``/``package`` but **no namespace** —
resolved namespaces live in the launch model
(``launch_analysis/models.py``: ``LaunchNodeAction.namespace``, "Resolved
namespace (may include pushed ns)").  Both the non-component factory emitter
and the per-instance component-list emitter need ``(node_name, namespace)``,
so the join lives here once.

The join reproduces ``graph/hierarchical.py:_node_key`` — the key under which a
launch node action was filed into ``SystemGraph.nodes`` — so it is exact rather
than heuristic.
"""

from __future__ import annotations

from dataclasses import dataclass

from colcon_lf.generation.factory_emitter import component_param_args, param_args_for
from colcon_lf.launch_analysis.ros_arguments import (
    launch_identity,
    parse_node_arguments,
    refuse_unsupported,
)

__all__ = ["NodeIdentity", "build_node_identities"]


@dataclass(frozen=True)
class NodeIdentity:
    """Wire identity of one node instance, as the launch model dictates it."""

    node_name: str
    namespace: str  # "" when the launch model gives no namespace
    # Launch ``remappings=`` pairs, in launch order.
    remappings: tuple[tuple[str, str], ...] = ()
    # Launch ``parameters=`` as --ros-args rules, inline then files.
    param_args: tuple[str, ...] = ()
    # True when a launch action gave namespace and remaps, so endpoint names
    # resolve exactly as the analyzer resolved them.
    from_launch: bool = False


def _launch_node_key(launch_node, own_name: str) -> str:
    """Mirror ``graph/hierarchical.py:_node_key`` for a LaunchNodeAction.

    The key is the identity the node runs with, ``arguments=`` remaps
    included.
    """
    identity = launch_identity(launch_node, own_name)
    ns = identity.namespace
    if ns:
        return f"{ns}/{identity.name}" if not ns.endswith("/") else f"{ns}{identity.name}"
    return identity.name


def build_node_identities(graph) -> dict[str, NodeIdentity]:
    """Map every ``graph.nodes`` key to its launch-model ``NodeIdentity``.

    Falls back to ``NodeInfo.name`` with an empty namespace for any node with
    no matching launch action (hand-built graphs, federate sub-graphs that
    carry no ``launch_graph``). Raises ``NodeArgumentsError`` when a node's
    launch ``arguments=`` hold a rule the lfc runtime cannot apply.
    """
    identities: dict[str, NodeIdentity] = {}

    launch_graph = getattr(graph, "launch_graph", None)
    all_launch_nodes = getattr(launch_graph, "all_nodes", []) or []

    for node_key, node_info in graph.nodes.items():
        # The launch action hierarchical.py filed under this key. Named
        # actions first: a nameless one keys by the code name and could
        # otherwise take a named sibling's key.
        candidates = sorted(all_launch_nodes, key=lambda n: not getattr(n, "name", None))
        launch_node = next(
            (n for n in candidates if _launch_node_key(n, node_info.name) == node_key), None)

        if launch_node is None:
            identities[node_key] = NodeIdentity(node_name=node_info.name, namespace="")
            continue

        refuse_unsupported(launch_node, node_key)
        identity = launch_identity(launch_node, node_info.name)
        namespace = identity.namespace or ""
        # -p rules from arguments= go before the launch parameters, as on
        # the stock command line. A node-targeted rule keeps its
        # prefix: rclcpp matches it against the node's name as stock does.
        arg_params: list[str] = []
        args = getattr(launch_node, "arguments", None) or ()
        for target, name, text in parse_node_arguments(args).params:
            arg_params += ["-p", f"{target}:{name}:={text}" if target else f"{name}:={text}"]
        sources = getattr(launch_node, "parameter_sources", None)
        if sources and getattr(launch_node, "is_component", False):
            param_args = tuple(component_param_args(
                sources, getattr(launch_node, "name", None), namespace, where=node_key,
            ))
        else:
            param_args = tuple(arg_params + param_args_for(
                getattr(launch_node, "parameters", None) or {},
                getattr(launch_node, "parameter_files", None) or [],
                where=node_key,
                sources=sources,
            ))
        identities[node_key] = NodeIdentity(
            node_name=identity.name, namespace=namespace, remappings=identity.remappings,
            param_args=param_args, from_launch=True,
        )

    return identities

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A launch ``Node``'s ``arguments=`` as the node identity rcl gives it.

``launch_ros`` builds a node's command line as ``<exe> <arguments>
[--ros-args <ros_arguments>] --ros-args -r __node:=<name> -r __ns:=<ns>
<parameters> <remappings>``. rcl takes the first remap rule that matches, so
a ``-r __node:=x`` or ``-r __ns:=/y`` in ``arguments`` beats ``name=`` and
``namespace=``, and a topic rule in ``arguments`` beats one in
``remappings=``. ``-p`` rules come before the launch ``parameters=``, so a
later launch parameter wins. A ``-p node:name:=value`` rule applies only to
the node whose fully qualified name, after remapping, is ``/node``; rcl
aborts on a prefix with ``/`` or a wildcard (Jazzy, checked with ``ros2
run`` on 2026-10-13).

The analyzer (node keys, topics, parameters) and the generator (the lfc
node's ``NodeOptions``) both read the identity from ``launch_identity``, so
system.json and the lfc run agree. The lfc runtime constructs the node in a
shared process and never runs its ``main()``, so it can apply only these
rules. ``NodeArguments.unsupported`` lists everything else; generation
refuses a node with any (``NodeArgumentsError``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import yaml

__all__ = [
    "LaunchIdentity",
    "NodeArguments",
    "NodeArgumentsError",
    "launch_identity",
    "parse_node_arguments",
    "refuse_unsupported",
]

_REMAP_FLAGS = ("-r", "--remap")
_PARAM_FLAGS = ("-p", "--param")
# rcl flags that take a value: refused together with it.
_VALUE_FLAGS = (
    "--params-file", "--log-level", "--log-config-file", "--log-file-name", "-e", "--enclave",
)
_NAME_KEYS = ("__node", "__name")


class NodeArgumentsError(ValueError):
    """A launch node's ``arguments=`` holds something the lfc runtime cannot apply."""


@dataclass(frozen=True)
class NodeArguments:
    """The rules of one ``arguments=`` list, in command-line order."""

    # (target, src, dst); target is "" for a rule that applies to every node.
    remaps: tuple[tuple[str, str, str], ...] = ()
    # (target, name, value text) of ``-p [target:]name:=value``; target is ""
    # for a rule that applies to every node. rcl reads the text as YAML.
    params: tuple[tuple[str, str, str], ...] = ()
    unsupported: tuple[str, ...] = ()


def parse_node_arguments(arguments) -> NodeArguments:
    """Split *arguments* into the remap and ``-p`` rules the lfc run applies."""
    args = [str(a) for a in arguments or ()]
    remaps: list[tuple[str, str, str]] = []
    params: list[tuple[str, str, str]] = []
    unsupported: list[str] = []
    in_ros_args = False
    i = 0
    while i < len(args):
        arg = args[i]
        value = args[i + 1] if i + 1 < len(args) else None
        if arg == "--ros-args":
            in_ros_args = True
        elif arg == "--" and in_ros_args:
            in_ros_args = False
        elif not in_ros_args:
            unsupported.append(
                f"'{arg}' is a command-line argument of the executable's main(), "
                "which the lfc runtime does not run")
        elif arg in _REMAP_FLAGS or arg in _PARAM_FLAGS:
            i += 1
            if value is None or ":=" not in value:
                unsupported.append(f"'{arg}' needs a 'name:=value' rule, got {value!r}")
            elif arg in _PARAM_FLAGS:
                name, text = value.split(":=", 1)
                target, _, name = name.rpartition(":")
                if "/" in target or "*" in target:
                    unsupported.append(
                        f"'{arg} {value}': rcl cannot parse a node prefix with '/' or a "
                        "wildcard and aborts the process")
                else:
                    params.append((target, name, text))
            else:
                src, dst = value.split(":=", 1)
                target, _, src = src.rpartition(":") if "://" not in src else ("", "", src)
                reason = _unsupported_remap(target, src, dst)
                if reason:
                    unsupported.append(f"'{arg} {value}': {reason}")
                else:
                    remaps.append((target, src, dst))
        elif arg in _VALUE_FLAGS:
            i += 1
            shown = f"{arg} {value}" if value is not None else arg
            unsupported.append(f"'{shown}'{_flag_hint(arg)}")
        else:
            unsupported.append(f"'{arg}' is a ROS argument the lfc runtime does not apply")
        i += 1
    return NodeArguments(tuple(remaps), tuple(params), tuple(unsupported))


def _unsupported_remap(target: str, src: str, dst: str) -> str:
    if "://" in src:
        return "a remap with a rostopic:// or rosservice:// prefix is not supported"
    if src in _NAME_KEYS and (not dst or "/" in dst):
        return "a node name cannot be empty or contain '/'"
    if src == "__ns" and not dst.startswith("/"):
        return "rcl needs an absolute namespace"
    if target and src not in _NAME_KEYS and src != "__ns":
        return "a topic remap targeted at one node is not supported; drop the 'node:' prefix"
    return ""


def _flag_hint(flag: str) -> str:
    if flag == "--params-file":
        return " (pass the file in parameters=[...] instead)"
    return " is process-wide in rcl and not applied per node"


@dataclass(frozen=True)
class LaunchIdentity:
    """The name, namespace and topic remaps a launch ``Node`` runs with."""

    name: str
    namespace: str | None
    # Topic remaps in rcl's order: ``arguments`` rules, then ``remappings=``.
    remappings: tuple[tuple[str, str], ...]
    # ``-p`` rules from ``arguments`` that apply to this node, as (name, value)
    # with the node prefix stripped; they come before ``parameters=``.
    params: tuple[tuple[str, Any], ...]


def launch_identity(launch_node, own_name: str) -> LaunchIdentity:
    """The identity rcl gives the node *launch_node* starts.

    The rules are walked in ``launch_ros``' command-line order:
    ``arguments``, then ``name=`` and ``namespace=``, then ``remappings=``
    (which may hold ``__node``/``__ns`` rules too). rcl takes the first
    ``__node`` and the first ``__ns`` rule that match. *own_name* is the
    node's name in its code: a rule targeted ``<node>:__node:=x`` applies
    when ``<node>`` is that name, as rcl matches it before remapping.
    """
    parsed = parse_node_arguments(getattr(launch_node, "arguments", None) or ())
    rules = list(parsed.remaps)
    if getattr(launch_node, "name", None):
        rules.append(("", "__node", launch_node.name))
    if getattr(launch_node, "namespace", None):
        rules.append(("", "__ns", launch_node.namespace))
    for src, dst in getattr(launch_node, "remappings", None) or ():
        target, _, src = str(src).rpartition(":") if "://" not in str(src) else ("", "", str(src))
        rules.append((target, src, str(dst)))
    name: str | None = None
    namespace: str | None = None
    for target, src, dst in rules:
        if target and target != own_name:
            continue
        if src in _NAME_KEYS:
            name = dst if name is None else name
        elif src == "__ns":
            namespace = dst if namespace is None else namespace
    name = name or own_name
    # rcl matches a targeted topic rule against the remapped node name.
    topic_remaps = [
        (src, dst) for target, src, dst in rules
        if src not in _NAME_KEYS and src != "__ns" and (not target or target == name)
    ]
    fqn = f"{(namespace or '').rstrip('/')}/{name}"
    params = tuple(
        (n, yaml.safe_load(t) if t else t) for target, n, t in parsed.params
        if not target or f"/{target}" == fqn)
    return LaunchIdentity(
        name=name, namespace=namespace or None,
        remappings=tuple(topic_remaps), params=params)


def refuse_unsupported(launch_node, where: str) -> None:
    """Raise ``NodeArgumentsError`` if *launch_node*'s arguments hold a rule the lfc run drops."""
    parsed = parse_node_arguments(getattr(launch_node, "arguments", None) or ())
    if parsed.unsupported:
        raise NodeArgumentsError(
            f"launch node {where}: the lfc runtime cannot apply these arguments= entries"
            f": {'; '.join(parsed.unsupported)}. Only '--ros-args' with "
            "'-r'/'--remap' and '-p'/'--param' rules are applied; remove the others or "
            "express them as name=, namespace=, remappings= or parameters=.")

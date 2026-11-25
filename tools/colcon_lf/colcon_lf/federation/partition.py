# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Resolve federation partition from --federate argument.

Entry point: ``resolve_partition(federate_arg, graph, system_json) -> FederationMap``

Three modes:
  per-process  — one federate per LaunchGraph.process_groups entry
  per-node     — one federate per ROS2 node
  <path>.yaml  — explicit YAML partition map
"""

from __future__ import annotations

import hashlib
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Iterable

from colcon_lf.federation.yaml_schema import (
    BUDGET_KEYS,
    COORDINATION_CENTRALIZED,
    KEY_ABSENT_AFTER,
    KEY_AFTER,
    KEY_AFTER_OVERRIDES,
    KEY_CLOCK_ERROR,
    KEY_COORDINATION,
    KEY_FEDERATES,
    KEY_FROM,
    KEY_MAX_EXEC,
    KEY_MAX_LATENCY,
    KEY_NODES,
    KEY_STP_OFFSET,
    KEY_SYSTEM,
    KEY_TO,
    MODE_PER_NODE,
    MODE_PER_PROCESS,
    VALID_COORDINATION_MODES,
)
from colcon_lf.generation.lf_time import parse_lf_time_ns

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph

# Default declared logical-time offset (LF `after`, LET-style contract) applied
# to cross-federate links when no override is given. Not a latency measurement.
# 0 is ``after 0``, the same as an in-process link: the receiver
# runs at (t, m + 1), so splitting a system into federates adds no logical
# latency and the dispatch order matches the one-process build. A d > 0 is a
# LET budget the application states through after_overrides.
DEFAULT_CROSS_FEDERATE_AFTER_NS = 0

# Default STP (safe-to-process) offset applied to cross-federate links in
# decentralized coordination when no per-link override is given. A tuning
# starting point, not a measurement of any real transport latency, and no
# longer derived from the after default (it was twice the old 5 ms). Too small
# and records arrive tardy (counted and dropped by the ingress); too large and
# the receiver waits longer than it needs to. Soundness does not depend on it.
# See docs/design/time-and-coordination.md regime 3 and
# docs/features/federation.md.
DEFAULT_STP_OFFSET_NS = 10_000_000  # 10 ms

# lfc 0.12.1 keeps the last 39 characters of a generated C
# instance name. Inside federate F the network receiver `nr_federate__F` then
# gets the main reactor's name once F has 31 characters, lfc renames one of
# them with `_2`, and its own federate preamble still names the other, so the
# C compile fails. 30 characters compile (measured).
MAX_FEDERATE_NAME_LEN = 30
_FEDERATE_HASH_LEN = 8


def lfc_safe_federate_name(name: str) -> str:
    """*name*, or its head plus a hash of the whole name when lfc cannot compile it.

    Derived federate names (per-node, per-process) go through here. The hash
    keeps two long names with the same head apart.
    """
    if len(name) <= MAX_FEDERATE_NAME_LEN:
        return name
    digest = hashlib.sha1(name.encode("utf-8")).hexdigest()[:_FEDERATE_HASH_LEN]
    head = name[:MAX_FEDERATE_NAME_LEN - _FEDERATE_HASH_LEN - 1].rstrip("_")
    return f"{head}_{digest}"


def check_federate_names(fmap: "FederationMap") -> None:
    """Refuse a federate name lfc cannot compile, or a name used twice."""
    for name in fmap.federate_names:
        if len(name) > MAX_FEDERATE_NAME_LEN:
            raise ValueError(
                f"{fmap.source or 'federation'}: federate name '{name}' has {len(name)} "
                f"characters, lfc compiles federate names of at most "
                f"{MAX_FEDERATE_NAME_LEN} characters (longer ones collide after its C name "
                f"truncation). Choose a shorter name."
            )
    seen: set[str] = set()
    for name in fmap.federate_names:
        if name in seen:
            raise ValueError(
                f"{fmap.source or 'federation'}: federate name '{name}' is given to two federates"
            )
        seen.add(name)


@dataclass
class AfterOverride:
    """Per-link cross-federate after-delay (and, in decentralized coordination,
    STP-offset) override.

    ``after_ns`` is a declared logical-time offset (LET-style contract), not a
    latency measurement; a measured value may inform the choice of override.
    ``stp_offset_ns`` only applies under decentralized coordination; ``None``
    means "use the default" (``DEFAULT_STP_OFFSET_NS``), not "zero".
    """

    from_port: str  # "NodeName.port_name"
    to_port: str
    after_ns: int
    stp_offset_ns: int | None = None
    # Declared decentralized budget terms (item5 piece D); None means undeclared.
    max_latency_ns: int | None = None  # L
    clock_error_ns: int | None = None  # eps
    max_exec_ns: int | None = None  # X

    def declares_budget(self) -> bool:
        return any(v is not None for v in (
            self.max_latency_ns, self.clock_error_ns, self.max_exec_ns))


@dataclass
class FederationMap:
    """Maps each node name to a federate name, with optional per-link overrides."""

    # node_name -> federate_name
    node_to_federate: dict[str, str] = field(default_factory=dict)
    # ordered list of federate names (preserves user ordering)
    federate_names: list[str] = field(default_factory=list)
    # per-link after-delay overrides (from YAML after_overrides)
    after_overrides: list[AfterOverride] = field(default_factory=list)
    # Coordination mode for this federation: "centralized" (default) or
    # "decentralized". Only the custom-YAML partition mode can set this from
    # the file itself (KEY_COORDINATION); per-node/per-process default to
    # centralized here and are overridden by --coordination at the call site.
    coordination: str = COORDINATION_CENTRALIZED
    # source description for error messages
    source: str = ""
    # every (from_port, to_port) link key the generator looked up, so an
    # override that names no link can be reported
    queried_links: set[tuple[str, str]] = field(default_factory=set)

    def federate_for(self, node_name: str) -> str | None:
        return self.node_to_federate.get(node_name)

    def nodes_for(self, federate_name: str) -> list[str]:
        return [n for n, f in self.node_to_federate.items() if f == federate_name]

    def after_ns_for(self, from_port: str, to_port: str, in_process_ns: int | None = None) -> int:
        """The after of the cross-federate link ``from_port -> to_port``.

        An ``after_overrides`` entry wins. Without one the link keeps the delay
        it has in the one-process build (*in_process_ns*: ``--after-delay``,
        ``execution.yaml`` ``defaults.after_delay`` or a per-link
        ``after_delay_ns``), so a split changes no configured delay (an
        earlier finding: the per-node path dropped ``--after-delay 100ms`` to 0).
        Only a caller that has no in-process value gets the default.
        """
        self.queried_links.add((from_port, to_port))
        for ov in self.after_overrides:
            if ov.from_port == from_port and ov.to_port == to_port:
                return ov.after_ns
        if in_process_ns is not None:
            return in_process_ns
        return DEFAULT_CROSS_FEDERATE_AFTER_NS

    def stp_ns_for(self, from_port: str, to_port: str) -> int:
        """STP offset for a cross-federate link under decentralized coordination.

        Callers only call this when coordination is decentralized; centralized
        federations never look up an STP offset.
        """
        for ov in self.after_overrides:
            if ov.from_port == from_port and ov.to_port == to_port and ov.stp_offset_ns is not None:
                return ov.stp_offset_ns
        return DEFAULT_STP_OFFSET_NS


def budget_errors(
    fmap: FederationMap, *, generated_links: Iterable[tuple[str, str]],
) -> list[str]:
    """One message per decentralized link whose declared budget cannot hold.

    Item5 piece D (probes P27, P27b, P27c). A message is on time when
    ``L + eps + X + W_send <= d + W``:

    - ``d`` is the link's ``after``;
    - ``W`` the receiver's wait, its STP offset (``absent_after`` or
      ``stp_offset``; undeclared, the generator's default
      ``DEFAULT_STP_OFFSET_NS``, read through ``stp_ns_for`` like the
      generated code). The generator emits no LF ``maxwait``, so nothing else
      counts (probe P01);
    - ``L`` the declared max network latency (``max_latency``), ``eps`` the
      declared clock-sync error bound (``clock_error``), ``X`` the declared max
      execution time of the sender reaction (``max_exec``);
    - ``W_send`` the largest ``W`` over the sender federate's own inbound
      cross-federate links (0 without any): a federate with remote inputs holds
      its reactions at t until t + W (probe P07b), so its outputs leave up to
      ``W_send`` late. Only the direct sender counts, not a sum along the path
      (P27b); every link in *generated_links* counts with its effective ``W``
      (P27c).

    *generated_links* is the set of cross links the generator emits, the
    admitted xcorr edges' link ends included, keyed as ``generate.py`` keys
    them (``(source, user_destination or destination)``, the CONS3 key of a
    fan-in link). Only those links are checked and only they feed
    ``W_send``. An ``after_overrides`` entry whose key matches no emitted link
    carries no message: it gets no budget check and adds nothing to
    ``W_send``. Whether such an entry is an error is decided elsewhere and
    unchanged (``unmatched_override_errors``), as is the
    centralized-coordination refusal below, which applies to every entry.

    Equality passes. A link that declares some of L, eps, X must declare all
    three (``0`` included). A link that declares none is not checked and gets
    a warning. Under centralized coordination a declared term is an error: the
    RTI orders the tags and nothing would check it.
    """
    where = fmap.source or "federation"
    declared = [ov for ov in fmap.after_overrides if ov.declares_budget()]
    if fmap.coordination != "decentralized":
        return [
            f"{where}: link '{ov.from_port} -> {ov.to_port}' declares "
            f"{', '.join(BUDGET_KEYS)} terms, which only apply under decentralized "
            f"coordination (coordination: {fmap.coordination}); remove them"
            for ov in declared
        ]
    emitted = dict.fromkeys(tuple(link) for link in generated_links)
    w_send_by_fed: dict[str, int] = {}
    for from_port, to_port in emitted:
        src_fed = _federate_of_instance(fmap, from_port.split(".")[0])
        dst_fed = _federate_of_instance(fmap, to_port.split(".")[0])
        if dst_fed is None or src_fed == dst_fed:
            continue
        w_in = fmap.stp_ns_for(from_port, to_port)
        w_send_by_fed[dst_fed] = max(w_send_by_fed.get(dst_fed, 0), w_in)
    errors: list[str] = []
    for ov in fmap.after_overrides:
        if (ov.from_port, ov.to_port) not in emitted:
            continue  # no emitted link, no message crosses it
        edge = f"{ov.from_port} -> {ov.to_port}"
        src_fed = _federate_of_instance(fmap, ov.from_port.split(".")[0])
        if src_fed is not None and src_fed == _federate_of_instance(
                fmap, ov.to_port.split(".")[0]):
            continue  # a link inside one federate is no decentralized link
        terms = {KEY_MAX_LATENCY: ov.max_latency_ns, KEY_CLOCK_ERROR: ov.clock_error_ns,
                 KEY_MAX_EXEC: ov.max_exec_ns}
        missing = [k for k, v in terms.items() if v is None]
        if len(missing) == len(terms):
            print(f"  WARNING: {where}: decentralized link '{edge}' declares no budget "
                  f"({', '.join(BUDGET_KEYS)}), not checked", file=sys.stderr)
            continue
        if missing:
            errors.append(
                f"{where}: decentralized link '{edge}' declares part of its budget but "
                f"not {', '.join(missing)}; declare max_latency (L), clock_error (eps) and "
                f"max_exec (X), 0 included"
            )
            continue
        w_ns = fmap.stp_ns_for(ov.from_port, ov.to_port)
        w_note = " default" if ov.stp_offset_ns is None else ""
        lhs = ov.after_ns + w_ns
        sender_fed = _federate_of_instance(fmap, ov.from_port.split(".")[0])
        w_send_ns = w_send_by_fed.get(sender_fed, 0) if sender_fed else 0
        rhs = ov.max_latency_ns + ov.clock_error_ns + ov.max_exec_ns + w_send_ns
        if lhs < rhs:
            errors.append(
                f"{where}: decentralized link '{edge}' violates d + W >= L + eps + X + W_send: "
                f"d={ov.after_ns} ns (after), W={w_ns} ns (stp_offset{w_note}), "
                f"L={ov.max_latency_ns} ns (max_latency), eps={ov.clock_error_ns} ns "
                f"(clock_error), X={ov.max_exec_ns} ns (max_exec), W_send={w_send_ns} ns "
                f"(sender federate '{sender_fed}' waits for its own remote inputs): "
                f"d + W = {lhs} ns < L + eps + X + W_send = {rhs} ns, short by {rhs - lhs} ns"
            )
    return errors


# Generated correspondence ports (corr_e<N>, xcorr_e<N>)
# are looked up like user links but are no override a user would write.
_CORR_PORT_RE = re.compile(r"\.x?corr_e\d+$")


def unmatched_override_errors(fmap: FederationMap) -> list[str]:
    """One message per ``after_overrides`` entry that names no cross-federate link.

    Call it after the generator has looked up the after delay of every
    cross-federate link. An entry that matches none of those lookups changed
    nothing, so it is reported with the valid keys of the federate that owns
    its ``from`` instance (all valid keys if that instance is unknown),
    so a misspelled port never falls back to the default silently.
    """
    errors: list[str] = []
    for ov in fmap.after_overrides:
        if (ov.from_port, ov.to_port) in fmap.queried_links:
            continue
        src_inst = ov.from_port.split(".")[0]
        src_fed = _federate_of_instance(fmap, src_inst)
        valid = sorted(
            f"{a} -> {b}" for a, b in fmap.queried_links
            if not _CORR_PORT_RE.search(a)
            and (src_fed is None or _federate_of_instance(fmap, a.split(".")[0]) == src_fed)
        )
        scope = f"federate '{src_fed}'" if src_fed else "this federation"
        listing = ", ".join(valid) if valid else "(none)"
        errors.append(
            f"{fmap.source or 'federation'}: after_overrides entry "
            f"'{ov.from_port} -> {ov.to_port}' matches no cross-federate link. "
            f"Valid keys of {scope}: {listing}"
        )
    return errors


def _federate_of_instance(fmap: FederationMap, instance: str) -> str | None:
    """Federate of an LF instance name (a sanitized node key)."""
    from colcon_lf.generation.lf_naming import sanitize_lf_identifier

    for node, fed in fmap.node_to_federate.items():
        if sanitize_lf_identifier(node) == instance:
            return fed
    return None


def resolve_partition(
    federate_arg: str,
    graph: "SystemGraph",
    system_json: Path | None = None,
) -> FederationMap:
    """Resolve federation partition from --federate argument value."""
    if federate_arg == MODE_PER_PROCESS:
        fmap = _per_process(graph)
    elif federate_arg == MODE_PER_NODE:
        fmap = _per_node(graph)
    else:
        fmap = _resolve_yaml(federate_arg, graph)
    check_federate_names(fmap)
    return fmap


def _resolve_yaml(federate_arg: str, graph: "SystemGraph") -> FederationMap:
    # Treat as YAML path
    yaml_path = Path(federate_arg)
    if not yaml_path.is_file():
        print(
            f"ERROR: federation YAML not found: {yaml_path}",
            file=sys.stderr,
        )
        raise FileNotFoundError(yaml_path)
    return _from_yaml(yaml_path, graph)


# ---------------------------------------------------------------------------
# Mode implementations
# ---------------------------------------------------------------------------

def _per_process(graph: "SystemGraph") -> FederationMap:
    """One federate per OS process group derived from the launch graph.

    Groups nodes by composable_container: composable nodes sharing the same
    container form one federate. Non-component nodes each get their own federate
    (each is a separate executable process).
    Falls back to per-node if no launch graph information is available.
    """
    from colcon_lf.generation.lf_naming import sanitize_lf_identifier

    fmap = FederationMap(source="per-process")

    # Derive process groups from SystemGraph.nodes (NodeInfo.is_component / package)
    # and LaunchGraph node actions (composable_container field).
    # Strategy: group by composable_container; non-components are each their own group.
    launch_graph = getattr(graph, "launch_graph", None)
    if launch_graph is None:
        return _per_node(graph)

    # Build container -> [node_key] map from launch_graph.all_nodes
    container_to_nodes: dict[str, list[str]] = {}
    standalone_nodes: list[str] = []

    all_launch_nodes = getattr(launch_graph, "all_nodes", [])
    # Map launch node name -> graph node key
    # graph.nodes is keyed by launch-level node name (same as system.json)
    for node_key in graph.nodes:
        ni = graph.nodes[node_key]
        # Find the matching launch node action
        container = None
        for ln in all_launch_nodes:
            if getattr(ln, "is_component", False) and ln.composable_container:
                # Match by class name or node name
                if (ln.name == node_key or
                        getattr(ni, "class_name", "") == ln.executable.rsplit(".", 1)[-1]):
                    container = lfc_safe_federate_name(
                        sanitize_lf_identifier(ln.composable_container))
                    break
        if container:
            container_to_nodes.setdefault(container, []).append(node_key)
        else:
            standalone_nodes.append(node_key)

    if not container_to_nodes and not standalone_nodes:
        return _per_node(graph)

    # Assign container groups
    for container, node_keys in container_to_nodes.items():
        if container not in fmap.federate_names:
            fmap.federate_names.append(container)
        for nk in node_keys:
            fmap.node_to_federate[nk] = container

    # Standalone nodes each get their own federate (they are their own process)
    seen_names: dict[str, int] = {}
    for node_key in standalone_nodes:
        base = sanitize_lf_identifier(node_key)
        if base in seen_names:
            seen_names[base] += 1
            fed_name = f"{base}_{seen_names[base]}"
        else:
            seen_names[base] = 0
            fed_name = base
        fed_name = lfc_safe_federate_name(fed_name)
        fmap.federate_names.append(fed_name)
        fmap.node_to_federate[node_key] = fed_name

    # Sanity: assign any remaining unassigned nodes
    unassigned = [n for n in graph.nodes if n not in fmap.node_to_federate]
    if unassigned:
        for n in unassigned:
            base = lfc_safe_federate_name(sanitize_lf_identifier(n))
            fmap.federate_names.append(base)
            fmap.node_to_federate[n] = base

    if not fmap.federate_names:
        return _per_node(graph)

    return fmap


def _per_node(graph: "SystemGraph") -> FederationMap:
    """One federate per ROS2 node."""
    from colcon_lf.generation.lf_naming import sanitize_lf_identifier

    fmap = FederationMap(source="per-node")
    seen: dict[str, int] = {}

    for node_name in graph.nodes:
        base = sanitize_lf_identifier(node_name)
        if base in seen:
            seen[base] += 1
            fed_name = f"{base}_{seen[base]}"
        else:
            seen[base] = 0
            fed_name = base
        fed_name = lfc_safe_federate_name(fed_name)
        fmap.federate_names.append(fed_name)
        fmap.node_to_federate[node_name] = fed_name

    return fmap


def _from_yaml(yaml_path: Path, graph: "SystemGraph") -> FederationMap:
    """Load explicit partition from YAML file."""
    import yaml

    raw = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{yaml_path}: expected a YAML mapping at top level")

    federates_raw = raw.get(KEY_FEDERATES)
    if not federates_raw or not isinstance(federates_raw, dict):
        raise ValueError(f"{yaml_path}: missing or empty '{KEY_FEDERATES}' section")

    fmap = FederationMap(source=str(yaml_path))
    all_graph_nodes = set(graph.nodes.keys())

    for fed_name, fed_body in federates_raw.items():
        fed_id = str(fed_name)
        if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", fed_id):
            raise ValueError(
                f"{yaml_path}: federate name '{fed_id}' is not a valid identifier"
            )
        if fed_id in fmap.federate_names:
            raise ValueError(
                f"{yaml_path}: federate name '{fed_id}' appears more than once"
            )
        fmap.federate_names.append(fed_id)

        nodes = (fed_body or {}).get(KEY_NODES, []) if isinstance(fed_body, dict) else []
        if not nodes:
            raise ValueError(
                f"{yaml_path}: federate '{fed_id}' has no nodes listed under '{KEY_NODES}'"
            )
        for node_name in nodes:
            node_name = str(node_name)
            if node_name not in all_graph_nodes:
                raise ValueError(
                    f"{yaml_path}: node '{node_name}' in federate '{fed_id}' "
                    f"is not in system.json (was it renamed?)"
                )
            if node_name in fmap.node_to_federate:
                existing = fmap.node_to_federate[node_name]
                raise ValueError(
                    f"{yaml_path}: node '{node_name}' appears in both "
                    f"'{existing}' and '{fed_id}'"
                )
            fmap.node_to_federate[node_name] = fed_id

    # Check all graph nodes are assigned
    unassigned = all_graph_nodes - set(fmap.node_to_federate.keys())
    if unassigned:
        names = ", ".join(sorted(unassigned))
        raise ValueError(
            f"{yaml_path}: nodes not assigned to any federate: {names}"
        )

    # Parse after_overrides (after-delay, and optionally per-link stp_offset)
    overrides_raw = raw.get(KEY_AFTER_OVERRIDES) or []
    for entry in overrides_raw:
        if not isinstance(entry, dict):
            continue
        from_port = str(entry.get(KEY_FROM, ""))
        to_port = str(entry.get(KEY_TO, ""))
        after_val = entry.get(KEY_AFTER, "5ms")
        after_ns = _parse_duration_ns(str(after_val), yaml_path)
        stp_val = entry.get(KEY_STP_OFFSET)
        absent_val = entry.get(KEY_ABSENT_AFTER)
        if stp_val is not None and absent_val is not None:
            raise ValueError(
                f"{yaml_path}: link '{from_port} -> {to_port}' gives both "
                f"'{KEY_STP_OFFSET}' and '{KEY_ABSENT_AFTER}' (aliases), keep one"
            )
        if stp_val is None:
            stp_val = absent_val
        stp_ns = _parse_duration_ns(str(stp_val), yaml_path) if stp_val is not None else None
        budget = {
            key: (_parse_duration_ns(str(entry[key]), yaml_path)
                  if entry.get(key) is not None else None)
            for key in BUDGET_KEYS
        }
        fmap.after_overrides.append(AfterOverride(
            from_port, to_port, after_ns, stp_ns,
            max_latency_ns=budget[KEY_MAX_LATENCY], clock_error_ns=budget[KEY_CLOCK_ERROR],
            max_exec_ns=budget[KEY_MAX_EXEC],
        ))

    # Parse optional top-level coordination mode
    coordination_raw = raw.get(KEY_COORDINATION)
    if coordination_raw is not None:
        coordination = str(coordination_raw)
        if coordination not in VALID_COORDINATION_MODES:
            raise ValueError(
                f"{yaml_path}: '{KEY_COORDINATION}' must be one of "
                f"{VALID_COORDINATION_MODES}, got {coordination!r}"
            )
        fmap.coordination = coordination

    return fmap


def _parse_duration_ns(value: str, source: Path) -> int:
    """Parse a federation-YAML duration into nanoseconds.

    Delegates to the generator's `parse_lf_time_ns` so federation
    YAML accepts the same grammar as `--after-delay` and `execution.yaml`: the
    LF spelling (`5 msec`, `500 usec`) as well as the terse spelling (`5ms`,
    `100ns`) this key has always taken.

    The wrapper is kept rather than inlining the call because the useful part
    of the error is the YAML path, and because two inputs the old regex
    accepted are now rejected on purpose:

    * a bare number — it used to mean nanoseconds via `unit or "ns"`, which is
      the silent-unit-guess failure mode that made `--after-delay` a dead
      parameter. `0` is still fine without a unit.
    * an empty value — `parse_lf_time_ns` reports "not configured" as None, but
      an `after_overrides` entry that reached here was configured, so a None
      is an error rather than a value to propagate.
    """
    ns = parse_lf_time_ns(value, source=str(source))
    if ns is None:
        raise ValueError(f"{source}: '{KEY_AFTER}' must not be empty")
    return ns

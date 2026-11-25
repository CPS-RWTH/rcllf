# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Topology model loaded from ``system.json`` (output of ``colcon lf-analyze``).

``system.json`` (schema_version 3 as of the current generator —
``tools/colcon_lf/colcon_lf/verb/analyze.py``; the
bump from 2 to 3 added the ``diagnostics`` list) has the shape::

    {
      "name": "bringup",
      "nodes": {"<node key>": {"name", "class_name", "timers": [{"period_ms"}],
                               "publishers": [{"topic"}], "subscribers": [{"topic"}]}},
      "communication": {"topics": [{"topic", "publishers": [...], "subscribers": [...]}]},
      "schema_version": 3,
      "diagnostics": [...]
    }

Note the ``communication.topics[].publishers`` / ``.subscribers`` entries are
*node keys*, which in the reference-system model are lower-cased class names
(``frontlidardriver``) while ``nodes`` is keyed by class name
(``FrontLidarDriver``). :meth:`SystemModel.resolve_node` therefore matches
case-insensitively across key, ``name`` and ``class_name``.

:func:`parse_system` reads defensively (``.get(..., default)`` throughout) and
records whatever ``schema_version`` the file declares (or ``None`` when
absent) on :attr:`SystemModel.schema_version` rather than validating it — a
missing/unrecognised value does not raise here. Consumers that need to react
to an unexpected schema (e.g. the ``topology`` metric,
:mod:`lf_eval.metrics.topology`) check :attr:`SystemModel.schema_version`
themselves and note rather than crash on a value outside what they were
validated against.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterable, Optional


def canonical_key(ident: str) -> str:
    """Normalise a node identifier for cross-convention comparison.

    Lower-cases, strips a leading ``/`` namespace marker, and removes ``_``/``-``
    separators so that ``publisher_node``, ``PublisherNode`` and
    ``/publisher_node`` all collapse to ``publishernode``, and
    ``mpccontroller`` == ``MPCController``.

    This is what makes a trace's ``rcl_node_init.node_name`` comparable to
    whichever ``system.json`` field happens to match it on a given system.
    """
    return ident.strip().lstrip("/").replace("_", "").replace("-", "").lower()


@dataclass(frozen=True)
class TimerSpec:
    """A timer declared by a node."""

    period_ms: float
    callback: Optional[str] = None


@dataclass(frozen=True)
class NodeSpec:
    """One node in the extracted system model."""

    key: str
    name: str
    class_name: str
    package: str = ""
    timers: tuple[TimerSpec, ...] = ()
    publishes: tuple[str, ...] = ()
    subscribes: tuple[str, ...] = ()
    #: Topics this node is *expected* to publish/subscribe as a consequence of
    #: infrastructure it declares (action server/client endpoints, lifecycle
    #: transition events) rather than an explicit ``create_publisher`` call.
    #: Kept separate from :attr:`publishes`/:attr:`subscribes` so consumers that
    #: care about application data flow (chains, plots) are unaffected, while
    #: the topology diff can treat them as declared. See
    #: :func:`derive_infrastructure_edges`.
    infra_publishes: tuple[str, ...] = ()
    infra_subscribes: tuple[str, ...] = ()

    @property
    def period_ms(self) -> Optional[float]:
        """Period of the node's first timer, if it has one."""
        return self.timers[0].period_ms if self.timers else None

    @property
    def canonical_id(self) -> str:
        """Stable identity for this node across naming conventions.

        ``system.json``'s three identifiers disagree between systems, and the
        trace's ``rcl_node_init.node_name`` matches a *different* one depending
        on the system:

        ===================  ==============  ==============  ==============
        system               ``key``         ``name``        ``class_name``
        ===================  ==============  ==============  ==============
        pubsub_example       publisher_node  publisher_node  PublisherNode
        reference-system     MPCController   mpccontroller   MPCController
        ===================  ==============  ==============  ==============

        The pubsub trace reports ``publisher_node`` (matching ``name``); the
        reference-system trace reports ``MPCController`` (matching ``key`` and
        ``class_name``, differing from ``name`` only in case). So no single
        field works — see :meth:`SystemModel.canonical_of`.

        The canonical id is the model's ``key``, which is the one identifier
        guaranteed unique within a ``system.json``. It is only ever compared via
        :func:`canonical_key`, never matched literally against trace names.
        """
        return self.key

    @property
    def aliases(self) -> frozenset[str]:
        """Every identifier this node may be referred to by, canonicalised."""
        return frozenset(
            canonical_key(value)
            for value in (self.key, self.name, self.class_name)
            if value
        )

    @property
    def display_name(self) -> str:
        """Human-facing label: the class name when known, else the key.

        Chains and plots show this, so output keeps reading the way it did
        before canonicalisation (``FrontLidarDriver``, not ``frontlidardriver``).
        """
        return self.class_name or self.name or self.key


@dataclass(frozen=True)
class TopicSpec:
    """One topic edge in the extracted system model."""

    name: str
    message_type: str = ""
    publishers: tuple[str, ...] = ()
    subscribers: tuple[str, ...] = ()


@dataclass
class SystemModel:
    """Topology extracted from ``system.json``."""

    name: str
    nodes: dict[str, NodeSpec] = field(default_factory=dict)
    topics: tuple[TopicSpec, ...] = ()
    source_path: Optional[Path] = None
    #: ``schema_version`` as declared by the source ``system.json``, or
    #: ``None`` when the file predates the field / wasn't loaded from JSON.
    #: Recorded, not validated, here — see the module docstring.
    schema_version: Optional[int] = None

    # ── lookups ──────────────────────────────────────────────────────────

    def resolve_node(self, ident: str) -> Optional[NodeSpec]:
        """Find a node by key, ``name`` or ``class_name``.

        Matching is separator- and case-insensitive (see :func:`canonical_key`),
        so a trace node name resolves regardless of which ``system.json`` field
        it happens to correspond to on this system.
        """
        if ident in self.nodes:
            return self.nodes[ident]
        wanted = canonical_key(ident)
        for node in self.nodes.values():
            if wanted in node.aliases:
                return node
        return None

    def canonical_of(self, ident: str) -> Optional[str]:
        """Canonical id for ``ident``, or ``None`` when it is not in the model."""
        node = self.resolve_node(ident)
        return node.canonical_id if node else None

    def display_of(self, canonical_id: str) -> str:
        """Human-facing label for a canonical id (falls back to the id)."""
        node = self.nodes.get(canonical_id)
        return node.display_name if node else canonical_id

    def timer_periods(self) -> dict[str, float]:
        """Map ``canonical_id -> first timer period in ms`` for timer nodes."""
        return {
            node.canonical_id: node.timers[0].period_ms
            for node in self.nodes.values()
            if node.timers
        }

    def topic_qos_depths(self) -> dict[str, int]:
        """Not derivable from topics alone; see :func:`load_system` notes."""
        return {}

    # ── chain derivation ─────────────────────────────────────────────────

    def successors(self) -> dict[str, set[str]]:
        """Node-level DAG: ``canonical_id -> downstream canonical_ids``."""
        edges: dict[str, set[str]] = {
            n.canonical_id: set() for n in self.nodes.values()
        }
        for topic in self.topics:
            pubs = [self.resolve_node(p) for p in topic.publishers]
            subs = [self.resolve_node(s) for s in topic.subscribers]
            for pub in pubs:
                if pub is None:
                    continue
                for sub in subs:
                    if sub is None or sub.canonical_id == pub.canonical_id:
                        continue
                    edges.setdefault(pub.canonical_id, set()).add(sub.canonical_id)
        return edges

    def longest_chain(self) -> list[str]:
        """Longest simple path through the node DAG, as canonical ids.

        Replaces the hardcoded ``PRIMARY_CHAIN`` of the predecessor script. For
        the reference system this recovers the sensor-to-control path; for the
        pubsub example it yields the publisher -> subscriber pair.

        Cycles are tolerated: the DFS never revisits a node already on the
        current path, so a cyclic graph simply yields the longest acyclic walk.
        """
        edges = self.successors()
        best: list[str] = []

        # Sorted iteration keeps the result deterministic across runs.
        def walk(node: str, path: list[str]) -> None:
            nonlocal best
            if len(path) > len(best):
                best = list(path)
            for nxt in sorted(edges.get(node, ())):
                if nxt in path:
                    continue
                path.append(nxt)
                walk(nxt, path)
                path.pop()

        for start in sorted(edges):
            walk(start, [start])
        return best

    def chain_from_spec(self, spec: str) -> list[str]:
        """Parse a ``--chain A->B->C`` spec into canonical ids.

        Accepts ``->``, ``,`` or whitespace separators. Each element may be
        written in *any* of the model's naming conventions — ``publisher_node``,
        ``PublisherNode`` and ``/publisher_node`` are equivalent — because
        lookup goes through :meth:`resolve_node`. Every element must resolve, so
        typos fail loudly rather than silently producing an empty chain.
        """
        raw = [p.strip() for p in spec.replace("->", ",").split(",")]
        parts = [p for p in raw if p]
        if len(parts) < 2:
            raise ValueError(
                f"chain spec needs at least two nodes, got {spec!r}"
            )
        resolved: list[str] = []
        for part in parts:
            node = self.resolve_node(part)
            if node is None:
                known = ", ".join(
                    sorted(n.display_name for n in self.nodes.values())
                )
                raise ValueError(
                    f"unknown node {part!r} in chain spec; known nodes: {known}"
                )
            resolved.append(node.canonical_id)
        return resolved

    def timer_gated_nodes(self) -> set[str]:
        """Nodes that own a timer *and* only publish from it.

        Such a node re-times data: incoming messages wait for the next tick
        instead of propagating immediately (the reference system's
        ``BehaviorPlanner`` is the canonical case, which the predecessor script
        special-cased with a hardcoded ``TIMER_GATED_NODES`` set). Detected
        here as "has a timer and has at least one subscription", which is the
        structural signature of a timer-gated fusion node.

        Returns canonical ids.
        """
        gated: set[str] = set()
        for node in self.nodes.values():
            if node.timers and node.subscribes:
                gated.add(node.canonical_id)
        return gated


def load_system(path: Path) -> SystemModel:
    """Load and normalise a ``system.json`` file."""
    path = Path(path)
    with path.open() as handle:
        data = json.load(handle)
    return parse_system(data, source_path=path)


#: Base classes whose nodes publish ``<node>/transition_event``.
_LIFECYCLE_BASE_MARKERS = ("lifecyclenode",)

#: The two action endpoints that are ROS *topics*. An action's goal, result and
#: cancel endpoints are services, so they never appear as publisher/subscriber
#: init events in a trace and must not be derived here — deriving them would
#: silently suppress a real finding. Confirmed against the Phase-7 soundness
#: traces of the action_example bringup launch, where the
#: server publishes and the client subscribes to exactly these two.
_ACTION_TOPIC_ENDPOINTS = ("status", "feedback")


def _is_lifecycle_node(raw: dict) -> bool:
    """Whether a raw ``system.json`` node entry describes a lifecycle node."""
    base = str(raw.get("base_class") or "").replace("_", "").replace(":", "").lower()
    if any(marker in base for marker in _LIFECYCLE_BASE_MARKERS):
        return True
    # Heuristic fallback mirroring the extractor's own: a node with lifecycle
    # callbacks but an unresolved base class (missing headers) is still one.
    return not base and bool(raw.get("lifecycle_callbacks"))


def derive_infrastructure_edges(
    data: dict, nodes: dict[str, "NodeSpec"]
) -> dict[str, tuple[tuple[str, ...], tuple[str, ...]]]:
    """Infrastructure topics implied by a system's actions and lifecycle nodes.

    Returns ``canonical_id -> (infra_publishes, infra_subscribes)``.

    ROS creates topics that no ``create_publisher``/``create_subscription``
    call in the source mentions, so the static extractor never lists them and
    the topology diff reported them as ``missing_from_model``.
    They are nonetheless *derivable* from data ``system.json`` already carries:

    * ``communication.actions[*]`` gives the action's fully-qualified name plus
      its ``server`` and ``clients``; the server publishes and each client
      subscribes to ``<action>/_action/{status,feedback}``.
    * a node whose ``base_class`` is a lifecycle base publishes
      ``/<ros_node_name>/transition_event``.

    Deriving beats blanket-excluding: an action edge on a node that is *not*
    that action's server/client, or a ``transition_event`` from a non-lifecycle
    node, still surfaces as a finding.
    """
    derived: dict[str, tuple[list[str], list[str]]] = {
        cid: ([], []) for cid in nodes
    }

    def _resolve(ident: str) -> Optional[str]:
        """Map a system.json node reference to a canonical id."""
        if not ident:
            return None
        if ident in nodes:
            return ident
        wanted = canonical_key(ident)
        for cid, spec in nodes.items():
            if wanted in spec.aliases:
                return cid
        return None

    communication = data.get("communication") or {}
    for action in communication.get("actions") or []:
        action_name = str(action.get("action") or "").strip()
        if not action_name:
            continue
        endpoints = [
            f"{action_name.rstrip('/')}/_action/{suffix}"
            for suffix in _ACTION_TOPIC_ENDPOINTS
        ]
        server_cid = _resolve(str(action.get("server") or ""))
        if server_cid is not None:
            derived[server_cid][0].extend(endpoints)
        for client in action.get("clients") or []:
            client_cid = _resolve(str(client))
            if client_cid is not None:
                derived[client_cid][1].extend(endpoints)

    for key, raw in (data.get("nodes") or {}).items():
        if not _is_lifecycle_node(raw):
            continue
        cid = _resolve(key)
        if cid is None:
            continue
        ros_name = str(raw.get("name") or key).strip().lstrip("/")
        if ros_name:
            derived[cid][0].append(f"/{ros_name}/transition_event")

    return {
        cid: (tuple(pubs), tuple(subs)) for cid, (pubs, subs) in derived.items()
    }


def parse_system(data: dict, source_path: Optional[Path] = None) -> SystemModel:
    """Build a :class:`SystemModel` from parsed ``system.json`` data."""
    nodes: dict[str, NodeSpec] = {}
    for key, raw in (data.get("nodes") or {}).items():
        name = raw.get("name") or key
        class_name = raw.get("class_name") or name
        timers = tuple(
            TimerSpec(
                period_ms=float(t.get("period_ms") or 0.0),
                callback=t.get("callback"),
            )
            for t in (raw.get("timers") or [])
            if (t.get("period_ms") or 0) > 0
        )
        nodes[key] = NodeSpec(
            key=key,
            name=name,
            class_name=class_name,
            package=raw.get("package", ""),
            timers=timers,
            publishes=tuple(
                p.get("topic", "") for p in (raw.get("publishers") or [])
            ),
            subscribes=tuple(
                s.get("topic", "") for s in (raw.get("subscribers") or [])
            ),
        )

    # Infrastructure edges (action endpoints, lifecycle transition events) are
    # derived once here and attached to the nodes they belong to, so the
    # topology diff sees them as declared without them leaking into the
    # application-level publishes/subscribes used for chains.
    for cid, (infra_pubs, infra_subs) in derive_infrastructure_edges(
        data, nodes
    ).items():
        if infra_pubs or infra_subs:
            nodes[cid] = replace(
                nodes[cid],
                infra_publishes=infra_pubs,
                infra_subscribes=infra_subs,
            )

    topics = tuple(
        TopicSpec(
            name=t.get("topic", ""),
            message_type=t.get("message_type", ""),
            publishers=tuple(t.get("publishers") or ()),
            subscribers=tuple(t.get("subscribers") or ()),
        )
        for t in ((data.get("communication") or {}).get("topics") or [])
    )

    return SystemModel(
        name=data.get("name", source_path.stem if source_path else "system"),
        nodes=nodes,
        topics=topics,
        source_path=source_path,
        schema_version=data.get("schema_version"),
    )


def find_system_json(candidates: Iterable[Path]) -> Optional[Path]:
    """Return the first existing path among ``candidates``."""
    for candidate in candidates:
        candidate = Path(candidate)
        if candidate.is_file():
            return candidate
    return None

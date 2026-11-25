# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``ExecConfig`` — load and validate the sparse ``execution.yaml`` file.

Entry point: ``ExecConfig.load(path, graph) -> ExecConfig``.

Canonical key form:
node and topic keys are byte-identical to ``system.json``
(``SystemGraph.nodes`` keys / ``SubscriberInfo.topic``). Input validation is
by runtime name: a node key names the node whose fully
qualified name it spells (``controller`` and ``/controller`` are one node), and
a topic key names the model topic that rclcpp resolves to the same name for that
node (relative names against the node's namespace, absolute names as written).
For a node in the root namespace ``/sensor_data`` and ``sensor_data`` are one
topic; for a node in ``/robot`` they are two. Keys normalize to the model form.
Keys that match nothing in the model, and two spellings in one section that
resolve to one runtime name, are a hard ``ValueError``.

All validation errors are ``ValueError`` prefixed ``"{path}: ..."`` (the
federation YAML idiom, ``federation/partition.py``).
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import yaml

from colcon_lf.exec_config.policies import (
    INPUT_ALIASES,
    INPUT_NAMES,
    REGISTRY,
    SCOPE_DEFAULTS,
    SCOPE_NODES,
    SCOPE_SUBSCRIPTIONS,
    parse_timer_offset,
    parse_timer_period,
)
from colcon_lf.exec_config.yaml_schema import (
    EXT_ANCHOR_ARRIVAL,
    EXT_ANCHOR_MAX_NS,
    EXT_ANCHOR_STAMP,
    EXT_ANCHORS,
    EXT_ENTRY_KEYS,
    KEY_ACKNOWLEDGED_HAZARDS,
    KEY_CONNECTIONS,
    KEY_EXT_ANCHOR,
    KEY_EXT_BASIS,
    KEY_EXT_DELAY_NS,
    KEY_EXT_GRID_NS,
    KEY_EXT_RESERVE_NS,
    KEY_EXT_TOPIC,
    KEY_EXTERNAL_TOPICS,
    KEY_DEFAULTS,
    KEY_NODES,
    KEY_PUB_BASIS,
    KEY_PUB_OUTPUT_DEADLINE_NS,
    KEY_PUBLISHERS,
    KEY_SCHEMA_VERSION,
    KEY_SUBSCRIPTIONS,
    KEY_TIMER_OFFSET,
    KEY_TIMER_OFFSET_RUNTIME,
    KEY_TIMER_PERIOD,
    KEY_TF2,
    KEY_TF2_ON_TIMEOUT,
    KEY_TF2_WAIT_BOUND,
    KEY_TIMERS,
    TF2_KEYS,
    SCHEMA_VERSION,
    TIMER_KEYS,
    PUB_DEADLINE_MAX_NS,
    PUB_ENTRY_KEYS,
    TOP_LEVEL_KEYS,
)

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph


@dataclass
class ExecConfig:
    """Parsed + validated sparse ``execution.yaml``.

    Keys in ``defaults``/``nodes``/``subscriptions`` are already normalized to
    the canonical (model) form. ``nodes``/``subscriptions`` are only present
    for node/topic pairs that actually exist in the model.
    """

    schema_version: int
    defaults: dict[str, object] = field(default_factory=dict)
    # node_key -> {policy_key: value}
    nodes: dict[str, dict[str, object]] = field(default_factory=dict)
    # node_key -> topic_key -> {policy_key: value}
    subscriptions: dict[str, dict[str, dict[str, object]]] = field(default_factory=dict)
    acknowledged_hazards: list[str] = field(default_factory=list)
    # Resolved topic names (``TopicConnection.topic`` form) declared as fed
    # from outside the system.
    external_topics: list[str] = field(default_factory=list)
    # Resolved topic -> {anchor, delay_ns, ingress_reserve_ns, grid_ns, basis}
    # for every external topic written as a mapping. A topic
    # absent here is anchored at arrival.
    external_anchors: dict[str, dict] = field(default_factory=dict)
    # Link name -> {after_delay, reason, source?, links?: {(sender, receiver): {...}}}
    # (``exec_config/connections.py``).
    connections: dict[str, dict] = field(default_factory=dict)
    # node_key -> publisher topic -> {output_deadline_ns, basis}.
    publishers: dict[str, dict[str, dict]] = field(default_factory=dict)
    source: str = ""
    # Value path (e.g. ``("subscriptions", node, topic, "take_mode")``) ->
    # the layer that set it, filled by ``load_layered``.
    origins: dict[tuple, str] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path, graph: "SystemGraph") -> "ExecConfig":
        """Load and validate a sparse ``execution.yaml`` against *graph*."""
        path = Path(path)
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        return cls.from_dict(raw, graph, source=str(path))

    @classmethod
    def empty(cls) -> "ExecConfig":
        """An empty config (used when no ``execution.yaml`` is present)."""
        return cls(schema_version=SCHEMA_VERSION, source="<built-in defaults>")

    @classmethod
    def load_layered(
        cls,
        workspace: Path | None,
        package: Path | None,
        graph: "SystemGraph",
        *,
        user_label: str = "user",
    ) -> "ExecConfig":
        """Layer the user file over the package-owned file, value by value.

        Each file is validated on its own (errors name that file). Then every
        policy value is merged at its own path: a value the user file sets
        wins over the same path in the package file, and every path the user
        file leaves unset (the scaffolded file leaves all of them unset) comes
        from the package file. ``external_topics`` and ``acknowledged_hazards``
        are the union of both layers. Scope precedence (defaults < nodes <
        subscriptions) applies after this merge, in ``resolve``. ``origins``
        records the layer of every merged value. With neither file present
        this is ``empty()``.
        """
        layers: list[tuple[str, ExecConfig]] = []
        for label, path in (("package", package), (user_label, workspace)):
            if path is not None and Path(path).is_file():
                layers.append((f"{label} {path}", cls.load(Path(path), graph)))
        if not layers:
            return cls.empty()
        merged = cls(
            schema_version=SCHEMA_VERSION,
            source=" over ".join(label for label, _ in reversed(layers)),
        )
        for label, layer in layers:
            for key, value in layer.defaults.items():
                merged.defaults[key] = value
                merged.origins[(KEY_DEFAULTS, key)] = label
            for node, body in layer.nodes.items():
                target = merged.nodes.setdefault(node, {})
                for key, value in body.items():
                    if key == KEY_TF2:
                        # Per field, like timers.
                        tf2 = target.setdefault(KEY_TF2, {})
                        for name, val in value.items():
                            tf2[name] = val
                            merged.origins[(KEY_NODES, node, KEY_TF2, name)] = label
                        continue
                    if key == KEY_TIMERS:
                        # Per field: a user `offset` keeps the package `period`.
                        timers = target.setdefault(KEY_TIMERS, {})
                        for timer, fields in value.items():
                            for name, literal in fields.items():
                                timers.setdefault(timer, {})[name] = literal
                                merged.origins[
                                    (KEY_NODES, node, KEY_TIMERS, timer, name)
                                ] = label
                        continue
                    target[key] = value
                    merged.origins[(KEY_NODES, node, key)] = label
            for node, topics in layer.subscriptions.items():
                for topic, body in topics.items():
                    target = merged.subscriptions.setdefault(node, {}).setdefault(topic, {})
                    for key, value in body.items():
                        target[key] = value
                        merged.origins[(KEY_SUBSCRIPTIONS, node, topic, key)] = label
            for code in layer.acknowledged_hazards:
                if code not in merged.acknowledged_hazards:
                    merged.acknowledged_hazards.append(code)
                    merged.origins[(KEY_ACKNOWLEDGED_HAZARDS, code)] = label
            for topic in layer.external_topics:
                if topic not in merged.external_topics:
                    merged.external_topics.append(topic)
                    merged.origins[(KEY_EXTERNAL_TOPICS, topic)] = label
            # A mapping entry replaces the topic's whole anchor (the user
            # file's constants replace the package file's as one unit).
            for topic, anchor in layer.external_anchors.items():
                merged.external_anchors[topic] = dict(anchor)
                merged.origins[(KEY_EXTERNAL_TOPICS, topic, KEY_EXT_ANCHOR)] = label
            # A publisher entry replaces the whole entry (the bound
            # and its basis are one unit, as an external anchor's constants).
            for node, topics in layer.publishers.items():
                for topic, body in topics.items():
                    merged.publishers.setdefault(node, {})[topic] = dict(body)
                    merged.origins[(KEY_PUBLISHERS, node, topic)] = label
            # A link's own value and each of its pairs merge as one entry each:
            # the user file's entry replaces the package file's whole entry.
            for name, body in layer.connections.items():
                target = merged.connections.setdefault(name, {})
                own = {k: v for k, v in body.items() if k != "links"}
                if own:
                    for k in ("after_delay", "reason", "source"):
                        target.pop(k, None)
                    target.update(own)
                    merged.origins[(KEY_CONNECTIONS, name)] = label
                for pair, pair_body in body.get("links", {}).items():
                    target.setdefault("links", {})[pair] = pair_body
                    merged.origins[(KEY_CONNECTIONS, name, pair)] = label
        return merged

    def origin(self, path: tuple) -> str:
        """The layer that set the value at *path* (see ``load_layered``)."""
        return self.origins.get(path, self.source)

    @classmethod
    def from_dict(
        cls,
        raw: object,
        graph: "SystemGraph",
        *,
        source: str,
    ) -> "ExecConfig":
        """Validate a parsed YAML mapping against *graph*, returning an ``ExecConfig``."""
        if raw is None:
            raw = {}
        if not isinstance(raw, dict):
            raise ValueError(f"{source}: expected a YAML mapping at top level")

        unknown_top = set(raw.keys()) - TOP_LEVEL_KEYS
        if unknown_top:
            names = ", ".join(sorted(map(str, unknown_top)))
            raise ValueError(f"{source}: unknown top-level key(s): {names}")

        schema_version = raw.get(KEY_SCHEMA_VERSION, SCHEMA_VERSION)
        if not isinstance(schema_version, int) or schema_version != SCHEMA_VERSION:
            raise ValueError(
                f"{source}: unsupported schema_version {schema_version!r} "
                f"(expected {SCHEMA_VERSION})"
            )

        node_keys = set(graph.nodes.keys())
        # topic keys per node, from the model (slash-tolerant matching target)
        topic_keys_by_node: dict[str, set[str]] = {}
        for node_key, node_info in graph.nodes.items():
            topic_keys_by_node[node_key] = {
                sub.topic for sub in node_info.subscribers
            }

        defaults_raw = raw.get(KEY_DEFAULTS) or {}
        if not isinstance(defaults_raw, dict):
            raise ValueError(f"{source}: '{KEY_DEFAULTS}' must be a mapping")
        defaults = _validate_scope(defaults_raw, SCOPE_DEFAULTS, source, context="defaults")

        nodes_raw = raw.get(KEY_NODES) or {}
        if not isinstance(nodes_raw, dict):
            raise ValueError(f"{source}: '{KEY_NODES}' must be a mapping")
        nodes: dict[str, dict[str, object]] = {}
        nodes_spelled = _Spellings(KEY_NODES, source)
        for node_key_in, body in nodes_raw.items():
            node_key = _resolve_node_key(node_key_in, node_keys, source)
            nodes_spelled.claim(node_key_in, node_key, node_fqn(node_key), _NODE_HINT)
            if not isinstance(body, dict):
                raise ValueError(
                    f"{source}: nodes.'{node_key_in}' must be a mapping"
                )
            # `timers` nests one level deeper (per-timer, not per-node) than
            # the flat REGISTRY policy keys, so it is pulled out and validated
            # separately (maintainer decision P3).
            # A block key written without a value (YAML null) is absent, the
            # rule every block of the file follows (`defaults:`, `nodes:`, ...;
            # docs/features/execution-config.md "Other validation rules").
            body = dict(body)
            timers_raw = body.pop(KEY_TIMERS, None)
            tf2_raw = body.pop(KEY_TF2, None)
            resolved = _validate_scope(
                body, SCOPE_NODES, source, context=f"nodes.'{node_key_in}'"
            )
            if timers_raw is not None:
                resolved[KEY_TIMERS] = _validate_timers(
                    timers_raw, node_key, node_key_in, graph, source
                )
            if tf2_raw is not None:
                resolved[KEY_TF2] = _validate_tf2(tf2_raw, node_key, node_key_in, graph, source)
            nodes[node_key] = resolved

        subs_raw = raw.get(KEY_SUBSCRIPTIONS) or {}
        if not isinstance(subs_raw, dict):
            raise ValueError(f"{source}: '{KEY_SUBSCRIPTIONS}' must be a mapping")
        subscriptions: dict[str, dict[str, dict[str, object]]] = {}
        subs_spelled = _Spellings(KEY_SUBSCRIPTIONS, source)
        for node_key_in, topics_raw in subs_raw.items():
            node_key = _resolve_node_key(node_key_in, node_keys, source)
            subs_spelled.claim(node_key_in, node_key, node_fqn(node_key), _NODE_HINT)
            if not isinstance(topics_raw, dict):
                raise ValueError(
                    f"{source}: subscriptions.'{node_key_in}' must be a mapping"
                )
            known_topics = topic_keys_by_node.get(node_key, set())
            topics: dict[str, dict[str, object]] = {}
            topics_spelled = _Spellings(f"{KEY_SUBSCRIPTIONS}.'{node_key_in}'", source)
            for topic_key_in, body in topics_raw.items():
                topic_key = _resolve_topic_key(
                    topic_key_in, known_topics, node_key, node_key_in, source
                )
                topics_spelled.claim(
                    topic_key_in, topic_key, topic_runtime_name(topic_key, node_key),
                    _TOPIC_HINT,
                )
                if not isinstance(body, dict):
                    raise ValueError(
                        f"{source}: subscriptions.'{node_key_in}'.'{topic_key_in}' "
                        f"must be a mapping"
                    )
                topics[topic_key] = _validate_scope(
                    body,
                    SCOPE_SUBSCRIPTIONS,
                    source,
                    context=f"subscriptions.'{node_key_in}'.'{topic_key_in}'",
                )
            subscriptions[node_key] = topics

        ack_raw = raw.get(KEY_ACKNOWLEDGED_HAZARDS) or []
        if not isinstance(ack_raw, list) or not all(isinstance(c, str) for c in ack_raw):
            raise ValueError(
                f"{source}: '{KEY_ACKNOWLEDGED_HAZARDS}' must be a list of strings"
            )

        external_topics, external_anchors = _validate_external_topics(
            raw.get(KEY_EXTERNAL_TOPICS), graph, source
        )

        from colcon_lf.exec_config.connections import validate_connections

        connections = validate_connections(raw.get(KEY_CONNECTIONS), graph, source)
        publishers = _validate_publishers(raw.get(KEY_PUBLISHERS), graph, source)

        return cls(
            schema_version=schema_version,
            defaults=defaults,
            nodes=nodes,
            subscriptions=subscriptions,
            acknowledged_hazards=list(ack_raw),
            external_topics=external_topics,
            external_anchors=external_anchors,
            connections=connections,
            publishers=publishers,
            source=source,
        )


def package_exec_config_path(graph: "SystemGraph") -> Path | None:
    """The package-owned ``<pkg>/lf/<launch stem>.execution.yaml``.

    Checked in next to the launch file's package, so an example carries its
    own ``external_topics`` without a flag. The stem strips the launch suffix
    the same way ``colcon lf-analyze`` names its output directory. ``None``
    when the launch file has no enclosing package directory on this host.
    """
    root = getattr(graph.launch_graph, "root", None)
    if root is None or not root.file_path:
        return None
    launch = Path(root.file_path)
    stem = launch.name
    for suffix in (".launch.xml", ".launch.yaml", ".launch.py", "_launch.py", ".py"):
        stem = stem.replace(suffix, "")
    for parent in launch.parents:
        if (parent / "package.xml").is_file():
            return parent / "lf" / f"{stem}.execution.yaml"
    return None


def _validate_scope(
    body: dict, scope: str, source: str, *, context: str
) -> dict[str, object]:
    """Validate a ``{policy_key: value}`` mapping against ``REGISTRY`` for *scope*."""
    out: dict[str, object] = {}
    for key, (written, value) in _fold_input_names(body, source, context=context).items():
        spec = REGISTRY.get(key)
        if spec is None:
            raise ValueError(f"{source}: {context}: unknown policy key '{written}'")
        if scope not in spec.scopes:
            raise ValueError(
                f"{source}: {context}: policy key '{written}' is not valid at "
                f"'{scope}' scope"
            )
        try:
            out[key] = spec.parse(value)
        except ValueError as exc:
            msg = str(exc)
            name = INPUT_NAMES.get(key, key)
            if written != name:
                msg = msg.replace(f"'{name}'", f"'{written}'")
            raise ValueError(f"{source}: {context}: {msg}") from exc
    return out


def _fold_input_names(
    body: dict, source: str, *, context: str
) -> dict[str, tuple[str, object]]:
    """Map every written key to its registry key, as ``{key: (written, value)}``.

    ``maxwait_ms`` is the written name of the registry key
    ``take_timeout_ms``, which stays accepted as its alias. Both names in one
    mapping with different values is an error naming both keys and values; the
    same value under both is accepted with one warning. The fold keeps the
    position of the first name, so a file written with either name yields the
    same mapping, and it runs per scope and per file, so a value written under
    one name overrides one written under the other in the layering and the
    scope precedence alike.
    """
    out: dict[str, tuple[str, object]] = {}
    for written, value in body.items():
        key = INPUT_ALIASES.get(written, written)
        if key not in out:
            out[key] = (written, value)
            continue
        first, first_value = out[key]
        name, alias = INPUT_NAMES[key], key
        value_of = {first: first_value, written: value}
        if type(first_value) is not type(value) or first_value != value:
            raise ValueError(
                f"{source}: {context}: '{name}' and its alias '{alias}' are both set "
                f"with different values ({name}: {value_of[name]!r}, "
                f"{alias}: {value_of[alias]!r}); keep only '{name}'"
            )
        warnings.warn(
            f"{source}: {context}: '{alias}' is an alias of '{name}' and both are set "
            f"to {value!r}; keep only '{name}'",
            UserWarning, stacklevel=2,
        )
    return out


def _validate_timers(
    timers_raw: object,
    node_key: str,
    node_key_in: str,
    graph: "SystemGraph",
    source: str,
) -> dict[str, dict[str, str]]:
    """Validate ``nodes.<node>.timers`` -> ``{timer_name: {period?, offset?}}``.

    Timer names are matched against the node's real timer callback names
    (``TimerInfo.callback``, ``system.json``) — an unknown name is a hard
    ``ValueError`` (maintainer decision P3), same fail-closed style
    as an unknown node or topic key. Each entry sets ``period``, ``offset``
     or both, as validated LF time literals, and optionally
    ``offset_runtime``, a bool.
    """
    if not isinstance(timers_raw, dict):
        raise ValueError(f"{source}: nodes.'{node_key_in}'.{KEY_TIMERS} must be a mapping")

    known_timers = {t.callback for t in graph.nodes[node_key].timers}
    parsers = {KEY_TIMER_PERIOD: parse_timer_period, KEY_TIMER_OFFSET: parse_timer_offset,
               KEY_TIMER_OFFSET_RUNTIME: _parse_offset_runtime}
    out: dict[str, dict[str, str]] = {}
    for timer_name, body in timers_raw.items():
        where = f"{source}: nodes.'{node_key_in}'.{KEY_TIMERS}.'{timer_name}'"
        if timer_name not in known_timers:
            known = ", ".join(sorted(known_timers)) or "none"
            raise ValueError(
                f"{where}: no timer named '{timer_name}' on '{node_key_in}' "
                f"(known timers: {known})"
            )
        if not isinstance(body, dict):
            raise ValueError(f"{where} must be a mapping with '{KEY_TIMER_OFFSET}' "
                             f"and/or '{KEY_TIMER_PERIOD}'")
        unknown = sorted(set(body) - set(TIMER_KEYS), key=str)
        if unknown:
            raise ValueError(
                f"{where}: unknown key(s) {', '.join(repr(k) for k in unknown)} "
                f"(expected '{KEY_TIMER_OFFSET}', '{KEY_TIMER_OFFSET_RUNTIME}' "
                f"and/or '{KEY_TIMER_PERIOD}')"
            )
        if not body:
            raise ValueError(f"{where} needs '{KEY_TIMER_OFFSET}' or '{KEY_TIMER_PERIOD}'")
        fields: dict[str, str] = {}
        for name in TIMER_KEYS:
            if name not in body:
                continue
            try:
                fields[name] = parsers[name](body[name])
            except ValueError as exc:
                raise ValueError(f"{where}: {exc}") from exc
        out[timer_name] = fields
    return out


# ros2_lf_tf2_config fills a char[256] including the terminator.
TF2_ON_TIMEOUT_MAX = 255


def _validate_tf2(
    raw: object, node_key: str, node_key_in: str, graph: "SystemGraph", source: str
) -> dict[str, object]:
    """Validate ``nodes.<node>.tf2`` -> ``{wait_bound_ns?, on_timeout?}``.

    The node must construct at least one admitted ``rcllf_tf2::AcquisitionLookup``
    (the analyzer's tf2 scan), else the keys would configure nothing.
    ``wait_bound`` is an LF time literal > 0; ``on_timeout`` is ``reject``,
    ``failure`` or ``fallback:<name>``.
    """
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    where = f"{source}: nodes.'{node_key_in}'.{KEY_TF2}"
    if not isinstance(raw, dict) or not raw:
        raise ValueError(f"{where} must be a mapping with '{KEY_TF2_WAIT_BOUND}' "
                         f"and/or '{KEY_TF2_ON_TIMEOUT}'")
    unknown = sorted(set(raw) - set(TF2_KEYS), key=str)
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {', '.join(repr(k) for k in unknown)} "
                         f"(expected '{KEY_TF2_WAIT_BOUND}' and/or '{KEY_TF2_ON_TIMEOUT}')")
    tf2 = getattr(graph.nodes[node_key], "tf2", None)
    if tf2 is None or not getattr(tf2, "adapters", None):
        raise ValueError(f"{where}: '{node_key_in}' constructs no rcllf_tf2::AcquisitionLookup "
                         "the analyzer admitted, so there is nothing to configure")
    out: dict[str, object] = {}
    if KEY_TF2_WAIT_BOUND in raw:
        try:
            ns = parse_lf_time_ns(raw[KEY_TF2_WAIT_BOUND])
        except ValueError as exc:
            raise ValueError(f"{where}.{KEY_TF2_WAIT_BOUND}: {exc}") from exc
        if not ns:
            raise ValueError(f"{where}.{KEY_TF2_WAIT_BOUND} must be > 0, got "
                             f"{raw[KEY_TF2_WAIT_BOUND]!r}")
        out["wait_bound_ns"] = ns
    if KEY_TF2_ON_TIMEOUT in raw:
        val = raw[KEY_TF2_ON_TIMEOUT]
        ok = isinstance(val, str) and (
            val in ("reject", "failure")
            or (val.startswith("fallback:") and len(val) > len("fallback:")
                and all("!" <= c <= "~" and c not in '"\\' for c in val)))
        if not ok:
            raise ValueError(f"{where}.{KEY_TF2_ON_TIMEOUT} must be 'reject', 'failure' or "
                             f"'fallback:<name>' (printable ASCII, no quote, backslash or "
                             f"space), got {val!r}")
        # The runtime hook copies the value into a 256-byte buffer
        # (ros2_lf_tf2_config); a longer one would throw at node construction.
        if len(val) > TF2_ON_TIMEOUT_MAX:
            raise ValueError(f"{where}.{KEY_TF2_ON_TIMEOUT} is {len(val)} characters, "
                             f"longer than the runtime limit of {TF2_ON_TIMEOUT_MAX}")
        out["on_timeout"] = val
    return out


def _parse_offset_runtime(value: object) -> bool:
    """``offset_runtime`` is a plain bool."""
    if not isinstance(value, bool):
        raise ValueError(f"'{KEY_TIMER_OFFSET_RUNTIME}' must be true or false, got {value!r}")
    return value


def _validate_external_topics(
    raw: object, graph: "SystemGraph", source: str
) -> tuple[list[str], dict[str, dict]]:
    """Resolve ``external_topics`` against the system's topic connections.

    Each entry is a topic name or a mapping with a ``topic`` key
    (``anchor: stamp``). The topic must be one some node of the system
    subscribes to and no node of the system publishes. Names
    are slash-tolerant and normalized to the resolved ``TopicConnection.topic``
    form. Returns the topics and, per topic written as a mapping, its anchor.
    """
    if raw is None:
        return [], {}
    if not isinstance(raw, list) or not all(isinstance(t, (str, dict)) for t in raw):
        raise ValueError(
            f"{source}: '{KEY_EXTERNAL_TOPICS}' must be a list of topic names or "
            f"mappings with a '{KEY_EXT_TOPIC}' key"
        )
    by_topic = {tc.topic: tc for tc in graph.communication.topics}
    out: list[str] = []
    anchors: dict[str, dict] = {}
    for entry in raw:
        name = entry if isinstance(entry, str) else entry.get(KEY_EXT_TOPIC)
        if not isinstance(name, str) or not name:
            raise ValueError(
                f"{source}: {KEY_EXTERNAL_TOPICS}: a mapping entry needs a "
                f"'{KEY_EXT_TOPIC}' string, got {entry!r}"
            )
        candidates = [name, "/" + name]
        if name.startswith("/"):
            candidates.append(name[1:])
        tc = next((by_topic[c] for c in candidates if c in by_topic), None)
        if tc is None or not tc.subscribers:
            raise ValueError(
                f"{source}: {KEY_EXTERNAL_TOPICS}: unknown topic '{name}' "
                f"(no node in system.json subscribes to it)"
            )
        if tc.publishers:
            raise ValueError(
                f"{source}: {KEY_EXTERNAL_TOPICS}: topic '{name}' is published "
                f"inside the system by {', '.join(sorted(tc.publishers))}; "
                f"only topics with no in-system publisher can be external"
            )
        if tc.topic not in out:
            out.append(tc.topic)
        if isinstance(entry, dict):
            if tc.topic in anchors:
                raise ValueError(
                    f"{source}: {KEY_EXTERNAL_TOPICS}: topic '{name}' has two "
                    f"mapping entries; write its anchor once"
                )
            anchors[tc.topic] = _validate_external_anchor(entry, name, source)
    return out, anchors


def _validate_external_anchor(entry: dict, name: str, source: str) -> dict:
    """One mapping entry of ``external_topics``.

    ``anchor: arrival`` (the default of a plain string entry) takes no other
    key. ``anchor: stamp`` needs ``delay_ns`` (T, the arrival deadline after
    ``header.stamp``) and ``basis`` (the written justification of the
    constants); ``ingress_reserve_ns`` defaults to 0 and ``grid_ns`` to 1,
    both the identity and both printed by ``colcon lf-configure``.
    """
    ctx = f"{source}: {KEY_EXTERNAL_TOPICS}: topic '{name}'"
    unknown = set(entry) - EXT_ENTRY_KEYS
    if unknown:
        raise ValueError(
            f"{ctx}: unknown key(s) {sorted(unknown)}; allowed: "
            f"{sorted(EXT_ENTRY_KEYS)}"
        )
    anchor = entry.get(KEY_EXT_ANCHOR, EXT_ANCHOR_ARRIVAL)
    if anchor not in EXT_ANCHORS:
        raise ValueError(
            f"{ctx}: '{KEY_EXT_ANCHOR}' must be one of {list(EXT_ANCHORS)}, got {anchor!r}"
        )
    if anchor == EXT_ANCHOR_ARRIVAL:
        extra = set(entry) - {KEY_EXT_TOPIC, KEY_EXT_ANCHOR}
        if extra:
            raise ValueError(
                f"{ctx}: {sorted(extra)} apply to '{KEY_EXT_ANCHOR}: {EXT_ANCHOR_STAMP}' only"
            )
        return {KEY_EXT_ANCHOR: EXT_ANCHOR_ARRIVAL}
    if KEY_EXT_DELAY_NS not in entry:
        raise ValueError(
            f"{ctx}: '{KEY_EXT_ANCHOR}: {EXT_ANCHOR_STAMP}' needs '{KEY_EXT_DELAY_NS}' "
            f"(T, the arrival deadline in ns after header.stamp); there is no default"
        )
    basis = entry.get(KEY_EXT_BASIS)
    if not isinstance(basis, str) or not basis.strip():
        raise ValueError(
            f"{ctx}: '{KEY_EXT_ANCHOR}: {EXT_ANCHOR_STAMP}' needs a non-empty "
            f"'{KEY_EXT_BASIS}' string stating where {KEY_EXT_DELAY_NS}, "
            f"{KEY_EXT_RESERVE_NS} and {KEY_EXT_GRID_NS} come from (a measurement, "
            f"or a placeholder named as such)"
        )
    out = {KEY_EXT_ANCHOR: EXT_ANCHOR_STAMP, KEY_EXT_BASIS: basis.strip()}
    for key, default, least in (
        (KEY_EXT_DELAY_NS, None, 0), (KEY_EXT_RESERVE_NS, 0, 0), (KEY_EXT_GRID_NS, 1, 1),
    ):
        value = entry.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or value < least:
            raise ValueError(
                f"{ctx}: '{key}' must be an integer >= {least} (nanoseconds), got {value!r}"
            )
        if value > EXT_ANCHOR_MAX_NS:
            raise ValueError(
                f"{ctx}: '{key}' must be an integer <= 2**60 (nanoseconds, about 36 years, "
                f"so the binding's tag arithmetic fits 64 bits), got {value!r}"
            )
        out[key] = value
    return out


def _validate_publishers(
    raw: object, graph: "SystemGraph", source: str
) -> dict[str, dict[str, dict]]:
    """``publishers: {<node>: {<topic>: {output_deadline_ns, basis}}}``.

    The node must be in system.json and the topic one it publishes (the
    analysis' publisher list, slash-tolerant). ``output_deadline_ns`` is an
    integer >= 0 with no default, and ``basis`` a non-empty string that
    states where the bound comes from (house rule on constants).
    """
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"{source}: '{KEY_PUBLISHERS}' must be a mapping of node to topics")
    node_keys = set(graph.nodes.keys())
    out: dict[str, dict[str, dict]] = {}
    # 'controller' and '/controller' name one node; the
    # second entry used to replace the first and drop its bounds.
    nodes_spelled = _Spellings(KEY_PUBLISHERS, source)
    for node_key_in, topics_raw in raw.items():
        node_key = _resolve_node_key(node_key_in, node_keys, source)
        nodes_spelled.claim(node_key_in, node_key, node_fqn(node_key), _NODE_HINT)
        if not isinstance(topics_raw, dict) or not topics_raw:
            raise ValueError(
                f"{source}: {KEY_PUBLISHERS}.'{node_key_in}' must be a non-empty mapping "
                f"of publisher topic to {{{KEY_PUB_OUTPUT_DEADLINE_NS}, {KEY_PUB_BASIS}}}"
            )
        known = {p.topic for p in graph.nodes[node_key].publishers}
        topics: dict[str, dict] = {}
        # 'actuation' and '/actuation' can both be publisher
        # topics in system.json; for a node in the root namespace rclcpp
        # resolves both to '/actuation' and output_deadline::arm keys by (rid,
        # resolved topic), so the later bound silently replaced the earlier
        # one. For a node in '/robot' they are two topics.
        topics_spelled = _Spellings(f"{KEY_PUBLISHERS}.'{node_key_in}'", source)
        for topic_in, body in topics_raw.items():
            topic = _resolve_publisher_topic(topic_in, known, node_key, node_key_in, source)
            ctx = f"{source}: {KEY_PUBLISHERS}.'{node_key_in}'.'{topic_in}'"
            topics_spelled.claim(
                topic_in, topic, topic_runtime_name(topic, node_key),
                "write the topic once with one bound",
            )
            if not isinstance(body, dict):
                raise ValueError(f"{ctx}: must be a mapping")
            unknown = set(body) - PUB_ENTRY_KEYS
            if unknown:
                raise ValueError(
                    f"{ctx}: unknown key(s) {sorted(unknown)}; allowed: {sorted(PUB_ENTRY_KEYS)}"
                )
            if KEY_PUB_OUTPUT_DEADLINE_NS not in body:
                raise ValueError(
                    f"{ctx}: needs '{KEY_PUB_OUTPUT_DEADLINE_NS}' (the actuation lateness "
                    f"bound in ns); there is no default"
                )
            bound = body[KEY_PUB_OUTPUT_DEADLINE_NS]
            if isinstance(bound, bool) or not isinstance(bound, int) or bound < 0:
                raise ValueError(
                    f"{ctx}: '{KEY_PUB_OUTPUT_DEADLINE_NS}' must be an integer >= 0 "
                    f"(nanoseconds), got {bound!r}"
                )
            if bound > PUB_DEADLINE_MAX_NS:
                raise ValueError(
                    f"{ctx}: '{KEY_PUB_OUTPUT_DEADLINE_NS}' must be an integer <= 2**60 "
                    f"(nanoseconds, about 36 years), got {bound!r}"
                )
            basis = body.get(KEY_PUB_BASIS)
            if not isinstance(basis, str) or not basis.strip():
                raise ValueError(
                    f"{ctx}: needs a non-empty '{KEY_PUB_BASIS}' string stating where "
                    f"{KEY_PUB_OUTPUT_DEADLINE_NS} comes from (a measurement, or a "
                    f"placeholder named as such)"
                )
            topics[topic] = {KEY_PUB_OUTPUT_DEADLINE_NS: bound, KEY_PUB_BASIS: basis.strip()}
        out[node_key] = topics
    return out


_NODE_HINT = "write the node once and list all its entries under it"
_TOPIC_HINT = "write the topic once"


def node_fqn(node_key: str) -> str:
    """The fully qualified name of a node key (``ns/n``, ``/ns/n`` -> ``/ns/n``).

    Node keys in system.json are the node's full name with or without the
    leading slash (``graph/hierarchical.py::_node_key``), so a user key names a
    node by the same rule: rclcpp has no relative node names.
    """
    return "/" + node_key.lstrip("/")


def topic_runtime_name(topic: str, node_key: str) -> str:
    """The name rclcpp resolves *topic* to for the node *node_key*.

    ``rcl_expand_topic_name``: an absolute name stays as written, ``~`` expands
    to the node's full name, and a relative name is joined to the node's
    namespace. Launch remappings are not applied: config keys name the topic as
    the node's code spells it, which is how system.json records it.
    """
    fqn = node_fqn(node_key)
    namespace = fqn.rsplit("/", 1)[0]  # '' for the root namespace
    if topic.startswith("/"):
        return topic
    if topic == "~":
        return fqn
    if topic.startswith("~/"):
        return fqn + topic[1:]
    return f"{namespace}/{topic}"


class _Spellings:
    """Refuse two keys of one section that resolve to one runtime name.

    ``claim`` records the key a runtime name was first written as; a second
    key with the same runtime name raises a ``ValueError`` naming both, since
    the section is keyed by model key and the later entry would replace the
    earlier one without a word.
    """

    def __init__(self, section: str, source: str) -> None:
        self._section = section
        self._source = source
        self._first: dict[str, object] = {}

    def claim(self, key_in: object, model_key: str, runtime: str, hint: str) -> None:
        first = self._first.get(runtime)
        if first is not None:
            raise ValueError(
                f"{self._source}: {self._section}.'{key_in}' and "
                f"{self._section}.'{first}' both resolve to '{runtime}' at runtime "
                f"(model key '{model_key}'); {hint}"
            )
        self._first[runtime] = key_in


def _match_topic(topic_in: object, known: set[str], node_key: str) -> str | None:
    """The model topic of *node_key* that *topic_in* names at runtime, if any.

    The exact spelling wins; otherwise the one model topic whose runtime name
    equals that of *topic_in* (``topic_runtime_name``). For a node in the root
    namespace this is the old slash tolerance; for a namespaced node an
    absolute key no longer names a relative model topic, or vice versa.
    """
    if not isinstance(topic_in, str):
        return None
    if topic_in in known:
        return topic_in
    runtime = topic_runtime_name(topic_in, node_key)
    hits = sorted(t for t in known if topic_runtime_name(t, node_key) == runtime)
    return hits[0] if hits else None


def _runtime_names(known: set[str], node_key: str) -> str:
    return ", ".join(f"'{t}' -> '{topic_runtime_name(t, node_key)}'" for t in sorted(known))


def _resolve_publisher_topic(
    topic_in: object, known: set[str], node_key: str, node_key_in: str, source: str
) -> str:
    """Resolve a ``publishers:`` topic key to the node's publisher topic by runtime name."""
    topic = _match_topic(topic_in, known, node_key)
    if topic is not None:
        return topic
    resolved = (
        f"it resolves to '{topic_runtime_name(topic_in, node_key)}'; "
        if isinstance(topic_in, str) else ""
    )
    raise ValueError(
        f"{source}: {KEY_PUBLISHERS}: unknown topic {topic_in!r} on node '{node_key_in}' "
        f"({resolved}not a publisher of that node in system.json, which publishes "
        f"{_runtime_names(known, node_key) or 'nothing'})"
    )


def _resolve_node_key(node_key_in: str, node_keys: set[str], source: str) -> str:
    """Resolve a user-written node key to the model key with the same full name."""
    if node_key_in in node_keys:
        return node_key_in
    if isinstance(node_key_in, str):
        fqn = node_fqn(node_key_in)
        hits = sorted(k for k in node_keys if node_fqn(k) == fqn)
        if hits:
            return hits[0]
    raise ValueError(f"{source}: unknown node '{node_key_in}' (not in system.json)")


def _resolve_topic_key(
    topic_key_in: str, known_topics: set[str], node_key: str, node_key_in: str, source: str
) -> str:
    """Resolve a ``subscriptions:`` topic key to the node's subscription by runtime name."""
    topic = _match_topic(topic_key_in, known_topics, node_key)
    if topic is not None:
        return topic
    resolved = (
        f"it resolves to '{topic_runtime_name(topic_key_in, node_key)}'; "
        if isinstance(topic_key_in, str) else ""
    )
    raise ValueError(
        f"{source}: unknown topic '{topic_key_in}' on node '{node_key_in}' "
        f"({resolved}not a subscription in system.json, which subscribes to "
        f"{_runtime_names(known_topics, node_key) or 'nothing'})"
    )

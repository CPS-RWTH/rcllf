# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Resolve ``ExecConfig`` x ``SystemGraph`` -> the frozen A<->C contract table.

The frozen contract:

1. ``exec_config.json`` shape: ``{"schema_version": 1, "defaults": {...},
   "subscriptions": [{"node_class": ..., "topic": ..., <policy keys>}, ...]}``.
   Keyed by ``(node_class, topic-as-in-model)`` — the key the C++ runtime can
   actually resolve.  The field keeps its frozen name ``node_class`` but carries
   the **binding key** (``generation/binding_key.py``): equal to the C++ class
   name only when that class has a single instance, otherwise
   ``Class__<node-key>``.  RIDs, the reactor definition, the
   ``ros2_lf_bind_node`` string and this field all derive from that one helper
   in one generator run.
2. Precedence is PER-KEY: for each policy key independently,
   built-in default < ``defaults:`` < ``nodes:`` < ``subscriptions:``.
   (Env-var override is a C++/slice-C runtime concern, not resolved here.)
3. FQN -> binding-key fold is 1:1 with node instances, so per-instance policy
   is representable: two instances of one class with different ``take_mode``
   resolve to two rows.  Two node keys folding to one binding key is therefore
   impossible by construction; the check is retained as a synthesis-bug
   assertion (design note ``10_per_instance_rids_design.md`` sections 2 and 5).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.policies import (
    CONTROL_FAILURE_STOP,
    JOIN_DEFERRED,
    KEY_CONTROL_FAILURE_POLICY,
    KEY_JOIN,
    KEY_MAX_PUBLISHES_PER_INVOCATION,
    KEY_MAX_STAMP_LAG,
    KEY_MAX_WORKER_LAG_MS,
    KEY_MIN_INPUT_PERIOD_MS,
    KEY_READER_DEPTH,
    KEY_REQUIRE_ADMITTED,
    KEY_WORKERS,
    KEY_STAMP_PROVENANCE,
    KEY_TAKE_TIMEOUT_MS,
    REGISTRY,
    RUNTIME_REGISTRY,
    input_name,
)
from colcon_lf.exec_config.yaml_schema import SCHEMA_VERSION

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph


@dataclass
class ResolvedSubscription:
    """One resolved (binding key, topic) row — the C++-facing lookup key.

    ``node_class`` keeps the frozen contract field name but holds the binding
    key (equal to the C++ class name only for a single-instance class).
    """

    node_class: str
    topic: str
    values: dict[str, object]
    # Node names (FQN) this row was folded from — for diagnostics only.
    node_keys: tuple[str, ...] = field(default_factory=tuple)
    # Policy key -> "<layer> (<scope>)" that set it, or "built-in".
    origins: dict[str, str] = field(default_factory=dict)


@dataclass
class ResolvedExecConfig:
    """Fully resolved table: registry defaults < defaults: < nodes: < subscriptions:."""

    defaults: dict[str, object]
    subscriptions: list[ResolvedSubscription]
    warnings: list[str] = field(default_factory=list)
    # Policy key of ``defaults`` -> layer that set it, or "built-in".
    default_origins: dict[str, str] = field(default_factory=dict)

    def to_json_dict(self) -> dict:
        """Emit exactly the frozen ``exec_config.json`` shape (section 2.4)."""
        return {
            "schema_version": SCHEMA_VERSION,
            "defaults": dict(self.defaults),
            "subscriptions": [
                {"node_class": row.node_class, "topic": row.topic, **row.values}
                for row in self.subscriptions
            ],
        }


def resolve(config: ExecConfig, graph: "SystemGraph") -> ResolvedExecConfig:
    """Resolve *config* against *graph* into the frozen (node_class, topic) table."""
    from colcon_lf.generation.binding_key import build_binding_keys

    binding_keys = build_binding_keys(graph.nodes)
    # Only runtime keys enter the resolved table: generation-time keys (e.g.
    # after_delay) are read straight off config.defaults by the generator and
    # must not change exec_config.json's frozen shape.
    registry_defaults = {key: spec.default for key, spec in RUNTIME_REGISTRY.items()}
    generation_keys = {k for k, s in REGISTRY.items() if not s.runtime}
    user_defaults = {k: v for k, v in config.defaults.items() if k not in generation_keys}
    defaults = {**registry_defaults, **user_defaults}
    default_origins = {
        key: (
            f"{config.origin(('defaults', key))} (defaults)" if key in user_defaults
            else "built-in"
        )
        for key in defaults
    }

    # First pass: per (node_key, topic) merged values (defaults < nodes < subscriptions).
    per_instance_rows: list[tuple[str, str, dict[str, object]]] = []
    row_origins: dict[tuple[str, str], dict[str, str]] = {}
    for node_key, node_info in graph.nodes.items():
        node_overrides = {
            k: v for k, v in config.nodes.get(node_key, {}).items() if k in RUNTIME_REGISTRY
        }
        sub_overrides_by_topic = config.subscriptions.get(node_key, {})
        for sub in node_info.subscribers:
            topic_overrides = {
                k: v for k, v in sub_overrides_by_topic.get(sub.topic, {}).items()
                if k in RUNTIME_REGISTRY
            }
            merged = {**defaults, **node_overrides, **topic_overrides}
            origins = dict(default_origins)
            for key in node_overrides:
                origins[key] = f"{config.origin(('nodes', node_key, key))} (nodes)"
            for key in topic_overrides:
                origins[key] = (
                    f"{config.origin(('subscriptions', node_key, sub.topic, key))} "
                    f"(subscriptions)"
                )
            per_instance_rows.append((node_key, sub.topic, merged))
            row_origins.setdefault((node_key, sub.topic), origins)

    # Second pass: fold node_key -> binding key. The mapping is 1:1 with node
    # instances, so a single binding key can only ever be claimed by one node
    # key; two node keys landing on the same key means binding-key synthesis is
    # broken. The check below is retained purely as that tripwire.
    # Key: (binding_key, topic) -> {values, contributing node_keys}
    folded: dict[tuple[str, str], dict[str, object]] = {}
    folded_node_keys: dict[tuple[str, str], list[str]] = {}
    for node_key, topic, values in per_instance_rows:
        binding_key = binding_keys[node_key]
        fold_key = (binding_key, topic)
        if fold_key not in folded:
            folded[fold_key] = values
            folded_node_keys[fold_key] = [node_key]
            continue
        if node_key in folded_node_keys[fold_key]:
            # Same node instance, same topic subscribed more than once — the
            # rows are identical by construction; keep the first.
            continue
        other_nodes = ", ".join(folded_node_keys[fold_key])
        raise ValueError(
            f"exec-config resolve: binding key '{binding_key}' is claimed by more "
            f"than one node instance for topic '{topic}': node(s) [{other_nodes}] "
            f"and node '{node_key}'. Binding keys are 1:1 with node instances, so "
            f"this is a binding-key synthesis bug in the generator "
            f"(generation/binding_key.py), not a configuration error."
        )

    warnings: list[str] = []
    external = {t.lstrip("/") for t in config.external_topics}
    rows: list[ResolvedSubscription] = []
    for (binding_key, topic), values in sorted(folded.items()):
        node_keys = tuple(folded_node_keys[(binding_key, topic)])
        origins = row_origins[(node_keys[0], topic)]
        warnings.extend(_inert_timeout_warning(values, binding_key, topic, external))
        rows.append(
            ResolvedSubscription(
                node_class=binding_key, topic=topic, values=values, node_keys=node_keys,
                origins={k: v for k, v in origins.items() if k in values},
            )
        )

    return ResolvedExecConfig(
        defaults=defaults, subscriptions=rows, warnings=warnings,
        default_origins=default_origins,
    )


@dataclass
class CorrBoundsConfig:
    """Exec-config inputs of the correspondence bounds (W3 design 3.3 to 3.6).

    Keyed by launch-level node key and the node-local subscription topic, the
    same keys ``CorrEdge.sub_node`` / ``CorrEdge.sub_topic`` carry. A missing
    entry means "undeclared": ``burst`` falls back to the publish count the
    analyzer proves and refuses admission only when there is none,
    ``min_input_period_ms`` refuses admission when it is needed,
    ``take_timeout_ms`` falls back to the
    registry default and ``max_worker_lag_ms`` None means twice the largest
    timeout of an admitted edge.
    """

    burst: dict[str, int] = field(default_factory=dict)
    take_timeout_ms: dict[tuple[str, str], int] = field(default_factory=dict)
    min_input_period_ms: dict[tuple[str, str], float] = field(default_factory=dict)
    max_worker_lag_ms: int | None = None
    reader_depth: dict[tuple[str, str], int] = field(default_factory=dict)
    # Subscriptions whose admitted edge joins deferred.
    deferred_join: set[tuple[str, str]] = field(default_factory=set)
    # Declared stamp provenance of a subscription, BNA inputs only.
    stamp_provenance: dict[tuple[str, str], str] = field(default_factory=dict)
    # Declared max_stamp_lag (ns) of a subscription, BNA inputs only.
    max_stamp_lag_ns: dict[tuple[str, str], int] = field(default_factory=dict)

    # Any refused topic edge is a generation error.
    require_admitted: bool = False
    # Control_failure_policy is 'stop'.
    control_failure_stop: bool = False
    # The LF 'workers' target property, None = reactor-c default.
    workers: int | None = None

    def depth(self, node_key: str, topic: str) -> int:
        """Resolved ``reader_depth`` of one subscription."""
        default = int(REGISTRY[KEY_READER_DEPTH].default)
        return self.reader_depth.get((node_key, topic), default)

    def deferred(self, node_key: str, topic: str) -> bool:
        """Whether one subscription's edge joins deferred."""
        return (node_key, topic) in self.deferred_join

    def timeout_ms(self, node_key: str, topic: str) -> int:
        """Resolved ``take_timeout_ms`` of one subscription."""
        default = int(REGISTRY[KEY_TAKE_TIMEOUT_MS].default)
        return self.take_timeout_ms.get((node_key, topic), default)


def resolve_corr_bounds(config: ExecConfig, graph: "SystemGraph") -> CorrBoundsConfig:
    """Per-key precedence (defaults < nodes < subscriptions) for the bound inputs."""
    out = CorrBoundsConfig(
        max_worker_lag_ms=config.defaults.get(KEY_MAX_WORKER_LAG_MS),
        require_admitted=bool(config.defaults.get(KEY_REQUIRE_ADMITTED, False)),
        control_failure_stop=(
            config.defaults.get(KEY_CONTROL_FAILURE_POLICY) == CONTROL_FAILURE_STOP),
        workers=config.defaults.get(KEY_WORKERS),
    )
    for node_key, node_info in graph.nodes.items():
        node_scope = {**config.defaults, **config.nodes.get(node_key, {})}
        if node_scope.get(KEY_MAX_PUBLISHES_PER_INVOCATION) is not None:
            out.burst[node_key] = int(node_scope[KEY_MAX_PUBLISHES_PER_INVOCATION])
        for sub in node_info.subscribers:
            merged = {
                **node_scope, **config.subscriptions.get(node_key, {}).get(sub.topic, {}),
            }
            if merged.get(KEY_TAKE_TIMEOUT_MS) is not None:
                out.take_timeout_ms[(node_key, sub.topic)] = int(merged[KEY_TAKE_TIMEOUT_MS])
            if merged.get(KEY_MIN_INPUT_PERIOD_MS) is not None:
                out.min_input_period_ms[(node_key, sub.topic)] = merged[KEY_MIN_INPUT_PERIOD_MS]
            if merged.get(KEY_READER_DEPTH) is not None:
                out.reader_depth[(node_key, sub.topic)] = int(merged[KEY_READER_DEPTH])
            if merged.get(KEY_JOIN) == JOIN_DEFERRED:
                out.deferred_join.add((node_key, sub.topic))
            if merged.get(KEY_STAMP_PROVENANCE) is not None:
                out.stamp_provenance[(node_key, sub.topic)] = merged[KEY_STAMP_PROVENANCE]
            if merged.get(KEY_MAX_STAMP_LAG) is not None:
                from colcon_lf.generation.lf_time import parse_lf_time_ns

                out.max_stamp_lag_ns[(node_key, sub.topic)] = int(
                    parse_lf_time_ns(merged[KEY_MAX_STAMP_LAG]) or 0)
    return out


def _inert_timeout_warning(
    values: dict[str, object], binding_key: str, topic: str, external: set[str]
) -> list[str]:
    """The warning when a set (non-default) maxwait_ms cannot act.

    The timeout bounds the wait of a take in ``skip`` and ``block`` mode alike:
    a take polls up to it for a sample its trigger counted, and an admitted edge's join waits up to it. The one
    subscription the generator knows never waits is an ``external_topics``
    one: its message enters as a physical action the ingress thread has
    already taken when the reaction runs. A take whose write
    count is unknown at run time (``RCLLF_COUNT_PUBLISHES=0``, a publisher
    outside any reaction) skips at once too, but that is a run-time property
    the generator cannot see per edge.
    """
    if topic.lstrip("/") not in external:
        return []
    default_timeout = REGISTRY[KEY_TAKE_TIMEOUT_MS].default
    if values.get(KEY_TAKE_TIMEOUT_MS, default_timeout) == default_timeout:
        return []
    return [
        f"{binding_key}.{topic}: '{input_name(KEY_TAKE_TIMEOUT_MS)}' is set for an external topic; "
        f"an external_topics message is already taken when the reaction runs, so the "
        f"take never waits and the timeout is inert. A take on an edge inside the "
        f"system waits up to the timeout for a counted sample in skip and block mode alike."
    ]


def print_warnings(warnings: list[str], *, file=None) -> None:
    """Print resolve-time warnings in the style used elsewhere in the CLI."""
    # Look sys.stderr up per call: a default bound at import time keeps
    # whatever stream was current then (a test's capture buffer, for one).
    file = file if file is not None else sys.stderr
    for w in warnings:
        print(f"  Warning: {w}", file=file)

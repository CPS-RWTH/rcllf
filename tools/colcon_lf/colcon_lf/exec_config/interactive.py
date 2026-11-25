# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``--interactive`` prompt loop for ``colcon lf-configure``.

A stdlib-only prompt
loop over defaults -> nodes -> subscriptions, driven by ``REGISTRY`` (its
``doc`` lines as help text). Reads and writes ONLY the sparse
``execution.yaml`` — no comment round-trip problem, because the reference
file is a separate, write-only artifact (``scaffold.py``).

Deliberately minimal: sequenced last in the slice, and the design explicitly
permits dropping this to backlog without violating exit criteria if it
proves fragile.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable

import yaml

from colcon_lf.exec_config.policies import REGISTRY, input_name

if TYPE_CHECKING:
    from colcon_lf.graph.models import SystemGraph

# Sentinel the user types to leave a key unset (skip / keep inherited).
SKIP_TOKEN = ""
# Sentinel the user types to end the whole loop early.
QUIT_TOKEN = "q"


def run_interactive(
    sparse_path: Path,
    graph: "SystemGraph",
    *,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[..., None] = print,
) -> None:
    """Run the interactive prompt loop, writing only *sparse_path*.

    ``input_fn``/``print_fn`` are injectable for scripted-stdin testing.
    """
    raw = yaml.safe_load(sparse_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raw = {}
    raw.setdefault("schema_version", 1)
    defaults = raw.setdefault("defaults", {}) or {}
    nodes = raw.setdefault("nodes", {}) or {}
    subscriptions = raw.setdefault("subscriptions", {}) or {}
    raw.setdefault("acknowledged_hazards", [])
    raw["defaults"] = defaults
    raw["nodes"] = nodes
    raw["subscriptions"] = subscriptions

    print_fn(
        "Interactive execution-policy configurator. "
        f"Press enter to leave a key unset (inherit), '{QUIT_TOKEN}' to stop early."
    )

    quit_requested = _prompt_scope(
        "defaults", defaults, input_fn=input_fn, print_fn=print_fn
    )

    if not quit_requested:
        print_fn("\n-- nodes --")
        for node_key in sorted(graph.nodes.keys()):
            body = nodes.get(node_key, {}) or {}
            quit_requested = _prompt_scope(
                f"nodes.{node_key}", body, input_fn=input_fn, print_fn=print_fn
            )
            if body:
                nodes[node_key] = body
            if quit_requested:
                break

    if not quit_requested:
        print_fn("\n-- subscriptions --")
        for node_key in sorted(graph.nodes.keys()):
            node_info = graph.nodes[node_key]
            if not node_info.subscribers:
                continue
            node_subs = subscriptions.get(node_key, {}) or {}
            for sub in node_info.subscribers:
                body = node_subs.get(sub.topic, {}) or {}
                quit_requested = _prompt_scope(
                    f"subscriptions.{node_key}.{sub.topic}",
                    body,
                    input_fn=input_fn,
                    print_fn=print_fn,
                )
                if body:
                    node_subs[sub.topic] = body
                if quit_requested:
                    break
            if node_subs:
                subscriptions[node_key] = node_subs
            if quit_requested:
                break

    sparse_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    print_fn(f"Wrote {sparse_path}")


def _prompt_scope(
    label: str,
    body: dict,
    *,
    input_fn: Callable[[str], str],
    print_fn: Callable[..., None],
) -> bool:
    """Prompt for every registry key at one scope. Returns True if the user quit."""
    print_fn(f"[{label}]")
    for spec in REGISTRY.values():
        # Prompt and write the execution.yaml name (maxwait_ms);
        # a value the file holds under the alias (take_timeout_ms) is current.
        name = input_name(spec.key)
        current = body.get(name, body.get(spec.key, "<inherit>"))
        choices_str = f" ({'/'.join(spec.choices)})" if spec.choices else ""
        prompt = f"  {name}{choices_str} [{current}] — {spec.doc}\n  > "
        raw_value = input_fn(prompt)
        if raw_value == QUIT_TOKEN:
            return True
        if raw_value == SKIP_TOKEN:
            continue
        try:
            parsed = spec.parse(_coerce(raw_value, spec))
        except ValueError as exc:
            print_fn(f"    invalid value, skipping: {exc}")
            continue
        if name != spec.key:
            body.pop(spec.key, None)
        body[name] = parsed
    return False


def _coerce(raw_value: str, spec) -> object:
    """Best-effort str -> int coercion before handing to spec.parse."""
    if spec.choices is not None:
        return raw_value
    try:
        return int(raw_value)
    except ValueError:
        return raw_value

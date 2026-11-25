# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-connection after-delays with a stated reason.

``execution.yaml`` section::

    connections:
      /sentry/image:                 # topic, service or action name
        after_delay: "400 usec"
        reason: measured             # measured | budgeted | default
        source: "run <dir>: camera_node callback max 390 us"
        links:                       # optional, one sender -> receiver pair
          "detector_node -> diagnostics_sink_node":
            after_delay: "20 msec"
            reason: budgeted
            source: "diagnostics are not on the control path"

A pair is publisher -> subscriber for a topic and client -> server for a
service or an action (both directions of the request take the value; an action
uses it on its goal channels, the others keep ``ACTION_ASYNC_AFTER_DELAY_NS``).
The generator takes a pair's value before the name's value, and either before
``system.json``'s ``after_delay_ns``, the ``--after-delay`` flag and
``defaults.after_delay``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from colcon_lf.exec_config.policies import KEY_AFTER_DELAY, _parse_after_delay

if TYPE_CHECKING:
    from colcon_lf.exec_config.model import ExecConfig
    from colcon_lf.graph.models import SystemGraph

KEY_REASON = "reason"
KEY_SOURCE = "source"
KEY_LINKS = "links"
REASON_MEASURED = "measured"
REASON_BUDGETED = "budgeted"
REASON_DEFAULT = "default"
REASONS = (REASON_MEASURED, REASON_BUDGETED, REASON_DEFAULT)
# A measured or budgeted value must say where it came from.
_REASONS_NEEDING_SOURCE = (REASON_MEASURED, REASON_BUDGETED)
_ENTRY_KEYS = frozenset({KEY_AFTER_DELAY, KEY_REASON, KEY_SOURCE})
PAIR_SEPARATOR = "->"


@dataclass(frozen=True)
class LinkDelay:
    """One configured after-delay and why it has that value."""

    ns: int
    literal: str
    reason: str
    source: str
    #: ``execution.yaml:connections.<name>`` or ``...links.<sender> -> <receiver>``
    evidence: str

    def comment(self) -> str:
        """The line the generated ``.lf`` carries above the connection."""
        why = f"{self.reason} ({self.source})" if self.source else self.reason
        return f"after-delay {self.literal}: {why} [{self.evidence}]"


class LinkDelays:
    """Configured delays by link name and by (name, sender, receiver)."""

    def __init__(self) -> None:
        self.by_name: dict[str, LinkDelay] = {}
        self.by_pair: dict[tuple[str, str, str], LinkDelay] = {}

    def get(self, name: str, sender: str, receiver: str) -> LinkDelay | None:
        """The pair's value, else the name's value, else None."""
        return self.by_pair.get((name, sender, receiver)) or self.by_name.get(name)

    def __bool__(self) -> bool:
        return bool(self.by_name or self.by_pair)


def link_delays(config: "ExecConfig | None") -> LinkDelays:
    """The ``LinkDelays`` of a loaded config (empty without ``connections:``)."""
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    out = LinkDelays()
    if config is None:
        return out
    for name, body in config.connections.items():
        prefix = f"execution.yaml:connections.{name}"
        if KEY_AFTER_DELAY in body:
            out.by_name[name] = _link_delay(body, prefix, parse_lf_time_ns)
        for (sender, receiver), pair in body.get(KEY_LINKS, {}).items():
            out.by_pair[(name, sender, receiver)] = _link_delay(
                pair, f"{prefix}.links.{sender} -> {receiver}", parse_lf_time_ns)
    return out


def _link_delay(body: dict, evidence: str, parse) -> LinkDelay:
    literal = body[KEY_AFTER_DELAY]
    return LinkDelay(
        ns=parse(literal, source=evidence), literal=literal,
        reason=body[KEY_REASON], source=body.get(KEY_SOURCE, ""), evidence=evidence,
    )


def validate_connections(raw: object, graph: "SystemGraph", source: str) -> dict[str, dict]:
    """Validate the ``connections:`` section against *graph*'s links.

    Names and node keys are slash-tolerant and normalized to the model form.
    An unknown name, an unknown pair or a pair that is not an end of the link
    is a hard ``ValueError``, like every other unknown key of the file.
    """
    from colcon_lf.comm_analysis.endpoints import endpoint_key_map

    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"{source}: 'connections' must be a mapping")
    r2k = endpoint_key_map(graph)
    comm = graph.communication
    ends: dict[str, set[tuple[str, str]]] = {}
    for tc in comm.topics:
        ends.setdefault(tc.topic, set()).update(
            (r2k.get(p, p), r2k.get(s, s)) for p in tc.publishers for s in tc.subscribers)
    for sc in comm.services:
        ends.setdefault(sc.service, set()).update(
            (r2k.get(c, c), r2k.get(sc.server, sc.server)) for c in sc.clients)
    for ac in comm.actions:
        if ac.server:
            ends.setdefault(ac.action, set()).update(
                (r2k.get(c, c), r2k.get(ac.server, ac.server)) for c in ac.clients)

    out: dict[str, dict] = {}
    for name_in, body in raw.items():
        name = _match(str(name_in), ends)
        if name is None:
            raise ValueError(
                f"{source}: connections: unknown link '{name_in}' "
                "(no topic, service or action of that name in system.json)")
        ctx = f"connections.'{name_in}'"
        if not isinstance(body, dict):
            raise ValueError(f"{source}: {ctx} must be a mapping")
        body = dict(body)
        links_raw = body.pop(KEY_LINKS, None)
        entry = _validate_entry(body, source, ctx, required=links_raw is None) if body else {}
        if links_raw is not None:
            if not isinstance(links_raw, dict) or not links_raw:
                raise ValueError(f"{source}: {ctx}.{KEY_LINKS} must be a non-empty mapping")
            pairs: dict[tuple[str, str], dict] = {}
            for pair_in, pair_body in links_raw.items():
                pair = _match_pair(str(pair_in), ends[name], source, f"{ctx}.{KEY_LINKS}")
                if not isinstance(pair_body, dict):
                    raise ValueError(f"{source}: {ctx}.{KEY_LINKS}.'{pair_in}' must be a mapping")
                pairs[pair] = _validate_entry(
                    pair_body, source, f"{ctx}.{KEY_LINKS}.'{pair_in}'", required=True)
            entry[KEY_LINKS] = pairs
        out[name] = entry
    return out


def _validate_entry(body: dict, source: str, ctx: str, *, required: bool) -> dict:
    unknown = set(body) - _ENTRY_KEYS
    if unknown:
        raise ValueError(f"{source}: {ctx}: unknown key(s) {', '.join(sorted(map(str, unknown)))}")
    if KEY_AFTER_DELAY not in body:
        if required or body:
            raise ValueError(f"{source}: {ctx}: '{KEY_AFTER_DELAY}' is required")
        return {}
    try:
        literal = _parse_after_delay(body[KEY_AFTER_DELAY])
    except ValueError as exc:
        raise ValueError(f"{source}: {ctx}: {exc}") from exc
    reason = body.get(KEY_REASON)
    if reason not in REASONS:
        raise ValueError(
            f"{source}: {ctx}: '{KEY_REASON}' must be one of {', '.join(REASONS)}, got {reason!r}")
    note = body.get(KEY_SOURCE)
    if note is not None and not isinstance(note, str):
        raise ValueError(f"{source}: {ctx}: '{KEY_SOURCE}' must be a string")
    note = (note or "").strip()
    if reason in _REASONS_NEEDING_SOURCE and not note:
        raise ValueError(
            f"{source}: {ctx}: a {reason} after_delay needs a '{KEY_SOURCE}' "
            "(the run, budget or document the value comes from)")
    entry = {KEY_AFTER_DELAY: literal, KEY_REASON: reason}
    if note:
        entry[KEY_SOURCE] = note
    return entry


def _match(name: str, known) -> str | None:
    for cand in (name, "/" + name, name[1:] if name.startswith("/") else None):
        if cand is not None and cand in known:
            return cand
    return None


def _match_pair(pair_in: str, ends: set[tuple[str, str]], source: str, ctx: str) -> tuple[str, str]:
    parts = [p.strip() for p in pair_in.split(PAIR_SEPARATOR)]
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"{source}: {ctx}: '{pair_in}' must be '<sender> -> <receiver>'")
    nodes = {n for pair in ends for n in pair}
    sender, receiver = (_match(p, nodes) or p for p in parts)
    if (sender, receiver) not in ends:
        listed = ", ".join(f"{s} -> {r}" for s, r in sorted(ends))
        raise ValueError(
            f"{source}: {ctx}: '{pair_in}' is not a sender -> receiver pair of this link "
            f"(pairs: {listed})")
    return sender, receiver

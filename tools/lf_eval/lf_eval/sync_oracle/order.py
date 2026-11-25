# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The records of one converted sync run log.

Reads a ``ros2 launch`` log of a generated system run with
``RCLLF_TRACE_CALLBACKS=1``:

* ``RCLLF_SYNC {json}`` lines of the showcase nodes (config, signal, drop,
  corpus, done, branch), grouped by their ``node`` field;
* ``rcllf corr: admitted edge <id> <pub> -> <sub> on '<topic>' (d = <ns> ns``
  lines, the edges and their link delay, and the ``admitted ingress
  edge`` and ``... with a deferred reader`` forms of the same record
  (``kind`` ingress / deferred, no delay for the deferred form). The binding
  prints one of the three for every edge it admits into a node;
* ``callback_start`` trace records of ``ros2_lf_corr_drain``: one per
  dispatched sample, with the writer's ``pub_gid``/``pub_seq`` and the tag;
* ``corr`` ``dispatch`` records and their outcome (MATCHED, LOST, TIMED_OUT,
  push_overflow, schedule_refused, ...);
* ``rcllf corr: legacy edge <topic> <pub> -> <sub>: <reason>`` lines;
* the binding's reconcile problems (``rcllf corr: reconcile: burst_exceeded
  edge 0: ...``, ``corr_runtime.cpp`` reconcile()).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field

_ADMITTED = re.compile(
    r"rcllf corr: admitted (ingress )?edge (\d+) (\S+) -> (\S+) on '([^']+)'"
    r"(?: \(d = (\d+) ns| (with a deferred reader))")
_LEGACY = re.compile(r"rcllf corr: run_epoch \d+, (\d+) admitted edge\(s\), (\d+) legacy edge\(s\)")
_LEGACY_EDGE = re.compile(r"rcllf corr: legacy edge (\S+) (\S+) -> (\S+): (.*)$")
_FAIL = re.compile(r"rcllf corr: reconcile: (.*)$")


@dataclass
class Run:
    sync: dict[str, list[dict]] = field(default_factory=dict)
    edges: dict[int, dict] = field(default_factory=dict)
    drains: list[dict] = field(default_factory=list)
    dispatch: list[dict] = field(default_factory=list)
    legacy: int | None = None
    legacy_edges: list[dict] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)

    def drains_of(self, node: str) -> list[dict]:
        """Drain dispatches of one showcase node, in execution order. The
        reactor name is ``<Class>__<node>``, or the node name itself."""
        return [d for d in self.drains
                if d["node"] == node or d["node"].endswith("__" + node)]

    def edges_into(self, node: str) -> dict[int, dict]:
        return {i: e for i, e in self.edges.items() if _is_node(e["sub"], node)}

    def legacy_into(self, node: str) -> list[dict]:
        return [e for e in self.legacy_edges if _is_node(e["sub"], node)]


def _is_node(reactor: str, node: str) -> bool:
    return reactor == node or reactor.endswith("__" + node)


def _json_after(line: str, marker: str) -> dict | None:
    i = line.find(marker)
    if i < 0:
        return None
    return json.loads(line[i + marker.index("{"):])


def parse_run(text: str) -> Run:
    run = Run()
    for line in text.splitlines():
        rec = _json_after(line, "RCLLF_SYNC {")
        if rec is not None:
            run.sync.setdefault(rec.get("node", ""), []).append(rec)
            continue
        m = _ADMITTED.search(line)
        if m:
            kind = "ingress" if m.group(1) else "deferred" if m.group(7) else "local"
            run.edges[int(m.group(2))] = {
                "pub": m.group(3), "sub": m.group(4), "topic": m.group(5), "kind": kind,
                "delay_ns": None if m.group(6) is None else int(m.group(6))}
            continue
        m = _LEGACY.search(line)
        if m:
            run.legacy = int(m.group(2))
            continue
        m = _LEGACY_EDGE.search(line)
        if m:
            run.legacy_edges.append({"topic": m.group(1), "pub": m.group(2), "sub": m.group(3),
                                     "reason": m.group(4).strip()})
            continue
        if '"event":"callback_start"' in line and '"ros2_lf_corr_drain"' in line:
            run.drains.append(_json_after(line, '{"event":"callback_start"'))
            continue
        if '"event":"corr","kind":"dispatch"' in line:
            run.dispatch.append(_json_after(line, '{"event":"corr"'))
            continue
        m = _FAIL.search(line)
        if m:
            run.failures.append(m.group(1).strip())
    return run


def input_of_callback(callback: str, inputs: tuple[str, str]) -> int | None:
    """Input index of a drained callback: ``<sync cb>_via_<member>``."""
    for i, member in enumerate(inputs):
        if callback.endswith("_via_" + member):
            return i
    return None

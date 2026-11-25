# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Sync corpora, the Python reader of ``corpus.hpp``'s format.

A corpus is a strict YAML subset (see ``sync_oracle_showcase/include/
sync_oracle_showcase/corpus.hpp``)::

    name: B1
    policy: exact
    queue: 3
    rows:
      - {tick: 0, input: 0, stamp_ms: 5, payload: 1}
    expect:
      - signal 1 2

Row r is the r-th entry of ``rows`` (0-based), its stamp is ``stamp_ms`` * 1e6
ns. The digest is FNV-1a 64 over the file bytes, the same value the corpus
player and oracle O1 log, so both sides provably read one file.
"""

from __future__ import annotations

import re
from pathlib import Path

DOMAIN = "mf-sync/1"
_FLOW = re.compile(r"(?:^|[{ ,])(\w+):\s*(-?\d+)")


def fnv1a64(data: bytes) -> str:
    h = 1469598103934665603
    for b in data:
        h ^= b
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"fnv1a64:{h:016x}"


def _strip_comment(line: str) -> str:
    for i, c in enumerate(line):
        if c == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i]
    return line


def parse_corpus(data: bytes) -> dict:
    """The corpus as ``{name, policy, queue, kind, rows, expect, digest}``."""
    c: dict = {"name": "", "policy": "", "queue": 0, "kind": "", "rows": [],
               "expect": [], "digest": fnv1a64(data)}
    section = ""
    for raw in data.decode().splitlines():
        line = _strip_comment(raw).strip()
        if not line:
            continue
        if line.startswith("-"):
            item = line[1:].strip()
            if section == "rows":
                f = dict((k, int(v)) for k, v in _FLOW.findall(item))
                row = {"row": len(c["rows"]), "tick": f["tick"], "input": f["input"],
                       "stamp_ns": f["stamp_ms"] * 1_000_000, "payload": f["payload"]}
                if row["input"] not in (0, 1) or row["stamp_ns"] <= 0 or row["tick"] < 0:
                    raise ValueError(f"corpus row out of range: {item}")
                c["rows"].append(row)
            elif section == "expect":
                c["expect"].append(item.strip('"'))
            else:
                raise ValueError(f"list item outside rows/expect: {line}")
            continue
        key, sep, value = line.partition(":")
        if not sep:
            raise ValueError(f"corpus line without ':': {line}")
        key, value = key.strip(), value.strip()
        section = key if not value else ""
        if key == "queue":
            c["queue"] = int(value)
        elif key in ("name", "policy", "kind"):
            c[key] = value
    return c


def load_corpus(path) -> dict:
    return parse_corpus(Path(path).read_bytes())


def arrival_order(rows: list[dict]) -> list[dict]:
    """lf_arrival_order: (tick, input, publish order).

    One firing of an input's d = 0 drain takes every record due at
    the tag, and input 0's drain reaction precedes input 1's (ruling
    2026-11-13 of the plan; ``corpus.hpp`` holds the same rule).
    """
    return sorted(rows, key=lambda r: (r["tick"], r["input"]))


def row_of(rows: list[dict], input_index: int, ordinal: int) -> dict | None:
    """The row a writer edge's ``ordinal`` (1-based, per input) carries: the
    player publishes tick by tick, each tick in file order."""
    mine = [r for r in arrival_order(rows) if r["input"] == input_index]
    return mine[ordinal - 1] if 0 < ordinal <= len(mine) else None

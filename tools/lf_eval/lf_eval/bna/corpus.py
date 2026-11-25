# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""BNA corpora: the ``.bna`` text form, JSONL, compact outcome strings, random corpora.

A ``.bna`` file is one statement per line, times in milliseconds (decimals
allowed), ``@T`` a tag at T ms microstep 0 and ``@T+M`` microstep M::

    params epoch=1 delta=10 lateness=0,0 capacity=8,8 retention=1000 timeout=47
    admit <input> <stamp> <key> @T
    missing <input> <key> @T
    wake @T
    close <shutdown|epoch|cancelled> @T
    expect D <anchor-key> MATCHED <partner-key> @T
    expect D <anchor-key> REJECTED(<reason>) @T
    expect S <key> <OUTCOME>|FAULT(<reason>) @T

``expect`` lines are the hand-derived expectation, written from the plan's
section 3 before the oracle ran. They are not input events.
"""

from __future__ import annotations

import json
import random
from decimal import Decimal
from pathlib import Path

MS = 1_000_000


def ms_to_ns(text: str) -> int:
    v = Decimal(text) * MS
    if v != v.to_integral_value():
        raise ValueError(f"{text} ms is not a whole number of ns")
    return int(v)


def ns_to_ms(ns: int) -> str:
    v = Decimal(ns) / MS
    return format(v.normalize(), "f") if v != v.to_integral_value() else str(int(v))


def parse_tag(text: str) -> list[int]:
    if not text.startswith("@"):
        raise ValueError(f"tag {text!r} must start with @")
    time, _, micro = text[1:].partition("+")
    return [ms_to_ns(time), int(micro or 0)]


def tag_text(tag) -> str:
    return "@" + ns_to_ms(tag[0]) + (f"+{tag[1]}" if tag[1] else "")


def parse_bna(text: str) -> tuple[list[dict], list[str]]:
    """Events (JSON records, ns) and the hand expectation strings of a ``.bna`` text."""
    events: list[dict] = []
    expect: list[str] = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        w = line.split()
        try:
            if w[0] == "params":
                kv = dict(x.split("=", 1) for x in w[1:])
                events.append({
                    "ev": "params", "sync": 0, "epoch": int(kv["epoch"]),
                    "delta": ms_to_ns(kv["delta"]),
                    "lateness": [ms_to_ns(x) for x in kv["lateness"].split(",")],
                    "capacity": [int(x) for x in kv["capacity"].split(",")],
                    "retention": ms_to_ns(kv["retention"]),
                    "timeout": ms_to_ns(kv["timeout"]),
                })
            elif w[0] == "admit":
                events.append({"ev": "admit", "sync": 0, "input": int(w[1]),
                               "stamp": ms_to_ns(w[2]), "key": w[3], "tag": parse_tag(w[4])})
            elif w[0] == "missing":
                events.append({"ev": "missing", "sync": 0, "input": int(w[1]), "key": w[2],
                               "tag": parse_tag(w[3])})
            elif w[0] == "wake":
                events.append({"ev": "wake", "sync": 0, "tag": parse_tag(w[1])})
            elif w[0] == "close":
                events.append({"ev": "close", "sync": 0, "reason": w[1],
                               "tag": parse_tag(w[2])})
            elif w[0] == "expect":
                parse_tag(w[-1])  # validate
                expect.append(" ".join(w[1:]))
            else:
                raise ValueError(f"unknown statement {w[0]!r}")
        except (KeyError, IndexError, ValueError) as exc:
            raise ValueError(f"line {n}: {raw!r}: {exc}") from exc
    return events, expect


def load_bna(path: Path) -> tuple[list[dict], list[str]]:
    return parse_bna(Path(path).read_text(encoding="utf-8"))


def compact(r: dict) -> str:
    """One output record in the ``expect`` notation."""
    if r["ev"] == "decision":
        what = (f"MATCHED {r['partner']}" if r["outcome"] == "MATCHED"
                else f"REJECTED({r['reason']})")
        return f"D {r['anchor']} {what} {tag_text(r['tag'])}"
    what = f"FAULT({r['reason']})" if r["outcome"] == "FAULT" else r["outcome"]
    return f"S {r['key']} {what} {tag_text(r['tag'])}"


def dumps(records) -> str:
    return "".join(json.dumps(r, separators=(",", ":")) + "\n" for r in records)


def read_jsonl(path: Path, prefix: str = "") -> list[dict]:
    """JSON records of a file. With ``prefix``, only lines carrying it (log scraping)."""
    out = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if prefix:
            i = line.find(prefix)
            if i < 0:
                continue
            line = line[i + len(prefix):]
        line = line.strip()
        if line.startswith("{"):
            out.append(json.loads(line))
    return out


# -- seeded random corpora (P1) ------------------------------------------

def random_corpus(seed: int, n_events: int = 60, stamped_missing: bool = False) -> list[dict]:
    """A corpus that exercises every rule: ties, late, faults, overflow, timeouts.

    *stamped_missing* gives every missing report a stamp bound."""
    rng = random.Random(seed)
    grid = 5 * MS
    events: list[dict] = [{
        "ev": "params", "sync": 0, "epoch": 1,
        "delta": rng.choice([0, 5, 10, 15]) * MS,
        "lateness": [rng.choice([0, 10, 25]) * MS, rng.choice([0, 10, 25]) * MS],
        "capacity": [rng.randint(1, 5), rng.randint(1, 5)],
        "retention": rng.choice([30, 80, 300]) * MS,
        "timeout": rng.choice([17, 47, 120]) * MS,
    }]
    t, micro = 0, 0
    ordinal = {}
    for _ in range(n_events):
        step = rng.choice([0, 0, 1, 2, 3, 5, 10, 20]) * MS
        t, micro = (t + step, 0) if step else (t, micro + 1)
        tag = [t, micro]
        roll = rng.random()
        if roll < 0.08:
            events.append({"ev": "wake", "sync": 0, "tag": tag})
            continue
        i = rng.randint(0, 1)
        gid = rng.choice(["a", "b"]) if i == 0 else rng.choice(["c", "d"])
        last = ordinal.get(gid, 0)
        ordv = last + 1
        fault = rng.random()
        if fault < 0.04 and last:
            ordv = last  # duplicate
        elif fault < 0.07 and last > 1:
            ordv = last - 1  # out of order
        else:
            ordinal[gid] = ordv
        epoch = 2 if rng.random() < 0.02 else 1
        key = f"{epoch}:{gid}:{ordv}"
        if roll < 0.13:
            events.append({"ev": "missing", "sync": 0, "input": i, "key": key, "tag": tag})
            if stamped_missing:
                events[-1]["stamp"] = (t // grid) * grid - rng.choice([0, 1, 2, 6]) * grid
            continue
        stamp = (t // grid) * grid - rng.choice([0, 0, 1, 2, 3, 6]) * grid
        if rng.random() < 0.02:
            stamp = -grid
        events.append({"ev": "admit", "sync": 0, "input": i, "stamp": stamp,
                       "key": key, "tag": tag})
    t += rng.choice([0, 50, 400]) * MS
    events.append({"ev": "close", "sync": 0,
                   "reason": rng.choice(["shutdown", "epoch", "cancelled"]), "tag": [t + MS, 0]})
    return events


FLUSH_STAMP = 10 ** 15


def permuted_corpus(seed: int, perm_seed: int, n: int = 30) -> list[dict]:
    """The same sample set in another admission order, for order insensitivity.

    Capacity, retention, timeout and lateness are large enough that capacity,
    expiry, timeouts and lateness never act, and a final pair of flush samples
    lifts both watermarks so every other anchor is decided.
    """
    rng = random.Random(seed)
    samples = []
    for j in range(n):
        i = rng.randint(0, 1)
        # One writer per sample, so any admission order keeps ordinals monotone.
        samples.append((i, rng.randrange(0, 40) * 5 * MS, f"1:w{j:03d}:1"))
    big = 10 ** 13
    params = {"ev": "params", "sync": 0, "epoch": 1, "delta": rng.choice([0, 5, 10]) * MS,
              "lateness": [big, big], "capacity": [n + 2, n + 2], "retention": big,
              "timeout": big}
    order = list(samples)
    random.Random(perm_seed).shuffle(order)
    events = [params]
    for j, (i, stamp, key) in enumerate(order):
        events.append({"ev": "admit", "sync": 0, "input": i, "stamp": stamp, "key": key,
                       "tag": [(j + 1) * MS, 0]})
    t = (len(order) + 1) * MS
    events.append({"ev": "admit", "sync": 0, "input": 1, "stamp": FLUSH_STAMP + big,
                   "key": "1:zz1:1", "tag": [t, 0]})
    events.append({"ev": "admit", "sync": 0, "input": 0, "stamp": FLUSH_STAMP + big,
                   "key": "1:zz0:1", "tag": [t, 1]})
    events.append({"ev": "close", "sync": 0, "reason": "shutdown", "tag": [t + MS, 0]})
    return events

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""First-difference comparison of BNA outputs (BNA specification).

Checks, in order, the parameter block, the input stream, the number of terminal
outcomes per admitted key, each outcome record against the oracle's (by key and
occurrence, including a sample's input and stamp and a decision's anchor
stamp), and finally the order of the records. Returns the first report or
``None``.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from typing import Optional

from lf_eval.bna.oracle import INPUT_EVENTS, OUTPUT_EVENTS


def _key(r: dict) -> str:
    return r["anchor"] if r["ev"] == "decision" else r["key"]


def describe(r: Optional[dict]) -> str:
    if r is None:
        return "none"
    if r["ev"] == "decision":
        if r["outcome"] == "MATCHED":
            return f"MATCHED {r['partner']}@{r['partner_stamp']}"
        return f"REJECTED({r['reason']})"
    return f"FAULT({r['reason']})" if r["outcome"] == "FAULT" else r["outcome"]


def _tag(r: dict) -> str:
    return f"[{r['tag'][0]},{r['tag'][1]}]"


def _identity(r: Optional[dict]) -> str:
    """The fields of a record that ``describe`` leaves out."""
    if r is None:
        return "none"
    if r["ev"] == "decision":
        return f"anchor stamp {r.get('stamp')}"
    return f"input {r.get('input')} stamp {r.get('stamp')}"


def _subject(r: dict) -> str:
    if r["ev"] == "decision":
        return f"decision_mismatch at anchor {r['anchor']} (stamp {r['stamp']})"
    return f"outcome_mismatch at sample {r['key']}"


def _inputs_strip(r: dict) -> dict:
    return {k: v for k, v in r.items() if k != "sync"}


def compare_inputs(expected: list[dict], actual: list[dict]) -> Optional[str]:
    """Parameter block, then the input stream, record by record."""
    e = [_inputs_strip(r) for r in expected if r.get("ev") in INPUT_EVENTS]
    a = [_inputs_strip(r) for r in actual if r.get("ev") in INPUT_EVENTS]
    ep = next((r for r in e if r["ev"] == "params"), None)
    ap = next((r for r in a if r["ev"] == "params"), None)
    if ep != ap:
        if ep is None or ap is None:
            return f"params_mismatch: expected {ep}, implementation {ap}"
        for k in sorted(set(ep) | set(ap)):
            if ep.get(k) != ap.get(k):
                return f"params_mismatch: {k} expected {ep.get(k)}, implementation {ap.get(k)}"
    e = [r for r in e if r["ev"] != "params"]
    a = [r for r in a if r["ev"] != "params"]
    for n, (x, y) in enumerate(zip(e, a)):
        if x != y:
            return f"input_mismatch at event {n}: expected {x}, implementation {y}"
    if len(e) != len(a):
        return f"input_mismatch: expected {len(e)} input events, implementation {len(a)}"
    return None


def compare_outputs(inputs: list[dict], oracle: list[dict],
                    impl: list[dict]) -> Optional[str]:
    """First difference between the oracle's outputs and the implementation's."""
    oracle = [r for r in oracle if r.get("ev") in OUTPUT_EVENTS]
    impl = [r for r in impl if r.get("ev") in OUTPUT_EVENTS]
    admits = Counter(r["key"] for r in inputs if r.get("ev") == "admit")
    got: dict[str, list[dict]] = defaultdict(list)
    for r in impl:
        got[_key(r)].append(r)
    # Exactly one terminal outcome per admitted sample (fix 3).
    for r in impl:
        k = _key(r)
        if len(got[k]) != admits.get(k, 0):
            outs = ", ".join(describe(x) for x in got[k])
            return (f"outcome_count_mismatch at sample {k}: {admits.get(k, 0)} admission(s), "
                    f"implementation {len(got[k])} outcome(s) ({outs})")
    seen: Counter = Counter()
    for r in oracle:
        k = _key(r)
        n = seen[k]
        seen[k] += 1
        other = got[k][n] if n < len(got[k]) else None
        if other is None or describe(r) != describe(other) or r["ev"] != other["ev"]:
            return f"{_subject(r)}: oracle {describe(r)}, implementation {describe(other)}"
        if _identity(r) != _identity(other):
            return f"{_subject(r)}: oracle {_identity(r)}, implementation {_identity(other)}"
        if r["tag"] != other["tag"]:
            return f"{_subject(r)}: oracle tag {_tag(r)}, implementation tag {_tag(other)}"
    for k, rs in got.items():
        if len(rs) > seen[k]:
            return f"extra_outcome at sample {k}: implementation {describe(rs[seen[k]])}"
    for n, (x, y) in enumerate(zip(oracle, impl)):
        if (_key(x), describe(x)) != (_key(y), describe(y)):
            return (f"order_mismatch at position {n}: oracle {_key(x)} {describe(x)}, "
                    f"implementation {_key(y)} {describe(y)}")
    return None


def compare(inputs: list[dict], oracle: list[dict], impl_inputs: list[dict],
            impl: list[dict]) -> Optional[str]:
    return compare_inputs(inputs, impl_inputs) or compare_outputs(inputs, oracle, impl)

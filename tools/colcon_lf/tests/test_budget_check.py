# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Item5 piece D (probes P27 to P27c): generation-time budget check on decentralized links.

A link passes when d + W >= L + eps + X + W_send (equality passes).
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from colcon_lf.federation.partition import _from_yaml, budget_errors

_HEAD = """\
system: s
coordination: {coord}
federates:
  a: {{nodes: [src]}}
  b: {{nodes: [dst]}}
after_overrides:
  - from: src.out
    to: dst.in
"""


def _fmap(tmp_path, coord="decentralized", **keys):
    body = _HEAD.format(coord=coord) + "".join(
        f"    {k}: {v}\n" for k, v in keys.items() if v is not None)
    path = tmp_path / "fed.yaml"
    path.write_text(body, encoding="utf-8")
    return _from_yaml(path, SimpleNamespace(nodes={"src": None, "dst": None}))


def _check(fm, extra=()):
    """budget_errors with every override's link emitted, plus *extra* links."""
    links = [(ov.from_port, ov.to_port) for ov in fm.after_overrides] + list(extra)
    return budget_errors(fm, generated_links=links)


# (id, keys, expected error substrings, [] means pass)
CASES = [
    ("clear_pass", dict(after="5ms", absent_after="5ms", max_latency="2ms",
                        clock_error="1ms", max_exec="1ms"), []),
    ("clear_refuse", dict(after="1ms", absent_after="1ms", max_latency="5ms",
                          clock_error="1ms", max_exec="2ms"),
     ["src.out -> dst.in", "d=1000000 ns", "W=1000000 ns", "L=5000000 ns",
      "eps=1000000 ns", "X=2000000 ns", "short by 6000000 ns"]),
    ("equality", dict(after="5ms", absent_after="3ms", max_latency="4ms",
                      clock_error="2ms", max_exec="2ms"), []),
    ("eps_zero", dict(after="2ms", absent_after="1ms", max_latency="3ms",
                      clock_error="0", max_exec="1ms"),
     ["eps=0 ns", "short by 1000000 ns"]),
    ("missing_x", dict(after="5ms", absent_after="5ms", max_latency="2ms",
                       clock_error="1ms"),
     ["src.out -> dst.in", "max_exec"]),
]


@pytest.mark.parametrize("name,keys,expected", CASES, ids=[c[0] for c in CASES])
def test_budget_table(tmp_path, name, keys, expected):
    errors = _check(_fmap(tmp_path, **keys))
    if not expected:
        assert errors == []
    else:
        assert len(errors) == 1
        for needle in expected:
            assert needle in errors[0]


def test_maxwait_is_no_credit(tmp_path):
    # The generator emits no LF maxwait (probe P01), so a maxwait key is not part
    # of W: d=1, W=0 declared, L+eps+X = 7 ms is refused with or without it.
    keys = dict(after="1ms", absent_after="0ms", max_latency="4ms", clock_error="1ms",
                max_exec="2ms")
    assert len(_check(_fmap(tmp_path, **keys))) == 1
    assert len(_check(_fmap(tmp_path, maxwait="6ms", **keys))) == 1


def test_stp_offset_is_alias_of_absent_after(tmp_path):
    keys = dict(after="5ms", max_latency="4ms", clock_error="2ms", max_exec="2ms")
    assert _check(_fmap(tmp_path, stp_offset="3ms", **keys)) == []
    assert len(_check(_fmap(tmp_path, stp_offset="2ms", **keys))) == 1
    assert _check(_fmap(tmp_path, absent_after="3ms", **keys)) == []
    with pytest.raises(ValueError, match="gives both 'stp_offset' and 'absent_after'"):
        _fmap(tmp_path, stp_offset="3ms", absent_after="3ms", **keys)


def test_undeclared_link_warned_and_not_checked(tmp_path, capsys):
    assert _check(_fmap(tmp_path, after="1ms")) == []
    assert "WARNING" in (err := capsys.readouterr().err) and "not checked" in err


def test_budget_terms_under_centralized_are_refused(tmp_path):
    # Under centralized coordination the RTI orders the tags and nothing checks
    # the terms, so declaring them is an error instead of a silent no-op.
    bad = dict(after="1ms", max_latency="5ms", clock_error="1ms", max_exec="2ms")
    errors = _check(_fmap(tmp_path, coord="centralized", **bad))
    assert len(errors) == 1
    assert "only apply under decentralized coordination" in errors[0]
    assert "src.out -> dst.in" in errors[0]
    assert _check(_fmap(tmp_path, coord="centralized", after="1ms")) == []


# --- Probe P27b: the sender federate's own W_send joins the left side ---------

def _chain(tmp_path, links, same_fed=()):
    """links: list of (from_node, to_node, keys). Every node is its own federate
    unless listed in same_fed (pairs sharing a federate)."""
    nodes = sorted({n for a, b, _ in links for n in (a, b)})
    feds = {n: n for n in nodes}
    for a, b in same_fed:
        feds[b] = feds[a]
    body = "system: s\ncoordination: decentralized\nfederates:\n"
    for fed in sorted(set(feds.values())):
        members = ", ".join(n for n in nodes if feds[n] == fed)
        body += f"  {fed}: {{nodes: [{members}]}}\n"
    body += "after_overrides:\n"
    for a, b, keys in links:
        body += f"  - from: {a}.out\n    to: {b}.in\n"
        body += "".join(f"    {k}: {v}\n" for k, v in keys.items())
    path = tmp_path / "chain.yaml"
    path.write_text(body, encoding="utf-8")
    return _from_yaml(path, SimpleNamespace(nodes={n: None for n in nodes}))


_BASE = dict(after="5ms", absent_after="5ms", max_latency="1ms", clock_error="0",
             max_exec="1ms")


def _w_in(ms):
    """An inbound link for 'front' with W = ms (undeclared budget, only W counts)."""
    return ("sensor", "front", dict(after="5ms", absent_after=f"{ms}ms"))


def test_p07b_f1_refused_by_sender_w(tmp_path):
    # d+W = 10 ms, L+eps+X = 2 ms (old check passes), W_send = 30 ms: 10 < 32
    links = [_w_in(30), ("front", "back", dict(_BASE))]
    errors = _check(_chain(tmp_path, links))
    assert len(errors) == 1
    for needle in ("front.out -> back.in", "W_send=30000000 ns", "short by 22000000 ns"):
        assert needle in errors[0]


def test_p07b_config_b_passes(tmp_path):
    # W_send = 3 ms: 10 >= 2 + 3
    assert _check(_chain(tmp_path, [_w_in(3), ("front", "back", dict(_BASE))])) == []


@pytest.mark.parametrize("w_send_ms,short_ns", [(8, None), (9, 1000000)])
def test_sender_w_boundary(tmp_path, w_send_ms, short_ns):
    # rhs = 2 + W_send, lhs = 10: equality at 8 passes, 9 is short by 1 ms
    errors = _check(_chain(tmp_path, [_w_in(w_send_ms), ("front", "back", dict(_BASE))]))
    if short_ns is None:
        assert errors == []
    else:
        assert len(errors) == 1 and f"short by {short_ns} ns" in errors[0]


def test_sender_without_inbound_adds_zero(tmp_path):
    errors = _check(_chain(tmp_path, [("front", "back", dict(_BASE))]))
    assert errors == []


def test_chain_adds_direct_sender_only(tmp_path):
    # z->a->b->c->d, every link d=5 W=5 (lhs 10), base 2. Direct sender: rhs 7
    # on every link, pass. A sum along the path would give 7, 12, 17 and refuse.
    keys = dict(_BASE)
    chain = [("z", "a", keys), ("a", "b", keys), ("b", "c", keys), ("c", "d", keys)]
    assert _check(_chain(tmp_path, chain)) == []


def test_chain_large_w_upstream_hits_only_next_link(tmp_path):
    # z->a has W = 20 ms: a->b needs 2 + 20 > 10 and is refused. b->c sees only
    # b's direct inbound a->b (W 5), rhs 7, passes: W does not accumulate.
    keys = dict(_BASE)
    zk = dict(_BASE, absent_after="20ms")
    chain = [("z", "a", zk), ("a", "b", keys), ("b", "c", keys)]
    errors = _check(_chain(tmp_path, chain))
    assert len(errors) == 1
    assert "a.out -> b.in" in errors[0] and "W_send=20000000 ns" in errors[0]


def test_sender_w_is_max_of_inbound_links(tmp_path):
    links = [("s1", "front", dict(after="5ms", absent_after="4ms")),
             ("s2", "front", dict(after="5ms", absent_after="9ms")),
             ("front", "back", dict(_BASE))]
    errors = _check(_chain(tmp_path, links))
    assert len(errors) == 1 and "W_send=9000000 ns" in errors[0]


def test_same_federate_inbound_not_remote(tmp_path):
    # 'loc' shares a federate with 'front', so loc->front is no remote input
    links = [("loc", "front", dict(after="5ms", absent_after="30ms")),
             ("front", "back", dict(_BASE))]
    assert _check(_chain(tmp_path, links, same_fed=[("front", "loc")])) == []


def test_same_federate_link_not_budget_checked(tmp_path, capsys):
    # loc->front lies inside one federate: its terms would be refused as a
    # decentralized link (d + W = 2 ms < 4 ms) although nothing crosses one
    links = [("loc", "front", dict(after="1ms", absent_after="1ms", max_latency="2ms",
                                   clock_error="1ms", max_exec="1ms"))]
    assert _check(_chain(tmp_path, links, same_fed=[("front", "loc")])) == []
    assert "declares no budget" not in capsys.readouterr().err


# --- Probe P27c: W_send uses the W the generator emits ------------------------
# front->back: d=5, W=5, L+eps+X = 2 ms, so lhs 10 ms. An inbound link into
# front adds the generator's default absent_after (10 ms, DEFAULT_STP_OFFSET_NS)
# when it declares none: rhs = 2 + 10 = 12 ms, short by 2 ms. The old check
# counted the undeclared term as 0 (rhs 2 ms) and passed.

def test_undeclared_absent_after_counts_generator_default(tmp_path):
    links = [("sensor", "front", dict(after="5ms")), ("front", "back", dict(_BASE))]
    errors = _check(_chain(tmp_path, links))
    assert len(errors) == 1, "expected a refusal, check passed with W_send=0 (default 10 ms ignored)"
    for needle in ("front.out -> back.in", "W_send=10000000 ns", "short by 2000000 ns"):
        assert needle in errors[0]


def test_default_w_equals_declared_default(tmp_path):
    # an undeclared absent_after behaves exactly like declaring the default 10 ms
    undeclared = [("sensor", "front", dict(after="5ms")), ("front", "back", dict(_BASE))]
    declared = [("sensor", "front", dict(after="5ms", absent_after="10ms")),
                ("front", "back", dict(_BASE))]
    assert _check(_chain(tmp_path, undeclared)) == _check(_chain(tmp_path, declared))


def test_own_link_w_uses_default_too(tmp_path):
    # the checked link itself: undeclared absent_after is 10 ms, not 0.
    # d=1, W=0+10, L+eps+X=11: equality passes, 12 is short by 1 ms
    ok = dict(after="1ms", max_latency="10ms", clock_error="0", max_exec="1ms")
    bad = dict(ok, max_exec="2ms")
    assert _check(_chain(tmp_path, [("a", "b", ok)])) == []
    errors = _check(_chain(tmp_path, [("a", "b", bad)]))
    assert len(errors) == 1 and "W=10000000 ns" in errors[0] and "short by 1000000 ns" in errors[0]


def test_generated_inbound_without_override_counts(tmp_path):
    # An admitted xcorr edge (or any generated cross link) into front that has no
    # after_overrides entry still runs at the default W of 10 ms.
    fm = _chain(tmp_path, [("front", "back", dict(_BASE))])
    assert _check(fm) == []
    errors = _check(fm, extra=[("sensor.out", "front.in")])
    assert len(errors) == 1, "expected a refusal, check passed with W_send=0 (xcorr inbound invisible)"
    assert "W_send=10000000 ns" in errors[0] and "short by 2000000 ns" in errors[0]


def test_generated_inbound_uses_declared_override_over_default(tmp_path):
    links = [("sensor", "front", dict(after="5ms", absent_after="3ms")),
             ("front", "back", dict(_BASE))]
    fm = _chain(tmp_path, links)
    assert _check(fm, extra=[("sensor.out", "front.in")]) == []


def test_generated_same_federate_link_not_remote(tmp_path):
    fm = _chain(tmp_path, [("front", "back", dict(_BASE)), ("loc", "front", dict(after="5ms"))],
                same_fed=[("front", "loc")])
    assert _check(fm, extra=[("loc.out", "front.in")]) == []


# --- only emitted links are checked and feed W_send --------------------------
# An override whose key matches no link the generator emits carries no
# message. a623b529 put every override into W_send and checked every override.

def test_stale_override_into_sender_adds_no_w_send(tmp_path):
    # sensor->front declares W = 30 ms but is not emitted; only front->back is.
    # d + W = 10 ms >= L + eps + X = 2 ms with W_send 0: the valid link passes.
    fm = _chain(tmp_path, [_w_in(30), ("front", "back", dict(_BASE))])
    errors = budget_errors(fm, generated_links=[("front.out", "back.in")])
    assert errors == [], f"valid link refused by a non-emitted inbound override: {errors}"


def test_stale_override_with_failing_terms_not_checked(tmp_path):
    # src.out -> dst.in fails its own terms (short by 6 ms) but is not emitted.
    fm = _fmap(tmp_path, after="1ms", absent_after="1ms", max_latency="5ms",
               clock_error="1ms", max_exec="2ms")
    errors = budget_errors(fm, generated_links=[("other.out", "dst.in")])
    assert errors == [], f"non-emitted override refused: {errors}"

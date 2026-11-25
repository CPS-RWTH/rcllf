# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Schedule-derived correspondence bounds (design sections 3.3 to 3.6).

``burst = k_e``, ``carrier = N_e(d_e)``, ``retain = N_e(d_e + L) + 1`` with
``N_e(W) = k_e * sum over publishing callbacks of (floor(W / P) + 1)`` for a
timer and the summed input edges for a subscription. Every expected value
below is derived by hand in the comment next to it, never read back from the
generator. Every assertion names the edge it is about.
"""

from __future__ import annotations

import dataclasses
import re

import pytest

from colcon_lf.node_analysis.models import CallbackInfo, PublisherInfo, SubscriberInfo, TimerInfo

from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.resolve import CorrBoundsConfig, resolve, resolve_corr_bounds
from colcon_lf.generation.binding_header_emitter import collect_rid_table, emit_rid_init_cpp
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.generation.lf_model import CorrEdge
from colcon_lf.generation.trace_manifest import build_static_manifest

from ..corr_declared import declared
from .test_correspondence_emission import MSG, _edge, _graph, _pub, _sub, _tc

MS = 1_000_000


def _fixture_graph(period_ms=200.0):
    """The correspondence_fixture shape: one timer publisher feeding one subscriber."""
    pub = _pub("pub", "PubNode")
    pub.timers[0].period_ms = period_ms
    return _graph([pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])


def _chain_graph():
    """pub (100 ms timer) -> /data -> sub (republishes on /out) -> /out -> sub2."""
    return _graph(
        [_pub("pub", "PubNode"), _sub("sub", "SubNode", publishes="out"),
         _sub("sub2", "Sub2Node", topic="out")],
        [_tc("/data", ["pub"], ["sub"]), _tc("/out", ["sub"], ["sub2"])],
    )


def _gen(graph, bounds, d_ms=1):
    return generate_lf_system(graph, after_delay_ns=d_ms * MS, corr_bounds=bounds)


def _triple(e):
    return (e.retain, e.carrier, e.burst)


# k, expected (retain, carrier, burst). P 200 ms, d 1 ms, T 100 ms (default),
# L = 2T = 200 ms, W = d + L = 201 ms. Invocations: floor(201/200) + 1 = 2 and
# floor(1/200) + 1 = 1. k 1: retain 2 + 1 = 3, carrier 1. k 3: retain 6 + 1 = 7,
# carrier 3. These are the S2 output-check values of scenarios one and many.
@pytest.mark.parametrize("k, want", [(1, (3, 1, 1)), (3, (7, 3, 3))])
def test_fixture_shaped_edge_bounds(k, want):
    graph = _fixture_graph()
    e = _edge(_gen(graph, declared(graph, k)), "pub", "sub")
    assert (e.status, e.edge_id) == ("admitted", 0), f"edge {e.label()}: {e}"
    assert _triple(e) == want, f"edge {e.label()}: (retain, carrier, burst) {_triple(e)}"
    assert (e.timeout_ns, e.max_lag_ns) == (100 * MS, 200 * MS), f"edge {e.label()}"
    assert e.bound_inputs == {
        "k": k, "d_ns": MS, "T_ns": 100 * MS, "L_ns": 200 * MS, "W_ns": 201 * MS,
        "periods_ns": {"pub.tick": [200 * MS]},
        "k_per_callback": {"pub.tick": {"k": k, "source": "declared"}},
    }, f"edge {e.label()}: {e.bound_inputs}"


def test_init_cpp_carries_the_bounds():
    graph = _fixture_graph()
    system = _gen(graph, declared(graph, 3))
    e = _edge(system, "pub", "sub")
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    # The last field is the reader depth, the registry default 8.
    tail = "1000000LL, 100000000LL, 200000000LL, 3u, 7u, 3u, 8u});"
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"


def test_burst_above_a_declared_reader_depth_is_refused():
    # k 9 publishes per invocation, the declared reader depth is 8.
    # One invocation would overwrite its own first sample before ingress takes.
    graph = _fixture_graph()
    bounds = declared(graph, 9)
    bounds = dataclasses.replace(bounds, reader_depth={**bounds.reader_depth, ("sub", "data"): 8})
    e = _edge(_gen(graph, bounds), "pub", "sub")
    assert (e.status, e.reason, e.edge_id) == ("legacy", "reader_depth_below_burst", None), (
        f"edge {e.label()} was admitted with k 9 above its declared reader depth 8, so one "
        f"invocation can overwrite its own sample in DDS: {e}")


def test_undeclared_reader_depth_is_raised_to_the_burst():
    # Without a declared depth the reader holds one whole
    # invocation, the registry default 8 raised to k 9. The binding deepens the
    # reader to it and refuses to start when the real depth is lower.
    graph = _fixture_graph()
    e = _edge(_gen(graph, declared(graph, 9)), "pub", "sub")
    assert (e.status, e.reader_depth) == ("admitted", 9), (
        f"edge {e.label()}: expected admitted with reader depth 9 (the burst), got "
        f"{e.status} {e.reason} depth {e.reader_depth}")


def test_declared_reader_depth_admits_the_burst_and_reaches_the_runtime():
    graph = _fixture_graph()
    system = _gen(graph, declared(graph, 9, reader_depth={("sub", "data"): 9}))
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reader_depth) == ("admitted", 9), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    assert line.endswith(", 9u});"), f"edge {e.label()}: reader depth 9 not emitted in {line!r}"


def test_undeclared_burst_is_refused():
    graph = _fixture_graph()
    e = _edge(_gen(graph, None), "pub", "sub")
    assert (e.status, e.reason, e.edge_id) == ("legacy", "burst_unbounded", None), (
        f"edge {e.label()}: {e}")


# Analyzer-proven burst bound (slice S3) ------------------------------------


def _proven(k_tick, topic="data"):
    """The fixture graph with the analyzer's publish bound on ``pub.tick``."""
    graph = _fixture_graph()
    tick = graph.nodes["pub"].callbacks[0]
    tick.publish_bounds = {topic: k_tick}
    if k_tick is None:
        tick.publish_unbounded = {topic: "loop at line 7"}
    return graph


# Proven k 1, nothing declared: the S2 values of k 1, retain 3, carrier 1, burst 1.
def test_proven_burst_admits_without_a_declaration():
    e = _edge(_gen(_proven(1), None), "pub", "sub")
    assert (e.status, e.edge_id) == ("admitted", 0), f"edge {e.label()}: {e}"
    assert _triple(e) == (3, 1, 1), f"edge {e.label()}: (retain, carrier, burst) {_triple(e)}"
    assert e.bound_inputs["k_per_callback"] == {"pub.tick": {"k": 1, "source": "proven"}}, (
        f"edge {e.label()}: {e.bound_inputs}")


# Two publish sites outside a loop: k 2, retain 2 * 2 + 1 = 5, carrier 2, burst 2.
def test_proven_burst_counts_call_sites():
    e = _edge(_gen(_proven(2), None), "pub", "sub")
    assert _triple(e) == (5, 2, 2), f"edge {e.label()}: (retain, carrier, burst) {_triple(e)}"


def test_publish_in_a_loop_is_refused_without_a_declaration():
    e = _edge(_gen(_proven(None), None), "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "burst_unbounded"), f"edge {e.label()}: {e}"


# A declaration wins over the proof: declared k 3 gives the S2 "many" values.
def test_declared_burst_wins_over_the_proven_one():
    graph = _proven(None)
    e = _edge(_gen(graph, declared(graph, 3)), "pub", "sub")
    assert _triple(e) == (7, 3, 3), f"edge {e.label()}: (retain, carrier, burst) {_triple(e)}"
    assert e.bound_inputs["k_per_callback"] == {"pub.tick": {"k": 3, "source": "declared"}}


# Two timer callbacks on /data, proven k 1 (tick) and k 2 (tock), both 100 ms.
# W 201 ms: tick 3 invocations, tock 3. N = 1 * 3 + 2 * 3 = 9, retain 10.
# carrier = 1 * 1 + 2 * 1 = 3. burst is the larger k, 2.
def test_burst_is_the_largest_k_of_the_publishing_callbacks():
    from colcon_lf.node_analysis.models import CallbackInfo

    pub = _pub("pub", "PubNode", callbacks=[
        CallbackInfo(name="tick", publishers_used=["data"], publish_bounds={"data": 1}),
        CallbackInfo(name="tock", publishers_used=["data"], publish_bounds={"data": 2}),
    ])
    graph = _graph([pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    e = _edge(_gen(graph, None), "pub", "sub")
    assert _triple(e) == (10, 3, 2), f"edge {e.label()}: (retain, carrier, burst) {_triple(e)}"


def test_one_unbounded_callback_refuses_the_edge():
    from colcon_lf.node_analysis.models import CallbackInfo

    pub = _pub("pub", "PubNode", callbacks=[
        CallbackInfo(name="tick", publishers_used=["data"], publish_bounds={"data": 1}),
        CallbackInfo(name="tock", publishers_used=["data"], publish_bounds={"data": None}),
    ])
    graph = _graph([pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    e = _edge(_gen(graph, None), "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "burst_unbounded"), f"edge {e.label()}: {e}"


def test_serialized_publish_is_refused_as_unsupported_api():
    graph = _proven(1)
    graph.nodes["pub"].publishers[0].serialized_publish = True
    e = _edge(_gen(graph, declared(graph)), "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "unsupported_api"), f"edge {e.label()}: {e}"


# H001 text per edge status ----------------------------------------------------


def _h001(graph, bounds):
    from colcon_lf.features.diagnostics import DetectionContext
    from colcon_lf.features.modules.best_effort_input import detect, h001_by_edge_status

    graph.nodes["sub"].subscribers[0].qos_reliability = "best_effort"
    diags = detect(DetectionContext(graph=graph))
    return diags, h001_by_edge_status(diags, _gen(graph, bounds).corr_edges)


def test_h001_on_an_admitted_edge_is_an_info_note():
    from colcon_lf.features.diagnostics import Severity

    _, ((sev, msg),) = _h001(_proven(1), None)
    assert sev == Severity.INFO, msg
    assert "admitted correspondence edge(s) /data (pub -> sub)" in msg, msg
    assert "Drops become missing outcomes, see the deadline hook" in msg, msg


def test_h001_on_a_legacy_edge_keeps_the_warning_text():
    from colcon_lf.features.diagnostics import Severity

    (diag,), ((sev, msg),) = _h001(_proven(None), None)
    assert (sev, msg) == (Severity.WARNING, diag.message)
    assert "do not compensate for transport drops" in msg, msg


def test_unknown_timer_period_is_refused():
    # A timer without a period refuses the whole generation; it no
    # longer gets a 1000 msec placeholder next to a legacy edge.
    graph = _fixture_graph(period_ms=None)
    with pytest.raises(ValueError, match="timer callback 'tick' has no period"):
        _gen(graph, declared(graph))


def test_chain_bounds_compose():
    graph = _chain_graph()
    system = _gen(graph, CorrBoundsConfig(burst={"pub": 1, "sub": 2, "sub2": 1}))
    data, out = _edge(system, "pub", "sub"), _edge(system, "sub", "sub2")
    # T_out is its own 100 ms (no chaining), so L = 2 * 100 = 200 ms, W 201 ms.
    # /data: P 100 ms, W 201 ms: N = 3, retain 4, carrier floor(1/100) + 1 = 1.
    assert _triple(data) == (4, 1, 1), f"edge {data.label()}: {_triple(data)}"
    # /out: sub runs once per /data message, k 2: N(201) = 2 * 3 = 6, retain 7,
    # carrier 2 * N_data(1) = 2.
    assert _triple(out) == (7, 2, 2), f"edge {out.label()}: {_triple(out)}"


# No timeout chaining (causal windows) ----------------------------------------
# The binding counts T from the publication's commit, so a relay that
# publishes after an upstream join still keeps its full d + T. Every edge
# keeps its subscription's own take_timeout_ms.


def test_relay_edge_keeps_its_own_timeout():
    # pub -> /data -> sub (relay) -> /out -> sub2, default T 100 ms each.
    graph = _chain_graph()
    system = _gen(graph, CorrBoundsConfig(burst={"pub": 1, "sub": 1, "sub2": 1}))
    data, out = _edge(system, "pub", "sub"), _edge(system, "sub", "sub2")
    assert (data.timeout_ns, out.timeout_ns) == (100 * MS, 100 * MS), (
        f"T /data {data.timeout_ns}, /out {out.timeout_ns}: the relay edge /out was chained "
        f"over its publisher's input, the causal cutoff needs no chain")
    assert (data.max_lag_ns, out.max_lag_ns) == (200 * MS, 200 * MS), (data, out)
    assert out.bound_inputs["T_ns"] == 100 * MS, out.bound_inputs
    for e in (data, out):
        assert "T_upstream_ns" not in e.bound_inputs, f"edge {e.label()}: {e.bound_inputs}"


def _long_chain_graph():
    """pub -> /a -> r1 -> /b -> r2 -> /c -> sink, and pub2 -> /x -> r2 (fan-in at r2)."""
    return _graph(
        [_pub("pub", "PubNode", topic="a"), _pub("pub2", "Pub2Node", topic="x"),
         _sub("r1", "R1Node", topic="a", publishes="b"),
         _r2_node(), _sub("sink", "SinkNode", topic="c")],
        [_tc("/a", ["pub"], ["r1"]), _tc("/x", ["pub2"], ["r2"]),
         _tc("/b", ["r1"], ["r2"]), _tc("/c", ["r2"], ["sink"])],
    )


def _r2_node():
    """Relay r2: ``on_data`` (/b) and ``on_x`` (/x) both publish /c."""
    node = _sub("r2", "R2Node", topic="b", publishes="c")
    node.callbacks.append(CallbackInfo(name="on_x", publishers_used=["c"]))
    node.subscribers.append(SubscriberInfo(topic="x", message_type=MSG, callback="on_x"))
    return node


def test_path_and_fan_in_keep_the_declared_timeouts():
    # Declared own T: /a 30, /x 120, /b 50, /c 70 ms. Each edge keeps it.
    graph = _long_chain_graph()
    t = {("r1", "a"): 30, ("r2", "x"): 120, ("r2", "b"): 50, ("sink", "c"): 70}
    bounds = CorrBoundsConfig(burst={n: 1 for n in ("pub", "pub2", "r1", "r2")},
                              take_timeout_ms=t)
    system = _gen(graph, bounds)
    got = {e.topic: e.timeout_ns // MS for e in system.corr_edges if e.status == "admitted"}
    assert got == {"/a": 30, "/x": 120, "/b": 50, "/c": 70}, (
        f"T per edge {got}, want the declared take_timeout_ms with no chain")
    # L = 2 * the largest T = 240 ms.
    assert _edge(system, "r2", "sink").max_lag_ns == 240 * MS


def test_declared_lag_below_a_timeout_is_a_config_error():
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"pub": 1, "sub": 1, "sub2": 1}, max_worker_lag_ms=50)
    with pytest.raises(ValueError, match=r"max_worker_lag_ms=50 is below the largest "
                                         r"maxwait_ms of an admitted correspondence edge "
                                         r"\(100\)"):
        _gen(graph, bounds)


def test_edge_leaving_a_self_loop_keeps_its_own_timeout():
    # loop: timer tick publishes /s, on_s (subscribed to /s) publishes /out.
    loop = _pub("loop", "LoopNode", topic="s")
    loop.callbacks.append(CallbackInfo(name="on_s", publishers_used=["out"]))
    loop.subscribers.append(SubscriberInfo(topic="s", message_type=MSG, callback="on_s"))
    loop.publishers.append(PublisherInfo(topic="out", message_type=MSG))
    graph = _graph([loop, _sub("sink", "SinkNode", topic="out")],
                   [_tc("/s", ["loop"], ["loop"]), _tc("/out", ["loop"], ["sink"])])
    system = _gen(graph, CorrBoundsConfig(burst={"loop": 1}))
    got = {e.topic: (e.status, e.timeout_ns // MS) for e in system.corr_edges}
    assert got == {"/s": ("admitted", 100), "/out": ("admitted", 100)}, (
        f"(status, T) per edge {got}, /out keeps its own 100 ms")
    for e in system.corr_edges:
        assert "timeout_chain_cycle" not in e.bound_inputs, e.bound_inputs


def test_refusal_reaches_a_fixed_point_downstream():
    graph = _chain_graph()
    system = _gen(graph, CorrBoundsConfig(burst={"sub": 1, "sub2": 1}))
    data, out = _edge(system, "pub", "sub"), _edge(system, "sub", "sub2")
    assert (data.status, data.reason) == ("legacy", "burst_unbounded"), f"edge {data.label()}"
    assert (out.status, out.reason) == ("legacy", "rate_unbounded"), (
        f"edge {out.label()}: its only input {data.label()} was refused, got {out}")


def test_declared_min_input_period_bounds_an_unbounded_input():
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 50.0})
    system = _gen(graph, bounds)
    data, out = _edge(system, "pub", "sub"), _edge(system, "sub", "sub2")
    assert (data.status, data.reason) == ("legacy", "burst_unbounded"), f"edge {data.label()}"
    # /out: sub invoked at most floor(201/50) + 1 = 5 times in W, k 1: retain 6,
    # carrier floor(1/50) + 1 = 1. Renumbered densely to id 0.
    assert (out.status, out.edge_id) == ("admitted", 0), f"edge {out.label()}: {out}"
    assert _triple(out) == (6, 1, 1), f"edge {out.label()}: {_triple(out)}"
    assert out.bound_inputs["min_input_periods_ns"] == {"sub:data": 50 * MS}, out.bound_inputs


def test_declared_lag_and_timeout_set_the_window():
    graph = _fixture_graph()
    # L 500 ms: W 501 ms, floor(501/200) + 1 = 3, retain 4.
    e = _edge(_gen(graph, declared(graph, max_worker_lag_ms=500)), "pub", "sub")
    assert (e.max_lag_ns, e.retain) == (500 * MS, 4), f"edge {e.label()}: {e}"
    # T 300 ms, no L: L = 600 ms, W 601 ms, floor(601/200) + 1 = 4, retain 5.
    bounds = declared(graph, take_timeout_ms={("sub", "data"): 300})
    e = _edge(_gen(graph, bounds), "pub", "sub")
    assert (e.timeout_ns, e.max_lag_ns, e.retain) == (300 * MS, 600 * MS, 5), (
        f"edge {e.label()}: {e}")


def test_carrier_above_the_old_fixed_capacity_is_emitted_as_is():
    graph = _fixture_graph()
    # k 9: carrier 9 * (floor(1/200) + 1) = 9 > 8, the binding's former fixed
    # kCarrierCap. The table now sizes the carrier from the spec, so no warning.
    system = _gen(graph, declared(graph, 9, reader_depth={("sub", "data"): 9}))
    e = _edge(system, "pub", "sub")
    assert e.carrier == 9, f"edge {e.label()}: {e.carrier}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    # retain 9 * (floor(201/200) + 1) + 1 = 19.
    assert line.endswith("9u, 19u, 9u, 9u});"), f"edge {e.label()}: {line!r}"


# Review findings ------------------------------------------------


def test_retain_above_uint32_is_refused_at_generation():
    graph = _fixture_graph()
    # k 2^31: retain 2^31 * (floor(201/200) + 1) + 1 = 4294967297 > 4294967295,
    # which would narrow in the generated uint32_t literal.
    with pytest.raises(ValueError, match=r"retain = 4294967297 exceeds the C\+\+ field limit "
                                         r"4294967295 \(uint32_t\)"):
        _gen(graph, declared(graph, 2**31, reader_depth={("sub", "data"): 2**31}))


def test_lag_above_int64_is_refused_at_generation():
    graph = _fixture_graph()
    # L 2^63 / 10^6 ms rounded up: L in ns is above 2^63 - 1.
    lag_ms = 2**63 // MS + 1
    with pytest.raises(ValueError, match=rf"max_lag_ns = {lag_ms * MS} exceeds the C\+\+ field "
                                         r"limit 9223372036854775807 \(int64_t\)"):
        _gen(graph, declared(graph, max_worker_lag_ms=lag_ms))


# 1e-7 ms (0.1 ns) and 6e-7 ms (0.6 ns) are below 1 ns, the second used to round
# up to 1 ns. 1e303 ms overflows the int64 nanosecond range (it used to scale to inf).
OUT_OF_RANGE_MS = [1e-7, 6e-7, 1e303]


@pytest.mark.parametrize("period_ms", OUT_OF_RANGE_MS)
def test_declared_min_input_period_out_of_range_is_a_config_error(period_ms):
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): period_ms})
    want = (f"min_input_period_ms={period_ms!r} of sub:data is not a finite period from 1 ns "
            "to the int64 nanosecond range")
    with pytest.raises(ValueError, match=re.escape(want)):
        _gen(graph, bounds)


@pytest.mark.parametrize("period_ms", OUT_OF_RANGE_MS)
def test_timer_period_out_of_range_is_refused(period_ms):
    graph = _fixture_graph(period_ms=period_ms)
    # Outside 1 ns to the int64 range there is no LF time literal
    # the binding can match ("1e-07 msec" is rejected by lfc), so generation
    # refuses instead of emitting the timer next to a legacy edge.
    with pytest.raises(ValueError, match="is not a nanosecond count from 1 ns"):
        _gen(graph, declared(graph))


def test_timer_period_is_floored_not_rounded_up():
    # 1.6e-6 ms is 1.6 ns. Rounding to 2 ns would halve the counted publications.
    graph = _fixture_graph(period_ms=1.6e-6)
    e = _edge(_gen(graph, declared(graph)), "pub", "sub")
    assert e.status == "admitted", f"edge {e.label()}: {e}"
    assert e.bound_inputs["periods_ns"] == {"pub.tick": [1]}, f"edge {e.label()}: {e.bound_inputs}"
    # W = d + L = 201 ms = 201000000 ns at P 1 ns: N = 201000001, retain N + 1.
    assert e.retain == 201 * MS + 2, f"edge {e.label()}: {_triple(e)}"


def test_declared_min_input_period_is_floored_not_rounded_up():
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 1.6e-6})
    out = _edge(_gen(graph, bounds), "sub", "sub2")
    assert out.status == "admitted", f"edge {out.label()}: {out}"
    assert out.bound_inputs["min_input_periods_ns"] == {"sub:data": 1}, out.bound_inputs


def test_two_timers_on_one_callback_both_appear_in_the_inputs():
    graph = _fixture_graph()
    graph.nodes["pub"].timers.append(TimerInfo(period_ms=300.0, callback="tick"))
    e = _edge(_gen(graph, declared(graph)), "pub", "sub")
    # W 201 ms: floor(201/200) + 1 = 2 and floor(201/300) + 1 = 1, retain 3 + 1 = 4.
    # d 1 ms: 1 + 1 = 2 carrier.
    assert _triple(e) == (4, 2, 1), f"edge {e.label()}: {_triple(e)}"
    assert e.bound_inputs["periods_ns"] == {"pub.tick": [200 * MS, 300 * MS]}, (
        f"edge {e.label()}: {e.bound_inputs}")


def test_lag_below_take_timeout_is_a_config_error():
    graph = _fixture_graph()
    with pytest.raises(ValueError, match="max_worker_lag_ms=50 is below the largest maxwait_ms"):
        _gen(graph, declared(graph, max_worker_lag_ms=50))


def test_manifest_lists_the_bounds():
    graph = _fixture_graph()
    system = _gen(graph, declared(graph))
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    doc = build_static_manifest(
        executables=[system.name], corr_edges=system.corr_edges, rid_table=rid,
        model_hash="sha256:m", generated_source_hash="s" * 64, provider_marker="rcllf 2.13.7",
    )
    (row,) = doc["edges"]
    assert row["bounds"] == {
        "burst": 1, "retain": 3, "carrier": 1, "reader_depth": 8, "timeout_ns": 100 * MS,
        "max_lag_ns": 200 * MS, "join": "blocking",
        # A local edge is decided at publish, it has no cutoff.
        "local_now": True, "cutoff": "none",
        "inputs": {"k": 1, "d_ns": MS, "T_ns": 100 * MS, "L_ns": 200 * MS, "W_ns": 201 * MS,
                   "periods_ns": {"pub.tick": [200 * MS]},
                   "k_per_callback": {"pub.tick": {"k": 1, "source": "declared"}}},
    }, row


def test_refused_manifest_row_has_no_bounds():
    graph = _fixture_graph()
    system = _gen(graph, None)
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    doc = build_static_manifest(
        executables=[system.name], corr_edges=system.corr_edges, rid_table=rid,
        model_hash="sha256:m", generated_source_hash="s" * 64, provider_marker="rcllf 2.13.7",
    )
    (row,) = doc["edges"]
    assert row["reason"] == "burst_unbounded" and "bounds" not in row, row


# Exec-config keys -----------------------------------------------------------


def test_exec_config_precedence_and_scopes():
    graph = _chain_graph()
    raw = {
        "defaults": {"max_publishes_per_invocation": 1, "max_worker_lag_ms": 400,
                     "min_input_period_ms": 20},
        "nodes": {"sub": {"max_publishes_per_invocation": 2, "take_timeout_ms": 150}},
        "subscriptions": {"sub2": {"out": {"min_input_period_ms": 7.5}}},
    }
    b = resolve_corr_bounds(ExecConfig.from_dict(raw, graph, source="t"), graph)
    assert b.burst == {"pub": 1, "sub": 2, "sub2": 1}, b
    assert b.max_worker_lag_ms == 400, b
    assert b.take_timeout_ms == {("sub", "data"): 150}, b
    assert b.min_input_period_ms == {("sub", "data"): 20, ("sub2", "out"): 7.5}, b
    raw["subscriptions"]["sub2"]["out"]["reader_depth"] = 16
    raw["nodes"]["sub"]["reader_depth"] = 4
    b = resolve_corr_bounds(ExecConfig.from_dict(raw, graph, source="t"), graph)
    assert b.reader_depth == {("sub", "data"): 4, ("sub2", "out"): 16}, b
    assert b.depth("sub2", "out") == 16 and b.depth("nobody", "x") == 8, b
    rows = resolve(ExecConfig.from_dict(raw, graph, source="t"), graph)
    text = repr(rows)
    for key in ("max_publishes_per_invocation", "max_worker_lag_ms", "min_input_period_ms",
                "reader_depth"):
        assert key not in text, f"generation-time key {key} leaked into runtime rows: {text}"


@pytest.mark.parametrize("raw, match", [
    ({"nodes": {"sub": {"max_worker_lag_ms": 100}}}, "max_worker_lag_ms"),
    ({"subscriptions": {"sub": {"data": {"max_publishes_per_invocation": 1}}}},
     "max_publishes_per_invocation"),
    ({"defaults": {"max_publishes_per_invocation": 0}}, "max_publishes_per_invocation"),
    ({"defaults": {"max_worker_lag_ms": -1}}, "max_worker_lag_ms"),
    ({"defaults": {"min_input_period_ms": 0}}, "min_input_period_ms"),
    ({"defaults": {"min_input_period_ms": 1e-7}}, "min_input_period_ms"),
    ({"defaults": {"min_input_period_ms": 6e-7}}, "min_input_period_ms"),
    ({"defaults": {"min_input_period_ms": 1e303}}, "min_input_period_ms"),
    ({"defaults": {"min_input_period_ms": float("inf")}}, "min_input_period_ms"),
    ({"defaults": {"min_input_period_ms": float("nan")}}, "min_input_period_ms"),
    ({"defaults": {"max_publishes_per_invocation": True}}, "max_publishes_per_invocation"),
    ({"defaults": {"reader_depth": 0}}, "reader_depth"),
])
def test_exec_config_rejects_bad_keys(raw, match):
    graph = _chain_graph()
    # ValueError only: 1e303 used to escape as OverflowError from round(inf).
    with pytest.raises(ValueError, match=match):
        ExecConfig.from_dict(raw, graph, source="t")


def test_exec_config_accepts_exactly_one_ns():
    graph = _chain_graph()
    cfg = ExecConfig.from_dict({"defaults": {"min_input_period_ms": 0.000001}}, graph, source="t")
    assert cfg is not None


# join: deferred ------------------------------------------------


def test_join_key_resolves_per_subscription_and_rejects_other_values():
    graph = _chain_graph()
    raw = {"defaults": {"join": "blocking"}, "subscriptions": {"sub": {"data": {"join": "deferred"}}}}
    b = resolve_corr_bounds(ExecConfig.from_dict(raw, graph, source="t"), graph)
    assert b.deferred_join == {("sub", "data")}, b
    assert b.deferred("sub", "data") and not b.deferred("sub2", "out"), b
    rows = resolve(ExecConfig.from_dict(raw, graph, source="t"), graph)
    assert "join" not in repr(rows), f"generation-time key join leaked into runtime rows: {rows}"
    with pytest.raises(ValueError, match="'join' must be one of"):
        ExecConfig.from_dict({"defaults": {"join": "lazy"}}, graph, source="t")


def test_blocking_inputs_give_their_own_timeouts_and_no_hold_warning(recwarn):
    # A lost sample's join may hold sub, but /out's cutoff counts from
    # its own commit, so the hold costs /out no slack. No hold
    # warning and no chain. (the deferred variant of this chain is
    # a generation error now, both edges are local.)
    graph = _chain_graph()
    burst = {"pub": 1, "sub": 1, "sub2": 1}
    system = _gen(graph, CorrBoundsConfig(burst=burst))
    data, out = _edge(system, "pub", "sub"), _edge(system, "sub", "sub2")
    assert not data.deferred, data
    assert (data.timeout_ns, out.timeout_ns) == (100 * MS, 100 * MS), (
        f"T /data {data.timeout_ns}, /out {out.timeout_ns}")
    assert "join_hold_may_delay" not in out.bound_inputs, out.bound_inputs
    holds = [str(w.message) for w in recwarn if "holds its LF program" in str(w.message)]
    assert holds == [], holds


def test_deferred_edge_emits_its_header_line_and_manifest_join():
    # Only a cross-federate ingress row can be deferred, so the
    # emitter and the manifest are checked on the fixture edge as one.
    graph = _fixture_graph()
    system = _gen(graph, declared(graph))
    rows = [dataclasses.replace(e, role="ingress", deferred=True, federate="sub", global_id=0,
                                peer_federate="pub", peer_node="/pub", pub_rids=[])
            for e in system.corr_edges]
    e = rows[0]
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    cpp = emit_rid_init_cpp(rid, "pkg", corr_edges=rows)
    assert f"  ros2_lf_binding::set_corr_edge_deferred({e.edge_id});" in cpp.splitlines(), cpp
    doc = build_static_manifest(
        executables=[system.name], corr_edges=rows, rid_table=rid,
        model_hash="sha256:m", generated_source_hash="s" * 64, provider_marker="rcllf 2.13.7",
    )
    assert doc["edges"][0]["bounds"]["join"] == "deferred", doc["edges"][0]
    assert "cutoff" not in doc["edges"][0]["bounds"], doc["edges"][0]
    blocking = _gen(graph, declared(graph))
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in blocking.reactors}, graph),
                            "pkg", corr_edges=blocking.corr_edges)
    assert "set_corr_edge_deferred" not in cpp, cpp


def test_self_edge_with_join_deferred_is_a_generation_error():
    loop = _pub("loop", "LoopNode", topic="s")
    loop.callbacks.append(CallbackInfo(name="on_s", publishers_used=[]))
    loop.subscribers.append(SubscriberInfo(topic="s", message_type=MSG, callback="on_s"))
    graph = _graph([loop], [_tc("/s", ["loop"], ["loop"])])
    with pytest.raises(ValueError, match=r"'join: deferred' on loop 's' is not supported.*"
                                         r"self edge whose sample is present at the drain"):
        _gen(graph, CorrBoundsConfig(burst={"loop": 1}, deferred_join={("loop", "s")}))


def test_egress_row_marked_deferred_is_refused_by_the_emitter():
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = _gen(graph, declared(graph))
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    egress = [dataclasses.replace(e, role="egress", deferred=True, global_id=0, peer_node="/sub")
              for e in system.corr_edges]
    with pytest.raises(ValueError, match=r"egress edge .* is marked 'join: deferred'"):
        emit_rid_init_cpp(rid, "pkg", corr_edges=egress)


# cutoff_anchor (tag only) -----------------------

_SENDER_START_REMOVED = (
    r"'cutoff_anchor: sender_start' was removed: the cutoff of a "
    r"cross-federate edge always counts from the publication tag, "
    r"t \+ max\(0, d\) \+ maxwait_ms \+ eps")


@pytest.mark.parametrize("raw", [
    {"defaults": {"cutoff_anchor": "sender_start"}},
    {"nodes": {"sub": {"cutoff_anchor": "sender_start"}}},
    {"subscriptions": {"sub2": {"out": {"cutoff_anchor": "sender_start"}}}},
], ids=["defaults", "nodes", "subscriptions"])
def test_cutoff_anchor_sender_start_is_rejected_with_the_752_message(raw):
    # Falsification: the removed physical-start anchor is an error
    # in every scope, with the message that names the issue and the tag rule.
    graph = _chain_graph()
    with pytest.raises(ValueError, match=_SENDER_START_REMOVED):
        ExecConfig.from_dict(raw, graph, source="t")


def test_cutoff_anchor_tag_parses_and_other_values_are_refused():
    graph = _chain_graph()
    for raw in ({"defaults": {"cutoff_anchor": "tag"}},
                {"subscriptions": {"sub2": {"out": {"cutoff_anchor": "tag"}}}}):
        cfg = ExecConfig.from_dict(raw, graph, source="t")
        rows = resolve(cfg, graph)
        assert "cutoff_anchor" not in repr(rows), (
            f"generation-time key cutoff_anchor leaked into runtime rows: {rows}")
        resolve_corr_bounds(cfg, graph)
    with pytest.raises(ValueError, match=r"'cutoff_anchor' must be one of \('tag',\)"):
        ExecConfig.from_dict({"defaults": {"cutoff_anchor": "commit"}}, graph, source="t")


def test_no_build_emits_a_tag_anchor_call():
    # The tag rule is unconditional in the binding, so the key
    # changes no generated text. `cutoff_anchor: tag` and no key give the same
    # init code and correspondence rows, and neither names the removed API.
    graph = _chain_graph()
    tagged = resolve_corr_bounds(
        ExecConfig.from_dict({"defaults": {"cutoff_anchor": "tag"}}, graph, source="t"), graph)
    plain = resolve_corr_bounds(ExecConfig.from_dict({}, graph, source="t"), graph)
    assert tagged == plain, (tagged, plain)
    system = _gen(graph, declared(graph))
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    cpp = emit_rid_init_cpp(rid, "pkg", corr_edges=system.corr_edges)
    assert "tag_anchor" not in cpp, cpp
    assert all("cutoff_anchor" not in e.to_json() for e in system.corr_edges), (
        [e.to_json() for e in system.corr_edges])


def test_declared_min_input_period_reaches_the_runtime_check():
    """The declared period that sized /out is registered with the
    binding under the subscriber's binding key, which counts every run of the
    callback closer than it. Without the registration the check never runs."""
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 50.0})
    system = _gen(graph, bounds)
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    rows = [ln.strip() for ln in cpp.splitlines() if "register_min_input_period" in ln]
    assert rows == ['ros2_lf_binding::register_min_input_period("SubNode", "data", 50000000LL);'], (
        f"the declared min_input_period_ms of sub:data (50 ms) is never checked at run time "
        f": {rows}")


def test_declaration_of_a_node_in_another_federate_is_left_to_that_federate():
    """A federate's RID table holds its own nodes only (federated generation of
    brake_fusion r2 tinf refused here before). The subscriber's own federate
    registers the check."""
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 50.0})
    system = _gen(graph, bounds)
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    other = [row for row in table if row[1] != "SubNode"]
    assert other and len(other) < len(table)
    cpp = emit_rid_init_cpp(other, "pkg", corr_edges=system.corr_edges)
    assert "register_min_input_period" not in cpp


def test_declaration_without_its_subscription_row_refuses():
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 50.0})
    system = _gen(graph, bounds)
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    # SubNode stays in the table, its subscription is renamed away.
    no_sub = [tuple(row[:4]) + ("other",) + tuple(row[5:])
              if row[1] == "SubNode" and row[3] == "sub" else row for row in table]
    assert no_sub != table
    with pytest.raises(ValueError, match="no subscription of 'SubNode' to 'data'"):
        emit_rid_init_cpp(no_sub, "pkg", corr_edges=system.corr_edges)


def test_two_periods_for_one_input_refuse():
    """The binding checks one period per input, so two
    recorded values for one input refuse instead of one silently winning."""
    graph = _chain_graph()
    bounds = CorrBoundsConfig(burst={"sub": 1}, min_input_period_ms={("sub", "data"): 50.0})
    system = _gen(graph, bounds)
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    edge = next(e for e in system.corr_edges
                if (e.bound_inputs or {}).get("min_input_periods_ns"))
    twin = dataclasses.replace(edge, bound_inputs={"min_input_periods_ns": {"sub:data": 1}})
    with pytest.raises(ValueError, match="the binding checks one period per input"):
        emit_rid_init_cpp(table, "pkg", corr_edges=list(system.corr_edges) + [twin])


def test_no_declaration_registers_no_check():
    graph = _fixture_graph()
    system = _gen(graph, declared(graph, 1))
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    assert "register_min_input_period" not in cpp

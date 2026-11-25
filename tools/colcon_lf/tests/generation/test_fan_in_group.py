# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Fan-in groups: several publisher nodes into one subscription.

All edges into one ``(subscriber, callback)`` on one topic form a group that
is admitted or refused as a unit, since an admitted group's reader belongs to
ingress and a legacy take on it would steal the members' samples. A refused
member keeps its own reason and the others get ``fan_in_member_refused``;
the group-only refusals name the group rule that failed. Every assertion
names the edge it is about.
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.endpoints import endpoint_key_map
from colcon_lf.exec_config.connections import LinkDelay, LinkDelays
from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation.binding_header_emitter import collect_rid_table, emit_rid_init_cpp
from colcon_lf.generation.binding_key import build_binding_keys
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import _build_corr_edges
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    MessageFiltersInfo,
    MessageFiltersSyncInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)
from ..corr_declared import declared, generate_declared
from .test_correspondence_emission import MSG, _edge, _graph, _pub, _reactor_block, _sub, _tc
from .test_multi_node_fan_in import _accepted
from .test_shared_publisher_port import _lfc


def _rows(system, *pairs):
    """``(status, reason)`` of each ``(pub, sub)`` edge, keyed by ``pub->sub``."""
    return {f"{p}->{s}": (_edge(system, p, s).status, _edge(system, p, s).reason)
            for p, s in pairs}


def _fan_in(*nodes, pubs=("pa", "pb"), **gen):
    graph = _graph([*nodes, _sub("sub", "SubNode")], [_tc("/data", list(pubs), ["sub"])])
    return graph, generate_declared(graph, after_delay_ns=0, **gen)


def test_group_demoted_when_one_member_refused():
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB", durability="transient_local"))
    got = _rows(system, ("pa", "sub"), ("pb", "sub"))
    assert got == {"pa->sub": ("legacy", "fan_in_member_refused"),
                   "pb->sub": ("legacy", "transient_local")}, got
    note = _edge(system, "pa", "sub").note or ""
    assert "pb -> sub" in note and "transient_local" in note, f"edge pa->sub note: {note!r}"
    text = emit_lf_system(system)
    # A refused group keeps the stub path, one input per publisher node.
    assert "pa.data -> sub.data after 0" in text and "pb.data -> sub.data_fanin_1 after 0" in text
    assert "corr_e" not in text, text


def test_group_demoted_when_a_member_is_unsized():
    # pb declares no burst bound and has no proven one: the sizing loop
    # refuses it (burst_unbounded) and must take pa with it.
    graph = _graph([_pub("pa", "PubA"), _pub("pb", "PubB"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pa", "pb"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=0,
                               corr_bounds=CorrBoundsConfig(burst={"pa": 1, "sub": 1}))
    got = _rows(system, ("pa", "sub"), ("pb", "sub"))
    assert got == {"pa->sub": ("legacy", "fan_in_member_refused"),
                   "pb->sub": ("legacy", "burst_unbounded")}, got


def test_fan_in_rank_follows_instance_name_not_edge_order():
    # Listed pb first: pb gets edge 0 and leads, pa still ranks first.
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB"), pubs=("pb", "pa"))
    got = {p: (_edge(system, p, "sub").edge_id, _edge(system, p, "sub").group_lead,
               _edge(system, p, "sub").fan_in_rank) for p in ("pa", "pb")}
    assert got == {"pb": (0, 0, 1), "pa": (1, 0, 0)}, got


def test_unequal_delay_group_refused():
    delays = LinkDelays()
    for pub, ns in (("pa", 1_000_000), ("pb", 2_000_000)):
        delays.by_pair[("/data", pub, "sub")] = LinkDelay(
            ns=ns, literal=f"{ns // 1_000_000} msec", reason="test", source="", evidence="t")
    graph = _graph([_pub("pa", "PubA"), _pub("pb", "PubB"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pa", "pb"], ["sub"])])
    edges, _ = _build_corr_edges(
        graph.communication, graph.nodes, after_delay_ns=0, link_delays=delays,
        runtime_to_key=endpoint_key_map(graph), binding_keys=build_binding_keys(graph.nodes),
        bounds=declared(graph))
    got = {f"{e.pub_node}->{e.sub_node}": (e.status, e.reason) for e in edges}
    assert got == {"pa->sub": ("legacy", "fan_in_release_mismatch"),
                   "pb->sub": ("legacy", "fan_in_release_mismatch")}, got
    assert "pa 1000000 ns, pb 2000000 ns" in edges[0].note, edges[0].note
    # The refused group keeps the stub fan-in, which probe P17h rejects as on main.
    with pytest.raises(ValueError, match="probe P17h"):
        generate_declared(graph, after_delay_ns=0, link_delays=delays)


def _self_sub() -> NodeInfo:
    """Subscriber that also publishes /data from a timer in its one reactor."""
    return NodeInfo(
        name="sub", class_name="SubNode", package="test_pkg", source_file="/fake/sub.cpp",
        callbacks=[CallbackInfo(name="on_data"), CallbackInfo(name="tick", publishers_used=["data"])],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        subscribers=[SubscriberInfo(topic="data", message_type=MSG, callback="on_data")],
        publishers=[PublisherInfo(topic="data", message_type=MSG, created_in="constructor",
                                  unconditional=True)])


def test_self_member_refuses_group():
    graph = _graph([_pub("pa", "PubA"), _self_sub()], [_tc("/data", ["pa", "sub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=0)
    got = _rows(system, ("pa", "sub"), ("sub", "sub"))
    assert got == {"pa->sub": ("legacy", "fan_in_self_loop"),
                   "sub->sub": ("legacy", "fan_in_self_loop")}, got


def test_rebind_member_refuses_group():
    lifecycle = _pub("pa", "PubA", base_class="rclcpp_lifecycle::LifecycleNode")
    assert lifecycle.is_lifecycle
    _, system = _fan_in(lifecycle, _pub("pb", "PubB"))
    got = _rows(system, ("pa", "sub"), ("pb", "sub"))
    assert got == {"pa->sub": ("legacy", "fan_in_rebind"),
                   "pb->sub": ("legacy", "fan_in_rebind")}, got


def _relay(name: str, cls: str, topic: str, publishes: str) -> NodeInfo:
    return _sub(name, cls, topic=topic, publishes=publishes)


def _fan_in_loop_graph():
    """/y: s -> a, /x: a, b -> s, /z: s -> b, with relative publisher names.

    Both a -> s and b -> s close a node loop at zero delay. s's tick
    publishes y and z, its on_x only stores (a latch).
    """
    s = NodeInfo(
        name="s", class_name="SNode", package="test_pkg", source_file="/fake/s.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=["y", "z"]),
                   CallbackInfo(name="on_x")],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        subscribers=[SubscriberInfo(topic="x", message_type=MSG, callback="on_x")],
        publishers=[PublisherInfo(topic=t, message_type=MSG, created_in="constructor",
                                  unconditional=True) for t in ("y", "z")])
    return _graph([s, _relay("a", "ANode", "y", "x"), _relay("b", "BNode", "z", "x")],
                  [_tc("/y", ["s"], ["a"]), _tc("/x", ["a", "b"], ["s"]), _tc("/z", ["s"], ["b"])])


def test_zero_delay_loops_through_a_fan_in_group_are_admitted():
    # a -> s is a zero-delay loop but no causality loop, so it and b -> s
    # are admitted. s declares tick before its on_x latch
    # (timers_first, the default), so the latch reaches no output at its tag:
    # no causality loop, and every edge is admitted.
    system = generate_declared(_fan_in_loop_graph(), after_delay_ns=0)
    got = _rows(system, ("s", "a"), ("a", "s"), ("b", "s"), ("s", "b"))
    assert got == {"s->a": ("admitted", None), "a->s": ("admitted", None),
                   "b->s": ("admitted", None), "s->b": ("admitted", None)}, got


def test_latch_before_tick_in_a_fan_in_loop_is_a_causality_loop():
    # Under subscriptions_first s declares on_x before
    # tick, so LF orders tick after it and /x reaches /y and /z at its tag:
    # a causality loop, refused. The publishers are named relative ("y",
    # "z"); comparing the resolved topic hid this loop before the F2 fix.
    with pytest.raises(ValueError, match=r"zero-delay causality loop: "):
        generate_declared(_fan_in_loop_graph(), after_delay_ns=0,
                          reaction_order={"s": "subscriptions_first"})


def test_reader_depth_counts_member_burst_sum():
    # One invocation each of pa and pb fills 1 + 1 slots of a depth-1 reader.
    bounds = CorrBoundsConfig(burst={"pa": 1, "pb": 1, "sub": 1},
                              reader_depth={("sub", "data"): 1})
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB"), corr_bounds=bounds)
    got = _rows(system, ("pa", "sub"), ("pb", "sub"))
    assert got == {"pa->sub": ("legacy", "reader_depth_below_burst"),
                   "pb->sub": ("legacy", "reader_depth_below_burst")}, got
    # One member alone fits: the bound is the sum, not each member's burst.
    _, single = _fan_in(_pub("pa", "PubA"), pubs=("pa",), corr_bounds=bounds)
    assert _edge(single, "pa", "sub").status == "admitted"


def test_group_members_share_the_subscription_timeout():
    # pa relays /in, pb is a timer. Before the causal windows the relay chained its own T
    # onto pa -> sub, so the members had different T_e. T_e is now the
    # subscription's own take timeout, one value for every member of the
    # group. (the deferred variant of this local group is a
    # generation error, a local edge is decided at publish.)
    pa = _relay("pa", "PubA", "in", "data")
    graph = _graph([_pub("src", "Src", "in"), pa, _pub("pb", "PubB"), _sub("sub", "SubNode")],
                   [_tc("/in", ["src"], ["pa"]), _tc("/data", ["pa", "pb"], ["sub"])])
    bounds = CorrBoundsConfig(burst={n: 1 for n in graph.nodes})
    system = generate_declared(graph, after_delay_ns=0, corr_bounds=bounds)
    got = _rows(system, ("src", "pa"), ("pa", "sub"), ("pb", "sub"))
    assert got == {"src->pa": ("admitted", None),
                   "pa->sub": ("admitted", None),
                   "pb->sub": ("admitted", None)}, got
    timeouts = {_edge(system, p, "sub").timeout_ns for p in ("pa", "pb")}
    assert len(timeouts) == 1, f"the members' T_e differ: {timeouts}"
    deferred = CorrBoundsConfig(burst={n: 1 for n in graph.nodes},
                                deferred_join={("sub", "data")})
    with pytest.raises(ValueError, match=r"local edge .* is decided at publish: "):
        generate_declared(graph, after_delay_ns=0, corr_bounds=deferred)


def _bna_fuse() -> NodeInfo:
    sync = MessageFiltersSyncInfo(
        member_variable="sync_", policy="BoundedNearestToAnchor", queue_size=None,
        inputs=["sub_a_", "sub_b_"], callback="cb",
        bna_params={"delta": 5_000_000, "lateness0": 20_000_000, "lateness1": 0,
                    "capacity0": 4, "capacity1": 8, "retention": 50_000_000,
                    "timeout": 100_000_000})
    return NodeInfo(
        name="fuse", class_name="FuseNode", package="test_pkg", source_file="/fake/fuse.cpp",
        callbacks=[CallbackInfo(name="cb_via_sub_a"), CallbackInfo(name="cb_via_sub_b")],
        subscribers=[SubscriberInfo(topic="a", message_type=MSG, callback="cb_via_sub_a"),
                     SubscriberInfo(topic="b", message_type=MSG, callback="cb_via_sub_b")],
        message_filters=MessageFiltersInfo(synchronizers=[sync]))


def test_fan_in_into_bna_input_names_its_reason():
    graph = _graph([_pub("pa", "PaNode", "a"), _pub("pa2", "Pa2Node", "a"),
                    _pub("pb", "PbNode", "b"), _bna_fuse()],
                   [_tc("/a", ["pa", "pa2"], ["fuse"]), _tc("/b", ["pb"], ["fuse"])])
    bounds = CorrBoundsConfig(burst={n: 1 for n in graph.nodes}, stamp_provenance={
        ("fuse", "a"): "lf_clock", ("fuse", "b"): "lf_clock"})
    with pytest.raises(ValueError, match=(
            "RCLLF-H003 bna_input_legacy:fan_in_bna_input: BoundedNearestToAnchor input sub_a_")):
        generate_declared(graph, after_delay_ns=0, corr_bounds=bounds)
    # One publisher per input is admitted as before.
    graph.communication.topics[0].publishers = ["pa"]
    system = generate_declared(graph, after_delay_ns=0, corr_bounds=bounds)
    assert _edge(system, "pa", "fuse").status == "admitted"


def test_repeat_listing_of_a_refused_group_stays_legacy():
    # A node listed twice (the shape) must not be admitted alone on its
    # second listing once its group is refused: the reader is shared.
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB", durability="transient_local"),
                        pubs=("pa", "pb", "pa"))
    rows = [(e.pub_node, e.status, e.reason) for e in system.corr_edges]
    assert rows == [("pa", "legacy", "fan_in_member_refused"),
                    ("pb", "legacy", "transient_local"),
                    ("pa", "legacy", "fan_in_member_refused")], rows


def test_refused_group_leaves_no_path_for_later_edges():
    # A group is admitted all or none, so a refused group
    # puts no member into the zero-delay graph. /x: a, b -> s is refused (b
    # is transient_local); /y: s -> a comes later and closes no cycle.
    s = NodeInfo(
        name="s", class_name="SNode", package="test_pkg", source_file="/fake/s.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=["y"]), CallbackInfo(name="on_x")],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        subscribers=[SubscriberInfo(topic="x", message_type=MSG, callback="on_x")],
        publishers=[PublisherInfo(topic="y", message_type=MSG, created_in="constructor",
                                  unconditional=True)])
    graph = _graph([s, _relay("a", "ANode", "y", "x"),
                    _pub("b", "BNode", topic="x", durability="transient_local")],
                   [_tc("/x", ["a", "b"], ["s"]), _tc("/y", ["s"], ["a"])])
    system = generate_declared(graph, after_delay_ns=0)
    got = _rows(system, ("a", "s"), ("b", "s"), ("s", "a"))
    assert got == {"a->s": ("legacy", "fan_in_member_refused"),
                   "b->s": ("legacy", "transient_local"),
                   "s->a": ("admitted", None)}, got


def test_shared_publisher_instance_refuses_group():
    # Node pa publishes /data from two publisher objects (listed
    # twice, the shape). Its two writers would share pa's rank, so the
    # group of pa and pb is refused as a whole, with the group reason.
    pa = _pub("pa", "PubA")
    pa.publishers.append(PublisherInfo(topic="data", message_type=MSG, created_in="constructor",
                                       unconditional=True))
    _, system = _fan_in(pa, _pub("pb", "PubB"), pubs=("pa", "pb", "pa"))
    rows = [(e.pub_node, e.status, e.reason) for e in system.corr_edges]
    assert rows == [("pa", "legacy", "fan_in_shared_publisher"),
                    ("pb", "legacy", "fan_in_shared_publisher"),
                    ("pa", "legacy", "ambiguous_endpoint")], rows
    note = _edge(system, "pb", "sub").note or ""
    assert "pa publish(es) on /data from two publisher objects" in note, f"pb->sub note: {note!r}"
    assert "corr_e" not in emit_lf_system(system)


def test_fan_in_group_emits_one_drain_triggered_by_every_member():
    # The subscriber declares every member port and one
    # drain reaction triggered by all of them, so LF runs it after every
    # member publisher at (t, m) and every record is filed before the first
    # pop; one close per member.
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB"))
    text = emit_lf_system(system)
    group = _reactor_block(text, "SubNode_DefaultGroup")
    outer = _reactor_block(text, "SubNode")
    assert group.count("logical action corr_drain_e") == 1, group
    assert group.count("ros2_lf_corr_drain(") == 1, group
    assert "reaction(corr_drain_e0, corr_e0, corr_e1) rcllf_bound -> corr_drain_e0 {=" in group, group
    for e in (0, 1):
        assert f"input corr_e{e}: rcllf_corr_t" in group, f"edge {e}:\n{group}"
        assert f"corr_e{e} -> default_group.corr_e{e}" in outer, f"edge {e}:\n{outer}"
        assert f"ros2_lf_corr_close({e});" in group, f"edge {e}:\n{group}"
        assert f"reaction(corr_e{e}) " not in group, f"edge {e}: separate arrive\n{group}"
        arrive = (f"if (corr_e{e}->is_present) {{ int64_t a = "
                  f"ros2_lf_corr_arrive(self, {e}, corr_e{e}->value.count); "
                  f"if (a >= 0) {{ w = a; }} }}")
        assert arrive in group, f"edge {e}:\n{group}"
    drain = 'ros2_lf_corr_drain(self, 0, RID_SubNode_on_data, "data");'
    assert (group.index("ros2_lf_corr_arrive(self, 0,") < group.index("ros2_lf_corr_arrive(self, 1,")
            < group.index("if (w == 0 || corr_drain_e0->is_present)") < group.index(drain)), group
    # Both members are dispatched by the one drain, so they share its RID.
    rids = {p: _edge(system, p, "sub").sub_rid for p in ("pa", "pb")}
    assert rids["pa"] == rids["pb"] >= 0, rids


def test_fan_in_split_escape_emits_n_arrives_then_one_drain(monkeypatch):
    # RCLLF_LOCAL_SPLIT_DRAIN=1 restores the earlier shape.
    monkeypatch.setenv("RCLLF_LOCAL_SPLIT_DRAIN", "1")
    _, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB"))
    group = _reactor_block(emit_lf_system(system), "SubNode_DefaultGroup")
    for e in (0, 1):
        arrive = (f"reaction(corr_e{e}) rcllf_bound -> corr_drain_e0 {{=\n"
                  f"        int64_t w = ros2_lf_corr_arrive(self, {e}, corr_e{e}->value.count);\n"
                  f"        w = ros2_lf_corr_claim_wake(0, w);\n"
                  f"        if (w >= 0 && lf_schedule(corr_drain_e0, w) <= 0) "
                  f"{{ ros2_lf_corr_schedule_refused(0); }}")
        assert arrive in group, f"edge {e}:\n{group}"
    drain = 'ros2_lf_corr_drain(self, 0, RID_SubNode_on_data, "data");'
    assert "reaction(corr_drain_e0) rcllf_bound -> corr_drain_e0 {=" in group, group
    # The drain is declared first and the split arrives last.
    assert (group.index("reaction(corr_drain_e0)") < group.index(drain)
            < group.index("reaction(corr_e0)") < group.index("reaction(corr_e1)")), group


def _rid_init(graph, system) -> list[str]:
    """The lines of the generated binding header's registration function."""
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    return emit_rid_init_cpp(rid, "pkg", corr_edges=system.corr_edges).splitlines()


def test_binding_header_registers_each_member_after_its_edge():
    # Every member of a group of two or more names its lead and
    # rank right after its own registration (the runtime reads them at
    # startup); a lone edge gets no call, so its header stays as on main.
    graph, system = _fan_in(_pub("pa", "PubA"), _pub("pb", "PubB"), pubs=("pb", "pa"))
    rows = [ln.split("(")[0].strip() + "(" + ln.split("(")[1].split(",")[0].lstrip("{")
            + (ln[ln.index(","):] if "fan_in" in ln else "")
            for ln in _rid_init(graph, system)
            if "register_corr_edge(" in ln or "set_corr_edge_fan_in(" in ln]
    assert rows == ["ros2_lf_binding::register_corr_edge(0",
                    "ros2_lf_binding::set_corr_edge_fan_in(0, 0, 1u);",
                    "ros2_lf_binding::register_corr_edge(1",
                    "ros2_lf_binding::set_corr_edge_fan_in(1, 0, 0u);"], (
        "edges pb->sub (0, rank 1) and pa->sub (1, rank 0) must each register their "
        f"group after their own row: {rows}")
    graph, lone = _fan_in(_pub("pa", "PubA"), pubs=("pa",))
    calls = [ln for ln in _rid_init(graph, lone) if "set_corr_edge_fan_in" in ln]
    assert calls == [], f"edge pa->sub is a group of one, its header must not change: {calls}"


def _gg_graph():
    # Same-class fan-in experiment: one publisher class launched twice
    # into one subscriber, every link after 5 ms.
    return _graph([_pub("gg_pub_a", "GgPublisher", "cmd"), _pub("gg_pub_b", "GgPublisher", "cmd"),
                   _sub("gg_sub", "GgSubscriber", "cmd")],
                  [_tc("/cmd", ["gg_pub_a", "gg_pub_b"], ["gg_sub"])])


def test_lfc_accepts_gg_one_process(tmp_path):
    system = generate_declared(_gg_graph(), after_delay_ns=5_000_000)
    rows = [(e.pub_node, e.status, e.edge_id, e.group_lead, e.fan_in_rank)
            for e in system.corr_edges]
    assert rows == [("gg_pub_a", "admitted", 0, 0, 0),
                    ("gg_pub_b", "admitted", 1, 0, 1)], rows
    text = emit_lf_system(system)
    _accepted(_lfc(tmp_path, text), text)

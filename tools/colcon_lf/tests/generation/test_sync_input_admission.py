# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""ExactTime/ApproximateTime inputs on the mf-sync/1 domain (S3).

The stock policies pair in arrival order, so a converted synchronizer is
deterministic only when both inputs are admitted correspondence edges with one
link delay. A legacy input arrives in wall-clock order and used to
generate silently; it now refuses with ``RCLLF-H003 sync_input_legacy:<reason>``
(the BNA rule, ``bna_input_legacy``), unequal delays with
``sync_input_delay_unequal``. The input drains are declared input 0 first,
which is the same-tag arrival order the oracle domain assumes (plan risk 2).
"""

from __future__ import annotations

import re

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    MessageFiltersInfo,
    MessageFiltersSyncInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)
from ..corr_declared import generate_declared

MSG = "custom_interfaces::msg::SyncSample"


def _pub(name: str, cls: str, topic: str, durability: str | None = None) -> NodeInfo:
    return NodeInfo(
        name=name, class_name=cls, package="p", source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=[topic])],
        timers=[TimerInfo(period_ms=50.0, callback="tick")],
        publishers=[PublisherInfo(topic=topic, message_type=MSG, qos_durability=durability,
                                  created_in="constructor", unconditional=True)],
    )


def _sync(policy: str = "ExactTime") -> NodeInfo:
    sync = MessageFiltersSyncInfo(member_variable="sync_", policy=policy, queue_size=3,
                                  inputs=["sub_a_", "sub_b_"], callback="cb")
    return NodeInfo(
        name="join", class_name="Join", package="p", source_file="/fake/join.cpp",
        callbacks=[CallbackInfo(name="cb_via_sub_a"), CallbackInfo(name="cb_via_sub_b")],
        subscribers=[SubscriberInfo(topic="in0", message_type=MSG, callback="cb_via_sub_a"),
                     SubscriberInfo(topic="in1", message_type=MSG, callback="cb_via_sub_b")],
        message_filters=MessageFiltersInfo(synchronizers=[sync]),
    )


def _generate(durability: str | None = None, delays=(None, None), policy="ExactTime"):
    nodes = [_pub("src_a", "SrcA", "in0", durability), _pub("src_b", "SrcB", "in1"),
             _sync(policy)]
    topics = [TopicConnection(topic=t, message_type=MSG, publishers=[p], subscribers=["join"],
                              after_delay_ns=d)
              for t, p, d in (("in0", "src_a", delays[0]), ("in1", "src_b", delays[1]))]
    graph = SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="/fake/b.launch.py",
                                                     package="p"), all_nodes=[]),
        nodes={n.name: n for n in nodes},
        communication=SystemCommunication(topics=topics, services=[]),
    )
    return generate_declared(graph, after_delay_ns=0)


@pytest.mark.parametrize("policy", ["ExactTime", "ApproximateTime"])
def test_legacy_sync_input_refuses(policy):
    # Old code: generation succeeds and the input runs in wall-clock order.
    with pytest.raises(ValueError, match=(
            rf"RCLLF-H003 sync_input_legacy:transient_local: {policy} input sub_a_ is not an "
            r"admitted correspondence edge")):
        _generate(durability="transient_local", policy=policy)


def test_unequal_input_delays_refuse():
    with pytest.raises(ValueError, match=(
            r"RCLLF-H003 sync_input_delay_unequal: ExactTime inputs have different link "
            r"delays \(sub_a_ 0 ns, sub_b_ 1000000 ns\)")):
        _generate(delays=(0, 1_000_000))


def _drain_order(text: str) -> list[str]:
    start = text.index("reactor Join_DefaultGroup")
    block = text[start:text.index("\n}\n", start)]
    return re.findall(r"ros2_lf_corr_drain\(self, \d+, RID_\w+?_(cb_via_sub_[ab])", block)


def test_admitted_inputs_generate_with_input_zero_drained_first():
    system = _generate(delays=(0, 0))
    edges = [e for e in system.corr_edges if e.sub_node == "join"]
    assert [(e.status, e.delay_ns) for e in edges] == [("admitted", 0), ("admitted", 0)], edges
    assert _drain_order(emit_lf_system(system)) == ["cb_via_sub_a", "cb_via_sub_b"]


def test_reverse_drains_hook_swaps_the_declaration_order(monkeypatch):
    """``RCLLF_TEST_SYNC_REVERSE_DRAINS=1``: the converted-side falsification of
    the arrival-order check (plan output verification 3)."""
    monkeypatch.setenv("RCLLF_TEST_SYNC_REVERSE_DRAINS", "1")
    assert _drain_order(emit_lf_system(_generate(delays=(0, 0)))) == [
        "cb_via_sub_b", "cb_via_sub_a"]


def test_resolved_model_records_the_domain_and_input_delays():
    """``node:<fqn>#sync.message_filters`` carries policy, q, the inputs with
    their link delay and the library pins of mf-sync/1 (plan S3)."""
    from colcon_lf.resolved_model.model import PROVENANCE_EXTRACTED, Record
    from colcon_lf.resolved_model.resolve import _sync_records

    def link(src, topic, ns):
        return Record(contract_id=f"link:{src}->join/{topic}#after_delay", property="after_delay",
                      subject={"kind": "link", "link": "topic", "src": src, "dst": "join",
                               "name": topic},
                      value=ns, unit="ns", provenance=PROVENANCE_EXTRACTED, evidence="t")

    node = _sync()
    [rec] = _sync_records("join", node, node.message_filters,
                          [link("src_a", "in0", 0), link("src_b", "in1", 0)])
    assert rec.contract_id == "node:join#sync.message_filters"
    assert rec.value == {
        "domain": "mf-sync/1", "member": "sync_", "policy": "ExactTime", "queue_size": 3,
        "callback": "cb",
        "inputs": [{"member": "sub_a_", "topic": "in0", "delay_ns": 0},
                   {"member": "sub_b_", "topic": "in1", "delay_ns": 0}],
        "library_pins": {"message_filters": "4.11.17", "rclcpp": "28.1.22"},
    }
    # An input no link of the system feeds has no delay (generation refuses it).
    [rec] = _sync_records("join", node, node.message_filters, [link("src_a", "in0", 0)])
    assert rec.value["inputs"][1]["delay_ns"] is None


def test_inputs_declared_in_reverse_source_order_still_drain_input_zero_first(monkeypatch):
    """The subscriptions' source order is not the synchronizer's input order.

    Reactions follow ``node_info.callbacks``; a node that creates ``sub_b_``
    before ``sub_a_`` used to declare input 1's drain first, so at one tag
    input 1's samples reached the policy first, against the mf-sync/1 arrival
    order (tick, input, publish order) the oracle feeds.
    """
    real = _sync

    def flipped(policy="ExactTime"):
        node = real(policy)
        node.callbacks.reverse()
        assert [c.name for c in node.callbacks] == ["cb_via_sub_b", "cb_via_sub_a"]
        return node

    monkeypatch.setitem(globals(), "_sync", flipped)
    system = _generate(delays=(0, 0))
    assert _drain_order(emit_lf_system(system)) == ["cb_via_sub_a", "cb_via_sub_b"], (
        "input 1's drain is declared before input 0's")


def _corr_edge(cb: str, topic: str, pub: str, delay_ns: int, edge_id: int | None,
               reason: str | None = None):
    from colcon_lf.generation.lf_model import CorrEdge
    return CorrEdge(topic=topic, pub_node=pub, sub_node="join", pub_key=pub, sub_key="join",
                    delay_ns=delay_ns, status="admitted" if edge_id is not None else "legacy",
                    reason=reason, edge_id=edge_id, pub_topic=topic, sub_topic=topic,
                    pub_callbacks=["tick"], sub_callback=cb, role="local")


def test_fan_in_input_with_mixed_delays_refuses():
    """Two admitted edges into input 0 with delays 0 and 1 ms, input 1 at 1 ms.

    The old per-callback delay map kept the last edge's d (1 ms), so both
    inputs looked equal and the node generated with input 0's samples landing
    at two different offsets."""
    from colcon_lf.generation import lf_generator

    corr = lf_generator._node_corr_from_edges([
        _corr_edge("cb_via_sub_a", "in0", "src_a", 0, 0),
        _corr_edge("cb_via_sub_a", "in0", "src_c", 1_000_000, 1),
        _corr_edge("cb_via_sub_b", "in1", "src_b", 1_000_000, 2),
    ])["join"]
    with pytest.raises(ValueError, match=(
            r"RCLLF-H003 sync_input_delay_unequal: ExactTime inputs have different link "
            r"delays \(sub_a_ 0/1000000 ns, sub_b_ 1000000 ns\)")):
        lf_generator._check_sync_inputs(_sync(), corr)


def test_legacy_edge_next_to_an_admitted_one_refuses():
    """Input 0 has an admitted edge and a legacy one (``transient_local``).

    The old check only asked whether the callback had any admitted edge, so the
    legacy publisher's samples reached the policy in wall-clock order."""
    from colcon_lf.generation import lf_generator

    corr = lf_generator._node_corr_from_edges([
        _corr_edge("cb_via_sub_a", "in0", "src_a", 0, 0),
        _corr_edge("cb_via_sub_a", "in0", "src_c", 0, None, reason="transient_local"),
        _corr_edge("cb_via_sub_b", "in1", "src_b", 0, 1),
    ])["join"]
    with pytest.raises(ValueError, match=(
            r"RCLLF-H003 sync_input_legacy:transient_local: ExactTime input sub_a_ is not an "
            r"admitted correspondence edge")):
        lf_generator._check_sync_inputs(_sync(), corr)


def test_legacy_twin_subscription_on_the_topic_does_not_refuse_an_admitted_input():
    """A second subscription of the node on ``in0`` is legacy (no callback on
    the edge, so its reason is kept per topic); the sync input's own edge is
    admitted, and a fan-in group is admitted or refused as a unit."""
    from colcon_lf.generation import lf_generator

    corr = lf_generator._node_corr_from_edges([
        _corr_edge("cb_via_sub_a", "in0", "src_a", 0, 0),
        _corr_edge("", "in0", "src_a", 0, None, reason="subscriber_callback"),
        _corr_edge("cb_via_sub_b", "in1", "src_b", 0, 1),
    ])["join"]
    assert corr.legacy_topic_reasons == {"in0": "subscriber_callback"}
    lf_generator._check_sync_inputs(_sync(), corr)

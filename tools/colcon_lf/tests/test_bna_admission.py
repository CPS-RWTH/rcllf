# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""BoundedNearestToAnchor admission and generation.

The scan admits the policy with literal parameters and refuses every other
form with the plan's reason slugs. The generator adds the declare, wake and
close reactions to the reactor holding both input drains and refuses a
split or unadmitted input and a missing stamp provenance.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.exec_config.policies import REGISTRY, KEY_STAMP_PROVENANCE
from colcon_lf.generation import lf_generator as g
from colcon_lf.generation.lf_model import LFReaction
from colcon_lf.node_analysis.message_filters_scan import bna_literal_params, scan
from colcon_lf.node_analysis.models import (
    MessageFiltersInfo,
    MessageFiltersSyncInfo,
    NodeInfo,
    SubscriberInfo,
)

HEADER = """
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "rcllf_message_filters/bna_policy.hpp"
class Fuse : public rclcpp::Node {
  using Policy = rcllf_message_filters::sync_policies::BoundedNearestToAnchor<M, M>;
  void cb(const M::ConstSharedPtr & a, const M::ConstSharedPtr & b);
  message_filters::Subscriber<M> sub_a_;
  message_filters::Subscriber<M> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
};
"""

CPP = """
Fuse::Fuse() : Node("fuse") {
  rclcpp::SubscriptionOptions opts;
  sub_a_.subscribe(this, "in/a", rmw_qos_profile_default, opts);
  sub_b_.subscribe(this, "in/b");
  sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
    Policy(5'000'000, 20 * 1000000, 0, 4, 8, 50000000LL, (100 * 1000000)), sub_a_, sub_b_);
  sync_->registerCallback(std::bind(&Fuse::cb, this, std::placeholders::_1,
                                    std::placeholders::_2));
}
"""

PARAMS = {"delta": 5_000_000, "lateness0": 20_000_000, "lateness1": 0, "capacity0": 4,
          "capacity1": 8, "retention": 50_000_000, "timeout": 100_000_000}


def test_bna_admitted_with_literal_params_and_options_form():
    r = scan([CPP, HEADER], "Fuse")
    assert r.info.unsupported == []
    [s] = r.info.synchronizers
    assert (s.policy, s.inputs, s.callback, s.bna_params) == (
        "BoundedNearestToAnchor", ["sub_a_", "sub_b_"], "cb", PARAMS)
    assert [(i.topic, i.callback, i.qos_args) for i in r.inputs] == [
        ("in/a", "cb_via_sub_a", "rmw_qos_profile_default"), ("in/b", "cb_via_sub_b", None)]


@pytest.mark.parametrize("edit, reason", [
    (("50000000LL", "retention_ns_"), "bna_params_not_literal: synchronizer sync_"),
    (("0, 4, 8", "0, 0, 8"), "bna_params_not_literal"),  # capacity0 < 1
    (("0, 4, 8, 50000000LL, ", "0, 4, 8, "), "bna_params_not_literal"),  # six args
    (("BoundedNearestToAnchor<M, M>", "BoundedNearestToAnchor<M, M, M>"),
     "input_count: synchronizer sync_ has 3 inputs"),
])
def test_bna_refusals(edit, reason):
    old, new = edit
    src = [CPP.replace(old, new), HEADER.replace(old, new)]
    assert src != [CPP, HEADER], "edit did not apply"
    r = scan(src, "Fuse")
    assert r.info.synchronizers == []
    assert any(reason in u for u in r.info.unsupported), r.info.unsupported


def test_bna_multiple_refused():
    header = HEADER.replace(
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;",
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;\n"
        "  message_filters::Subscriber<M> sub_c_;\n  message_filters::Subscriber<M> sub_d_;\n"
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync2_;")
    cpp = CPP + """
void Fuse::more() {
  sub_c_.subscribe(this, "in/c");
  sub_d_.subscribe(this, "in/d");
  sync2_ = std::make_shared<message_filters::Synchronizer<Policy>>(
    Policy(1, 1, 1, 1, 1, 1, 1), sub_c_, sub_d_);
  sync2_->registerCallback(std::bind(&Fuse::cb, this, std::placeholders::_1,
                                     std::placeholders::_2));
}
"""
    r = scan([cpp, header], "Fuse")
    assert any(u.startswith("bna_multiple: synchronizer sync2_") for u in r.info.unsupported), \
        r.info.unsupported


def _stock(cpp: str):
    header = HEADER.replace(
        "rcllf_message_filters::sync_policies::BoundedNearestToAnchor",
        "message_filters::sync_policies::ApproximateTime")
    cpp = cpp.replace("Policy(5'000'000, 20 * 1000000, 0, 4, 8, 50000000LL, (100 * 1000000))",
                      "Policy(7)")
    return scan([cpp, header], "Fuse")


def test_four_arg_subscribe_is_accepted_for_stock_policies():
    # Probe P20: ApproximateTime inputs may pass SubscriptionOptions
    # too, so the node can register its message_lost handler.
    r = _stock(CPP)
    assert not any("must be subscribed as" in u for u in r.info.unsupported), r.info.unsupported
    assert [i.topic for i in r.inputs] == ["in/a", "in/b"]


def test_five_arg_subscribe_stays_refused_for_stock_policies():
    r = _stock(CPP.replace('"in/a", rmw_qos_profile_default, opts)',
                           '"in/a", rmw_qos_profile_default, opts, extra)'))
    assert any('subscribe(this, "topic"[, qos[, options]])' in u for u in r.info.unsupported)


def test_literal_params_parser():
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 7)") == dict(
        zip(PARAMS, [1, 2, 3, 4, 5, 6, 7]))
    assert bna_literal_params("P(1, 2, 3, 4, 5, 0, 7)") is None  # retention 0
    assert bna_literal_params("P(-1, 2, 3, 4, 5, 6, 7)") is None  # not a literal
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 99999999999999999999)") is None


def test_literal_params_follow_cpp_integer_types():
    # int * int overflows in C++ although the product fits int64.
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 3000 * 1000000)") is None
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 3000u * 2000000u)") is None  # wraps
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 010)") is None  # octal
    for expr in ("3000LL * 1000000", "3000 * 1000000LL", "3000000000", "3000L * 1'000'000"):
        assert bna_literal_params(f"P(1, 2, 3, 4, 5, 6, {expr})")["timeout"] == 3_000_000_000
    assert bna_literal_params("P(1, 2, 3, 4, 5, 6, 2147483647 * 1)")["timeout"] == 2147483647


# --- generator -------------------------------------------------------------

KEY = "fuse"


def _node(group_a: str = "default_group", group_b: str = "default_group") -> NodeInfo:
    sync = MessageFiltersSyncInfo(
        member_variable="sync_", policy="BoundedNearestToAnchor", queue_size=None,
        inputs=["sub_a_", "sub_b_"], callback="cb", bna_params=dict(PARAMS))
    return NodeInfo(
        name="fuse", class_name="Fuse", package="p", source_file=Path("f.cpp"),
        subscribers=[SubscriberInfo(topic="in/a", message_type="M", callback="cb_via_sub_a",
                                    callback_group=group_a),
                     SubscriberInfo(topic="in/b", message_type="M", callback="cb_via_sub_b",
                                    callback_group=group_b)],
        message_filters=MessageFiltersInfo(synchronizers=[sync]))


def _drain(cb: str, edge: int) -> LFReaction:
    return LFReaction(
        triggers=[g.corr_drain_action_name(edge)], effects=[g.corr_drain_action_name(edge)],
        body=f'int64_t w = ros2_lf_corr_drain(self, {edge}, RID, "t");\n'
        f"{g._corr_resched_line(edge)}",
        kind="sub", rid_name=cb, corr_drain_edge=edge)


def _corr(**kw) -> g._NodeCorr:
    nc = g._NodeCorr(**kw)
    return g._with_bna_inputs(_node(), nc)


def test_bna_reactions_wake_after_drains_declare_first_close_last():
    corr = _corr(sub_edges={"cb_via_sub_a": 0, "cb_via_sub_b": 1})
    timer = LFReaction(triggers=["t"], effects=[], body="x", kind="timer")
    out = g._add_bna_reactions([_drain("cb_via_sub_a", 0), _drain("cb_via_sub_b", 1), timer],
                               corr, KEY)
    assert [r.kind for r in out] == ["bna_declare", "sub", "sub", "bna_wake", "timer",
                                     "bna_close"]
    assert out[0].triggers == ["startup"]
    assert out[0].body == ('ros2_lf_bna_declare_input("fuse", RID_fuse_cb_via_sub_a, 0);\n'
                           'ros2_lf_bna_declare_input("fuse", RID_fuse_cb_via_sub_b, 1);')
    loop = ('{ int64_t bw; while ((bw = ros2_lf_bna_take_wake("fuse")) >= 0) '
            "{ lf_schedule(bna_wake, bw); } }")
    for drain, edge in ((out[1], 0), (out[2], 1)):
        lines = drain.body.split("\n")
        assert lines[-2:] == [loop, g._corr_resched_line(edge)]
        assert "bna_wake" in drain.effects
    assert out[3].triggers == ["bna_wake"] and out[3].effects == ["bna_wake"]
    assert out[3].body == f'ros2_lf_bna_wake("fuse");\n{loop}'
    assert out[5].triggers == ["shutdown"] and out[5].body == 'ros2_lf_bna_close("fuse");'
    assert g._corr_logical_actions(out)[-1] == "bna_wake"
    assert {"bna_declare", "bna_wake", "bna_close"} <= g.CORR_NO_RID_KINDS


def test_bna_split_refused():
    corr = _corr(sub_edges={"cb_via_sub_a": 0, "cb_via_sub_b": 1})
    with pytest.raises(ValueError, match="RCLLF-H003 bna_split"):
        g._add_bna_reactions([_drain("cb_via_sub_a", 0)], corr, KEY)


def test_reactor_without_bna_inputs_is_unchanged():
    corr = _corr(sub_edges={"cb_via_sub_a": 0, "cb_via_sub_b": 1})
    rs = [_drain("other", 3)]
    assert g._add_bna_reactions(rs, corr, KEY) == rs


@pytest.mark.parametrize("kw, reason", [
    ({"sub_edges": {"cb_via_sub_a": 0}, "legacy_reasons": {"cb_via_sub_b": "burst_unbounded"}},
     "bna_input_legacy:burst_unbounded"),
    ({"sub_edges": {"cb_via_sub_a": 0}, "external_subs": {"cb_via_sub_b": "ext"}},
     "bna_input_legacy:external"),
    ({"sub_edges": {"cb_via_sub_a": 0}}, "bna_input_legacy:no_edge"),
])
def test_bna_input_legacy_refused(kw, reason):
    with pytest.raises(ValueError, match=f"RCLLF-H003 {reason}: BoundedNearestToAnchor "
                                         "input sub_b_"):
        _corr(**kw)


def test_reentrant_split_refused_through_build_reactor():
    """Inputs in a Reentrant group land in two sub-reactors (bna_split)."""
    from colcon_lf.node_analysis.models import CallbackGroupInfo, CallbackInfo

    node = _node("g", "g")
    node.callbacks = [CallbackInfo(name="cb_via_sub_a"),
                      CallbackInfo(name="cb_via_sub_b")]
    node.callback_groups = [CallbackGroupInfo(name="g", type="Reentrant")]
    corr = g._NodeCorr(sub_edges={"cb_via_sub_a": 0, "cb_via_sub_b": 1})
    with pytest.raises(ValueError, match="bna_split"):
        g._build_reactor(node, binding_key=KEY, corr=corr)


def test_ingress_subscription_is_triggered_by_its_xcorr_port():
    """A cross-federate ingress edge triggers its subscription through the
    ``xcorr_e`` input port. It used to name the publisher-side flush action, so
    a grouped subscriber federate referenced a port it never declared and lfc
    failed (found with the federated bna_example)."""
    corr = g._NodeCorr(sub_edges={"in": 0, "remote": 1, "self": 2}, remote={1}, local={2})
    assert g._corr_sub_triggers(corr) == {
        "in": g.corr_port_name(0), "remote": g.xcorr_port_name(1),
        "self": g.corr_drain_action_name(2)}


def test_federated_naming_pass_skips_the_bna_input_check():
    """The whole-system pass of a federated build admits no edges and only
    names the system, the federate passes check the BNA inputs."""
    with pytest.raises(ValueError, match="bna_input_legacy:no_edge"):
        g._build_reactor(_node(), binding_key=KEY)
    reactor, _ = g._build_reactor(_node(), binding_key=KEY, check_bna=False)
    assert not any(r.kind.startswith("bna_") for r in reactor.reactions)


def test_stamp_provenance_required():
    from colcon_lf.exec_config.resolve import CorrBoundsConfig

    class Graph:
        nodes = {KEY: _node()}

    with pytest.raises(ValueError, match="'in/a' has no stamp_provenance"):
        g._check_bna_stamp_provenance(Graph, CorrBoundsConfig())
    with pytest.raises(ValueError, match="'in/b' has no stamp_provenance"):
        g._check_bna_stamp_provenance(
            Graph, CorrBoundsConfig(stamp_provenance={(KEY, "in/a"): "lf_clock"}))
    g._check_bna_stamp_provenance(Graph, CorrBoundsConfig(stamp_provenance={
        (KEY, "in/a"): "lf_clock", (KEY, "in/b"): "declared:camera driver"}))


def test_stamp_provenance_parse():
    parse = REGISTRY[KEY_STAMP_PROVENANCE].parse
    assert parse("lf_clock") == "lf_clock"
    assert parse("declared:lidar driver") == "declared:lidar driver"
    for bad in ("declared:", "declared:  ", "wall", 3):
        with pytest.raises(ValueError):
            parse(bad)
    assert REGISTRY[KEY_STAMP_PROVENANCE].runtime is False


def test_bna_input_refused_before_subscriber_matching_names_its_reason():
    """A BNA input whose edge is refused before subscriber matching (here the
    publishers are transient_local) is reported with that reason. The edge's
    ``sub_callback`` is set only on a successful match, so the check used to
    say ``bna_input_legacy:no_edge`` and lose the real reason (batch BK)."""
    from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
    from colcon_lf.node_analysis.models import CallbackInfo, PublisherInfo, TimerInfo

    def pub(name: str, topic: str, durability: str | None) -> NodeInfo:
        return NodeInfo(
            name=name, class_name=name.title(), package="p", source_file=Path(f"{name}.cpp"),
            publishers=[PublisherInfo(topic=topic, message_type="M", qos_durability=durability)],
            timers=[TimerInfo(period_ms=10.0, callback="tick")],
            callbacks=[CallbackInfo(name="tick", publishers_used=[topic])])

    fuse = _node()
    fuse.callbacks = [CallbackInfo(name="cb_via_sub_a"), CallbackInfo(name="cb_via_sub_b")]
    nodes = {"pa": pub("pa", "/in/a", "transient_local"),
             "pb": pub("pb", "/in/b", "transient_local"),
             "fuse": fuse}
    comm = SystemCommunication(topics=[TopicConnection("/in/a", "M", ["pa"], ["fuse"]),
                                       TopicConnection("/in/b", "M", ["pb"], ["fuse"])])
    edges, corr = g._build_corr_edges(comm, nodes, after_delay_ns=0, runtime_to_key={},
                                      binding_keys={k: k for k in nodes})
    reasons = {e.topic: (e.status, e.reason, e.sub_callback) for e in edges}
    assert reasons["/in/a"] == ("legacy", "transient_local", ""), reasons
    with pytest.raises(ValueError, match="RCLLF-H003 bna_input_legacy:transient_local: "
                                         "BoundedNearestToAnchor input sub_a_"):
        g._with_bna_inputs(fuse, corr.get("fuse", g._NO_CORR))  # as _build_lf_system


def test_max_stamp_lag_parse():
    """A time literal >= 0, kept as written."""
    from colcon_lf.exec_config.policies import KEY_MAX_STAMP_LAG

    parse = REGISTRY[KEY_MAX_STAMP_LAG].parse
    assert parse("20 ms") == "20 ms"
    assert parse(0) == "0"
    assert parse(5) == "5 nsec"
    for bad in ("-1 ms", "soon", None, True, 1.5):
        with pytest.raises(ValueError):
            parse(bad)
    assert REGISTRY[KEY_MAX_STAMP_LAG].runtime is False


def test_max_stamp_lag_marks_lf_clock_inputs_only():
    """Only a declared lag gives a lost message a stamp bound,
    and a lag on a ``declared:`` input is refused."""
    from colcon_lf.exec_config.resolve import CorrBoundsConfig

    class Graph:
        nodes = {KEY: _node()}

    corr = {KEY: g._NodeCorr()}
    g._mark_bna_stamp_lag(Graph, CorrBoundsConfig(stamp_provenance={
        (KEY, "in/a"): "lf_clock", (KEY, "in/b"): "lf_clock"},
        max_stamp_lag_ns={(KEY, "in/b"): 20_000_000}), corr)
    assert corr[KEY].bna_stamp_lag == {"cb_via_sub_b": 20_000_000}
    with pytest.raises(ValueError, match="needs stamp_provenance 'lf_clock'"):
        g._mark_bna_stamp_lag(Graph, CorrBoundsConfig(stamp_provenance={
            (KEY, "in/a"): "lf_clock", (KEY, "in/b"): "declared:camera"},
            max_stamp_lag_ns={(KEY, "in/b"): 0}), {KEY: g._NodeCorr()})
    with pytest.raises(ValueError, match="max_stamp_lag on topic 'in/c', which is no "
                                         "BoundedNearestToAnchor input"):
        g._mark_bna_stamp_lag(Graph, CorrBoundsConfig(
            stamp_provenance={(KEY, "in/c"): "lf_clock"},
            max_stamp_lag_ns={(KEY, "in/c"): 0}), {KEY: g._NodeCorr()})
    # a key of a node outside this graph (another federate's pass) is not ours
    g._mark_bna_stamp_lag(Graph, CorrBoundsConfig(
        max_stamp_lag_ns={("other", "in/c"): 0}), {KEY: g._NodeCorr()})

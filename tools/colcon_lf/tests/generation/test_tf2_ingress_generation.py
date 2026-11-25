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
"""Generator side of the LF-owned tf2 listener inputs (S2).

The analyzer gives an admitted TransformListener two synthetic inputs, /tf
and /tf_static. Their RID rows carry kGroupTf2Listener (-4), so the binding
matches them only to the handles of the group it detached from tf2_ros's
executor. A /tf_static edge from a StaticTransformBroadcaster to such an input
is admitted although both ends are transient_local (ruling in
``_tf2_static_edge``); any other transient_local edge stays legacy.
"""

from __future__ import annotations

from colcon_lf.generation.binding_header_emitter import collect_rid_table
from colcon_lf.node_analysis.models import (
    CallbackInfo, NodeInfo, PublisherInfo, SubscriberInfo, TimerInfo, Tf2AdapterInfo,
    Tf2BroadcasterInfo, Tf2Info, Tf2ListenerInfo,
)
from colcon_lf.node_analysis.tf2_scan import TF_MESSAGE_TYPE, ingest_callback

from ..corr_declared import generate_declared
from .test_correspondence_emission import _edge, _graph, _tc


def _broadcaster(static: bool) -> NodeInfo:
    topic = "/tf_static" if static else "/tf"
    return NodeInfo(
        name="br", class_name="BrNode", package="test_pkg", source_file="/fake/br.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=[topic])],
        timers=[TimerInfo(period_ms=20.0, callback="tick")],
        publishers=[PublisherInfo(
            topic=topic, message_type=TF_MESSAGE_TYPE, member_variable="br_",
            created_in="constructor", unconditional=True, qos_depth=1 if static else 100,
            qos_reliability="reliable",
            qos_durability="transient_local" if static else "volatile")],
        tf2=Tf2Info(broadcasters=[Tf2BroadcasterInfo(member="br_", static=static)]),
    )


def _listener() -> NodeInfo:
    subs, cbs = [], []
    for topic, durability in (("/tf", "volatile"), ("/tf_static", "transient_local")):
        name = ingest_callback("lst_", topic)
        cbs.append(CallbackInfo(name=name))
        subs.append(SubscriberInfo(
            topic=topic, message_type=TF_MESSAGE_TYPE, callback=name, member_variable="lst_",
            created_in="constructor", tf2_listener="lst_", qos_depth=100,
            qos_reliability="reliable", qos_durability=durability))
    return NodeInfo(
        name="lst", class_name="LstNode", package="test_pkg", source_file="/fake/lst.cpp",
        callbacks=cbs, subscribers=subs,
        tf2=Tf2Info(listeners=[Tf2ListenerInfo(member="lst_", buffer_member="buf_")]))


def _system(static: bool, listener: NodeInfo | None = None):
    topic = "/tf_static" if static else "/tf"
    other = "/tf" if static else "/tf_static"
    # As build_communication lists it: the listener's other input has no
    # publisher in the system.
    graph = _graph([_broadcaster(static), listener or _listener()],
                   [_tc(topic, ["br"], ["lst"]), _tc(other, [], ["lst"])])
    return graph, generate_declared(graph)


def test_listener_inputs_get_the_tf2_listener_group_in_the_rid_table():
    graph, system = _system(static=False)
    rows = {(r[1], r[2]): r for r in collect_rid_table({r.name: r for r in system.reactors}, graph)}
    got = {cb: (rows[("LstNode", cb)][3], rows[("LstNode", cb)][4], rows[("LstNode", cb)][7])
           for cb in ("tf_ingest_via_lst", "tf_static_ingest_via_lst")}
    assert got == {"tf_ingest_via_lst": ("sub", "/tf", -4),
                   "tf_static_ingest_via_lst": ("sub", "/tf_static", -4)}, got
    # The broadcaster's own timer keeps the default group.
    assert rows[("BrNode", "tick")][7] == -1


def test_tf_edge_from_a_broadcaster_is_admitted():
    _, system = _system(static=False)
    e = _edge(system, "br", "lst")
    assert (e.topic, e.status, e.reason) == ("/tf", "admitted", None), e


def test_tf_static_edge_from_a_static_broadcaster_is_admitted():
    _, system = _system(static=True)
    e = _edge(system, "br", "lst")
    assert (e.topic, e.status, e.reason) == ("/tf_static", "admitted", None), e


def test_tf_static_edge_to_a_plain_subscription_stays_transient_local_legacy():
    """The ruling covers the tf2 listener input only: a user subscription on
    /tf_static with transient_local keeps today's refusal."""
    lst = _listener()
    for s in lst.subscribers:
        s.tf2_listener = None
    lst.tf2 = None
    _, system = _system(static=True, listener=lst)
    e = _edge(system, "br", "lst")
    assert (e.status, e.reason) == ("legacy", "transient_local"), e


# --- S3: the AcquisitionLookup wake path ------------------------------------

def _listener_with_adapter() -> NodeInfo:
    """The listener plus a sample input whose callback makes requests."""
    lst = _listener()
    lst.callbacks.append(CallbackInfo(name="on_sample"))
    lst.subscribers.append(SubscriberInfo(
        topic="/sample", message_type="std_msgs::msg::Header", callback="on_sample",
        member_variable="sample_sub_", created_in="constructor", qos_depth=10,
        qos_reliability="reliable", qos_durability="volatile"))
    lst.tf2.adapters = [Tf2AdapterInfo(member="lk_", buffer_member="buf_",
                                       wait_bound_ns=55_000_000, on_timeout="reject")]
    return lst


def _lst_reactor(system):
    [r] = [r for r in system.reactors if any("LstNode" in x.body for x in r.reactions)]
    return r


def test_adapter_node_gets_the_wake_loop_ingest_notice_and_wake_reaction():
    graph = _graph([_broadcaster(False), _listener_with_adapter()],
                   [_tc("/tf", ["br"], ["lst"]), _tc("/tf_static", [], ["lst"]),
                    _tc("/sample", [], ["lst"])])
    system = generate_declared(graph)
    reactor = _lst_reactor(system)
    assert "tf2_wake" in reactor.logical_actions
    loop = ('{ int64_t tw; while ((tw = ros2_lf_tf2_take_wake("LstNode")) >= 0) '
            '{ lf_schedule(tf2_wake, tw); } }')
    by_cb = {}
    for r in reactor.reactions:
        for cb in ("tf_ingest_via_lst", "tf_static_ingest_via_lst", "on_sample"):
            if f"RID_LstNode_{cb}" in r.body and r.kind == "sub":
                by_cb[cb] = r
    assert set(by_cb) == {"tf_ingest_via_lst", "tf_static_ingest_via_lst", "on_sample"}, by_cb
    for cb, r in by_cb.items():
        lines = r.body.split("\n")
        assert loop in lines and "tf2_wake" in r.effects, (cb, r.body)
        # The notice follows the user callback, the loop follows the notice.
        notice = 'ros2_lf_tf2_ingested("LstNode");'
        if cb == "on_sample":
            assert notice not in lines
        else:
            assert lines.index(notice) == lines.index(loop) - 1, r.body
    kinds = [r.kind for r in reactor.reactions]
    wake = kinds.index("tf2_wake")
    assert wake > max(reactor.reactions.index(r) for r in by_cb.values())
    assert reactor.reactions[wake].triggers == ["tf2_wake"]
    assert reactor.reactions[wake].body == f'ros2_lf_tf2_wake("LstNode");\n{loop}'
    assert all(k == "corr_arrive" for k in kinds[wake + 1:] if k not in ("tf2_close",))
    assert kinds.count("tf2_close") == 1


def test_listener_without_an_adapter_gets_no_tf2_reactions():
    _, system = _system(static=False)
    reactor = _lst_reactor(system)
    assert "tf2_wake" not in reactor.logical_actions
    assert not any("ros2_lf_tf2_" in r.body for r in reactor.reactions)

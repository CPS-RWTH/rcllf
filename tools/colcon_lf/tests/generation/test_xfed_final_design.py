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
"""The final cross-federate design is the default.

d = 0 on every link (``after 0``, one microstep) unless a link states a d.
The publishing reaction sends its records over ``after d`` itself (PUB);
only a decentralized d > 0 edge keeps the 1 ns flush. The ingress members of
a fan-in group arrive in their drain reaction (MERGED).
"""
from __future__ import annotations

import pytest

from colcon_lf.federation.partition import DEFAULT_CROSS_FEDERATE_AFTER_NS, AfterOverride
from colcon_lf.resolved_model.resolve import DEFAULT_LINK_AFTER_DELAY
from colcon_lf.verb.generate import DEFAULT_AFTER_DELAY

from .test_correspondence_emission import (
    MSG, CallbackInfo, NodeInfo, PublisherInfo, SubscriberInfo, TimerInfo, _graph, _pub, _sub,
    _tc, generate_declared,
)
from ..corr_declared import declared
from .test_xfed_correspondence import _federate, _pubsub, _reactor, _row

_FIVE_MS = [AfterOverride("pub.data", "sub.data", 5_000_000)]
_ONE_NS = [AfterOverride("pub.data", "sub.data", 1)]


def _reaction(lf: str, trigger: str) -> tuple[str, str]:
    """(head, body) of the first reaction triggered by *trigger*."""
    r = lf[lf.index(f"reaction({trigger})"):]
    head, body = r.split("{=", 1)
    return head, body[:body.index("=}")]


def _fan_in(tmp_path):
    """Two publishers of /a on front feed mid on back, which relays /b to sink."""
    nodes = [_pub("s1", "S1Node", "a"), _pub("s2", "S2Node", "a"),
             _sub("mid", "MidNode", "a", publishes="b"), _sub("sink", "SinkNode", "b")]
    return _federate(tmp_path, nodes,
                     [_tc("/a", ["s1", "s2"], ["mid"]), _tc("/b", ["mid"], ["sink"])],
                     {"s1": "front", "s2": "front", "mid": "back", "sink": "back"})


def test_every_default_after_is_zero():
    # The CLI, the resolver's record of it and the cross-federate default
    # agree: one microstep, no logical latency added by the tool.
    assert (DEFAULT_AFTER_DELAY, DEFAULT_LINK_AFTER_DELAY, DEFAULT_CROSS_FEDERATE_AFTER_NS) == (
        "0", "0", 0), "a default after delay is not 0"


def test_default_cross_link_is_plain_sent_by_the_publisher(tmp_path):
    # The default (d = 0, centralized) cross-federate link is a
    # plain connection, records arrive at their publication tag (t, m).
    out = _pubsub(tmp_path)
    assert "    pub.xcorr_g0 -> sub.xcorr_g0\n" in out["top"], (
        "the default cross-federate link is not a plain connection:\n" + out["top"])
    assert ("{ rcllf_xcorr_t x; if (ros2_lf_corr_send_remote(self, 0, &x) > 0) "
            "lf_set(xcorr_e0, x); }") in out["pub"]["lf"]
    assert _row(out["pub"]["rows"])["delay_ns"] == 0


def test_centralized_d_sends_at_the_publishing_tag_without_a_flush(tmp_path):
    out = _pubsub(tmp_path, overrides=_FIVE_MS)
    assert "pub.xcorr_g0 -> sub.xcorr_g0 after 5 msec" in out["top"], (
        "a centralized d > 0 link is not `after d`: the 1 ns flush is still on\n" + out["top"])
    assert "xcorr_flush" not in out["pub"]["lf"], "PUB is the default, no flush action"
    assert "ros2_lf_corr_send_remote(self, 0, &x)" in out["pub"]["lf"]
    assert ("if (xcorr_e0->is_present) { int64_t a = "
            "ros2_lf_corr_arrive_remote(self, 0, &xcorr_e0->value);") in out["sub"]["lf"]


def test_relay_sends_from_its_drain(tmp_path):
    # A relay drain runs at several microsteps of one time; each sends its own
    # message, there is no flush to collect them.
    nodes = [_pub("src", "SrcNode", "a"), _sub("mid", "MidNode", "a", publishes="b"),
             _sub("sink", "SinkNode", "b")]
    out = _federate(tmp_path, nodes, [_tc("/a", ["src"], ["mid"]), _tc("/b", ["mid"], ["sink"])],
                    {"src": "front", "mid": "front", "sink": "back"})
    cross, local = _row(out["front"]["rows"], "/b"), _row(out["front"]["rows"], "/a")
    e = cross["id"]
    group = _reactor(out["front"]["lf"], "MidNode_DefaultGroup")
    assert "xcorr_flush" not in group, group
    # The local d = 0 edge /a merges its port into the drain.
    l = local["id"]
    _, drain = _reaction(group, f"corr_drain_e{l}, corr_e{l}")
    assert f"ros2_lf_corr_send_remote(self, {e}, &x)" in drain


def test_decentralized_d_keeps_the_flush(tmp_path):
    # Under decentralized coordination a second message of one time
    # at the receiver is tardy, so one flush per time stays.
    # The flush carries a COMPLETE number, so the edge is admitted.
    out = _pubsub(tmp_path, overrides=_FIVE_MS, coordination="decentralized")
    assert "pub.xcorr_g0 -> sub.xcorr_g0 after 4999999 nsec" in out["top"], out["top"]
    assert "lf_schedule(xcorr_flush_e0, NSEC(1))" in out["pub"]["lf"]
    assert "ros2_lf_corr_send_remote" not in out["pub"]["lf"]


def test_decentralized_zero_sends_at_the_publishing_tag(tmp_path):
    out = _pubsub(tmp_path, coordination="decentralized")
    assert "pub.xcorr_g0 -> sub.xcorr_g0 after 0" in out["top"], out["top"]
    assert "xcorr_flush" not in out["pub"]["lf"]


def test_flush_escape_switch_restores_the_flush(tmp_path, monkeypatch):
    monkeypatch.setenv("RCLLF_XFED_FLUSH", "1")
    out = _pubsub(tmp_path, overrides=_FIVE_MS)
    assert "pub.xcorr_g0 -> sub.xcorr_g0 after 4999999 nsec" in out["top"], out["top"]
    assert "lf_schedule(xcorr_flush_e0, NSEC(1))" in out["pub"]["lf"]
    # d = 0 never flushes, the switch or not, and stays admitted.
    out0 = _pubsub(tmp_path / "zero")
    assert "xcorr_flush" not in out0["pub"]["lf"]
    assert _row(out0["pub"]["rows"])["status"] == "admitted"


def test_one_ns_is_admitted_without_a_flush_and_refused_with_one(tmp_path, monkeypatch):
    # lf_delay_tag((t, m), 1 ns) = (t + 1 ns, 0): with no flush there is no
    # 2 ns floor. A flush edge needs d - 1 ns > 0.
    eg = _row(_pubsub(tmp_path, overrides=_ONE_NS)["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("admitted", None), eg
    eg = _row(_pubsub(tmp_path / "dec", overrides=_ONE_NS,
                      coordination="decentralized")["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("legacy", "xfed_flush_min"), eg


def test_flush_edges_are_admitted_with_complete_accounting(tmp_path, monkeypatch):
    # The flush message carries a COMPLETE number (one per time),
    # so a dropped flush message is a CONTROL_FAILURE and every flush edge is
    # admitted again (an earlier version refused them as xfed_flush_no_complete), the
    # decentralized d > 0 one and the RCLLF_XFED_FLUSH=1 one alike.
    eg = _row(_pubsub(tmp_path / "dec", overrides=_FIVE_MS,
                      coordination="decentralized")["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("admitted", None), eg
    eg = _row(_pubsub(tmp_path / "cen", overrides=_FIVE_MS)["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("admitted", None), eg
    eg = _row(_pubsub(tmp_path / "dec0", coordination="decentralized")["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("admitted", None), eg
    monkeypatch.setenv("RCLLF_XFED_FLUSH", "1")
    eg = _row(_pubsub(tmp_path / "sw", overrides=_FIVE_MS)["pub"]["rows"])
    assert (eg["status"], eg["reason"]) == ("admitted", None), eg


def test_fan_in_members_arrive_in_the_drain_reaction(tmp_path):
    out = _fan_in(tmp_path)
    a, b = sorted(r["id"] for r in out["back"]["rows"] if r["topic"] == "/a")
    group = _reactor(out["back"]["lf"], "MidNode_DefaultGroup")
    trigger = f"corr_drain_e{a}, xcorr_e{a}, xcorr_e{b}"
    assert f"reaction({trigger})" in group, (
        "fan-in members arrive in separate reactions, not in the drain: each "
        "drains its own record at one microstep in edge order, not rank order\n" + group)
    assert f"reaction(xcorr_e{a})" not in group and f"reaction(xcorr_e{b})" not in group
    _, body = _reaction(group, trigger)
    files = body[:body.index("if (w == 0 || ")]
    for m in (a, b):
        assert (f"if (xcorr_e{m}->is_present) {{ int64_t a = "
                f"ros2_lf_corr_arrive_remote(self, {m}, &xcorr_e{m}->value);") in files, files
    assert f"if (w == 0 || corr_drain_e{a}->is_present) {{" in body
    assert body.count("ros2_lf_corr_drain(") == 1


def test_split_drain_escape_switch_restores_one_arrive_per_member(tmp_path, monkeypatch):
    monkeypatch.setenv("RCLLF_XFED_SPLIT_DRAIN", "1")
    out = _fan_in(tmp_path)
    a, b = sorted(r["id"] for r in out["back"]["rows"] if r["topic"] == "/a")
    group = _reactor(out["back"]["lf"], "MidNode_DefaultGroup")
    for m in (a, b):
        _, body = _reaction(group, f"xcorr_e{m}")
        assert f"ros2_lf_corr_arrive_remote(self, {m}, " in body, body
        assert "ros2_lf_corr_drain(" not in body, body


# ---------------------------------------------------------------------------
# A cross-federate edge closes a zero-delay cycle like a local one
# ---------------------------------------------------------------------------


def _brake_loop():
    """The brake_fusion loop in small (lf_generated_brake_fusion_t2approx): ``ctl``
    publishes /cmd to ``sim`` from its /obs and /speed callbacks, ``sim`` publishes
    /state to ``whl`` from a catch-up loop the analyzer cannot bound, ``whl``
    publishes /speed back to ``ctl``. /state is judged before /speed, so the cycle
    check counts it, and sizing refuses it later (burst_unbounded), which leaves
    no admitted loop for the slack rule. Only the zero-delay cycle check can
    refuse /speed, and in one process it does."""
    sim = NodeInfo(
        name="sim", class_name="SimNode", package="test_pkg", source_file="/fake/sim.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=["/state"]),
                   CallbackInfo(name="on_cmd")],
        timers=[TimerInfo(period_ms=1.0, callback="tick")],
        publishers=[PublisherInfo(topic="/state", message_type=MSG)],
        subscribers=[SubscriberInfo(topic="/cmd", message_type=MSG, callback="on_cmd")],
    )
    ctl = NodeInfo(
        name="ctl", class_name="CtlNode", package="test_pkg", source_file="/fake/ctl.cpp",
        callbacks=[CallbackInfo(name="on_obs", publishers_used=["/cmd"]),
                   CallbackInfo(name="on_speed", publishers_used=["/cmd"])],
        publishers=[PublisherInfo(topic="/cmd", message_type=MSG)],
        subscribers=[SubscriberInfo(topic="/obs", message_type=MSG, callback="on_obs"),
                     SubscriberInfo(topic="/speed", message_type=MSG, callback="on_speed")],
    )
    whl = _sub("whl", "WhlNode", topic="/state", publishes="/speed")
    nodes = [_pub("cam", "CamNode", topic="/obs"), ctl, sim, whl]
    topics = [_tc("/obs", ["cam"], ["ctl"]), _tc("/cmd", ["ctl"], ["sim"]),
              _tc("/state", ["sim"], ["whl"]), _tc("/speed", ["whl"], ["ctl"])]
    return nodes, topics


# The brake_fusion bounds in small (the earlier config/t2/tinf*.execution.yaml):
# a 1 ms own T on the simulator's command input, long T elsewhere, the
# controller's and the wheel node's input rates declared (the plant step),
# and no burst bound for the simulator.
_BRAKE_BOUNDS = {
    "take_timeout_ms": {("sim", "/cmd"): 1, ("ctl", "/obs"): 1000, ("ctl", "/speed"): 1000,
                        ("whl", "/state"): 1000},
    "min_input_period_ms": {("ctl", "/speed"): 1.0, ("whl", "/state"): 1.0},
}


def _brake_bounds(graph):
    cfg = declared(graph, **_BRAKE_BOUNDS)
    del cfg.burst["sim"]
    return cfg


def _admission(rows) -> dict[str, tuple[str, str | None]]:
    """topic -> (status, reason) over the rows of every federate (one edge per topic)."""
    out: dict[str, tuple[str, str | None]] = {}
    for r in rows:
        out.setdefault(r["topic"], (r["status"], r["reason"]))
    return out


@pytest.mark.parametrize("split,overrides", [
    ({"cam": "front", "ctl": "front", "sim": "back", "whl": "back"}, ()),
    ({"cam": "front", "ctl": "front", "sim": "back", "whl": "back"},
     [AfterOverride("whl.speed", "ctl.speed", 0), AfterOverride("ctl.cmd", "sim.cmd", 0)]),
    ({"cam": "front", "ctl": "front", "sim": "front", "whl": "front"}, ()),
], ids=["two_federates_default_after", "two_federates_after_0", "one_federate"])
@pytest.mark.parametrize("sim_order,speed", [
    ("subscriptions_first", ("admitted", None)),
    ("timers_first", ("admitted", None)),
], ids=["latch_before_step", "step_before_latch"])
def test_cross_federate_edge_closing_a_loop_is_judged_like_one_process(
        tmp_path, split, overrides, sim_order, speed):
    # A run saw /wheel/speed refused in one process but admitted on two hosts:
    # the zero-delay cycle check skipped cross-federate members although the
    # cross default after is 0 (partition.py DEFAULT_CROSS_FEDERATE_AFTER_NS).
    # A later fix then refused the loop only with sim's /cmd latch
    # declared before its publishing step. The corr_arrive is
    # declared last in every order and lfc sees no cycle, so /speed is
    # admitted in both orders, in one process and on every split alike; the
    # one refusal left (/state) is the missing burst bound.
    nodes, topics = _brake_loop()
    graph = _graph(nodes, topics)
    order = {"sim": sim_order}
    one = generate_declared(graph, target_lfc=True, corr_bounds=_brake_bounds(graph),
                            reaction_order=order)
    want = {e.topic: (e.status, e.reason) for e in one.corr_edges}
    assert want == {"/obs": ("admitted", None), "/cmd": ("admitted", None),
                    "/state": ("legacy", "burst_unbounded"),
                    "/speed": speed}, want
    out = _federate(tmp_path, nodes, topics, split, overrides=overrides, bounds=_brake_bounds,
                    reaction_order=order)
    rows = [r for fed in dict.fromkeys(split.values()) for r in out[fed]["rows"]]
    assert _admission(rows) == want, (
        f"the {'/'.join(dict.fromkeys(split.values()))} build admits differently from one "
        f"process: {_admission(rows)} != {want}")

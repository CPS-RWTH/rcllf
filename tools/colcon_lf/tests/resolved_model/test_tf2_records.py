# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""tf2 resolved-model records and the RCLLF-R002 subcode messages.

The scan result of a source snippet goes through the real record builder and
the real generate gate, so the refusal text asserted here is what
`colcon lf-generate` prints. N3 on the real tf_listener_example is in
test_gate.py (it needs the ROS headers).
"""

from __future__ import annotations

from types import SimpleNamespace

from colcon_lf.node_analysis.tf2_scan import TF2_ROS_PIN, scan
from colcon_lf.resolved_model.model import PROVENANCE_EXTRACTED, PROVENANCE_UNRESOLVED
from colcon_lf.resolved_model.resolve import _tf2_records
from colcon_lf.verb.generate import _check_resolved_model_gate

ADMITTED = (
    "Req::Req() : Node(\"req\") {\n"
    "  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());\n"
    "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);\n"
)


def _gate(tf2, capsys, acked=()):
    records = _tf2_records("req", tf2)
    rc = _check_resolved_model_gate(SimpleNamespace(records=records), set(acked))
    return rc, capsys.readouterr().err, records


def test_n2_blocking_lookup_message_verbatim(capsys):
    tf2 = scan([("req.cpp", ADMITTED + "}\nvoid Req::on_tick() {\n"
                 "  auto t = buf_->lookupTransform(\"map\", \"base\", stamp, 50ms);\n}\n")], "Req")
    rc, err, _ = _gate(tf2, capsys)
    assert rc == 1
    assert err == (
        "  Error: generation refused, RCLLF-R002 tf2_blocking_lookup: req "
        "buf_->lookupTransform at req.cpp:6 waits on its timeout for a /tf message that can "
        "only arrive in a later reaction, so it blocks the LF worker; a waiting lookup cannot "
        "see /tf messages from later reactions; use rcllf_tf2::AcquisitionLookup with its wait "
        "bound instead (or list RCLLF-R002 in acknowledged_hazards to proceed as unsupported)\n"
    )


def test_buffer_clock_mismatch_message_verbatim(capsys):
    tf2 = scan([("req.cpp", ADMITTED.replace("this->get_clock()", "wall_clock_") + "}\n")], "Req")
    rc, err, _ = _gate(tf2, capsys)
    assert rc == 1
    assert err == (
        "  Error: generation refused, RCLLF-R002 tf2_clock_mismatch: req Buffer buf_ at "
        "req.cpp:2 runs on clock 'wall_clock_', not the node's own get_clock(); under rcllf "
        "only the node clock follows LF logical time, so the Buffer's cache time would not be "
        "the tag; construct the Buffer with the node's get_clock() and declare the provenance "
        "of external /tf in execution.yaml (or list RCLLF-R002 in acknowledged_hazards to "
        "proceed as unsupported)\n"
    )


def test_acknowledged_subcode_proceeds_with_the_old_warning(capsys):
    tf2 = scan([("req.cpp", ADMITTED + "}\nvoid Req::f() { buf_->canTransform(\"a\", \"b\", t); }\n")],
               "Req")
    rc, err, _ = _gate(tf2, capsys, acked=["RCLLF-R002"])
    assert rc == 0
    assert "Warning: acknowledged hazard RCLLF-R002 present (req)" in err


def test_broadcaster_only_node_is_resolved_not_refused(capsys):
    """A node that only broadcasts (tf_broadcaster_example's static node)
    is resolved, not refused for its tf2_ros include."""
    tf2 = scan([("b.cpp", "B::B() : Node(\"b\") {\n"
                 "  st_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);\n}\n")], "B")
    rc, err, records = _gate(tf2, capsys)
    assert rc == 0 and err == ""
    [rec] = records
    assert rec.provenance == PROVENANCE_EXTRACTED
    assert rec.value == {"listeners": [], "buffer_clock": None, "ingress": None,
                         "broadcasters": [{"member": "st_", "topic": "/tf_static"}]}


def test_adapter_gets_a_lookup_record_with_its_literal_bound(capsys):
    tf2 = scan([("req.cpp", ADMITTED + "  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
                 "*buf_, this, 60ms, rcllf_tf2::OnTimeout::fallback(\"identity\"));\n}\n"
                 "void Req::on_tick() { lk_->request(\"a\", \"b\", now(), cb); }\n")], "Req")
    tf2.stamps = [_stamp("node_clock", "now()")]  # the AST's result (test_tf2_stamp_ast.py)
    rc, _, records = _gate(tf2, capsys)
    assert rc == 0
    lookup = [r for r in records if r.property == "tf2.lookup.lk_"]
    assert [(r.value, r.unit, r.provenance) for r in lookup] == [(
        {"wait_bound_ns": 60_000_000, "on_timeout": "fallback:identity", "buffer": "buf_",
         "buffer_clock": "node", "tf2_ros_version": TF2_ROS_PIN}, "ns", PROVENANCE_EXTRACTED)]
    assert lookup[0].evidence == "literal AcquisitionLookup arguments at req.cpp:4"


def test_system_json_without_scan_keeps_the_blanket_refusal(capsys):
    rc, err, records = _gate(None, capsys)
    assert rc == 1
    assert records[0].provenance == PROVENANCE_UNRESOLVED
    assert err == (
        "  Error: generation refused, RCLLF-R002 req depends on unsupported tf2 (no rcllf "
        "contract for it; list RCLLF-R002 in acknowledged_hazards to proceed as unsupported)\n"
    )


def _listener_graph(published: tuple[str, ...]):
    from colcon_lf.node_analysis.models import SubscriberInfo
    subs = [SubscriberInfo(topic=t, message_type="tf2_msgs::msg::TFMessage",
                           callback=f"tf_ingest_{t.strip('/')}", member_variable="lst_",
                           created_in="constructor", tf2_listener="lst_")
            for t in ("/tf", "/tf_static")]
    topics = [SimpleNamespace(topic=t, publishers=["src"] if t in published else [],
                              subscribers=["acq"]) for t in ("/tf", "/tf_static")]
    return SimpleNamespace(nodes={"acq": SimpleNamespace(subscribers=subs)},
                           communication=SimpleNamespace(topics=topics))


def _n1_line(topic):
    return (
        f"  Error: generation refused, RCLLF-R002 tf2_clock_mismatch: acq TransformListener "
        f"lst_ subscribes '{topic}', which no node in this system publishes; transforms from "
        f"outside have no declared stamp provenance, so the Buffer's content at a tag would "
        f"depend on arrival time; broadcast them from a node of this system, or declare "
        f"'{topic}' in execution.yaml external_topics with 'anchor: stamp' so each TFMessage "
        f"enters at its header stamp (or list RCLLF-R002 in acknowledged_hazards "
        f"to proceed as unsupported)\n")


def test_n1_external_tf_without_provenance_message_verbatim(capsys):
    """N1: a listener whose /tf comes from outside the system."""
    from colcon_lf.verb.generate import _check_tf2_external_ingress
    rc = _check_tf2_external_ingress(_listener_graph(("/tf_static",)), {}, set())
    assert rc == 1
    assert capsys.readouterr().err == _n1_line("/tf")


def test_n1_in_system_broadcasters_or_acknowledgement_pass(capsys):
    from colcon_lf.verb.generate import _check_tf2_external_ingress
    assert _check_tf2_external_ingress(_listener_graph(("/tf", "/tf_static")), {}, set()) == 0
    assert _check_tf2_external_ingress(_listener_graph(()), {}, {"RCLLF-R002"}) == 0
    assert capsys.readouterr().err == ""


def test_n1_stamp_anchored_external_tf_is_admitted_arrival_anchor_is_not(capsys):
    """X91r2 item 1: a declared ``anchor: stamp`` is the stamp provenance; a
    plain (arrival-anchored) external_topics entry is not."""
    from colcon_lf.verb.generate import _check_tf2_external_ingress
    stamp = {"anchor": "stamp", "basis": "robot clock", "delay_ns": 30_000_000}
    graph = _listener_graph(("/tf_static",))
    assert _check_tf2_external_ingress(graph, {}, set(), {"/tf": stamp}) == 0, (
        "generation refused: a /tf declared 'anchor: stamp' in execution.yaml is still "
        "refused as tf2_clock_mismatch")
    assert capsys.readouterr().err == ""
    assert _check_tf2_external_ingress(graph, {}, set(), {}) == 1
    assert capsys.readouterr().err == _n1_line("/tf")


def test_tf_topic_provenance_record_per_listener_topic(capsys):
    """X91r2 item 2 (OWED tf2-provenance-record): one ``topic:<tf>#provenance``
    record per listener topic, in-system / stamp-anchored / undeclared."""
    from colcon_lf.resolved_model.model import PROVENANCE_EXPLICITLY_CONFIGURED
    from colcon_lf.resolved_model.resolve import _tf2_topic_records
    graph = _listener_graph(("/tf_static",))
    recs = {r.contract_id: r for r in _tf2_topic_records(
        graph, SimpleNamespace(external_anchors={}))}
    assert sorted(recs) == ["topic:/tf#provenance", "topic:/tf_static#provenance"], (
        f"resolved model has no topic:/tf#provenance record per listener topic: {sorted(recs)}")
    st = recs["topic:/tf_static#provenance"]
    assert (st.provenance, st.value) == (PROVENANCE_EXTRACTED, {
        "source": "in_system", "publishers": ["src"], "listeners": ["acq.lst_"]})
    tf = recs["topic:/tf#provenance"]
    assert (tf.provenance, tf.reason, tf.value) == (
        PROVENANCE_UNRESOLVED, "RCLLF-R002",
        {"source": "external_undeclared", "listeners": ["acq.lst_"]})

    stamp = {"anchor": "stamp", "basis": "robot clock", "delay_ns": 30_000_000}
    tf = {r.contract_id: r for r in _tf2_topic_records(
        graph, SimpleNamespace(external_anchors={"/tf": stamp}))}["topic:/tf#provenance"]
    assert (tf.provenance, tf.reason) == (PROVENANCE_EXPLICITLY_CONFIGURED, None)
    assert tf.value == {"source": "external", "listeners": ["acq.lst_"], **stamp,
                        "tf_rule": "common_stamp_or_tf_mixed_stamps"}


def test_unresolved_tf_topic_record_passes_the_gate_only_when_acknowledged(capsys):
    """The gate read subject['node'] on every R002 record; a topic record
    must refuse with its topic, not raise KeyError."""
    from colcon_lf.resolved_model.resolve import _tf2_topic_records
    recs = [r for r in _tf2_topic_records(_listener_graph(("/tf_static",)),
                                          SimpleNamespace(external_anchors={}))
            if r.provenance == PROVENANCE_UNRESOLVED]
    assert _check_resolved_model_gate(SimpleNamespace(records=recs), set()) == 1
    assert capsys.readouterr().err == (
        "  Error: generation refused, RCLLF-R002 tf2_clock_mismatch: '/tf' reaches a "
        "TransformListener from outside the system with no declared stamp provenance; "
        "declare it in execution.yaml external_topics with 'anchor: stamp' (or list "
        "RCLLF-R002 in acknowledged_hazards to proceed as unsupported)\n")
    assert _check_resolved_model_gate(SimpleNamespace(records=recs), {"RCLLF-R002"}) == 0
    assert "Warning: acknowledged hazard RCLLF-R002 present (/tf)" in capsys.readouterr().err


# --- tf2.sample_stamp.<adapter> (acceptance criterion 5) ----------

ADAPTER = ("  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
           "*buf_, this, 60ms, rcllf_tf2::OnTimeout::reject());\n}\n")


def _stamp(kind, expr, detail="", **kw):
    """One request stamp as the AST classifier (tf2_scan.ast_stamps) records it;
    the classification itself is tested on real TUs in test_tf2_stamp_ast.py."""
    from colcon_lf.node_analysis.models import Tf2StampInfo
    return Tf2StampInfo(adapter="lk_", kind=kind, expr=expr, detail=detail,
                        file="req.cpp", line=7, **kw)


def _stamp_gate(stamps, capsys, *, subs=(), published=(), anchors=None):
    from colcon_lf.node_analysis.models import SubscriberInfo
    tf2 = scan([("req.cpp", ADMITTED + ADAPTER)], "Req")
    tf2.stamps = list(stamps)
    node = SimpleNamespace(subscribers=[
        SubscriberInfo(topic=t, message_type="sensor_msgs::msg::Imu", callback=cb)
        for t, cb in subs])
    graph = SimpleNamespace(communication=SimpleNamespace(topics=[
        SimpleNamespace(topic=t, publishers=["imu_node"], subscribers=["req"])
        for t in published]))
    records = _tf2_records("req", tf2, node_info=node, graph=graph,
                           exec_config=SimpleNamespace(external_anchors=anchors or {}))
    rc = _check_resolved_model_gate(SimpleNamespace(records=records), set())
    stamp = [r for r in records if r.property == "tf2.sample_stamp.lk_"]
    return rc, capsys.readouterr().err, stamp


def test_unknown_stamp_provenance_refuses_with_the_record(capsys):
    """Falsification (criterion 5): on origin/main there is no sample stamp
    record, so a request at a stamp of unknown source generated (rc 0)."""
    rc, err, stamp = _stamp_gate([_stamp(
        "unknown", "stamp_", "'stamp_' is a member with no initializer or write")], capsys)
    assert stamp, "no tf2.sample_stamp record: an unknown request stamp did not refuse generation"
    assert rc == 1
    assert err == (
        "  Error: generation refused, RCLLF-R002 tf2_stamp_unknown: req lk_->request at "
        "req.cpp:7 passes stamp 'stamp_': 'stamp_' is a member with no initializer or write; "
        "pass the request a stamp the scan can follow (now(), msg->header.stamp in the "
        "subscription callback, or a variable set from one of them) (or list RCLLF-R002 in acknowledged_hazards to proceed as unsupported)\n")


def test_foreign_clock_stamp_is_tf2_stamp_mismatch(capsys):
    expr = "rclcpp::Clock(RCL_STEADY_TIME).now()"
    rc, err, [rec] = _stamp_gate([_stamp("foreign_clock", expr, "a separate rclcpp::Clock")],
                                 capsys)
    assert rc == 1 and rec.provenance == PROVENANCE_UNRESOLVED
    assert err.startswith(
        "  Error: generation refused, RCLLF-R002 tf2_stamp_mismatch: req lk_->request at "
        f"req.cpp:7 passes stamp '{expr}': a separate rclcpp::Clock, but the Buffer runs on "
        "the node clock")


def test_node_clock_stamp_is_extracted(capsys):
    rc, err, [rec] = _stamp_gate([_stamp("node_clock", "now()")], capsys)
    assert (rc, err) == (0, "")
    assert (rec.contract_id, rec.provenance) == ("node:req#tf2.sample_stamp.lk_",
                                                 PROVENANCE_EXTRACTED)
    assert rec.value == {"buffer_clock": "node", "requests": [
        {"expr": "now()", "kind": "node_clock", "file": "req.cpp", "line": 7,
         "source": "node_clock"}]}


MSG = [_stamp("message", "msg->header.stamp", "header.stamp of 'msg'", callback="on_imu")]


def test_message_stamp_needs_an_in_system_or_stamp_anchored_topic(capsys):
    from colcon_lf.resolved_model.model import PROVENANCE_EXPLICITLY_CONFIGURED
    rc, err, [rec] = _stamp_gate(MSG, capsys, subs=[("imu", "on_imu")], published=["/imu"])
    assert (rc, err, rec.provenance) == (0, "", PROVENANCE_EXTRACTED)
    assert rec.value["requests"][0]["sources"] == [
        {"topic": "/imu", "source": "in_system", "publishers": ["imu_node"]}]

    anchor = {"/imu": {"anchor": "stamp", "basis": "robot clock", "delay_ns": 1}}
    rc, err, [rec] = _stamp_gate(MSG, capsys, subs=[("/imu", "on_imu")], anchors=anchor)
    assert (rc, err, rec.provenance) == (0, "", PROVENANCE_EXPLICITLY_CONFIGURED)

    rc, err, _ = _stamp_gate(MSG, capsys, subs=[("/imu", "on_imu")],
                             anchors={"/imu": {"anchor": "arrival"}})
    assert rc == 1 and "RCLLF-R002 tf2_stamp_mismatch: req lk_->request at req.cpp:7" in err
    assert "of '/imu', which comes from outside the system without 'anchor: stamp'" in err

    rc, err, _ = _stamp_gate(MSG, capsys)
    assert rc == 1 and "'on_imu', which is no subscription callback of the node" in err

    lam = [_stamp("message", "m->header.stamp", "header.stamp of 'm'", topic="imu")]
    rc, err, [rec] = _stamp_gate(lam, capsys, published=["/imu"])
    assert (rc, err, rec.value["requests"][0]["topics"]) == (0, "", ["/imu"])


def test_adapter_without_a_seen_request_or_without_stamp_scan_refuses(capsys):
    rc, err, _ = _stamp_gate([], capsys)
    assert rc == 1 and "the scan finds no lk_->request(...) call" in err
    tf2 = scan([("req.cpp", ADMITTED + ADAPTER)], "Req")
    assert tf2.stamps is None  # the text scan leaves stamps to the AST
    [rec] = [r for r in _tf2_records("req", tf2) if r.property.startswith("tf2.sample_stamp")]
    assert rec.value["refusals"][0]["detail"].endswith("re-run colcon lf-analyze")

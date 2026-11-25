# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The tf2 detection split and its RCLLF-R002 subcodes (S1).

The admitted form is a tf2_ros::Buffer on the node clock, one
TransformListener(*buffer, this) member created in the constructor, and
broadcasters as ordinary publishers. A direct or blocking Buffer lookup, a
second or differently constructed listener and a Buffer on another clock are
refused, each with a subcode and a message that names the line.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.node_analysis.tf2_scan import duration_ns, scan

from .ros_env import requires_ros_headers

REPO = Path(__file__).resolve().parents[3]

HEADER = """
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  void on_tick();
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> lst_;
};
"""


def _cpp(ctor_body: str, cb_body: str = "") -> str:
    return (
        '#include "req.hpp"\n'
        "Req::Req() : Node(\"req\") {\n" + ctor_body + "\n}\n"
        "void Req::on_tick() {\n" + cb_body + "\n}\n"
    )


ADMITTED = (
    "  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());\n"
    "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);"
)


def _scan(ctor: str, cb: str = ""):
    return scan([("req.hpp", HEADER), ("req.cpp", _cpp(ctor, cb))], "Req")


def test_admitted_listener_is_found_and_not_refused():
    info = _scan(ADMITTED)
    assert [(b.member, b.clock) for b in info.buffers] == [("buf_", "node")]
    assert [(x.member, x.buffer_member, x.line) for x in info.listeners] == [("lst_", "buf_", 4)]
    assert info.refusals == []


def test_no_tf2_use_gives_none():
    assert scan([("a.cpp", "int main() { return 0; }")], "A") is None


def test_direct_latest_lookup_is_tf2_direct_lookup():
    info = _scan(ADMITTED, '  auto t = buf_->lookupTransform("a", "b", tf2::TimePointZero);')
    assert [(r.subcode, r.detail) for r in info.refusals] == [(
        "tf2_direct_lookup",
        "buf_->lookupTransform at req.cpp:7 answers with what the Buffer holds at that "
        "instant (the latest transform for TimePointZero) and bypasses the bounded "
        "acquisition-time request",
    )]


@pytest.mark.parametrize("timeout", ["50ms", "std::chrono::milliseconds(50)",
                                     "rclcpp::Duration::from_seconds(0.05)",
                                     "tf2::durationFromSec(0.05)"])
def test_lookup_with_a_timeout_is_tf2_blocking_lookup(timeout):
    info = _scan(ADMITTED, f'  auto t = buf_->lookupTransform("a", "b", stamp, {timeout});')
    assert [r.subcode for r in info.refusals] == ["tf2_blocking_lookup"]
    assert info.refusals[0].detail.startswith("buf_->lookupTransform at req.cpp:7 waits")


def test_wait_for_transform_is_blocking_and_can_transform_direct():
    info = _scan(ADMITTED, '  buf_->waitForTransform("a", "b", s, 1s, cb);\n'
                           '  if (buf_->canTransform("a", "b", tf2::TimePointZero)) {}')
    assert [r.subcode for r in info.refusals] == ["tf2_blocking_lookup", "tf2_direct_lookup"]


def test_lookup_in_a_comment_is_not_a_lookup():
    info = _scan(ADMITTED, '  // buf_->lookupTransform("a", "b", tf2::TimePointZero);\n'
                           '  /* buf_->canTransform("a", "b", t) */')
    assert info.refusals == []


def test_zero_timeout_argument_is_direct_not_blocking():
    info = _scan(ADMITTED, '  buf_->lookupTransform("a", "b", t, 0ms);')
    assert [r.subcode for r in info.refusals] == ["tf2_direct_lookup"]


@pytest.mark.parametrize(("ctor", "why"), [
    ("  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());\n"
     "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_);",
     "it is not given the node itself (`this`), so it creates a node of its own"),
    ("  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());\n"
     "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this, false);",
     "spin_thread is 'false', not the default true"),
])
def test_unadmitted_listener_construction_is_refused(ctor, why):
    info = _scan(ctor)
    # Recorded, but the refusal keeps the parser from claiming its inputs.
    assert [x.member for x in info.listeners] == ["lst_"]
    assert [r.subcode for r in info.refusals] == ["tf2_listener_unadmitted"]
    assert info.refusals[0].detail.startswith("TransformListener lst_ at req.cpp:4: " + why)


def test_two_listeners_are_refused():
    info = _scan(ADMITTED + "\n  lst2_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);")
    assert [r.subcode for r in info.refusals] == ["tf2_listener_unadmitted"]
    assert info.refusals[0].detail.startswith("2 TransformListeners (lst_ at req.cpp:4, lst2_ at")


def test_buffer_on_another_clock_is_tf2_clock_mismatch():
    info = _scan("  buf_ = std::make_shared<tf2_ros::Buffer>(steady_clock_);\n"
                 "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);")
    assert [r.subcode for r in info.refusals] == ["tf2_clock_mismatch"]
    assert info.refusals[0].detail.startswith(
        "Buffer buf_ at req.cpp:3 runs on clock 'steady_clock_', not the node's own get_clock()")


def test_broadcasters_are_recorded_static_or_not():
    info = scan([("b.cpp",
                  "B::B() : Node(\"b\") {\n"
                  "  dyn_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);\n"
                  "  st_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);\n}\n")],
                 "B")
    assert [(b.member, b.static, b.line) for b in info.broadcasters] == [
        ("dyn_", False, 2), ("st_", True, 3)]
    assert info.refusals == []


def test_acquisition_lookup_adapter_literal_is_read():
    info = _scan(ADMITTED + "\n  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
                 "*buf_, this, 60ms, rcllf_tf2::OnTimeout::reject());")
    assert [(a.member, a.buffer_member, a.wait_bound_ns, a.on_timeout) for a in info.adapters] == [
        ("lk_", "buf_", 60_000_000, "reject")]
    assert info.refusals == []


def test_adapter_without_literal_bound_is_refused():
    info = _scan(ADMITTED + "\n  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
                 "*buf_, this, bound_, rcllf_tf2::OnTimeout::reject());")
    assert [r.subcode for r in info.refusals] == ["tf2_listener_unadmitted"]


def test_adapter_on_a_buffer_the_listener_does_not_feed_is_refused():
    # Before the check this scanned with no refusal: the adapter would read a
    # Buffer that never receives /tf from LF (review finding F4).
    info = _scan(ADMITTED + "\n  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
                 "*other_buf_, this, 60ms, rcllf_tf2::OnTimeout::reject());")
    assert [(r.subcode, r.detail) for r in info.refusals] == [(
        "tf2_listener_unadmitted",
        "AcquisitionLookup lk_ at req.cpp:5 reads Buffer 'other_buf_', but the node's "
        "TransformListener feeds 'buf_'")]


@pytest.mark.parametrize(("expr", "ns"), [
    ("60ms", 60_000_000), ("std::chrono::milliseconds(60)", 60_000_000),
    ("2s", 2_000_000_000), ("150us", 150_000), ("x", None), ("0.5s", None),
])
def test_duration_ns(expr, ns):
    assert duration_ns(expr) == ns


# --- the real packages ---------------------------------------------------------

def _real(pkg: str, name: str) -> list[tuple[str, str]]:
    src = REPO / "packages" / "src" / "fixtures" / pkg / "src" / name
    return [(str(src), src.read_text())]


def test_tf_listener_example_is_refused_as_direct_lookup():
    info = scan(_real("tf_listener_example", "tf_listener_node.cpp"), "TfListenerNode")
    assert [(x.member, x.buffer_member) for x in info.listeners] == [("tf_listener_", "tf_buffer_")]
    assert [r.subcode for r in info.refusals] == ["tf2_direct_lookup"]
    assert info.refusals[0].detail.startswith("tf_buffer_->lookupTransform at tf_listener_node.cpp:44 ")


def test_tf_broadcaster_example_nodes_are_admitted():
    dyn = scan(_real("tf_broadcaster_example", "dynamic_tf_broadcaster_node.cpp"),
               "DynamicTfBroadcasterNode")
    st = scan(_real("tf_broadcaster_example", "static_tf_broadcaster_node.cpp"),
              "StaticTfBroadcasterNode")
    assert [(b.member, b.static) for b in dyn.broadcasters] == [("dynamic_broadcaster_", False)]
    assert [(b.member, b.static) for b in st.broadcasters] == [("static_broadcaster_", True)]
    assert dyn.refusals == [] and st.refusals == []


@requires_ros_headers("the tf2 parser integration test")
def test_parser_turns_tf2_endpoints_into_graph_edges():
    """Broadcasters publish /tf and /tf_static; the admitted listener
    subscribes both with tf2_ros's own QoS (qos.hpp of the pinned tf2_ros)."""
    from colcon_lf.node_analysis.clang_parser import parse_node

    src = REPO / "packages" / "src" / "fixtures" / "tf_broadcaster_example" / "src"
    dyn = parse_node(src / "dynamic_tf_broadcaster_node.cpp")
    st = parse_node(src / "static_tf_broadcaster_node.cpp")
    assert [(p.topic, p.message_type, p.qos_depth, p.qos_durability) for p in dyn.publishers] == [
        ("/tf", "tf2_msgs::msg::TFMessage", 100, "volatile")]
    assert [(p.topic, p.qos_depth, p.qos_durability) for p in st.publishers] == [
        ("/tf_static", 1, "transient_local")]
    # The sendTransform calls count as publishes of the timer callback.
    assert [(t.callback, t.period_ms) for t in dyn.timers] == [("timer_callback", 33.0)]
    [cb] = [c for c in dyn.callbacks if c.name == "timer_callback"]
    assert cb.publishers_used == ["/tf"]


@requires_ros_headers("the tf2 parser integration test")
def test_parser_does_not_synthesize_listener_inputs_for_a_refused_node():
    from colcon_lf.node_analysis.clang_parser import parse_node

    info = parse_node(REPO / "packages" / "src" / "fixtures" / "tf_listener_example" / "src"
                      / "tf_listener_node.cpp")
    assert [r.subcode for r in info.tf2.refusals] == ["tf2_direct_lookup"]
    assert [s.topic for s in info.subscribers if s.tf2_listener] == []


# Request stamp provenance (criterion 5) is classified on the AST since the
# Tests in test_tf2_stamp_ast.py.
def test_source_scan_leaves_request_stamps_to_the_ast():
    info = _scan(ADMITTED + "\n  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>("
                 "*buf_, this, 60ms, rcllf_tf2::OnTimeout::reject());",
                 '  lk_->request("a", "b", this->now(), cb);')
    assert [a.member for a in info.adapters] == ["lk_"]
    assert info.stamps is None

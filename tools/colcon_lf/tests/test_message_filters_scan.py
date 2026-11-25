# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the message_filters source scan (route A).

The admitted shapes are the ones in packages/src/fusion_example and
qos_hazard_example/message_filters_subscriber_node. Everything else must be
refused with a reason that names what is unsupported.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.node_analysis.message_filters_scan import qos_from_args, scan

from .ros_env import requires_ros_headers

REPO = Path(__file__).resolve().parents[3]

HEADER = """
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
class Fuse : public rclcpp::Node {
  using Policy = message_filters::sync_policies::ApproximateTime<M, M>;
  void cb(const M::ConstSharedPtr & a, const M::ConstSharedPtr & b);
  message_filters::Subscriber<M> sub_a_;  // input a
  message_filters::Subscriber<M> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
};
"""

CPP = """
Fuse::Fuse() : Node("fuse") {
  sub_a_.subscribe(this, "in/a");
  sub_b_.subscribe(this, "in/b", rmw_qos_profile_sensor_data);
  /* sync_.reset(); in a comment does not count */
  sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(7), sub_a_, sub_b_);
  sync_->registerCallback(std::bind(&Fuse::cb, this, std::placeholders::_1,
                                    std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "message_filters node up");
}
"""


def test_admitted_approximate_time_two_inputs():
    r = scan([CPP, HEADER], "Fuse")
    assert r.info.unsupported == []
    [s] = r.info.synchronizers
    assert (s.member_variable, s.policy, s.queue_size, s.inputs, s.callback) == (
        "sync_", "ApproximateTime", 7, ["sub_a_", "sub_b_"], "cb")
    assert [(i.topic, i.callback, i.sync_callback, i.message_type) for i in r.inputs] == [
        ("in/a", "cb_via_sub_a", "cb", "M"),
        ("in/b", "cb_via_sub_b", "cb", "M"),
    ]
    assert r.inputs[1].qos_args == "rmw_qos_profile_sensor_data"


def test_admitted_exact_time_inline_policy_init_list_form():
    """Plain Synchronizer member, constructor-form Subscribers, &Class::cb overload."""
    header = """
class Fuse : public rclcpp::Node {
  message_filters::Subscriber<pkg::msg::A> a_;
  message_filters::Subscriber<pkg::msg::B> b_;
  message_filters::Synchronizer<
    message_filters::sync_policies::ExactTime<pkg::msg::A, pkg::msg::B>> sync_;
};
"""
    cpp = """
Fuse::Fuse()
: Node("fuse"), a_(this, "a", rclcpp::QoS(3).get_rmw_qos_profile()), b_(this, "b"),
  sync_(message_filters::sync_policies::ExactTime<pkg::msg::A, pkg::msg::B>(5), a_, b_)
{
  sync_.registerCallback(&Fuse::on_pair, this);
}
"""
    r = scan([cpp, header], "Fuse")
    assert r.info.unsupported == []
    [s] = r.info.synchronizers
    assert (s.policy, s.queue_size, s.inputs, s.callback) == (
        "ExactTime", 5, ["a_", "b_"], "on_pair")
    assert [(i.topic, i.message_type) for i in r.inputs] == [
        ("a", "pkg::msg::A"), ("b", "pkg::msg::B")]
    assert qos_from_args(r.inputs[0].qos_args)["qos_depth"] == 3


def test_fusion_example_sources_are_admitted():
    for cls, stem in (("ExactFusionNode", "exact_fusion_node"),
                      ("ApproxFusionNode", "approx_fusion_node")):
        pkg = REPO / "packages/src/fusion_example"
        r = scan([(pkg / f"src/{stem}.cpp").read_text(),
                  (pkg / f"include/fusion_example/{stem}.hpp").read_text()], cls)
        assert r.info.unsupported == []
        assert [i.topic for i in r.inputs] == ["fusion/a", "fusion/b"]


@pytest.mark.parametrize("cls, stem", [("ExactFusionNode", "exact_fusion_node"),
                                        ("ApproximateFusionNode", "approximate_fusion_node")])
def test_brake_fusion_sources_are_admitted(cls, stem):
    """The approximate fusion's sample ledger once added a
    second observer per Subscriber, so every rcllf build of t2_approx refused
    with RCLLF-H003. The fusion nodes keep one observer per input."""
    pkg = REPO / "packages/src/brake_fusion"
    r = scan([(pkg / f"src/{stem}.cpp").read_text(),
              (pkg / f"include/brake_fusion/{stem}.hpp").read_text()], cls)
    assert r.info is not None and r.info.unsupported == [], \
        f"{cls} would refuse with RCLLF-H003: {r.info and r.info.unsupported}"
    assert [i.topic for i in r.inputs] == ["/radar/range", "/camera/range"]


def test_cache_fixture_is_refused():
    pkg = REPO / "packages/src/fixtures/qos_hazard_example"
    r = scan([(pkg / "src/message_filters_cache_node.cpp").read_text(),
              (pkg / "include/qos_hazard_example/message_filters_cache_node.hpp").read_text()],
             "MessageFiltersCacheNode")
    assert r.inputs == []
    assert any("message_filters::Cache is not supported" in u for u in r.info.unsupported)
    assert any("connectInput" in u for u in r.info.unsupported)


@pytest.mark.parametrize("edit, reason", [
    (("ApproximateTime<M, M>", "ApproximateTime<M, M, M>"), "has 3 inputs"),
    (("ApproximateTime<M, M>", "LatestTime<M, M>"), "sync policy LatestTime is not supported"),
    (('"in/a"', "topic_name_"), "must be subscribed as subscribe(this"),
    (("std::bind(&Fuse::cb, this, std::placeholders::_1,\n"
      "                                    std::placeholders::_2)", "lambda_"),
     "needs exactly one registerCallback"),
    (("Policy(7), sub_a_, sub_b_", "Policy(7), sub_a_, cache_"), "input cache_ is not a"),
    (("sub_a_.subscribe(this", "sub_a_.subscribe(other_node_"), "must be subscribed as"),
    (('sub_b_.subscribe(this, "in/b", rmw_qos_profile_sensor_data);',
      'sub_b_.subscribe(this, "in/b"); sub_b_.subscribe(this, "in/c");'),
     "Subscriber sub_b_ must be subscribed exactly once, found 2"),
    (("Policy(7), sub_a_, sub_b_", "Policy(7), sub_a_, sub_a_"),
     "Subscriber sub_a_ feeds 2 synchronizer inputs"),
    (('  RCLCPP_INFO(get_logger(), "message_filters node up");',
      '  sub_b_.unsubscribe();'),
     "Subscriber sub_b_ calls unsubscribe()"),
    (('  RCLCPP_INFO(get_logger(), "message_filters node up");',
      '  sub_a_->unsubscribe();'),
     "Subscriber sub_a_ calls unsubscribe()"),
])
def test_unsupported_forms_are_refused_with_reason(edit, reason):
    old, new = edit
    src = [CPP.replace(old, new), HEADER.replace(old, new)]
    assert src != [CPP, HEADER], "edit did not apply"
    r = scan(src, "Fuse")
    assert r.info.synchronizers == []
    assert r.inputs == []
    assert any(reason in u for u in r.info.unsupported), r.info.unsupported


def test_subscriber_shared_across_synchronizers_is_refused():
    """Two synchronizers fed by one Subscriber would give one runtime
    subscription two reaction IDs."""
    header = HEADER.replace(
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;",
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;\n"
        "  message_filters::Subscriber<M> sub_c_;\n"
        "  std::shared_ptr<message_filters::Synchronizer<Policy>> sync2_;")
    cpp = CPP.replace(
        '  RCLCPP_INFO(get_logger(), "message_filters node up");',
        '  sub_c_.subscribe(this, "in/c");\n'
        "  sync2_ = std::make_shared<message_filters::Synchronizer<Policy>>"
        "(Policy(7), sub_a_, sub_c_);\n"
        "  sync2_->registerCallback(std::bind(&Fuse::cb, this, std::placeholders::_1,\n"
        "                                     std::placeholders::_2));")
    assert header != HEADER and cpp != CPP, "edit did not apply"
    r = scan([cpp, header], "Fuse")
    assert "Subscriber sub_a_ feeds 2 synchronizer inputs " \
        "(one Subscriber member per input, it is one subscription)" in r.info.unsupported
    assert r.info.synchronizers == []
    assert r.inputs == []


def test_namespace_alias_is_refused():
    header = HEADER.replace("class Fuse", "namespace mf = message_filters;\nclass Fuse")
    r = scan([CPP, header], "Fuse")
    assert any("namespace alias or using-directive" in u for u in r.info.unsupported)


def test_no_message_filters_returns_none():
    assert scan(["int main() {}"], "X").info is None


def test_qos_mapping():
    assert qos_from_args(None) == {"qos_depth": 10, "qos_reliability": "reliable",
                                   "qos_durability": "volatile", "qos_history": "keep_last"}
    assert qos_from_args("rclcpp::SensorDataQoS()")["qos_reliability"] == "best_effort"
    assert qos_from_args("custom_profile_")["qos_depth"] is None


# fusion_example's custom message headers exist only in a built workspace.
@requires_ros_headers("the fusion_example parse test")
def test_parse_node_emits_one_subscription_per_input():
    """End to end through parse_node on the real fusion_example source."""
    pytest.importorskip("clang.cindex", reason="libclang not available")
    from colcon_lf.node_analysis.clang_parser import parse_node

    node = parse_node(REPO / "packages/src/fusion_example/src/exact_fusion_node.cpp",
                      workspace_root=REPO / "packages")
    subs = {s.topic: s for s in node.subscribers}
    assert sorted(subs) == ["fusion/a", "fusion/b"]
    assert subs["fusion/a"].callback == "sync_callback_via_sub_a"
    assert subs["fusion/a"].sync_callback == "sync_callback"
    assert subs["fusion/a"].callback_group == "default_group"
    cbs = {c.name: c for c in node.callbacks}
    # Each input's reaction carries the synchronizer callback's publisher.
    assert cbs["sync_callback_via_sub_b"].publishers_used == ["fusion/exact_pairs"]
    assert node.message_filters.unsupported == []


# One observer callback per Subscriber (an arrival log). The
# library calls it from the same rclcpp subscription callback as the
# synchronizer input, so it runs inside that input's reaction.
OBSERVER = ('  RCLCPP_INFO(get_logger(), "message_filters node up");',
            "  sub_a_.registerCallback(std::bind(&Fuse::seen_a, this, std::placeholders::_1));")


def test_subscriber_observer_callback_is_admitted():
    src = [CPP.replace(*OBSERVER), HEADER]
    r = scan(src, "Fuse")
    assert r.info.unsupported == []
    assert [(i.member, i.callback, i.observer) for i in r.inputs] == [
        ("sub_a_", "cb_via_sub_a", "seen_a"),
        ("sub_b_", "cb_via_sub_b", None),
    ]


@pytest.mark.parametrize("observer, reason", [
    ("  sub_a_.registerCallback(std::bind(&Fuse::seen_a, this, std::placeholders::_1));\n"
     "  sub_a_.registerCallback(std::bind(&Fuse::seen_b, this, std::placeholders::_1));",
     "Subscriber sub_a_ admits one observer registerCallback("
     "std::bind(&Class::method, this, _1)), found 2"),
    ("  sub_a_.registerCallback(lambda_);",
     "Subscriber sub_a_ admits one observer registerCallback("
     "std::bind(&Class::method, this, _1)), found 1"),
    ("  sub_a_.registerCallback(std::bind(&Other::seen_a, this, std::placeholders::_1));",
     "Subscriber sub_a_ admits one observer registerCallback("),
])
def test_unsupported_observer_forms_are_refused(observer, reason):
    src = [CPP.replace(OBSERVER[0], observer), HEADER]
    r = scan(src, "Fuse")
    assert r.inputs == []
    assert any(reason in u for u in r.info.unsupported), r.info.unsupported


def test_observer_with_effects_is_refused():
    from colcon_lf.node_analysis.message_filters_scan import observer_refusal
    from colcon_lf.node_analysis.models import CallbackInfo

    assert observer_refusal(CallbackInfo(name="seen_a", variables_written=["n_"])) is None
    for field_name in ("publishers_used", "service_clients_used", "action_clients_used",
                       "spawns_callbacks", "feedback_published", "result_published"):
        cb = CallbackInfo(name="seen_a", **{field_name: ["x"]})
        assert observer_refusal(cb) == (
            f"observer seen_a has effects ({field_name}: x); an observer on a "
            "synchronizer input may only read and log"), field_name


@requires_ros_headers("the brake_fusion observer parse test")
def test_parse_node_brake_fusion_observers():
    """brake_fusion's ExactFusionNode with arrival observers."""
    pytest.importorskip("clang.cindex", reason="libclang not available")
    from colcon_lf.node_analysis.clang_parser import parse_node

    node = parse_node(REPO / "packages/src/brake_fusion/src/exact_fusion_node.cpp",
                      workspace_root=REPO / "packages")
    assert node.message_filters.unsupported == []
    subs = {s.topic: s.callback for s in node.subscribers}
    assert subs == {"/radar/range": "sync_callback_via_radar_sub",
                    "/camera/range": "sync_callback_via_camera_sub"}
    cbs = {c.name: c for c in node.callbacks}
    assert cbs["sync_callback_via_radar_sub"].publishers_used == ["/fused/range"]
    assert "radar_in_" in cbs["sync_callback_via_radar_sub"].variables_written
    assert "camera_in_" in cbs["sync_callback_via_camera_sub"].variables_written


# A message_filters input's QoS resolves like a create_subscription
# QoS. brake_fusion passes local rclcpp::QoS variables as
# qos.get_rmw_qos_profile() (approximate_fusion_node.cpp:145-153,
# exact_fusion_node.cpp:50-58), which the text reading leaves null. Expected
# values read off the source by hand: KeepLast(10) is depth 10, radar
# .best_effort(), camera .reliable(), both .durability_volatile(), history
# keep_last (the KeepLast constructor). The deadline is
# rclcpp::Duration(kRadarDeadline), a named constant, which the shared
# create_subscription helper leaves None as well.
BF_QOS = {
    "/radar/range": {"depth": 10, "reliability": "best_effort",
                     "durability": "volatile", "history": "keep_last"},
    "/camera/range": {"depth": 10, "reliability": "reliable",
                      "durability": "volatile", "history": "keep_last"},
}


def _qos_of(sub):
    return {"depth": sub.qos_depth, "reliability": sub.qos_reliability,
            "durability": sub.qos_durability, "history": sub.qos_history}


@requires_ros_headers("the brake_fusion message_filters QoS parse test")
@pytest.mark.parametrize("stem", ["approximate_fusion_node", "exact_fusion_node"])
def test_parse_node_brake_fusion_input_qos(stem):
    pytest.importorskip("clang.cindex", reason="libclang not available")
    from colcon_lf.node_analysis.clang_parser import parse_node

    node = parse_node(REPO / f"packages/src/brake_fusion/src/{stem}.cpp",
                      workspace_root=REPO / "packages")
    got = {s.topic: _qos_of(s) for s in node.subscribers}
    assert got == BF_QOS, f"{stem}: message_filters input QoS {got}"


SNIPPET = """\
#include <functional>
#include <memory>
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using M = sensor_msgs::msg::Range;

// A project function named like message_filters' subscribe that changes the
// profile it gets by reference.
static void subscribe(rmw_qos_profile_t & p) {p.depth = 2;}

class Fuse : public rclcpp::Node
{
public:
  Fuse() : Node("fuse")
  {
    auto qos_a = rclcpp::QoS(rclcpp::KeepLast(4)).best_effort();
%s
    sub_a_.subscribe(this, "in/a", qos_a.get_rmw_qos_profile());
    sub_b_.subscribe(this, "in/b", rclcpp::QoS(6).get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(7), sub_a_, sub_b_);
    sync_->registerCallback(std::bind(&Fuse::cb, this, std::placeholders::_1,
                                      std::placeholders::_2));
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<M, M>;
  void cb(const M::ConstSharedPtr &, const M::ConstSharedPtr &) {}
  message_filters::Subscriber<M> sub_a_;
  message_filters::Subscriber<M> sub_b_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
};
"""

UNKNOWN = {"depth": None, "reliability": None, "durability": None, "history": None}


@requires_ros_headers("the message_filters QoS snippet test")
@pytest.mark.parametrize("extra, want_a", [
    # Declared once, only read: follows the variable.
    ("", {"depth": 4, "reliability": "best_effort", "durability": "volatile",
          "history": "keep_last"}),
    # Changed after its declaration, through the profile reference or a
    # builder statement: not followed, stays unknown (RCLLF-H000/H001).
    ("    qos_a.get_rmw_qos_profile().depth = 2;", UNKNOWN),
    ("    qos_a.reliable();", UNKNOWN),
    # A non-message_filters ``subscribe`` taking the profile by reference is
    # not a read.
    ("    subscribe(qos_a.get_rmw_qos_profile());", UNKNOWN),
])
def test_parse_node_message_filters_qos_variable(tmp_path, extra, want_a):
    pytest.importorskip("clang.cindex", reason="libclang not available")
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "fuse_node.cpp"
    cpp.write_text(SNIPPET % extra, encoding="utf-8")
    node = parse_node(cpp, workspace_root=tmp_path)
    assert node.message_filters.unsupported == []
    got = {s.topic: _qos_of(s) for s in node.subscribers}
    assert got == {"in/a": want_a,
                   "in/b": {"depth": 6, "reliability": "reliable",
                            "durability": "volatile", "history": "keep_last"}}, got

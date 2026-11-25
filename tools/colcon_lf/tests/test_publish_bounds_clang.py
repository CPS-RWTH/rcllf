# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Publish call-site bounds per callback (W3 slice S3).

The analyzer proves the burst bound k of a correspondence edge by counting
the publish call sites of each publishing callback outside any loop. Each
snippet's expected count is read off its own source text below.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the publish-bound snippet tests")


_NODE = """\
#include <algorithm>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "std_msgs/msg/string.hpp"

using Pub = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
{pre}
class BoundNode : public rclcpp::Node
{{
public:
  BoundNode() : Node("bound_node")
  {{
    out_ = create_publisher<std_msgs::msg::String>("/out", 10);
    aux_ = create_publisher<std_msgs::msg::String>("/aux", 10);
{ctor}
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&BoundNode::on_tick, this));
  }}

private:
  void on_tick()
  {{
{tick}
  }}
  void helper()
  {{
    std_msgs::msg::String m;
    out_->publish(m);
  }}
{extra}
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aux_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundNode>());
  rclcpp::shutdown();
  return 0;
}}
"""


def _parse(tmp_path: Path, tick: str, ctor: str = "", extra: str = "", pre: str = ""):
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "bound_node.cpp"
    cpp.write_text(_NODE.format(tick=tick, ctor=ctor, extra=extra, pre=pre), encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path)


def _cb(node, name):
    return next(cb for cb in node.callbacks if cb.name == name)


M = "    std_msgs::msg::String m;\n"


def test_one_site_proves_k_1(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);"), "on_tick")
    assert tick.publish_bounds == {"/out": 1}, (tick.publish_bounds, tick.publish_unbounded)


def test_sites_on_two_topics_count_per_topic(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    if (m.data.empty()) {out_->publish(m);}\n"
                      "    aux_->publish(m);"), "on_tick")
    assert tick.publish_bounds == {"/out": 2, "/aux": 1}, tick.publish_bounds


def test_site_in_a_loop_has_no_bound(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    for (int i = 0; i < 3; ++i) {out_->publish(m);}\n"
                      "    aux_->publish(m);"), "on_tick")
    assert tick.publish_bounds == {"/out": None, "/aux": 1}, tick.publish_bounds
    assert tick.publish_unbounded["/out"].startswith("loop at line"), tick.publish_unbounded


def test_site_in_a_nested_lambda_has_no_bound(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    auto f = [this, &m]() {out_->publish(m);};\n    f();"), "on_tick")
    assert tick.publish_bounds == {"/out": None}, tick.publish_bounds
    assert tick.publish_unbounded["/out"].startswith("lambda at line"), tick.publish_unbounded


def test_member_helper_counts_once_per_call_site(tmp_path):
    tick = _cb(_parse(tmp_path, "    helper();\n    helper();"), "on_tick")
    assert tick.publish_bounds == {"/out": 2}, (tick.publish_bounds, tick.publish_unbounded)


def test_member_helper_in_a_loop_has_no_bound(tmp_path):
    tick = _cb(_parse(tmp_path, "    while (true) {helper(); break;}"), "on_tick")
    assert tick.publish_bounds == {"/out": None}, tick.publish_bounds
    assert "through helper" in tick.publish_unbounded["/out"], tick.publish_unbounded


def test_goto_makes_every_topic_unbounded(tmp_path):
    tick = _cb(_parse(tmp_path, M + "  again:\n    out_->publish(m);\n    if (m.data.empty()) {goto again;}\n"
                      "    aux_->publish(m);"), "on_tick")
    assert tick.publish_bounds == {"/out": None, "/aux": None}, tick.publish_bounds


def test_serialized_publish_marks_the_publisher(tmp_path):
    node = _parse(tmp_path, "    rclcpp::SerializedMessage s;\n    out_->publish(s);")
    flags = {p.topic: p.serialized_publish for p in node.publishers}
    assert flags == {"/out": True, "/aux": False}, flags


def test_deadline_lambda_publishes_are_effects_of_the_subscription_callback(tmp_path):
    ctor = """\
    rclcpp::SubscriptionOptions so;
    so.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &) {
        std_msgs::msg::String m;
        aux_->publish(m);
      };
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1), so);"""
    extra = "  void on_in(const std_msgs::msg::String & m) {out_->publish(m);}\n"
    on_in = _cb(_parse(tmp_path, "", ctor, extra), "on_in")
    assert on_in.missing_handler_publishers == ["/aux"], on_in
    assert sorted(on_in.publishers_used) == ["/aux", "/out"], on_in.publishers_used
    assert on_in.publish_bounds == {"/out": 1, "/aux": 1}, on_in.publish_bounds


def test_deadline_member_handler_counts_with_max_not_sum(tmp_path):
    # The handler runs instead of the callback in a drain reaction, so the
    # bound on /out is max(1, 2) = 2, not 3.
    ctor = """\
    rclcpp::SubscriptionOptions so;
    so.event_callbacks.deadline_callback =
      std::bind(&BoundNode::on_missing, this, std::placeholders::_1);
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1), so);"""
    extra = ("  void on_in(const std_msgs::msg::String & m) {out_->publish(m);}\n"
             "  void on_missing(rclcpp::QOSDeadlineRequestedInfo &)\n"
             "  {std_msgs::msg::String m; out_->publish(m); out_->publish(m);}\n")
    on_in = _cb(_parse(tmp_path, "", ctor, extra), "on_in")
    assert on_in.missing_handler_publishers == ["/out"], on_in
    assert on_in.publish_bounds == {"/out": 2}, on_in.publish_bounds


# Escapes, member options and unnamed serialized publishes --
# A proven bound never undercounts: what the analyzer cannot rule out leaves
# the bound unknown, so the edge is refused instead of admitted with a false k.


def test_free_function_that_receives_the_publisher_leaves_no_bound(tmp_path):
    # send_twice publishes twice through its parameter, so on_tick makes 3.
    pre = "void send_twice(const Pub & p) {std_msgs::msg::String m; p->publish(m); p->publish(m);}"
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    send_twice(out_);", pre=pre), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert "send_twice" in tick.publish_unbounded["/out"], tick.publish_unbounded


def test_opaque_function_that_receives_the_publisher_leaves_no_bound(tmp_path):
    pre = "void send_elsewhere(const Pub & p);"  # declared only, no visible body
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    send_elsewhere(out_);", pre=pre), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert "call to send_elsewhere" in tick.publish_unbounded["/out"], tick.publish_unbounded


def test_for_each_with_a_bound_member_leaves_no_bound(tmp_path):
    # helper_n publishes once per element of v, which the analyzer cannot count.
    tick = _cb(_parse(
        tmp_path,
        M + "    out_->publish(m);\n    std::vector<int> v{1, 2};\n"
        "    std::for_each(v.begin(), v.end(), std::bind(&BoundNode::helper_n, this, std::placeholders::_1));",
        extra="  void helper_n(int) {std_msgs::msg::String m; out_->publish(m);}\n"), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert "call to for_each" in tick.publish_unbounded["/out"], tick.publish_unbounded


def test_publishing_lambda_stored_in_the_constructor_leaves_no_bound(tmp_path):
    # cb_() runs helper(), one more /out publish the call site does not show.
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    cb_();",
                      ctor="    cb_ = [this]() {helper();};",
                      extra="  std::function<void()> cb_;\n"), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert tick.publish_unbounded["/out"].startswith("the node escapes: call to operator="), (
        tick.publish_unbounded)


def test_plain_calls_keep_the_bound(tmp_path):
    tick = _cb(_parse(
        tmp_path,
        M + '    RCLCPP_INFO(this->get_logger(), "n=%s", m.data.c_str());\n'
        "    double y = std::sqrt(2.0);\n    m.data = std::to_string(y);\n"
        "    std::vector<int> v{1, 2};\n    std::for_each(v.begin(), v.end(), [](int) {});\n"
        "    out_->publish(m);"), "on_tick")
    assert tick.publish_bounds == {"/out": 1}, (tick.publish_bounds, tick.publish_unbounded)


def test_publishing_lambda_on_a_spawned_thread_keeps_the_bound(tmp_path):
    # The thread publishes outside any callback dispatch, so on_tick still makes 1.
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);",
                      ctor="    worker_ = std::thread([this]() {helper();});",
                      extra="  std::thread worker_;\n"), "on_tick")
    assert tick.publish_bounds == {"/out": 1}, (tick.publish_bounds, tick.publish_unbounded)


_MEMBER_OPTIONS_CTOR = """\
    setup_options();
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1), options_);"""


def test_deadline_handler_on_member_options_set_in_another_method_counts(tmp_path):
    # The handler publishes /out twice, the callback once: max(1, 2) = 2.
    extra = """\
  void setup_options()
  {
    options_.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &) {
        std_msgs::msg::String m;
        out_->publish(m);
        out_->publish(m);
      };
  }
  void on_in(const std_msgs::msg::String & m) {out_->publish(m);}
  rclcpp::SubscriptionOptions options_;
"""
    on_in = _cb(_parse(tmp_path, "", _MEMBER_OPTIONS_CTOR, extra), "on_in")
    assert (on_in.publish_bounds, on_in.missing_handler_publishers) == ({"/out": 2}, ["/out"]), (
        on_in.publish_bounds, on_in.publish_unbounded)


def test_deadline_handler_the_analyzer_cannot_resolve_leaves_no_bound(tmp_path):
    # make_options() has no visible body, so its handler may publish anything.
    ctor = """\
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1), make_options());"""
    extra = "  void on_in(const std_msgs::msg::String & m) {out_->publish(m);}\n"
    on_in = _cb(_parse(tmp_path, "", ctor, extra, pre="rclcpp::SubscriptionOptions make_options();"),
                "on_in")
    assert on_in.publish_bounds == {"/out": None}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert "make_options" in on_in.publish_unbounded["/out"], on_in.publish_unbounded


def test_serialized_publish_on_an_alias_marks_every_publisher(tmp_path):
    node = _parse(tmp_path, "    rclcpp::SerializedMessage s;\n    auto p = out_;\n    p->publish(s);")
    flags = {p.topic: p.serialized_publish for p in node.publishers}
    assert flags == {"/out": True, "/aux": True}, flags


_GLOBAL_HOOK = """\
rclcpp::Publisher<std_msgs::msg::String> * g_pub = nullptr;
std::function<void()> g_hook;
void publish_via_global() {std_msgs::msg::String m; g_pub->publish(m);}"""


def test_publisher_stored_in_a_global_published_from_its_own_callback_leaves_no_bound(tmp_path):
    # g_hook() runs publish_via_global, a second /out publish of on_tick that
    # the call site does not show: storing out_ in g_pub is an escape.
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    g_hook();",
                      ctor="    g_pub = out_.get();\n    g_hook = publish_via_global;",
                      pre=_GLOBAL_HOOK), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert "stores a publisher in the global or static g_pub" in tick.publish_unbounded["/out"], (
        tick.publish_unbounded)


def test_publisher_stored_in_a_static_local_leaves_no_bound(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    g_hook();",
                      ctor="    static auto * keep = aux_.get();\n    g_pub = keep;",
                      pre=_GLOBAL_HOOK), "on_tick")
    assert tick.publish_bounds == {"/out": None}, (tick.publish_bounds, tick.publish_unbounded)
    assert "the global or static keep" in tick.publish_unbounded["/out"], tick.publish_unbounded


def test_value_stored_in_a_member_or_local_keeps_the_bound(tmp_path):
    tick = _cb(_parse(tmp_path, M + "    out_->publish(m);\n    auto * p = aux_.get();\n    (void)p;",
                      ctor="    alias_ = out_;", extra="  Pub alias_;\n"), "on_tick")
    assert tick.publish_bounds == {"/out": 1}, (tick.publish_bounds, tick.publish_unbounded)


# The correspondence_fixture sources the runtime cases build ---------------------


@pytest.mark.parametrize("cpp, cb, bounds, handler", [
    # One publish per tick.
    ("corr_publisher_one.cpp", "on_tick", {"/corr/a": 1}, []),
    # One relay publish in on_a, one in the deadline lambda: max(1, 1) = 1.
    ("corr_relay.cpp", "on_a", {"/corr/r": 1}, ["/corr/r"]),
    # Scenario "many" publishes /corr/a in a for loop. Scenario "nested" stores
    # pub_b_ in the global g_echo_pub, which publish_nested_echo publishes
    # through: an escape, so no bound for /corr/b either.
    ("corr_publisher.cpp", "on_tick", {"/corr/a": None, "/corr/b": None}, []),
])
def test_fixture_bounds(packages_src, packages_root, cpp, cb, bounds, handler):
    from colcon_lf.node_analysis.clang_parser import parse_node

    node = parse_node(packages_src / "correspondence_fixture" / "src" / cpp, workspace_root=packages_root)
    got = _cb(node, cb)
    assert (got.publish_bounds, got.missing_handler_publishers) == (bounds, handler), (
        f"{cpp} {cb}: {got.publish_bounds} {got.publish_unbounded} {got.missing_handler_publishers}")
    if cpp == "corr_publisher.cpp":
        assert "g_echo_pub" in got.publish_unbounded["/corr/b"], got.publish_unbounded

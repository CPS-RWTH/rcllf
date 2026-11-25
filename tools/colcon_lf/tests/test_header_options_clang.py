# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Subscription options built by a helper defined in a header.

``parse_node`` skipped the function bodies of the included headers,
so a helper such as brake_fusion's ``loss_options()`` had no visible
definition and the analyzer declared every publish of the subscription's
callback unbounded ("options of create_subscription ... come from
make_options, which has no visible body"). The analyzer now parses the
translation unit a second time, in full, to read such a helper's body. The
header here is written next to the node source and included first, so it
is part of the preamble whose bodies the first parse drops.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the header-options snippet tests")


_HEADER = """\
#pragma once
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using Pub = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
{globals}

inline rclcpp::SubscriptionOptions make_options(
  rclcpp::Clock::SharedPtr clock, const std::string & topic)
{{
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.deadline_callback =
    [clock, topic](rclcpp::QOSDeadlineRequestedInfo & info) {{
{handler}
    }};
  return options;
}}
"""

_NODE = """\
#include "header_options_helper.hpp"

class BoundNode : public rclcpp::Node
{{
public:
  BoundNode() : Node("bound_node")
  {{
    out_ = create_publisher<std_msgs::msg::String>("/out", 10);
{ctor}
  }}

private:
  void on_in(const std_msgs::msg::String & m) {{out_->publish(m);}}
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundNode>());
  rclcpp::shutdown();
  return 0;
}}
"""

_LOGGING_HANDLER = """\
      (void)info;
      RCLCPP_INFO(rclcpp::get_logger("bound"), "%s missed at %ld", topic.c_str(),
        static_cast<long>(clock->now().nanoseconds()));"""

_GLOBAL_PUBLISH_HANDLER = """\
      (void)info; (void)clock; (void)topic;
      std_msgs::msg::String m;
      g_pub->publish(m);"""

_DIRECT = """\
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1),
      make_options(this->get_clock(), "/in"));"""

_VARIABLE = """\
    rclcpp::SubscriptionOptions so = make_options(this->get_clock(), "/in");
    sub_ = create_subscription<std_msgs::msg::String>(
      "/in", 10, std::bind(&BoundNode::on_in, this, std::placeholders::_1), so);"""


def _parse(tmp_path: Path, ctor: str, handler: str, globals_: str = ""):
    from colcon_lf.node_analysis.clang_parser import parse_node

    (tmp_path / "header_options_helper.hpp").write_text(
        _HEADER.format(handler=handler, globals=globals_), encoding="utf-8")
    cpp = tmp_path / "bound_node.cpp"
    cpp.write_text(_NODE.format(ctor=ctor), encoding="utf-8")
    node = parse_node(cpp, workspace_root=tmp_path)
    return next(cb for cb in node.callbacks if cb.name == "on_in")


def test_header_helper_in_the_argument_list_is_followed(tmp_path):
    # The helper's deadline lambda only logs: the callback keeps its one site.
    on_in = _parse(tmp_path, _DIRECT, _LOGGING_HANDLER)
    assert on_in.publish_bounds == {"/out": 1}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert on_in.publish_unbounded == {}, on_in.publish_unbounded


def test_header_helper_initializing_an_options_variable_is_followed(tmp_path):
    # brake_fusion's form: `rclcpp::SubscriptionOptions so = loss_options(...)`.
    on_in = _parse(tmp_path, _VARIABLE, _LOGGING_HANDLER)
    assert on_in.publish_bounds == {"/out": 1}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert on_in.publish_unbounded == {}, on_in.publish_unbounded


def test_header_helper_whose_handler_publishes_keeps_the_callback_unbounded(tmp_path):
    # Reading the body must not make the analyzer trusting: a handler that
    # publishes through a global publisher is a publish the callback owns.
    on_in = _parse(tmp_path, _DIRECT, _GLOBAL_PUBLISH_HANDLER, globals_="extern Pub g_pub;")
    assert on_in.publish_bounds == {"/out": None}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert "publisher the analyzer cannot name" in on_in.publish_unbounded["/out"], (
        on_in.publish_unbounded)


def test_header_helper_defined_in_a_second_header_is_followed(tmp_path):
    # Prototype in one header, inline body in another (both in the preamble):
    # the resolver searches the declaring file first, then every other
    # non-system file of the unit.
    ctor = _DIRECT.replace("make_options(", "make_options_split(")
    (tmp_path / "header_options_impl.hpp").write_text(
        _HEADER.format(handler=_LOGGING_HANDLER, globals="")
        .replace("make_options(", "make_options_split("), encoding="utf-8")
    globals_ = (
        "rclcpp::SubscriptionOptions make_options_split("
        "rclcpp::Clock::SharedPtr clock, const std::string & topic);\n"
        "#include \"header_options_impl.hpp\"")
    on_in = _parse(tmp_path, ctor, _LOGGING_HANDLER, globals_=globals_)
    assert on_in.publish_bounds == {"/out": 1}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert on_in.publish_unbounded == {}, on_in.publish_unbounded


def test_header_helper_declared_but_defined_nowhere_stays_opaque(tmp_path):
    # No body anywhere: the second parse finds nothing and the old verdict stands.
    ctor = _DIRECT.replace("make_options(", "make_options_elsewhere(")
    on_in = _parse(
        tmp_path, ctor, _LOGGING_HANDLER,
        globals_="rclcpp::SubscriptionOptions make_options_elsewhere("
                 "rclcpp::Clock::SharedPtr clock, const std::string & topic);")
    assert on_in.publish_bounds == {"/out": None}, (on_in.publish_bounds, on_in.publish_unbounded)
    assert "make_options_elsewhere, which has no visible body" in on_in.publish_unbounded["/out"], (
        on_in.publish_unbounded)

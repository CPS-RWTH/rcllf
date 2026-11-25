# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests: the topic name comes from the topic argument only.

The classifier looks at the topic argument (index 0 for ``create_publisher``,
index 1 for the Nav2 wrappers), so the name must come from that same argument.
Before the fix the name was the first string literal anywhere in the call, so
``make_unique<TwistPublisher>(get_node("base"), "cmd_vel", qos)`` recorded
``"base"``, and a constant topic next to a logging lambda recorded the log text.
A static classification with no resolvable name must be dynamic (RCLLF-H004),
never an empty static topic.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the topic argument snippet tests")


_NODE = """\
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_util
{{
class TwistPublisher
{{
public:
  TwistPublisher(rclcpp::Node * node, const std::string & topic, const rclcpp::QoS & qos) {{}}
}};
}}  // namespace nav2_util

constexpr char kTopic[] = "cmd_vel";
constexpr char kSubTopic[] = "chatter";
{extra_decls}

class ArgNode : public rclcpp::Node
{{
public:
  ArgNode() : Node("arg_node")
  {{
    {body}
  }}

  rclcpp::Node * get_node(const std::string & name) {{ (void)name; return this; }}

private:
  std::unique_ptr<nav2_util::TwistPublisher> vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArgNode>());
  rclcpp::shutdown();
  return 0;
}}
"""


def _parse(tmp_path: Path, body: str, extra_decls: str = ""):
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "arg_node.cpp"
    cpp.write_text(_NODE.format(body=body, extra_decls=extra_decls), encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path)


def _topics(node):
    return [(p.topic, p.topic_dynamic) for p in node.publishers] + [
        (s.topic, s.topic_dynamic) for s in node.subscribers
    ]


def test_wrapper_topic_ignores_a_literal_in_the_node_argument(tmp_path):
    node = _parse(tmp_path, 'vel_pub_ = std::make_unique<nav2_util::TwistPublisher>('
                            'get_node("base"), "cmd_vel", rclcpp::QoS(1));')
    got = _topics(node)
    assert got == [("cmd_vel", False)], f"wrapper topic taken from the node argument: {got}"


def test_wrapper_constant_topic_resolves(tmp_path):
    node = _parse(tmp_path, 'vel_pub_ = std::make_unique<nav2_util::TwistPublisher>('
                            'get_node("base"), kTopic, rclcpp::QoS(1));')
    got = _topics(node)
    assert got == [("cmd_vel", False)], f"wrapper constant topic not resolved: {got}"


def test_index0_constant_topic_ignores_a_literal_in_the_callback(tmp_path):
    body = (
        'pub_ = create_publisher<std_msgs::msg::String>(kTopic, 10);\n'
        '    sub_ = create_subscription<std_msgs::msg::String>(kSubTopic, 10,\n'
        '      [this](std_msgs::msg::String::SharedPtr) {'
        ' RCLCPP_INFO(get_logger(), "got one"); });'
    )
    got = _topics(_parse(tmp_path, body))
    assert got == [("cmd_vel", False), ("chatter", False)], (
        f"index 0 constant topic taken from another argument: {got}"
    )


def test_static_classification_without_a_name_is_dynamic(tmp_path):
    # A constant whose initializer is an empty literal resolves to "": that is
    # no topic name, so it must raise H004 instead of an empty static topic.
    body = (
        'pub_ = create_publisher<std_msgs::msg::String>(kEmpty, 10);\n'
        '    vel_pub_ = std::make_unique<nav2_util::TwistPublisher>(this, kEmpty, rclcpp::QoS(1));'
    )
    got = _topics(_parse(tmp_path, body, extra_decls='constexpr char kEmpty[] = "";'))
    assert ("", False) not in got, f"empty static topic with no H004: {got}"
    assert got == [("", True), ("", True)], got

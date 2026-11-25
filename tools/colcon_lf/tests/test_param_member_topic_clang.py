# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests: a topic held in a member set from a declared parameter.

A node that stores ``declare_parameter<std::string>("twin_topic", "")`` in a
private member in its constructor and later subscribes on that member reads
the launch value of ``twin_topic``. The parser records the parameter name in
``topic_param`` so the launch overlay can fill in the topic. Before the fix the
row stayed ``''`` and dynamic while the parameter row carried the launch value,
so correspondence and the running node disagreed on the reader's twins.

Any other write to the member (a second assignment, ``get_parameter`` into it,
a mutable reference handed out) or a subscription made before the write keeps
the topic unresolved: the member could then hold a value the launch file does
not name.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the parameter member topic snippet tests")


_NODE = """\
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int64.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ParamNode : public rclcpp_lifecycle::LifecycleNode
{{
public:
  ParamNode() : LifecycleNode("reader"){init}
  {{
    {ctor}
  }}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {{
    {configure}
    return CallbackReturn::SUCCESS;
  }}

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {{
    twin_sub_.reset();
    return CallbackReturn::SUCCESS;
  }}

  void on_twin(const std_msgs::msg::Int64::SharedPtr msg) {{ (void)msg; }}
  {methods}

{access}:
  {field}
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr twin_sub_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamNode>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}}
"""

_DECLARE = 'twin_topic_ = this->declare_parameter<std::string>("twin_topic", "");'
_SUBSCRIBE = ('twin_sub_ = this->create_subscription<std_msgs::msg::Int64>(twin_topic_, 10, '
              'std::bind(&ParamNode::on_twin, this, std::placeholders::_1));')


def _twin(tmp_path: Path, ctor: str = _DECLARE, configure: str = _SUBSCRIBE,
          methods: str = "", init: str = "", access: str = "private",
          field: str = "std::string twin_topic_;"):
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "param_node.cpp"
    cpp.write_text(_NODE.format(ctor=ctor, configure=configure, methods=methods,
                                init=init, access=access, field=field), encoding="utf-8")
    subs = [s for s in parse_node(cpp, workspace_root=tmp_path).subscribers
            if s.callback.endswith("on_twin")]
    assert len(subs) == 1, f"expected one on_twin subscription, got {subs}"
    s = subs[0]
    return s.topic, s.topic_dynamic, getattr(s, "topic_param", None)


@pytest.mark.parametrize("ctor, configure, init", [
    (_DECLARE, _SUBSCRIBE, ""),
    (_DECLARE + "\n    " + _SUBSCRIBE, "", ""),
    ("", _SUBSCRIBE,
     ', twin_topic_(this->declare_parameter<std::string>("twin_topic", ""))'),
], ids=["hook_subscribes", "constructor_subscribes_after_write", "member_initializer"])
def test_member_set_once_from_a_declared_parameter_names_the_parameter(
        tmp_path, ctor, configure, init):
    got = _twin(tmp_path, ctor=ctor, configure=configure, init=init)
    assert got == ("", True, "twin_topic"), (
        f"on_twin (topic, topic_dynamic, topic_param) is {got}: the topic member is "
        "written once, in the constructor, from declare_parameter(\"twin_topic\"), "
        "so the row must name that parameter for the launch overlay")


@pytest.mark.parametrize("kwargs", [
    {"methods": 'void retopic() { twin_topic_ = "/other"; }'},
    {"configure": 'this->get_parameter("twin_topic", twin_topic_);\n    ' + _SUBSCRIBE},
    {"methods": "std::string & topic() { return twin_topic_; }"},
    {"ctor": _SUBSCRIBE + "\n    " + _DECLARE, "configure": ""},
    {"ctor": 'twin_topic_ = std::getenv("TWIN") ? std::getenv("TWIN") : "";'},
    {"access": "public"},
    {"methods": "friend void retopic(ParamNode & n);"},
    {"methods": "struct Retopic { void run(ParamNode & n) { n.twin_topic_ = \"/other\"; } };"},
], ids=["second_write", "get_parameter_into_member", "mutable_reference",
        "subscribed_before_write", "not_a_parameter", "public_member", "friend", "nested_class"])
def test_member_the_parameter_does_not_pin_stays_unresolved(tmp_path, kwargs):
    got = _twin(tmp_path, **kwargs)
    assert got == ("", True, None), (
        f"on_twin (topic, topic_dynamic, topic_param) is {got}: the member can hold a "
        "value the launch parameter does not name, so the topic must stay unresolved")


# Each case below was accepted on 8bf64025 although
# the proof is incomplete. Never guess: the topic stays unresolved.
_IN_IF = "if (get_name()[0] == 'r') {\n      " + _DECLARE + "\n    }"
_RETURN_FIRST = "if (get_name()[0] != 'r') {return;}\n    " + _DECLARE


@pytest.mark.parametrize("kwargs", [
    # 1. The write does not run on every construction path.
    {"methods": 'explicit ParamNode(int) : LifecycleNode("reader2") {}'},
    {"ctor": _IN_IF},
    {"ctor": _RETURN_FIRST},
    # 2. A method that may write the member has no body in the parsed file.
    {"methods": "void retopic();"},
    # 3. Not a plain std::string value member.
    {"field": "mutable std::string twin_topic_;", "methods": "void peek() const;"},
    {"field": "const std::string & twin_topic_;", "ctor": "",
     "init": ', twin_topic_(this->declare_parameter<std::string>("twin_topic", ""))'},
    # 4. A use the classifier does not prove read-only is a write.
    {"methods": 'void retopic() { const_cast<std::string &>(twin_topic_) = "/other"; }'},
    {"methods": "void drain() { std::string t = static_cast<std::string &&>(twin_topic_); }"},
    {"methods": 'void retopic(bool b) { std::string o; (b ? twin_topic_ : o) = "/other"; }'},
], ids=["other_constructor", "write_inside_if", "return_before_write", "method_without_body",
        "mutable_member", "reference_member", "const_cast", "static_cast_move",
        "conditional_operator"])
def test_incomplete_proof_leaves_the_topic_unresolved(tmp_path, kwargs):
    got = _twin(tmp_path, **kwargs)
    assert got == ("", True, None), (
        f"on_twin (topic, topic_dynamic, topic_param) is {got}: the analysis cannot prove "
        "the member holds the launch value of twin_topic when the subscription is made, "
        "so it must not name the parameter")

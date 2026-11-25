# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Review round 5, WP-C: inheritance, #ifdef mains, late endpoints, main() work.

Each test names the message it gave before the fix (analysis of a node whose
endpoints live in a base class found no subscriber; a node built from an
#ifdef main analyzed the wrong class; late endpoints and main() work were
invisible, so nothing was refused).
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

_HEAD = (
    "#include <functional>\n#include <memory>\n#include <chrono>\n"
    "#include \"rclcpp/rclcpp.hpp\"\n#include \"std_msgs/msg/int32.hpp\"\n"
    "using Int32 = std_msgs::msg::Int32;\n"
)


def _parse(tmp_path: Path, name: str, body: str, **kw):
    from colcon_lf.node_analysis.clang_parser import clear_parse_node_cache, parse_node

    clear_parse_node_cache()
    cpp = tmp_path / name
    cpp.write_text(_HEAD + body, encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path, **kw)


_PLAIN_BASE = """
class Base : public rclcpp::Node {
public:
  Base(const std::string & n) : Node(n) {
    sub_ = create_subscription<Int32>("in", 10,
      std::bind(&Base::on_in, this, std::placeholders::_1));
    pub_ = create_publisher<Int32>("out", 10);
  }
protected:
  virtual int f(int x) = 0;
private:
  void on_in(Int32::ConstSharedPtr m) { Int32 o; o.data = f(m->data); pub_->publish(o); }
  rclcpp::Subscription<Int32>::SharedPtr sub_;
  rclcpp::Publisher<Int32>::SharedPtr pub_;
};
class Derived : public Base {
public:
  Derived() : Base("derived") {}
protected:
  int f(int x) override { return x + 1000; }
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Derived>());
}
"""


@requires_ros_headers("base class walk")
def test_endpoints_in_workspace_base_are_extracted(tmp_path: Path):
    # Before: assert [] == ['in'] (the subscription lives in Base's constructor).
    info = _parse(tmp_path, "plain.cpp", _PLAIN_BASE)
    assert info.class_name.endswith("Derived")
    assert [s.topic for s in info.subscribers] == ["in"]
    assert [p.topic for p in info.publishers] == ["out"]
    assert info.base_classes_merged == ["Base"]
    assert not info.template_base_endpoints


_TEMPLATE_BASE = """
template<typename MsgT>
class EchoBase : public rclcpp::Node {
public:
  EchoBase(const std::string & n) : Node(n) {
    sub_ = this->template create_subscription<MsgT>("in", 10,
      std::bind(&EchoBase::on_in, this, std::placeholders::_1));
  }
private:
  void on_in(typename MsgT::ConstSharedPtr) {}
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
};
class Derived : public EchoBase<Int32> {
public:
  Derived() : EchoBase("derived") {}
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Derived>());
}
"""


@requires_ros_headers("template base")
def test_template_base_endpoints_become_h015_fact(tmp_path: Path):
    # Before: the CLASS_TEMPLATE was picked as the node and nothing was refused.
    info = _parse(tmp_path, "tmpl.cpp", _TEMPLATE_BASE)
    assert info.class_name.endswith("Derived")
    assert [f["base"] for f in info.template_base_endpoints] == ["EchoBase"]


_LATE = """
class Late : public rclcpp::Node {
public:
  Late() : Node("late") {
    t_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Late::tick, this));
  }
private:
  void tick() {
    sub_ = create_subscription<Int32>("later", 10, [this](Int32::ConstSharedPtr) {});
  }
  rclcpp::TimerBase::SharedPtr t_;
  rclcpp::Subscription<Int32>::SharedPtr sub_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Late>());
}
"""


@requires_ros_headers("late endpoint")
def test_subscription_created_in_method_is_unbound_fact(tmp_path: Path):
    # Before: assert [] == [('subscription', 'later', 'tick')] (silently dropped).
    info = _parse(tmp_path, "late.cpp", _LATE)
    assert [(e["kind"], e["topic"], e["method"]) for e in info.unbound_endpoints] == [
        ("subscription", "later", "tick")]
    assert info.unbound_endpoints[0]["callback"].startswith("lambda at late.cpp:")
    assert info.subscribers == []


def test_h013_message_text():
    from colcon_lf.features.modules import endpoint_created_in_callback as m

    node = SimpleNamespace(unbound_endpoints=[
        {"kind": "subscription", "topic": "later", "method": "tick", "callback": ""}])
    ctx = SimpleNamespace(graph=SimpleNamespace(nodes={"late": node}))
    [d] = m.detect(ctx)
    assert d.code == "RCLLF-H013"
    assert d.message == (
        "node 'late' creates subscription 'later' in 'tick'; under rcllf its "
        "callback never runs. Create it in the constructor.")


def test_h014_message_text():
    from colcon_lf.features.modules import main_outside_callbacks as m

    node = SimpleNamespace(main_outside_callbacks=[
        {"source": "mt_main.cpp", "call": "publish_from_main"}])
    ctx = SimpleNamespace(graph=SimpleNamespace(nodes={"mt_main": node}))
    [d] = m.detect(ctx)
    assert d.code == "RCLLF-H014"
    assert d.message == (
        "main() of mt_main.cpp runs node work outside callbacks "
        "(publish_from_main); rcllf replaces main(), so it never runs. "
        "Move it into a timer.")


_MAIN_BODY = """
class Loop : public rclcpp::Node {
public:
  Loop() : Node("loop") { pub_ = create_publisher<Int32>("out", 10); }
  void publish_from_main() { Int32 m; pub_->publish(m); }
private:
  rclcpp::Publisher<Int32>::SharedPtr pub_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Loop>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok()) {
    exec.spin_some();
    %s
  }
}
"""


@requires_ros_headers("main outside callbacks")
def test_publishing_call_in_spin_some_loop_is_h014_fact(tmp_path: Path):
    # Before: assert [] == [{'source': 'm1.cpp', 'call': 'publish_from_main'}].
    info = _parse(tmp_path, "m1.cpp", _MAIN_BODY % "node->publish_from_main();")
    assert info.main_outside_callbacks == [
        {"source": "m1.cpp", "call": "publish_from_main"}]


@requires_ros_headers("main outside callbacks")
def test_plain_spin_some_is_not_flagged(tmp_path: Path):
    info = _parse(tmp_path, "m2.cpp", _MAIN_BODY % "")
    assert info.main_outside_callbacks == []


_IFDEF = """
class A : public rclcpp::Node {
public:
  A() : Node("a") { p_ = create_publisher<Int32>("a_out", 10); }
  rclcpp::Publisher<Int32>::SharedPtr p_;
};
class B : public rclcpp::Node {
public:
  B() : Node("b") { p_ = create_publisher<Int32>("b_out", 10); }
  rclcpp::Publisher<Int32>::SharedPtr p_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
#ifdef USE_B
  rclcpp::spin(std::make_shared<B>());
#else
  rclcpp::spin(std::make_shared<A>());
#endif
}
"""


@requires_ros_headers("ifdef main")
def test_defines_select_the_class_and_cache_is_keyed_by_defines(tmp_path: Path):
    # Before: the second parse returned the cached class A for the USE_B target.
    from colcon_lf.node_analysis.clang_parser import clear_parse_node_cache, parse_node

    cpp = tmp_path / "var.cpp"
    cpp.write_text(_HEAD + _IFDEF, encoding="utf-8")
    clear_parse_node_cache()
    a = parse_node(cpp, workspace_root=tmp_path)
    b = parse_node(cpp, workspace_root=tmp_path, extra_defines=["-DUSE_B"])
    assert a.class_name.endswith("A")
    assert b.class_name.endswith("B")

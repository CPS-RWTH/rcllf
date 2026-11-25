# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests: the xtag publisher count stays exact only when it is.

The reader federate of a cross-federate tag link waits for the number of gated
publishers the analysis counts (``PublisherInfo.unconditional is True``, see
``test_publisher_unconditional_clang.py`` for the branch, loop and lambda
cases). Three more cases made that count wrong:

1. a macro that spells ``&&`` / ``||`` hid the short circuit (overcount);
2. a backward ``goto`` can run a create call twice (undercount);
3. a publisher the node does not keep alive (a local, a discarded result, a member
   set twice or reset, also by code whose body is in another .cpp) is still counted
   (overcount).

Each now leaves the publisher not counted as made on every path, so the link keeps
the count rule. Positive controls show that a publisher kept in a member stays
exact.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the publisher count snippet tests")


_NODE = """\
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using Pub = rclcpp::Publisher<std_msgs::msg::Int64>;
{prelude}
class DataNode : public rclcpp::Node
{{
public:
  DataNode() : Node("data_node"){init}
  {{
    const bool twin = this->declare_parameter<bool>("twin", false);
    (void)twin;
    pub_ = this->create_publisher<std_msgs::msg::Int64>("data", 10);
{body}
  }}

  void tick()
  {{
    std_msgs::msg::Int64 msg;
    pub_->publish(msg);
{method}
  }}
{decls}

private:
  Pub::SharedPtr pub_;
  Pub::SharedPtr twin_;
  std::weak_ptr<Pub> weak_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataNode>());
  rclcpp::shutdown();
  return 0;
}}
"""

_CREATE = 'this->create_publisher<std_msgs::msg::Int64>("data", 10)'
_TWIN = f"twin_ = {_CREATE};"

# case -> (constructor initializer, constructor body, tick() body, expected flag of
# every "data" publisher after the first, expected flag of the first)
CASES = {
    # A macro spelling the operator, or holding the whole
    # short circuit. The AST opcode is &&, so the create call is conditional.
    "macro_and": ("", f"#define AND &&\n    twin AND ({_TWIN});", "", False, True),
    "macro_whole": ("", f"#define MAYBE(x) (twin && (x))\n    MAYBE({_TWIN});", "", False, True),
    # Item 3: a backward goto runs the create call again.
    "goto_backward": (
        "", f"    int round = 0;\nagain:\n    {_TWIN}\n    if (++round < 2) {{\n"
        "      goto again;\n    }", "", None, None),
    # Item 1: lifetime.
    "local": ("", f"    auto local = {_CREATE};\n    (void)local;", "", None, True),
    "discarded": ("", f"    {_CREATE};", "", None, True),
    "set_twice": ("", f"    {_TWIN}\n    {_TWIN}", "", None, True),
    "reset_in_constructor": ("", f"    {_TWIN}\n    twin_.reset();", "", None, True),
    "reset_in_method": ("", f"    {_TWIN}", "    twin_.reset();", None, True),
    "moved_out_in_method": ("", f"    {_TWIN}", "    auto gone = std::move(twin_);", None, True),
    # A weak_ptr member does not keep the publisher, and a
    # scope guard's destructor (not the node's) may reset the member at once.
    "weak_member": ("", f"    weak_ = {_CREATE};", "", None, True),
    "reset_by_guard": (
        "", "    struct Guard { DataNode * n; ~Guard() { n->twin_.reset(); } };\n"
        f"    {_TWIN}\n    Guard guard{{this}};", "", None, True),
    "reset_through_alias": (
        "", f"    {_TWIN}", "    auto & alias = twin_;\n    alias.reset();", None, True),
    # Positive controls: kept in a member and only read (a call through it,
    # .get(), a dereference as in sentry_pipeline's PublishLog::open, a copy).
    "kept": ("", f"    {_TWIN}", "    twin_->publish(msg);", True, True),
    "kept_this_arrow": ("", f"    this->{_TWIN}", "    (void)twin_.get();", True, True),
    "kept_initializer": (f",\n    twin_({_CREATE})", "", "    twin_->publish(msg);", True, True),
    "kept_dereferenced": (
        "", f"    {_TWIN}\n    const Pub & ref = *twin_;\n    (void)ref;", "", True, True),
    "kept_copied": ("", f"    {_TWIN}", "    auto copy = twin_;\n    copy->publish(msg);",
                    True, True),
    # Round 2 of the orchestrator review: a truth test or a comparison only reads.
    "kept_truth_test": ("", f"    {_TWIN}", "    if (twin_) {\n      twin_->publish(msg);\n    }",
                        True, True),
    "kept_negated": ("", f"    {_TWIN}", "    if (!twin_) {\n      return;\n    }", True, True),
    "kept_cast_bool": ("", f"    {_TWIN}", "    (void)static_cast<bool>(twin_);", True, True),
    "kept_not_null": ("", f"    {_TWIN}", "    (void)(twin_ != nullptr);", True, True),
    "kept_is_null": ("", f"    {_TWIN}", "    (void)(nullptr == twin_);", True, True),
    "kept_use_count": ("", f"    {_TWIN}", "    (void)twin_.use_count();", True, True),
    "reset_after_truth_test": (
        "", f"    {_TWIN}", "    if (twin_) {\n      twin_.reset();\n    }", None, True),
    # Code whose body is in another .cpp may reset any member (round 2): every
    # publisher of the node is then undecided. An unreferenced private method
    # cannot.
    "unseen_method": ("", f"    {_TWIN}", "", None, None),
    "unseen_friend": ("", f"    {_TWIN}", "", None, None),
    "unseen_nested": ("", f"    {_TWIN}", "", None, None),
    "unseen_private_unused": ("", f"    {_TWIN}", "", True, True),
    # A pure virtual method may have a body in another .cpp; a const
    # method may reset a member through const_cast<DataNode *>(this); a
    # destructor may reset another instance's member. Each is unseen code now.
    "unseen_pure_virtual": ("", f"    {_TWIN}", "", None, None),
    "unseen_const_method": ("", f"    {_TWIN}", "", None, None),
    "unseen_destructor": ("", f"    {_TWIN}", "", None, None),
    # Seen bodies: a project operator* may reset its operand; the
    # node's destructor may reset a peer's member, while its own members go
    # with it; a seen const_cast reset is a write.
    "project_dereference": (
        "", f"    {_TWIN}", "    const Pub & ref = *twin_;\n    (void)ref;", None, True),
    "destructor_resets_peer": ("", f"    {_TWIN}", "", None, True),
    "destructor_resets_own": ("", f"    {_TWIN}", "", True, True),
    "seen_const_cast": ("", f"    {_TWIN}", "", None, True),
    # A const method may reset a mutable member, any code
    # may reset a public one, and a project operator== may reset its operand.
    "unseen_const_mutable": ("", f"    held_ = {_CREATE};", "", None, None),
    "public_member": ("", f"    open_ = {_CREATE};", "", None, True),
    "project_comparison": ("", f"    {_TWIN}", "    (void)(twin_ == nullptr);", None, True),
}

# case -> declarations in the class body after tick(); their definitions are not
# in the parsed file (a sibling .cpp of the package would hold them).
DECLS = {
    "unseen_method": "  void stop();",
    "unseen_friend": "  friend void stop_node(DataNode & node);",
    "unseen_nested": "  struct Stopper\n  {{\n    void stop(DataNode & node);\n  }};",
    "unseen_const_method": "  int count() const;",
    "unseen_destructor": "  ~DataNode() override;",
    "unseen_pure_virtual": "  virtual void clear() = 0;",
    "destructor_resets_peer": (
        "  ~DataNode() override\n  {{\n    if (peer_ != nullptr) {{\n"
        "      peer_->twin_.reset();\n    }}\n  }}\n  DataNode * peer_ = nullptr;"),
    "destructor_resets_own": (
        "  ~DataNode() override\n  {{\n    twin_.reset();\n    this->pub_.reset();\n  }}"),
    "seen_const_cast": (
        "  int count() const\n  {{\n    const_cast<DataNode *>(this)->twin_.reset();\n"
        "    return 0;\n  }}"),
    "unseen_private_unused": "\nprivate:\n  void helper();\n\npublic:",
    "unseen_const_mutable": (
        "  int count() const;\n\nprivate:\n  mutable Pub::SharedPtr held_;\n\npublic:"),
    "public_member": "  Pub::SharedPtr open_;",
}

# case -> namespace-scope code before the class.
PRELUDE = {
    "project_comparison": (
        "\n// Chosen over std's template for a non-const operand; it resets the member.\n"
        "bool operator==(Pub::SharedPtr & p, std::nullptr_t)\n{\n  p.reset();\n"
        "  return true;\n}\n"),
    "project_dereference": (
        "\n// Chosen over std's operator* for a non-const operand; it resets the member.\n"
        "Pub & operator*(Pub::SharedPtr & p)\n{\n  Pub & obj = *p.get();\n  p.reset();\n"
        "  return obj;\n}\n"),
}


def _parse(tmp_path: Path, case: str):
    from colcon_lf.node_analysis.clang_parser import parse_node

    init, body, method, _rest, _first = CASES[case]
    cpp = tmp_path / "data_node.cpp"
    cpp.write_text(_NODE.format(init=init, body=body, method=method,
                                decls=DECLS.get(case, "").format(),
                                prelude=PRELUDE.get(case, "")), encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path)


@pytest.mark.parametrize("case", sorted(CASES))
def test_publisher_count_flag(tmp_path, case):
    *_, rest_expected, first_expected = CASES[case]
    node = _parse(tmp_path, case)
    pubs = [p for p in node.publishers if p.topic == "data"]
    assert len(pubs) >= 2, [dataclasses.asdict(p) for p in node.publishers]
    first = next(p for p in pubs if p.member_variable == "pub_")
    rest = [p for p in pubs if p is not first]
    assert all(p.created_in == "constructor" for p in pubs), case
    got = [p.unconditional for p in rest]
    harm = ("a None here drops an exact xtag publisher count the node has"
            if rest_expected is True else
            "a True here makes the xtag publisher count exact when it is not")
    assert got == [rest_expected] * len(rest), (
        f"{case}: second publisher of /data recorded unconditional={got}, expected "
        f"{rest_expected}; {harm}")
    assert first.unconditional is first_expected, (case, first.unconditional)


def test_unreadable_operator_kind_is_undecided(tmp_path, monkeypatch):
    """Without the AST opcode, a macro operator between the operands is not read as plain."""
    from colcon_lf.node_analysis import clang_parser

    monkeypatch.setattr(clang_parser, "_ast_binary_operator", lambda _c: None, raising=False)
    node = _parse(tmp_path, "macro_and")
    second = [p for p in node.publishers if p.member_variable == "twin_"]
    assert [p.unconditional for p in second] == [None], (
        "macro_and without the AST opcode: the AND token between the operands was read "
        f"as a plain operator: {[p.unconditional for p in second]}")


def _xtag_lf(tmp_path: Path, case: str) -> str:
    """Generate the pa -> sub cross-federate link with pa's parsed publishers."""
    from .generation.test_correspondence_emission import _graph, _tc
    from .generation.test_fan_in_tag_split import _nodes
    from .generation.test_xtag_link import APART, _lf, _run

    snippet = tmp_path / "snippet"
    snippet.mkdir()
    parsed = _parse(snippet, case)
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = list(parsed.publishers)
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    return _lf(tmp_path, "sub")


@pytest.mark.parametrize("case,reason", [
    ("macro_and", "may skip creating a publisher of /data"),
    ("goto_backward", "did not decide whether its constructor always creates its publisher "
                      "of /data"),
    ("local", "did not decide whether its constructor always creates its publisher of /data"),
])
def test_inexact_count_keeps_the_count_rule(tmp_path, case, reason):
    sub = _xtag_lf(tmp_path, case)
    assert "ros2_lf_xtag_expect" not in sub, (
        f"{case}: the generated reader expects an exact publisher count\n{sub}")
    assert "carries only a count (publisher count not exact: conditional creation" in sub, sub
    assert reason in sub, sub


def test_kept_publishers_are_still_expected(tmp_path):
    sub = _xtag_lf(tmp_path, "kept")
    assert sub.count('ros2_lf_xtag_expect("/data", 2);') == 1, sub

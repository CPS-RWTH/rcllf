# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""declare_parameter defaults keep their sign.

`_extract_literal_value` used to skip the unary minus, so a default of -1 was
recorded as 1. Each snippet's expected value is read off its own source text.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the declare_parameter default snippet tests")

_NODE = """\
#include "rclcpp/rclcpp.hpp"

static constexpr int64_t kConst = 5;

class ParamNode : public rclcpp::Node
{{
public:
  ParamNode() : Node("param_node")
  {{
{decls}
  }}
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamNode>());
  rclcpp::shutdown();
  return 0;
}}
"""


def _defaults(tmp_path: Path, decls: str) -> dict:
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "param_node.cpp"
    cpp.write_text(_NODE.format(decls=decls), encoding="utf-8")
    node = parse_node(cpp, workspace_root=tmp_path)
    return {p.name: p.default_value for p in node.parameters}


def test_signed_literals_keep_their_sign(tmp_path):
    got = _defaults(
        tmp_path,
        '    declare_parameter<int64_t>("neg_int", -1);\n'
        '    declare_parameter<double>("neg_dbl", -1.0);\n'
        '    declare_parameter<int64_t>("pos_int", 7);\n'
        '    declare_parameter<int64_t>("plus_int", +3);\n'
        '    declare_parameter<double>("paren_neg", -(2.5));\n',
    )
    expect = {"neg_int": -1, "neg_dbl": -1.0, "pos_int": 7, "plus_int": 3, "paren_neg": -2.5}
    assert got == expect, f"GOT {got}\nEXPECT {expect}"


def test_expressions_it_cannot_evaluate_stay_unresolved(tmp_path):
    got = _defaults(
        tmp_path,
        '    declare_parameter<int64_t>("neg_const", -kConst);\n'
        '    declare_parameter<int64_t>("binary", 1 - 2);\n'
        '    declare_parameter<int64_t>("not_int", ~1);\n',
    )
    expect = {"neg_const": None, "binary": None, "not_int": None}
    assert got == expect, f"GOT {got}\nEXPECT {expect}"

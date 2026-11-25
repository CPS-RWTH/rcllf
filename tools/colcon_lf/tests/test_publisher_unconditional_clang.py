# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests for ``PublisherInfo.unconditional``.

The receiver of a cross-federate tag link waits for an exact number of
announced publishers. A constructor that creates a second publisher of the
topic only when a parameter is set makes that count an overcount, and the
samples of a writer outside rcllf are held for good. The parser records
whether each create call runs on every path through the constructor; the
generator counts only publishers that do.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the publisher unconditional snippet tests")


_NODE = """\
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class DataNode : public rclcpp::Node
{{
public:
  DataNode() : Node("data_node")
  {{
    const bool twin = this->declare_parameter<bool>("twin", false);
    (void)twin;
    pub_ = this->create_publisher<std_msgs::msg::Int64>("data", 10);
{body}
  }}

private:
  void make_twin()
  {{
    twin_ = this->create_publisher<std_msgs::msg::Int64>("data", 10);
  }}

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr twin_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr> many_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataNode>());
  rclcpp::shutdown();
  return 0;
}}
"""

_TWIN = 'twin_ = this->create_publisher<std_msgs::msg::Int64>("data", 10);'

# body -> whether the second publisher is created on every path
CASES = {
    "plain": (f"    {_TWIN}", True),
    "if": (f"    if (twin) {{\n      {_TWIN}\n    }}", False),
    "ternary": ('    twin_ = twin ? this->create_publisher<std_msgs::msg::Int64>("data", 10)'
                " : nullptr;", False),
    "lambda": (f"    auto make = [this]() {{\n      {_TWIN}\n    }};\n    make();", False),
    "loop": ("    for (int i = 0; i < 2; ++i) {\n"
             '      many_.push_back(this->create_publisher<std_msgs::msg::Int64>("data", 10));\n'
             "    }", False),
    "switch": (f"    switch (static_cast<int>(twin)) {{\n      case 1: {_TWIN} break;\n"
               "      default: break;\n    }", False),
    "short_circuit": ('    twin && (twin_ = this->create_publisher<std_msgs::msg::Int64>'
                      '("data", 10));', False),
    "early_return": (f"    if (!twin) {{\n      return;\n    }}\n    {_TWIN}", False),
    "try": (f"    try {{\n      {_TWIN}\n    }} catch (...) {{\n    }}", False),
    "helper_plain": ("    make_twin();", True),
    "helper_in_if": ("    if (twin) {\n      make_twin();\n    }", False),
    "return_in_lambda_only": (
        f"    auto f = []() {{ return 1; }};\n    (void)f;\n    {_TWIN}", True),
}


def _parse(tmp_path: Path, body: str):
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "data_node.cpp"
    cpp.write_text(_NODE.format(body=body), encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path)


@pytest.mark.parametrize("case", sorted(CASES))
def test_second_publisher_unconditional_flag(tmp_path, case):
    body, expected = CASES[case]
    node = _parse(tmp_path, body)
    pubs = [p for p in node.publishers if p.topic == "data"]
    assert len(pubs) == 2, [dataclasses.asdict(p) for p in node.publishers]
    first, second = pubs
    assert first.created_in == "constructor"
    assert first.unconditional is True, case
    assert second.created_in == "constructor"
    assert second.unconditional is expected, case


def _xtag_lf(tmp_path: Path, body: str) -> str:
    """Generate the pa -> sub cross-federate link with pa's parsed publishers."""
    from .generation.test_correspondence_emission import _graph, _tc
    from .generation.test_fan_in_tag_split import _nodes
    from .generation.test_xtag_link import APART, _lf, _run

    snippet = tmp_path / "snippet"
    snippet.mkdir()
    parsed = _parse(snippet, body)
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = list(parsed.publishers)
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    return _lf(tmp_path, "sub")


def test_publisher_made_only_when_a_parameter_is_set_keeps_the_count_rule(tmp_path):
    sub = _xtag_lf(tmp_path, CASES["if"][0])
    assert "ros2_lf_xtag_expect" not in sub, sub
    assert "input data: stub_t" in sub, sub
    assert ("cross-federate take link pa.data -> sub.data on topic '/data' carries only a "
            "count (publisher count not exact: conditional creation: node pa may skip "
            "creating a publisher of /data") in sub, sub


def test_two_publishers_made_on_every_path_are_expected(tmp_path):
    sub = _xtag_lf(tmp_path, CASES["plain"][0])
    assert sub.count('ros2_lf_xtag_expect("/data", 2);') == 1, sub
    assert "carries only a count" not in sub, sub

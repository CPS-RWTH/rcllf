# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A timer literal reaches the binding as its exact nanoseconds.

The binding binds a timer handle only to the RID of the same period in whole
nanoseconds, and the generator floors the decimal of the analyzed period_ms
(policies.period_ms_to_ns). The analyzer used to scale a literal by a float
product, 5 * 1e-6 = 4.9999999999999996e-06 ms, which floors to 4 ns, so a
``5ns`` timer had a 4 ns RID, no handle fit it and the node refused to start.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.exec_config.policies import period_ms_to_ns
from colcon_lf.node_analysis.clang_parser import _parse_chrono_ms

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

# Literals whose old float product floored 1 ns short (found by a sweep).
_SHORT = ["5", "10", "15", "19", "20", "25"]


@pytest.mark.parametrize(("unit", "scale"), [("ns", 1), ("us", 1_000), ("ms", 1_000_000)])
def test_every_literal_up_to_100000_keeps_its_nanoseconds(unit, scale):
    wrong = [
        n for n in range(1, 100_001)
        if period_ms_to_ns(_parse_chrono_ms([str(n), unit])) != n * scale
    ]
    assert not wrong, f"{len(wrong)} {unit} literals lost a nanosecond, first {wrong[:6]}"


@pytest.mark.parametrize("literal", ["10000000000000001", "9007199254740993", "123456789012345678"])
def test_a_literal_no_float_carries_is_unresolved_not_rounded(literal):
    # Past about 16 significant digits the nearest float of the exact ms is
    # another value, so its RID would be ns off the timer it names.
    ms = _parse_chrono_ms([literal, "ns"])
    assert ms is None or period_ms_to_ns(ms) == int(literal), (
        f"a {literal} ns literal reaches the binding as {period_ms_to_ns(ms)} ns")


def test_a_long_literal_a_float_carries_still_resolves():
    assert period_ms_to_ns(_parse_chrono_ms(["1000000000000000", "ns"])) == 10**15


@requires_ros_headers("the nanosecond timer snippet test")
def test_parsed_nanosecond_timer_keeps_its_period(tmp_path: Path):
    from colcon_lf.node_analysis.clang_parser import parse_node

    timers = "\n".join(
        f"    t{i}_ = create_wall_timer({n}ns, std::bind(&NsNode::on_{i}, this));"
        for i, n in enumerate(_SHORT))
    callbacks = "\n".join(f"  void on_{i}() {{}}" for i in range(len(_SHORT)))
    members = "\n".join(f"  rclcpp::TimerBase::SharedPtr t{i}_;" for i in range(len(_SHORT)))
    cpp = tmp_path / "ns_node.cpp"
    cpp.write_text(
        "#include <chrono>\n#include <functional>\n#include \"rclcpp/rclcpp.hpp\"\n"
        "using namespace std::chrono_literals;\n"
        "class NsNode : public rclcpp::Node\n{\npublic:\n  NsNode() : Node(\"ns_node\")\n  {\n"
        f"{timers}\n  }}\n\nprivate:\n{callbacks}\n{members}\n}};\n",
        encoding="utf-8")
    info = parse_node(cpp, workspace_root=tmp_path)
    got = {t.callback: period_ms_to_ns(t.period_ms) for t in info.timers}
    # Expected: each literal's own nanoseconds, from the snippet.
    want = {f"on_{i}": int(n) for i, n in enumerate(_SHORT)}
    assert got == want, f"timer RID periods {got}, the literals say {want}"

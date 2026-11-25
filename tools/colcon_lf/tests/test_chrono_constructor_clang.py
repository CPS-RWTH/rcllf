# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A timer period written as a std::chrono duration constructor resolves.

tf_broadcaster_example's dynamic node creates its timer with
``std::chrono::milliseconds(33)``. The analyzer only read ``33ms`` literals
and rclcpp::Duration factories, so the period stayed null and generation
refused it with RCLLF-R001 (found while checking the tf2 packages).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.exec_config.policies import period_ms_to_ns

pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

REPO = Path(__file__).resolve().parents[3]

# (argument as written, its nanoseconds derived by hand from the unit)
_CASES = [
    ("std::chrono::milliseconds(33)", 33_000_000),
    ("std::chrono::microseconds(250)", 250_000),
    ("std::chrono::nanoseconds(5)", 5),
    ("std::chrono::seconds(2)", 2_000_000_000),
    ("std::chrono::minutes(1)", 60_000_000_000),
]


@requires_ros_headers("the chrono constructor timer test")
def test_duration_constructor_timer_keeps_its_period(tmp_path: Path):
    from colcon_lf.node_analysis.clang_parser import parse_node

    timers = "\n".join(
        f"    t{i}_ = create_wall_timer({arg}, std::bind(&CtorNode::on_{i}, this));"
        for i, (arg, _) in enumerate(_CASES))
    callbacks = "\n".join(f"  void on_{i}() {{}}" for i in range(len(_CASES)))
    members = "\n".join(f"  rclcpp::TimerBase::SharedPtr t{i}_;" for i in range(len(_CASES)))
    cpp = tmp_path / "ctor_node.cpp"
    cpp.write_text(
        "#include <chrono>\n#include <functional>\n#include \"rclcpp/rclcpp.hpp\"\n"
        "class CtorNode : public rclcpp::Node\n{\npublic:\n  CtorNode() : Node(\"ctor_node\")\n  {\n"
        f"{timers}\n  }}\n\nprivate:\n{callbacks}\n{members}\n}};\n",
        encoding="utf-8")
    info = parse_node(cpp, workspace_root=tmp_path)
    got = {t.callback: (period_ms_to_ns(t.period_ms) if t.period_ms is not None else None)
           for t in info.timers}
    want = {f"on_{i}": ns for i, (_, ns) in enumerate(_CASES)}
    assert got == want, f"timer periods {got}, the constructors say {want}"


@requires_ros_headers("the tf_broadcaster_example timer test")
def test_tf_broadcaster_example_timer_is_33_ms():
    from colcon_lf.node_analysis.clang_parser import parse_node

    info = parse_node(REPO / "packages" / "src" / "fixtures" / "tf_broadcaster_example" / "src"
                      / "dynamic_tf_broadcaster_node.cpp")
    assert [(t.callback, t.period_ms) for t in info.timers] == [("timer_callback", 33.0)], (
        "std::chrono::milliseconds(33) left the dynamic broadcaster's period unresolved")

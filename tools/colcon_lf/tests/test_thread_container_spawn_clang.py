# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests for thread spawns into a thread container.

A callback that starts a method on a new thread marks the method as spawned
(``CallbackInfo.spawns_callbacks``). An action server's ``execute`` gets its
feedback and result attributed to the action through that link. Besides a
``std::thread`` construction, user code builds the thread in place inside a
container: ``workers_.emplace_back(&Node::execute, this, goal_handle)``.
The container is recognised by the clang type of its elements, so a container
of ``std::thread`` or ``std::jthread`` counts whatever its name, and a
container of anything else does not.
"""

from __future__ import annotations

import json
import shlex
from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the thread-container spawn snippet tests")


_NODE_TEMPLATE = """\
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SnippetNode : public rclcpp::Node
{{
public:
  SnippetNode() : Node("snippet_node")
  {{
    timer_ = create_wall_timer(100ms, std::bind(&SnippetNode::on_timer, this));
  }}

private:
  void on_timer()
  {{
{body}
  }}

  void work(int n)
  {{
    (void)n;
  }}

  rclcpp::TimerBase::SharedPtr timer_;
  {container_decl}
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SnippetNode>());
  rclcpp::shutdown();
  return 0;
}}
"""


def _spawns(tmp_path: Path, container_decl: str, body: str, std: str = "") -> list[str]:
    """The methods ``on_timer`` spawns. *std* overrides the C++ standard
    through a compile_commands.json, as a package build would provide it."""
    from colcon_lf.node_analysis.clang_parser import _load_include_flags, parse_node

    cpp_path = tmp_path / "snippet_node.cpp"
    cpp_path.write_text(
        _NODE_TEMPLATE.format(container_decl=container_decl, body=body), encoding="utf-8")
    if std:
        flags = [f for f in _load_include_flags(cpp_path, tmp_path) if not f.startswith("-std=")]
        ccj = tmp_path / "build" / "snippet" / "compile_commands.json"
        ccj.parent.mkdir(parents=True)
        command = shlex.join(["c++", f"-std={std}", *flags, "-c", str(cpp_path)])
        ccj.write_text(json.dumps([{"directory": str(tmp_path), "file": str(cpp_path),
                                    "command": command}]))
    node = parse_node(cpp_path, workspace_root=tmp_path)
    assert node is not None, "snippet node class not found"
    cb_map = {cb.name: cb for cb in node.callbacks}
    assert "on_timer" in cb_map, f"timer callback missing, callbacks: {sorted(cb_map)}"
    return cb_map["on_timer"].spawns_callbacks


class TestThreadContainerSpawn:
    def test_emplace_back_member_pointer_with_this(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::vector<std::thread> workers_;",
            "    workers_.emplace_back(&SnippetNode::work, this, 1);")
        assert "work" in spawns

    def test_emplace_back_bind(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::vector<std::thread> workers_;",
            "    workers_.emplace_back(std::bind(&SnippetNode::work, this, 1));")
        assert "work" in spawns

    def test_push_back_member_pointer_with_this(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::vector<std::thread> workers_;",
            "    workers_.push_back(std::thread(&SnippetNode::work, this, 1));")
        assert "work" in spawns

    def test_push_back_bind(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::vector<std::thread> workers_;",
            "    workers_.push_back(std::thread(std::bind(&SnippetNode::work, this, 1)));")
        assert "work" in spawns

    def test_emplace_at_position(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::list<std::thread> workers_;",
            "    workers_.emplace(workers_.end(), &SnippetNode::work, this, 1);")
        assert "work" in spawns

    def test_emplace_at_position_bind(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::list<std::thread> workers_;",
            "    workers_.emplace(workers_.end(), std::bind(&SnippetNode::work, this, 1));")
        assert "work" in spawns

    def test_jthread_container(self, tmp_path):
        spawns = _spawns(
            tmp_path, "std::vector<std::jthread> workers_;",
            "    workers_.emplace_back(&SnippetNode::work, this, 1);", std="c++20")
        assert "work" in spawns

    def test_type_not_name_decides(self, tmp_path):
        """A thread container with an unremarkable name and a typedef still counts."""
        spawns = _spawns(
            tmp_path, "using Pool = std::vector<std::thread>;\n  Pool items_;",
            "    items_.emplace_back(&SnippetNode::work, this, 1);")
        assert "work" in spawns

    def test_non_thread_container_is_not_a_spawn(self, tmp_path):
        """A container named like a thread pool but holding callables runs nothing."""
        spawns = _spawns(
            tmp_path, "std::vector<std::function<void()>> threads_;",
            "    threads_.emplace_back(std::bind(&SnippetNode::work, this, 1));")
        assert spawns == []

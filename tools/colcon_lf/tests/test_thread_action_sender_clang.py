# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests for threads that send on an action handle.

``NodeInfo.thread_action_senders`` lists every thread the node starts itself
(in the constructor or in any method) whose target sends action feedback or a
result. It feeds RCLLF-H011. A send from an LF timer callback is not a thread
and is not listed.

To falsify: make ``_find_thread_action_senders`` return ``[]`` and
``test_constructor_worker_is_listed`` fails with an empty list.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the thread action sender snippet tests")


_NODE_TEMPLATE = """\
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

{prelude}
using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class SnippetServer : public rclcpp::Node
{{
public:
  SnippetServer() : Node("snippet_server")
  {{
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "fibonacci",
      std::bind(&SnippetServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SnippetServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SnippetServer::handle_accepted, this, std::placeholders::_1));
{ctor_body}
  }}

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>)
  {{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }}

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {{
    return rclcpp_action::CancelResponse::ACCEPT;
  }}

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {{
{accepted_body}
  }}

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {{
    finish(goal_handle);
  }}

  void finish(const std::shared_ptr<GoalHandle> goal_handle)
  {{
    goal_handle->succeed(std::make_shared<Fibonacci::Result>());
  }}

  void worker_loop()
  {{
    if (active_) {{
      active_->publish_feedback(std::make_shared<Fibonacci::Feedback>());
    }}
  }}

  void step()
  {{
    if (active_) {{
      active_->publish_feedback(std::make_shared<Fibonacci::Feedback>());
    }}
  }}

  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandle> active_;
  std::thread worker_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SnippetServer>());
  rclcpp::shutdown();
  return 0;
}}
"""


def _senders(
    tmp_path: Path, ctor_body: str = "", accepted_body: str = "", prelude: str = "",
) -> list[dict]:
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp_path = tmp_path / "snippet_server.cpp"
    cpp_path.write_text(
        _NODE_TEMPLATE.format(ctor_body=ctor_body, accepted_body=accepted_body, prelude=prelude),
        encoding="utf-8")
    node = parse_node(cpp_path, workspace_root=tmp_path)
    assert node is not None, "snippet node class not found"
    assert [s.action_name for s in node.action_servers] == ["fibonacci"]
    return node.thread_action_senders


class TestThreadActionSenders:
    def test_constructor_worker_is_listed(self, tmp_path):
        senders = _senders(
            tmp_path, ctor_body="    worker_ = std::thread(&SnippetServer::worker_loop, this);")
        assert senders == [
            {"thread": "worker_loop", "started_in": "constructor", "action": "fibonacci"}]

    def test_execute_thread_through_helper_is_listed(self, tmp_path):
        senders = _senders(
            tmp_path, accepted_body=(
                "    std::thread{std::bind(&SnippetServer::execute, this, "
                "std::placeholders::_1), goal_handle}.detach();"))
        assert senders == [
            {"thread": "execute", "started_in": "handle_accepted", "action": "fibonacci"}]

    def test_lambda_thread_is_listed(self, tmp_path):
        senders = _senders(
            tmp_path, accepted_body=(
                "    std::thread([goal_handle]() {\n"
                "      goal_handle->succeed(std::make_shared<Fibonacci::Result>());\n"
                "    }).detach();"))
        assert senders == [{
            "thread": "lambda in handle_accepted", "started_in": "handle_accepted",
            "action": "fibonacci"}]

    def test_timer_stepped_server_is_not_listed(self, tmp_path):
        senders = _senders(
            tmp_path,
            ctor_body="    timer_ = create_wall_timer(100ms, std::bind(&SnippetServer::step, this));",
            accepted_body="    active_ = goal_handle;")
        assert senders == []

    def test_member_thread_from_lambda_calling_a_method_is_listed(self):
        """A review claimed a constructed ``std::thread([this] { ... })``
        is missed because it is no CALL_EXPR. libclang reports a constructor
        call as CALL_EXPR, so the thread is listed."""
        senders = _senders(
            tmp_path=self._tmp, ctor_body="    worker_ = std::thread([this]() { worker_loop(); });")
        assert {s["started_in"] for s in senders} == {"constructor"}
        assert "lambda in constructor" in {s["thread"] for s in senders}

    def test_named_thread_variable_from_lambda_is_listed(self):
        senders = _senders(
            tmp_path=self._tmp, accepted_body=(
                "    std::thread t([this, goal_handle]() { execute(goal_handle); });\n"
                "    t.detach();"))
        assert {s["started_in"] for s in senders} == {"handle_accepted"}
        assert "lambda in handle_accepted" in {s["thread"] for s in senders}

    def test_jthread_is_listed(self):
        """A ``std::jthread`` starts a thread like ``std::thread``. Before the
        fix only the spellings ``thread`` and ``async`` counted, so the list
        was empty. The snippet parses as C++17, so it declares jthread itself."""
        senders = _senders(
            tmp_path=self._tmp,
            prelude=(
                "namespace std {\n"
                "class jthread {\n"
                " public:\n"
                "  template<class F> explicit jthread(F && f) { f(); }\n"
                "  void detach() {}\n"
                "};\n"
                "}  // namespace std\n"),
            accepted_body=(
                "    std::jthread([goal_handle]() {\n"
                "      goal_handle->succeed(std::make_shared<Fibonacci::Result>());\n"
                "    }).detach();"))
        assert senders == [{
            "thread": "lambda in handle_accepted", "started_in": "handle_accepted",
            "action": "fibonacci"}]

    @pytest.fixture(autouse=True)
    def _tmp_dir(self, tmp_path):
        self._tmp = tmp_path

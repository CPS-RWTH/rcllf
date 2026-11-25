# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Node::create_timer and the free rclcpp::create_timer are read as timers.

A timer on the node's ROS clock (``this->create_timer(period, callback)``) runs on
/clock under use_sim_time. The analyzer used to skip it, so the node had no timer in
system.json. The free function ``rclcpp::create_timer`` has three
overloads with other argument layouts; the analyzer reads them by parameter name.
A free call for another node is not given to this node, and a project
function that is also spelled ``create_timer`` is not read as rclcpp's.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the create_timer snippet tests")


_NODE = """\
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace my_pkg
{
// A project function with rclcpp's parameter names; it is not rclcpp's timer factory.
template<typename CallbackT>
rclcpp::TimerBase::SharedPtr create_timer(
  rclcpp::Node * node, rclcpp::Clock::SharedPtr clock, rclcpp::Duration period,
  CallbackT && callback)
{
  (void)node; (void)clock; (void)period; (void)callback;
  return nullptr;
}
}  // namespace my_pkg

class TimerNode : public rclcpp::Node
{
public:
  TimerNode() : Node("timer_node")
  {
    ros_timer_ = this->create_timer(103ms, std::bind(&TimerNode::on_ros, this));
    implicit_timer_ = create_timer(113ms, std::bind(&TimerNode::on_implicit, this));
    wall_timer_ = create_wall_timer(150ms, std::bind(&TimerNode::on_wall, this));
    free_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(50ms), std::bind(&TimerNode::on_free, this));
    seconds_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.25),
      std::bind(&TimerNode::on_seconds, this));
    sci_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1e-1), std::bind(&TimerNode::on_sci, this));
    base_timer_ = rclcpp::create_timer(
      get_node_base_interface(), get_node_timers_interface(), get_clock(),
      rclcpp::Duration(70ms), std::bind(&TimerNode::on_base, this));
    clock_timer_ = rclcpp::create_timer(
      get_clock(), 80ms, std::bind(&TimerNode::on_clock_first, this), nullptr,
      get_node_base_interface().get(), get_node_timers_interface().get());
    other_timer_ = rclcpp::create_timer(
      other_, other_->get_clock(), rclcpp::Duration(90ms), std::bind(&TimerNode::on_other, this));
    project_timer_ = my_pkg::create_timer(
      this, get_clock(), rclcpp::Duration(60ms), std::bind(&TimerNode::on_project, this));
  }

private:
  void on_ros() {}
  void on_implicit() {}
  void on_wall() {}
  void on_free() {}
  void on_seconds() {}
  void on_sci() {}
  void on_base() {}
  void on_clock_first() {}
  void on_other() {}
  void on_project() {}
  rclcpp::Node::SharedPtr other_ = std::make_shared<rclcpp::Node>("other_node");
  rclcpp::TimerBase::SharedPtr ros_timer_;
  rclcpp::TimerBase::SharedPtr implicit_timer_;
  rclcpp::TimerBase::SharedPtr wall_timer_;
  rclcpp::TimerBase::SharedPtr free_timer_;
  rclcpp::TimerBase::SharedPtr seconds_timer_;
  rclcpp::TimerBase::SharedPtr sci_timer_;
  rclcpp::TimerBase::SharedPtr base_timer_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::TimerBase::SharedPtr other_timer_;
  rclcpp::TimerBase::SharedPtr project_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}
"""


def _parse(tmp_path: Path):
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp = tmp_path / "timer_node.cpp"
    cpp.write_text(_NODE, encoding="utf-8")
    return parse_node(cpp, workspace_root=tmp_path)


def test_member_create_timer_is_a_timer(tmp_path: Path):
    timers = sorted((t.callback, t.period_ms) for t in _parse(tmp_path).timers)
    # Expected from the snippet: two member create_timer calls and one wall timer.
    member = [t for t in timers if t[0] in ("on_implicit", "on_ros", "on_wall")]
    assert member == [("on_implicit", 113.0), ("on_ros", 103.0), ("on_wall", 150.0)], timers


def test_free_create_timer_is_a_timer(tmp_path: Path):
    info = _parse(tmp_path)
    timers = {t.callback: t for t in info.timers}
    # Expected from the snippet, one per overload: (node, clock, Duration(50ms)),
    # (node, clock, Duration::from_seconds(0.25)) = 250 ms, from_seconds(1e-1) = 100 ms,
    # (node_base, node_timers, clock, Duration(70ms)) and (clock, 80ms, callback, group,
    # node_base, node_timers).
    expected = {"on_free": 50.0, "on_seconds": 250.0, "on_sci": 100.0, "on_base": 70.0,
                "on_clock_first": 80.0}
    read = {cb: timers[cb].period_ms for cb in expected if cb in timers}
    assert read == expected, (
        f"free rclcpp::create_timer not read as a timer of the node: "
        f"read {read}, expected {expected}; all timers {sorted(timers)}")
    for cb in expected:
        assert timers[cb].created_in == "constructor", (cb, timers[cb].created_in)
        assert timers[cb].member_variable, (cb, timers[cb])


def test_free_create_timer_for_another_node_is_not_this_nodes(tmp_path: Path):
    timers = {t.callback for t in _parse(tmp_path).timers}
    assert "on_other" not in timers, (
        "rclcpp::create_timer(other_, ...) was given to this node; its timer runs on other_")


def test_project_create_timer_is_not_a_timer(tmp_path: Path):
    info = _parse(tmp_path)
    timers = {t.callback: t.period_ms for t in info.timers}
    assert "on_project" not in timers, (
        "my_pkg::create_timer(this, get_clock(), Duration(60ms), ...) was read as an "
        f"rclcpp timer of the node: {timers}")

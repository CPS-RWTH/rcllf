# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Callers of a member helper and the fold of its publishes.

Uses the publish-bound snippet node: ``on_tick`` is the timer callback and
``helper`` a private method that publishes on /out.
"""

from __future__ import annotations

import pytest

pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402
from .test_publish_bounds_clang import M, _cb, _parse  # noqa: E402

pytestmark = requires_ros_headers("the helper caller snippet tests")


def test_helper_called_from_the_timer_lists_it_and_folds(tmp_path):
    node = _parse(tmp_path, M + "    helper();")
    assert _cb(node, "helper").callers == ["on_tick"]
    assert "/out" in _cb(node, "on_tick").publishers_used


def test_helper_through_a_second_helper_folds_transitively(tmp_path):
    node = _parse(tmp_path, "    relay();", extra="  void relay() { helper(); }\n")
    assert _cb(node, "relay").callers == ["on_tick"]
    assert _cb(node, "helper").callers == ["relay"]
    assert _cb(node, "on_tick").publishers_used == ["/out"]


def test_constructor_call_is_an_outside_caller(tmp_path):
    node = _parse(tmp_path, "    helper();", ctor="    helper();")
    assert _cb(node, "helper").callers == ["<constructor>", "on_tick"]


def test_use_that_is_not_a_call_is_recorded(tmp_path):
    node = _parse(
        tmp_path, "    helper();",
        ctor="    sub_ = create_subscription<std_msgs::msg::String>(\"/in\", 10,"
             " [this](std_msgs::msg::String::SharedPtr) { auto f = &BoundNode::helper; (this->*f)(); });")
    callers = _cb(node, "helper").callers
    assert "on_tick" in callers
    assert any(c.startswith("<used, not called") for c in callers), callers


def test_public_helper_has_no_closed_caller_set(tmp_path):
    node = _parse(
        tmp_path, "    pub_helper();",
        extra="public:\n  void pub_helper() { std_msgs::msg::String m; out_->publish(m); }\nprivate:\n")
    assert _cb(node, "pub_helper").callers is None
    assert "/out" in _cb(node, "on_tick").publishers_used

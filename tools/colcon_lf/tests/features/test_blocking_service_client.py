# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H009, a blocking service client.

The diagnostic runs on a hand-built graph through the whole registry, so the
test also fails when the module is not registered. The real-source case is the
playground example ``blocking_service_client_example/bringup``.

To falsify: remove ``blocking_service_client`` from the module imports in
``features/registry.py`` and ``test_blocking_future_client_is_refused`` fails
with ``RCLLF-H009`` missing from the emitted codes.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.registry import run_detection, severity_for
from colcon_lf.node_analysis.models import ServiceClientInfo
from tests.features.conftest import make_graph, make_node

CODE_H009 = "RCLLF-H009"


def _client_node(name: str, call_style: str | None):
    node = make_node(name)
    node.service_clients = [
        ServiceClientInfo(
            service_name="process_data",
            service_type="custom_interfaces/srv/ProcessData",
            call_style=call_style,
        )
    ]
    return node


def _h009(graph) -> list:
    return [d for d in run_detection(DetectionContext(graph=graph)) if d.code == CODE_H009]


class TestDiagnostic:
    def test_blocking_future_client_is_refused(self):
        graph = make_graph({"future_client": _client_node("future_client", "blocking_future")})
        diags = _h009(graph)
        assert [d.code for d in diags] == [CODE_H009], (
            f"expected one {CODE_H009}, got codes "
            f"{[d.code for d in run_detection(DetectionContext(graph=graph))]}"
        )
        d = diags[0]
        assert d.node == "future_client"
        assert d.topic == "process_data"
        assert "'process_data'" in d.message
        assert "future.wait" in d.message
        assert "#62" not in d.message
        assert "async_send_request" in d.message

    def test_blocking_spin_client_is_refused(self):
        graph = make_graph({"spin_client": _client_node("spin_client", "blocking_spin")})
        diags = _h009(graph)
        assert len(diags) == 1
        assert "spin_until_future_complete" in diags[0].message

    def test_async_callback_client_is_not_reported(self):
        graph = make_graph({"async_client": _client_node("async_client", "async_callback")})
        assert _h009(graph) == []

    def test_undetermined_style_is_not_reported(self):
        """None stays on the existing async path, so no refusal (conservative)."""
        graph = make_graph({"client": _client_node("client", None)})
        assert _h009(graph) == []

    def test_severity_is_error(self):
        assert severity_for(CODE_H009) == Severity.ERROR


def _method(source: str):
    """A stand-in method cursor: clang-like tokens (identifiers, ``::``, ``->``, punctuation)."""
    import re
    from types import SimpleNamespace

    tokens = [
        SimpleNamespace(spelling=t)
        for t in re.findall(r"\w+|::|->|\S", source)
    ]
    return SimpleNamespace(get_tokens=lambda: iter(tokens))


_SEND = (
    "void C::tick() { auto req = std::make_shared<Req>(); "
    "client_->async_send_request(req, [this](Future f) { done_ = true; (void)f.get(); }); "
)


class TestClassifyWaitUntilResponse:
    """A wait for the response in the same callback is blocking.

    To falsify: drop ``_waits_after_send`` from ``_classify_service_client_call``
    and every positive case here returns ``async_callback``.
    """

    def _style(self, source: str):
        from colcon_lf.node_analysis.clang_parser import _classify_service_client_call

        return _classify_service_client_call(_method(source))

    def test_spin_some_loop_is_blocking(self):
        src = _SEND + "while (!done_) { rclcpp::spin_some(this->get_node_base_interface()); } }"
        assert self._style(src) == "blocking_wait"

    def test_spin_once_loop_is_blocking(self):
        src = _SEND + "while (!done_) { executor_.spin_once(10ms); } }"
        assert self._style(src) == "blocking_wait"

    def test_executor_spin_until_future_complete_after_callback_is_blocking(self):
        src = _SEND + "executor_.spin_until_future_complete(fut, 5s); }"
        assert self._style(src) in ("blocking_spin", "blocking_wait")

    def test_sleep_loop_on_flag_is_blocking(self):
        src = _SEND + "while (!done_.load()) { std::this_thread::sleep_for(10ms); } }"
        assert self._style(src) == "blocking_wait"

    def test_condition_variable_wait_is_blocking(self):
        src = (
            _SEND
            + "std::unique_lock<std::mutex> lk(m_); "
            + "cv_.wait(lk, [this] { return done_.load(); }); }"
        )
        assert self._style(src) == "blocking_wait"

    def test_condition_variable_wait_for_is_blocking(self):
        src = _SEND + "std::unique_lock<std::mutex> lk(m_); cv_.wait_for(lk, 5s); }"
        assert self._style(src) == "blocking_wait"

    def test_plain_async_callback_is_not_blocking(self):
        """Negative: the response callback sets a flag and calls get(), nothing waits."""
        src = _SEND + 'RCLCPP_INFO(get_logger(), "sent"); ++count_; }'
        assert self._style(src) == "async_callback"


def test_blocking_wait_client_is_refused():
    graph = make_graph({"wait_client": _client_node("wait_client", "blocking_wait")})
    diags = _h009(graph)
    assert len(diags) == 1
    assert "blocking_wait" in diags[0].message
    assert "wait loop" in diags[0].message

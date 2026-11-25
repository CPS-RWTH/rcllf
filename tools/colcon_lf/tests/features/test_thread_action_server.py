# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H011, an action server that sends from its own thread.

The diagnostic runs on a hand-built graph through the whole registry, so the
test also fails when the module is not registered. The analysis half (which
threads send on an action handle) is in
``tests/test_thread_action_sender_clang.py``. The real-source case is the
playground example ``action_full_example/bringup``.

To falsify: remove ``thread_action_server`` from the module imports in
``features/registry.py`` and ``test_constructor_worker_is_refused`` fails with
``RCLLF-H011`` missing from the emitted codes.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.registry import run_detection, severity_for
from tests.features.conftest import make_graph, make_node

CODE_H011 = "RCLLF-H011"


def _node(name: str, senders: list[dict[str, str]]):
    node = make_node(name)
    node.thread_action_senders = senders
    return node


def _h010(graph) -> list:
    return [d for d in run_detection(DetectionContext(graph=graph)) if d.code == CODE_H011]


class TestDiagnostic:
    def test_constructor_worker_is_refused(self):
        graph = make_graph({"server": _node("server", [
            {"thread": "worker_loop", "started_in": "constructor", "action": "fibonacci"}])})
        diags = _h010(graph)
        assert [d.code for d in diags] == [CODE_H011], (
            f"expected one {CODE_H011}, got codes "
            f"{[d.code for d in run_detection(DetectionContext(graph=graph))]}"
        )
        d = diags[0]
        assert d.node == "server"
        assert d.topic == "fibonacci"
        assert "'fibonacci'" in d.message
        assert "'worker_loop'" in d.message
        assert "constructor" in d.message
        assert "#122" not in d.message

    def test_execute_thread_of_handle_accepted_is_refused(self):
        graph = make_graph({"server": _node("server", [
            {"thread": "execute", "started_in": "handle_accepted", "action": "analyze"}])})
        diags = _h010(graph)
        assert len(diags) == 1
        assert "handle_accepted" in diags[0].message

    def test_unresolved_action_names_every_server(self):
        graph = make_graph({"server": _node("server", [
            {"thread": "execute", "started_in": "handle_accepted", "action": "?"}])})
        assert "one of its action servers" in _h010(graph)[0].message

    def test_stepped_server_is_not_reported(self):
        graph = make_graph({"server": _node("server", [])})
        assert _h010(graph) == []

    def test_severity_is_error(self):
        assert severity_for(CODE_H011) == Severity.ERROR

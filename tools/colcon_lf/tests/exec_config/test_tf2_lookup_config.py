# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``nodes.<node>.tf2`` sets the wait bound and the
timeout outcome of every rcllf_tf2::AcquisitionLookup the node constructs.

The yaml wins over the constructor literals: the ``tf2.lookup.<member>``
record carries the configured values, ``explicitly_configured``, and keeps the
literals under ``literal``; the generated RID init registers them for the
binding. The literals stay the stock-ROS values (no execution.yaml there).
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.generation.binding_header_emitter import collect_tf2_configs, emit_rid_init_cpp
from colcon_lf.node_analysis.models import Tf2AdapterInfo, Tf2BufferInfo, Tf2Info
from colcon_lf.resolved_model.model import (
    PROVENANCE_EXPLICITLY_CONFIGURED,
    PROVENANCE_EXTRACTED,
)
from colcon_lf.resolved_model.resolve import _tf2_records

from .conftest import make_graph, make_node


def _graph(with_adapter=True):
    node = make_node("acq", class_name="AcqNode")
    node.tf2 = Tf2Info(
        buffers=[Tf2BufferInfo(member="buf_", clock="node")],
        adapters=[Tf2AdapterInfo(member="lk_", buffer_member="buf_", wait_bound_ns=55_000_000,
                                 on_timeout="reject", file="acq.cpp", line=12)]
        if with_adapter else [])
    return make_graph({"acq": node})


def _load(body, graph=None):
    return ExecConfig.from_dict({"nodes": {"acq": {"tf2": body}}}, graph or _graph(),
                                source="test")


def test_wait_bound_and_on_timeout_parse():
    cfg = _load({"wait_bound": "125 msec", "on_timeout": "failure"})
    assert cfg.nodes["acq"]["tf2"] == {"wait_bound_ns": 125_000_000, "on_timeout": "failure"}
    assert _load({"on_timeout": "fallback:identity"}).nodes["acq"]["tf2"] == {
        "on_timeout": "fallback:identity"}
    longest = "fallback:" + "x" * 246  # 255 characters, the runtime buffer's limit
    assert _load({"on_timeout": longest}).nodes["acq"]["tf2"] == {"on_timeout": longest}


@pytest.mark.parametrize("body, match", [
    ({"wait_bound": "0"}, "must be > 0"),
    ({"wait_bound": "55"}, "needs a unit"),
    ({"wait_bound": "soon"}, "wait_bound"),
    ({"on_timeout": "drop"}, "must be 'reject', 'failure' or 'fallback:<name>'"),
    ({"on_timeout": "fallback:"}, "must be 'reject', 'failure' or 'fallback:<name>'"),
    ({"on_timeout": "fallback:a b"}, "printable ASCII"),
    ({"on_timeout": "fallback:a\nb"}, "printable ASCII"),
    ({"on_timeout": "fallback:" + "x" * 247}, "longer than the runtime limit of 255"),
    ({"bound": "5ms"}, "unknown key"),
    ({}, "must be a mapping with 'wait_bound'"),
])
def test_bad_values_are_refused_at_load(body, match):
    with pytest.raises(ValueError, match=match):
        _load(body)


def test_node_without_an_adapter_is_refused():
    with pytest.raises(ValueError, match="constructs no rcllf_tf2::AcquisitionLookup"):
        _load({"on_timeout": "failure"}, _graph(with_adapter=False))


def test_yaml_wins_over_the_literal_in_the_record_and_the_generated_init():
    graph = _graph()
    tf2 = graph.nodes["acq"].tf2
    [plain] = [r for r in _tf2_records("acq", tf2) if r.property == "tf2.lookup.lk_"]
    assert (plain.provenance, plain.value["wait_bound_ns"], plain.value["on_timeout"]) == (
        PROVENANCE_EXTRACTED, 55_000_000, "reject")

    cfg = _load({"wait_bound": "125 msec", "on_timeout": "failure"}, graph)
    [rec] = [r for r in _tf2_records("acq", tf2, cfg.nodes["acq"]["tf2"])
             if r.property == "tf2.lookup.lk_"]
    assert rec.provenance == PROVENANCE_EXPLICITLY_CONFIGURED, (
        "execution.yaml nodes.acq.tf2 did not make the lookup record explicitly_configured")
    assert (rec.value["wait_bound_ns"], rec.value["on_timeout"]) == (125_000_000, "failure"), (
        "the constructor literal won over execution.yaml nodes.acq.tf2")
    assert rec.value["literal"] == {"wait_bound_ns": 55_000_000, "on_timeout": "reject"}
    assert rec.value["configured"] == ["on_timeout", "wait_bound_ns"]

    configs = collect_tf2_configs(SimpleNamespace(records=[rec]), graph)
    assert configs == [("AcqNode", 125_000_000, "failure")]
    cpp = emit_rid_init_cpp([], "pkg", tf2_configs=configs)
    assert '  ros2_lf_binding::register_tf2_config("AcqNode", 125000000LL, "failure");' in cpp

    # Only the keys the yaml set are passed on; the other stays the literal.
    cfg = _load({"on_timeout": "failure"}, graph)
    [rec] = [r for r in _tf2_records("acq", tf2, cfg.nodes["acq"]["tf2"])
             if r.property == "tf2.lookup.lk_"]
    assert collect_tf2_configs(SimpleNamespace(records=[rec]), graph) == [("AcqNode", 0, "failure")]
    assert "register_tf2_config" not in emit_rid_init_cpp([], "pkg")

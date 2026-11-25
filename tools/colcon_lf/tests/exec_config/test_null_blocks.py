# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A block key with a YAML null value is absent.

``nodes.<node>.tf2: null`` and ``nodes.<node>.timers: null`` follow the rule of
every other block of execution.yaml (``defaults:``, ``nodes:``, ...): the key
written without a value loads as if it were left out. A null policy value is
still refused (docs/features/execution-config.md, "Other validation rules").
"""

from __future__ import annotations

import pytest

from colcon_lf.exec_config.model import ExecConfig

from .conftest import make_graph, make_node


def _graph():
    return make_graph({"n": make_node("n", timers=["tick"])})


@pytest.mark.parametrize("key", ["timers", "tf2"])
def test_null_node_block_loads_as_absent(key):
    with_null = ExecConfig.from_dict({"nodes": {"n": {key: None}}}, _graph(), source="t")
    without = ExecConfig.from_dict({"nodes": {"n": {}}}, _graph(), source="t")
    assert with_null.nodes == without.nodes == {"n": {}}


def test_null_tf2_on_a_node_without_an_adapter_is_not_a_tf2_block():
    # A tf2 *block* on a node without an admitted AcquisitionLookup is refused;
    # a null one is no block at all (node "n" has no tf2 adapter).
    with pytest.raises(ValueError):
        ExecConfig.from_dict({"nodes": {"n": {"tf2": {"on_timeout": "reject"}}}}, _graph(),
                             source="t")
    ExecConfig.from_dict({"nodes": {"n": {"tf2": None}}}, _graph(), source="t")


@pytest.mark.parametrize("key", ["defaults", "nodes", "subscriptions", "external_topics",
                                 "connections", "publishers", "acknowledged_hazards"])
def test_null_top_level_block_loads_as_absent(key):
    assert ExecConfig.from_dict({key: None}, _graph(), source="t").nodes == {}


def test_null_policy_value_is_refused():
    with pytest.raises(ValueError, match="'take_timeout_ms' must be an int > 0, got None"):
        ExecConfig.from_dict({"nodes": {"n": {"take_timeout_ms": None}}}, _graph(), source="t")

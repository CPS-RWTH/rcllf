# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The generation-time per-node `auto_transition` opt-out.

The key decides whether the generated factory builds a lifecycle
configure/activate thunk at all. It is generation-time (`runtime=False`), so it
must never reach `exec_config.json`, whose `{schema_version, defaults,
subscriptions[]}` shape is a frozen contract the C++ side fails closed on.
"""

from __future__ import annotations

import pytest
import yaml

from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.policies import (
    KEY_AUTO_TRANSITION,
    REGISTRY,
    RUNTIME_REGISTRY,
    SCOPE_DEFAULTS,
    SCOPE_NODES,
    SCOPE_SUBSCRIPTIONS,
)
from colcon_lf.exec_config.resolve import resolve
from colcon_lf.generation.factory_emitter import (
    emit_headerless_factory_suffix,
    emit_node_factories_cpp,
)

from .conftest import make_graph, make_node


def _write(tmp_path, body: dict):
    path = tmp_path / "execution.yaml"
    path.write_text(yaml.safe_dump(body), encoding="utf-8")
    return path


@pytest.fixture
def graph():
    # `publisher` has no subscriptions at all, mirroring the lifecycle example.
    # That is why this key cannot live in the subscription-keyed runtime table.
    return make_graph({
        "publisher": make_node("publisher", "PublisherNode"),
        "listener": make_node("listener", "ListenerNode", subscriptions=["sensor_data"]),
    })


class TestRegistryShape:
    def test_settable_in_defaults_and_nodes_only(self):
        spec = REGISTRY[KEY_AUTO_TRANSITION]
        assert SCOPE_DEFAULTS in spec.scopes
        assert SCOPE_NODES in spec.scopes
        assert SCOPE_SUBSCRIPTIONS not in spec.scopes

    def test_defaults_to_true(self):
        assert REGISTRY[KEY_AUTO_TRANSITION].default is True

    def test_is_not_a_runtime_key(self):
        """It must stay out of exec_config.json's frozen shape."""
        assert KEY_AUTO_TRANSITION not in RUNTIME_REGISTRY


class TestLoading:
    def test_accepted_under_defaults(self, tmp_path, graph):
        path = _write(tmp_path, {"defaults": {KEY_AUTO_TRANSITION: False}})
        config = ExecConfig.load(path, graph)
        assert config.defaults[KEY_AUTO_TRANSITION] is False

    def test_accepted_per_node(self, tmp_path, graph):
        path = _write(tmp_path, {"nodes": {"publisher": {KEY_AUTO_TRANSITION: False}}})
        config = ExecConfig.load(path, graph)
        assert config.nodes["publisher"][KEY_AUTO_TRANSITION] is False

    def test_rejected_per_subscription(self, tmp_path, graph):
        path = _write(tmp_path, {
            "subscriptions": {"listener": {"sensor_data": {KEY_AUTO_TRANSITION: False}}},
        })
        with pytest.raises(ValueError, match="not valid at 'subscriptions' scope"):
            ExecConfig.load(path, graph)

    @pytest.mark.parametrize("bad", ["false", "no", 0, 1, "true"])
    def test_non_boolean_rejected(self, tmp_path, graph, bad):
        """A quoted 'false' is a string, and truthiness would read it as True."""
        path = _write(tmp_path, {"nodes": {"publisher": {KEY_AUTO_TRANSITION: bad}}})
        with pytest.raises(ValueError, match="must be true or false"):
            ExecConfig.load(path, graph)


class TestStaysOutOfTheRuntimeTable:
    def test_absent_from_resolved_json(self, tmp_path, graph):
        path = _write(tmp_path, {"defaults": {KEY_AUTO_TRANSITION: False}})
        resolved = resolve(ExecConfig.load(path, graph), graph)
        emitted = resolved.to_json_dict()
        assert KEY_AUTO_TRANSITION not in emitted["defaults"]
        for row in emitted["subscriptions"]:
            assert KEY_AUTO_TRANSITION not in row


class TestEmission:
    def test_default_emission_is_unchanged(self):
        """A node that does not opt out emits exactly the call from before the auto-transition setting existed."""
        entry = {
            "binding_key": "PlainNode", "class_name": "PlainNode", "package": "pkg",
            "header": "pkg/plain.hpp", "idx": 0, "node_name": "plain", "namespace": "",
        }
        with_flag = emit_node_factories_cpp([{**entry, "auto_transition": True}])
        without_flag = emit_node_factories_cpp([entry])
        assert with_flag == without_flag
        assert "auto_transition" not in with_flag

    def test_opt_out_emits_the_third_argument(self):
        out = emit_node_factories_cpp([{
            "binding_key": "LcNode", "class_name": "LcNode", "package": "pkg",
            "header": "pkg/lc.hpp", "idx": 0, "node_name": "lc", "namespace": "",
            "auto_transition": False,
        }])
        assert "/*auto_transition=*/false" in out

    def test_headerless_opt_out_bakes_the_flag_in(self):
        """The flag is baked into the generated suffix, not threaded through
        HEADERLESS_FACTORY_SIGNATURE, since it is a generation-time decision."""
        opted_out = emit_headerless_factory_suffix(3, "Head", auto_transition=False)
        default = emit_headerless_factory_suffix(3, "Head")
        assert "/*auto_transition=*/false" in opted_out
        assert "auto_transition" not in default


class TestResolvePrecedence:
    """Registry default (True) < execution.yaml `defaults` < per-node `nodes`."""

    @staticmethod
    def _resolve(tmp_path, graph, body):
        from colcon_lf.verb.generate import _resolve_auto_transition
        (tmp_path / "config").mkdir(exist_ok=True)
        _write(tmp_path / "config", body)
        return _resolve_auto_transition(
            system_json=tmp_path / "system.json", graph=graph, exec_config_arg=None,
        )

    def test_no_config_leaves_every_node_on_the_default(self, tmp_path, graph):
        from colcon_lf.verb.generate import _resolve_auto_transition
        resolved = _resolve_auto_transition(
            system_json=tmp_path / "system.json", graph=graph, exec_config_arg=None,
        )
        # No execution.yaml at all: ExecConfig.empty(), so every node defaults.
        # Asserted as an exact mapping, not `all(...)`, which an empty dict
        # would satisfy vacuously.
        assert resolved == {"publisher": True, "listener": True}

    def test_defaults_apply_system_wide(self, tmp_path, graph):
        resolved = self._resolve(tmp_path, graph, {"defaults": {KEY_AUTO_TRANSITION: False}})
        assert resolved == {"publisher": False, "listener": False}

    def test_node_scope_beats_defaults(self, tmp_path, graph):
        resolved = self._resolve(tmp_path, graph, {
            "defaults": {KEY_AUTO_TRANSITION: False},
            "nodes": {"listener": {KEY_AUTO_TRANSITION: True}},
        })
        assert resolved == {"publisher": False, "listener": True}

    def test_node_scope_alone(self, tmp_path, graph):
        resolved = self._resolve(tmp_path, graph, {
            "nodes": {"publisher": {KEY_AUTO_TRANSITION: False}},
        })
        assert resolved == {"publisher": False, "listener": True}

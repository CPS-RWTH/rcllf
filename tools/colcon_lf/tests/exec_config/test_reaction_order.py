# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``reaction_order`` sets the order of a reactor's reactions at one tag.

LF runs the reactions of one reactor that the dependency graph leaves unordered at a
tag in declaration order. The generator declares timer reactions before subscription
reactions (``timers_first``, the default since the follow-up).
``subscriptions_first`` (the order before this key existed) declares subscription
reactions first. The key may be set under ``defaults`` and per node, and is read
back as ``node:<fqn>#reaction_order``.

To falsify: drop ``reaction_order`` from ``REGISTRY`` (every config here is refused
as an unknown policy key), or ignore it in ``_build_reactions`` (the timer reaction
stays after the subscription), or drop the record from ``resolved_model/resolve.py``,
or set the registry default back to ``subscriptions_first`` (the default tests fail
with ``['sub', 'timer']`` / ``['on_in', 'tick']``).
"""

from __future__ import annotations

import json
import textwrap

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo, NodeInfo, PublisherInfo, SubscriberInfo, TimerInfo,
)
from colcon_lf.output.serializer import serialize

MSG = "std_msgs/msg/Int64"


def _consumer(name: str = "cons", cls: str = "ConsNode") -> NodeInfo:
    """A timer callback defined first in source, then a subscription callback."""
    return NodeInfo(
        name=name, class_name=cls, package="test_pkg", source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name="tick"), CallbackInfo(name="on_in")],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        subscribers=[SubscriberInfo(topic="/in", message_type=MSG, callback="on_in")],
    )


def _producer() -> NodeInfo:
    return NodeInfo(
        name="prod", class_name="ProdNode", package="test_pkg", source_file="/fake/prod.cpp",
        callbacks=[CallbackInfo(name="emit", publishers_used=["/in"])],
        timers=[TimerInfo(period_ms=100.0, callback="emit")],
        publishers=[PublisherInfo(topic="/in", message_type=MSG)],
    )


def _graph(*consumers: NodeInfo, launch: str = "/fake/bringup.launch.py") -> SystemGraph:
    nodes = {"prod": _producer(), **{c.name: c for c in consumers}}
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=LaunchFileInfo(file_path=launch, package="test_pkg"),
                                 all_nodes=[]),
        nodes=nodes,
        communication=SystemCommunication(topics=[TopicConnection(
            topic="/in", message_type=MSG, publishers=["prod"],
            subscribers=[c.name for c in consumers],
        )]),
    )


def _kinds(system, reactor_name: str) -> list[str]:
    (reactor,) = [r for r in system.reactors if r.name == reactor_name]
    return [r.kind for r in reactor.reactions if r.kind in ("timer", "sub")]


def _invoke_order(text: str, cls: str) -> list[str]:
    """The callbacks of *cls* in the order the emitted LF text invokes them."""
    marks = {f"RID_{cls}_tick": "tick", f"RID_{cls}_on_in": "on_in"}
    hits = sorted((text.index(m), cb) for m, cb in marks.items() if m in text)
    return [cb for _, cb in hits]


class TestSchema:
    def _from(self, raw: dict) -> ExecConfig:
        return ExecConfig.from_dict(raw, _graph(_consumer()), source="test")

    @pytest.mark.parametrize("value", ["subscriptions_first", "timers_first"])
    def test_defaults_and_node_values_parse(self, value):
        config = self._from({"defaults": {"reaction_order": value},
                             "nodes": {"cons": {"reaction_order": value}}})
        assert config.defaults["reaction_order"] == value
        assert config.nodes["cons"]["reaction_order"] == value

    def test_unknown_value_names_both_choices(self):
        with pytest.raises(ValueError, match=(
                r"'reaction_order' must be one of \('subscriptions_first', "
                r"'timers_first'\), got 'source_order'")):
            self._from({"defaults": {"reaction_order": "source_order"}})

    def test_non_string_value_is_refused(self):
        with pytest.raises(ValueError, match="'reaction_order' must be one of"):
            self._from({"nodes": {"cons": {"reaction_order": 1}}})

    def test_unknown_node_is_refused(self):
        with pytest.raises(ValueError, match=r"unknown node 'ghost' \(not in system.json\)"):
            self._from({"nodes": {"ghost": {"reaction_order": "timers_first"}}})

    def test_subscription_scope_is_refused(self):
        with pytest.raises(ValueError, match="reaction_order"):
            self._from({"subscriptions": {"cons": {"/in": {"reaction_order": "timers_first"}}}})


class TestGeneration:
    def test_default_declares_timers_first(self):
        system = generate_lf_system(_graph(_consumer()), target_lfc=True)
        assert _kinds(system, "ConsNode_DefaultGroup") == ["timer", "sub"]
        assert _invoke_order(emit_lf_system(system), "ConsNode") == ["tick", "on_in"]

    def test_subscriptions_first_declares_the_subscription_first(self):
        system = generate_lf_system(_graph(_consumer()), target_lfc=True,
                                    reaction_order={"cons": "subscriptions_first"})
        assert _kinds(system, "ConsNode_DefaultGroup") == ["sub", "timer"]
        assert _invoke_order(emit_lf_system(system), "ConsNode") == ["on_in", "tick"]

    def test_timers_first_declares_the_timer_first(self):
        system = generate_lf_system(_graph(_consumer()), target_lfc=True,
                                    reaction_order={"cons": "timers_first"})
        assert _kinds(system, "ConsNode_DefaultGroup") == ["timer", "sub"]
        assert _invoke_order(emit_lf_system(system), "ConsNode") == ["tick", "on_in"]

    def test_explicit_timers_first_is_the_default_text(self):
        graph = _graph(_consumer())
        default = emit_lf_system(generate_lf_system(graph, target_lfc=True))
        explicit = emit_lf_system(generate_lf_system(
            graph, target_lfc=True, reaction_order={"cons": "timers_first"}))
        assert explicit == default

    def test_unnamed_node_takes_the_registry_default(self):
        # A node the resolved map leaves out (or an empty map after a config
        # error) falls back to the registry default, not to the old order.
        graph = _graph(_consumer())
        unnamed = emit_lf_system(generate_lf_system(
            graph, target_lfc=True, reaction_order={"other": "subscriptions_first"}))
        assert _invoke_order(unnamed, "ConsNode") == ["tick", "on_in"]

    def test_order_is_per_node(self):
        system = generate_lf_system(
            _graph(_consumer("a", "ANode"), _consumer("b", "BNode")), target_lfc=True,
            reaction_order={"a": "timers_first", "b": "subscriptions_first"},
        )
        assert _kinds(system, "ANode_DefaultGroup") == ["timer", "sub"]
        assert _kinds(system, "BNode_DefaultGroup") == ["sub", "timer"]

    def test_reaction_ids_follow_the_declared_order(self):
        system = generate_lf_system(_graph(_consumer()), target_lfc=True,
                                    reaction_order={"cons": "timers_first"})
        (reactor,) = [r for r in system.reactors if r.name == "ConsNode_DefaultGroup"]
        ids = {r.kind: r.reaction_id for r in reactor.reactions if r.kind in ("timer", "sub")}
        assert ids["timer"] + 1 == ids["sub"]

    def test_unknown_order_is_refused(self):
        with pytest.raises(ValueError, match="reaction order"):
            generate_lf_system(_graph(_consumer()), target_lfc=True,
                               reaction_order={"cons": "random"})


def _stage(tmp_path, graph_nodes=None):
    pkg = tmp_path / "test_pkg"
    (pkg / "launch").mkdir(parents=True)
    (pkg / "lf").mkdir()
    (pkg / "package.xml").write_text("<package/>")
    launch = pkg / "launch" / "bringup.launch.py"
    launch.write_text("")
    graph = _graph(*(graph_nodes or [_consumer()]), launch=str(launch))
    system_json = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
    system_json.parent.mkdir(parents=True)
    system_json.write_text(serialize(graph))
    return graph, system_json


class TestResolvedModel:
    def _record(self, tmp_path, raw: dict):
        from colcon_lf.resolved_model.resolve import resolve

        graph, system_json = _stage(tmp_path)
        cfg = ExecConfig.from_dict(raw, graph, source="execution.yaml") if raw else (
            ExecConfig.empty())
        model = resolve(system_json, exec_config=cfg, workspace=tmp_path)
        (rec,) = [r for r in model.records if r.contract_id == "node:cons#reaction_order"]
        return rec

    def test_node_value_is_read_back_as_configured(self, tmp_path):
        rec = self._record(tmp_path, {"defaults": {"reaction_order": "subscriptions_first"},
                                      "nodes": {"cons": {"reaction_order": "timers_first"}}})
        assert (rec.value, rec.provenance) == ("timers_first", "explicitly_configured")
        assert rec.evidence == "execution.yaml:nodes.cons.reaction_order"

    def test_defaults_value_is_read_back_as_configured(self, tmp_path):
        rec = self._record(tmp_path, {"defaults": {"reaction_order": "timers_first"}})
        assert (rec.value, rec.provenance) == ("timers_first", "explicitly_configured")
        assert rec.evidence == "execution.yaml:defaults.reaction_order"

    def test_unset_is_read_back_as_registry_default(self, tmp_path):
        rec = self._record(tmp_path, {})
        assert rec.value == "timers_first"
        assert rec.evidence == "registry-default:reaction_order"


@pytest.fixture
def _no_lfc_package_emission(monkeypatch):
    """The .lf text is written before the lfc package is emitted. The package
    stages the user's C++ sources, which this synthetic graph does not have."""
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_emit_lfc_artifacts", lambda **kwargs: "lf_generated_test")


@pytest.mark.usefixtures("_no_lfc_package_emission")
class TestGenerateVerb:
    @pytest.fixture(autouse=True)
    def _capsys(self, capsys):
        self.capsys = capsys

    def _generate(self, tmp_path, yaml_text: str, nodes=None):
        from colcon_lf.verb.generate import _generate_one

        _, system_json = _stage(tmp_path, nodes)
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent(yaml_text))
        out = tmp_path / "lf"
        rc = _generate_one(system_json=system_json, output_dir=str(out), after_delay=None,
                           exec_config_arg=str(exec_config))
        assert rc == 0, self.capsys.readouterr().err
        (lf_file,) = out.glob("*.lf")
        return lf_file.read_text(), system_json

    def test_node_override_beats_defaults(self, tmp_path):
        text, system_json = self._generate(tmp_path, """\
            defaults:
              reaction_order: subscriptions_first
            nodes:
              a:
                reaction_order: timers_first
        """, nodes=[_consumer("a", "ANode"), _consumer("b", "BNode")])
        assert _invoke_order(text, "ANode") == ["tick", "on_in"]
        assert _invoke_order(text, "BNode") == ["on_in", "tick"]
        model = json.loads((system_json.parent / "resolved_model.json").read_text())
        values = {r["contract_id"]: r["value"] for r in model["records"]
                  if r["contract_id"].endswith("#reaction_order")}
        assert values["node:a#reaction_order"] == "timers_first"
        assert values["node:b#reaction_order"] == "subscriptions_first"

    def test_defaults_value_applies_to_every_node(self, tmp_path):
        text, _ = self._generate(tmp_path, """\
            defaults:
              reaction_order: timers_first
        """)
        assert _invoke_order(text, "ConsNode") == ["tick", "on_in"]

    def test_no_reaction_order_key_declares_timers_first(self, tmp_path):
        text, system_json = self._generate(tmp_path, """\
            defaults:
              take_timeout_ms: 10
        """)
        assert _invoke_order(text, "ConsNode") == ["tick", "on_in"]
        model = json.loads((system_json.parent / "resolved_model.json").read_text())
        (rec,) = [r for r in model["records"] if r["contract_id"] == "node:cons#reaction_order"]
        assert rec["value"] == "timers_first"
        assert rec["evidence"] == "registry-default:reaction_order"

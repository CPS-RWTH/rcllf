# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``nodes.<node>.timers.<timer>.offset`` sets a timer's start offset.

The offset reaches the emitted LF ``timer t_<cb>(<offset>, <period>)`` and the
resolved model (``timer:<node>/<cb>#offset``), next to the existing per-timer
``period`` override (P3).

To falsify: drop ``offset`` from ``_validate_timers`` (the config is refused
with "exactly one key, 'period'"), or stop applying the offsets to the graph in
``verb/generate.py`` (the emitted timer keeps ``(0, 25 msec)``), or drop the
``#offset`` record from ``resolved_model/resolve.py``.
"""

from __future__ import annotations

import json
import textwrap

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import CallbackInfo, NodeInfo, PublisherInfo, TimerInfo
from colcon_lf.output.serializer import serialize

from .conftest import make_graph, make_node

MSG = "std_msgs/msg/String"


def _from(timers: dict, timer_names=("timer_callback",)) -> ExecConfig:
    graph = make_graph({"n": make_node("n", timers=list(timer_names))})
    return ExecConfig.from_dict({"nodes": {"n": {"timers": timers}}}, graph, source="test")


class TestOffsetSchema:
    def test_offset_alone_parses(self):
        config = _from({"timer_callback": {"offset": "17 msec"}})
        assert config.nodes["n"]["timers"] == {"timer_callback": {"offset": "17 msec"}}

    def test_offset_and_period_parse_together(self):
        config = _from({"timer_callback": {"offset": "5 msec", "period": "10 msec"}})
        assert config.nodes["n"]["timers"] == {
            "timer_callback": {"offset": "5 msec", "period": "10 msec"},
        }

    def test_mixed_type_unknown_timer_keys_raise_value_error(self):
        # YAML key `1` parses as int; sorting it against 'phase' used to TypeError.
        with pytest.raises(ValueError, match="unknown key"):
            _from({"timer_callback": {1: "x", "phase": "y"}})

    def test_mixed_type_unknown_top_level_keys_raise_value_error(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        with pytest.raises(ValueError, match="unknown top-level key"):
            ExecConfig.from_dict({1: {}, "phase": {}}, graph, source="test")

    def test_zero_offset_is_accepted(self):
        config = _from({"timer_callback": {"offset": 0}})
        assert config.nodes["n"]["timers"] == {"timer_callback": {"offset": "0"}}

    def test_offset_longer_than_period_is_a_start_delay(self):
        config = _from({"timer_callback": {"offset": "40 msec", "period": "25 msec"}})
        assert config.nodes["n"]["timers"]["timer_callback"]["offset"] == "40 msec"

    def test_two_timers_on_one_node(self):
        config = _from(
            {"fast": {"offset": "1 msec"}, "slow": {"offset": "3 msec", "period": "1 sec"}},
            timer_names=("fast", "slow"),
        )
        assert config.nodes["n"]["timers"] == {
            "fast": {"offset": "1 msec"},
            "slow": {"offset": "3 msec", "period": "1 sec"},
        }

    def test_negative_offset_raises(self):
        with pytest.raises(ValueError, match="'offset' must not be negative"):
            _from({"timer_callback": {"offset": "-5 msec"}})

    def test_zero_period_raises(self):
        with pytest.raises(ValueError, match="'period' must be > 0"):
            _from({"timer_callback": {"period": "0"}})

    def test_empty_entry_raises(self):
        with pytest.raises(ValueError, match="needs 'offset' or 'period'"):
            _from({"timer_callback": {}})

    def test_other_key_raises(self):
        with pytest.raises(ValueError, match=r"unknown key\(s\) 'phase'"):
            _from({"timer_callback": {"phase": "5 msec"}})

    def test_unknown_timer_lists_known_timers(self):
        with pytest.raises(
            ValueError,
            match=r"no timer named 'tick' on 'n' \(known timers: fast, slow\)",
        ):
            _from({"tick": {"offset": "1 msec"}}, timer_names=("slow", "fast"))


def _node(offset_ms: float = 0.0, period_ms: float | None = 25.0) -> NodeInfo:
    return NodeInfo(
        name="ctrl", class_name="CtrlNode", package="test_pkg", source_file="/fake/ctrl.cpp",
        callbacks=[CallbackInfo(name="control_callback", publishers_used=["/cmd"])],
        timers=[TimerInfo(period_ms=period_ms, callback="control_callback",
                          offset_ms=offset_ms)],
        publishers=[PublisherInfo(topic="/cmd", message_type=MSG)],
    )


def _graph(launch, node: NodeInfo) -> SystemGraph:
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(
            root=LaunchFileInfo(file_path=str(launch), package="test_pkg"), all_nodes=[],
        ),
        nodes={"ctrl": node},
        communication=SystemCommunication(topics=[TopicConnection(
            topic="/cmd", message_type=MSG, publishers=["ctrl"], subscribers=[],
        )]),
    )


def _stage(tmp_path, node: NodeInfo, package_yaml: str | None = None):
    pkg = tmp_path / "test_pkg"
    (pkg / "launch").mkdir(parents=True)
    (pkg / "lf").mkdir()
    (pkg / "package.xml").write_text("<package/>")
    launch = pkg / "launch" / "bringup.launch.py"
    launch.write_text("")
    if package_yaml is not None:
        (pkg / "lf" / "bringup.execution.yaml").write_text(package_yaml)
    graph = _graph(launch, node)
    system_json = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
    system_json.parent.mkdir(parents=True)
    system_json.write_text(serialize(graph))
    return graph, system_json, pkg / "lf" / "bringup.execution.yaml"


class TestLayering:
    def test_user_offset_keeps_the_package_period(self, tmp_path):
        graph, system_json, package = _stage(tmp_path, _node(), package_yaml=textwrap.dedent("""\
            nodes:
              ctrl:
                timers:
                  control_callback: {period: "10 msec"}
        """))
        user = tmp_path / "user.yaml"
        user.write_text(textwrap.dedent("""\
            nodes:
              ctrl:
                timers:
                  control_callback: {offset: "5 msec"}
        """))
        config = ExecConfig.load_layered(user, package, graph)
        assert config.nodes["ctrl"]["timers"] == {
            "control_callback": {"period": "10 msec", "offset": "5 msec"},
        }
        assert config.origin(("nodes", "ctrl", "timers", "control_callback", "offset")) == (
            f"user {user}")
        assert config.origin(("nodes", "ctrl", "timers", "control_callback", "period")) == (
            f"package {package}")


class TestResolvedModelOffset:
    def _offset_record(self, model):
        (rec,) = [r for r in model.records
                  if r.contract_id == "timer:ctrl/control_callback#offset"]
        return rec

    def test_configured_offset_is_read_back(self, tmp_path):
        from colcon_lf.resolved_model.resolve import resolve

        graph, system_json, _ = _stage(tmp_path, _node())
        cfg = ExecConfig.from_dict(
            {"nodes": {"ctrl": {"timers": {"control_callback": {"offset": "17 msec"}}}}},
            graph, source="execution.yaml",
        )
        rec = self._offset_record(resolve(system_json, exec_config=cfg, workspace=tmp_path))
        assert (rec.value, rec.unit, rec.provenance) == (17.0, "ms", "explicitly_configured")
        assert rec.evidence == "execution.yaml:nodes.ctrl.timers.control_callback.offset"

    def test_analyzed_offset_is_read_back_as_extracted(self, tmp_path):
        from colcon_lf.resolved_model.resolve import resolve

        _, system_json, _ = _stage(tmp_path, _node())
        rec = self._offset_record(
            resolve(system_json, exec_config=ExecConfig.empty(), workspace=tmp_path))
        assert (rec.value, rec.provenance) == (0.0, "extracted")


@pytest.fixture
def _no_lfc_package_emission(monkeypatch):
    """The .lf text is written before the lfc package is emitted. The package
    stages the user's C++ sources, which this synthetic graph does not have."""
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_emit_lfc_artifacts", lambda **kwargs: "lf_generated_test")


@pytest.mark.usefixtures("_no_lfc_package_emission")
class TestGenerateVerbEmitsOffset:
    @pytest.fixture(autouse=True)
    def _capsys(self, capsys):
        self.capsys = capsys

    def _generate(self, tmp_path, yaml_text: str, node: NodeInfo | None = None):
        from colcon_lf.verb.generate import _generate_one

        _, system_json, _ = _stage(tmp_path, node or _node())
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent(yaml_text))
        out = tmp_path / "lf"
        rc = _generate_one(system_json=system_json, output_dir=str(out), after_delay=None,
                           exec_config_arg=str(exec_config))
        assert rc == 0, self.capsys.readouterr().err
        (lf_file,) = out.glob("*.lf")
        return lf_file.read_text(), system_json

    def test_offset_reaches_the_lf_timer(self, tmp_path):
        text, system_json = self._generate(tmp_path, """\
            nodes:
              ctrl:
                timers:
                  control_callback: {offset: "17 msec"}
        """)
        assert "timer t_control_callback(17 msec, 25 msec)" in text, (
            "emitted timer line: "
            + next(line.strip() for line in text.splitlines() if "timer t_" in line))
        model = json.loads((system_json.parent / "resolved_model.json").read_text())
        (rec,) = [r for r in model["records"]
                  if r["contract_id"] == "timer:ctrl/control_callback#offset"]
        assert rec["value"] == 17.0

    def test_offset_and_period_reach_the_lf_timer(self, tmp_path):
        text, _ = self._generate(tmp_path, """\
            nodes:
              ctrl:
                timers:
                  control_callback: {offset: "5 msec", period: "10 msec"}
        """)
        assert "timer t_control_callback(5 msec, 10 msec)" in text, (
            "emitted timer line: "
            + next(line.strip() for line in text.splitlines() if "timer t_" in line))

    def test_sub_millisecond_offset_keeps_its_value(self, tmp_path):
        text, _ = self._generate(tmp_path, """\
            nodes:
              ctrl:
                timers:
                  control_callback: {offset: "1500 usec"}
        """)
        # Written as exact nsec, lfc rejects a fractional "1.5 msec".
        assert "timer t_control_callback(1500000 nsec, 25 msec)" in text, (
            "emitted timer line: "
            + next(line.strip() for line in text.splitlines() if "timer t_" in line))

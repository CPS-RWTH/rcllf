# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Stamp-anchored external topics (plan 5.3 item 1).

An ``external_topics`` entry may be a mapping ``{topic, anchor: stamp,
delay_ns, ingress_reserve_ns, grid_ns, basis}``. ``delay_ns`` (T) and
``basis`` have no default (T is a justified per-topic
value, never a silent default); the identity defaults 0 and 1 of the
reserve and the grid are printed by ``colcon lf-configure`` with their
origin. The generator declares the constants to the binding right after
the topic's ``ros2_lf_runtime_declare_external_topic`` line.
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import CallbackInfo, NodeInfo, SubscriberInfo
from colcon_lf.verb.generate import _declare_external_topics

STAMP = {
    "topic": "scan", "anchor": "stamp", "delay_ns": 200_000_000,
    "ingress_reserve_ns": 50_000_000, "grid_ns": 10_000_000,
    "basis": "placeholder until the streaming rig's p99 is measured (plan §8 Q5)",
}


def _graph(launch="/fake/bringup.launch.py"):
    node = NodeInfo(
        name="listener", class_name="ListenerNode", package="test_pkg",
        source_file="/fake/listener.cpp", callbacks=[CallbackInfo(name="on_scan")],
        subscribers=[SubscriberInfo(
            topic="scan", message_type="sensor_msgs/msg/LaserScan", callback="on_scan",
        )],
    )
    tc = TopicConnection(
        topic="/scan", message_type="sensor_msgs/msg/LaserScan",
        publishers=[], subscribers=["listener"],
    )
    root = LaunchFileInfo(file_path=launch, package="test_pkg")
    return SystemGraph(
        name="bringup", launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={node.name: node}, communication=SystemCommunication(topics=[tc]),
    )


def _load(entries):
    return ExecConfig.from_dict({"external_topics": entries}, _graph(), source="t")


class TestStampAnchorYaml:
    def test_mapping_entry_declares_the_topic_and_its_anchor(self):
        config = _load([STAMP])
        assert config.external_topics == ["/scan"]
        assert config.external_anchors == {"/scan": {
            "anchor": "stamp", "delay_ns": 200_000_000, "ingress_reserve_ns": 50_000_000,
            "grid_ns": 10_000_000, "basis": STAMP["basis"],
        }}

    def test_string_entry_has_no_anchor(self):
        config = _load(["/scan"])
        assert config.external_topics == ["/scan"]
        assert config.external_anchors == {}

    def test_reserve_and_grid_default_to_the_identity(self):
        config = _load([{k: v for k, v in STAMP.items()
                         if k not in ("ingress_reserve_ns", "grid_ns")}])
        anchor = config.external_anchors["/scan"]
        assert (anchor["ingress_reserve_ns"], anchor["grid_ns"]) == (0, 1)

    def test_delay_ns_has_no_default(self):
        with pytest.raises(ValueError, match="needs 'delay_ns'.*there is no default"):
            _load([{k: v for k, v in STAMP.items() if k != "delay_ns"}])

    @pytest.mark.parametrize("basis", [None, "", "   "])
    def test_basis_is_required_and_non_empty(self, basis):
        entry = dict(STAMP)
        if basis is None:
            entry.pop("basis")
        else:
            entry["basis"] = basis
        with pytest.raises(ValueError, match="needs a non-empty 'basis'"):
            _load([entry])

    @pytest.mark.parametrize("key, value", [
        ("delay_ns", -1), ("delay_ns", "200ms"), ("delay_ns", True),
        ("ingress_reserve_ns", -5), ("grid_ns", 0), ("grid_ns", 1.5),
    ])
    def test_constants_are_non_negative_integers(self, key, value):
        with pytest.raises(ValueError, match=f"'{key}' must be an integer"):
            _load([dict(STAMP, **{key: value})])

    @pytest.mark.parametrize("key", ["delay_ns", "ingress_reserve_ns", "grid_ns"])
    def test_constants_are_bounded_so_the_tag_arithmetic_fits(self, key):
        with pytest.raises(ValueError, match=f"'{key}' must be an integer <= 2\\*\\*60"):
            _load([dict(STAMP, **{key: 2**60 + 1})])
        assert _load([dict(STAMP, **{key: 2**60})]).external_anchors["/scan"][key] == 2**60

    def test_unknown_key_is_refused(self):
        with pytest.raises(ValueError, match=r"unknown key\(s\) \['delay_ms'\]"):
            _load([dict(STAMP, delay_ms=3)])

    def test_unknown_anchor_is_refused(self):
        with pytest.raises(ValueError, match="'anchor' must be one of"):
            _load([dict(STAMP, anchor="logical")])

    def test_arrival_anchor_takes_no_constants(self):
        with pytest.raises(ValueError, match="apply to 'anchor: stamp' only"):
            _load([{"topic": "scan", "anchor": "arrival", "delay_ns": 1}])

    def test_arrival_mapping_is_an_arrival_anchor(self):
        config = _load([{"topic": "/scan", "anchor": "arrival"}])
        assert config.external_anchors == {"/scan": {"anchor": "arrival"}}

    def test_mapping_without_topic_is_refused(self):
        with pytest.raises(ValueError, match="needs a 'topic' string"):
            _load([{"anchor": "stamp"}])

    def test_two_mappings_for_one_topic_are_refused(self):
        with pytest.raises(ValueError, match="two mapping entries"):
            _load([STAMP, dict(STAMP, topic="/scan")])

    def test_unknown_topic_is_refused_in_mapping_form(self):
        with pytest.raises(ValueError, match="unknown topic 'odom'"):
            _load([dict(STAMP, topic="odom")])


class TestLayering:
    def _setup(self, tmp_path, user_yaml):
        from colcon_lf.output.serializer import serialize

        pkg = tmp_path / "test_pkg"
        (pkg / "launch").mkdir(parents=True)
        (pkg / "lf").mkdir()
        (pkg / "package.xml").write_text("<package/>")
        launch = pkg / "launch" / "bringup.launch.py"
        launch.write_text("")
        package = pkg / "lf" / "bringup.execution.yaml"
        package.write_text(
            "external_topics:\n"
            "  - topic: /scan\n"
            "    anchor: stamp\n"
            "    delay_ns: 200000000\n"
            "    basis: package placeholder\n"
        )
        graph = _graph(str(launch))
        system_json = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
        (system_json.parent / "config").mkdir(parents=True)
        system_json.write_text(serialize(graph))
        user = system_json.parent / "config" / "execution.yaml"
        user.write_text(user_yaml)
        return graph, system_json, package, user

    def test_user_mapping_replaces_the_package_anchor_as_one_unit(self, tmp_path):
        graph, _, package, user = self._setup(
            tmp_path,
            "external_topics:\n"
            "  - topic: /scan\n"
            "    anchor: stamp\n"
            "    delay_ns: 300000000\n"
            "    grid_ns: 5000000\n"
            "    basis: rig measurement 2026-11-11\n",
        )
        config = ExecConfig.load_layered(user, package, graph)
        assert config.external_topics == ["/scan"]
        assert config.external_anchors["/scan"] == {
            "anchor": "stamp", "delay_ns": 300_000_000, "ingress_reserve_ns": 0,
            "grid_ns": 5_000_000, "basis": "rig measurement 2026-11-11",
        }
        assert config.origin(("external_topics", "/scan")) == f"package {package}"
        assert config.origin(("external_topics", "/scan", "anchor")) == f"user {user}"

    def test_user_string_entry_keeps_the_package_anchor(self, tmp_path):
        graph, _, package, user = self._setup(tmp_path, "external_topics: [/scan]\n")
        config = ExecConfig.load_layered(user, package, graph)
        assert config.external_anchors["/scan"]["delay_ns"] == 200_000_000
        assert config.origin(("external_topics", "/scan", "anchor")) == f"package {package}"

    def test_show_prints_every_constant_with_its_origin(self, tmp_path, capsys, monkeypatch):
        from colcon_lf.verb.configure import configure_one

        _, system_json, package, _ = self._setup(tmp_path, "external_topics: [/scan]\n")
        monkeypatch.delenv("ROS2_LF_TAKE_MODE", raising=False)
        monkeypatch.delenv("ROS2_LF_TAKE_TIMEOUT_MS", raising=False)
        assert configure_one(system_json=system_json, show=True) == 0
        out = capsys.readouterr().out
        assert f"      - /scan  <- package {package}" in out
        assert f"        anchor: stamp  <- package {package}" in out
        assert f"        delay_ns: 200000000  <- package {package}" in out
        # The identity defaults are printed, never silent.
        assert f"        ingress_reserve_ns: 0  <- package {package}" in out
        assert f"        grid_ns: 1  <- package {package}" in out
        assert f'        basis: "package placeholder"  <- package {package}' in out


class TestStartupDeclaration:
    START = (
        "    char** argv = const_cast<char**>(argv0);\n"
        "    ros2_lf_runtime_init(argc, argv);\n"
    )

    def test_stamp_anchor_is_declared_right_after_its_topic(self):
        anchors = {"/scan": {
            "anchor": "stamp", "delay_ns": 200_000_000, "ingress_reserve_ns": 50_000_000,
            "grid_ns": 10_000_000, "basis": "x",
        }}
        text = _declare_external_topics(self.START, {"/scan", "/a"}, anchors)
        assert text == (
            "    char** argv = const_cast<char**>(argv0);\n"
            '    ros2_lf_runtime_declare_external_topic("/a");\n'
            '    ros2_lf_runtime_declare_external_topic("/scan");\n'
            '    ros2_lf_runtime_declare_external_stamp_anchor("/scan", 200000000LL, '
            "50000000LL, 10000000LL);\n"
            "    ros2_lf_runtime_init(argc, argv);\n"
        )

    def test_arrival_anchor_adds_no_declaration(self):
        text = _declare_external_topics(self.START, {"/scan"}, {"/scan": {"anchor": "arrival"}})
        assert text == (
            "    char** argv = const_cast<char**>(argv0);\n"
            '    ros2_lf_runtime_declare_external_topic("/scan");\n'
            "    ros2_lf_runtime_init(argc, argv);\n"
        )

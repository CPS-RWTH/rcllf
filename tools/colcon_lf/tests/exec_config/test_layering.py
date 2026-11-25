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
"""Layer precedence of execution.yaml.

built-in < package ``lf/<launch>.execution.yaml`` < user file (the analysis
dir's ``config/execution.yaml``, or ``--exec-config`` in its place) < env.
Layers merge value by value; scope precedence (defaults < nodes <
subscriptions) applies after the merge.
"""

from __future__ import annotations

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig, package_exec_config_path
from colcon_lf.exec_config.resolve import resolve
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.output.serializer import serialize
from colcon_lf.verb.generate import _load_exec_config

PACKAGE_YAML = """\
defaults:
  take_mode: block
  take_timeout_ms: 250
nodes:
  listener:
    timers:
      tick: {period: "20 msec"}
subscriptions:
  listener:
    /chatter:
      take_timeout_ms: 300
external_topics:
  - /chatter
acknowledged_hazards: [H1]
"""

USER_YAML = """\
defaults:
  take_mode: skip
nodes:
  listener:
    take_timeout_ms: 400
acknowledged_hazards: [H2]
"""


def _graph(launch):
    node = NodeInfo(
        name="listener",
        class_name="ListenerNode",
        package="test_pkg",
        source_file="/fake/listener.cpp",
        callbacks=[CallbackInfo(name="on_msg"), CallbackInfo(name="tick")],
        subscribers=[SubscriberInfo(
            topic="/chatter", message_type="std_msgs/msg/String", callback="on_msg",
        )],
        timers=[TimerInfo(period_ms=10.0, callback="tick")],
    )
    tc = TopicConnection(
        topic="/chatter", message_type="std_msgs/msg/String",
        publishers=[], subscribers=["listener"],
    )
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(
            root=LaunchFileInfo(file_path=str(launch), package="test_pkg"), all_nodes=[],
        ),
        nodes={"listener": node},
        communication=SystemCommunication(topics=[tc]),
    )


def _setup(tmp_path, user_yaml=USER_YAML):
    pkg = tmp_path / "test_pkg"
    (pkg / "launch").mkdir(parents=True)
    (pkg / "lf").mkdir()
    (pkg / "package.xml").write_text("<package/>")
    launch = pkg / "launch" / "bringup.launch.py"
    launch.write_text("")
    (pkg / "lf" / "bringup.execution.yaml").write_text(PACKAGE_YAML)
    graph = _graph(launch)
    system_json = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
    (system_json.parent / "config").mkdir(parents=True)
    system_json.write_text(serialize(graph))
    user = system_json.parent / "config" / "execution.yaml"
    if user_yaml is not None:
        user.write_text(user_yaml)
    return graph, system_json, pkg / "lf" / "bringup.execution.yaml", user


class TestLayerMerge:
    def test_user_value_wins_at_its_path_and_package_fills_the_rest(self, tmp_path):
        graph, _, package, user = _setup(tmp_path)
        config = ExecConfig.load_layered(user, package, graph)
        assert config.defaults == {"take_mode": "skip", "take_timeout_ms": 250}
        assert config.origin(("defaults", "take_mode")) == f"user {user}"
        assert config.origin(("defaults", "take_timeout_ms")) == f"package {package}"
        # A node scope the user sets keeps the package's timers of that node.
        assert config.nodes["listener"]["take_timeout_ms"] == 400
        assert config.nodes["listener"]["timers"] == {"tick": {"period": "20 msec"}}
        assert config.origin(("nodes", "listener", "timers", "tick", "period")) == (
            f"package {package}")

    def test_declaration_lists_union_with_their_layer(self, tmp_path):
        graph, _, package, user = _setup(tmp_path)
        config = ExecConfig.load_layered(user, package, graph)
        assert config.acknowledged_hazards == ["H1", "H2"]
        assert config.external_topics == ["/chatter"]
        assert config.origin(("acknowledged_hazards", "H2")) == f"user {user}"
        assert config.origin(("external_topics", "/chatter")) == f"package {package}"

    def test_each_layer_is_validated_on_its_own(self, tmp_path):
        graph, _, package, user = _setup(tmp_path, user_yaml="defaults: {take_mode: maybe}\n")
        try:
            ExecConfig.load_layered(user, package, graph)
        except ValueError as exc:
            assert str(exc).startswith(f"{user}: defaults:"), exc
        else:
            raise AssertionError("an invalid user file must be refused")


class TestPrecedenceAfterResolve:
    def test_scope_precedence_applies_after_the_layer_merge(self, tmp_path):
        """A narrower scope wins even from a lower layer.

        take_timeout_ms is 250 at package defaults, 400 at user nodes and 300
        at package subscriptions. The subscription scope is the narrowest, so
        300 wins although the user layer is higher.
        """
        graph, _, package, user = _setup(tmp_path)
        resolved = resolve(ExecConfig.load_layered(user, package, graph), graph)
        (row,) = resolved.subscriptions
        assert row.values["take_mode"] == "skip"
        assert row.values["take_timeout_ms"] == 300
        assert row.origins["take_mode"] == f"user {user} (defaults)"
        assert row.origins["take_timeout_ms"] == f"package {package} (subscriptions)"

    def test_unset_everywhere_is_built_in(self, tmp_path):
        graph, _, package, _ = _setup(tmp_path, user_yaml=None)
        (tmp_path / "test_pkg" / "lf" / "bringup.execution.yaml").write_text(
            "external_topics: [/chatter]\n",
        )
        resolved = resolve(ExecConfig.load_layered(None, package, graph), graph)
        (row,) = resolved.subscriptions
        assert row.origins == {"take_mode": "built-in", "take_timeout_ms": "built-in"}
        assert resolved.default_origins["take_mode"] == "built-in"

    def test_exec_config_flag_replaces_the_user_file_not_the_package(self, tmp_path):
        graph, system_json, package, _ = _setup(tmp_path)
        explicit = tmp_path / "explicit.yaml"
        explicit.write_text("defaults: {take_timeout_ms: 50}\n")
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=str(explicit),
        )
        # The analysis dir's user file (take_mode: skip) is not read.
        assert config.defaults == {"take_mode": "block", "take_timeout_ms": 50}
        assert config.origin(("defaults", "take_timeout_ms")) == f"--exec-config {explicit}"
        assert config.external_topics == ["/chatter"]

    def test_package_path_follows_the_launch_file(self, tmp_path):
        graph, _, package, _ = _setup(tmp_path)
        assert package_exec_config_path(graph) == package


class TestShowNamesTheLayer:
    def test_show_prints_the_layer_of_every_value(self, tmp_path, capsys, monkeypatch):
        from colcon_lf.verb.configure import configure_one

        _, system_json, package, user = _setup(tmp_path)
        monkeypatch.setenv("ROS2_LF_TAKE_MODE", "block")
        monkeypatch.delenv("ROS2_LF_TAKE_TIMEOUT_MS", raising=False)
        assert configure_one(system_json=system_json, show=True) == 0
        out = capsys.readouterr().out
        assert f"      take_mode: \"skip\"  <- user {user} (defaults)" in out
        assert f"      take_timeout_ms: 300  <- package {package} (subscriptions)" in out
        assert f"      - /chatter  <- package {package}" in out
        assert f"      - H2  <- user {user}" in out
        assert "    env ROS2_LF_TAKE_MODE=block overrides that key" in out
        assert "ROS2_LF_TAKE_TIMEOUT_MS" not in out

    def test_show_says_the_timeout_env_skips_admitted_edges(self, tmp_path, capsys, monkeypatch):
        # The binding reads ROS2_LF_TAKE_TIMEOUT_MS for an admitted edge only when
        # its generated timeout is negative, which the generator never emits
        # (corr_runtime.cpp, add_edge call), so the message must not claim
        # "every subscription".
        from colcon_lf.verb.configure import configure_one

        _, system_json, _, _ = _setup(tmp_path)
        monkeypatch.delenv("ROS2_LF_TAKE_MODE", raising=False)
        monkeypatch.setenv("ROS2_LF_TAKE_TIMEOUT_MS", "3600000")
        assert configure_one(system_json=system_json, show=True) == 0
        out = capsys.readouterr().out
        line = next(ln for ln in out.splitlines() if "ROS2_LF_TAKE_TIMEOUT_MS" in ln)
        assert "for every subscription" not in line
        assert "An admitted correspondence edge keeps its generated timeout" in line
        assert "regenerate" in line

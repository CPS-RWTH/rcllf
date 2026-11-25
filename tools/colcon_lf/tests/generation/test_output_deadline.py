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
"""Per-publisher output deadline (plan 2026-11-09 5.3 item 3).

``publishers: {<node>: {<topic>: {output_deadline_ns, basis}}}`` in
``execution.yaml`` makes every reaction of the node whose callback publishes
the topic arm the binding's actuation lateness check right before its
``ros2_lf_invoke_*`` call. The generated program has no LF ``deadline``: a
missed LF deadline runs its handler instead of the body and drops the message.
"""

from __future__ import annotations

import re

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import CallbackInfo, NodeInfo, PublisherInfo, TimerInfo
from colcon_lf.verb.generate import output_deadlines_of

BOUND = 20_000_000
BASIS = "test: 20 ms, a fifth of the 100 ms period"


def _graph(publishers_used=("actuation",)):
    node = NodeInfo(
        name="controller", class_name="Controller", package="test_pkg",
        source_file="/fake/controller.cpp",
        callbacks=[
            CallbackInfo(name="on_tick", publishers_used=list(publishers_used)),
            CallbackInfo(name="on_idle"),
        ],
        timers=[
            TimerInfo(period_ms=100.0, callback="on_tick"),
            TimerInfo(period_ms=500.0, callback="on_idle"),
        ],
        publishers=[
            PublisherInfo(topic="actuation", message_type="std_msgs/msg/Int64"),
            PublisherInfo(topic="status", message_type="std_msgs/msg/Int64"),
        ],
    )
    tcs = [
        TopicConnection(topic=f"/{t}", message_type="std_msgs/msg/Int64",
                        publishers=["controller"], subscribers=[])
        for t in ("actuation", "status")
    ]
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup", launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={node.name: node}, communication=SystemCommunication(topics=tcs),
    )


def _load(publishers, graph=None):
    return ExecConfig.from_dict({"publishers": publishers}, graph or _graph(), source="t")


class TestPublishersYaml:
    def test_entry_resolves_node_and_topic(self):
        config = _load({"controller": {"/actuation": {"output_deadline_ns": BOUND,
                                                      "basis": BASIS}}})
        assert config.publishers == {
            "controller": {"actuation": {"output_deadline_ns": BOUND, "basis": BASIS}},
        }
        assert output_deadlines_of(config) == {"controller": {"actuation": BOUND}}

    def test_absent_key_means_no_check(self):
        config = ExecConfig.from_dict({}, _graph(), source="t")
        assert config.publishers == {}
        assert output_deadlines_of(config) == {}

    @pytest.mark.parametrize("body, match", [
        ({"basis": BASIS}, r"needs 'output_deadline_ns'.*there is no default"),
        ({"output_deadline_ns": BOUND}, r"needs a non-empty 'basis'"),
        ({"output_deadline_ns": BOUND, "basis": "  "}, r"needs a non-empty 'basis'"),
        ({"output_deadline_ns": -1, "basis": BASIS}, r"integer >= 0"),
        ({"output_deadline_ns": 2.5e7, "basis": BASIS}, r"integer >= 0"),
        ({"output_deadline_ns": True, "basis": BASIS}, r"integer >= 0"),
        ({"output_deadline_ns": 2**60 + 1, "basis": BASIS}, r"<= 2\*\*60"),
        ({"output_deadline_ns": BOUND, "basis": BASIS, "deadline": 1}, r"unknown key"),
    ])
    def test_bad_entry_is_refused(self, body, match):
        with pytest.raises(ValueError, match=match):
            _load({"controller": {"actuation": body}})

    def test_unknown_topic_is_refused(self):
        with pytest.raises(ValueError, match=r"unknown topic 'cmd' on node 'controller'"):
            _load({"controller": {"cmd": {"output_deadline_ns": BOUND, "basis": BASIS}}})

    def test_unknown_node_is_refused(self):
        with pytest.raises(ValueError, match=r"nobody"):
            _load({"nobody": {"actuation": {"output_deadline_ns": BOUND, "basis": BASIS}}})

    def test_two_spellings_of_one_node_are_refused(self):
        # 'controller' and '/controller' resolve to the same
        # node; the second entry used to replace the first, dropping its bound.
        with pytest.raises(
            ValueError,
            match=r"publishers\.'/controller' and publishers\.'controller' both resolve "
                  r"to '/controller' at runtime",
        ):
            _load({
                "controller": {"actuation": {"output_deadline_ns": BOUND, "basis": BASIS}},
                "/controller": {"status": {"output_deadline_ns": BOUND, "basis": BASIS}},
            })

    def test_two_spellings_of_one_topic_are_refused(self):
        # A node that publishes on both 'actuation' and
        # '/actuation' (both resolve to '/actuation' at runtime) kept two keys,
        # and the binding's arm keyed by resolved topic let the later bound win.
        graph = _graph()
        graph.nodes["controller"].publishers.append(
            PublisherInfo(topic="/actuation", message_type="std_msgs/msg/Int64"))
        with pytest.raises(
            ValueError,
            match=r"publishers\.'controller'\.'/actuation' and publishers\.'controller'\."
                  r"'actuation' both resolve to '/actuation' at runtime",
        ):
            _load({"controller": {
                "actuation": {"output_deadline_ns": 0, "basis": BASIS},
                "/actuation": {"output_deadline_ns": BOUND, "basis": BASIS},
            }}, graph)

    def test_namespaced_relative_and_absolute_topics_are_two_topics(self):
        # For a node in '/robot', 'actuation' resolves to
        # '/robot/actuation' and '/actuation' stays '/actuation'. Two runtime
        # topics, two bounds; the root-namespace check refused this.
        graph = _graph()
        graph.nodes["controller"].publishers.append(
            PublisherInfo(topic="/actuation", message_type="std_msgs/msg/Int64"))
        graph.nodes = {"/robot/controller": graph.nodes.pop("controller")}
        config = _load({"robot/controller": {
            "actuation": {"output_deadline_ns": 0, "basis": BASIS},
            "/actuation": {"output_deadline_ns": BOUND, "basis": BASIS},
        }}, graph)
        assert config.publishers == {"/robot/controller": {
            "actuation": {"output_deadline_ns": 0, "basis": BASIS},
            "/actuation": {"output_deadline_ns": BOUND, "basis": BASIS},
        }}

    def test_zero_bound_is_allowed(self):
        config = _load({"controller": {"actuation": {"output_deadline_ns": 0, "basis": BASIS}}})
        assert output_deadlines_of(config) == {"controller": {"actuation": 0}}


class TestEmission:
    def _text(self, deadlines, graph=None):
        system = generate_lf_system(graph or _graph(), target_lfc=True,
                                    output_deadlines=deadlines)
        return emit_lf_system(system)

    def test_arm_line_sits_right_before_the_publishing_invoke(self):
        text = self._text({"controller": {"actuation": BOUND}})
        lines = [ln.strip() for ln in text.splitlines()]
        arms = [i for i, ln in enumerate(lines) if ln.startswith("ros2_lf_output_deadline_arm(")]
        assert len(arms) == 1, "one publishing reaction, one bounded topic"
        arm = lines[arms[0]]
        assert re.fullmatch(
            r'ros2_lf_output_deadline_arm\((RID_\w+_on_tick), "actuation", 20000000LL\);'
            r"  /\* publishers: output_deadline_ns \*/", arm), arm
        rid = re.match(r"ros2_lf_output_deadline_arm\((RID_\w+),", arm).group(1)
        assert lines[arms[0] + 1] == f"ros2_lf_invoke_timer(self, {rid});"

    def test_only_the_publishing_reaction_is_armed(self):
        text = self._text({"controller": {"actuation": BOUND}})
        assert "on_idle);" in text
        for ln in text.splitlines():
            if "ros2_lf_output_deadline_arm(" in ln:
                assert "on_idle" not in ln

    def test_no_lf_deadline_anywhere(self):
        text = self._text({"controller": {"actuation": BOUND}})
        assert not re.search(r"\bdeadline\s*\(", text), "no LF deadline clause"

    def test_without_entry_the_text_is_unchanged(self):
        assert self._text(None) == self._text({})
        assert "ros2_lf_output_deadline_arm" not in self._text(None)

    def test_bounded_topic_nobody_publishes_is_refused(self):
        graph = _graph(publishers_used=())
        with pytest.raises(ValueError, match=r"no reaction of the node publishes that topic"):
            self._text({"controller": {"actuation": BOUND}}, graph=graph)

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
"""External topic input.

A subscription on a topic with no in-system publisher used to get an LF input
port that nothing connected, so its callback never ran. Now the topic must be
declared in execution.yaml ``external_topics:``. A declared topic's callback is
triggered by a physical action the ingress thread schedules at arrival, and an
undeclared one is a generation-time error.
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.model import ExecConfig, package_exec_config_path
from colcon_lf.generation.binding_header_emitter import collect_rid_table
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_naming import (
    action_to_goal_req_port_name,
    service_to_request_port_name,
)
from colcon_lf.generation.lf_generator import _stub_add, generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    SubscriberInfo,
)
from colcon_lf.verb.generate import _check_external_topics, _declare_external_topics


def _sub_node(name, cls, topic="chatter", cb="on_msg", group=None, groups=None):
    return NodeInfo(
        name=name,
        class_name=cls,
        package="test_pkg",
        source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name=cb)],
        subscribers=[SubscriberInfo(
            topic=topic, message_type="std_msgs/msg/String", callback=cb,
            callback_group=group,
        )],
        callback_groups=groups or [],
    )


def _pub_node(name, cls, topic="chatter"):
    return NodeInfo(
        name=name,
        class_name=cls,
        package="test_pkg",
        source_file=f"/fake/{name}.cpp",
        publishers=[PublisherInfo(topic=topic, message_type="std_msgs/msg/String")],
    )


def _graph(nodes, topics, launch="/fake/bringup.launch.py"):
    root = LaunchFileInfo(file_path=launch, package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={n.name: n for n in nodes},
        communication=SystemCommunication(topics=topics),
    )


def _listener_graph(launch="/fake/bringup.launch.py", **kw):
    """One node subscribing /chatter, nobody in the system publishing it."""
    node = _sub_node("listener", "ListenerNode", **kw)
    tc = TopicConnection(
        topic="/chatter", message_type="std_msgs/msg/String",
        publishers=[], subscribers=["listener"],
    )
    return _graph([node], [tc], launch=launch)


def _talker_listener_graph():
    tc = TopicConnection(
        topic="/chatter", message_type="std_msgs/msg/String",
        publishers=["talker"], subscribers=["listener"],
    )
    return _graph(
        [_pub_node("talker", "TalkerNode"), _sub_node("listener", "ListenerNode")], [tc],
    )


# --- yaml validation -------------------------------------------------------

class TestExternalTopicsYaml:
    def test_valid_declaration_resolves_slash_tolerantly(self):
        config = ExecConfig.from_dict(
            {"external_topics": ["chatter"]}, _listener_graph(), source="t",
        )
        assert config.external_topics == ["/chatter"]

    def test_absent_key_means_none(self):
        assert ExecConfig.from_dict({}, _listener_graph(), source="t").external_topics == []

    def test_unknown_topic_is_an_error(self):
        with pytest.raises(ValueError, match=r"external_topics: unknown topic '/chater'"):
            ExecConfig.from_dict(
                {"external_topics": ["/chater"]}, _listener_graph(), source="t",
            )

    def test_topic_published_inside_the_system_is_an_error(self):
        with pytest.raises(
            ValueError, match=r"topic '/chatter' is published inside the system by talker",
        ):
            ExecConfig.from_dict(
                {"external_topics": ["/chatter"]}, _talker_listener_graph(), source="t",
            )

    def test_not_a_list_is_an_error(self):
        with pytest.raises(ValueError, match="'external_topics' must be a list of topic names"):
            ExecConfig.from_dict(
                {"external_topics": "/chatter"}, _listener_graph(), source="t",
            )


# --- undeclared: generation-time error ---------------------------------------

class TestUndeclaredExternalTopic:
    def test_undeclared_topic_refuses_with_node_topic_and_yaml_hint(self, tmp_path, capsys):
        yaml_path = tmp_path / "config" / "execution.yaml"
        rc, declared, _anchors = _check_external_topics(
            graph=_listener_graph(),
            system_json=tmp_path / "system.json",
            exec_config_arg=None,
        )
        err = capsys.readouterr().err
        assert rc == 1
        assert declared == set()
        assert (
            "Error: node 'listener' (class ListenerNode) subscribes to '/chatter', which no "
            "node in this system publishes. Under rcllf its callback would never be "
            "dispatched." in err
        )
        assert "Refusing to generate instead of dropping their messages silently." in err
        assert f"declare the topic(s) external in {yaml_path}:" in err
        assert "    external_topics:\n      - /chatter\n" in err

    def test_one_error_per_node_instance_and_topic(self, tmp_path, capsys):
        """launch_features `_secondary` lists one node twice for a topic.

        Before the fix the same error line was printed once per listing.
        """
        tc = TopicConnection(
            topic="/chatter", message_type="std_msgs/msg/String",
            publishers=[], subscribers=["listener", "listener", "other"],
        )
        graph = _graph(
            [_sub_node("listener", "ListenerNode"), _sub_node("other", "ListenerNode")],
            [tc],
        )
        rc, _, _anchors = _check_external_topics(
            graph=graph, system_json=tmp_path / "system.json", exec_config_arg=None,
        )
        err = capsys.readouterr().err
        assert rc == 1
        assert err.count("Error: node 'listener' (class ListenerNode) subscribes to '/chatter'") == 1
        assert err.count("Error: node 'other' (class ListenerNode) subscribes to '/chatter'") == 1
        assert err.count("      - /chatter\n") == 1

    def test_declared_topic_passes(self, tmp_path, capsys):
        yaml_path = tmp_path / "execution.yaml"
        yaml_path.write_text("external_topics:\n  - /chatter\n")
        rc, declared, _anchors = _check_external_topics(
            graph=_listener_graph(),
            system_json=tmp_path / "system.json",
            exec_config_arg=str(yaml_path),
        )
        assert rc == 0
        assert declared == {"/chatter"}
        assert capsys.readouterr().err == ""

    def test_in_system_publisher_needs_no_declaration(self, tmp_path):
        rc, declared, _anchors = _check_external_topics(
            graph=_talker_listener_graph(),
            system_json=tmp_path / "system.json",
            exec_config_arg=None,
        )
        assert (rc, declared) == (0, set())


class TestExternalTakeMode:
    """Block cannot apply to physical-action input."""

    def _check(self, tmp_path, yaml_text, graph=None):
        yaml_path = tmp_path / "execution.yaml"
        yaml_path.write_text(yaml_text)
        rc, _, _anchors = _check_external_topics(
            graph=graph or _listener_graph(),
            system_json=tmp_path / "system.json",
            exec_config_arg=str(yaml_path),
        )
        return rc, yaml_path

    def test_block_on_an_external_subscription_is_refused(self, tmp_path, capsys):
        rc, yaml_path = self._check(
            tmp_path, "defaults: {take_mode: block}\nexternal_topics: [/chatter]\n",
        )
        err = capsys.readouterr().err
        assert rc == 1
        assert (
            "Error: take_mode 'block' on node 'listener' (binding key ListenerNode) for "
            f"external topic '/chatter', set by --exec-config {yaml_path} (defaults)." in err
        )
        assert "'block' cannot apply" in err
        assert "    subscriptions:\n      listener:\n        /chatter:\n" in err
        assert "          take_mode: skip" in err

    def test_skip_for_the_external_subscription_passes(self, tmp_path, capsys):
        rc, _ = self._check(
            tmp_path,
            "defaults: {take_mode: block}\n"
            "subscriptions: {listener: {/chatter: {take_mode: skip}}}\n"
            "external_topics: [/chatter]\n",
        )
        assert rc == 0
        assert capsys.readouterr().err == ""

    def test_block_on_an_internal_subscription_stays_allowed(self, tmp_path, capsys):
        rc, _, _anchors = _check_external_topics(
            graph=_talker_listener_graph(),
            system_json=tmp_path / "system.json",
            exec_config_arg=str(self._check(tmp_path, "defaults: {take_mode: block}\n",
                                            graph=_talker_listener_graph())[1]),
        )
        assert rc == 0


# --- package-owned lf/<launch stem>.execution.yaml ---------------------------

def _package(tmp_path, yaml_text=None):
    """A package dir with launch/bringup.launch.py and optionally lf/bringup.execution.yaml."""
    pkg = tmp_path / "test_pkg"
    (pkg / "launch").mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>")
    launch = pkg / "launch" / "bringup.launch.py"
    launch.write_text("")
    if yaml_text is not None:
        (pkg / "lf").mkdir()
        (pkg / "lf" / "bringup.execution.yaml").write_text(yaml_text)
    return pkg, launch


class TestPackageExecConfig:
    def test_path_is_lf_dir_next_to_the_package_xml(self, tmp_path):
        pkg, launch = _package(tmp_path)
        assert package_exec_config_path(_listener_graph(launch=str(launch))) == (
            pkg / "lf" / "bringup.execution.yaml"
        )

    def test_no_enclosing_package_means_none(self):
        assert package_exec_config_path(_listener_graph()) is None

    def test_package_file_declares_without_a_flag(self, tmp_path, capsys):
        _, launch = _package(tmp_path, "external_topics:\n  - /chatter\n")
        analysis = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        rc, declared, _anchors = _check_external_topics(
            graph=_listener_graph(launch=str(launch)),
            system_json=analysis / "system.json", exec_config_arg=None,
        )
        assert (rc, declared) == (0, {"/chatter"})
        assert capsys.readouterr().err == ""

    def test_scaffolded_workspace_file_does_not_mask_the_package_file(self, tmp_path):
        """`colcon lf` scaffolds config/execution.yaml with every key empty."""
        _, launch = _package(tmp_path, "external_topics:\n  - /chatter\n")
        ws = tmp_path / "execution.yaml"
        ws.write_text("defaults: {}\nnodes: {}\nsubscriptions: {}\nacknowledged_hazards: []\n")
        graph = _listener_graph(launch=str(launch))
        config = ExecConfig.load_layered(ws, package_exec_config_path(graph), graph)
        assert config.external_topics == ["/chatter"]

    def test_declaration_lists_are_the_union_of_both_layers(self, tmp_path):
        _, launch = _package(
            tmp_path, "external_topics:\n  - /chatter\nacknowledged_hazards: [H1]\n",
        )
        ws = tmp_path / "execution.yaml"
        ws.write_text("acknowledged_hazards: [H2]\n")
        graph = _listener_graph(launch=str(launch))
        config = ExecConfig.load_layered(ws, package_exec_config_path(graph), graph)
        assert config.acknowledged_hazards == ["H1", "H2"]
        assert config.external_topics == ["/chatter"]

    def test_undeclared_hint_names_the_package_file(self, tmp_path, capsys):
        pkg, launch = _package(tmp_path)
        rc, _, _anchors = _check_external_topics(
            graph=_listener_graph(launch=str(launch)),
            system_json=tmp_path / "system.json", exec_config_arg=None,
        )
        assert rc == 1
        assert f"external in {pkg / 'lf' / 'bringup.execution.yaml'}:" in capsys.readouterr().err


# --- generator emission -----------------------------------------------------

def _reactor(system, name):
    return next(r for r in system.reactors if r.name == name)


class TestExternalEmission:
    def test_without_declaration_the_old_shape_is_an_unconnected_port(self):
        """The old shape: an input port nothing feeds and no physical action."""
        system = generate_lf_system(_listener_graph(), target_lfc=True)
        text = emit_lf_system(system)
        assert "physical action ext_" not in text
        assert "ros2_lf_invoke_external_sub" not in text

    def test_declared_topic_gets_physical_action_and_register(self):
        system = generate_lf_system(
            _listener_graph(), target_lfc=True, external_topics={"/chatter"},
        )
        text = emit_lf_system(system)
        assert "physical action ext_on_msg: int64_t" in text
        assert "ros2_lf_external_register(self, " in text
        assert ", rcllf_external_wake, ext_on_msg);" in text
        assert "lf_schedule_copy(action, 0, &record, 1)" in text  # the wake thunk
        assert "ros2_lf_invoke_external_sub(self, " in text
        assert ", ext_on_msg->value);" in text
        # The callback reaction is triggered by the action, not by a port.
        reaction = next(
            r for rr in system.reactors for r in rr.reactions
            if "ros2_lf_invoke_external_sub" in r.body
        )
        assert reaction.triggers == ["ext_on_msg"]
        # No reactor keeps an input port for the external topic.
        for r in system.reactors:
            inputs = [p.name for p in r.ports if p.direction == "input"]
            assert all("chatter" not in n for n in inputs), (r.name, inputs)
        assert "ros2_lf_invoke_sub(" not in text

    def test_rid_row_carries_the_topic(self):
        """The binding matches the subscription handle by topic at bind time.

        Found in the first rcllf run of callback_group_example: an empty topic
        in the RID row made ros2_lf_bind_node refuse with "RID 5
        (CallbackGroupNode::chatter_callback, sub, topic '') has no unbound
        subscription handle".
        """
        graph = _listener_graph()
        system = generate_lf_system(graph, target_lfc=True, external_topics={"/chatter"})
        table = collect_rid_table({r.name: r for r in system.reactors}, graph)
        rows = [row for row in table if row[2] == "on_msg"]
        assert len(rows) == 1
        assert rows[0][3:5] == ("sub", "chatter"), rows[0]

    def test_group_subreactor_gets_the_action(self):
        groups = [CallbackGroupInfo(name="sub_group_", type="MutuallyExclusive")]
        system = generate_lf_system(
            _listener_graph(group="sub_group_", groups=groups),
            target_lfc=True, external_topics={"/chatter"},
        )
        owners = [r.name for r in system.reactors if r.external_inputs]
        assert len(owners) == 1
        owner = _reactor(system, owners[0])
        assert owner.external_inputs[0][0] == "ext_on_msg"
        assert any(
            "ros2_lf_invoke_external_sub" in x.body for x in owner.reactions
        ), owner.name


class TestExternalSenders:
    """An external callback may send a request or goal.
    """

    def _sender_graph(self, *, service=False, action=False):
        graph = _listener_graph()
        node = graph.nodes["listener"]
        if service:
            node.service_clients = [ServiceClientInfo(
                service_name="process_data", service_type="custom_interfaces/srv/ProcessData",
                call_style="async_callback", response_callback="on_response",
            )]
            node.callbacks[0].service_clients_used = ["process_data"]
            node.callbacks.append(CallbackInfo(name="on_response"))
        if action:
            node.action_clients = [ActionClientInfo(
                action_name="analyze", action_type="custom_interfaces/action/Analyze",
            )]
            node.callbacks[0].action_clients_used = ["analyze"]
        return graph

    def _external_reaction(self, graph):
        system = generate_lf_system(graph, target_lfc=True, external_topics={"/chatter"})
        (reaction,) = [
            r for rr in system.reactors for r in rr.reactions
            if "ros2_lf_invoke_external_sub" in r.body
        ]
        table = collect_rid_table({r.name: r for r in system.reactors}, graph)
        (row,) = [row for row in table if row[2] == "on_msg"]
        return reaction, row

    @pytest.mark.parametrize(
        "kw, port",
        [
            ({"service": True}, service_to_request_port_name("process_data")),
            ({"action": True}, action_to_goal_req_port_name("analyze")),
        ],
        ids=["service_request", "action_goal"],
    )
    def test_sender_stays_an_external_subscription_that_sets_the_port(self, kw, port):
        reaction, row = self._external_reaction(self._sender_graph(**kw))
        assert reaction.triggers == ["ext_on_msg"]
        assert reaction.kind == "sub"
        assert port in reaction.effects
        if "action" in kw:
            # An action port carries the count of goals really sent.
            assert 'ros2_lf_action_sent("analyze", RCLLF_ACTION_GOAL_REQ)' in reaction.body
            assert f"lf_set({port}, " in reaction.body
        else:
            # One write per callback the binding ran.
            assert "int rcllf_ran = ros2_lf_invoke_external_sub(self, " in reaction.body
            assert (f"if (rcllf_ran > 0) {{ {_stub_add(port, 'rcllf_ran')} }}"
                    in reaction.body), reaction.body
        # Bound as the subscription the ingress thread feeds, not as a timer.
        assert row[3:5] == ("sub", "chatter"), row


class TestStartupDeclaresExternalTopics:
    """The readiness wait skips external topics."""

    START = (
        "    char** argv = const_cast<char**>(argv0);\n"
        "    ros2_lf_runtime_init(argc, argv);\n"
        "    ros2_lf_runtime_wait_ready();\n"
    )

    def test_declarations_come_before_init_and_the_wait(self):
        # Before init, because init logs the ROS2_LF_TAKE_MODE override that
        # leaves external subscriptions out.
        text = _declare_external_topics(self.START, {"/b", "/a"})
        assert text == (
            "    char** argv = const_cast<char**>(argv0);\n"
            '    ros2_lf_runtime_declare_external_topic("/a");\n'
            '    ros2_lf_runtime_declare_external_topic("/b");\n'
            "    ros2_lf_runtime_init(argc, argv);\n"
            "    ros2_lf_runtime_wait_ready();\n"
        )

    def test_no_declaration_leaves_the_startup_alone(self):
        assert _declare_external_topics(self.START, set()) == self.START

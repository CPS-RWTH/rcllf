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
"""Launch remaps to a different topic leaf name.

The analyzer resolves every endpoint through the launch remaps, so a topic
edge carries the remapped name. Port names stay the node's declared names,
and the connection between them follows the remapped topic. Before the fix
the generator named the remapped leaf as a port, and lfc rejected it with
``Not a port`` and ``Couldn't resolve reference to Variable``.
"""

from __future__ import annotations

import re
import shutil
import subprocess

import pytest

from colcon_lf.comm_analysis.topic_mapper import build_communication
from colcon_lf.generation.factory_emitter import ros_args_for
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.generation.node_identity import build_node_identities
from colcon_lf.verb.generate import _factory_entries, build_component_instances

from ..corr_declared import generate_declared

MSG = "std_msgs::msg::Int32"


def _pub(name, cls, topic="data") -> NodeInfo:
    return NodeInfo(
        name=name, class_name=cls, package="test_pkg", source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=[topic])],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        publishers=[PublisherInfo(topic=topic, message_type=MSG)],
    )


def _sub(name, cls, topics=("data",)) -> NodeInfo:
    return NodeInfo(
        name=name, class_name=cls, package="test_pkg", source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name=f"on_{i}") for i, _ in enumerate(topics)],
        subscribers=[SubscriberInfo(topic=t, message_type=MSG, callback=f"on_{i}")
                     for i, t in enumerate(topics)],
    )


def _graph(entries) -> SystemGraph:
    """*entries*: ``(NodeInfo, remappings)`` pairs, joined through a real launch model."""
    actions = [
        LaunchNodeAction(package="test_pkg", executable=ni.name, name=ni.name,
                         remappings=list(remaps))
        for ni, remaps in entries
    ]
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=actions),
        nodes={ni.name: ni for ni, _ in entries},
        communication=build_communication(
            [(a, ni) for a, (ni, _) in zip(actions, entries)]),
    )


def _lfc(tmp_path, text: str) -> subprocess.CompletedProcess:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    src = tmp_path / "src" / "Bringup.lf"
    src.parent.mkdir(parents=True)
    src.write_text(text)
    return subprocess.run([lfc, "--no-compile", str(src)], capture_output=True, text=True,
                          timeout=300)


def _assert_lfc_accepts(tmp_path, text: str) -> None:
    """lfc's own diagnostics are the failure message, then the program text."""
    proc = _lfc(tmp_path, text)
    errors = [ln for ln in (proc.stdout + proc.stderr).splitlines() if "error" in ln]
    assert proc.returncode == 0, "lfc rejected the program:\n" + "\n".join(errors) + "\n" + text


def _connections(text: str) -> list[str]:
    return re.findall(r"^\s*(\w+\.\w+ -> \w+\.\w+)", text, flags=re.M)


REMAP_BOTH = [(_pub("pub", "PubNode"), [("data", "data_remapped")]),
              (_sub("sub", "SubNode"), [("data", "data_remapped")])]


def test_analyzer_resolves_the_edge_to_the_remapped_name():
    graph = _graph(REMAP_BOTH)
    assert [t.topic for t in graph.communication.topics] == ["/data_remapped"]


def test_legacy_link_names_the_declared_ports(tmp_path):
    # No burst bound is declared, so the edge stays a legacy stub link.
    system = generate_lf_system(_graph(REMAP_BOTH), after_delay_ns=1_000_000)
    text = emit_lf_system(system)
    _assert_lfc_accepts(tmp_path, text)
    assert _connections(text)[-1:] == ["pub.data -> sub.data"], text
    assert "data_remapped" not in "\n".join(_connections(text)), text


def test_admitted_edge_resolves_the_declared_endpoints(tmp_path):
    system = generate_declared(_graph(REMAP_BOTH), after_delay_ns=1_000_000)
    _assert_lfc_accepts(tmp_path, emit_lf_system(system))
    (e,) = system.corr_edges
    assert (e.status, e.pub_topic, e.sub_topic) == ("admitted", "data", "data"), e


def test_subscriber_side_remap_connects_to_the_declared_input(tmp_path):
    graph = _graph([(_pub("pub", "PubNode"), []),
                    (_sub("sub", "SubNode", topics=("in",)), [("in", "data")])])
    text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert "pub.data -> sub.in" in _connections(text), text


def test_a_remapped_away_endpoint_no_longer_matches_its_old_leaf(tmp_path):
    # sub's 'data' goes elsewhere, its 'feed' is what now reads /data. The
    # path-suffix rule alone would pick 'data' for /data.
    graph = _graph([(_pub("pub", "PubNode"), []),
                    (_sub("sub", "SubNode", topics=("data", "feed")),
                     [("data", "/elsewhere"), ("feed", "data")])])
    text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert "pub.data -> sub.feed" in _connections(text), text
    assert "pub.data -> sub.data" not in _connections(text), text


def test_launch_remaps_reach_the_runtime_identity():
    graph = _graph(REMAP_BOTH)
    ids = build_node_identities(graph)
    assert ids["pub"].remappings == (("data", "data_remapped"),)
    assert ros_args_for("pub", "", ids["pub"].remappings) == [
        "--ros-args", "-r", "__node:=pub", "-r", "data:=data_remapped"]
    assert ros_args_for("", "", ()) == []
    entries = _factory_entries(list(graph.nodes.items()), {k: k for k in graph.nodes}, ids)
    assert entries[0]["remappings"] == [["data", "data_remapped"]]
    graph.nodes["sub"].is_component = True
    comps = build_component_instances(graph, {k: k for k in graph.nodes}, ids)
    assert [c["remap_rules"] for c in comps] == [["data:=data_remapped"]]


def test_an_unremapped_endpoint_does_not_capture_a_remapped_edge(tmp_path):
    # Sub's 'feed' is remapped to /other/data, its 'data' is
    # not and resolves to /data. The suffix rule alone also matched 'data' to
    # /other/data, and the ambiguity fallback wired sub.data.
    graph = _graph([(_pub("pub", "PubNode", "/other/data"), []),
                    (_sub("sub", "SubNode", topics=("data", "feed")),
                     [("feed", "/other/data")])])
    text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert _connections(text) == ["pub.other_data -> sub.feed"], text


def test_two_subscriptions_joined_by_a_remap_get_one_link_each(tmp_path):
    # 'a' and 'b' both remapped to /shared. Before, the
    # generator wired the resolved leaf, sub.shared, which is not a port.
    graph = _graph([(_pub("pub", "PubNode", "/shared"), []),
                    (_sub("sub", "SubNode", topics=("a", "b")),
                     [("a", "/shared"), ("b", "/shared")])])
    text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert _connections(text) == ["pub.shared -> sub.a", "pub.shared -> sub.b"], text


def test_two_publications_joined_by_a_remap_are_refused_by_name(tmp_path):
    # One stub input takes one connection, so two declared
    # publishers remapped onto one topic are refused with a diagnostic, never
    # wired as the resolved leaf (pub.shared, not a port).
    pub = _pub("pub", "PubNode", "a")
    pub.publishers.append(PublisherInfo(topic="b", message_type=MSG))
    pub.callbacks[0].publishers_used.append("b")
    graph = _graph([(pub, [("a", "/shared"), ("b", "/shared")]),
                    (_sub("sub", "SubNode", topics=("shared",)), [])])
    try:
        text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    except ValueError as exc:
        assert "publishers a, b all resolve to '/shared'" in str(exc), exc
        return
    _assert_lfc_accepts(tmp_path, text)
    assert not any(c.startswith("pub.shared ") for c in _connections(text)), text


def _unscoped(nodes):
    """Hand-built graph, no launch actions: the nodes have no launch identity."""
    from .test_correspondence_emission import _graph as _bare_graph, _tc
    return _bare_graph(nodes, [_tc("/ns/data", ["pub"], ["sub"])])


def test_unconfirmed_publisher_suffix_matches_keep_the_fallback(tmp_path):
    # Re-review: without a launch identity 'data' and '/other/ns/data'
    # both suffix-match /ns/data, and neither is confirmed by its resolved
    # name. That is ambiguity, not two publishers, so the old fallback stays.
    pub = _pub("pub", "PubNode", "data")
    pub.publishers.append(PublisherInfo(topic="/other/ns/data", message_type=MSG))
    pub.callbacks[0].publishers_used.append("/other/ns/data")
    text = emit_lf_system(generate_lf_system(
        _unscoped([pub, _sub("sub", "SubNode")]), after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert _connections(text) == ["pub.data -> sub.data"], text


def test_unconfirmed_subscriber_suffix_matches_add_no_link(tmp_path):
    # Re-review: the same ambiguity on the subscriber side must not
    # link sub.other_ns_data as well.
    text = emit_lf_system(generate_lf_system(
        _unscoped([_pub("pub", "PubNode", "data"),
                   _sub("sub", "SubNode", topics=("data", "/other/ns/data"))]),
        after_delay_ns=0))
    _assert_lfc_accepts(tmp_path, text)
    assert _connections(text) == ["pub.data -> sub.data"], text

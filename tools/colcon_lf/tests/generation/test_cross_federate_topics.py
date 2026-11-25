# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A federate's system.json lists the topics that cross its boundary.

The per-federate system.json keeps only topics whose every endpoint is local, so
the runtime's match warm-up needs the crossing topics and their system-wide
endpoint counts under ``cross_federate_topics``.
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

from colcon_lf.comm_analysis.topic_mapper import build_communication
from colcon_lf.federation.partition import FederationMap
from colcon_lf.node_analysis.models import NodeInfo, PublisherInfo, SubscriberInfo
from colcon_lf.verb.generate import (
    CROSS_FEDERATE_TOPICS_KEY, _generate_federated, cross_federate_topics,
)
from ..corr_declared import declared
from .test_correspondence_emission import _graph, _pub, _sub, _tc

FED = "lf_generated_test_pkg_bringup_federated"


def test_cross_federate_topics_counts_whole_system():
    topics = [
        {"topic": "/local", "publishers": ["a"], "subscribers": ["b"]},
        {"topic": "/out", "publishers": ["a"], "subscribers": ["c", "d"]},
        {"topic": "/in", "publishers": ["c", "a"], "subscribers": ["b"]},
        {"topic": "/remote", "publishers": ["c"], "subscribers": ["d"]},
        {"topic": "/clock", "publishers": ["a"], "subscribers": []},
    ]
    got = cross_federate_topics(topics, {"a", "b"}, {})
    assert got == [
        {"topic": "/out", "publishers": 1, "subscribers": 2},
        {"topic": "/in", "publishers": 2, "subscribers": 1},
    ]


def test_cross_federate_topics_maps_runtime_names_to_keys():
    topics = [{"topic": "/t", "publishers": ["ros_a"], "subscribers": ["ros_b"]}]
    assert cross_federate_topics(topics, {"KeyA"}, {"ros_a": "KeyA", "ros_b": "KeyB"}) == [
        {"topic": "/t", "publishers": 1, "subscribers": 1},
    ]
    assert cross_federate_topics(topics, {"KeyA", "KeyB"}, {"ros_a": "KeyA", "ros_b": "KeyB"}) == []



def test_cross_federate_topics_skips_lifecycle_hook_endpoints():
    """A reader a lifecycle node makes in on_configure has no
    DDS entity before tag 0; the peer check skips it, so must the counts."""
    topics = [{"topic": "/t", "publishers": ["a"], "subscribers": ["lc", "plain"]}]
    nodes = {
        "lc": {"base_class": "rclcpp_lifecycle::LifecycleNode", "subscribers": [
            {"topic": "t", "created_in": "on_configure"}]},
        "plain": {"base_class": "rclcpp::Node", "subscribers": [
            {"topic": "t", "created_in": "on_configure"}]},
    }
    assert cross_federate_topics(topics, {"a"}, {}, nodes) == [
        {"topic": "/t", "publishers": 1, "subscribers": 1},
    ]
    nodes["lc"]["subscribers"][0]["created_in"] = "constructor"
    assert cross_federate_topics(topics, {"a"}, {}, nodes) == [
        {"topic": "/t", "publishers": 1, "subscribers": 2},
    ]

def test_cross_federate_topics_counts_twin_endpoints_of_one_node():
    """A node with two readers on one topic.

    The analysis lists the node once per endpoint, and the writer's DDS count
    counts both readers, so the warm-up target must be 2. A count of nodes
    (1) would let the writer lift its barrier with the late twin unmatched.
    """
    twin = NodeInfo(name="twin", class_name="Twin", package="p", source_file=Path("t.cpp"))
    twin.subscribers = [
        SubscriberInfo(topic="/t", message_type="std_msgs::msg::Int32", callback="first"),
        SubscriberInfo(topic="/t", message_type="std_msgs::msg::Int32", callback="late"),
    ]
    pub = NodeInfo(name="pub", class_name="Pub", package="p", source_file=Path("p.cpp"))
    pub.publishers = [PublisherInfo(topic="/t", message_type="std_msgs::msg::Int32")]
    comm = build_communication([(None, twin), (None, pub)])
    topics = [dataclasses.asdict(t) for t in comm.topics]
    assert topics[0]["subscribers"] == ["twin", "twin"]
    got = cross_federate_topics(topics, {"pub"}, {})
    assert got == [{"topic": "/t", "publishers": 1, "subscribers": 2}], (
        f"the twin node's two readers counted as {got[0]['subscribers']}: the writer would "
        "lift the warm-up barrier with one reader unmatched")


def test_cross_federate_topics_counts_each_endpoint_of_a_mixed_lifecycle_node():
    """A lifecycle node with one reader made in its constructor and one in a hook.

    Only the constructor's reader exists before tag 0. Counting both would make
    the writer wait for a reader that comes only after tag 0 and refuse a
    sound start at the bound.
    """
    topics = [{"topic": "/t", "publishers": ["a"], "subscribers": ["lc", "lc"]}]
    nodes = {"lc": {"base_class": "rclcpp_lifecycle::LifecycleNode", "subscribers": [
        {"topic": "t", "created_in": "constructor"},
        {"topic": "t", "created_in": "on_configure"}]}}
    got = cross_federate_topics(topics, {"a"}, {}, nodes)
    assert got == [{"topic": "/t", "publishers": 1, "subscribers": 1}], (
        f"the hook-made reader of a mixed lifecycle node counted ({got[0]['subscribers']} "
        "readers wanted): the writer waits for a reader that exists only after tag 0")
    nodes["lc"]["subscribers"][1]["created_in"] = "constructor"
    assert cross_federate_topics(topics, {"a"}, {}, nodes) == [
        {"topic": "/t", "publishers": 1, "subscribers": 2},
    ]


def test_federate_system_json_carries_cross_topics(tmp_path: Path):
    nodes = [_pub("pub", "PubNode"), _sub("sub", "SubNode")]
    topics = [_tc("/data", ["pub"], ["sub"])]
    src = tmp_path / "packages/src/test_pkg/src"
    src.mkdir(parents=True)
    real = []
    for n in nodes:
        f = src / f"{n.name}.cpp"
        f.write_text("// generated by the test\n")
        real.append(dataclasses.replace(n, source_file=str(f)))
    graph = _graph(real, topics)
    adir = tmp_path / "lf-analysis/test_pkg/bringup"
    adir.mkdir(parents=True)
    sj = adir / "system.json"
    sj.write_text(json.dumps({
        "name": "bringup", "nodes": {},
        "communication": {"topics": [
            {"topic": "/data", "publishers": ["pub"], "subscribers": ["sub"]},
        ]},
    }))
    fmap = FederationMap(node_to_federate={"pub": "pub", "sub": "sub"},
                         federate_names=["pub", "sub"], after_overrides=[],
                         coordination="centralized")
    rc = _generate_federated(system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec",
                             generate_launch=False, corr_bounds=declared(graph, 1))
    assert rc == 0
    gen = tmp_path / "lf-generated"
    for fed in ("pub", "sub"):
        doc = json.loads((gen / f"{FED}_{fed}/system.json").read_text())
        assert doc["communication"]["topics"] == []
        assert doc[CROSS_FEDERATE_TOPICS_KEY] == [
            {"topic": "/data", "publishers": 1, "subscribers": 1},
        ]

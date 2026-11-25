# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A cross-federate legacy topic link carries the writer's tag log.

The writer federate packs the write count with its tag log events of the
topic into an ``rcllf_xtag_t`` and the receiver imports them before the take,
so a late sample is dropped by tag as in one process. Publisher nodes of one
federate leave on one link each, packed per link; publisher nodes in two
federates are refused, since their events are numbered apart. A link
generated with ``RCLLF_COUNT_PUBLISHES=0`` carries only a count, and the
receiving federate warns at startup naming the edge.

The fixtures declare no burst bound, so the edge is not admitted to
correspondence and stays a legacy link. A decentralized one-publisher edge is
admitted and crosses as correspondence records instead.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.federation.partition import AfterOverride, FederationMap
from colcon_lf.generation.lf_generator import xtag_publisher_errors
from colcon_lf.node_analysis.models import PublisherInfo
from colcon_lf.verb import generate as gen
from ..corr_declared import declared
from .test_correspondence_emission import _graph, _pub, _sub, _tc
from .test_fan_in_tag_split import TOGETHER, _generate_federated, _nodes

APART = {"pa": "pa", "sub": "sub"}
# No burst bound declared: the edge is not admitted and keeps the legacy link.
UNDECLARED = CorrBoundsConfig(burst={})


def _lf(tmp_path: Path, federate: str) -> str:
    files = list(tmp_path.rglob(f"lf/{federate}/*.lf"))
    assert len(files) == 1, files
    return files[0].read_text()


def _lines(text: str) -> set[str]:
    return {line.strip() for line in text.splitlines()}


def _one_pub(tmp_path: Path, coordination: str) -> int:
    return _generate_federated(tmp_path, APART, (), coordination,
                               topics=[_tc("/data", ["pa"], ["sub"])], corr_bounds=UNDECLARED)


def test_writer_federate_packs_its_tag_log(tmp_path):
    assert _one_pub(tmp_path, "decentralized") == 0
    pa = _lf(tmp_path, "pa")
    assert "output data: rcllf_xtag_t" in pa, pa
    assert "reaction(pa.data) -> data {=" in pa, pa
    assert 'ros2_lf_xtag_pack("/data", "data", pa.data->value, &rcllf_x);' in pa, pa
    assert "pa.data -> data" not in _lines(pa), "the plain forwarding connection is still emitted"


def test_receiver_federate_imports_before_the_take(tmp_path):
    assert _one_pub(tmp_path, "decentralized") == 0
    sub = _lf(tmp_path, "sub")
    assert "input data: rcllf_xtag_t" in sub, sub
    imp = 'ros2_lf_xtag_import("/data", "pa.data -> sub.data", &data->value);'
    # Once in the body, once in the STP handler: a tardy message is imported too.
    assert sub.count(imp) == 2, sub
    assert "reaction(data) -> sub.data {=" in sub, sub
    assert "=} STP(" in sub, sub
    assert "data -> sub.data" not in _lines(sub), "the plain forwarding connection is still emitted"
    # The receiver learns how many gated publishers its one writer federate
    # announces, before the first import runs.
    expect = 'ros2_lf_xtag_expect("/data", 1);'
    assert sub.count(expect) == 1, sub
    assert sub.index(expect) < sub.index(imp), "the expectation is declared after the import"
    assert "ros2_lf_xtag_expect" not in _lf(tmp_path, "pa"), "the writer federate expects writers"


@pytest.mark.parametrize("coordination", ["centralized", "decentralized"])
def test_two_publishers_of_one_federate_pack_per_link(tmp_path, monkeypatch, coordination):
    # Regression: one shared pack cursor sent the second link an empty
    # message past the receiver's next ordinal, which broke the receiver's log.
    # d = 0 is admitted; 1 ns under the flush stays legacy.
    monkeypatch.setenv("RCLLF_XFED_FLUSH", "1")
    overrides = [AfterOverride("pa.data", "sub.data", 1),
                 AfterOverride("pb.data", "sub.data", 1)]
    assert _generate_federated(tmp_path, TOGETHER, overrides, coordination) == 0
    pubs = _lf(tmp_path, "pubs")
    assert 'ros2_lf_xtag_pack("/data", "data", pa.data->value, &rcllf_x);' in pubs, pubs
    assert 'ros2_lf_xtag_pack("/data", "pb_data", pb.data->value, &rcllf_x);' in pubs, pubs
    sub = _lf(tmp_path, "sub")
    assert 'ros2_lf_xtag_import("/data", "pa.data -> sub.data", &data->value);' in sub, sub
    assert 'ros2_lf_xtag_import("/data", "pb.data -> sub.data", &data_fanin_1->value);' \
        in sub, sub
    # Third review: two publisher nodes of one writer federate, so
    # the receiver waits for two announced publishers before it reads an
    # unannounced writer as plain.
    assert sub.count('ros2_lf_xtag_expect("/data", 2);') == 1, sub


def test_expected_publishers_count_every_publisher_of_the_topic(tmp_path):
    # A node with two publishers of /data announces two GIDs: the receiver
    # waits for both, so the one created later is never taken for plain.
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = pa.publishers + [dataclasses.replace(pa.publishers[0])]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    adir = tmp_path / "lf-analysis/test_pkg/bringup"
    adir.mkdir(parents=True, exist_ok=True)
    sj = adir / "system.json"
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate=dict(APART), federate_names=["pa", "sub"],
                         coordination="centralized")
    assert gen._generate_federated(system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec",
                                   generate_launch=False, corr_bounds=declared(graph, 1)) == 0
    sub = _lf(tmp_path, "sub")
    assert sub.count('ros2_lf_xtag_expect("/data", 2);') == 1, sub


def _conn(src: str, dst: str) -> SimpleNamespace:
    return SimpleNamespace(source=src, destination=dst, topic="/data")


def test_publishers_in_two_federates_are_refused_naming_topic_and_publishers():
    feds = {"pa": "fa", "pb": "fb", "sub": "fs"}
    errors = xtag_publisher_errors(
        [_conn("pa.data", "sub.data"), _conn("pb.data", "sub.data")], feds.get)
    assert errors and errors[0].startswith(
        "cross-federate take link on topic /data: publisher nodes pa (federate fa), "
        "pb (federate fb) publish from different federates"), errors


def test_publishers_of_one_federate_are_not_refused():
    feds = {"pa": "pubs", "pb": "pubs", "sub": "fs"}
    assert xtag_publisher_errors(
        [_conn("pa.data", "sub.data"), _conn("pb.data", "sub.data")], feds.get) == []


def test_count_only_link_warns_at_startup_naming_the_edge(tmp_path, monkeypatch):
    monkeypatch.setenv("RCLLF_COUNT_PUBLISHES", "0")
    assert _one_pub(tmp_path, "decentralized") == 0
    sub = _lf(tmp_path, "sub")
    assert "input data: stub_t" in sub, sub
    assert "cross-federate take link pa.data -> sub.data on topic '/data' carries only a " \
        "count (generated with RCLLF_COUNT_PUBLISHES=0)" in sub, sub
    assert "rcllf_xtag" not in sub + _lf(tmp_path, "pa")
    assert "ros2_lf_xtag_expect" not in sub


def _run(tmp_path: Path, graph, node_fed: dict[str, str]) -> int:
    """Generate *graph* federated by *node_fed* under decentralized coordination."""
    adir = tmp_path / "lf-analysis/test_pkg/bringup"
    adir.mkdir(parents=True, exist_ok=True)
    sj = adir / "system.json"
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate=dict(node_fed),
                         federate_names=list(dict.fromkeys(node_fed.values())),
                         coordination="decentralized")
    return gen._generate_federated(system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec",
                                   generate_launch=False, corr_bounds=UNDECLARED)


def test_namespaced_node_counts_both_publishers_of_its_relative_name(tmp_path):
    # Fourth review (A): a node in namespace /robot with two
    # publishers named "data" writes /robot/data. The count matched only
    # "/data", fell back to 1, and the second publisher's sample was read as
    # plain, a tag early. Both publishers resolve to /robot/data: expect 2.
    from colcon_lf.launch_analysis.models import LaunchNodeAction
    nodes = {n.name: n for n in _nodes(tmp_path)}
    pa, sub = nodes["pa"], nodes["sub"]
    pa.publishers = pa.publishers + [dataclasses.replace(pa.publishers[0])]
    graph = _graph([], [_tc("/robot/data", ["/robot/pa"], ["/robot/sub"])])
    graph.nodes = {"/robot/pa": pa, "/robot/sub": sub}
    graph.launch_graph.all_nodes = [
        LaunchNodeAction(package="test_pkg", executable=n, name=n, namespace="/robot")
        for n in ("pa", "sub")]
    assert _run(tmp_path, graph, {"/robot/pa": "pa", "/robot/sub": "sub"}) == 0
    sub_lf = _lf(tmp_path, "sub")
    assert sub_lf.count('ros2_lf_xtag_expect("/robot/data", 2);') == 1, sub_lf


def test_lifecycle_publisher_keeps_the_count_rule_with_the_reason(tmp_path):
    # Fourth review (B): a lifecycle node's publisher made at
    # configure may never exist, so an expected count of 1 would hold a plain
    # writer's samples for good. The link keeps the count rule and the
    # receiver warns at startup naming the edge and the reason.
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.base_class = "rclcpp_lifecycle::LifecycleNode"
    pa.publishers = [dataclasses.replace(pa.publishers[0], is_lifecycle=True,
                                         created_in="on_configure")]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert "ros2_lf_xtag_expect" not in sub, sub
    assert "rcllf_xtag" not in sub + _lf(tmp_path, "pa")
    assert "input data: stub_t" in sub, sub
    assert "cross-federate take link pa.data -> sub.data on topic '/data' carries only a " \
        "count (publisher count not exact: node pa creates a publisher of /data in " \
        "on_configure, which may never run)" in sub, sub


def test_lifecycle_publisher_made_in_the_constructor_is_counted(tmp_path):
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.base_class = "rclcpp_lifecycle::LifecycleNode"
    pa.publishers = [dataclasses.replace(pa.publishers[0], is_lifecycle=True,
                                         created_in="constructor")]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert sub.count('ros2_lf_xtag_expect("/data", 1);') == 1, sub


def test_dynamic_publisher_name_keeps_the_count_rule(tmp_path):
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = pa.publishers + [PublisherInfo(topic="", message_type="x",
                                                   topic_dynamic=True)]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert "ros2_lf_xtag_expect" not in sub, sub
    assert "publisher count not exact: node pa has a publisher with a dynamic or " \
        "unresolved topic name" in sub, sub


@pytest.mark.parametrize("unconditional", [False, None])
def test_conditionally_made_second_publisher_keeps_the_count_rule(tmp_path, unconditional):
    # Fifth review: a constructor that makes a second publisher of
    # /data only when a parameter is set (unconditional False), or one the
    # analysis did not decide (None), may announce 1 publisher where 2 are
    # expected, holding a writer outside rcllf for good. Count rule + WARN.
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = pa.publishers + [dataclasses.replace(pa.publishers[0],
                                                         unconditional=unconditional)]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert "ros2_lf_xtag_expect" not in sub, sub
    assert "input data: stub_t" in sub, sub
    assert "cross-federate take link pa.data -> sub.data on topic '/data' carries only a " \
        "count (publisher count not exact: conditional creation: " in sub, sub


def test_two_publishers_made_on_every_path_in_the_constructor_are_expected(tmp_path):
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = pa.publishers + [dataclasses.replace(pa.publishers[0])]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert sub.count('ros2_lf_xtag_expect("/data", 2);') == 1, sub
    assert "carries only a count" not in sub, sub


def test_publisher_with_unknown_creation_site_keeps_the_count_rule(tmp_path):
    # created_in None: no clang analysis said where the publisher is made.
    nodes = _nodes(tmp_path)
    pa = next(n for n in nodes if n.name == "pa")
    pa.publishers = [dataclasses.replace(pa.publishers[0], created_in=None)]
    graph = _graph(nodes, [_tc("/data", ["pa"], ["sub"])])
    assert _run(tmp_path, graph, APART) == 0
    sub = _lf(tmp_path, "sub")
    assert "ros2_lf_xtag_expect" not in sub, sub
    assert "carries only a count (publisher count not exact: creation site unknown: " \
        "the analysis of node pa does not say where it creates its publisher of /data)" \
        in sub, sub


# --- a relay federate's import must be declared before its pack ---

RELAY = {"pa": "pa", "mid": "mid", "sub": "sub"}


def _relay_nodes(tmp_path: Path):
    """pa publishes /data; mid relays it to /out; sub takes /out."""
    src = tmp_path / "packages/src/test_pkg/src"
    src.mkdir(parents=True, exist_ok=True)
    nodes = []
    for n in [_pub("pa", "PubA"), _sub("mid", "MidNode", publishes="out"),
              _sub("sub", "SubNode", topic="out")]:
        f = src / f"{n.name}.cpp"
        f.write_text("// generated by the test\n")
        nodes.append(dataclasses.replace(n, source_file=str(f)))
    return nodes


def _relay(tmp_path: Path, after_delay: str = "5 msec") -> int:
    graph = _graph(_relay_nodes(tmp_path),
                   [_tc("/data", ["pa"], ["mid"]), _tc("/out", ["mid"], ["sub"])])
    adir = tmp_path / "lf-analysis/test_pkg/bringup"
    adir.mkdir(parents=True, exist_ok=True)
    sj = adir / "system.json"
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate=dict(RELAY), federate_names=["pa", "mid", "sub"],
                         coordination="centralized")
    return gen._generate_federated(system_json=sj, graph=graph, fmap=fmap,
                                   after_delay=after_delay, generate_launch=False,
                                   corr_bounds=UNDECLARED)


def test_relay_federate_imports_before_it_packs(tmp_path):
    # The k3assigned controller takes /fused/range and publishes
    # /brake_cmd over two tag links. The pack reaction of the output was
    # declared before the import reaction of the input, and LF orders the
    # reactions of one reactor by declaration: pack -> import -> inner node
    # -> output -> pack is a zero-delay cycle inside the federate reactor
    # that no cross-federate `after` can break ("Main reactor has causality
    # cycles").
    assert _relay(tmp_path) == 0
    mid = _lf(tmp_path, "mid")
    pack = 'ros2_lf_xtag_pack("/out", "out", mid.out->value, &rcllf_x);'
    imp = 'ros2_lf_xtag_import("/data", "pa.data -> mid.data", &data->value);'
    assert pack in mid and imp in mid, mid
    assert mid.index(imp) < mid.index(pack), (
        "the pack reaction of mid.out is declared before the import reaction of "
        "mid.data: declaration order plus the inner path data -> out closes a "
        "zero-delay cycle inside the federate reactor")


LFC = shutil.which("lfc") or ("/opt/lfc/bin/lfc" if Path("/opt/lfc/bin/lfc").exists() else None)


@pytest.mark.skipif(LFC is None, reason="lfc not installed")
def test_relay_federation_is_accepted_by_lfc(tmp_path):
    # The same shape through the real compiler: the federation must leave
    # lfc's causality check without a cycle.
    assert _relay(tmp_path) == 0
    lf_dir = next(tmp_path.rglob("lf/*_federated.lf")).parent
    out = tmp_path / "lfc-out"
    out.mkdir()
    top = next(lf_dir.glob("*_federated.lf"))
    res = subprocess.run([LFC, "--no-compile", "-o", str(out), top.name], cwd=lf_dir,
                         capture_output=True, text=True, timeout=300)
    log = res.stdout + res.stderr
    assert "causality cycles" not in log, log
    assert res.returncode == 0, log

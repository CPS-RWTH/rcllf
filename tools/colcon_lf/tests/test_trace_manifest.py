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
"""Static trace manifest and the executable's build info (T4)."""

from __future__ import annotations

import hashlib
import json
import re
from types import SimpleNamespace

from colcon_lf.generation.binding_header_emitter import collect_rid_table
from colcon_lf.generation.lf_generator import generate_lf_system
from .corr_declared import declared, generate_declared
from colcon_lf.generation.trace_manifest import (
    BUILD_INFO_CPP,
    STATIC_MANIFEST_NAME,
    build_static_manifest,
    find_provider_marker,
    generated_source_hash,
)
from colcon_lf.verb.generate import _emit_trace_manifest
from tests.generation.test_correspondence_emission import _graph, _loop_graph, _pub, _sub, _tc


def _pubsub_system():
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    return graph, generate_declared(graph, target_lfc=True)


def _manifest(system, graph, source_hash="s" * 64):
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    return build_static_manifest(
        executables=[system.name], corr_edges=system.corr_edges, rid_table=rid,
        model_hash="sha256:m", generated_source_hash=source_hash, provider_marker="rcllf 2.13.7",
    )


def test_pubsub_edge_is_admitted():
    graph, system = _pubsub_system()
    doc = _manifest(system, graph)
    assert doc["schema"] == "rcllf.manifest/1" and doc["kind"] == "static"
    (edge,) = doc["edges"]
    assert (edge["topic"], edge["status"], edge["id"]) == ("/data", "admitted", 0), edge
    pub = edge["publisher"]
    assert "rid" not in pub and "callback" not in pub, edge  # lists only
    assert pub["callbacks"] == ["tick"] and len(pub["rids"]) == 1 and pub["rids"][0] >= 0, edge
    assert edge["subscriber"]["rid"] >= 0, edge
    assert {r["callback"] for r in doc["rids"]} >= {"tick", "on_data"}, doc["rids"]


def test_refused_edge_carries_its_reason_and_a_loop_is_admitted():
    # Both edges of the a <-> b loop are admitted; a refused edge
    # (b's publisher burst undeclared) carries its reason in the manifest.
    graph = _loop_graph()
    system = generate_declared(graph, target_lfc=True)
    rows = {(e["publisher"]["node"], e["subscriber"]["node"]): e for e in _manifest(system, graph)["edges"]}
    assert rows[("b", "a")]["status"] == "admitted" and "reason" not in rows[("b", "a")], rows
    assert "reason" not in rows[("a", "b")]
    bounds = declared(graph, 1)
    del bounds.burst["b"]
    system = generate_declared(graph, target_lfc=True, corr_bounds=bounds)
    rows = {(e["publisher"]["node"], e["subscriber"]["node"]): e for e in _manifest(system, graph)["edges"]}
    assert rows[("b", "a")]["status"] == "legacy"
    assert rows[("b", "a")]["reason"] == "burst_unbounded", rows[("b", "a")]


def test_fan_in_group_rows_carry_lead_and_rank():
    # Pb is listed first, so it gets edge 0 and leads while pa
    # ranks first. The group of one into `one` keeps its row unchanged.
    graph = _graph([_pub("pa", "PubA"), _pub("pb", "PubB"), _sub("sub", "SubNode"),
                    _pub("pc", "PubC", "solo"), _sub("one", "OneSub", "solo")],
                   [_tc("/data", ["pb", "pa"], ["sub"]), _tc("/solo", ["pc"], ["one"])])
    system = generate_declared(graph, target_lfc=True)
    rows = {e["publisher"]["node"]: e for e in _manifest(system, graph)["edges"]}
    got = {p: (rows[p]["id"], rows[p].get("group_lead"), rows[p].get("fan_in_rank"))
           for p in ("pa", "pb")}
    assert got == {"pb": (0, 0, 1), "pa": (1, 0, 0)}, f"fan-in member rows (id, lead, rank): {got}"
    assert rows["pc"]["status"] == "admitted", rows["pc"]
    assert not {"group_lead", "fan_in_rank"} & set(rows["pc"]), (
        f"a group of one gained group fields: {rows['pc']}")


def test_hash_stable_across_generations_and_changed_by_an_edge_change():
    g1, s1 = _pubsub_system()
    g2, s2 = _pubsub_system()
    h = lambda d: hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()  # noqa: E731
    assert h(_manifest(s1, g1)) == h(_manifest(s2, g2))
    g3 = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                [_tc("/data", ["pub"], ["sub"], 5_000_000)])
    s3 = generate_declared(g3, target_lfc=True)
    assert h(_manifest(s3, g3)) != h(_manifest(s1, g1)), "a delay change must change the hash"


def test_emitted_symbol_hash_equals_file_hash(tmp_path):
    graph, system = _pubsub_system()
    stem_dir = tmp_path / "lf-analysis" / "pkg" / "stem"
    stem_dir.mkdir(parents=True)
    pkg_dir = tmp_path / "gen"
    (pkg_dir / "src").mkdir(parents=True)
    (pkg_dir / "src" / "ros2_lf_startup.cpp").write_text("int x;\n")
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    manifest_hash = _emit_trace_manifest(
        system_json=stem_dir / "system.json", pkg_dir=pkg_dir, lf_system=system, rid_table=rid)
    data = (stem_dir / STATIC_MANIFEST_NAME).read_bytes()
    assert hashlib.sha256(data).hexdigest() == manifest_hash
    cpp = (pkg_dir / "src" / BUILD_INFO_CPP).read_text()
    got = re.findall(r'"([0-9a-f]{64})"', cpp)
    assert got[0] == manifest_hash, cpp
    assert got[1] == json.loads(data)["generated_source_hash"] == generated_source_hash(pkg_dir)
    # the build-info file is excluded from the source hash it embeds
    assert generated_source_hash(pkg_dir) == json.loads(data)["generated_source_hash"]
    assert "rcllf_trace_build_info[3]" in cpp
    # the binding reads the reactor-c patch markers through these bounds
    assert "rcllf_reactor_c_patch_markers[2]" in cpp
    assert "__start_rcllf_reactor_c_patches" in cpp


def test_provider_marker_found_on_the_prefix_path(tmp_path):
    share = tmp_path / "share" / "ros2_lf_binding"
    share.mkdir(parents=True)
    (share / "trace_provider").write_text("rcllf 2.13.7\n")
    assert find_provider_marker([str(tmp_path / "none"), str(tmp_path)]) == "rcllf 2.13.7"
    assert find_provider_marker([str(tmp_path / "none")]) is None


def test_federated_manifest_and_one_build_info_per_federate(tmp_path):
    """Fed-trace-build-info: every federate sub-package carries the
    federation's static manifest hash, so federated runs are not manifest_mismatch."""
    from colcon_lf.federation.partition import FederationMap
    from colcon_lf.verb.generate import _generate_federated
    from tests.generation.test_federation import _make_graph

    analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
    analysis_dir.mkdir(parents=True)
    system_json = analysis_dir / "system.json"
    system_json.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    graph = _make_graph(["talker", "listener"], src_dir=tmp_path / "packages" / "src" / "test_pkg" / "src")
    fmap = FederationMap(node_to_federate={"talker": "talker", "listener": "listener"},
                         federate_names=["talker", "listener"])
    assert _generate_federated(system_json=system_json, graph=graph, fmap=fmap,
                               after_delay="1 msec", generate_launch=False) == 0
    assert not (analysis_dir / STATIC_MANIFEST_NAME).exists(), "must not clobber the lfc manifest"
    data = (analysis_dir / "rcllf_trace_manifest_federated.json").read_bytes()
    doc = json.loads(data)
    assert doc["federated"] is True
    assert doc["executables"] == ["federate__listener", "federate__talker"], doc["executables"]
    manifest_hash = hashlib.sha256(data).hexdigest()
    infos = sorted((tmp_path / "lf-generated").rglob(BUILD_INFO_CPP))
    assert len(infos) == 2, infos
    for cpp in infos:
        assert re.findall(r'"([0-9a-f]{64})"', cpp.read_text())[0] == manifest_hash, cpp
    dispatch = next((tmp_path / "lf-generated").rglob("ros2_lf_bindings.cmake")).read_text()
    assert dispatch.count("src/rcllf_trace_build_info.cpp") == 2, dispatch

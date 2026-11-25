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
"""Two publisher nodes on one topic into one stub input.

Before the fix both links went into the subscriber's one input port, and lfc
rejected the program with "Port named 'data' may only appear once on the right
side of a connection". Each publisher node now links into its own input port
(``data``, ``data_fanin_1``), and the subscriber node reactor sums their write
counts into one event, so the subscription runs once per write.
"""

from __future__ import annotations

from pathlib import Path
import re
import shutil
import subprocess

import pytest

from colcon_lf.federation.partition import AfterOverride
from colcon_lf.generation import lf_generator
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import PORT_MERGE_KIND, generate_lf_system

from ..corr_declared import generate_declared
from .test_correspondence_emission import _graph, _pub, _sub, _tc
from .test_shared_publisher_port import _lfc
from .test_stub_write_count import _exec, _run, _scope, _sub_reaction
from .test_xfed_correspondence import FED, _federate

LFC_ERROR = "may only appear once on the right side of a connection"


def _nodes(pb_durability=None):
    return [_pub("pa", "PubA"), _pub("pb", "PubB", durability=pb_durability),
            _sub("sub", "SubNode")]


# _federate declares every burst, so a fan-in group is admitted as
# a unit and links no stub. A transient_local member keeps the group legacy,
# which pins the stub fan-in path these federated tests are about.
LEGACY = "transient_local"


def _system():
    graph = _graph(_nodes(), [_tc("/data", ["pa", "pb"], ["sub"])])
    return generate_lf_system(graph, after_delay_ns=1_000_000)


def _accepted(proc: subprocess.CompletedProcess, text: str) -> None:
    out = proc.stdout + proc.stderr
    assert LFC_ERROR not in out, f"lfc: {LFC_ERROR}\n{text}"
    errors = [ln for ln in out.splitlines() if "rror" in ln]
    assert proc.returncode == 0, "lfc rejected the program:\n" + "\n".join(errors) + "\n" + text


def _merge(system):
    (outer,) = [r for r in system.reactors if r.name == "SubNode"]
    merges = [r for r in outer.reactions if r.kind == PORT_MERGE_KIND]
    assert len(merges) == 1, f"SubNode: expected one merge reaction, got {outer.reactions}"
    return outer, merges[0]


def test_lfc_accepts_two_publisher_nodes_into_one_input(tmp_path):
    text = emit_lf_system(_system())
    _accepted(_lfc(tmp_path, text), text)


def test_each_publisher_node_links_its_own_input():
    text = emit_lf_system(_system())
    assert "pa.data -> sub.data after 1 msec" in text, text
    assert "pb.data -> sub.data_fanin_1 after 1 msec" in text, text
    outer, merge = _merge(_system())
    inputs = sorted(p.name for p in outer.ports if p.direction == "input")
    assert inputs == ["data", "data_fanin_1"], outer.ports
    assert merge.triggers == ["data", "data_fanin_1"], merge
    assert merge.effects == ["default_group.data"], merge
    assert not [c for c in outer.connections if c.source == "data"], outer.connections


def test_merge_sums_both_publishers_and_the_subscription_runs_per_write(tmp_path):
    # pa wrote twice at this tag (value 2), pb three times. The group input
    # carries one event of 5, and the subscription takes 5 messages.
    system = _system()
    _, merge = _merge(system)
    body = (
        "  port_t pa = {1, 2}, pb = {1, 3}, grp = {0, 0};\n"
        "  struct { port_t* data; } default_group = {&grp};\n"
        + _scope({"data": "pa", "data_fanin_1": "pb"}, [merge.body])
        + _scope({"data": "grp"}, [_sub_reaction(system).body])
        + '  printf("present=%d value=%d ", grp.is_present, grp.value);\n'
    )
    assert _run(tmp_path, body) == "present=1 value=5 timers=0 subs=5 servers=0", merge.body


def test_merge_with_one_publisher_present_forwards_its_count(tmp_path):
    _, merge = _merge(_system())
    body = (
        "  port_t pa = {0, 0}, pb = {1, 1}, grp = {0, 0};\n"
        "  struct { port_t* data; } default_group = {&grp};\n"
        + _scope({"data": "pa", "data_fanin_1": "pb"}, [merge.body])
        + '  printf("present=%d value=%d ", grp.is_present, grp.value);\n'
    )
    assert _run(tmp_path, body) == "present=1 value=1 timers=0 subs=0 servers=0", merge.body


def test_merge_sum_overflow_exits_instead_of_wrapping(tmp_path):
    _, merge = _merge(_system())
    body = (
        "  port_t pa = {1, 2147483647}, pb = {1, 1}, grp = {0, 0};\n"
        "  struct { port_t* data; } default_group = {&grp};\n"
        + _scope({"data": "pa", "data_fanin_1": "pb"}, [merge.body])
    )
    proc = _exec(tmp_path, body)
    assert proc.returncode == 3, proc
    assert "write count of port 'data' overflows int" in proc.stderr, proc


def test_three_publisher_nodes_get_numbered_inputs():
    nodes = _nodes() + [_pub("pc", "PubC")]
    graph = _graph(nodes, [_tc("/data", ["pa", "pb", "pc"], ["sub"])])
    text = emit_lf_system(generate_lf_system(graph, after_delay_ns=0))
    for src, dst in (("pa", "data"), ("pb", "data_fanin_1"), ("pc", "data_fanin_2")):
        assert f"{src}.data -> sub.{dst} after 0" in text, text


def test_fan_in_input_named_like_a_declared_port_is_refused():
    sub = _sub("sub", "SubNode")
    sub.subscribers.append(type(sub.subscribers[0])(
        topic="data_fanin_1", message_type=sub.subscribers[0].message_type, callback="on_data"))
    graph = _graph([_pub("pa", "PubA"), _pub("pb", "PubB"), sub],
                   [_tc("/data", ["pa", "pb"], ["sub"])])
    with pytest.raises(ValueError, match="fan-in input 'data_fanin_1' of 'data'"):
        generate_lf_system(graph, after_delay_ns=0)


def _lfc_file(path: Path) -> subprocess.CompletedProcess:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    # *path* has no src/ parent, so lfc writes bin/ into the working directory
    # unless given -o, and fed-gen/ into it even then.
    return subprocess.run([lfc, "--no-compile", "-o", str(path.parent), str(path)],
                          cwd=path.parent, capture_output=True, text=True, timeout=600)


def test_lfc_file_writes_nothing_into_the_working_directory(tmp_path, monkeypatch):
    lf = tmp_path / "lf" / "Tiny.lf"
    lf.parent.mkdir()
    lf.write_text("target C\n\nreactor A {\n}\n\nfederated reactor Tiny {\n  a = new A()\n}\n")
    cwd = tmp_path / "cwd"
    cwd.mkdir()
    monkeypatch.chdir(cwd)
    proc = _lfc_file(lf)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    left = sorted(p.name for p in cwd.iterdir())
    assert left == [], f"lfc wrote {left} into the working directory {cwd}"


def _refused(tmp_path, capsys, nodes, topics, node_fed, dst="sub.data", **kw) -> None:
    try:
        _federate(tmp_path, nodes, topics, node_fed, **kw)
        rc = 0
    except AssertionError:  # _federate asserts exit 0
        rc = 1
    err = capsys.readouterr().err
    assert rc == 1 and f"legacy fan-in into {dst} on topic /data" in err, (
        f"split fan-in into {dst} was not refused at generation (exit {rc}):\n{err}")


FEDERATIONS = {
    # Both publishers in one federate leave on their own boundary outputs,
    # not one shared 'data' output.
    "publishers_together": ({"pa": "pubs", "pb": "pubs", "sub": "sub"},
                            ["pubs.data -> sub.data after",
                             "pubs.pb_data -> sub.data_fanin_1 after"]),
    # Writers in two federates can land on different tags, so the merged port
    # could hand one writer's sample to the other's tag (probe P17h).
    "per_node": ({"pa": "pa", "pb": "pb", "sub": "sub"}, None),
    "one_local": ({"pa": "pa", "pb": "sub", "sub": "sub"}, None),
}


@pytest.mark.parametrize("case", sorted(FEDERATIONS))
def test_federated_fan_in(tmp_path, capsys, case):
    node_fed, links = FEDERATIONS[case]
    topics = [_tc("/data", ["pa", "pb"], ["sub"])]
    if links is None:
        _refused(tmp_path, capsys, _nodes(LEGACY), topics, node_fed)
        return
    out = _federate(tmp_path, _nodes(LEGACY), topics, node_fed)
    sub_lf = out["sub"]["lf"]
    assert "input data_fanin_1: stub_t" in sub_lf, sub_lf
    assert "reaction(data, data_fanin_1)" in sub_lf, sub_lf
    for link in links:
        assert link in out["top"], f"{case}: {link!r} missing:\n{out['top']}"
    pubs = out["pubs"]["lf"]
    # Each publisher leaves on its own tag link, packed per port.
    assert 'ros2_lf_xtag_pack("/data", "data", pa.data->value, &rcllf_x);' in pubs, pubs
    assert 'ros2_lf_xtag_pack("/data", "pb_data", pb.data->value, &rcllf_x);' in pubs, pubs
    proc = _lfc_file(tmp_path / "lf-generated" / FED / "lf" / "Bringup_federated.lf")
    _accepted(proc, out["top"] + "\n" + sub_lf)


# Two subscriber instances of one class, each fed by several
# publisher nodes. Each instance has its own reactor definition (binding key
# per instance), so each merges its own fan-in count.
SAME_CLASS = {
    "two_and_two": ({"s1": ["pa", "pb"], "s2": ["pa", "pb"]}, {"s1": 2, "s2": 2}),
    "two_and_three": ({"s1": ["pa", "pb"], "s2": ["pa", "pb", "pc"]}, {"s1": 2, "s2": 3}),
}


def _same_class(case, pb_durability=None):
    feeds, _ = SAME_CLASS[case]
    nodes = [_pub("pa", "PubA"), _pub("pb", "PubB", durability=pb_durability),
             _pub("pc", "PubC"), _sub("s1", "SubNode"), _sub("s2", "SubNode")]
    return nodes, [_tc("/data", pubs, [s]) for s, pubs in feeds.items()]


def _fan_in_inputs(lf: str, reactor: str) -> list[str]:
    block = lf[lf.index(f"reactor {reactor} "):]
    block = block[:block.index("\n}\n")]
    return sorted(ln.split()[1].rstrip(":") for ln in block.splitlines()
                  if ln.strip().startswith("input data"))


@pytest.mark.parametrize("case", sorted(SAME_CLASS))
def test_two_subscriber_instances_of_one_class(tmp_path, case):
    nodes, topics = _same_class(case)
    _, counts = SAME_CLASS[case]
    text = emit_lf_system(generate_lf_system(_graph(nodes, topics), after_delay_ns=0))
    for s, n in counts.items():
        want = ["data"] + [f"data_fanin_{k}" for k in range(1, n)]
        assert _fan_in_inputs(text, f"SubNode__{s}") == sorted(want), text
        for k, src in enumerate(sorted(SAME_CLASS[case][0][s])):
            port = "data" if k == 0 else f"data_fanin_{k}"
            assert f"{src}.data -> {s}.{port} after 0" in text, text
    _accepted(_lfc(tmp_path, text), text)


# The publishers share one federate so each instance's fan-in is accepted;
# the subscriber instances sit apart or together.
SAME_CLASS_FEDS = {
    "apart": {"pa": "pubs", "pb": "pubs", "pc": "pubs", "s1": "s1", "s2": "s2"},
    "together": {"pa": "pubs", "pb": "pubs", "pc": "pubs", "s1": "subs", "s2": "subs"},
}


@pytest.mark.parametrize("fed", sorted(SAME_CLASS_FEDS))
@pytest.mark.parametrize("case", sorted(SAME_CLASS))
def test_two_subscriber_instances_of_one_class_federated(tmp_path, case, fed):
    nodes, topics = _same_class(case, LEGACY)
    out = _federate(tmp_path, nodes, topics, SAME_CLASS_FEDS[fed])
    _, counts = SAME_CLASS[case]
    for s, n in counts.items():
        lf = out[SAME_CLASS_FEDS[fed][s]]["lf"]
        want = ["data"] + [f"data_fanin_{k}" for k in range(1, n)]
        assert _fan_in_inputs(lf, f"SubNode__{s}") == sorted(want), lf
    proc = _lfc_file(tmp_path / "lf-generated" / FED / "lf" / "Bringup_federated.lf")
    _accepted(proc, out["top"])


@pytest.mark.parametrize("case", sorted(SAME_CLASS))
def test_two_subscriber_instances_of_one_class_publishers_apart_refused(tmp_path, capsys, case):
    nodes, topics = _same_class(case, LEGACY)
    node_fed = {"pa": "pa", "pb": "pb", "pc": "pc", "s1": "s1", "s2": "s2"}
    _refused(tmp_path, capsys, nodes, topics, node_fed, dst="s1.data")


# ---------------------------------------------------------------------------
# An admitted fan-in group across federates
# ---------------------------------------------------------------------------

def _spy_plan(monkeypatch) -> dict:
    """Record the system table that ``_generate_federated`` admits."""
    seen: dict = {}
    real = lf_generator.plan_federated_corr

    def spy(*args, **kwargs):
        seen["system"], seen["per_fed"] = real(*args, **kwargs)
        return seen["system"], seen["per_fed"]

    monkeypatch.setattr(lf_generator, "plan_federated_corr", spy)
    return seen


def _fan_in_calls(init: str) -> dict[int, tuple[int, int]]:
    """``set_corr_edge_fan_in`` calls of a binding init: edge id -> (lead, rank)."""
    calls = re.findall(r"set_corr_edge_fan_in\((\d+), (\d+), (\d+)u\);", init)
    return {int(e): (int(lead), int(rank)) for e, lead, rank in calls}


# The publishers are listed pb first, so edge order and rank order differ, and
# an edge /other x -> y between two more federates takes global id 0, so the
# group's global ids (1, 2) are not the subscriber federate's ids (0, 1).
GG_FEDERATIONS = {
    "per_node": {"pa": "pa", "pb": "pb", "sub": "sub", "x": "x", "y": "y"},
    "publishers_together": {"pa": "pubs", "pb": "pubs", "sub": "sub", "x": "x", "y": "y"},
}


@pytest.mark.parametrize("case", sorted(GG_FEDERATIONS))
def test_per_node_fan_in_group_generates(tmp_path, case):
    # A stub fan-in would fail the per-node split (P17h): its
    # writers sit in two federates. The admitted group links no stub.
    nodes = _nodes() + [_pub("x", "PubX", "other"), _sub("y", "SubY", "other")]
    topics = [_tc("/other", ["x"], ["y"]), _tc("/data", ["pb", "pa"], ["sub"])]
    node_fed = GG_FEDERATIONS[case]
    out = _federate(tmp_path, nodes, topics, node_fed)
    rows = [r for r in out["sub"]["rows"] if r["topic"] == "/data"]
    assert [(r["role"], r["status"]) for r in rows] == [("ingress", "admitted")] * 2, rows
    calls = _fan_in_calls(out["sub"]["init"])
    ids = sorted(r["id"] for r in rows)
    assert sorted(calls) == ids and {lead for lead, _ in calls.values()} == {ids[0]}, (
        f"the subscriber federate's group must name its lead by the federate's own "
        f"edge id {ids[0]}: (edge, lead, rank) {calls}")
    one = generate_declared(_graph(nodes, topics), after_delay_ns=5_000_000)
    want = {e.pub_key: e.fan_in_rank for e in one.corr_edges if e.topic == "/data"}
    got = {r["pub"]: calls[r["id"]][1] for r in rows}
    assert got == want, f"federated ranks {got} differ from the one-process ranks {want}"
    for fed in sorted(set(node_fed.values()) - {"sub"}):
        assert not _fan_in_calls(out[fed]["init"]), (
            f"federate {fed} registers a fan-in group on its egress rows: "
            f"{_fan_in_calls(out[fed]['init'])}")


def test_refused_group_keeps_p17h(tmp_path, capsys, monkeypatch):
    # A group that stays legacy (a transient_local member) links its stub
    # fan-in across federates, and P17h still refuses it.
    seen = _spy_plan(monkeypatch)
    _refused(tmp_path, capsys, _nodes(LEGACY), [_tc("/data", ["pa", "pb"], ["sub"])],
             {"pa": "pa", "pb": "pb", "sub": "sub"})
    reasons = {e.pub_node: e.reason for e in seen["system"] if e.topic == "/data"}
    assert reasons["pa"] == "fan_in_member_refused", reasons
    assert reasons["pb"] not in (None, "fan_in_member_refused"), reasons


def test_mixed_locality_group_refused(tmp_path, capsys, monkeypatch):
    # pb sits in the subscriber's federate, pa in its own, both 5 ms (the
    # cross default is 0, so pa's link states it). A local
    # record arrives at its publication tag and a remote one only at its
    # release, so the group's one wake could dispatch a remote record late
    # (case P5). The group stays legacy, and its stub fan-in
    # spans two federates (P17h). One process admits the same system.
    seen = _spy_plan(monkeypatch)
    topics = [_tc("/data", ["pa", "pb"], ["sub"])]
    _refused(tmp_path, capsys, _nodes(), topics, {"pa": "pa", "pb": "sub", "sub": "sub"},
             after_delay="5 msec", overrides=[AfterOverride("pa.data", "sub.data", 5_000_000)])
    edges = [e for e in seen["system"] if e.topic == "/data"]
    assert {e.delay_ns for e in edges} == {5_000_000}, [(e.label(), e.delay_ns) for e in edges]
    assert [e.reason for e in edges] == ["fan_in_mixed_locality"] * 2, (
        "a group with a local and a remote member was not refused as mixed locality: "
        f"{[(e.label(), e.status, e.reason) for e in edges]}")
    one = generate_declared(_graph(_nodes(), topics), after_delay_ns=5_000_000)
    assert [e.status for e in one.corr_edges] == ["admitted"] * 2, one.corr_edges

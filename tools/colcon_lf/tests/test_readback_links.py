# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Link identities the generated program registers for the read-back.

The read-back judges an after-delay per contract: the benchmark harness looks up each
``link:<src>-><dst>/<name>#after_delay`` record by ``(kind, src, dst, name)``
among the carriers the program registers. These tests generate from the
system.json files lf-generate wrote in the dev container (the benchmark harness's
fixtures of the same runs) and check that every resolved-model link has a
carrier under the same identity, and nothing else does.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.generation.lf_model import LFReactor, LFSystem
from colcon_lf.generation.readback_links import class_rows, emit_cpp, link_rows, port_rows
from colcon_lf.output.serializer import deserialize_system_graph

from .dev_only import requires_lf_bench

needs_lf_bench = requires_lf_bench("the read-back fixtures, tests/fixtures/readback3*")

FIXTURES = (Path(__file__).resolve().parents[2] / "lf_bench" / "tests" / "fixtures"
            / "readback362")


def _system(name: str, after_delay_ns: int | None = None) -> LFSystem:
    graph = deserialize_system_graph((FIXTURES / name / "system.json").read_text())
    return generate_lf_system(graph, after_delay_ns=after_delay_ns)


def _model_links(name: str) -> set[tuple]:
    """``(kind, src, dst, name)`` of each directed link of the resolved model."""
    out = set()
    for rec in json.loads((FIXTURES / name / "resolved_model.json").read_text())["records"]:
        s = rec.get("subject") or {}
        if s.get("kind") != "link" or rec.get("property") != "after_delay":
            continue
        if s["link"] == "service":
            out |= {("service request", s["src"], s["dst"], s["name"]),
                    ("service response", s["dst"], s["src"], s["name"])}
        else:
            out.add((s["link"], s["src"], s["dst"], s["name"]))
    return out


@needs_lf_bench
@pytest.mark.parametrize("name", ["aeb_mujoco", "service_example"])
def test_every_configured_link_has_one_carrier_under_its_identity(name):
    rows = link_rows(_system(name))
    keys = [(r["kind"], r["src"], r["dst"], r["name"]) for r in rows]
    assert sorted(keys) == sorted(_model_links(name))


@needs_lf_bench
def test_lf_carriers_name_the_destination_port():
    rows = [r for r in link_rows(_system("aeb_mujoco")) if r["src"] == "aeb_sim"]
    assert sorted((r["carrier"], r["src_instance"], r["dst_instance"], r["dst_port"])
                  for r in rows) == [
        ("lf", "aeb_sim", "aeb_controller", "aeb_odom"),
        ("lf", "aeb_sim", "aeb_controller", "aeb_range"),
    ]


@needs_lf_bench
def test_ports_are_the_reaction_triggering_inputs():
    ports = port_rows(_system("aeb_mujoco"))
    assert ("AebControllerNode_DefaultGroup", "aeb_odom") in ports
    assert ("AebControllerNode_DefaultGroup", "aeb_range") in ports


@needs_lf_bench
@pytest.mark.parametrize("name", ["aeb_mujoco", "service_example"])
def test_declared_classes_are_what_the_running_program_had(name):
    """The per-class reaction counts the program declares equal the counts the
    binding's scan found in the real run (one instance per class there); lfc's
    own delay reactors are not declared. The program is generated with the
    after-delay the recorded run had (aeb_mujoco: ``--after-delay "1 msec"``,
    read from its admitted edge), since a d = 0 edge has one reaction fewer.
    """
    doc = json.loads((FIXTURES / name / "readback_valid.json").read_text())
    found = doc["lf_reaction_graph"]["classes"]
    delays = {e["delay_ns"] for e in doc["corr_edges"]}
    assert len(delays) <= 1, delays
    assert class_rows(_system(name, next(iter(delays), None))) == sorted(
        (c, n, 1) for c, n in found.items() if not c.startswith("__lf_"))


@needs_lf_bench
def test_zero_delay_in_process_edge_has_no_separate_arrive_reaction():
    """At d = 0 the admitted in-process /aeb/brake_cmd edge runs its
    callback at the publication tag, so the arrive reaction is merged into the
    drain and the simulator's group declares one reaction fewer than the 1 ms
    recording."""
    zero = dict((c, n) for c, n, _ in class_rows(_system("aeb_mujoco", 0)))
    one_ms = dict((c, n) for c, n, _ in class_rows(_system("aeb_mujoco", 1_000_000)))
    assert zero["_aebmujocosimnode_defaultgroup"] == one_ms["_aebmujocosimnode_defaultgroup"] - 1
    assert {c: n for c, n in zero.items() if c != "_aebmujocosimnode_defaultgroup"} == {
        c: n for c, n in one_ms.items() if c != "_aebmujocosimnode_defaultgroup"}


@needs_lf_bench
def test_emitted_cpp_includes_schedule_before_class_headers():
    """The class header carries the preamble, whose wake thunks call lf_schedule."""
    includes, body = emit_cpp(_system("aeb_mujoco"))
    assert includes.index('#include "include/api/schedule.h"') < includes.index(
        '#include "_aebcontrollernode_defaultgroup.h"')
    assert ('  ros2_lf_binding::register_lf_port("_aebcontrollernode_defaultgroup", '
            'offsetof(_aebcontrollernode_defaultgroup_self_t, _lf__aeb_odom), "aeb_odom");') in body
    assert ('  ros2_lf_binding::register_lf_link({"topic", "aeb_sim", "aeb_controller", '
            '"/aeb/odom", "lf", "aeb_sim", "aeb_controller", "aeb_odom", -1});') in body


def test_reactor_class_named_like_lfc_generated_class_is_refused():
    """The read-back recognises lfc's delay reactors by the class prefix
    ``_lf`` (C name ``__lf_gendelay_<hash>``); no generated class may take it."""
    system = LFSystem(name="X", reactors=[LFReactor(name="_lf_GenDelay_user")],
                      main_instances=[])
    with pytest.raises(ValueError, match="names starting with '_lf' are lfc's"):
        emit_lf_system(system)

# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The federated resolved model records the after-delay a link across
federates runs.

Generated from the system.json and resolved_model.json lf-generate wrote for
the RQ1ext loss system in the dev container (the benchmark harness's ``loss_fed``
fixture). Its edge ``/loss/counter`` is admitted; under ``--federate
per-node`` with ``--after-delay 1 msec`` the cross link keeps the 1 ms it has
in one process (an earlier finding: it used to fall to
``DEFAULT_CROSS_FEDERATE_AFTER_NS`` = 0, so the split ran another program
than the one process). Without a configured delay the default is ``after 0``
(the records leave at the publishing tag). An override of d > 0 is
sent at the publishing tag too; only a flush edge (decentralized, or
``RCLLF_XFED_FLUSH=1``) runs ``after d - 1 ns``.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from colcon_lf.federation.partition import (
    DEFAULT_CROSS_FEDERATE_AFTER_NS, AfterOverride, resolve_partition,
)
from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.resolved_model.model import from_json
from colcon_lf.resolved_model.resolve import federate_link_records
from colcon_lf.verb.generate import (
    FEDERATED_RESOLVED_MODEL, _federated_link_facts, _generate_federated,
)

from ..dev_only import requires_lf_bench

needs_lf_bench = requires_lf_bench("the loss_fed fixture, tests/fixtures/readback362")

REPO = Path(__file__).resolve().parents[4]
FIXTURE = REPO / "tools" / "lf_bench" / "tests" / "fixtures" / "readback362" / "loss_fed"
LINK = "link:loss_publisher->loss_subscriber/loss/counter#after_delay"
NETWORK = "link:loss_publisher->loss_subscriber/loss/counter[topic]#after_delay.network"


def _generate(tmp_path: Path, overrides=(), after_delay="1 msec") -> tuple[dict, dict, str]:
    analysis = tmp_path / "lf-analysis" / "timing_gaps" / "loss"
    analysis.mkdir(parents=True)
    shutil.copy(FIXTURE / "resolved_model.json", analysis / "resolved_model.json")
    system_json = analysis / "system.json"
    # Recorded in the dev container: its source paths are /workspace paths.
    system_json.write_text((FIXTURE / "system.json").read_text().replace(
        '"/workspace/', f'"{REPO}/'))
    graph = deserialize_system_graph(system_json.read_text())
    fmap = resolve_partition("per-node", graph)
    fmap.after_overrides = list(overrides)
    assert _generate_federated(system_json=system_json, graph=graph, fmap=fmap,
                               after_delay=after_delay, generate_launch=False) == 0
    records = {r.contract_id: r for r in
               from_json((analysis / FEDERATED_RESOLVED_MODEL).read_text()).records}
    base = {r.contract_id: r for r in
            from_json((analysis / "resolved_model.json").read_text()).records}
    top = next((tmp_path / "lf-generated").rglob("Loss_federated.lf")).read_text()
    return records, base, top


@needs_lf_bench
def test_a_configured_in_process_delay_reaches_the_cross_link(tmp_path):
    """Earlier finding: ``--after-delay`` reached the one-process links only, the
    cross link ran ``after 0``. Falsified on 91e3ae71: ``assert 0 == 1000000``."""
    records, base, top = _generate(tmp_path)
    assert base[LINK].value == 1_000_000, "the unfederated model is left alone"
    assert records[LINK].value == 1_000_000
    assert records[LINK].provenance == "explicitly_configured"
    assert "in-process delay" in records[LINK].evidence
    assert records[NETWORK].value == 1_000_000
    assert "loss_publisher.xcorr_g0 -> loss_subscriber.xcorr_g0 after 1 msec\n" in top, top


@needs_lf_bench
def test_model_and_program_agree_on_the_default_cross_federate_delay(tmp_path):
    records, _base, top = _generate(tmp_path, after_delay=None)
    assert records[LINK].value == DEFAULT_CROSS_FEDERATE_AFTER_NS == 0
    assert records[LINK].provenance == "extracted"
    assert "DEFAULT_CROSS_FEDERATE_AFTER_NS" in records[LINK].evidence
    assert records[NETWORK].value == 0
    # A centralized d = 0 record link is a plain zero-delay
    # connection, so the records arrive at their publication tag (t, m).
    assert "loss_publisher.xcorr_g0 -> loss_subscriber.xcorr_g0\n" in top, top
    assert "xcorr_g0 after" not in top, top


@needs_lf_bench
def test_an_after_override_is_explicitly_configured(tmp_path):
    ov = AfterOverride(from_port="loss_publisher.loss_counter",
                       to_port="loss_subscriber.loss_counter", after_ns=3_000_000)
    records, _base, top = _generate(tmp_path, [ov])
    assert "after 3 msec" in top, top
    assert records[LINK].value == 3_000_000
    assert records[LINK].provenance == "explicitly_configured"
    assert records[LINK].evidence.endswith(
        "after_overrides loss_publisher.loss_counter -> loss_subscriber.loss_counter")
    assert records[NETWORK].value == 3_000_000
    assert records[NETWORK].subject["carrier"] == "correspondence"


PUB, SUB = "federate__loss_publisher", "federate__loss_subscriber"


def _facts(overrides, legacy=False):
    """``_federated_link_facts`` of the loss link with *overrides*: the
    admitted edge the per-node loss federation runs, or a legacy link."""
    fmap = SimpleNamespace(after_overrides=overrides, source="fed.yaml",
                           coordination="centralized")
    ports = ("loss_publisher.loss_counter", "loss_subscriber.loss_counter")
    link = ("topic", "loss_publisher", "loss_subscriber", "/loss/counter")
    if legacy:
        conn = SimpleNamespace(source=ports[0], destination=ports[1], link=link,
                               federate_after_ns=5_000_000)
        return _federated_link_facts([conn], {}, {}, {"loss_publisher": PUB,
                                                      "loss_subscriber": SUB}, {}, fmap)
    conn = SimpleNamespace(source="loss_publisher.xcorr_g0",
                           destination="loss_subscriber.xcorr_g0", link=None)
    edge = SimpleNamespace(edge_id=0, pub_node=link[1], sub_node=link[2], topic=link[3],
                           delay_ns=5_000_000, link_ports=ports)
    return _federated_link_facts([conn], {"xcorr_g0": 0}, {0: (edge, {PUB: 0, SUB: 0})},
                                 {"loss_publisher": PUB, "loss_subscriber": SUB}, {}, fmap)


@pytest.mark.parametrize("legacy", [False, True])
def test_falsification_override_on_other_ports_is_not_cited(legacy):
    """Review item 4: an override between the same instances, on other ports,
    with the delay the link happens to run, used to be cited as its source."""
    other = AfterOverride(from_port="loss_publisher.other", to_port="loss_subscriber.other",
                          after_ns=5_000_000)
    (row,), _net = _facts([other], legacy)
    # Not the other ports' override: the link's own (in-process) delay.
    assert row["provenance"] == "explicitly_configured"
    assert "in-process delay" in row["evidence"], row["evidence"]
    assert row["admitted"] is not legacy
    own = AfterOverride(from_port="loss_publisher.loss_counter",
                        to_port="loss_subscriber.loss_counter", after_ns=5_000_000)
    (row,), _net = _facts([other, own], legacy)
    assert (row["provenance"], row["evidence"]) == (
        "explicitly_configured",
        "fed.yaml: after_overrides loss_publisher.loss_counter -> loss_subscriber.loss_counter")


@needs_lf_bench
def test_channels_with_different_delays_leave_the_record_unresolved():
    base = from_json((FIXTURE / "resolved_model.json").read_text()).records
    rows = [{"kind": "topic", "src": "loss_publisher", "dst": "loss_subscriber",
             "name": "/loss/counter", "after_ns": ns, "network_ns": ns,
             "provenance": "extracted", "evidence": "e", "network_evidence": "e"}
            for ns in (2, 3)]
    with pytest.raises(ValueError):
        federate_link_records(base, [dict(rows[0], name="/not/in/the/model")])
    by_id = {r.contract_id: r for r in federate_link_records(base, rows)}
    assert by_id[LINK].provenance == "unresolved"
    assert by_id[LINK].value is None


@needs_lf_bench
def test_each_federate_registers_its_end_of_the_link(tmp_path):
    _generate(tmp_path)
    cpp = {p.parent.parent.name: p.read_text()
           for p in (tmp_path / "lf-generated").rglob("ros2_lf_reaction_ids_init.cpp")}
    pub = next(t for k, t in cpp.items() if "loss_publisher" in k)
    sub = next(t for k, t in cpp.items() if "loss_subscriber" in k)
    assert '"network-out"' in pub and '"network"' not in pub
    assert ('register_lf_link({"topic", "loss_publisher", "loss_subscriber", "/loss/counter", '
            '"network", "loss_publisher", "loss_subscriber", "xcorr_e0", 0});') in sub
    assert json.dumps("network-out") not in sub

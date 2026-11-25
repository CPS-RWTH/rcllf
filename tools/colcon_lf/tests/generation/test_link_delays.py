# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-connection after-delays with a stated reason.

``execution.yaml``'s ``connections:`` gives a link its own delay, by name or by
one sender -> receiver pair, and says why. The generated ``.lf`` carries the
value (``after`` on a legacy link, ``delay_ns`` of an admitted edge) and a
comment with the reason; the resolved model's link record names the entry.

Falsification: drop ``link_delays=`` from the ``generate_lf_system`` call in
``_build_connections``' caller, and ``test_legacy_links_carry_configured_after``
fails with ``after 1 msec`` where ``after 3 msec`` is expected.
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import ServiceConnection
from colcon_lf.exec_config.connections import link_delays
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.node_analysis.models import (
    CallbackInfo, NodeInfo, ServiceClientInfo, ServiceServerInfo,
)
from colcon_lf.resolved_model.resolve import link_delay_rows

from ..corr_declared import generate_declared
from .test_correspondence_emission import _edge, _graph, _pub, _sub, _tc

MS = 1_000_000

CONNECTIONS = {
    "/data": {
        "after_delay": "3 msec",
        "reason": "measured",
        "source": "run r1: pub tick max 2.6 ms",
        "links": {
            "pub -> sub2": {"after_delay": "20 msec", "reason": "budgeted",
                            "source": "sub2 is a logger"},
        },
    },
}


def _fan_out(delay=None):
    return _graph([_pub("pub", "PubNode"), _sub("sub1", "Sub1Node"), _sub("sub2", "Sub2Node")],
                  [_tc("/data", ["pub"], ["sub1", "sub2"], delay)])


def _config(graph, connections=CONNECTIONS, **raw):
    return ExecConfig.from_dict({"connections": connections, **raw}, graph, source="t.yaml")


def _main_lines(text: str) -> list[str]:
    main = text[text.index("main reactor"):]
    return [ln.strip() for ln in main.splitlines()]


def _line_after(lines: list[str], fragment: str) -> tuple[str, str]:
    """The connection line holding *fragment* and the line above it."""
    (i,) = [i for i, ln in enumerate(lines) if fragment in ln and "->" in ln and not ln.startswith("//")]
    return lines[i - 1], lines[i]


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("connections,message", [
    ({"/nope": {"after_delay": "1 msec", "reason": "default"}}, "unknown link '/nope'"),
    ({"/data": {"after_delay": "1 msec", "reason": "guessed"}}, "'reason' must be one of"),
    ({"/data": {"after_delay": "1 msec", "reason": "measured"}}, "needs a 'source'"),
    ({"/data": {"after_delay": "soon", "reason": "default"}}, "cannot parse time literal 'soon'"),
    ({"/data": {"reason": "default"}}, "'after_delay' is required"),
    ({"/data": {"after_delay": "1 msec", "reason": "default", "why": "x"}}, "unknown key(s) why"),
    ({"/data": {"links": {"sub1 -> pub": {"after_delay": "1 msec", "reason": "default"}}}},
     "not a sender -> receiver pair"),
    ({"/data": {"links": {"pub sub1": {"after_delay": "1 msec", "reason": "default"}}}},
     "must be '<sender> -> <receiver>'"),
])
def test_invalid_entries_are_refused(connections, message):
    with pytest.raises(ValueError, match="t.yaml") as exc:
        _config(_fan_out(), connections)
    assert message in str(exc.value)


def test_names_are_slash_tolerant_and_default_needs_no_source():
    cfg = _config(_fan_out(), {"data": {"after_delay": "2 msec", "reason": "default"}})
    delay = link_delays(cfg).get("/data", "pub", "sub1")
    assert (delay.ns, delay.reason, delay.source) == (2 * MS, "default", "")


def test_pair_beats_name():
    delays = link_delays(_config(_fan_out()))
    assert delays.get("/data", "pub", "sub1").ns == 3 * MS
    assert delays.get("/data", "pub", "sub2").ns == 20 * MS
    assert delays.get("/data", "pub", "sub2").evidence == (
        "execution.yaml:connections./data.links.pub -> sub2")


# ---------------------------------------------------------------------------
# Generated program
# ---------------------------------------------------------------------------


def test_legacy_links_carry_configured_after():
    graph = _fan_out()
    # No declared burst bound: both edges stay legacy LF connections.
    system = generate_lf_system(graph, after_delay_ns=MS, link_delays=link_delays(_config(graph)))
    lines = _main_lines(emit_lf_system(system))
    comment1, conn1 = _line_after(lines, "sub1.")
    comment2, conn2 = _line_after(lines, "sub2.")
    assert conn1.endswith("after 3 msec"), conn1
    assert conn2.endswith("after 20 msec"), conn2
    assert comment1 == ("// after-delay 3 msec: measured (run r1: pub tick max 2.6 ms) "
                        "[execution.yaml:connections./data]")
    assert comment2 == ("// after-delay 20 msec: budgeted (sub2 is a logger) "
                        "[execution.yaml:connections./data.links.pub -> sub2]")


def test_unconfigured_links_keep_the_default_and_no_comment():
    graph = _fan_out()
    lines = _main_lines(emit_lf_system(generate_lf_system(graph, after_delay_ns=MS)))
    above, conn = _line_after(lines, "sub1.")
    assert conn.endswith("after 1 msec") and not above.startswith("//"), (above, conn)


def test_admitted_edges_take_the_configured_delay():
    graph = _fan_out()
    system = generate_declared(graph, after_delay_ns=MS, link_delays=link_delays(_config(graph)))
    assert _edge(system, "pub", "sub1").status == "admitted"
    assert _edge(system, "pub", "sub1").delay_ns == 3 * MS
    assert _edge(system, "pub", "sub2").delay_ns == 20 * MS
    lines = _main_lines(emit_lf_system(system))
    comment, conn = _line_after(lines, "sub2.corr_e")
    assert comment.startswith("// after-delay 20 msec: budgeted"), (comment, conn)


def test_connections_beat_system_json_after_delay_ns():
    graph = _fan_out(delay=7 * MS)
    plain = generate_declared(graph, after_delay_ns=MS)
    assert _edge(plain, "pub", "sub1").delay_ns == 7 * MS
    configured = generate_declared(graph, after_delay_ns=MS, link_delays=link_delays(_config(graph)))
    assert _edge(configured, "pub", "sub1").delay_ns == 3 * MS


def _service_graph():
    client = NodeInfo(
        name="cli", class_name="CliNode", package="test_pkg", source_file="/fake/cli.cpp",
        callbacks=[CallbackInfo(name="tick", service_clients_used=["svc"])],
        service_clients=[ServiceClientInfo(service_name="svc", service_type="x::srv::Y",
                                           call_style="async_callback")],
    )
    server = NodeInfo(
        name="srv", class_name="SrvNode", package="test_pkg", source_file="/fake/srv.cpp",
        callbacks=[CallbackInfo(name="handle")],
        service_servers=[ServiceServerInfo(service_name="svc", service_type="x::srv::Y",
                                           callback="handle")],
    )
    return _graph([client, server], [],
                  [ServiceConnection(service="/svc", service_type="x::srv::Y",
                                     server="srv", clients=["cli"])])


def test_service_link_carries_the_delay_both_ways():
    graph = _service_graph()
    cfg = _config(graph, {"/svc": {"links": {"cli -> srv": {
        "after_delay": "4 msec", "reason": "budgeted", "source": "server budget"}}}})
    system = generate_lf_system(graph, after_delay_ns=MS, link_delays=link_delays(cfg))
    service = [c for c in system.main_connections if c.link and c.link[0].startswith("service")]
    assert sorted(c.link[0] for c in service) == ["service request", "service response"]
    assert {c.after_delay_ns for c in service} == {4 * MS}


# ---------------------------------------------------------------------------
# Resolved model (read-back gate input)
# ---------------------------------------------------------------------------


def test_resolved_model_records_value_reason_and_provenance():
    graph = _fan_out()
    rows = {r.contract_id: r for r in link_delay_rows(graph, _config(graph))}
    r1, r2 = rows["link:pub->sub1/data#after_delay"], rows["link:pub->sub2/data#after_delay"]
    assert (r1.value, r1.provenance) == (3 * MS, "measured_profiled")
    assert r1.evidence == "execution.yaml:connections./data (measured: run r1: pub tick max 2.6 ms)"
    assert (r2.value, r2.provenance) == (20 * MS, "explicitly_configured")


def test_layering_user_entry_replaces_package_entry(tmp_path):
    graph = _fan_out()
    pkg, user = tmp_path / "pkg.yaml", tmp_path / "user.yaml"
    pkg.write_text("connections:\n  /data:\n    after_delay: 3 msec\n    reason: default\n"
                   "    links:\n      pub -> sub2:\n        after_delay: 9 msec\n"
                   "        reason: default\n")
    user.write_text("connections:\n  /data:\n    after_delay: 5 msec\n    reason: budgeted\n"
                    "    source: user budget\n")
    delays = link_delays(ExecConfig.load_layered(user, pkg, graph))
    assert delays.get("/data", "pub", "sub1").ns == 5 * MS
    assert delays.get("/data", "pub", "sub1").source == "user budget"
    assert delays.get("/data", "pub", "sub2").ns == 9 * MS

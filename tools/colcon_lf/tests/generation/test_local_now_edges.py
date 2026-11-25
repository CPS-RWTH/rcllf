# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Local edges decided at publish (plan slice S3, generator side).

A local edge (both ends in one process) is decided when its publish returns:
the publishing thread takes the sample from the subscriber's reader, so the
join never waits and nothing has a cutoff. The generator side of that:

- ``join: deferred`` on a two-node local edge is a generation error (a design
  decision), never accepted and emitted silently.
- A declared ``take_timeout_ms`` on a subscription with only local edges
  warns that it only sizes L and retention (question 5).
- Under ``require_admitted`` a local ``sub_rebind`` edge, which cannot
  be decided at publish, is refused (``local_sub_rebind``, question 3).
- The trace manifest marks a local edge ``local_now`` with ``cutoff: none``,
  the keys of its reconcile row, and leaves cross-federate rows alone.

Every expected value is derived from the issue's rule, not read back.
"""

from __future__ import annotations

import json
import warnings

import pytest

from colcon_lf.generation.binding_header_emitter import collect_rid_table, emit_rid_init_cpp
from colcon_lf.generation.trace_manifest import FEDERATED_MANIFEST_NAME, build_static_manifest

from ..corr_declared import declared
from .test_correspondence_emission import _edge, _graph, _lifecycle_pair, _pub, _sub, _tc
from .test_corr_bounds import _gen
from .test_xfed_correspondence import _pubsub

MS = 1_000_000
ISSUE = "only sizes L: its edge(s)"


def _pair():
    """pub (timer) -> /data -> sub, one process: one local edge."""
    return _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                  [_tc("/data", ["pub"], ["sub"])])


def _manifest(system, graph):
    rid = collect_rid_table({r.name: r for r in system.reactors}, graph)
    return build_static_manifest(
        executables=[system.name], corr_edges=system.corr_edges, rid_table=rid,
        model_hash="sha256:m", generated_source_hash="s" * 64, provider_marker="rcllf 2.13.7",
    )


def _ours(record):
    return [str(w.message) for w in record if ISSUE in str(w.message)]


# Question 6: join deferred on a local edge ----------------------------------


def test_join_deferred_on_a_two_node_local_edge_is_a_generation_error():
    graph = _pair()
    bounds = declared(graph, deferred_join={("sub", "data")})
    try:
        system = _gen(graph, bounds)
    except ValueError as exc:
        msg = str(exc)
    else:
        e = _edge(system, "pub", "sub")
        pytest.fail(f"'join: deferred' on local edge {e.label()} was accepted silently "
                    f"(status {e.status}, deferred={e.deferred}): a local edge is decided at "
                    f"publish, its join never waits")
    assert ("'join: deferred' on sub 'data' is not supported, local edge "
            "/data (pub -> sub) is decided at publish") in msg, msg
    assert "cross-federate edges only" in msg and "#698" not in msg, msg


def test_join_deferred_on_a_cross_federate_edge_stays_accepted(tmp_path):
    # Regression guard: the ingress row of a cross-federate edge keeps its
    # deferred join, only local edges are refused.
    out = _pubsub(tmp_path, deferred_join={("sub", "data")})
    assert "  ros2_lf_binding::set_corr_edge_deferred(0);" in out["sub"]["init"].splitlines(), (
        out["sub"]["init"])


# Question 5: T on a local-only subscription only sizes L --------------------


def test_take_timeout_on_a_local_only_subscription_warns_it_only_sizes_l():
    graph = _pair()
    with warnings.catch_warnings(record=True) as record:
        warnings.simplefilter("always")
        system = _gen(graph, declared(graph, take_timeout_ms={("sub", "data"): 30}))
    got = _ours(record)
    assert got == [
        "execution config: 'maxwait_ms' on sub 'data' only sizes L: its edge(s) "
        "/data (pub -> sub) are local and decided when the publish returns, so no join "
        "waits for it and no cutoff counts from it; it only sizes the worker lag bound "
        "L = 2 x the largest maxwait_ms and through L the retention bound",
    ], f"no only-sizes-L warning for a take_timeout_ms that bounds no wait: {got}"
    # Q5: L and retention stay what T sized before. P 100 ms (the _pub
    # default), d 1 ms, T 30 ms, L = 2T = 60 ms, W = d + L = 61 ms:
    # floor(61/100) + 1 = 1 invocation of k 1, retain 1 + 1 = 2.
    e = _edge(system, "pub", "sub")
    assert (e.timeout_ns, e.max_lag_ns, e.retain) == (30 * MS, 60 * MS, 2), (
        f"edge {e.label()}: T {e.timeout_ns}, L {e.max_lag_ns}, retain {e.retain}")


def test_take_timeout_with_a_declared_lag_names_the_lag_check():
    graph = _pair()
    with warnings.catch_warnings(record=True) as record:
        warnings.simplefilter("always")
        _gen(graph, declared(graph, take_timeout_ms={("sub", "data"): 30}, max_worker_lag_ms=50))
    got = _ours(record)
    assert len(got) == 1 and "it only enters the max_worker_lag_ms check" in got[0], got


def test_undeclared_take_timeout_and_cross_edges_do_not_warn(tmp_path):
    graph = _pair()
    with warnings.catch_warnings(record=True) as record:
        warnings.simplefilter("always")
        _gen(graph, declared(graph))
        # The cross-federate edge's T bounds its window, nothing to warn.
        _pubsub(tmp_path, take_timeout_ms={("sub", "data"): 30})
    assert _ours(record) == [], _ours(record)


# Question 3: local sub_rebind edges under require_admitted ------------------


def test_local_sub_rebind_edge_is_refused_under_require_admitted():
    graph, _ = _lifecycle_pair("on_configure")
    bounds = declared(graph, require_admitted=True)
    try:
        system = _gen(graph, bounds)
    except ValueError as exc:
        msg = str(exc)
    else:
        e = _edge(system, "pub", "sub")
        pytest.fail(f"require_admitted accepted local sub_rebind edge {e.label()} (status "
                    f"{e.status}, sub_rebind={e.sub_rebind}): it keeps a cutoff and is not "
                    f"decided at publish")
    assert "require_admitted: refused topic edge(s) /data pub -> sub: local_sub_rebind" in msg, msg


def test_local_sub_rebind_edge_stays_admitted_without_require_admitted():
    graph, system = _lifecycle_pair("on_configure")
    e = _edge(system, "pub", "sub")
    assert (e.status, e.sub_rebind) == ("admitted", True), f"edge {e.label()}: {e}"
    row = _manifest(system, graph)["edges"][0]
    # It keeps today's cutoff path, so the manifest does not mark it.
    assert "cutoff" not in row["bounds"] and "local_now" not in row["bounds"], row


# Trace manifest ---------------------------------------------------------------


def test_manifest_marks_a_local_edge_cutoff_none():
    graph = _pair()
    system = _gen(graph, declared(graph))
    (row,) = _manifest(system, graph)["edges"]
    got = {k: row["bounds"].get(k) for k in ("local_now", "cutoff")}
    assert got == {"local_now": True, "cutoff": "none"}, (
        f"manifest row of local edge {row['topic']} does not say it has no cutoff: {row}")


def test_manifest_keeps_the_cross_federate_rows_unmarked(tmp_path):
    _pubsub(tmp_path)
    doc = json.loads((tmp_path / "lf-analysis/test_pkg/bringup" / FEDERATED_MANIFEST_NAME)
                     .read_text())
    roles = {r["role"]: r["bounds"] for r in doc["edges"] if r["status"] == "admitted"}
    assert set(roles) == {"egress", "ingress"}, doc["edges"]
    for role, bounds in roles.items():
        assert "cutoff" not in bounds and "local_now" not in bounds, (role, bounds)


def test_local_now_needs_no_new_header_line():
    # The binding derives local_now from the role (Table::set_local_now), so
    # the generated init code has no extra call.
    graph = _pair()
    system = _gen(graph, declared(graph))
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    assert "local_now" not in cpp, cpp


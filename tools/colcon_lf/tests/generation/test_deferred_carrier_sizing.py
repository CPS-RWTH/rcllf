# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A deferred edge's carrier and retention cover its cutoff wait.

``join: deferred`` releases an ingress record at the cutoff tag
t + d + T, not at t + d (``release_micro_locked`` in correspondence.cpp). The
generator sized the carrier from d alone, ``N_e(d)``. In the E5 deferred
brake_fusion federation the 25 ms brake edge (d = 5 ms, T = 100 ms) got
carrier 1 and the binding rejected the newest record whenever one was already
waiting: the simulator reconciled ``MATCHED 33,
push_overflow 135`` of 169 brake commands, one sample in five, and the car ran
the 300 m horizon in the issue's run.

The expected occupancies below are counted by brute force over a timer
schedule, not by the generator's N(W) arithmetic.
"""

from __future__ import annotations

import json

from .test_xfed_correspondence import _federate, _pub, _sub, _tc

MS = 1_000_000
PERIOD, D, T = 25 * MS, 5 * MS, 100 * MS
L = 2 * T  # max_worker_lag_ms unset: twice the largest admitted timeout


def _most_held(start_off: int, end_off: int, k: int = 1) -> int:
    """Most records held at once when tag t's k records live on [t + start, t + end]."""
    tags = [i * PERIOD for i in range(400)]
    events = sorted({t + start_off for t in tags} | {t + end_off for t in tags})
    return max(k * sum(1 for t in tags if t + start_off <= tau <= t + end_off)
               for tau in events if tags[100] <= tau <= tags[300])


def _ingress_bounds(tmp_path, deferred: bool) -> dict:
    _federate(tmp_path, [_pub("pub", "PubNode"), _sub("sub", "SubNode")],
              [_tc("/data", ["pub"], ["sub"])], {"pub": "pub", "sub": "sub"},
              after_delay="5 msec", timer_period_overrides={"pub": {"tick": 25.0}},
              take_timeout_ms={("sub", "data"): 100},
              deferred_join={("sub", "data")} if deferred else set())
    man = json.loads((tmp_path / "lf-analysis/test_pkg/bringup/"
                      "rcllf_trace_manifest_federated.json").read_text())
    rows = [r for r in man["edges"] if r["topic"] == "/data" and r["role"] == "ingress"]
    assert len(rows) == 1, man["edges"]
    b = rows[0]["bounds"]
    assert b["join"] == ("deferred" if deferred else "blocking"), b
    assert (b["inputs"]["d_ns"], b["inputs"]["T_ns"], b["inputs"]["L_ns"]) == (D, T, L), b
    return b


def test_deferred_carrier_holds_every_record_until_its_cutoff(tmp_path):
    # A cross record arrives at its release tag t + d (the record port's
    # after delay) and a deferred one waits in the carrier until t + d + T.
    held = _most_held(D, D + T)
    assert held == 5
    b = _ingress_bounds(tmp_path, deferred=True)
    if b["carrier"] < held:
        raise AssertionError(
            f"deferred edge carrier {b['carrier']} < {held} records waiting for their "
            f"cutoff at once: every newer record is push_overflow, the deferred join "
            f"starves")
    assert b["carrier"] == 5, b  # N(d + T) = floor(105 / 25) + 1


def test_deferred_retention_covers_the_later_drain(tmp_path):
    # A retained sample lives from its arrival (no earlier than t - eps, eps
    # < T on a cross edge) to the end of its drain: the deferred drain runs at
    # t + d + T and may lag L past that tag.
    held = _most_held(-(T - 1), D + T + L)
    assert held == 17
    b = _ingress_bounds(tmp_path, deferred=True)
    assert b["retain"] >= held, (
        f"deferred edge retain {b['retain']} < {held} samples held until the deferred "
        f"drain")
    assert b["retain"] == 18 and b["inputs"]["W_ns"] == D + T + L + T, b


def test_blocking_edge_keeps_its_bounds(tmp_path):
    # Not deferred: released at t + d, carrier N(d) = 1, window d + L + eps.
    b = _ingress_bounds(tmp_path, deferred=False)
    assert b["carrier"] == 1 and b["retain"] == 14 and b["inputs"]["W_ns"] == D + L + T, b

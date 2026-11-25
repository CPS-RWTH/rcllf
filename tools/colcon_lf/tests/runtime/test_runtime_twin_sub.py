# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Twin subscriptions get one correspondence edge each.

``handle_matching_example/twin_sub.launch.py``, built without an exec-config:
twin_publisher's 100 ms timer publishes 0, 1, 2, ... on /twin, and
twin_subscriber has two subscriptions on /twin (``on_first`` created before
``on_second``) that log ``TWIN first <n>`` and ``TWIN second <n>``. Each
subscription has its own RID, so the generator admits one edge per
subscription and the binding picks each edge's subscription handle by the
RID's creation index. Expected tags come from the 100 ms period and the
default ``after 0``: sample n (ordinal n + 1) at tag (n * 100 ms, 1).
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts
from .test_runtime_correspondence import D_NS, HOP_MICRO, _events, _reconcile

pytestmark = pytest.mark.runtime

PERIOD_NS = 100_000_000
SAMPLES = 10


def test_twin_subscriptions_each_get_an_admitted_edge(monkeypatch):
    case = rts.require_case("twin_sub")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    result = rts.launch_and_capture(
        case, 1.0, until=lambda log: f"TWIN second {SAMPLES - 1}" in log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    log = result.log
    admitted = re.findall(r"rcllf corr: admitted edge (\d+) (\S+) -> (\S+) on '([^']+)'", log)
    assert sorted(admitted) == [
        ("0", "TwinPublisherNode", "TwinSubscriberNode", "/twin"),
        ("1", "TwinPublisherNode", "TwinSubscriberNode", "/twin"),
    ], f"expected two admitted /twin edges, got {admitted}\n{log[-2000:]}"
    dispatch = _events(log, "dispatch")
    for edge in (0, 1):
        tags = [(e["ordinal"], e["elapsed"], e["microstep"], e["outcome"])
                for e in dispatch if e["edge"] == edge][:SAMPLES]
        assert tags == [(n + 1, n * PERIOD_NS + D_NS, HOP_MICRO, "MATCHED") for n in range(SAMPLES)], (
            f"edge {edge}: {tags}")
    for which in ("first", "second"):
        heard = [int(n) for n in re.findall(rf"TWIN {which} (\d+)", log)][:SAMPLES]
        assert heard == list(range(SAMPLES)), f"on_{which} heard {heard}"
    rec = _reconcile(log)
    assert rec["reconciled"] is True, rec

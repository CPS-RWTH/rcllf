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
"""Fan-in order check on the recorded G-G trace."""

from __future__ import annotations

import random

import pytest

from lf_eval.semantic.fan_in import order_key

from .golden_support import Golden, text


def _order(block) -> dict:
    return block["metrics"]["semantic_fan_in_order"]["data"]


def _members(g: Golden) -> dict[str, int]:
    """Edge id of each fan-in member, keyed by its publisher node."""
    return {r["publisher"]["node"]: int(r["id"]) for r in g.static["edges"]
            if r.get("group_lead") is not None}


def _dispatches(g: Golden, members) -> list:
    """The group's dispatches in the order the drain ran them (dispatch tags)."""
    ds = [e for e in g.events if e.name == "dispatch_begin" and int(e.fields["edge"]) in members]
    return sorted(ds, key=lambda d: (int(d.fields["tag_time"]), int(d.fields["tag_microstep"])))


def test_recorded_group_dispatches_in_rank_order():
    g = Golden("fanin_gg")
    m = _members(g)
    assert sorted(m) == ["gg_pub_a", "gg_pub_b"], g.static["edges"]
    a, b = m["gg_pub_a"], m["gg_pub_b"]
    seq = [int(d.fields["edge"]) for d in _dispatches(g, {a, b})]
    # The plan's G-G sequence a0 b0 a1 b1 ...: both publish at every tick,
    # gg_pub_a ranks first. The run stops by SIGINT, possibly after an a.
    assert len(seq) >= 40 and seq == [a, b] * (len(seq) // 2) + [a] * (len(seq) % 2), seq[:12]
    got = _order(g.analyze())
    assert sorted(got) == [str(min(a, b))], got
    one = got[str(min(a, b))]
    assert (one["members"], one["dispatches"]) == (sorted([a, b]), len(seq)), one
    assert (one["violations"], one["unkeyed"]) == (0, 0), one
    for e in (a, b):
        c = g.claim(str(e))
        assert "fan_in_order" not in c["reconciliation_codes"], text(c, "reconciliation")


def test_swapped_dispatch_is_an_order_violation():
    """Named mutation: the drain ran gg_pub_b's record before gg_pub_a's of one release."""
    g = Golden("fanin_gg").copy()
    m = _members(g)
    a, b = m["gg_pub_a"], m["gg_pub_b"]
    x, y = _dispatches(g, {a, b})[2:4]
    assert [int(x.fields["edge"]), int(y.fields["edge"])] == [a, b], "a1 then b1 in the recording"
    for f in ("tag_time", "tag_microstep"):
        x.fields[f], y.fields[f] = y.fields[f], x.fields[f]
    one = _order(g.analyze())[str(min(a, b))]
    assert one["violations"] == 1, f"b dispatched before a at one release was not reported: {one}"
    assert one["first_violation"]["dispatched"]["edge"] == a, one
    for e in (a, b):
        c = g.claim(str(e))
        assert c["reconciliation"] == "unreconciled" and "fan_in_order" in c["reconciliation_codes"], (
            f"edge {e}: {text(c, 'reconciliation')}")


def test_order_check_reads_no_timestamp():
    g = Golden("fanin_gg")
    base = _order(g.analyze())
    p = g.copy()
    ts = [e.ts for e in p.events]
    rng = random.Random(450)
    rng.shuffle(ts)
    for e, t in zip(p.events, ts):
        e.ts = t
    rng.shuffle(p.events)
    assert _order(p.analyze()) == base


@pytest.mark.parametrize("delay, join, timeout, want", [
    (0, "blocking", 0, (1000, 3, 1, 7)),   # d = 0: lf_delay_tag((t, m), 0) = (t, m + 1)
    (5, "blocking", 0, (1005, 0, 1, 7)),   # d > 0: (t + d, 0), microsteps collapse
    (0, "deferred", 30, (1030, 0, 1, 7)),  # a deferred join releases at t + d + T
])
def test_order_key_is_the_delivery_tag_then_rank_then_ordinal(delay, join, timeout, want):
    row = {"delay_ns": delay, "fan_in_rank": 1, "bounds": {"join": join, "timeout_ns": timeout}}
    assert order_key(row, 1000, 2, 7) == want


@pytest.mark.parametrize("delay, join, timeout, want", [
    (0, "blocking", 0, (1000, 2, 1, 7)),   # a same-tag row releases at (t, m) itself
    (5, "blocking", 0, (1005, 0, 1, 7)),   # d > 0 is unchanged
    (0, "deferred", 30, (1030, 0, 1, 7)),  # a deferred join is unchanged
])
def test_same_tag_row_releases_at_its_publication_tag(delay, join, timeout, want):
    row = {"delay_ns": delay, "fan_in_rank": 1, "same_tag": True,
           "bounds": {"join": join, "timeout_ns": timeout}}
    assert order_key(row, 1000, 2, 7) == want

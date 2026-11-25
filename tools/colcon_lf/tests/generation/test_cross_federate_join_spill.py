# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A blocking join's stall spills across a federation link.

A blocking join on an admitted cross-federate edge x holds its subscriber's
whole federate F until x's cutoff, publication tag + d_x + T_x. A sample F
publishes at the release tag t of x then leaves F at t + T_x or later, while
the cutoff of an admitted edge y out of F counts from the tag: t + d_y + T_y.
When y closes a cycle back to x's publisher federate and
T_x > T_y + d_y, a loss on x turns y's samples NOT_READY / publisher_late:
the brake_first case (brake edge T = 200 ms, sensor edges T = 100 ms,
d = 5 ms). The radar_k case of the same config (radar stall 100 ms, brake
edge in the other direction T = 200 ms) does not spill. Generation warns,
it does not refuse.

Every expected value is derived from that rule by hand, not read back.
"""

from __future__ import annotations

import warnings

from colcon_lf.federation.partition import AfterOverride

from .test_correspondence_emission import _loop_graph
from .test_xfed_correspondence import _federate

MS = 1_000_000
ISSUE = "federation: a loss on blocking edge"
FEDS = {"a": "fa", "b": "fb"}


def _loop(tmp_path, cmd_ms: int, state_ms: int, *, overrides=(), deferred=frozenset(),
          node_fed=None):
    """a's timer -> /cmd -> b, b's callback -> /state -> a, one node per federate.

    /cmd is the loop's forward edge (T = *cmd_ms*, like the brake edge),
    /state the backward one (T = *state_ms*, like the sensor edges).
    Returns the warnings and the generated federates.
    """
    graph = _loop_graph()
    with warnings.catch_warnings(record=True) as record:
        warnings.simplefilter("always")
        out = _federate(tmp_path, graph.nodes.values(), graph.communication.topics,
                        node_fed or FEDS, overrides=overrides,
                        take_timeout_ms={("b", "/cmd"): cmd_ms, ("a", "/state"): state_ms},
                        deferred_join=set(deferred))
    return [str(w.message) for w in record if ISSUE in str(w.message)], out


def _admitted(out, fed, topic):
    rows = [r for r in out[fed]["rows"] if r["topic"] == topic]
    assert rows and all(r["status"] == "admitted" for r in rows), rows
    return rows[0]


def test_brake_shaped_stall_longer_than_the_other_direction_warns(tmp_path):
    # brake_first shape: the edge into fb has T = 200 ms, the edge fb
    # publishes back on has T = 100 ms and d = 0: 200 > 100 + 0.
    got, out = _loop(tmp_path, cmd_ms=200, state_ms=100)
    assert _admitted(out, "fb", "/cmd")["role"] == "ingress"
    assert _admitted(out, "fa", "/state")["role"] == "ingress"
    if not got:
        raise AssertionError(
            "no warning for blocking edge /cmd (a -> b) T = 200 ms against /state (b -> a) "
            "T = 100 ms + d = 0 ms: a loss on /cmd holds fb past /state's cutoff")
    assert len(got) == 1, got
    msg = got[0]
    assert msg.startswith(
        "federation: a loss on blocking edge /cmd (a -> b) holds federate fb for its "
        "T = 200 ms, longer than the slack T + d of the admitted edge(s) fb publishes on "
        "in the other direction of the cycle through federate fa: "
        "/state (b -> a) T = 100 ms + d = 0 ms."), msg
    assert "the T of a blocking edge is at most the smallest T + d" in msg, msg
    assert "raise those maxwait_ms to at least 200 ms" in msg, msg


def test_radar_shaped_stall_shorter_than_the_other_direction_does_not_warn(tmp_path):
    # radar_k shape: the stalled edge (/state into fa) has T = 100 ms, the edge
    # fa publishes back on (/cmd) has T = 200 ms: 100 > 200 is false. Only
    # the /cmd stall of the same config warns.
    got, _ = _loop(tmp_path, cmd_ms=200, state_ms=100)
    assert not [m for m in got if "blocking edge /state" in m], got


def test_mirror_config_warns_on_the_other_edge(tmp_path):
    got, _ = _loop(tmp_path, cmd_ms=100, state_ms=200)
    assert len(got) == 1 and got[0].startswith(
        "federation: a loss on blocking edge /state (b -> a) holds federate fa for its "
        "T = 200 ms"), got
    assert "/cmd (a -> b) T = 100 ms + d = 0 ms." in got[0], got


def test_equal_timeouts_do_not_warn(tmp_path):
    got, _ = _loop(tmp_path, cmd_ms=100, state_ms=100)
    assert got == []


def test_the_link_delay_of_the_other_direction_is_slack(tmp_path):
    # d_state = 5 ms: 105 > 100 + 5 is false, 106 > 105 is true.
    five = [AfterOverride("b.state", "a.state", 5 * MS)]
    got, out = _loop(tmp_path, cmd_ms=105, state_ms=100, overrides=five)
    assert _admitted(out, "fa", "/state")["delay_ns"] == 5 * MS
    assert got == []
    got, _ = _loop(tmp_path, cmd_ms=106, state_ms=100, overrides=five)
    assert len(got) == 1 and "/state (b -> a) T = 100 ms + d = 5 ms." in got[0], got


def test_a_deferred_join_does_not_hold_its_federate(tmp_path):
    got, out = _loop(tmp_path, cmd_ms=200, state_ms=100, deferred={("b", "/cmd")})
    assert "set_corr_edge_deferred" in out["fb"]["init"]
    assert got == []


def test_one_federate_has_no_cross_cycle(tmp_path):
    # Both nodes in one federate: no edge crosses a federation link.
    got, _ = _loop(tmp_path, cmd_ms=200, state_ms=100, node_fed={"a": "fa", "b": "fa"})
    assert got == []

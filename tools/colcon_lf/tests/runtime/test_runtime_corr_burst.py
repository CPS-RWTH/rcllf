# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A declared burst bound below the real one fails the run (W3 slice S2).

Needs the fixture built with ``corr_many_k2_execution.yaml``: ``corr_publisher``
declares ``max_publishes_per_invocation: 2`` while scenario ``many`` publishes
three messages per tick on /corr/a. The generator then emits B 2, retain 5 and
carrier 2 for each /corr/a edge (P 200 ms, d 0, L 200 ms, so
N(200 ms) = 2 * 2 and N(0) = 2). Every tick emits three records in one reaction, so the
reconcile at shutdown fails with ``burst_exceeded`` and the process exits 3.
Expected values come from the fixture source and the exec-config, never from a
run. Build with
``LF_EXEC_CONFIG=$PWD/tools/colcon_lf/tests/runtime/corr_many_k2_execution.yaml
bash docker/lf_build.sh correspondence_fixture``.
"""

from __future__ import annotations

import os
import re

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

LAST_TICK = 10  # correspondence_fixture: 200 ms timer, 10 ticks
K2_TAIL = "2u, 5u, 2u, 8u});"  # burst, retain, carrier, reader depth of a corr_publisher edge at k 2


def _require_k2_build(case: rts.RuntimeCase) -> None:
    """Skip (fail under RCLLF_REQUIRE_RUNTIME_SMOKE=1) unless the k 2 build is installed."""
    init = (rts._PACKAGES_ROOT / "lf-generated" / case.gen_pkg / "src" /
            "ros2_lf_reaction_ids_init.cpp")
    text = init.read_text(encoding="utf-8") if init.is_file() else ""
    if K2_TAIL in text:
        return
    reason = (f"{case.gen_pkg} was not generated with corr_many_k2_execution.yaml "
              f"({K2_TAIL!r} not in {init})")
    if os.environ.get(rts.REQUIRE_ENV) == "1":
        pytest.fail(f"{rts.REQUIRE_ENV}=1 but {reason}")
    pytest.skip(reason)


def test_corr_burst_above_the_declared_bound_exits_3(monkeypatch):
    case = rts.require_case("corr")
    _require_k2_build(case)
    monkeypatch.setenv("CORR_SCENARIO", "many")
    result = rts.launch_and_capture(case, 1.0, until=lambda log: f"corr_pub: tick={LAST_TICK} " in log)
    log = result.log
    a_edges = re.findall(
        r"rcllf corr: admitted edge (\d+) CorrPublisher -> \S+ on '/corr/a' \(.*B = (\d+), "
        r"retain = (\d+), carrier = (\d+)[,)]", log)
    assert len(a_edges) == 2, "expected two admitted /corr/a edges:\n" + log[-2000:]
    for edge, b, retain, carrier in a_edges:
        assert (b, retain, carrier) == ("2", "5", "2"), (
            f"edge {edge}: B {b}, retain {retain}, carrier {carrier}; was the fixture built "
            "with corr_many_k2_execution.yaml?")
        want = f"rcllf corr: reconcile: burst_exceeded edge {edge}: 3 records in one reaction > B = 2"
        assert want in log, f"edge {edge}: no {want!r}:\n" + log[-3000:]
    assert re.search(r"process has died \[pid \d+, exit code 3", log), log[-2000:]

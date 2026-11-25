# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch remap to another topic leaf, run end to end.

``correspondence_fixture/corr_remap.launch.py`` remaps /corr/a to
/corr/a_remapped on corr_publisher_one and corr_sub_2. The publisher sends
t1k1 .. t10k1, one per 200 ms tick, so corr_sub_2 hears exactly that
sequence over the remapped edge, and the startup line names the edge by the
remapped topic.
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts
from .test_runtime_correspondence import _edges

pytestmark = pytest.mark.runtime

EXPECTED = [f"t{t}k1" for t in range(1, 11)]


def test_remapped_edge_carries_every_value_in_order():
    case = rts.require_case("corr_remap")
    result = rts.launch_and_capture(
        case, 1.0, until=lambda log: "payload=t10k1 stamp" in log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    log = result.log
    assert ("CorrSubscriberA", "/corr/a_remapped") in _edges(log), (
        f"edge not admitted on the remapped topic: {_edges(log)}")
    heard = re.findall(r"corr_sub: node=corr_sub_2 topic=\S+ payload=(\S+)", log)
    assert heard == EXPECTED, f"corr_sub_2 heard {heard}"

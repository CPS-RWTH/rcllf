# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""One topic published from two group sub-reactors, run end to end.

corr_two_group_publisher (two MutuallyExclusive groups) and
corr_reentrant_publisher (one Reentrant group, two callbacks) publish /corr/a
from on_a every 200 ms (A1..A6) and on_b every 310 ms (B1..B4), both skipping
their first tick. LF timers start at 0, so A<k> is at tag 200k ms and B<k> at
310k ms, never the same tag. corr_sub_2 must hear them in tag order.
"""

from __future__ import annotations

import re

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime


# Hand-derived from the tags: A1 200, B1 310, A2 400, A3 600, B2 620, A4 800,
# B3 930, A5 1000, A6 1200, B4 1240 (ms).
EXPECTED = ["A1", "B1", "A2", "A3", "B2", "A4", "B3", "A5", "A6", "B4"]


@pytest.mark.parametrize("case_key", ["corr_two_group", "corr_reentrant"])
def test_both_writers_reach_the_subscriber_in_tag_order(case_key):
    case = rts.require_case(case_key)
    result = rts.launch_and_capture(case, 1.0, until=lambda log: "payload=B4 stamp" in log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    heard = re.findall(r"corr_sub: node=corr_sub_2 topic=\S+ payload=(\S+)", result.log)
    assert heard == EXPECTED, f"{case_key}: corr_sub_2 heard {heard}, expected {EXPECTED}"

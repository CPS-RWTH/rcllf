#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Check a ``ros2 lf launch pubsub_example bringup`` log against derived values.

Expected values come from ``packages/src/pubsub_example/src/publisher_node.cpp``,
not from an earlier run. Publication k (from 0) carries temp=20+k,
humidity=40+k and pressure=1013+k, one every 500 ms. The getting-started page
(``docs/getting-started/first-lf-run.md``, step 5) says every ``Publishing`` is
followed by the matching ``I heard`` at the same logical time. So the log must
show

  - the bind lines for timer RID 0 (period 500000000 ns) and
    subscription RID 1 on ``sensor_data``,
  - at least ``--min`` publications with k = 0, 1, 2, ... and no gap,
  - ``Publishing k`` and ``I heard k`` strictly alternating, the last
    ``I heard`` allowed to be missing when the run was stopped between the two.

Every line that logs ``Publishing:`` or ``I heard:`` must hold exactly one event
and parse as three decimal numbers, so ``temp=nan``, a truncated line or a
malformed event followed by a valid one is a violation, not a skipped line.

Prints the counts and exits 1 naming the first violation.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# Any line that claims to be an event. Each one must then match _EVENT in full.
_CANDIDATE = re.compile(r"\]: (Publishing|I heard):")
_NUM = r"(\d+(?:\.\d+)?)"
_EVENT = re.compile(
    r"\[(publisher_node|subscriber_node)\]: (Publishing|I heard): "
    rf"temp={_NUM}, humidity={_NUM}, pressure={_NUM}\s*$"
)
_BOUND_TIMER = re.compile(
    r"Bound timer RID 0 \(PublisherNode::timer_callback, period 500000000 ns\)"
)
_BOUND_SUB = re.compile(r"Bound subscription RID 1 \(SubscriberNode::topic_callback, topic 'sensor_data'")


def expected(k: int) -> tuple[float, float, float]:
    return 20.0 + k, 40.0 + k, 1013.0 + k


def check(text: str, minimum: int) -> tuple[list[str], int, int]:
    problems: list[str] = []
    if not _BOUND_TIMER.search(text):
        problems.append("no 'Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)' line")
    if not _BOUND_SUB.search(text):
        problems.append("no 'Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data'' line")
    events = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        candidates = len(_CANDIDATE.findall(line))
        if not candidates:
            continue
        # One event per line: with two, _EVENT could match the valid trailing one
        # and skip a malformed one before it.
        m = _EVENT.search(line) if candidates == 1 else None
        if not m:
            problems.append(f"line {lineno} does not parse as an event: {line.strip()[:160]}")
            return problems, 0, 0
        events.append((m.group(2), tuple(float(x) for x in m.group(3, 4, 5))))
    published = heard = 0
    for i, (kind, values) in enumerate(events):
        k = i // 2
        want_kind = "Publishing" if i % 2 == 0 else "I heard"
        if kind != want_kind:
            problems.append(f"event {i}: '{kind}' where '{want_kind}' for k={k} was expected")
            break
        if values != expected(k):
            problems.append(f"event {i}: {kind} {values}, expected {expected(k)} for k={k}")
            break
        if kind == "Publishing":
            published += 1
        else:
            heard += 1
    if published < minimum:
        problems.append(f"{published} publications, expected at least {minimum}")
    if heard not in (published, published - 1):
        problems.append(f"{heard} receptions for {published} publications")
    return problems, published, heard


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", type=Path)
    parser.add_argument("--min", type=int, default=5, help="minimum publications (default: 5)")
    args = parser.parse_args(argv)
    problems, published, heard = check(args.log.read_text(errors="replace"), args.min)
    print(f"published {published} (k=0..{published - 1}), heard {heard}, "
          f"values 20+k/40+k/1013+k, alternating")
    for problem in problems:
        print(f"mismatch: {problem}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
